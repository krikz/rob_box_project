"""
test_audio_node_bytecode.py — Guard-тест на компиляцию audio_node.py.

История: issue #1125 / t_1bbc233a. В PR #1121 (pre-roll 1.0→1.5s +
телеметрия) в строке `self.get_logger().info(f'{pre-roll_msg})')`
была опечатка — `pre-roll_msg` с дефисом. Python-парсер НЕ считает
это атрибутом или переменной, а разбирает как **вычитание** двух
имён `pre` и `roll_msg`. Bytecode выглядит так:

    LOAD_GLOBAL pre
    LOAD_GLOBAL roll_msg
    BINARY_SUBTRACT
    FORMAT_VALUE 0

В результате на каждом timer-callback (каждые 50мс) VAD/DoA callback
кидал `NameError: name 'pre' is not defined` (раньше — и молча
ловился broad-catch без трейса). Контейнер voice-assistant на
роботе 10.1.1.21 спамил WARN ~20 раз/сек, что:

  1. Маскировало реальные ошибки (нет трейса, только `f'{e}'`)
  2. Съедало CPU на f-string + docker-logs sink
  3. Коррелировало с «робот не слышит wake word» (issue #1117)

Фикс в develop — PR #1124 (`pre-roll_msg` → `pre_roll_msg`).
Этот тест — **страховка от регрессии**: компилируем исходник в
Python 3.10 bytecode, walk'аем `co_consts` AudioNode, находим
`check_vad_and_doa`, и проверяем, что в `co_names` рядом с
`pre` НЕ лежит `roll_msg` (это бы означало, что компилятор
снова увидел `pre - roll_msg`).

Аналогично проверяем `audio_callback` — в нём используется
только `prefetch`, и bare `pre` появиться не должен.

Не трогаем ничего, кроме syntactic AST/bytecode-проверки.
Не импортируем rob_box_voice (нет ROS mocks для audio_node)
— достаточно ``compile()`` + ``marshal``.
"""

from __future__ import annotations

import marshal
import re
import types
from pathlib import Path

import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

REPO_ROOT = Path(__file__).resolve().parents[4]
AUDIO_NODE_PY = REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "audio_node.py"


def _load_methods_bytecode() -> dict[str, types.CodeType]:
    """Compile audio_node.py to a code object and return its top-level
    + nested method code objects, keyed by method name."""
    source = AUDIO_NODE_PY.read_text(encoding="utf-8")
    code = compile(source, str(AUDIO_NODE_PY), "exec")

    found: dict[str, types.CodeType] = {}

    def walk(c: types.CodeType) -> None:
        # co_consts contains nested code objects (functions, lambdas, comprehensions)
        for const in c.co_consts:
            if isinstance(const, types.CodeType):
                # Skip comprehension helpers like <listcomp>: not what we want
                if const.co_name == "<listcomp>" or const.co_name == "<module>":
                    walk(const)
                    continue
                # AudioNode.<method> — keep first occurrence
                found.setdefault(const.co_name, const)
                walk(const)

    walk(code)
    return found


# Python 3.10+ opcodes. Мы НЕ хардкодим числовые значения — берём из
# dis.opmap, потому что между 3.10 (BINARY_SUBTRACT) и 3.11+
# (BINARY_OP с arg=10 для '-') байткод поменялся.
import dis as _dis

_OP_LOAD_GLOBAL = _dis.opmap["LOAD_GLOBAL"]
# 3.10: BINARY_SUBTRACT; 3.11+: BINARY_OP 10 (-)
_OP_BINARY_SUBTRACT = _dis.opmap.get("BINARY_SUBTRACT")
_OP_BINARY_OP = _dis.opmap.get("BINARY_OP")


def _is_binary_subtract(instr) -> bool:
    """True если инструкция выполняет вычитание (3.10 BINARY_SUBTRACT
    или 3.11+ BINARY_OP с arg=10)."""
    if _OP_BINARY_SUBTRACT is not None and instr.opcode == _OP_BINARY_SUBTRACT:
        return True
    if _OP_BINARY_OP is not None and instr.opcode == _OP_BINARY_OP:
        return instr.arg == 10  # BINARY_OP 10 = '-'
    return False


def _has_pre_subtract_roll_msg_pattern(c: types.CodeType) -> bool:
    """Walk bytecode instructions looking for the regression signature:

        LOAD_GLOBAL pre
        LOAD_GLOBAL roll_msg
        BINARY_SUBTRACT | BINARY_OP 10 (-)

    Returns True if found, False otherwise. Looks at *all* nested
    code objects (inner functions, comprehensions) to be safe.
    """
    def walk_instrs(cc: types.CodeType) -> bool:
        instrs = list(_dis.get_instructions(cc))
        for i in range(len(instrs) - 2):
            a, b, c2 = instrs[i], instrs[i + 1], instrs[i + 2]
            if (
                a.opcode == _OP_LOAD_GLOBAL
                and a.argval == "pre"
                and b.opcode == _OP_LOAD_GLOBAL
                and b.argval == "roll_msg"
                and _is_binary_subtract(c2)
            ):
                return True
        # Recurse into nested code objects
        for const in cc.co_consts:
            if isinstance(const, types.CodeType):
                if walk_instrs(const):
                    return True
        return False

    return walk_instrs(c)


# ─────────────────────────────────────────────────────────────────────────────
# Sanity: file is readable and matches the expected layout
# ─────────────────────────────────────────────────────────────────────────────


def test_audio_node_source_exists() -> None:
    assert AUDIO_NODE_PY.is_file(), f"audio_node.py not found at {AUDIO_NODE_PY}"


def test_audio_node_has_check_vad_and_doa_def() -> None:
    """Source-side precondition."""
    text = AUDIO_NODE_PY.read_text(encoding="utf-8")
    assert re.search(r"^\s*def\s+check_vad_and_doa\s*\(", text, re.MULTILINE), (
        "expected `def check_vad_and_doa` in audio_node.py"
    )


def test_audio_node_has_audio_callback_def() -> None:
    text = AUDIO_NODE_PY.read_text(encoding="utf-8")
    assert re.search(r"^\s*def\s+audio_callback\s*\(", text, re.MULTILINE), (
        "expected `def audio_callback` in audio_node.py"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Bytecode guard: detect `pre - roll_msg` regression (issue #1125)
# ─────────────────────────────────────────────────────────────────────────────


def test_check_vad_and_doa_no_bare_pre_subtract_roll_msg() -> None:
    """Регрессия #1125: если в коде есть `pre-roll_msg` (с дефисом без
    пробелов), Python-парсер разбирает как `pre - roll_msg` и
    byte-code содержит LOAD_GLOBAL pre, LOAD_GLOBAL roll_msg,
    BINARY_SUBTRACT. Этот тест компилирует исходник и проверяет,
    что в `co_names` функции check_vad_and_doa имена `pre` и
    `roll_msg` НЕ присутствуют раздельно (то есть компилятор их
    НЕ интерпретировал как два голых имени).

    Примечание: имя `pre_roll_msg` (с подчёркиванием) — это нормально,
    компилятор хранит его как одно имя в ко_names.
    """
    methods = _load_methods_bytecode()
    assert "check_vad_and_doa" in methods, (
        "AudioNode.check_vad_and_doa not found in compiled bytecode"
    )

    fn = methods["check_vad_and_doa"]
    names = fn.co_names

    # Detect: и `pre`, и `roll_msg` рядом (раздельно) в одном code object.
    # Это явный признак `pre - roll_msg` (вычитание).
    has_pre = "pre" in names
    has_roll_msg = "roll_msg" in names

    if has_pre and has_roll_msg:
        # Дополнительно: возможно `pre` используется в КАЧЕСТВЕ атрибута
        # (например, self.pre). Проверяем, что `pre` не встречается
        # как LOAD_GLOBAL free-var (co_freevars). В исходнике bare `pre`
        # НЕ должен встречаться.
        # Если оба импортируются как globals — это скорее всего bad.
        pytest.fail(
            "AudioNode.check_vad_and_doa bytecode содержит ОБА `pre` и "
            "`roll_msg` как голые имена — это признак опечатки "
            "`pre-roll_msg` в f-string (Python парсит как `pre - roll_msg` "
            "→ BINARY_SUBTRACT → NameError на каждом timer-callback). "
            "Исправление: вернуть подчёркивание `pre_roll_msg` "
            "(issue #1125 / 104cd83d)."
        )


def test_audio_callback_no_bare_pre_subtract_roll_msg() -> None:
    """Аналогичная защита для audio_callback (PyAudio-stream callback,
    выполняется 100 раз/сек на 16kHz/1024-frame chunks)."""
    methods = _load_methods_bytecode()
    assert "audio_callback" in methods, (
        "AudioNode.audio_callback not found in compiled bytecode"
    )

    fn = methods["audio_callback"]
    names = fn.co_names

    if "pre" in names and "roll_msg" in names:
        pytest.fail(
            "AudioNode.audio_callback bytecode содержит ОБА `pre` и "
            "`roll_msg` — то же, что в issue #1125. Проверьте исходник."
        )


def test_check_vad_and_doa_vad_doa_warn_does_not_contain_pre_roll_msg() -> None:
    """Source-side double-check: строки broad-catch в check_vad_and_doa
    не должны содержать `pre-roll_msg` (с дефисом). Это синтаксически
    недопустимое имя переменной — Python молча парсит как вычитание."""
    text = AUDIO_NODE_PY.read_text(encoding="utf-8")
    # Find the function body
    m = re.search(
        r"def\s+check_vad_and_doa\s*\([^)]*\)\s*:(.*?)(?=^\s*def\s|\Z)",
        text,
        re.MULTILINE | re.DOTALL,
    )
    assert m, "could not find check_vad_and_doa body"
    body = m.group(1)
    # Bare `pre-roll_msg` (deficit) anywhere in the function body
    bad = re.search(r"\bpre-roll_msg\b", body)
    assert bad is None, (
        "Found bare `pre-roll_msg` (with hyphen) in check_vad_and_doa — "
        "rename to `pre_roll_msg` (issue #1125). "
        f"Match: {bad.group(0) if bad else 'n/a'}"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Verify the fix is present: source should reference pre_roll_msg (with _)
# ─────────────────────────────────────────────────────────────────────────────


def test_check_vad_and_doa_uses_pre_roll_msg_with_underscore() -> None:
    """Положительная проверка: после #1125 в коде ДОЛЖЕН быть
    `pre_roll_msg` (с подчёркиванием)."""
    text = AUDIO_NODE_PY.read_text(encoding="utf-8")
    m = re.search(
        r"def\s+check_vad_and_doa\s*\([^)]*\)\s*:(.*?)(?=^\s*def\s|\Z)",
        text,
        re.MULTILINE | re.DOTALL,
    )
    assert m
    body = m.group(1)
    assert "pre_roll_msg" in body, (
        "Expected `pre_roll_msg` (with underscore) in check_vad_and_doa "
        "after fix #1125. Did the variable get renamed back?"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Stronger guard: actual bytecode contains `pre - roll_msg` instructions
# ─────────────────────────────────────────────────────────────────────────────


def test_audio_node_no_pre_minus_roll_msg_bytecode() -> None:
    """Самый сильный guard: walk'аем bytecode audio_node.py и убеждаемся,
    что *нигде* в модуле не встречается sequence

        LOAD_GLOBAL pre
        LOAD_GLOBAL roll_msg
        BINARY_SUBTRACT

    Это точная сигнатура регрессии #1125. Если в коде снова появится
    `pre-roll_msg` (дефис), Python разберёт как вычитание и эти три
    инструкции появятся подряд — тест упадёт с понятным сообщением.

    Защита надёжнее, чем просто regex по source: regex не поймает
    случай, когда `pre` и `roll_msg` названы как отдельные переменные
    по соседству (например, `pre = ...; roll_msg = ...` тоже даст
    false positive, но инструкции BINARY_SUBTRACT не будет).
    """
    code = compile(
        AUDIO_NODE_PY.read_text(encoding="utf-8"),
        str(AUDIO_NODE_PY),
        "exec",
    )
    if _has_pre_subtract_roll_msg_pattern(code):
        pytest.fail(
            "audio_node.py bytecode contains the regression signature: "
            "LOAD_GLOBAL pre → LOAD_GLOBAL roll_msg → BINARY_SUBTRACT. "
            "This means a `pre-roll_msg` (with hyphen, no underscore) "
            "substring is being parsed as subtraction. "
            "Rename to `pre_roll_msg` (issue #1125 / 104cd83d)."
        )
