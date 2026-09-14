"""renardo_sanitizer.py — единый seam очистки Renardo-кода перед исполнением.

Раньше пять проходов жили приватными методами :class:`MusicManager` и
применялись вручную в ``execute_code``, а те же запреты дублировались в
промпте. Теперь весь pipeline — один вызов:

    result = sanitize_renando(code, max_amp)

    if result.security_error: ...   # фильтр безопасности
    if result.quality_errors: ...   # музыкальный валидатор (жёстко)
    if result.slot_error: ...       # d4+/p4+ переставить некуда
    code = result.code              # отсанированный код
    result.warnings                 # мягкие предупреждения для LLM

Модуль чистый: без Renardo, без ROS, без I/O. Его можно тестировать
напрямую, не поднимая ``MusicManager`` и звуковой стек.
"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Safety filter — compiled once at import time
# ---------------------------------------------------------------------------

_BLOCKED_TOKENS = re.compile(
    r"\b("
    r"import|os|sys|subprocess|shutil|socket|requests|urllib|http|ftplib|"
    r"importlib|builtins|__import__|__builtins__|__class__|__subclasses__|"
    r"open|exec|eval|compile|globals|locals|vars|delattr"
    r")\b"
)

#: Reflection builtins that stay allowed (legitimate Renardo use — e.g.
#: ``Clock.future(8, lambda: setattr(Clock, "bpm", 170))``) but only when the
#: attribute name is a plain string literal, never a computed one.
_LITERAL_ATTR_BUILTINS: frozenset = frozenset({"getattr", "setattr", "hasattr"})

# ---------------------------------------------------------------------------
# Issue #1016 — music-quality guardrail (dramaturgy validator)
# ---------------------------------------------------------------------------
# The safety filter above blocks *dangerous system tokens*. This separate
# guardrail validates *musical quality* before the code reaches Renardo so
# the LLM cannot regenerate a static 4-8 note loop:
#
#   1. Absolute frequencies (freq=440 / hz=220 / midinote=69) are rejected
#      — Renardo wants scale *degrees* (p1 >> pluck([0,4,7])), not Hz.
#   2. Every non-play player must carry an explicit ``dur=`` — otherwise
#      the pattern defaults to a staccato click-train.
#   3. (soft) A multi-part track without any developing pattern
#      (``.every`` / ``Pvar`` / ``linvar`` / ``Clock.future``) is a static
#      loop — warn so the LLM can fix it before the user hears it.
#
# Errors block execution; warnings are appended to the result message.

_ABSOLUTE_FREQ_RE = re.compile(
    r"\b(?:freq|frequency|hz|midinote|note)\s*=\s*(\d+(?:\.\d+)?)"
)
# Player creation lines: `p1 >> pluck([0,2,4], dur=0.5)` / `d1 >> play("x-o-")`
#
# 🔴 FIX (live 02.09): аргументы захватываются ДО КОНЦА СТРОКИ, а не до
# первой закрывающей скобки. С `[^)]*` любая вложенная скобка обрывала
# захват, и всё, что за ней, для валидатора не существовало. Аккорд пэда —
# PGroup, то есть круглые скобки (`p3 >> warmpad((0, 2, 4), dur=4, ...)`):
# захват обрывался на `(0, 2, 4)`, dur= в аргументы не попадал, и правило
# «у каждого не-play плеера должен быть dur» ругалось на строку, где dur
# есть. Та же слепота касалась inline `var(...)`/`Pvar(...)`.
_PLAYER_LINE_RE = re.compile(r"^\s*(\w+)\s*>>\s*(\w+)\s*\((.*)\)\s*$", re.MULTILINE)
# Developing patterns that break a static loop (issue #1016).
_DEV_PATTERN_RE = re.compile(
    r"\.every\(|Pvar\(|pvar\(|linvar\(|var\(|Clock\.future|chop=|stutter|shuffle|reverse"
)
# Hard-blocked hardware constraints (live 20.08): deepseek игнорирует
# промпт-запреты, поэтому ловим на уровне кода. chop= (не 0) → щелчки на
# 16 kHz DAC; spack= (не 0) → сырые глитчевые сэмплы pitchglitch-пака.
_CHOP_RE = re.compile(r"\bchop\s*=\s*(?!0\b)")
_SPACK_NONZERO_RE = re.compile(r"\bspack\s*=\s*[1-9]")

# Issue #1804 — на роботе физически смонтированы только d1-d3/p1-p3.
# Токен слева от ``>>`` в форме [dpsl]+цифра — это renardo-плеер; если он
# вне допустимой шестёрки, код обязан переставить слой в свободный слот
# (см. ``_remap_illegal_slots``), а не молча дать модели написать в d4/p5.
_ALLOWED_PLAYER_SLOTS: Tuple[str, ...] = ("d1", "d2", "d3", "p1", "p2", "p3")
_ALLOWED_PLAYER_SLOTS_SET: frozenset = frozenset(_ALLOWED_PLAYER_SLOTS)
_PLAYER_ASSIGN_RE = re.compile(
    r"^(?P<indent>[ \t]*)(?P<name>[dpsl]\d+)(?P<arrow>\s*>>\s*)(?P<synth>\w+)\s*\(",
    re.MULTILINE,
)

# Issue #1803 — длина рисунка play("...") задаёт его период; если она не
# делит такт, рисунок плывёт относительно соседних слоёв на каждом
# повторе (см. ``_fix_pattern_length``).
_PLAY_PATTERN_LEN_RE = re.compile(r"play\((\s*)(['\"])([^'\"]*)\2")


def _is_dunder(name: str) -> bool:
    """``True`` для ``__name__``-подобных имён."""
    return name.startswith("__") and name.endswith("__")


# ---------------------------------------------------------------------------
# Результат санации
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SanitizeResult:
    """Итог одного прохода :func:`sanitize_renando`.

    Attributes:
        code: отсанированный код (на жёсткой ошибке равен исходному —
            вызывающий всё равно вернёт ошибку, а не код).
        security_error: ошибка фильтра безопасности (нет ключа ``code``
            в ответе тула — код не должен уходить обратно модели).
        quality_errors: жёсткие ошибки музыкального валидатора.
        slot_error: d4+/p4+ некуда переставить — честная ошибка.
        warnings: мягкие предупреждения — идут в success-сообщение тула.
    """

    code: str
    security_error: Optional[str] = None
    quality_errors: Tuple[str, ...] = ()
    slot_error: Optional[str] = None
    warnings: Tuple[str, ...] = ()


# ---------------------------------------------------------------------------
# Security filter
# ---------------------------------------------------------------------------


def _filter_code(code: str) -> Tuple[bool, str]:
    """Проверить код на наличие опасных конструкций.

    Двухслойная проверка: текстовый blocklist (``_BLOCKED_TOKENS``) —
    быстрый отсев ``import`` / ``os`` / ``eval``; AST-проход
    (:func:`_filter_code_ast`) — обходы текстового фильтра (dunder-цепочки,
    ``getattr``/``setattr`` с вычисляемым именем).

    Returns:
        (is_safe, error_message) — (True, "") если код безопасен.
    """
    match = _BLOCKED_TOKENS.search(code)
    if match:
        return False, f"Запрещённый токен в коде: '{match.group()}'"
    return _filter_code_ast(code)


def _filter_code_ast(code: str) -> Tuple[bool, str]:
    """AST-слой фильтра безопасности (см. :func:`_filter_code`).

    Returns:
        (is_safe, error_message) — (True, "") если код безопасен.
    """
    try:
        tree = ast.parse(code)
    except SyntaxError as exc:
        return False, f"Синтаксическая ошибка в коде: {exc.msg}"

    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and _is_dunder(node.attr):
            return False, f"Запрещённый доступ к dunder-атрибуту: '{node.attr}'"
        if isinstance(node, ast.Name) and _is_dunder(node.id):
            return False, f"Запрещённое dunder-имя: '{node.id}'"
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id in _LITERAL_ATTR_BUILTINS and len(node.args) >= 2:
                attr_arg = node.args[1]
                if not isinstance(attr_arg, ast.Constant) or not isinstance(
                    attr_arg.value, str
                ):
                    return False, (
                        f"'{node.func.id}' допустим только со строковым "
                        "литералом в качестве имени атрибута"
                    )
                if _is_dunder(attr_arg.value):
                    return False, (
                        f"Запрещённый доступ к dunder-атрибуту: "
                        f"'{attr_arg.value}'"
                    )
    return True, ""


# ---------------------------------------------------------------------------
# Issue #1016 — music-quality guardrail (dramaturgy validator)
# ---------------------------------------------------------------------------


def _validate_music_code(code: str) -> Tuple[List[str], List[str]]:
    """Проверить музыкальное качество кода перед отправкой в Renardo.

    1. Абсолютные частоты (``freq=440`` / ``hz=220`` / ``midinote=69``)
       → HARD error (Renardo ожидает ступени).
    2. ``chop=`` (не ноль) → HARD error (щелчки на 16 kHz DAC).
    3. ``spack=`` (не ноль) → HARD error (сырые глитчевые сэмплы).
    4. ``dur=`` у каждого не-play плеера → WARNING.
    5. Многоголосный трек без развивающих паттернов → WARNING.

    Returns:
        (errors, warnings) — errors блокируют выполнение, warnings идут в
        result message (LLM их увидит).
    """
    errors: List[str] = []
    warnings: List[str] = []

    freq_match = _ABSOLUTE_FREQ_RE.search(code)
    if freq_match:
        errors.append(
            "Абсолютные частоты запрещены (Renardo ожидает ступени): "
            f"'{freq_match.group(0)}'. Используй степени, например "
            "p1 >> pluck([0,4,7]) или p1 >> pluck([0,4,7], oct=3)."
        )

    if _CHOP_RE.search(code):
        errors.append(
            "chop= запрещён — на 16 kHz DAC даёт щелчки, а не sidechain. "
            "Для дакинга используй amplify=var([1,0.3],[0.5,0.5])."
        )

    if _SPACK_NONZERO_RE.search(code):
        errors.append(
            "spack=1 (пак 1_pitchglitch_samples) запрещён — сырые "
            "глитчевые сэмплы звучат как «звук из базы». Используй "
            "дефолтный пак (без spack=) или sample=P[0,1,2,3]."
        )

    players = list(_PLAYER_LINE_RE.finditer(code))
    for m in players:
        player_name, synth, args = m.group(1), m.group(2), m.group(3)
        if synth == "play":
            continue  # play() имеет dur из строки паттерна
        if "dur" not in args:
            warnings.append(
                f"У '{player_name} >> {synth}(...)' нет dur= — паттерн "
                "будет звучать как staccato-щелчки. Добавь dur (например "
                "dur=0.5 или dur=[0.5,0.25])."
            )

    if len(players) >= 2 and not _DEV_PATTERN_RE.search(code):
        warnings.append(
            "В коде нет развивающих паттернов (.every/Pvar/linvar/"
            "Clock.future) — трек будет звучать как статичный луп из "
            "4-8 нот. Добавь хотя бы один: .every(4, 'stutter'), "
            "lpf=linvar([500,4000], 16) или Pvar-гармонию."
        )

    return errors, warnings


# ---------------------------------------------------------------------------
# Issue #1804 — d4+/p4+ не звучат на роботе, кода-стражи не было
# ---------------------------------------------------------------------------


def _remap_illegal_slots(code: str) -> Tuple[str, Optional[str]]:
    """Переставить d4+/p4+/s*/l* в свободный d1-d3/p1-p3 (issue #1804).

    play(...) — обычно барабаны/перкуссия → предпочитаем d-слот; любой
    другой синт → предпочитаем p-слот. Один и тот же недопустимый токен
    всегда переезжает в один и тот же новый слот.

    Returns:
        ``(код, None)`` если всё поместилось в 6 слотов, либо
        ``(исходный_код, сообщение_об_ошибке)`` если слотов не хватило.
    """
    occupied: set = {
        m.group("name")
        for m in _PLAYER_ASSIGN_RE.finditer(code)
        if m.group("name") in _ALLOWED_PLAYER_SLOTS_SET
    }
    remapped: Dict[str, str] = {}
    errors: List[str] = []

    def _remap(m: re.Match) -> str:
        name = m.group("name")
        synth = m.group("synth")
        if name in _ALLOWED_PLAYER_SLOTS_SET:
            return m.group(0)
        if name in remapped:
            new_name = remapped[name]
        else:
            preferred = (
                _ALLOWED_PLAYER_SLOTS
                if synth == "play"
                else _ALLOWED_PLAYER_SLOTS[3:] + _ALLOWED_PLAYER_SLOTS[:3]
            )
            free = next((slot for slot in preferred if slot not in occupied), None)
            if free is None:
                errors.append(
                    f"'{name} >> {synth}(...)' вне d1-d3/p1-p3, а все "
                    "6 слотов уже заняты — слой некуда переставить. "
                    "Убери один из существующих слоёв или объедини "
                    "паттерны."
                )
                return m.group(0)
            occupied.add(free)
            remapped[name] = free
            new_name = free
        return f"{m.group('indent')}{new_name}{m.group('arrow')}{synth}("

    fixed_code = _PLAYER_ASSIGN_RE.sub(_remap, code)
    if errors:
        return code, "⛔ Недопустимые слоты плееров: " + " ".join(errors)
    return fixed_code, None


# ---------------------------------------------------------------------------
# Issue #1803 — рисунок play(...), который не делит такт, плывёт
# ---------------------------------------------------------------------------


def _fix_pattern_length(code: str) -> str:
    """Достроить рисунок play("...") до степени двойки (issue #1803).

    Модель пишет рисунки, чья длина не делит такт — они расходятся по фазе
    с соседними слоями. Длина приводится к ближайшей СВЕРХУ степени двойки;
    хвостовые паузы (``.``) снимаются до первой степени двойки. Добивка —
    только ``.`` (настоящая пауза), никогда ``-`` (звучащий сэмпл).
    """

    def _pow2_at_least(value: int) -> int:
        target = 1
        while target < value:
            target *= 2
        return target

    def _pad(m: re.Match) -> str:
        ws, quote, pattern = m.group(1), m.group(2), m.group(3)
        if len(pattern) <= 1:
            return m.group(0)

        trimmed = pattern
        while (
            len(trimmed) > 1
            and _pow2_at_least(len(trimmed)) != len(trimmed)
            and trimmed[-1] == "."
        ):
            trimmed = trimmed[:-1]

        target = _pow2_at_least(len(trimmed))
        if target == len(trimmed):
            if trimmed == pattern:
                return m.group(0)
            return f"play({ws}{quote}{trimmed}{quote}"

        padded = trimmed + "." * (target - len(trimmed))
        return f"play({ws}{quote}{padded}{quote}"

    return _PLAY_PATTERN_LEN_RE.sub(_pad, code)


# ---------------------------------------------------------------------------
# Issue #1000 — anti-click caps (amp / amplify / oct)
# ---------------------------------------------------------------------------


def _cap_amp(code: str, max_amp: float) -> str:
    """Ограничить громкость/октаву в коде до безопасных пределов.

    - ``amp=0.9`` / ``amp=P[...]`` / ``amplify=var([...])`` / ``amplify=0.8``
      → капаются до ``max_amp``.
    - ``oct=9`` → ``oct=5`` (санитарный потолок; выше — алиасинг на 16 kHz).

    Потолок 5 покрывает регистры аранжировщика (бас 3, пэд 4, мелодия 5) —
    режется только то, что модель пишет от руки выше них.
    """
    max_oct = 5

    def _cap_p(m: re.Match) -> str:
        def _cap_num(n: re.Match) -> str:
            return f"{min(float(n.group()), max_amp):.3g}"
        return "amp=P[" + re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, m.group(1)) + "]"

    code = re.sub(r"amp\s*=\s*P\[([^\]]+)\]", _cap_p, code)

    def _cap_n(m: re.Match) -> str:
        return f"amp={min(float(m.group(1)), max_amp):.3g}"

    code = re.sub(r"amp\s*=\s*(\d+(?:\.\d*)?)", _cap_n, code)

    def _cap_amplify_var(m: re.Match) -> str:
        inner = m.group(1)

        def _cap_num(n: re.Match) -> str:
            return f"{min(float(n.group()), max_amp):.3g}"

        inner = re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, inner)
        return f"amplify=var({inner})"

    code = re.sub(r"amplify\s*=\s*var\(([^)]+)\)", _cap_amplify_var, code)

    def _cap_amplify_n(m: re.Match) -> str:
        return f"amplify={min(float(m.group(1)), max_amp):.3g}"

    code = re.sub(r"amplify\s*=\s*(\d+(?:\.\d*)?)", _cap_amplify_n, code)

    def _cap_oct(m: re.Match) -> str:
        return f"oct={min(int(m.group(1)), max_oct)}"

    code = re.sub(r"oct\s*=\s*(\d+)", _cap_oct, code)
    return code


# ---------------------------------------------------------------------------
# Единый seam
# ---------------------------------------------------------------------------


def sanitize_renando(code: str, max_amp: float) -> SanitizeResult:
    """Прогнать код через весь pipeline очистки за один вызов.

    Порядок байт-в-байт повторяет прежнюю ручную цепочку в
    ``MusicManager.execute_code``: безопасность → музыкальный валидатор →
    перестановка слотов → pianovel/piano→rhpiano → длина рисунка → кап amp.

    Args:
        code: строка Renardo-кода.
        max_amp: санитарный потолок амплитуды ОДНОГО слоя (0.0-1.0).

    Returns:
        :class:`SanitizeResult` с отсанированным кодом и ошибками/варнингами.
    """
    is_safe, security_error = _filter_code(code)
    if not is_safe:
        return SanitizeResult(code=code, security_error=security_error)

    quality_errors, warnings = _validate_music_code(code)
    if quality_errors:
        return SanitizeResult(
            code=code,
            quality_errors=tuple(quality_errors),
            warnings=tuple(warnings),
        )

    code, slot_error = _remap_illegal_slots(code)
    if slot_error:
        return SanitizeResult(code=code, slot_error=slot_error, warnings=tuple(warnings))

    # Live 11:41 «цоканье»: pianovel/piano → rhpiano (MdaPiano щёлкает).
    if "pianovel" in code:
        code = code.replace("pianovel", "rhpiano")
    code = re.sub(r"(?<![a-zA-Z])piano(?![a-zA-Z])", "rhpiano", code)

    code = _fix_pattern_length(code)
    code = _cap_amp(code, max_amp)

    return SanitizeResult(code=code, warnings=tuple(warnings))


__all__ = [
    "SanitizeResult",
    "sanitize_renando",
]
