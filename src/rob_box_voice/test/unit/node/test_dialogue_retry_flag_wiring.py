"""
test_dialogue_retry_flag_wiring.py — статический детектор NameError.

🔴 Два живых инцидента подряд, 30.08.2026, оба одного класса: имя читается
там, где оно не связано, `NameError` вылетает внутри asyncio-таски или в
конструкторе, и снаружи это выглядит НЕ как ошибка, а как молчащий робот.

1. `0e7bb478` завёл флаг ``is_code_retry`` в сигнатуре ``_dispatch_turn``,
   а читал его в теле ``_run_turn``, куда флаг не добавили и не пробросили::

       # _run_turn, ~26-я строка от начала функции
       if not is_code_retry:        # NameError: name 'is_code_retry'

   Падало ДО вызова LLM, на КАЖДОМ ходе. Снаружи: STT принимал фразу
   («✅ ПРИНЯТО»), играл звук подтверждения — и робот замолкал навсегда.
   Коммит откатили целиком (`db0fba22`), вместе с полезной частью фикса.

2. Переприменяя тот же фикс, блок вызова guard'а вставили в ``__init__``
   вместо ``_handle_result``::

       File ".../dialogue_node.py", line 645, in __init__
         if spoken and self._check_embedded_renardo_code_and_retry(
       NameError: name 'spoken' is not defined

   Нода не поднималась вообще.

Первая версия этого теста проверяла только флаги ``is_*_retry`` в двух
методах и второй инцидент пропустила. Поэтому теперь проверка общая: разбор
области видимости через :mod:`symtable` — то же, чем пользуется сам
интерпретатор. Имя, которое функция читает и которое разрешается в
глобальную область, обязано существовать на уровне модуля (присваивание,
импорт, def/class) или быть builtin.

Аннотации исключены сознательно: аннотация локальной переменной или
атрибута внутри функции по PEP 526 не вычисляется в рантайме, поэтому
``self._x: Optional[T] = v`` без импорта ``Optional`` — недочёт линтера,
а не NameError. Тест ловит только то, что реально падает.

Не требует ROS2 и не импортирует ноду — чистый статический анализ.
"""

import ast
import builtins
import symtable
from pathlib import Path

import pytest


_VOICE_PKG = Path(__file__).resolve().parents[3] / "rob_box_voice"
DIALOGUE_NODE = _VOICE_PKG / "dialogue_node.py"

#: Модульные dunder'ы, которые интерпретатор кладёт в globals сам.
MODULE_DUNDERS = frozenset({
    "__file__", "__name__", "__doc__", "__package__",
    "__spec__", "__loader__", "__builtins__", "__debug__",
})


# ---------------------------------------------------------------------------
# Статический анализ
# ---------------------------------------------------------------------------


def _annotation_only_names(src: str) -> set:
    """Имена, встречающиеся ТОЛЬКО в позиции аннотации.

    По PEP 526 аннотация внутри тела функции не вычисляется, поэтому такие
    имена не могут дать NameError и в отчёт попадать не должны.
    """
    tree = ast.parse(src)
    in_annotation, elsewhere = set(), set()

    def collect(node, sink):
        for sub in ast.walk(node):
            if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Load):
                sink.add(sub.id)

    annotation_nodes = []
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and node.annotation is not None:
            annotation_nodes.append(node.annotation)
        elif isinstance(node, ast.arg) and node.annotation is not None:
            annotation_nodes.append(node.annotation)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.returns:
            annotation_nodes.append(node.returns)
    for ann in annotation_nodes:
        collect(ann, in_annotation)

    annotation_ids = {id(n) for ann in annotation_nodes for n in ast.walk(ann)}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
                and id(node) not in annotation_ids):
            elsewhere.add(node.id)

    return in_annotation - elsewhere


def unbound_global_reads(path: Path) -> list:
    """Имена, которые функция читает, но которые нигде не связаны.

    Returns:
        Список ``(scope_path, name)``. Пустой список = NameError невозможен.
    """
    # utf-8-sig: часть файлов пакета лежит с BOM; сам интерпретатор его
    # съедает, а ``symtable`` на строке с ``﻿`` падает синтаксисом.
    src = path.read_text(encoding="utf-8-sig")
    top = symtable.symtable(src, str(path), "exec")
    known = (
        {s.get_name() for s in top.get_symbols()
         if s.is_assigned() or s.is_imported() or s.is_namespace()}
        | set(dir(builtins))
        | MODULE_DUNDERS
        | _annotation_only_names(src)
    )

    found = []

    def walk(table, trail):
        if table.get_type() == "function":
            for sym in table.get_symbols():
                if (sym.is_global() and sym.is_referenced()
                        and sym.get_name() not in known):
                    found.append((" > ".join(trail[1:]), sym.get_name()))
        for child in table.get_children():
            walk(child, trail + [child.get_name()])

    walk(top, [top.get_name()])
    return found


# ---------------------------------------------------------------------------
# Тесты
# ---------------------------------------------------------------------------


def _voice_sources() -> list:
    return sorted(
        p for p in _VOICE_PKG.rglob("*.py")
        if "__pycache__" not in p.parts
    )


def test_dialogue_node_has_no_unbound_names() -> None:
    """``dialogue_node.py`` — оба инцидента 30.08 были здесь."""
    found = unbound_global_reads(DIALOGUE_NODE)
    assert not found, (
        "имена читаются, но нигде не связаны — NameError в рантайме:\n"
        + "\n".join(f"    {scope} -> {name}" for scope, name in found)
    )


@pytest.mark.parametrize(
    "source", _voice_sources(), ids=lambda p: p.name,
)
def test_voice_package_has_no_unbound_names(source: Path) -> None:
    """Тот же контракт на весь пакет: молчащая нода не должна повториться."""
    found = unbound_global_reads(source)
    assert not found, (
        f"{source.name}: имена читаются, но нигде не связаны:\n"
        + "\n".join(f"    {scope} -> {name}" for scope, name in found)
    )


# ---------------------------------------------------------------------------
# Водопровод retry-флагов — прицельная проверка поверх общей
# ---------------------------------------------------------------------------


def _methods() -> dict:
    tree = ast.parse(DIALOGUE_NODE.read_text(encoding="utf-8-sig"))
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DialogueNode"
    )
    return {
        fn.name: fn for fn in cls.body
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def _params(fn) -> set:
    a = fn.args
    return {arg.arg for arg in a.posonlyargs + a.args + a.kwonlyargs}


def _retry_flags(names) -> set:
    """Флаги вида ``is_*_retry`` — те, что заводит каждый новый guard."""
    return {n for n in names if n.startswith("is_") and n.endswith("_retry")}


def test_every_retry_flag_is_forwarded_to_run_turn() -> None:
    """``_dispatch_turn`` обязан пробросить в ``_run_turn`` все retry-флаги.

    Флаг, объявленный в обеих сигнатурах, но не переданный в вызове, тихо
    остаётся дефолтным — одноразовый бюджет ретрая сбрасывается на самом
    ретрае, и guard уходит в бесконечный пинг-понг с LLM. NameError'а тут
    нет, поэтому общая проверка выше такое не поймает.
    """
    methods = _methods()
    dispatch, run_turn = methods["_dispatch_turn"], methods["_run_turn"]

    call = next(
        node for node in ast.walk(dispatch)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_run_turn"
    )
    forwarded = {kw.arg for kw in call.keywords}
    expected = _retry_flags(_params(run_turn))

    assert expected, "в _run_turn не осталось ни одного retry-флага — тест устарел"
    missing = sorted(expected - forwarded)
    assert not missing, (
        f"_dispatch_turn не пробрасывает {missing} в _run_turn — "
        f"флаг всегда останется дефолтным"
    )


def test_known_retry_flags_are_all_present() -> None:
    """Smoke: список флагов не должен молча сжаться.

    Bug B/C (music) живёт в ``MusicGuard`` и своего флага здесь не имеет;
    остальные guard'ы — имеют.
    """
    params = _params(_methods()["_run_turn"])
    for flag in ("is_babble_retry", "is_action_claim_retry", "is_code_retry"):
        assert flag in params, f"{flag} исчез из _run_turn"


def test_guard_call_sites_live_in_handle_result() -> None:
    """Каждый ``_check_*_and_retry`` вызывается из ``_handle_result``.

    Инцидент №2: блок вызова Bug C′ вставили в ``__init__`` — там нет ни
    ``spoken``, ни ``tools_called``, и нода не поднялась. Проверка
    прицельная: NameError общая проверка уже поймает, а вот вызов guard'а
    из, скажем, ``_on_stt`` был бы синтаксически валиден и молча неверен.
    """
    methods = _methods()
    handle_result = methods["_handle_result"]
    called_in_handle_result = {
        node.func.attr
        for node in ast.walk(handle_result)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    guards = {
        name for name in methods
        if name.startswith("_check_") and name.endswith("_and_retry")
    }
    assert guards, "guard-методы исчезли — тест устарел"
    missing = sorted(guards - called_in_handle_result)
    assert not missing, (
        f"guard-методы {missing} не вызываются из _handle_result — "
        f"проверь, куда попал блок вызова"
    )

def test_every_guard_marks_its_retry_as_dispatched() -> None:
    """Каждый ``_check_*_and_retry`` обязан звать ``_mark_retry_dispatched``.

    🔴 e2e 33251879328, шаги tc12_delete_track и tc16_delete_waypoint:
    гуард отправлял ретрай, родительский ход тут же слал ``DIALOGUE_END``,
    DSM падал в IDLE — и ``process_input`` ретрая коротил на закрытом
    диалоге, возвращая пустоту за 1 мс без единого HTTP-запроса::

        15:09:31.712  calling process_input: '[CRITICAL] Ты ответил ...'
        15:09:31.713  process_input returned: spoken='' tools=[]
        15:09:31.715  Empty assistant response (LLM вернул пустоту)

    Юзер слышал «Принял.», e2e ставил llm_error. Условие отсрочки
    ``DIALOGUE_END`` перечисляло гуарды поимённо, и оба новых в него не
    попали. Теперь флаг общий — тест следит, чтобы его ставили все.
    """
    methods = _methods()
    guards = {
        name for name in methods
        if name.startswith("_check_") and name.endswith("_and_retry")
    }
    assert guards, "guard-методы исчезли — тест устарел"

    missing = []
    for name in sorted(guards):
        called = {
            node.func.attr
            for node in ast.walk(methods[name])
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        if "_mark_retry_dispatched" not in called:
            missing.append(name)
    assert not missing, (
        f"{missing} отправляют ретрай, не пометив его — "
        f"DIALOGUE_END закроет диалог и ретрай вернётся пустым"
    )


def test_dialogue_end_defers_on_the_shared_flag() -> None:
    """Отсрочка ``DIALOGUE_END`` смотрит на общий флаг, а не на имена гуардов.

    Если условие снова начнёт перечислять ``_babble_retry_used`` и ему
    подобные, следующий гуард опять окажется забыт.
    """
    run_turn = _methods()["_run_turn"]
    reads = {
        node.attr
        for node in ast.walk(run_turn)
        if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
    }
    assert "_retry_dispatched_in_turn" in reads, (
        "_run_turn больше не читает общий флаг — отсрочка DIALOGUE_END сломана"
    )
    per_guard_flags = {"_babble_retry_used", "_code_speech_retry_used",
                       "_action_claim_retry_used"}
    leaked = sorted(per_guard_flags & reads)
    assert not leaked, (
        f"_run_turn снова читает поимённые флаги {leaked} — "
        f"отсрочка DIALOGUE_END должна идти через _retry_dispatched_in_turn"
    )
