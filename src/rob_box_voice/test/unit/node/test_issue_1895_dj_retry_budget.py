"""
test_issue_1895_dj_retry_budget.py — статический контракт синтетических ретраев.

Issue #1895 (follow-up к #1881): каждый guard-диспатч — синтетический ретрай
(``is_synthetic=True``) и списывает общий budget ``_synthetic_retries_left``
ДО диспатча (inline ``_consume_synthetic_retry`` или чистым ``core/``-хелпером).

ADR-0149 PR-13a: DJ_RETRY (``_apply_music_guard`` / ``_dispatch_dj_turn``) и
Bug C′ (``_check_embedded_renardo_code_and_retry``) удалены вместе со старым
путём музыки — их проверки тоже.

Без ROS2, чистый AST. Падает на любом расхождении с продом.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest


_VOICE_PKG = Path(__file__).resolve().parents[3] / "rob_box_voice"
DIALOGUE_NODE = _VOICE_PKG / "dialogue_node.py"


def _class_methods(src: str) -> dict:
    tree = ast.parse(src)
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DialogueNode"
    )
    return {
        fn.name: fn for fn in cls.body
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def _guard_methods(methods: dict) -> set:
    """Все методы, в которых guard может отправить синхронный ретрай.

    Идём по паттерну именования из e209e14c и старого кода:
    ``_check_*_and_retry`` (babble / action) + ``_apply_tool_skipped_guard``.
    """
    return {
        name for name in methods
        if (name.startswith("_check_") and name.endswith("_and_retry"))
        or name == "_apply_tool_skipped_guard"
    }


def _calls(method_node, attr: str) -> bool:
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == attr
        for node in ast.walk(method_node)
    )


#: Чистые ``core/turn.py``-хелперы, которым guard имеет право делегировать
#: списание общего budget вместо inline ``_consume_synthetic_retry``
#: (issue #2266 / ADR-0021 R2). Контракт хелпера: вернуть ``None``, когда
#: ``budget_left <= 0``, и отдать ``new_state`` с уже декрементнутым
#: счётчиком — то же поведение, что ``_consume_synthetic_retry`` → ``False``.
#: Расширять этот набор можно ТОЛЬКО вместе с юнит-тестами на сам хелпер
#: (см. ``test/unit/core/test_turn.py::TestBeginBabbleRetry``).
_CORE_BUDGET_DELEGATES = {"begin_babble_retry"}


def _calls_bare(method_node, func_name: str) -> bool:
    """``func(...)`` — вызов свободной функции (не метода ``self.``)."""
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == func_name
        for node in ast.walk(method_node)
    )


def _assigns_attr(method_node, attr: str) -> bool:
    """``self.<attr> = ...`` где-либо внутри метода."""
    for node in ast.walk(method_node):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (isinstance(target, ast.Attribute)
                    and target.attr == attr
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"):
                return True
    return False


def _all_dispatch_calls(method_node) -> list:
    """Все ``self._dispatch_turn(...)`` / ``self._dispatch_dj_turn(...)``."""
    out = []
    for node in ast.walk(method_node):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "self"
                and node.func.attr in {"_dispatch_turn", "_dispatch_dj_turn"}):
            out.append(node)
    return out


def _kw(call: ast.Call, name: str):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


# ---------------------------------------------------------------------------
# Тесты
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "guard_name",
    sorted({"_check_babble_and_retry",
            "_check_unbacked_action_claim_and_retry",
            "_check_universal_action_claim_and_retry",
            "_check_phantom_action_and_retry",
            "_apply_tool_skipped_guard"}),
)
def test_every_guard_dispatch_is_synthetic(guard_name: str) -> None:
    """Все guard-диспатчи синтетические (``is_synthetic=True`` для
    ``_dispatch_turn`` / ``from_tick=False`` для ``_dispatch_dj_turn``).

    Без этого синтетический промпт попадает в историю как реплика юзера
    (см. закрытый пункт acceptance #1881, вторая половина).
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    assert guard_name in methods, f"{guard_name} не существует — тест устарел"
    method = methods[guard_name]

    bad_calls = []
    for call in _all_dispatch_calls(method):
        attr = call.func.attr
        if attr == "_dispatch_turn":
            kw = _kw(call, "is_synthetic")
            if not (isinstance(kw, ast.Constant) and kw.value is True):
                bad_calls.append(
                    f"_dispatch_turn(...) без is_synthetic=True в строке {call.lineno}"
                )
        elif attr == "_dispatch_dj_turn":
            kw = _kw(call, "from_tick")
            # ``from_tick=False`` уже гарантирует синтетический ретрай
            # (см. test_dj_retry_branch_dispatches_with_from_tick_false);
            # для guard-диспатча этого достаточно. Если кто-то по ошибке
            # сменит на True — это будет значить «свежий DJ-tick», а не
            # ретрай, и следующий тест поймает.
            if not (isinstance(kw, ast.Constant) and kw.value is False):
                bad_calls.append(
                    f"_dispatch_dj_turn(...) без from_tick=False в строке {call.lineno}"
                )

    assert not bad_calls, (
        f"{guard_name}: не все guard-диспатчи синтетические:\n  "
        + "\n  ".join(bad_calls)
    )


@pytest.mark.parametrize(
    "guard_name",
    sorted({"_check_babble_and_retry",
            "_check_unbacked_action_claim_and_retry",
            "_check_universal_action_claim_and_retry",
            "_check_phantom_action_and_retry",
            "_apply_tool_skipped_guard"}),
)
def test_every_guard_consumes_synthetic_budget(guard_name: str) -> None:
    """Каждый guard, который диспатчит синхронный ретрай, обязан до этого
    списать общий budget — либо inline через ``_consume_synthetic_retry``,
    либо делегировав его чистому ``core/``-хелперу.

    Без этого guard теряет бюджет: ping-pong между разными guard'ами
    обходит общий счётчик (см. issue #1881, e209e14c).

    Issue #2266 / ADR-0021 R2 — после выноса бабл-гварда в ``core/turn.py``
    списание бюджета для него делает чистая функция
    ``begin_babble_retry`` (она возвращает ``None`` при ``budget_left <= 0``
    и отдаёт ``new_state`` с уже декрементнутым счётчиком). Инвариант тот
    же — «бюджет списан ДО диспатча», меняется только исполнитель. Поэтому
    guard считается корректным, если он ЛИБО зовёт
    ``_consume_synthetic_retry`` напрямую, ЛИБО зовёт ``core/``-хелпер из
    ``_CORE_BUDGET_DELEGATES`` и зеркалит результат в
    ``self._synthetic_retries_left``.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    assert guard_name in methods, f"{guard_name} не существует — тест устарел"
    method = methods[guard_name]

    if _calls(method, "_consume_synthetic_retry"):
        return

    delegate = next(
        (name for name in _CORE_BUDGET_DELEGATES if _calls_bare(method, name)),
        None,
    )
    assert delegate is not None, (
        f"{guard_name} не зовёт ни _consume_synthetic_retry, ни один из "
        f"core/-хелперов {sorted(_CORE_BUDGET_DELEGATES)} — "
        "общий budget _synthetic_retries_left остаётся нетронутым, "
        "guard может пинг-понгом уйти в бесконечный цикл"
    )
    assert _assigns_attr(method, "_synthetic_retries_left"), (
        f"{guard_name} делегирует бюджет в core/ через {delegate}(), но не "
        "зеркалит результат в self._synthetic_retries_left — "
        "legacy-счётчик разъедется с core/-side TurnState и остальные "
        "guard'ы увидят несписанный бюджет"
    )
