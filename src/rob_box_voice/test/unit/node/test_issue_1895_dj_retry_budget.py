"""
test_issue_1895_dj_retry_budget.py — статический контракт для DJ_RETRY-budget.

Issue #1895 (follow-up к #1881): ``MusicGuardVerdictKind.DJ_RETRY`` — такой же
синтетический ретрай, как и остальные, и должен:

1. Списывать общий budget ``_synthetic_retries_left`` через
   ``_consume_synthetic_retry(guard_name="music_dj")`` ДО диспатча ретрая;
   на исчерпании — НЕ диспатчить, вернуть ``False``.
2. Помечать «в этом ходе ретрай уже отправлен» через
   ``_mark_retry_dispatched()`` — иначе babble-/tool-guards в следующем
   цикле guard'ов могут выстрелить ещё раз (см. гейт
   ``_retry_dispatched_in_turn`` в начале ``_apply_music_guard``).
3. Передавать в ``_dispatch_dj_turn(..., from_tick=False)`` — это
   синхронный ретрай, а не свежий DJ-transition; НЕ сбрасываем
   ``MusicGuard._dj_retry_count`` (внутренний счётчик guard'а).
4. Все guard-диспатчи внутри ``_check_*_and_retry`` / ``_apply_*_guard``
   списывают общий budget (тот же контракт, что в карточке #1881 — здесь
   расширяем проверку, чтобы DJ_RETRY не уехал снова).

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
    ``_check_*_and_retry`` (babble / code / action) + ``_apply_*_guard``
    (music / tool_skipped).
    """
    return {
        name for name in methods
        if (name.startswith("_check_") and name.endswith("_and_retry"))
        or name in {"_apply_music_guard", "_apply_tool_skipped_guard"}
    }


def _calls(method_node, attr: str) -> bool:
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == attr
        for node in ast.walk(method_node)
    )


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


def test_dj_retry_branch_consumes_budget() -> None:
    """DJ_RETRY обязан списывать общий budget ``_synthetic_retries_left``.

    Без этого ping-pong DJ_RETRY ↔ babble/code/tool даёт до 9 LLM-вызовов
    на один DJ-переход без единого слова юзера (см. live-логи #1881).
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    music_guard = methods["_apply_music_guard"]
    # Ищем ветку ``if verdict.kind is MusicGuardVerdictKind.DJ_RETRY``.
    found = False
    for node in ast.walk(music_guard):
        if (isinstance(node, ast.If)
                and isinstance(node.test, ast.Compare)
                and any(
                    isinstance(cmp, ast.Attribute)
                    and cmp.attr == "DJ_RETRY"
                    for cmp in ast.walk(node.test)
                )):
            assert _calls(node, "_consume_synthetic_retry"), (
                "DJ_RETRY-ветка не вызывает _consume_synthetic_retry — "
                "общий budget остаётся нетронутым и ping-pong не ограничен"
            )
            found = True
            break
    assert found, "ветка MusicGuardVerdictKind.DJ_RETRY не найдена в _apply_music_guard"


def test_dj_retry_branch_marks_retry_dispatched() -> None:
    """DJ_RETRY обязан звать ``_mark_retry_dispatched()``.

    Без метки babble/tool guard'ы в том же turn'е могут выстрелить ещё раз
    (см. гейт ``_retry_dispatched_in_turn`` в начале ``_apply_music_guard``).
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    music_guard = methods["_apply_music_guard"]
    found = False
    for node in ast.walk(music_guard):
        if (isinstance(node, ast.If)
                and isinstance(node.test, ast.Compare)
                and any(
                    isinstance(cmp, ast.Attribute)
                    and cmp.attr == "DJ_RETRY"
                    for cmp in ast.walk(node.test)
                )):
            assert _calls(node, "_mark_retry_dispatched"), (
                "DJ_RETRY-ветка не вызывает _mark_retry_dispatched — "
                "babble/tool guard'ы могут выстрелить повторно"
            )
            found = True
            break
    assert found, "ветка MusicGuardVerdictKind.DJ_RETRY не найдена в _apply_music_guard"


def test_dj_retry_branch_dispatches_with_from_tick_false() -> None:
    """DJ_RETRY зовёт ``_dispatch_dj_turn(..., from_tick=False)``.

    ``from_tick=False`` отличает синхронный ретрай от свежего DJ-transition
    и НЕ сбрасывает ``MusicGuard._dj_retry_count`` (внутренний счётчик).
    Если ретрай случайно пошёл как свежий тик — guard теряет счёт и
    балансирует между двумя счётчиками.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    music_guard = methods["_apply_music_guard"]
    found = False
    for node in ast.walk(music_guard):
        if (isinstance(node, ast.If)
                and isinstance(node.test, ast.Compare)
                and any(
                    isinstance(cmp, ast.Attribute)
                    and cmp.attr == "DJ_RETRY"
                    for cmp in ast.walk(node.test)
                )):
            dj_calls = [
                c for c in _all_dispatch_calls(node)
                if c.func.attr == "_dispatch_dj_turn"
            ]
            assert dj_calls, (
                "DJ_RETRY-ветка не вызывает _dispatch_dj_turn — "
                "как тогда ретрай дойдёт до LLM?"
            )
            for call in dj_calls:
                from_tick = _kw(call, "from_tick")
                assert from_tick is not None, (
                    "_dispatch_dj_turn в DJ_RETRY вызван без from_tick=...; "
                    "по умолчанию from_tick=False, но явное лучше неявного — "
                    "иначе кто-то рано или поздно «упростит» вызов"
                )
                # ``from_tick=False`` литералом (не переменной, не выражением)
                assert isinstance(from_tick, ast.Constant) and from_tick.value is False, (
                    "DJ_RETRY-ветка передаёт from_tick не литералом False; "
                    "нужен явный False, иначе guard перепутает ретрай с тиком"
                )
            found = True
            break
    assert found, "ветка MusicGuardVerdictKind.DJ_RETRY не найдена в _apply_music_guard"


@pytest.mark.parametrize(
    "guard_name",
    sorted({"_check_babble_and_retry",
            "_check_embedded_renardo_code_and_retry",
            "_check_unbacked_action_claim_and_retry",
            "_apply_music_guard",
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
            "_check_embedded_renardo_code_and_retry",
            "_check_unbacked_action_claim_and_retry",
            "_apply_music_guard",
            "_apply_tool_skipped_guard"}),
)
def test_every_guard_consumes_synthetic_budget(guard_name: str) -> None:
    """Каждый guard, который диспатчит синхронный ретрай, обязан до этого
    списать общий budget через ``_consume_synthetic_retry``.

    Без этого guard теряет бюджет: ping-pong между разными guard'ами
    обходит общий счётчик (см. issue #1881, e209e14c).
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    assert guard_name in methods, f"{guard_name} не существует — тест устарел"
    method = methods[guard_name]

    assert _calls(method, "_consume_synthetic_retry"), (
        f"{guard_name} не зовёт _consume_synthetic_retry — "
        "общий budget _synthetic_retries_left остаётся нетронутым, "
        "guard может пинг-понгом уйти в бесконечный цикл"
    )
