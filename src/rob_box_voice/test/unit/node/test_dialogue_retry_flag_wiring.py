"""
test_dialogue_retry_flag_wiring.py — предохранитель от «робот просто акцептит».

🔴 Живой инцидент 30.08.2026. Коммит `0e7bb478` («play composed Renardo code
instead of reading it aloud») добавил флаг ``is_code_retry`` в сигнатуру
``DialogueNode._dispatch_turn``, но ЧИТАЛ его в теле ``_run_turn``, куда флаг
не добавили и не пробросили::

    # _run_turn, ~26-я строка от начала функции
    if not is_code_retry:            # <-- NameError: name 'is_code_retry'
        self._code_speech_retry_used = False

``NameError`` падал ДО вызова LLM, на КАЖДОМ ходе, внутри asyncio-таски.
Снаружи это выглядело так: STT принимал фразу («✅ ПРИНЯТО»), проигрывался
звук подтверждения — и робот замолкал навсегда. Коммит откатили целиком
(`db0fba22`), вместе с полезной частью фикса.

Ошибка структурная и повторяемая: каждый новый guard (Bug B/C/D/E/C′) заводит
свой одноразовый флаг, и каждый обязан пройти три места — сигнатуру
``_dispatch_turn``, проброс в ``_run_turn`` и сигнатуру ``_run_turn``.
Тест проверяет все три статически, без ROS2 и без импорта ноды.
"""

import ast
from pathlib import Path

import pytest


DIALOGUE_NODE = (
    Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "dialogue_node.py"
)


def _dialogue_node_class() -> ast.ClassDef:
    tree = ast.parse(DIALOGUE_NODE.read_text(encoding="utf-8"))
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DialogueNode"
    )


def _methods() -> dict:
    return {
        fn.name: fn
        for fn in _dialogue_node_class().body
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def _params(fn) -> set:
    a = fn.args
    return {arg.arg for arg in a.posonlyargs + a.args + a.kwonlyargs}


def _names_read(fn) -> set:
    return {
        node.id
        for node in ast.walk(fn)
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
    }


def _retry_flags(names) -> set:
    """Флаги вида ``is_*_retry`` — те, что заводит каждый новый guard.

    Сужено намеренно: ``is_metrics_enabled`` — импортированная функция, а не
    локальный флаг, и в этот отбор попадать не должна.
    """
    return {n for n in names if n.startswith("is_") and n.endswith("_retry")}


@pytest.mark.parametrize("method_name", ["_run_turn", "_dispatch_turn"])
def test_retry_flags_are_declared_where_they_are_read(method_name: str) -> None:
    """Каждый ``is_*_retry``, который метод читает, обязан быть его параметром.

    Именно это и сломалось в `0e7bb478`: ``_run_turn`` читал ``is_code_retry``,
    не объявляя его.
    """
    fn = _methods()[method_name]
    params = _params(fn)
    unbound = sorted(_retry_flags(_names_read(fn)) - params)
    assert not unbound, (
        f"{method_name} читает {unbound}, но не объявляет их в сигнатуре — "
        f"это NameError на каждом ходе (см. инцидент 0e7bb478)"
    )


def test_every_retry_flag_is_forwarded_to_run_turn() -> None:
    """``_dispatch_turn`` обязан пробросить в ``_run_turn`` все retry-флаги.

    Флаг, объявленный в обеих сигнатурах, но не переданный в вызове, тихо
    остаётся дефолтным — одноразовый бюджет ретрая сбрасывается на самом
    ретрае, и guard уходит в бесконечный пинг-понг с LLM.
    """
    methods = _methods()
    dispatch, run_turn = methods["_dispatch_turn"], methods["_run_turn"]

    call = next(
        node
        for node in ast.walk(dispatch)
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
