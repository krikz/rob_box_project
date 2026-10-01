"""Issue #3247 — финальный DJ_AUTO-переход не перезапускает сет.

Живой прогон 30.09.2026, сет «море и чайки» (``set6.log``): на
``[DJ_AUTO переход #4 — ФИНАЛЬНЫЙ ТРЕК]`` модель вызвала
``set_dj_mode(enabled=true, theme='пираты', next_transition_sec=45)`` —
тема сета №2, отыгранного 40 минут назад, — и сет шёл ещё три трека.

Проверяем жёсткий гард (не только правило в промпте):

* финальный промпт взводит ``DJState.final_prompted``; конец сета и
  генуинный старт его снимают;
* ``dj_mode.dj_final_turn`` — только для DJ_AUTO-хода;
* ``track_start_guard.dj_restart_refused`` — только
  ``set_dj_mode(enabled=true)`` в финальном DJ_AUTO-ходе;
* ``SchedulerToolExecutor`` такой вызов не исполняет и отдаёт модели
  честный отказ, ``enabled=false`` проходит.
"""

from __future__ import annotations

import asyncio
import json
import logging

import pytest

from rob_box_llm.provider import ToolCall, ToolResult
from rob_box_voice.core.dj_mode import DJModeController, dj_final_turn
from rob_box_voice.core.track_start_guard import (
    DJ_AUTO_FORBIDDEN_ERROR_CODE,
    DJ_FINAL_RESTART_ERROR_CODE,
    dj_restart_refused,
)
from rob_box_voice.core.turn_origin import TURN_DJ_SET_FINAL, TURN_IS_DJ_AUTO
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor


class _Hook:
    persona_default = "Роббокс"

    def __init__(self) -> None:
        self.dispatches: list = []
        self.on_stop = lambda persona: None

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:  # noqa: ARG002
        self.dispatches.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _controller(plan: str, theme: str = "море и чайки") -> DJModeController:
    dj = DJModeController(hook=_Hook(), logger=logging.getLogger("t3247"))
    dj.handle_message(json.dumps({"enabled": True, "theme": theme, "plan": plan}))
    return dj


def _controller_on_final() -> tuple[DJModeController, str]:
    dj = _controller("Трек 1: прибой\nТрек 2: чайки")
    dj.state.tracks_started = 1  # первый трек плана отыгран
    dj.state.transition_count = 2
    return dj, dj.build_auto_prompt(2)


# ── DJModeController ─────────────────────────────────────────────────


def test_final_prompt_marks_set_final_and_states_the_rule() -> None:
    dj, prompt = _controller_on_final()
    assert "ФИНАЛЬНЫЙ ТРЕК" in prompt
    assert "НЕ вызывай set_dj_mode(enabled=true)" in prompt
    assert dj.state.final_prompted is True
    assert dj_final_turn(dj, True) is True
    # Реплика человека в финале («давай ещё!») гардом не режется.
    assert dj_final_turn(dj, False) is False


def test_non_final_transition_does_not_mark() -> None:
    dj = _controller("Трек 1: а\nТрек 2: б\nТрек 3: в", theme="море")
    dj.state.tracks_started = 1
    prompt = dj.build_auto_prompt(2)
    assert "ФИНАЛЬНЫЙ ТРЕК" not in prompt
    assert dj_final_turn(dj, True) is False


def test_set_end_clears_final_mark() -> None:
    dj, _ = _controller_on_final()
    dj.handle_message(json.dumps({"enabled": False}))
    assert dj.state.final_prompted is False
    assert dj_final_turn(dj, True) is False


def test_fresh_start_clears_final_mark() -> None:
    dj, _ = _controller_on_final()
    dj.state.enabled = False  # сет закончился в обход _reset_state
    dj.handle_message(json.dumps({"enabled": True, "theme": "лес"}))
    assert dj.state.final_prompted is False


# ── track_start_guard.dj_restart_refused ─────────────────────────────


@pytest.mark.parametrize(
    ("dj_auto", "dj_final", "args", "refused"),
    [
        (True, True, {"enabled": True, "theme": "пираты"}, True),
        (True, True, {"enabled": "true"}, True),
        (True, True, {"enabled": False}, False),
        (True, True, {"enabled": "false"}, False),
        (True, True, {}, False),
        (True, False, {"enabled": True}, False),
        (False, True, {"enabled": True}, False),
    ],
)
def test_guard_refuses_only_enable_in_final_dj_auto_turn(
    dj_auto: bool, dj_final: bool, args: dict, refused: bool
) -> None:
    auto_token = TURN_IS_DJ_AUTO.set(dj_auto)
    final_token = TURN_DJ_SET_FINAL.set(dj_final)
    try:
        assert bool(dj_restart_refused("set_dj_mode", args)) is refused
        assert bool(dj_restart_refused("compose_music", args)) is False
    finally:
        TURN_DJ_SET_FINAL.reset(final_token)
        TURN_IS_DJ_AUTO.reset(auto_token)


# ── SchedulerToolExecutor ────────────────────────────────────────────


class _Underlying:
    def __init__(self) -> None:
        self.executed: list[ToolCall] = []

    async def discover(self):
        return ()

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call)
        return ToolResult(tool_call_id=call.id, content='{"success": true}')

    async def aclose(self) -> None:
        return None


def _run(calls: list[ToolCall], *, dj_auto: bool, dj_final: bool):
    underlying = _Underlying()
    executor = SchedulerToolExecutor(underlying)

    async def _body() -> list[ToolResult]:
        auto_token = TURN_IS_DJ_AUTO.set(dj_auto)
        final_token = TURN_DJ_SET_FINAL.set(dj_final)
        try:
            executor.begin_turn()
            return [await executor.execute(c) for c in calls]
        finally:
            TURN_DJ_SET_FINAL.reset(final_token)
            TURN_IS_DJ_AUTO.reset(auto_token)

    return asyncio.run(_body()), underlying


def test_executor_refuses_restart_in_final_turn_and_lets_disable_through() -> None:
    restart = ToolCall(
        id="r", name="set_dj_mode",
        arguments={"enabled": True, "theme": "пираты", "next_transition_sec": 45},
    )
    disable = ToolCall(id="d", name="set_dj_mode", arguments={"enabled": False})
    (refused, passed), underlying = _run(
        [restart, disable], dj_auto=True, dj_final=True
    )
    payload = json.loads(refused.content)
    assert refused.is_error is False  # не взводит Bug B (#2966)
    assert payload["success"] is False
    assert payload["error"] == DJ_FINAL_RESTART_ERROR_CODE
    assert "ФИНАЛЬНЫЙ" in payload["message"]
    assert [c.id for c in underlying.executed] == ["d"]
    assert json.loads(passed.content) == {"success": True}


def test_executor_keeps_3246_stop_music_refusal_in_final_turn() -> None:
    # Общая точка отказа (#3246 + #3247): stop_music в финале — тоже отказ.
    stop = ToolCall(id="s", name="stop_music", arguments={})
    (refused,), underlying = _run([stop], dj_auto=True, dj_final=True)
    assert json.loads(refused.content)["error"] == DJ_AUTO_FORBIDDEN_ERROR_CODE
    assert underlying.executed == []


def test_executor_passes_enable_outside_final() -> None:
    call = ToolCall(id="e", name="set_dj_mode", arguments={"enabled": True})
    _results, underlying = _run([call], dj_auto=True, dj_final=False)
    assert [c.id for c in underlying.executed] == ["e"]
