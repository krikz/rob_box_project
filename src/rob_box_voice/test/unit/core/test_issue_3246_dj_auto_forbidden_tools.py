"""Issue #3246 — stop_music запрещён в DJ_AUTO-ходе (жёсткий гард + промпт).

Живой прогон 30.09.2026, переход #3: LLM сама вызвала ``stop_music`` ->
~5 с цифровой тишины посреди сета.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time

from rob_box_llm.provider import ToolCall
from rob_box_voice.core.dj_mode import DJModeController
from rob_box_voice.core.track_start_guard import (
    DJ_AUTO_FORBIDDEN_ERROR_CODE,
    DJ_AUTO_FORBIDDEN_TOOLS,
    TrackStartGuard,
)
from rob_box_voice.core.turn_origin import TURN_IS_DJ_AUTO
from rob_box_voice.scheduler.task_scheduler import TaskScheduler
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor
from test.unit.core.test_issue_2878_dj_speak_limit import _FakeUnderlying


def test_guard_forbids_only_in_dj_auto_turn() -> None:
    guard = TrackStartGuard()
    guard.reset(dj_auto=True)
    assert guard.should_refuse_forbidden("stop_music") is True
    assert guard.should_refuse_forbidden("compose_music") is False
    guard.reset(dj_auto=False)
    assert guard.should_refuse_forbidden("stop_music") is False


def test_stop_music_is_in_forbidden_list() -> None:
    assert "stop_music" in DJ_AUTO_FORBIDDEN_TOOLS


async def _run(dj_auto: bool):
    underlying = _FakeUnderlying()
    sched = TaskScheduler()
    sched.start()
    executor = SchedulerToolExecutor(underlying, scheduler=sched)
    TURN_IS_DJ_AUTO.set(dj_auto)
    try:
        executor.begin_turn()
        res = await executor.execute(
            ToolCall(id="s", name="stop_music", arguments={}))
        await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        return res, underlying
    finally:
        sched.shutdown()


def test_executor_refuses_stop_music_in_dj_auto_turn() -> None:
    res, underlying = asyncio.run(_run(True))
    payload = json.loads(res.content)
    assert payload["success"] is False
    assert payload["error"] == DJ_AUTO_FORBIDDEN_ERROR_CODE
    assert "compose_music" in payload["message"]
    assert underlying.executed == []


def test_executor_allows_stop_music_in_user_turn() -> None:
    res, underlying = asyncio.run(_run(False))
    assert json.loads(res.content).get("status") == "queued"
    assert [c.name for c in underlying.executed] == ["stop_music"]


class _Hook:
    def __init__(self) -> None:
        self.prompts: list[str] = []
        self.persona_default = "Роббокс"
        self.on_stop = None

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:
        self.prompts.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def test_transition_prompt_forbids_stop_music() -> None:
    hook = _Hook()
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"))
    ctrl.state.enabled = True
    ctrl.state.transition_count = 2  # не «СТАРТ ВЕЧЕРИНКИ»
    ctrl.state.next_transition_at = time.time() - 1.0
    ctrl.state.form_ends_at = time.time() - 1.0
    ctrl.tick()
    assert hook.prompts, "переход не продиспатчен"
    assert "НИКОГДА не вызывай stop_music" in hook.prompts[0]
