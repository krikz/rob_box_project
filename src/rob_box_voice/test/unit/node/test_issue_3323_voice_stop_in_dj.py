"""Issue #3323 — «стоп музыка» человека не должна отклоняться как DJ_AUTO-ход.

Живой прогон 01.10.2026 16:24Z: media-router исполнил ``stop_music`` и
получил отказ #3246/#3247 ``tool_forbidden_in_dj_auto_turn``; роутер писал
``ok=True``. Причина: ``TrackStartGuard._dj_auto`` выставляет только
``begin_turn()`` хода LLM и он «залипает» от последнего DJ-перехода, а
роутер ``begin_turn`` не зовёт. Происхождение теперь берётся из контекста
исполняемого хода (``TURN_IS_DJ_AUTO``).
"""

from __future__ import annotations

import asyncio
import json

from rob_box_llm.provider import ToolCall, ToolResult

from rob_box_voice.core.media_router import (
    STOP_FAIL_TEXT,
    media_tool_succeeded,
)
from rob_box_voice.core.track_start_guard import DJ_AUTO_FORBIDDEN_ERROR_CODE
from rob_box_voice.core.turn_origin import TURN_IS_DJ_AUTO
from rob_box_voice.scheduler.task_scheduler import TaskScheduler
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor
from test.unit.core.test_issue_2878_dj_speak_limit import _FakeUnderlying

from .test_issue_3134_media_router_node import _make_node, _stt, run_plans  # noqa: F401


class _LazySchedulerExecutor:
    """Настоящий ``SchedulerToolExecutor``; планировщик стартует в loop'е теста."""

    def __init__(self) -> None:
        self.underlying = _FakeUnderlying()
        self.sched = None
        self.inner = None

    def _build(self) -> None:
        self.sched = TaskScheduler()
        self.inner = SchedulerToolExecutor(self.underlying, scheduler=self.sched)
        token = TURN_IS_DJ_AUTO.set(True)
        try:
            self.inner.begin_turn()
        finally:
            TURN_IS_DJ_AUTO.reset(token)
        assert self.inner._track_guard._dj_auto is True

    async def execute(self, call):
        if self.inner is None:
            self._build()
        self.sched.start()
        res = await self.inner.execute(call)
        await asyncio.wait_for(self.sched.wait_all(), timeout=2.0)
        self.sched.shutdown()
        return res


def _stale_dj_auto_executor() -> _LazySchedulerExecutor:
    """Исполнитель, чей гард помнит прошлый DJ_AUTO-ход (как на роботе)."""
    return _LazySchedulerExecutor()


def test_human_stop_via_router_executes_after_dj_auto_turn(run_plans):  # noqa: F811
    n = _make_node(playing=True, dj=True, track="Still Dre", state_name="DIALOGUE")
    ex = _stale_dj_auto_executor()
    n._scheduler_executor = ex
    _stt(n, "Робот стоп музыка")
    run_plans()
    assert [c.name for c in ex.underlying.executed] == ["stop_music"]
    n._speak_direct.assert_called_once()
    assert n._speak_direct.call_args[0][0] != STOP_FAIL_TEXT


def test_llm_in_dj_auto_turn_still_cannot_stop_music() -> None:
    async def _body():
        ex = _stale_dj_auto_executor()
        token = TURN_IS_DJ_AUTO.set(True)
        try:
            res = await ex.execute(ToolCall(id="s", name="stop_music", arguments={}))
        finally:
            TURN_IS_DJ_AUTO.reset(token)
        return res, ex

    res, ex = asyncio.run(_body())
    assert json.loads(res.content)["error"] == DJ_AUTO_FORBIDDEN_ERROR_CODE
    assert ex.underlying.executed == []


def test_media_tool_succeeded_sees_success_false_body() -> None:
    refusal = json.dumps({"success": False, "error": DJ_AUTO_FORBIDDEN_ERROR_CODE})
    assert media_tool_succeeded(False, refusal) is False
    assert media_tool_succeeded(True, "{}") is False
    assert media_tool_succeeded(False, json.dumps({"success": True})) is True
    assert media_tool_succeeded(False, "DJ-режим включён (следующий через 15с)") is True
    assert media_tool_succeeded(False, json.dumps({"status": "queued"})) is True


def test_router_logs_ok_false_and_says_fail_on_success_false_result(run_plans):  # noqa: F811
    class _Refusing:
        async def execute(self, call):
            return ToolResult(
                tool_call_id=call.id,
                content=json.dumps({"success": False, "error": "tool_forbidden"}),
                is_error=False,
            )

    n = _make_node(playing=True, track="Still Dre", state_name="DIALOGUE")
    n._scheduler_executor = _Refusing()
    _stt(n, "Робот стоп музыка")
    run_plans()
    logged = [c.args[0] for c in n.get_logger().info.call_args_list]
    assert any("stop_music" in m and "ok=False" in m for m in logged), logged
    n._speak_direct.assert_called_once_with(STOP_FAIL_TEXT)
