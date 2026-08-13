"""Unit tests for W7b :mod:`rob_box_voice.scheduler.tool_executor`.

Covers the SchedulerToolExecutor contract from
``docs/design/W7_INTEGRATION_PLAN.md`` §W7b:

* ``speak_text`` is queued on the VOICE channel (returns immediately,
  no ``await`` on the side effect — «LLM свободна»);
* ``stop_music`` is deferred until the VOICE channel drains (e2e v36
  regression: stop_music must not outrun the TTS chunk);
* music prelude tools go to the MUSIC channel;
* bypass tools (memory_* / search_* / get_*) execute directly;
* a scheduler failure fails OPEN — the underlying provider is called
  directly and the robot is never silenced by a scheduler bug;
* on_event lifecycle callback receives created/started/completed.
"""

from __future__ import annotations

import asyncio
import json

import pytest

from rob_box_llm.provider import ToolCall, ToolResult
from rob_box_voice.scheduler.task_scheduler import (
    ChannelKind,
    TaskScheduler,
)
from rob_box_voice.scheduler.tool_executor import (
    SchedulerToolExecutor,
    channel_for_tool,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeUnderlying:
    """Records every executed call; returns a JSON echo ToolResult."""

    def __init__(self) -> None:
        self.executed: list[ToolCall] = []
        self.discovered: bool = False

    async def discover(self):
        self.discovered = True
        return ()

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call)
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"ok": True, "tool": call.name}),
            is_error=False,
        )

    async def aclose(self) -> None:
        return None


def _call(cid: str, name: str) -> ToolCall:
    return ToolCall(id=cid, name=name, arguments={})


async def _run_until_complete(
    executor: SchedulerToolExecutor,
    scheduler: TaskScheduler,
) -> None:
    """Give the channel pumps a chance to settle (bounded)."""
    try:
        await asyncio.wait_for(scheduler.wait_all(), timeout=2.0)
    except asyncio.TimeoutError:
        pass


# ---------------------------------------------------------------------------
# Routing
# ---------------------------------------------------------------------------


def test_channel_for_tool_routes_channel_tools() -> None:
    assert channel_for_tool("speak_text") is ChannelKind.VOICE
    assert channel_for_tool("stop_music") is ChannelKind.MUSIC
    assert channel_for_tool("execute_music_code") is ChannelKind.MUSIC
    assert channel_for_tool("set_vibe_preset") is ChannelKind.MUSIC
    assert channel_for_tool("load_track") is ChannelKind.MUSIC
    assert channel_for_tool("play_animation") is ChannelKind.ANIM


def test_channel_for_tool_bypasses_unknown_and_instant_tools() -> None:
    assert channel_for_tool("memory_save") is None
    assert channel_for_tool("search_web") is None
    assert channel_for_tool("get_battery_level") is None
    assert channel_for_tool("estimate_tts_duration") is None
    assert channel_for_tool("navigate_to_waypoint") is None


# ---------------------------------------------------------------------------
# Queued contract («LLM свободна»)
# ---------------------------------------------------------------------------


def test_execute_returns_immediately_with_queued_status() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_call("c1", "speak_text"))
            # Returns instantly — no side effect happened yet.
            assert underlying.executed == []
            payload = json.loads(result.content)
            assert payload["status"] == "queued"
            assert payload["tool"] == "speak_text"
            assert payload["channel"] == "voice"
            assert payload["task_id"]
            assert result.is_error is False
            # Side effect happens asynchronously on the pump.
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            assert len(underlying.executed) == 1
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_two_speak_text_submits_do_not_block_each_other() -> None:
    """Acceptance: два подряд speak_text за < 50мс каждый (submit не блокирует)."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            import time

            t0 = time.monotonic()
            await executor.execute(_call("c1", "speak_text"))
            await executor.execute(_call("c2", "speak_text"))
            submit_ms = (time.monotonic() - t0) * 1000
            assert submit_ms < 50, f"submit took {submit_ms:.1f}ms"
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            assert len(underlying.executed) == 2
            # FIFO order preserved.
            assert [c.id for c in underlying.executed] == ["c1", "c2"]
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_bypass_tools_execute_directly_and_synchronously() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_call("c1", "memory_save"))
            assert underlying.executed == [_call("c1", "memory_save")]
            payload = json.loads(result.content)
            assert payload["ok"] is True
        finally:
            sched.shutdown()

    asyncio.run(_run())


# ---------------------------------------------------------------------------
# stop_music deferral (e2e v36 regression)
# ---------------------------------------------------------------------------


def test_stop_music_waits_for_voice_channel_to_drain() -> None:
    """stop_music must fire strictly after the last speak_text completes."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            # speak_text queued on VOICE; stop_music queued on MUSIC.
            await executor.execute(_call("v1", "speak_text"))
            await executor.execute(_call("m1", "stop_music"))
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            order = [c.name for c in underlying.executed]
            assert order == ["speak_text", "stop_music"], order
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_stop_music_without_voice_fires_immediately() -> None:
    """No queued speech → stop_music must not wait."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            await executor.execute(_call("m1", "stop_music"))
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            assert [c.name for c in underlying.executed] == ["stop_music"]
        finally:
            sched.shutdown()

    asyncio.run(_run())


# ---------------------------------------------------------------------------
# Fail-open safety
# ---------------------------------------------------------------------------


def test_scheduler_init_failure_falls_back_to_direct_execution() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        executor = SchedulerToolExecutor(underlying, scheduler=None)
        # Sabotage scheduler creation → _ensure_scheduler() returns None.
        executor._scheduler_attempted = True  # noqa: SLF001 — test escape hatch
        result = await executor.execute(_call("c1", "speak_text"))
        # Fail-open: direct execution, real result, not "queued".
        assert underlying.executed == [_call("c1", "speak_text")]
        assert json.loads(result.content)["ok"] is True

    asyncio.run(_run())


def test_scheduler_submit_failure_falls_back_to_direct_execution() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            # Sabotage submit to force the fail-open path.
            def _bad_submit(*_args, **_kwargs):
                raise RuntimeError("boom")
            sched.submit = _bad_submit  # type: ignore[assignment]
            result = await executor.execute(_call("c1", "execute_music_code"))
            assert underlying.executed == [_call("c1", "execute_music_code")]
            assert json.loads(result.content)["ok"] is True
        finally:
            sched.shutdown()

    asyncio.run(_run())


# ---------------------------------------------------------------------------
# W7c lifecycle events
# ---------------------------------------------------------------------------


def test_on_event_receives_lifecycle_events() -> None:
    underlying = _FakeUnderlying()
    events: list[tuple[str, str]] = []

    def _on_event(event: str, payload: dict) -> None:
        events.append((event, payload["tool"]))

    async def _run() -> None:
        sched = TaskScheduler(on_event=_on_event)
        sched.start()
        executor = SchedulerToolExecutor(
            underlying, scheduler=sched, on_event=_on_event
        )
        try:
            await executor.execute(_call("c1", "speak_text"))
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            kinds = [e[0] for e in events]
            assert "task.created" in kinds
            assert "task.started" in kinds
            assert "task.completed" in kinds
            # No failed/cancelled events in the happy path.
            assert "task.failed" not in kinds
            assert "task.cancelled" not in kinds
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_active_tasks_block_reflects_busy_channels() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            # Idle scheduler → empty block.
            assert executor.active_tasks_block() == ""
            await executor.execute(_call("c1", "speak_text"))
            # The VOICE channel is busy (queued or running) → block present.
            block = executor.active_tasks_block()
            assert "ACTIVE TASKS" in block
            assert "voice" in block
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            assert executor.active_tasks_block() == ""
        finally:
            sched.shutdown()

    asyncio.run(_run())
