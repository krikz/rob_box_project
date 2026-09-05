"""Unit tests for W7b :mod:`rob_box_voice.scheduler.tool_executor`.

Covers the SchedulerToolExecutor contract from
``docs/design/W7_INTEGRATION_PLAN.md`` §W7b:

* ``speak_text`` is queued on the VOICE channel (returns immediately,
  no ``await`` on the side effect — «LLM свободна»);
* ``stop_music`` is deferred until the VOICE channel drains (e2e v36
  regression: stop_music must not outrun the TTS chunk);
* music starters (``execute_music_code`` / ``set_vibe_preset`` /
  ``load_track``) bypass the scheduler and execute blocking (party
  regression live 19.08 — the LLM must see the real result);
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
    SchedulerTask,
    TaskResult,
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
    assert channel_for_tool("play_animation") is ChannelKind.ANIM


def test_channel_for_tool_bypasses_music_starters() -> None:
    # Music starters run BLOCKING (bypass) so the LLM sees the real
    # result — only stop_music stays on the scheduler (party regression).
    assert channel_for_tool("execute_music_code") is None
    assert channel_for_tool("set_vibe_preset") is None
    assert channel_for_tool("load_track") is None


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
            result = await executor.execute(_call("c1", "speak_text"))
            assert underlying.executed == [_call("c1", "speak_text")]
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


# ---------------------------------------------------------------------------
# S2.3 — begin_group() / seg_idx (scheduler-segments-merge plan)
# ---------------------------------------------------------------------------


def test_begin_group_assigns_shared_group_id_and_increasing_seg_idx() -> None:
    """Multiple speak_text calls in one LLM batch share a group_id and
    get a rising seg_idx (0, 1, 2, ...)."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            executor.begin_group()
            r1 = await executor.execute(_call("c1", "speak_text"))
            r2 = await executor.execute(_call("c2", "speak_text"))
            t1 = sched.get_task(json.loads(r1.content)["task_id"])
            t2 = sched.get_task(json.loads(r2.content)["task_id"])
            assert t1.group_id is not None
            assert t1.group_id == t2.group_id
            assert t1.seg_idx == 0
            assert t2.seg_idx == 1
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_begin_group_next_batch_gets_a_new_group_id() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            executor.begin_group()
            r1 = await executor.execute(_call("c1", "speak_text"))
            executor.begin_group()
            r2 = await executor.execute(_call("c2", "speak_text"))
            t1 = sched.get_task(json.loads(r1.content)["task_id"])
            t2 = sched.get_task(json.loads(r2.content)["task_id"])
            assert t1.group_id != t2.group_id
            assert t2.seg_idx == 0
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_without_begin_group_tasks_are_ungrouped() -> None:
    """Backward compat: calling execute() without begin_group() first
    must keep producing group_id=None (today's behaviour)."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            r1 = await executor.execute(_call("c1", "speak_text"))
            t1 = sched.get_task(json.loads(r1.content)["task_id"])
            assert t1.group_id is None
            assert t1.seg_idx is None
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        finally:
            sched.shutdown()

    asyncio.run(_run())


# ---------------------------------------------------------------------------
# S5.1 — [SEGMENT PLAN] block (scheduler-segments-merge plan)
# ---------------------------------------------------------------------------


def test_segment_plan_block_empty_when_no_active_group() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            # begin_group() never called — no group has ever been open.
            assert executor.segment_plan_block() == ""
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_segment_plan_block_empty_before_scheduler_exists() -> None:
    underlying = _FakeUnderlying()
    executor = SchedulerToolExecutor(underlying)  # no scheduler=, lazy-created
    assert executor.segment_plan_block() == ""


def test_segment_plan_block_shows_active_and_pending() -> None:
    block_event = asyncio.Event()

    class _BlockingUnderlying(_FakeUnderlying):
        async def execute(self, call: ToolCall) -> ToolResult:
            if call.id == "c1":
                await block_event.wait()
            return await super().execute(call)

    underlying = _BlockingUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            executor.begin_group()
            await executor.execute(
                ToolCall(id="c1", name="speak_text", arguments={"text": "куплет про комара"})
            )
            await executor.execute(
                ToolCall(id="c2", name="speak_text", arguments={"text": "куплет 2"})
            )
            # Give the pump a moment to pick up c1 (RUNNING, blocked).
            await asyncio.sleep(0.02)

            block = executor.segment_plan_block()
            assert block.startswith("[SEGMENT PLAN]")
            assert "ACTIVE:" in block
            assert "seg_0" in block
            assert "куплет про комара" in block
            assert "PENDING:" in block
            assert "seg_1" in block
            assert "REWRITEABLE_SEGMENTS: [seg_1]" in block
            assert "AT_RISK_ON_REPLACE: [seg_1]" in block

            block_event.set()
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_segment_plan_block_excludes_frozen_from_rewriteable() -> None:
    """S9.2 (§6.5): REWRITEABLE_SEGMENTS lists only PENDING_LIVE segments;
    AT_RISK_ON_REPLACE still lists every PENDING segment, FROZEN included —
    a REPLACE verdict blows away the whole group regardless of pre-gen
    state."""
    block_event = asyncio.Event()

    class _BlockingUnderlying(_FakeUnderlying):
        async def execute(self, call: ToolCall) -> ToolResult:
            if call.id == "c1":
                await block_event.wait()
            return await super().execute(call)

    underlying = _BlockingUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            group_id = executor.begin_group()
            await executor.execute(
                ToolCall(id="c1", name="speak_text", arguments={"text": "куплет про комара"})
            )
            await executor.execute(
                ToolCall(id="c2", name="speak_text", arguments={"text": "куплет про кузнечика"})
            )
            await executor.execute(
                ToolCall(id="c3", name="speak_text", arguments={"text": "куплет про енота"})
            )
            # Give the pump a moment to pick up c1 (RUNNING, blocked).
            await asyncio.sleep(0.02)

            # seg_1 (c2) is already pre-gen'd/in flight → FROZEN.
            # seg_2 (c3) is not yet reached → LIVE.
            sched.set_group_boundary(group_id, 2)

            block = executor.segment_plan_block()
            assert "seg_1" in block
            assert "seg_2" in block
            assert "REWRITEABLE_SEGMENTS: [seg_2]" in block
            assert "AT_RISK_ON_REPLACE: [seg_1, seg_2]" in block

            block_event.set()
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_segment_plan_block_empty_after_group_completes() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            executor.begin_group()
            await executor.execute(_call("c1", "speak_text"))
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            assert executor.segment_plan_block() == ""
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


# ---------------------------------------------------------------------------
# S6.2 — task_delta bypasses the queue entirely (scheduler-segments-merge)
#
# mcp_server (a SEPARATE ROS2 process) only advertises task_delta's schema
# (S6.1, TaskDeltaTool) — it has no access to this in-process
# TaskScheduler. The real execution must happen HERE, intercepted before
# ``channel_for_tool``'s general queued/bypass split, applied directly via
# TaskScheduler.update() (S3.2), same bypass philosophy as the music
# starters: the LLM must see the REAL per-op result, never a
# fire-and-forget {"status": "queued"}.
# ---------------------------------------------------------------------------


def _delta_call(group_id: str, ops: list) -> ToolCall:
    return ToolCall(
        id="d1", name="task_delta", arguments={"group_id": group_id, "ops": ops}
    )


def test_task_delta_rewrites_pending_segment_and_returns_real_outcome() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        block = asyncio.Event()

        async def blocked(task: SchedulerTask) -> TaskResult:
            await block.wait()
            return TaskResult(payload=dict(task.args))

        sched.submit(SchedulerTask(
            task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
            executor=blocked, args={"text": "verse0"}, group_id="g1", seg_idx=0,
        ))
        sched.submit(SchedulerTask(
            task_id="g1-1", tool="speak_text", channel=ChannelKind.VOICE,
            executor=blocked, args={"text": "verse1"}, group_id="g1", seg_idx=1,
        ))
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_delta_call(
                "g1",
                [{"kind": "rewrite", "seg_idx": 1, "args": {"text": "verse1 про енота"}}],
            ))
            payload = json.loads(result.content)
            # NOT the fire-and-forget queued contract (bypass, not queue).
            assert "status" not in payload
            assert payload["success"] is True
            assert payload["outcomes"] == [
                {
                    "kind": "rewrite",
                    "seg_idx": 1,
                    "applied": True,
                    "task_id": "g1-1",
                    "reason": "",
                }
            ]
            assert result.is_error is False
            segs = sched.segments("g1")
            assert segs[1].args == {"text": "verse1 про енота"}
            # The bypass never touched the underlying provider directly —
            # it went through TaskScheduler.update().
            assert underlying.executed == []
        finally:
            block.set()
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            sched.shutdown()

    asyncio.run(_run())


def test_task_delta_running_segment_ignored_not_error() -> None:
    """§2.3 invariant surfaces through the bypass too: RUNNING is never
    touched, but the tool call itself still succeeds (applied=False)."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        block = asyncio.Event()

        async def blocked(task: SchedulerTask) -> TaskResult:
            await block.wait()
            return TaskResult(payload=dict(task.args))

        sched.submit(SchedulerTask(
            task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
            executor=blocked, args={"text": "verse0"}, group_id="g1", seg_idx=0,
        ))
        # Let the pump pick up seg 0 (RUNNING) before we try to touch it.
        for _ in range(50):
            if sched.get_task("g1-0").status.name == "RUNNING":
                break
            await asyncio.sleep(0.01)
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_delta_call(
                "g1", [{"kind": "rewrite", "seg_idx": 0, "args": {"text": "hacked"}}]
            ))
            payload = json.loads(result.content)
            assert payload["success"] is True
            assert payload["outcomes"][0]["applied"] is False
            assert "RUNNING" in payload["outcomes"][0]["reason"]
        finally:
            block.set()
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            sched.shutdown()

    asyncio.run(_run())


def test_task_delta_append_dispatches_new_segment_through_underlying() -> None:
    """append's new segment must still reach the real ROS side effect
    (via the underlying provider) once the channel pump runs it."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()

        async def instant(task: SchedulerTask) -> TaskResult:
            return TaskResult(payload=dict(task.args))

        sched.submit(SchedulerTask(
            task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
            executor=instant, args={"text": "verse0"}, group_id="g1", seg_idx=0,
        ))
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_delta_call(
                "g1", [{"kind": "append", "args": {"text": "verse2 про енота"}}]
            ))
            payload = json.loads(result.content)
            assert payload["success"] is True
            assert payload["outcomes"][0]["kind"] == "append"
            assert payload["outcomes"][0]["applied"] is True
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            # The appended segment's executor dispatched a real speak_text
            # call to the underlying provider (not a bypass no-op).
            appended = [
                c for c in underlying.executed
                if c.arguments.get("text") == "verse2 про енота"
            ]
            assert len(appended) == 1
            assert appended[0].name == "speak_text"
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_task_delta_empty_group_id_rejected_honestly() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_delta_call("", [{"kind": "drop", "seg_idx": 0}]))
            payload = json.loads(result.content)
            assert payload["success"] is False
            assert result.is_error is True
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_task_delta_unknown_group_returns_honest_error() -> None:
    underlying = _FakeUnderlying()

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            result = await executor.execute(_delta_call(
                "no-such-group", [{"kind": "drop", "seg_idx": 0}]
            ))
            payload = json.loads(result.content)
            assert payload["success"] is False
            assert payload["error"] == "group_not_found"
            assert result.is_error is True
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_task_delta_falls_back_to_underlying_when_scheduler_unavailable() -> None:
    """Fail-open, same pattern as every other bypass path in this module:
    a dead scheduler must not silence task_delta — it goes to mcp_server's
    own capability-honest ``scheduler_unavailable`` failure instead."""
    underlying = _FakeUnderlying()

    async def _run() -> None:
        executor = SchedulerToolExecutor(underlying, scheduler=None)
        executor._scheduler_attempted = True  # noqa: SLF001 — test escape hatch
        result = await executor.execute(_delta_call("g1", [{"kind": "drop", "seg_idx": 0}]))
        assert underlying.executed == [_delta_call("g1", [{"kind": "drop", "seg_idx": 0}])]
        # Delegated verbatim to the underlying provider (mcp_server's
        # TaskDeltaTool), not fabricated here.
        assert json.loads(result.content)["ok"] is True

    asyncio.run(_run())


def test_segment_plan_block_exposes_group_id_for_task_delta() -> None:
    """Блок обязан печатать ``group_id`` — без него ``task_delta`` не вызвать.

    ``task_delta`` (``rob_box_mcp_tools/tools/scheduler.py``) объявляет
    ``group_id`` обязательным параметром и описывает его как «task_id
    активной группы сегментов из [SEGMENT PLAN]». Но блок печатал только
    метки сегментов (``seg_0``, ``seg_1``) — самого идентификатора группы
    в нём не было ни в каком виде.

    Значит модель, следуя RULE #SEGMENT-PLAN из
    ``master_prompt_compact.txt``, обязана передать значение, которого ей
    никогда не показывали. Любая догадка не совпадёт с реальным id
    (``uuid4().hex``, 32 символа), и ``_execute_task_delta`` вернёт
    ``{"success": false, "error": "group_not_found"}``.

    То есть MERGE — ради которого весь S5/S6 и делался — был недостижим:
    перебить исполнение и вплести правку в ещё не сыгранные сегменты
    модель не могла, ей оставался только перезапуск с начала.
    """

    async def _run() -> None:
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(_FakeUnderlying(), scheduler=sched)
        try:
            group_id = executor.begin_group()
            await executor.execute(
                ToolCall(id="c0", name="speak_text", arguments={"text": "куплет 1"})
            )
            await executor.execute(
                ToolCall(id="c1", name="speak_text", arguments={"text": "куплет 2"})
            )
            block = executor.segment_plan_block()
            assert group_id in block, (
                "group_id не выведен в [SEGMENT PLAN]; task_delta требует "
                f"его обязательным аргументом. Блок:\n{block}"
            )
        finally:
            sched.shutdown()

    asyncio.run(_run())


def test_task_created_carries_group_and_segment_index() -> None:
    """``task.created`` должен нести ``group_id``/``seg_idx``.

    Без них по ``/harness/task_events`` нельзя ответить на вопрос, от
    которого зависит вся сегментная модель: приезжает выступление одним
    батчем или по куску за итерацию тул-цикла.

    ``begin_group()`` вызывается ``agent_core`` на КАЖДЫЙ батч
    (``tool_executor.begin_group``), поэтому число разных ``group_id`` за
    тёрн = числу итераций, а ``seg_idx`` внутри группы = размеру батча.
    Оба поля у задачи уже есть (``SchedulerTask.group_id``/``seg_idx``),
    в событие они просто не попадали — измерить было нечем.

    Контракт ``W7_INTEGRATION_PLAN.md`` §W7c уже содержит ``seg_idx``
    (``task.segment_started(id, seg_idx)``), так что поля не чужеродны.
    """
    payloads: list[dict] = []

    def _on_event(event: str, payload: dict) -> None:
        if event == "task.created":
            payloads.append(payload)

    async def _run() -> None:
        sched = TaskScheduler(on_event=_on_event)
        sched.start()
        executor = SchedulerToolExecutor(
            _FakeUnderlying(), scheduler=sched, on_event=_on_event
        )
        try:
            group_id = executor.begin_group()
            await executor.execute(
                ToolCall(id="c0", name="speak_text", arguments={"text": "куплет 1"})
            )
            await executor.execute(
                ToolCall(id="c1", name="speak_text", arguments={"text": "куплет 2"})
            )
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)

            assert len(payloads) == 2, payloads
            assert [p.get("seg_idx") for p in payloads] == [0, 1]
            assert {p.get("group_id") for p in payloads} == {group_id}
        finally:
            sched.shutdown()

    asyncio.run(_run())
