"""test_llm_continue_hook.py — S10 auto-trigger tests (issue #968 §4.5).

``docs/design/SCHEDULER_DESIGN.md`` §4.5 (revision v5, §4.7 confirms it
calls the MAIN LLM, not a second/light model): after a MERGE/battery-
critical event lands in the scheduler and nobody brings a fresh user
turn to carry the resulting ``task_delta``, the scheduler must self-
drive an out-of-band LLM turn via ``llm_continue_hook`` — but only when
all three conditions hold simultaneously:

1. the group has PENDING (``QUEUED``/``SCHEDULED``) segments,
2. there is at least one unapplied, non-``low``-priority event for
   that group,
3. the owning channel "looks like it needs continuation" — voice:
   just finished its ACTIVE segment (idle, not never-started); music:
   still actively playing (matches the "комар+енот" example in §4.6,
   where MERGE lands while verse 1 is still ACTIVE).

The three explicit anti-patterns from §4.5 are each their own test:
ACTIVE (RUNNING) voice segment, ``priority=low`` events, and no
PENDING segments at all.

Mirrors the style of ``test_task_scheduler.py`` (same ``_make_scheduler``
shape, gated-executor helpers for deterministic RUNNING-state control).
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    EventEnvelope,
    LlmContinueContext,
    SchedulerTask,
    TaskResult,
    TaskScheduler,
    TaskStatus,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_scheduler(**kwargs) -> TaskScheduler:
    sched = TaskScheduler(**kwargs)
    sched.start()
    return sched


def _echo_executor(payload: object = "ok"):
    async def _exec(task: SchedulerTask) -> TaskResult:
        return TaskResult(payload=payload, message=f"ran {task.tool}")

    return _exec


def _gated_executor(gate: asyncio.Event, payload: object = "ok"):
    """Executor that blocks on *gate* — lets tests hold a segment RUNNING."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        await gate.wait()
        return TaskResult(payload=payload, message=f"ran {task.tool}")

    return _exec


def _started_then_gated_executor(started: asyncio.Event, gate: asyncio.Event, payload: object = "ok"):
    """Signals *started* the instant it begins, then blocks on *gate*.

    Used to deterministically observe "segment is now RUNNING" from the
    test coroutine before releasing it.
    """

    async def _exec(task: SchedulerTask) -> TaskResult:
        started.set()
        await gate.wait()
        return TaskResult(payload=payload, message=f"ran {task.tool}")

    return _exec


class _HookRecorder:
    """Records every ``llm_continue_hook`` invocation; asyncio-event-gated."""

    def __init__(self) -> None:
        self.calls: list[LlmContinueContext] = []
        self.event = asyncio.Event()

    async def __call__(self, context: LlmContinueContext) -> None:
        self.calls.append(context)
        self.event.set()


# ---------------------------------------------------------------------------
# True positive — all three conditions line up
# ---------------------------------------------------------------------------


class TestAutoTriggerFires:
    @pytest.mark.asyncio
    async def test_fires_after_active_voice_segment_completes_with_pending_and_event(self):
        """Voice: seg0 finishes (idle), seg1 is PENDING, event is normal-priority."""
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)

            gate1 = asyncio.Event()
            sched.submit(SchedulerTask(
                task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_echo_executor(), group_id="g1", seg_idx=0,
            ))
            sched.submit(SchedulerTask(
                task_id="g1-1", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_gated_executor(gate1), group_id="g1", seg_idx=1,
            ))

            event = EventEnvelope(topic="scheduler.merge.g1", payload={"decision": "MERGE"})
            sched.notify_event("g1", event)

            await asyncio.wait_for(recorder.event.wait(), timeout=1.0)
            assert len(recorder.calls) == 1
            ctx = recorder.calls[0]
            assert ctx.group_id == "g1"
            assert ctx.channel is ChannelKind.VOICE
            # Only the still-PENDING seg1 — seg0 (already COMPLETED) must
            # NOT show up, proving condition (1) reads live scheduler
            # state, not "everything ever submitted to the group".
            assert [s.task_id for s in ctx.pending_segments] == ["g1-1"]
            assert [e.event_id for e in ctx.events] == [event.event_id]

            gate1.set()
            await sched.wait_all()
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_fires_while_music_segment_is_still_playing(self):
        """Music: §4.5 branch is deliberately the opposite of voice — the
        trigger fires WHILE the segment is ACTIVE (matches the
        "комар+енот" §4.6 sequence: MERGE lands mid-verse-1)."""
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)

            started = asyncio.Event()
            gate0 = asyncio.Event()
            sched.submit(SchedulerTask(
                task_id="g1-0", tool="execute_music_code", channel=ChannelKind.MUSIC,
                executor=_started_then_gated_executor(started, gate0),
                group_id="g1", seg_idx=0,
            ))
            sched.submit(SchedulerTask(
                task_id="g1-1", tool="execute_music_code", channel=ChannelKind.MUSIC,
                executor=_echo_executor(), group_id="g1", seg_idx=1,
            ))
            await asyncio.wait_for(started.wait(), timeout=1.0)

            event = EventEnvelope(topic="scheduler.merge.g1", payload={"decision": "MERGE"})
            sched.notify_event("g1", event)

            await asyncio.wait_for(recorder.event.wait(), timeout=1.0)
            assert len(recorder.calls) == 1
            assert recorder.calls[0].channel is ChannelKind.MUSIC

            gate0.set()
            await sched.wait_all()
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Anti-patterns (§4.5, explicitly required by the W2-4 task body)
# ---------------------------------------------------------------------------


class TestAutoTriggerAntiPatterns:
    @pytest.mark.asyncio
    async def test_does_not_fire_while_voice_segment_is_active(self):
        """Anti-pattern 1: ACTIVE (RUNNING) voice segment — too early."""
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)

            started = asyncio.Event()
            gate0 = asyncio.Event()
            sched.submit(SchedulerTask(
                task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_started_then_gated_executor(started, gate0),
                group_id="g1", seg_idx=0,
            ))
            sched.submit(SchedulerTask(
                task_id="g1-1", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_echo_executor(), group_id="g1", seg_idx=1,
            ))
            await asyncio.wait_for(started.wait(), timeout=1.0)
            # seg0 is now RUNNING (ACTIVE) — the anti-pattern window.

            sched.notify_event(
                "g1", EventEnvelope(topic="scheduler.merge.g1", payload={"decision": "MERGE"}),
            )
            # Give the loop a chance to misfire, if it were going to.
            await asyncio.sleep(0.02)
            assert recorder.calls == []

            gate0.set()
            await sched.wait_all()
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_does_not_fire_for_low_priority_event(self):
        """Anti-pattern 2: only a ``priority=low`` event is pending."""
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)

            gate1 = asyncio.Event()
            sched.submit(SchedulerTask(
                task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_echo_executor(), group_id="g1", seg_idx=0,
            ))
            sched.submit(SchedulerTask(
                task_id="g1-1", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_gated_executor(gate1), group_id="g1", seg_idx=1,
            ))
            sched.notify_event(
                "g1",
                EventEnvelope(
                    topic="scheduler.merge.g1", payload={"decision": "MERGE"}, priority="low",
                ),
            )

            # Let seg0 finish (voice channel genuinely goes idle, seg1
            # genuinely PENDING) — the ONLY thing missing is a
            # non-low-priority event, and that alone must suppress the
            # trigger.
            deadline = asyncio.get_event_loop().time() + 1.0
            while sched.get_task("g1-0").status is not TaskStatus.COMPLETED:
                if asyncio.get_event_loop().time() >= deadline:
                    pytest.fail("seg0 never completed")
                await asyncio.sleep(0.005)
            await asyncio.sleep(0.02)
            assert recorder.calls == []

            gate1.set()
            await sched.wait_all()
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_does_not_fire_without_pending_segments(self):
        """Anti-pattern 3: the group has no PENDING segment left to extend."""
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)

            sched.submit(SchedulerTask(
                task_id="g1-0", tool="speak_text", channel=ChannelKind.VOICE,
                executor=_echo_executor(), group_id="g1", seg_idx=0,
            ))
            await sched.wait_all()  # only segment in the group — fully drains

            sched.notify_event(
                "g1", EventEnvelope(topic="scheduler.merge.g1", payload={"decision": "MERGE"}),
            )
            await asyncio.sleep(0.02)
            assert recorder.calls == []
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Hook registration / misc plumbing
# ---------------------------------------------------------------------------


class TestHookRegistration:
    def test_set_llm_continue_hook_none_is_a_valid_reset(self):
        """Mirrors ``set_eta_provider``'s Phase-3-hook pattern (§4.5 hook is
        a plain extension point — the scheduler must not import an LLM
        client to accept/clear it)."""
        sched = TaskScheduler(loop=asyncio.new_event_loop())
        sched.set_llm_continue_hook(None)  # must not raise

    @pytest.mark.asyncio
    async def test_notify_event_on_unknown_group_is_a_noop(self):
        sched = _make_scheduler()
        try:
            recorder = _HookRecorder()
            sched.set_llm_continue_hook(recorder)
            sched.notify_event(
                "no-such-group",
                EventEnvelope(topic="scheduler.merge.x", payload={}),
            )
            await asyncio.sleep(0.02)
            assert recorder.calls == []
        finally:
            sched.shutdown()
