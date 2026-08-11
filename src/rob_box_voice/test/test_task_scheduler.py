"""Unit tests for the Phase 1 MVP :mod:`rob_box_voice.scheduler`.

Acceptance criteria (per issue #968 §11.1, as captured in the
t_feac4bb8 task body):

* Unit tests cover the basic cycle — ``enqueue → schedule →
  execute → complete``.
* FIFO ordering per channel is preserved.
* Different channels do not block each other.
* Errors are surfaced as ``FAILED`` without taking down the
  pump; subsequent tasks keep running.
* ``cancel`` removes only QUEUED tasks; running tasks run to
  completion (Phase 2 will add preemption).
* ``channel_status`` / ``wait_all`` give the §7 ``[CHANNELS]``
  block the data it needs.

These tests deliberately do NOT touch
:mod:`rob_box_harness.core.acceptance` — Phase 1.5 has its own
suite (``test_acceptance_gate.py``) and the MVP scheduler is
designed to live below the acceptance layer.
"""

from __future__ import annotations

import asyncio
import threading
import time

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    ChannelStatus,
    SchedulerTask,
    TaskResult,
    TaskScheduler,
    TaskStatus,
    TaskSubmitError,
    TaskNotFoundError,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_scheduler(
    *, channels: tuple[ChannelKind, ...] = TaskScheduler.DEFAULT_CHANNELS,
) -> TaskScheduler:
    """Build a scheduler and immediately start its pumps.

    Returns an already-started scheduler; tests should call
    ``.shutdown()`` in a finally block.
    """
    sched = TaskScheduler(channels=channels)
    sched.start()
    return sched


def _echo_executor(payload: object) -> "_AsyncFactory":
    """Return a callable that yields ``payload`` wrapped in :class:`TaskResult`."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        return TaskResult(payload=payload, message=f"ran {task.tool}")

    _exec.__name__ = f"_echo_{id(payload)}"
    return _exec


class _AsyncFactory:
    """Marker for helpers that produce coroutines — clearer than bare lambdas."""


def _sleepy_executor(duration_s: float, payload: object) -> "_AsyncFactory":
    """Return a callable that sleeps *duration_s* then returns *payload*."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        await asyncio.sleep(duration_s)
        return TaskResult(payload=payload, message=f"ran {task.tool}")

    return _exec


def _raising_executor(exc: BaseException) -> "_AsyncFactory":
    """Return a callable that raises *exc* when awaited."""

    async def _exec(task: SchedulerTask) -> TaskResult:  # noqa: ARG001
        raise exc

    return _exec


# ---------------------------------------------------------------------------
# Construction / lifecycle
# ---------------------------------------------------------------------------


class TestConstruction:
    def test_requires_event_loop(self):
        """Scheduler must be constructed inside a running asyncio loop.

        Without ``loop=`` and outside a loop the ctor must raise
        ``RuntimeError`` — otherwise the channel ``asyncio.Lock``
        ends up bound to the wrong loop and deadlocks.
        """
        with pytest.raises(RuntimeError, match="asyncio loop"):
            TaskScheduler()

    def test_requires_at_least_one_channel(self):
        with pytest.raises(TaskSubmitError):
            TaskScheduler(channels=())


class TestLifecycle:
    @pytest.mark.asyncio
    async def test_start_is_idempotent(self):
        sched = _make_scheduler()
        try:
            sched.start()  # second call must not raise
            sched.start()
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_shutdown_is_idempotent(self):
        sched = _make_scheduler()
        sched.shutdown()
        sched.shutdown()  # second call must not raise


# ---------------------------------------------------------------------------
# Basic enqueue → schedule → execute → complete
# ---------------------------------------------------------------------------


class TestBasicCycle:
    @pytest.mark.asyncio
    async def test_single_task_completes(self):
        sched = _make_scheduler()
        try:
            task = sched.submit(SchedulerTask(
                task_id="t1",
                tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor({"speech_id": "abc"}),
                args={"text": "hi"},
            ))
            assert task.status is TaskStatus.QUEUED
            await sched.wait_all()
            assert task.status is TaskStatus.COMPLETED
            assert task.result is not None
            assert task.result.payload == {"speech_id": "abc"}
            assert task.started_at is not None
            assert task.finished_at is not None
            assert task.finished_at >= task.started_at
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_assigns_task_id_when_empty(self):
        sched = _make_scheduler()
        try:
            task = sched.submit(SchedulerTask(
                task_id="",
                tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor("ok"),
            ))
            assert task.task_id, "task_id must be assigned when empty"
            assert len(task.task_id) == 32  # uuid4 hex
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_unknown_channel_is_rejected(self):
        sched = _make_scheduler(channels=(ChannelKind.VOICE,))
        try:
            with pytest.raises(TaskSubmitError, match="unknown channel"):
                # MUSIC is a valid enum value but not present in
                # this scheduler's channel set — proves the
                # rejection is per-scheduler, not per-enum.
                sched.submit(SchedulerTask(
                    task_id="x",
                    tool="execute_music_code",
                    channel=ChannelKind.MUSIC,
                    executor=_echo_executor("ok"),
                ))
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_missing_executor_is_rejected(self):
        sched = _make_scheduler()
        try:
            with pytest.raises(TaskSubmitError, match="executor is required"):
                sched.submit(SchedulerTask(
                    task_id="x",
                    tool="speak_text",
                    channel=ChannelKind.VOICE,
                    executor=None,
                ))
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_submit_after_shutdown_is_rejected(self):
        sched = _make_scheduler()
        sched.shutdown()
        with pytest.raises(TaskSubmitError, match="shut down"):
            sched.submit(SchedulerTask(
                task_id="x",
                tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor("ok"),
            ))


# ---------------------------------------------------------------------------
# FIFO ordering & per-channel serialization
# ---------------------------------------------------------------------------


class TestFifoOrdering:
    @pytest.mark.asyncio
    async def test_per_channel_serial_execution(self):
        """Two VOICE tasks must run strictly sequentially — no overlap.

        The MVP enforces this with an ``asyncio.Lock`` per channel
        so two TTS requests never share the audio device.
        """
        sched = _make_scheduler()
        try:
            order: list[str] = []
            overlap = threading.Event()
            overlap_holder: list[bool] = []

            async def slow(task: SchedulerTask) -> TaskResult:
                if overlap.is_set():
                    overlap_holder.append(True)
                overlap.set()
                order.append(f"{task.task_id}:start")
                await asyncio.sleep(0.02)
                overlap.clear()
                order.append(f"{task.task_id}:end")
                return TaskResult(payload=task.task_id)

            t1 = sched.submit(SchedulerTask(task_id="t1", tool="speak_text",
                                            channel=ChannelKind.VOICE,
                                            executor=slow, args={"text": "first"}))
            t2 = sched.submit(SchedulerTask(task_id="t2", tool="speak_text",
                                            channel=ChannelKind.VOICE,
                                            executor=slow, args={"text": "second"}))

            await sched.wait_all()
            assert t1.status is TaskStatus.COMPLETED
            assert t2.status is TaskStatus.COMPLETED
            # Strict FIFO: t1 end before t2 start.
            assert order == ["t1:start", "t1:end", "t2:start", "t2:end"]
            assert overlap_holder == [], "second task entered the lock while the first was still running"
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_different_channels_do_not_block_each_other(self):
        """A slow VOICE task must NOT delay a parallel MUSIC task.

        Regression for issue #968: the legacy code serialised
        everything through ``_output_lock``, which is exactly
        what we are removing in Phase 1.
        """
        sched = _make_scheduler()
        try:
            music_done = threading.Event()

            async def slow_voice(task: SchedulerTask) -> TaskResult:
                await asyncio.sleep(0.05)
                return TaskResult(payload="voice")

            async def fast_music(task: SchedulerTask) -> TaskResult:
                music_done.set()
                return TaskResult(payload="music")

            voice_task = sched.submit(SchedulerTask(
                task_id="v1", tool="speak_text",
                channel=ChannelKind.VOICE, executor=slow_voice,
            ))
            music_task = sched.submit(SchedulerTask(
                task_id="m1", tool="execute_music_code",
                channel=ChannelKind.MUSIC, executor=fast_music,
            ))

            # MUSIC must finish well before VOICE (50ms voice vs ~0ms music).
            deadline = time.monotonic() + 0.04
            while not music_done.is_set():
                if time.monotonic() > deadline:
                    break
                await asyncio.sleep(0.005)

            assert music_done.is_set(), (
                "MUSIC task did not finish before VOICE — channels are blocking each other"
            )
            assert music_task.status is TaskStatus.COMPLETED
            assert voice_task.status is not TaskStatus.COMPLETED  # still in flight
            await sched.wait_all()
            assert voice_task.status is TaskStatus.COMPLETED
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Error handling — one bad task must not break the pump
# ---------------------------------------------------------------------------


class TestErrorHandling:
    @pytest.mark.asyncio
    async def test_failing_task_marks_failed_and_keeps_pump_alive(self):
        sched = _make_scheduler()
        try:
            failing = sched.submit(SchedulerTask(
                task_id="bad", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_raising_executor(RuntimeError("boom")),
            ))
            ok = sched.submit(SchedulerTask(
                task_id="ok", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor("ok"),
            ))
            await sched.wait_all()
            assert failing.status is TaskStatus.FAILED
            assert "RuntimeError: boom" in (failing.error or "")
            assert ok.status is TaskStatus.COMPLETED, (
                "pump must keep running after a failed task"
            )
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_failing_task_does_not_poison_channel_status(self):
        sched = _make_scheduler()
        try:
            sched.submit(SchedulerTask(
                task_id="bad", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_raising_executor(ValueError("nope")),
            ))
            await sched.wait_all()
            status = sched.channel_status(ChannelKind.VOICE)
            assert status.current_task_id is None
            assert status.current_tool is None
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Cancellation — MVP can remove QUEUED tasks, but cannot preempt running ones
# ---------------------------------------------------------------------------


class TestCancellation:
    @pytest.mark.asyncio
    async def test_cancel_queued_task(self):
        """A task that has not started yet can be removed from the queue."""
        sched = _make_scheduler()
        try:
            # Block the head of the queue so the second task stays QUEUED.
            blocker_done = threading.Event()

            async def blocker(task: SchedulerTask) -> TaskResult:
                blocker_done.set()
                await asyncio.sleep(0.05)
                return TaskResult(payload="blocker")

            async def victim(task: SchedulerTask) -> TaskResult:
                blocker_done.set()
                return TaskResult(payload="victim")

            t1 = sched.submit(SchedulerTask(
                task_id="blocker", tool="speak_text",
                channel=ChannelKind.VOICE, executor=blocker,
            ))
            t2 = sched.submit(SchedulerTask(
                task_id="victim", tool="speak_text",
                channel=ChannelKind.VOICE, executor=victim,
            ))
            # Wait for the blocker to be running so t2 is guaranteed queued.
            await asyncio.sleep(0.01)
            assert t1.status in (TaskStatus.SCHEDULED, TaskStatus.RUNNING)
            assert t2.status is TaskStatus.QUEUED

            removed = sched.cancel("victim")
            assert removed is True

            await sched.wait_all()
            assert t1.status is TaskStatus.COMPLETED
            assert t2.status is TaskStatus.CANCELLED
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_cancel_unknown_task_returns_false(self):
        sched = _make_scheduler()
        try:
            assert sched.cancel("does-not-exist") is False
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_cancel_running_task_is_noop_in_mvp(self):
        """MVP cannot preempt a running task — it must run to completion.

        This documents the Phase 1 limitation. Phase 2 wires the
        :class:`SchedulerEventBus` for proper preemption.
        """
        sched = _make_scheduler()
        try:
            started = threading.Event()

            async def slow(task: SchedulerTask) -> TaskResult:
                started.set()
                await asyncio.sleep(0.05)
                return TaskResult(payload="ran")

            t1 = sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE, executor=slow,
            ))
            # Wait until the executor is in flight.
            while not started.is_set():
                await asyncio.sleep(0.001)
            cancelled = sched.cancel("t1")
            assert cancelled is False, (
                "MVP must not claim to have cancelled a running task"
            )
            await sched.wait_all()
            assert t1.status is TaskStatus.COMPLETED
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Channel status snapshots
# ---------------------------------------------------------------------------


class TestChannelStatus:
    @pytest.mark.asyncio
    async def test_status_reflects_current_and_queue_depth(self):
        sched = _make_scheduler()
        try:
            started = threading.Event()

            async def hold(task: SchedulerTask) -> TaskResult:
                started.set()
                await asyncio.sleep(0.05)
                return TaskResult(payload="ok")

            t1 = sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE, executor=hold,
            ))
            t2 = sched.submit(SchedulerTask(
                task_id="t2", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor("queued"),
            ))
            await asyncio.sleep(0.01)
            started.wait(1.0)

            voice_status: ChannelStatus = sched.channel_status(ChannelKind.VOICE)
            assert voice_status.kind is ChannelKind.VOICE
            assert voice_status.current_task_id == "t1"
            assert voice_status.current_tool == "speak_text"
            # t2 is still queued (channel lock is held by t1).
            assert voice_status.queue_depth >= 1
            assert voice_status.eta_s is None, "MVP has no ETA provider"
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_all_statuses_returns_every_channel(self):
        sched = _make_scheduler()
        try:
            statuses = sched.all_statuses()
            assert set(statuses.keys()) == set(TaskScheduler.DEFAULT_CHANNELS)
            for s in statuses.values():
                assert isinstance(s, ChannelStatus)
                assert s.queue_depth == 0
                assert s.current_task_id is None
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_status_for_unknown_channel_raises(self):
        sched = _make_scheduler(channels=(ChannelKind.VOICE,))
        try:
            with pytest.raises(TaskSubmitError, match="unknown channel"):
                sched.channel_status(ChannelKind.MUSIC)
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_eta_provider_is_phase3_hook(self):
        """``set_eta_provider`` populates the §7 ``[CHANNELS]`` ETA column."""
        sched = _make_scheduler()
        try:
            captured = asyncio.Event()

            async def slow(task: SchedulerTask) -> TaskResult:
                # Hold the channel lock long enough for the test
                # to read ``current_task_id`` deterministically.
                await asyncio.sleep(0.05)
                captured.set()
                return TaskResult(payload="ok")

            sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE, executor=slow,
            ))
            # Yield to the pump so the channel lock is acquired
            # before we read status.
            await asyncio.sleep(0.005)
            assert sched.channel_status(ChannelKind.VOICE).current_task_id == "t1"

            def eta(task: SchedulerTask) -> float:
                return 12.5

            sched.set_eta_provider(eta)
            voice_status = sched.channel_status(ChannelKind.VOICE)
            assert voice_status.eta_s == 12.5

            # Clearing the provider falls back to None.
            sched.set_eta_provider(None)
            assert sched.channel_status(ChannelKind.VOICE).eta_s is None
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# wait() / wait_all() helpers
# ---------------------------------------------------------------------------


class TestWaitHelpers:
    @pytest.mark.asyncio
    async def test_wait_returns_when_task_completes(self):
        sched = _make_scheduler()
        try:
            t1 = sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_sleepy_executor(0.02, "ok"),
            ))
            await sched.wait_all()
            t = sched.wait("t1")
            assert t.status is TaskStatus.COMPLETED
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_wait_unknown_raises(self):
        sched = _make_scheduler()
        try:
            with pytest.raises(TaskNotFoundError):
                sched.wait("nope")
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_wait_timeout(self):
        sched = _make_scheduler()
        try:
            sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_sleepy_executor(0.2, "ok"),
            ))
            with pytest.raises(asyncio.TimeoutError):
                sched.wait("t1", timeout=0.02)
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_wait_all_blocks_until_every_channel_drains(self):
        sched = _make_scheduler()
        try:
            sched.submit(SchedulerTask(
                task_id="v", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_sleepy_executor(0.03, "ok"),
            ))
            sched.submit(SchedulerTask(
                task_id="m", tool="execute_music_code",
                channel=ChannelKind.MUSIC,
                executor=_sleepy_executor(0.03, "ok"),
            ))
            sched.submit(SchedulerTask(
                task_id="a", tool="play_animation",
                channel=ChannelKind.ANIM,
                executor=_sleepy_executor(0.03, "ok"),
            ))
            await sched.wait_all()
            assert sched.channel_status(ChannelKind.VOICE).queue_depth == 0
            assert sched.channel_status(ChannelKind.MUSIC).queue_depth == 0
            assert sched.channel_status(ChannelKind.ANIM).queue_depth == 0
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# Snapshot introspection
# ---------------------------------------------------------------------------


class TestSnapshot:
    @pytest.mark.asyncio
    async def test_tasks_snapshot_excludes_executor_callable(self):
        """The snapshot must be JSON-serialisable — no coroutines."""
        import json

        sched = _make_scheduler()
        try:
            t1 = sched.submit(SchedulerTask(
                task_id="t1", tool="speak_text",
                channel=ChannelKind.VOICE,
                executor=_echo_executor("ok"),
            ))
            await sched.wait_all()
            snap = sched.tasks_snapshot()
            assert "t1" in snap
            # Must round-trip through JSON.
            json.dumps(snap["t1"])
        finally:
            sched.shutdown()