"""Unit tests for C1+C2 of issue #1995 (operator-agent 07).

C1: ``TaskScheduler`` owns an :class:`EventBus` at init time and
    exposes it as :attr:`TaskScheduler.event_bus`.

C2: ``TaskScheduler.cancel`` actually preempts RUNNING tasks via
    the EventBus (Phase 2 of issue #968 §11.6). The MVP could
    only remove QUEUED tasks; now ``cancel`` cancels the
    executor's :class:`asyncio.Task` and publishes a
    ``scheduler.cancel`` envelope so subscribers see the
    preemption.

The tests live in their own file (rather than being merged into
``test_task_scheduler.py``) so the Phase-2 / Phase-3 split is
visible in the test index and so the EventBus-specific setup
(``asyncio.Queue`` per subscriber, ``fnmatch`` topic matching)
doesn't pollute the MVP suite.

C3 (fail-loud on scheduler init failure) lives in the same file
under :class:`TestFailLoudOnSchedulerFailure` and asserts the
executor raises :class:`RuntimeError` instead of silently
bypassing the queue.
"""

from __future__ import annotations

import asyncio
import threading

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    EventBus,
    EventEnvelope,
    SchedulerTask,
    TaskResult,
    TaskScheduler,
    TaskStatus,
)
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeUnderlying:
    """Stand-in for ``AgentCore``'s tool provider.

    Records every tool call passed to ``execute``. Used by the
    C3 test to assert the executor does NOT silently bypass to
    the underlying provider when the scheduler is unavailable.
    """

    def __init__(self) -> None:
        self.executed: list[str] = []

    async def execute(self, call):  # noqa: ARG002 — protocol shape
        self.executed.append(call.name)
        return _result_ok(call.name)


def _result_ok(name: str):
    from rob_box_llm.provider import ToolResult

    return ToolResult(
        id=f"r-{name}",
        name=name,
        content=f"{{\"status\": \"ok\", \"tool\": \"{name}\"}}",
    )


def _make_scheduler() -> TaskScheduler:
    sched = TaskScheduler()
    sched.start()
    return sched


# ---------------------------------------------------------------------------
# C1 — TaskScheduler owns EventBus at init time
# ---------------------------------------------------------------------------


class TestEventBusOwnership:
    """C1 (#1995): the scheduler creates and owns its EventBus."""

    @pytest.mark.asyncio
    async def test_scheduler_creates_event_bus_on_init(self) -> None:
        """Every TaskScheduler has a working :class:`EventBus`."""
        sched = _make_scheduler()
        try:
            assert isinstance(sched.event_bus, EventBus)
            # Sanity: bus is open and accepts subscriptions.
            sub = sched.event_bus.subscribe("test.*")
            try:
                env = EventEnvelope(topic="test.hello", payload={"k": 1})
                delivered = await sched.event_bus.publish(env)
                assert delivered == 1
                received = await asyncio.wait_for(sub.get(), timeout=0.5)
                assert received.payload == {"k": 1}
            finally:
                sub.close()
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# C2 — Cancel preempts RUNNING tasks via EventBus
# ---------------------------------------------------------------------------


class TestCancelPreemptViaEventBus:
    """C2 (#1995): cancel() routes through EventBus for preemption."""

    @pytest.mark.asyncio
    async def test_cancel_publishes_scheduler_cancel_envelope(self) -> None:
        """Both QUEUED-cancel and RUNNING-cancel paths emit one envelope
        on topic ``scheduler.cancel`` with task_id and reason.
        """
        sched = _make_scheduler()
        try:
            sub = sched.event_bus.subscribe("scheduler.cancel")
            try:
                # Submit two tasks, cancel the first while QUEUED,
                # cancel the second while RUNNING.
                blocker_done = threading.Event()

                async def blocker(task: SchedulerTask) -> TaskResult:  # noqa: ARG001
                    blocker_done.set()
                    await asyncio.sleep(0.3)
                    return TaskResult(payload="blocker")

                t1 = sched.submit(SchedulerTask(
                    task_id="t1", tool="speak_text",
                    channel=ChannelKind.VOICE, executor=blocker,
                ))
                t2 = sched.submit(SchedulerTask(
                    task_id="t2", tool="speak_text",
                    channel=ChannelKind.VOICE, executor=blocker,
                ))
                # t1 is running, t2 is queued. Wait for t1 to enter
                # the executor before issuing cancels so we have
                # deterministic ordering.
                while not blocker_done.is_set():
                    await asyncio.sleep(0.001)

                # Cancel t2 while QUEUED.
                assert sched.cancel("t2") is True
                env_q = await asyncio.wait_for(sub.get(), timeout=0.5)
                assert env_q.topic == "scheduler.cancel"
                assert env_q.payload["task_id"] == "t2"
                assert env_q.payload["reason"] == "cancelled before start"

                # Cancel t1 while RUNNING.
                assert sched.cancel("t1") is True
                env_r = await asyncio.wait_for(sub.get(), timeout=0.5)
                assert env_r.topic == "scheduler.cancel"
                assert env_r.payload["task_id"] == "t1"
                assert env_r.payload["reason"] == "cancelled mid-flight"

                await sched.wait_all()
                assert t1.status is TaskStatus.CANCELLED
                assert t2.status is TaskStatus.CANCELLED
            finally:
                sub.close()
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_cancel_preempt_propagates_cancelled_error_to_executor(
        self,
    ) -> None:
        """End-to-end sanity check: cancel mid-flight raises inside the
        executor's await, the pump flips status to CANCELLED, and the
        EventBus envelope is delivered to a subscriber registered before
        the cancel. Covers the integration between C1 (EventBus) and C2
        (preempt).
        """
        sched = _make_scheduler()
        try:
            entered = threading.Event()
            caught: dict[str, BaseException | None] = {"err": None}

            async def slow(task: SchedulerTask) -> TaskResult:  # noqa: ARG001
                entered.set()
                try:
                    await asyncio.sleep(1.0)
                    return TaskResult(payload="should not see this")
                except BaseException as exc:  # noqa: BLE001 — record
                    caught["err"] = exc
                    raise

            t = sched.submit(SchedulerTask(
                task_id="alive", tool="speak_text",
                channel=ChannelKind.VOICE, executor=slow,
            ))
            while not entered.is_set():
                await asyncio.sleep(0.001)

            assert sched.cancel("alive") is True
            await sched.wait_all()
            assert t.status is TaskStatus.CANCELLED
            # The executor's await was hit with CancelledError,
            # proving preemption actually reaches the coroutine.
            assert isinstance(caught["err"], asyncio.CancelledError)
        finally:
            sched.shutdown()


# ---------------------------------------------------------------------------
# C3 — Fail-LOUD, not fail-open
# ---------------------------------------------------------------------------


class TestFailLoudOnSchedulerFailure:
    """C3 (#1995): the scheduler executor raises on init failure.

    The previous fail-open path swallowed every
    ``TaskScheduler(...)`` / ``.start()`` error and returned
    ``None``, causing ``SchedulerToolExecutor.execute`` to
    silently bypass the queue. That regressed the v36 fix
    (``stop_music`` could again outrun ``speak_text``), and
    operators never learned the scheduler was broken.

    New contract: the executor raises :class:`RuntimeError` on
    init failure; the second call after a permanent failure
    raises again (no silent retry loop, but no silent bypass
    either).
    """

    @pytest.mark.asyncio
    async def test_ensure_scheduler_raises_when_task_scheduler_init_fails(
        self, monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        """Patch TaskScheduler.__init__ to raise — the executor must
        surface that as a :class:`RuntimeError`, not swallow it.

        The lazy ``_scheduler_attempted`` flag prevents a retry
        loop: the SECOND call raises the same error with a
        diagnostic message.
        """
        from rob_box_voice.scheduler import tool_executor as te_mod

        original_init = te_mod.TaskScheduler.__init__

        def _boom(self, *args, **kwargs):  # noqa: ANN001
            raise RuntimeError("simulated init failure")

        monkeypatch.setattr(te_mod.TaskScheduler, "__init__", _boom)

        underlying = _FakeUnderlying()
        executor = SchedulerToolExecutor(underlying, on_event=None)

        from rob_box_llm.provider import ToolCall

        call = ToolCall(id="c1", name="speak_text", arguments={})

        # First call: the init raises through.
        with pytest.raises(RuntimeError, match="simulated init failure"):
            await executor.execute(call)
        # Bypass must NOT have happened — no direct execution.
        assert underlying.executed == [], (
            "C3 (#1995): executor must not silently bypass to the "
            "underlying provider when the scheduler fails to come up"
        )

        # Second call: must raise again (with the diagnostic message).
        with pytest.raises(RuntimeError, match="previous init failed"):
            await executor.execute(call)
        assert underlying.executed == [], (
            "C3 (#1995): executor must not silently bypass on retry"
        )

        # Restore so monkeypatch teardown is clean.
        monkeypatch.setattr(te_mod.TaskScheduler, "__init__", original_init)

    @pytest.mark.asyncio
    async def test_ensure_scheduler_succeeds_after_init_recovery(
        self,
    ) -> None:
        """Once :class:`TaskScheduler` comes up, ``_ensure_scheduler``
        returns it on subsequent calls — the positive path stays
        cheap.
        """
        underlying = _FakeUnderlying()
        executor = SchedulerToolExecutor(underlying, on_event=None)
        # No scheduler injected — the executor must build its own.
        sched = executor._ensure_scheduler()  # noqa: SLF001
        assert sched is executor._scheduler  # noqa: SLF001
        # Second call returns the cached instance.
        assert executor._ensure_scheduler() is sched  # noqa: SLF001