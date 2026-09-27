"""Unit tests for C2+C3 of issue #1995 (operator-agent 07).

C2: ``TaskScheduler.cancel`` actually preempts RUNNING tasks (Phase 2
    of issue #968 §11.6). The MVP could only remove QUEUED tasks; now
    ``cancel`` cancels the executor's :class:`asyncio.Task` and the
    ``CancelledError`` really propagates into the running coroutine.

    ADR-0086 (2026-09-09): C2 used to also publish a ``scheduler.cancel``
    observability envelope on a pub/sub ``EventBus``. That bus (and its
    ownership test, C1) is removed — its only subscriber, the reflex
    bridge, was removed the same day, leaving zero subscribers. This
    file keeps only the preemption-itself assertion, which never
    depended on the bus.

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
# C2 — Cancel actually preempts a RUNNING task
# ---------------------------------------------------------------------------


class TestCancelPreempt:
    """C2 (#1995): cancel() preempts a RUNNING executor."""

    @pytest.mark.asyncio
    async def test_cancel_preempt_propagates_cancelled_error_to_executor(
        self,
    ) -> None:
        """End-to-end sanity check: cancel mid-flight raises inside the
        executor's await, and the pump flips status to CANCELLED.
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
