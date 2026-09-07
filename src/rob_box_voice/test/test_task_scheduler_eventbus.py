"""Unit tests for C1 of issue #1995 (operator-agent 07).

C1: ``TaskScheduler`` owns an :class:`EventBus` at init time and
    exposes it as :attr:`TaskScheduler.event_bus`.

The tests live in their own file (rather than being merged into
``test_task_scheduler.py``) so the Phase-2 / Phase-3 split is
visible in the test index and so the EventBus-specific setup
(``asyncio.Queue`` per subscriber, ``fnmatch`` topic matching)
doesn't pollute the MVP suite.

C2 (cancel-preempt via EventBus) and C3 (fail-loud on scheduler
init failure) land in their own commits and add their tests
in later edits to this file.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.scheduler import (
    EventBus,
    EventEnvelope,
    TaskScheduler,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


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