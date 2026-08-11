"""Phase 3 unit tests: SpeculativePreGenerator (§6.5 runtime-budget)."""
from __future__ import annotations

import asyncio
import time

import pytest

from rob_box_voice.scheduler import (
    EventBus,
    EventEnvelope,
    PreGenCandidate,
    PreGenCancelledError,
    PreGenPlan,
    SegmentEstimate,
    SpeculativePreGenerator,
)


def _candidate(
    task_id: str,
    duration_ms: float | None,
    *,
    confidence: float = 0.7,
    metadata: dict | None = None,
) -> PreGenCandidate:
    return PreGenCandidate(
        task_id=task_id,
        estimate=SegmentEstimate(
            duration_ms=duration_ms,
            cost=1.0,
            confidence=confidence,
        ),
        metadata=metadata or {},
    )


# ---------------------------------------------------------------------------
# Constructor / planner
# ---------------------------------------------------------------------------


def test_planner_rejects_negative_safety_margin() -> None:
    with pytest.raises(ValueError, match="safety_margin_ms"):
        SpeculativePreGenerator(safety_margin_ms=-1.0)


def test_build_plan_returns_none_boundary_for_zero_eta() -> None:
    pre = SpeculativePreGenerator()
    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", 2000.0),
        _candidate("c", 1500.0),
    ]
    plan = pre.build_plan(candidates, llm_eta_ms=0.0)
    # Cumulative 1000 + 500 margin = 1500 already past zero-eta
    # budget of 500 → boundary should land on the very first segment.
    assert plan.boundary_idx == 0
    assert plan.skipped_ids == ()


def test_build_plan_finds_boundary_at_first_segment_past_eta_plus_margin() -> None:
    pre = SpeculativePreGenerator(safety_margin_ms=500.0)
    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", 2000.0),
        _candidate("c", 1500.0),
    ]
    # LLM ETA 2500 → budget = 2500 + 500 = 3000.
    # cumulative: a=1000 (< 3000), b=3000 (= 3000) → boundary=1.
    plan = pre.build_plan(candidates, llm_eta_ms=2500.0)
    assert plan.boundary_idx == 1
    assert plan.cumulative_ms == pytest.approx(3000.0)
    assert plan.skipped_ids == ()


def test_build_plan_skips_segments_with_unknown_duration() -> None:
    pre = SpeculativePreGenerator()
    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", None),
        _candidate("c", 2000.0),
    ]
    plan = pre.build_plan(candidates, llm_eta_ms=1000.0, safety_margin_ms=500.0)
    assert plan.skipped_ids == ("b",)
    # cumulative = 1000 (a) + skip + 2000 (c) = 3000; budget = 1500;
    # the segment that crosses 1500 is "a" itself (1000 < 1500 → no),
    # then c=3000 (>= 1500) → boundary at the index of c, which is 2.
    assert plan.boundary_idx == 2
    assert plan.cumulative_ms == pytest.approx(3000.0)


def test_build_plan_returns_none_boundary_when_llm_is_very_slow() -> None:
    pre = SpeculativePreGenerator()
    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", 2000.0),
    ]
    plan = pre.build_plan(candidates, llm_eta_ms=20_000.0)
    # budget = 20000 + 500 = 20500; cumulative = 3000 → no boundary.
    assert plan.boundary_idx is None
    assert plan.cumulative_ms == pytest.approx(3000.0)


def test_build_plan_rejects_negative_inputs() -> None:
    pre = SpeculativePreGenerator()
    with pytest.raises(ValueError, match="llm_eta_ms"):
        pre.build_plan([_candidate("a", 1000.0)], llm_eta_ms=-1.0)
    with pytest.raises(ValueError, match="safety_margin_ms"):
        pre.build_plan(
            [_candidate("a", 1000.0)], llm_eta_ms=100.0, safety_margin_ms=-10.0
        )


def test_build_plan_handles_empty_candidates() -> None:
    pre = SpeculativePreGenerator()
    plan = pre.build_plan([], llm_eta_ms=1000.0)
    assert plan.boundary_idx is None
    assert plan.cumulative_ms == 0.0
    assert plan.skipped_ids == ()


# ---------------------------------------------------------------------------
# Runtime — scheduled / completed / claimed
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_run_schedules_only_past_boundary_and_caches_payload() -> None:
    pre = SpeculativePreGenerator(safety_margin_ms=500.0)
    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", 2000.0),
        _candidate("c", 1500.0),
    ]
    plan = pre.build_plan(candidates, llm_eta_ms=2500.0)
    assert plan.boundary_idx == 1

    async def factory(c: PreGenCandidate):
        await asyncio.sleep(0)
        return c.task_id, {"synth": c.task_id}

    scheduled = await pre.run(plan, candidates, factory)
    assert scheduled == ("b", "c")
    await pre.wait_all()
    assert pre.has_payload("b")
    assert pre.has_payload("c")
    assert pre.claim("b") == {"synth": "b"}
    # Claim is one-shot.
    assert not pre.has_payload("b")
    assert pre.completed_count == 2
    assert pre.cancelled_count == 0
    assert pre.failed_count == 0


@pytest.mark.asyncio
async def test_run_skips_when_boundary_is_none() -> None:
    pre = SpeculativePreGenerator()

    async def factory(c: PreGenCandidate):
        return c.task_id, {"x": 1}

    scheduled = await pre.run(
        PreGenPlan(boundary_idx=None, cumulative_ms=0.0, skipped_ids=()),
        [],
        factory,
    )
    assert scheduled == ()


@pytest.mark.asyncio
async def test_run_rejects_after_shutdown() -> None:
    pre = SpeculativePreGenerator()

    async def factory(c: PreGenCandidate):
        return c.task_id, {}

    await pre.shutdown()
    with pytest.raises(RuntimeError, match="closed"):
        await pre.run(
            PreGenPlan(boundary_idx=0, cumulative_ms=1000.0, skipped_ids=()),
            [_candidate("a", 1000.0)],
            factory,
        )


@pytest.mark.asyncio
async def test_run_does_not_reschedule_already_claimed_candidate() -> None:
    pre = SpeculativePreGenerator()
    candidates = [_candidate("a", 1000.0)]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=1000.0, skipped_ids=())

    calls: list[str] = []

    async def factory(c: PreGenCandidate):
        calls.append(c.task_id)
        return c.task_id, {"x": 1}

    scheduled_first = await pre.run(plan, candidates, factory)
    await pre.wait_all()
    assert scheduled_first == ("a",)
    # Claiming the cached result removes it from the cache.
    pre.claim("a")
    # A second run for the SAME candidate must reschedule because
    # the cache is empty (claim transferred ownership).
    scheduled_second = await pre.run(plan, candidates, factory)
    await pre.wait_all()
    assert scheduled_second == ("a",)
    assert calls == ["a", "a"]


# ---------------------------------------------------------------------------
# Cancellation
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_cancel_terminates_in_flight_tasks_and_emits_event() -> None:
    pre = SpeculativePreGenerator()
    started: list[str] = []
    finished: list[str] = []

    async def factory(c: PreGenCandidate):
        started.append(c.task_id)
        try:
            await asyncio.sleep(5.0)  # long enough to be cancelled
            finished.append(c.task_id)
        except asyncio.CancelledError:
            raise
        return c.task_id, {}

    candidates = [
        _candidate("a", 1000.0),
        _candidate("b", 2000.0),
    ]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=3000.0, skipped_ids=())
    await pre.run(plan, candidates, factory)
    # Yield once so both factories enter their sleep.
    await asyncio.sleep(0)
    assert sorted(started) == ["a", "b"]
    cancelled = await pre.cancel(reason="merge_touched_frozen")
    assert cancelled == 2
    assert pre.cancelled_count == 2
    assert pre.last_cancel_reason == "merge_touched_frozen"
    # No payload survives a cancellation.
    assert not pre.has_payload("a")
    assert not pre.has_payload("b")
    assert finished == []


@pytest.mark.asyncio
async def test_cancel_sets_event_for_cooperative_stop() -> None:
    pre = SpeculativePreGenerator()
    observed: list[bool] = []
    factory_started = asyncio.Event()

    async def factory(c: PreGenCandidate):
        factory_started.set()
        try:
            deadline = time.monotonic() + 0.5
            while not pre.cancel_event.is_set():
                if time.monotonic() >= deadline:
                    break
                await asyncio.sleep(0.001)
            return c.task_id, {}
        finally:
            # Record the observed state at the very last moment —
            # either we exit the loop cleanly, or ``cancel()``
            # already set the event and we are being torn down.
            observed.append(pre.cancel_event.is_set())

    candidates = [_candidate("a", 1000.0)]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=1000.0, skipped_ids=())
    await pre.run(plan, candidates, factory)
    # Wait until the factory actually starts polling the event.
    await asyncio.wait_for(factory_started.wait(), timeout=0.5)
    await pre.cancel(reason="cooperative_stop")
    assert observed == [True]


@pytest.mark.asyncio
async def test_cancel_is_safe_when_no_tasks_active() -> None:
    pre = SpeculativePreGenerator()
    cancelled = await pre.cancel(reason="noop")
    assert cancelled == 0
    assert pre.last_cancel_reason == "noop"


# ---------------------------------------------------------------------------
# Failure isolation
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_factory_exception_is_counted_and_does_not_corrupt_state() -> None:
    pre = SpeculativePreGenerator()

    async def flaky_factory(c: PreGenCandidate):
        if c.task_id == "bad":
            raise RuntimeError("tts-engine-offline")
        await asyncio.sleep(0)
        return c.task_id, {"ok": c.task_id}

    candidates = [
        _candidate("good", 1000.0),
        _candidate("bad", 1000.0),
    ]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=2000.0, skipped_ids=())
    await pre.run(plan, candidates, flaky_factory)
    await pre.wait_all()
    assert pre.completed_count == 1
    assert pre.failed_count == 1
    assert pre.cancelled_count == 0
    assert pre.has_payload("good")
    assert not pre.has_payload("bad")


# ---------------------------------------------------------------------------
# Telemetry — EventBus integration
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_emits_scheduled_completed_and_cancelled_events() -> None:
    bus = EventBus()
    subscription = bus.subscribe("pregen.*", max_queue_size=16)
    pre = SpeculativePreGenerator(event_bus=bus)

    async def factory(c: PreGenCandidate):
        if c.task_id == "slow":
            await asyncio.sleep(0.05)
        else:
            await asyncio.sleep(0)
        return c.task_id, {}

    candidates = [
        _candidate("fast", 1000.0),
        _candidate("slow", 1000.0),
    ]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=2000.0, skipped_ids=())
    try:
        await pre.run(plan, candidates, factory)
        # Cancel before the slow task finishes.
        await asyncio.sleep(0)  # let the loop schedule
        await pre.cancel(reason="telemetry_test")
        # Drain the topic stream.
        topics: list[str] = []
        # Two scheduled + at least two completed/cancelled — we
        # accept either order depending on scheduling luck.
        for _ in range(6):
            try:
                topics.append((await asyncio.wait_for(subscription.get(), timeout=0.5)).topic)
            except asyncio.TimeoutError:
                break
        assert topics.count("pregen.scheduled") == 2
        # Either completed (fast) or cancelled (slow) for each.
        assert topics.count("pregen.completed") + topics.count("pregen.cancelled") >= 1
    finally:
        subscription.close()
        await bus.close()


@pytest.mark.asyncio
async def test_telemetry_publish_failure_does_not_break_caller() -> None:
    class _BrokenBus:
        async def publish(self, event: EventEnvelope) -> int:
            raise RuntimeError("bus offline")

    pre = SpeculativePreGenerator(event_bus=_BrokenBus())  # type: ignore[arg-type]

    async def factory(c: PreGenCandidate):
        return c.task_id, {"ok": 1}

    candidates = [_candidate("a", 1000.0)]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=1000.0, skipped_ids=())
    await pre.run(plan, candidates, factory)
    await pre.wait_all()
    assert pre.completed_count == 1


# ---------------------------------------------------------------------------
# Reset / diagnostics
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_reset_clears_caches_and_counters_without_cancelling() -> None:
    pre = SpeculativePreGenerator()

    async def factory(c: PreGenCandidate):
        return c.task_id, {"x": 1}

    candidates = [_candidate("a", 1000.0)]
    plan = PreGenPlan(boundary_idx=0, cumulative_ms=1000.0, skipped_ids=())
    await pre.run(plan, candidates, factory)
    await pre.wait_all()
    assert pre.completed_count == 1
    pre.reset()
    assert pre.completed_count == 0
    assert pre.cancelled_count == 0
    assert pre.failed_count == 0
    assert pre.last_cancel_reason is None
    # The cached payload was wiped; the next claim raises.
    assert not pre.has_payload("a")


def test_snapshot_reports_internals() -> None:
    pre = SpeculativePreGenerator()
    snap = pre.snapshot()
    assert snap == {
        "completed": 0,
        "cancelled": 0,
        "failed": 0,
        "active": 0,
        "cached": 0,
        "last_cancel_reason": None,
    }


# ---------------------------------------------------------------------------
# Cooperative exception type
# ---------------------------------------------------------------------------


def test_pre_gen_cancelled_error_carries_metadata() -> None:
    err = PreGenCancelledError("seg_3", reason="merge_touched_frozen")
    assert err.task_id == "seg_3"
    assert err.reason == "merge_touched_frozen"
    assert isinstance(err, asyncio.CancelledError)
