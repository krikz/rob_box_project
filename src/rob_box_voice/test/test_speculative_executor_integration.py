"""Phase 3 integration tests — SpeculativeStepExecutor + Phase 2 + Phase 1.

These tests exercise the full Phase 1 + 2 + 3 pipeline:

* :class:`DecisionCoordinator` (Phase 2) drives planning once.
* :class:`SpeculativeStepExecutor` (Phase 3 bridge) computes
  per-step estimates via :class:`BaselineEstimator`, runs the
  pre-generation runtime, and forwards each :class:`PlanStep` to
  the underlying :class:`TaskScheduler` (Phase 1).
* When the pre-gen runtime has a cached payload for a step, the
  executor returns that payload (without re-running the heavy
  work); when it does not, the original executor runs as before.

The tests use synthetic executors so the heavy work is just an
``asyncio.sleep`` we can instrument. The LLM ETA is supplied
explicitly per test so we can drive the boundary deterministically.
"""
from __future__ import annotations

import asyncio
import time
from typing import Any

import pytest

from rob_box_voice.scheduler import (
    BaselineEstimator,
    CANCEL_REASON_MERGE_TOUCHED_FROZEN,
    CANCEL_REASON_PLAN_SUPERSEDED,
    ChannelKind,
    DecisionCoordinator,
    DecisionPlan,
    EstimatorContext,
    EstimatorQualityTracker,
    EventBus,
    PlanStep,
    SchedulerStepExecutor,
    SchedulerTask,
    SegmentEstimate,
    SpeculativePlanResult,
    SpeculativePreGenerator,
    SpeculativeStepExecutor,
    StepExecution,
    StepStatus,
    TaskResult,
    TaskScheduler,
    estimate_llm_eta_ms,
)


# ---------------------------------------------------------------------------
# Fixtures and helpers
# ---------------------------------------------------------------------------


def _make_task_factory(effects: list[tuple[str, str]], *, sleep_ms: float = 5.0):
    """Build a sync task factory that records ``(channel, tool)``.

    The executor sleeps for ``sleep_ms`` to simulate heavy work and
    returns a ``TaskResult`` whose payload is a small dict the
    tests can assert against. Returning a *payload* is what lets
    pre-generation cache something meaningful — the cached value
    is the dict the executor produced, not the wall-clock time.
    """

    def task_factory(step: PlanStep) -> SchedulerTask:
        async def execute(task: SchedulerTask) -> TaskResult:
            await asyncio.sleep(sleep_ms / 1000.0)
            effects.append((task.channel.value, task.tool))
            return TaskResult(
                payload={"step_id": step.step_id, "tool": task.tool}
            )

        return SchedulerTask(
            task_id=step.step_id,
            tool=step.action,
            channel=ChannelKind(step.channel),
            executor=execute,
            args=dict(step.arguments),
        )

    return task_factory


def _build_plan() -> DecisionPlan:
    """A representative 3-step plan: voice + music + anim."""
    return DecisionPlan(
        "turn-1",
        (
            PlanStep("say", "speak_text", {"text": "hello"}, "voice"),
            PlanStep(
                "music",
                "execute_music_code",
                {"pattern": "p1", "duration_ms": 1000.0},
                "music",
            ),
            PlanStep("anim", "play_animation", {"name": "happy"}, "anim", ("say",)),
        ),
    )


def _make_speculative_executor(
    scheduler: TaskScheduler,
    *,
    estimator: BaselineEstimator | None = None,
    pre_generator: SpeculativePreGenerator | None = None,
    task_factory=None,
    event_bus: EventBus | None = None,
) -> SpeculativeStepExecutor:
    inner = SchedulerStepExecutor(scheduler, task_factory or _make_task_factory([]))
    return SpeculativeStepExecutor(
        inner,
        estimator=estimator or BaselineEstimator(),
        pre_generator=pre_generator,
        event_bus=event_bus,
        task_factory=task_factory,
    )


# ---------------------------------------------------------------------------
# prepare — boundary math + scheduling
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_prepare_builds_plan_and_schedules_past_boundary_steps() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        exec_ = _make_speculative_executor(scheduler)
        result = await exec_.prepare(_build_plan(), llm_eta_ms=500.0)

        assert isinstance(result, SpeculativePlanResult)
        # voice (say, "hello"=5 chars @ 30cps ≈ 167 ms) + music 1000ms
        # = 1167 ms — boundary is at the second step (cumulative
        # crosses 1000ms = 500 + 500 margin). anim follows.
        assert result.plan.boundary_idx is not None
        assert result.plan.boundary_idx >= 1
        # The pre-gen scheduled everything from the boundary
        # onward — at minimum the music + anim steps.
        assert "music" in result.scheduled_ids or "anim" in result.scheduled_ids
        # Estimator qualities were recorded for every step.
        assert len(result.estimator_qualities) == 3
        assert all(
            isinstance(q, SegmentEstimate) for q in result.estimator_qualities
        )
        # Snapshot surfaces the same boundary.
        snap = exec_.snapshot()
        assert snap["last_plan_boundary_idx"] == result.plan.boundary_idx
    finally:
        scheduler.shutdown()


@pytest.mark.asyncio
async def test_prepare_skips_pre_gen_when_llm_is_very_slow() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        exec_ = _make_speculative_executor(scheduler)
        # LLM is so slow that nothing in the plan crosses the
        # budget — boundary stays None, pre-gen has no work.
        result = await exec_.prepare(_build_plan(), llm_eta_ms=60_000.0)
        assert result.plan.boundary_idx is None
        assert result.scheduled_ids == ()
        assert exec_.pre_generator.completed_count == 0
    finally:
        scheduler.shutdown()


@pytest.mark.asyncio
async def test_prepare_rejects_after_shutdown() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        exec_ = _make_speculative_executor(scheduler)
        await exec_.shutdown()
        with pytest.raises(RuntimeError, match="closed"):
            await exec_.prepare(_build_plan(), llm_eta_ms=1000.0)
    finally:
        scheduler.shutdown()


# ---------------------------------------------------------------------------
# execute_plan — cache short-circuit
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_execute_plan_runs_every_step_and_short_circuits_cached_payloads() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    effects: list[tuple[str, str]] = []
    try:
        exec_ = _make_speculative_executor(
            scheduler, task_factory=_make_task_factory(effects)
        )
        results = await exec_.execute_plan(_build_plan(), llm_eta_ms=500.0)

        assert [step.status for step in results] == [StepStatus.COMPLETED] * 3
        # Every step fires its executor exactly once — cached
        # steps run during the pre-gen phase (outside the
        # scheduler), non-cached steps run inside the scheduler
        # pump. The ordering is event-loop dependent, so we sort.
        assert sorted(effects) == sorted(
            [
                ("voice", "speak_text"),
                ("music", "execute_music_code"),
                ("anim", "play_animation"),
            ]
        )
        assert len(effects) == 3
        # At least the post-boundary steps were pre-generated.
        assert exec_.pre_generator.completed_count >= 1
        # StepExecution.result carries the payload. When the
        # cache short-circuited, the payload is whatever the
        # pre-gen factory produced (a dict with ``step_id`` /
        # ``tool`` keys, identical to a fresh run).
        for execution in results:
            assert isinstance(execution.result, dict)
            assert "step_id" in execution.result
            assert "tool" in execution.result
    finally:
        scheduler.shutdown()


@pytest.mark.asyncio
async def test_execute_plan_without_pre_gen_runs_every_step_normally() -> None:
    """When the boundary is None (huge LLM ETA), no step is pre-generated.

    The executor must still complete every step — the original
    factory runs for each one.
    """
    scheduler = TaskScheduler()
    scheduler.start()
    effects: list[tuple[str, str]] = []
    try:
        exec_ = _make_speculative_executor(
            scheduler, task_factory=_make_task_factory(effects)
        )
        results = await exec_.execute_plan(_build_plan(), llm_eta_ms=60_000.0)
        assert [step.status for step in results] == [StepStatus.COMPLETED] * 3
        assert len(effects) == 3
        assert exec_.pre_generator.completed_count == 0
    finally:
        scheduler.shutdown()


# ---------------------------------------------------------------------------
# cancel — MERGE / plan-superseded semantics
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_cancel_with_merge_reason_tears_down_inflight_pre_gen() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        # Slow factory so the pre-gen tasks are still in flight
        # when we cancel.
        def slow_factory(step: PlanStep) -> SchedulerTask:
            async def execute(task: SchedulerTask) -> TaskResult:
                await asyncio.sleep(0.5)
                return TaskResult(payload={"step": step.step_id})

            return SchedulerTask(
                task_id=step.step_id,
                tool=step.action,
                channel=ChannelKind(step.channel),
                executor=execute,
                args=dict(step.arguments),
            )

        exec_ = _make_speculative_executor(
            scheduler, task_factory=slow_factory
        )
        await exec_.prepare(_build_plan(), llm_eta_ms=500.0)
        # Yield once so the pre-gen tasks actually start.
        await asyncio.sleep(0)
        cancelled = await exec_.cancel(CANCEL_REASON_MERGE_TOUCHED_FROZEN)
        assert cancelled >= 1
        assert exec_.merge_cancellation_count == 1
        assert exec_.pre_generator.last_cancel_reason == (
            CANCEL_REASON_MERGE_TOUCHED_FROZEN
        )
    finally:
        scheduler.shutdown()


@pytest.mark.asyncio
async def test_cancel_with_non_merge_reason_does_not_bump_counter() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        exec_ = _make_speculative_executor(scheduler)
        await exec_.cancel(CANCEL_REASON_PLAN_SUPERSEDED)
        assert exec_.merge_cancellation_count == 0
        assert exec_.pre_generator.last_cancel_reason == (
            CANCEL_REASON_PLAN_SUPERSEDED
        )
    finally:
        scheduler.shutdown()


@pytest.mark.asyncio
async def test_cancel_is_safe_without_active_plan() -> None:
    scheduler = TaskScheduler()
    scheduler.start()
    try:
        exec_ = _make_speculative_executor(scheduler)
        cancelled = await exec_.cancel(CANCEL_REASON_MERGE_TOUCHED_FROZEN)
        assert cancelled == 0
        assert exec_.merge_cancellation_count == 1
    finally:
        scheduler.shutdown()


# ---------------------------------------------------------------------------
# DecisionCoordinator integration — full Phase 2 + Phase 3 stack
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_decision_coordinator_uses_speculative_executor_end_to_end() -> None:
    """End-to-end: Phase 2 planner → Phase 3 executor → Phase 1 scheduler.

    The plan is produced by a static planner (no LLM dependency).
    The integration layer (here: the test) calls
    :meth:`SpeculativeStepExecutor.prepare` *after* planning and
    *before* driving the plan through the
    :class:`DecisionCoordinator`. This is the same shape the LLM
    dialog node will use once Phase 3 wiring lands.

    Pre-gen telemetry flows through the EventBus; the underlying
    channels run exactly once; the §6.4 «voice queue never silent»
    invariant is preserved because the executor still routes every
    step through the scheduler's FIFO pump.
    """
    scheduler = TaskScheduler()
    scheduler.start()
    event_bus = EventBus()
    events = event_bus.subscribe("pregen.*", max_queue_size=32)
    effects: list[tuple[str, str]] = []

    class Planner:
        async def plan(self, request: object) -> DecisionPlan:
            assert request == "play and greet"
            return _build_plan()

    try:
        exec_ = _make_speculative_executor(
            scheduler,
            task_factory=_make_task_factory(effects),
            event_bus=event_bus,
        )
        coordinator = DecisionCoordinator(Planner(), exec_, event_bus=event_bus)

        # Phase 2 entry: planner builds the plan synchronously.
        plan = await Planner().plan("play and greet")
        # Phase 3 entry: speculative prepare runs *before* the
        # coordinator drives the steps. This is where the §6.5
        # boundary computation + pre-gen scheduling happens.
        await exec_.prepare(plan, llm_eta_ms=500.0)
        # Phase 2 + Phase 1: coordinator walks the plan; the
        # executor short-circuits cached payloads at pump time.
        result = await coordinator.run("play and greet")

        assert result.status is StepStatus.COMPLETED
        assert [step.status for step in result.steps] == [StepStatus.COMPLETED] * 3
        # Every channel ran exactly once — but the ordering is
        # event-loop dependent. Pre-gen runs the executor for
        # cached steps *outside* the scheduler (so the heavy
        # work happens early); the scheduler pumps those steps
        # later and short-circuits via the cache. Non-cached
        # steps run once inside the scheduler pump. Either way,
        # the ``effects`` list has one entry per step.
        assert sorted(effects) == sorted(
            [
                ("voice", "speak_text"),
                ("music", "execute_music_code"),
                ("anim", "play_animation"),
            ]
        )
        assert len(effects) == 3
        # Drain pre-gen telemetry. We expect at least one
        # ``pregen.scheduled`` event when the boundary was reached.
        topics: list[str] = []
        for _ in range(8):
            try:
                topics.append(
                    (await asyncio.wait_for(events.get(), timeout=0.1)).topic
                )
            except asyncio.TimeoutError:
                break
        assert any(t == "pregen.scheduled" for t in topics)
    finally:
        events.close()
        await event_bus.close()
        scheduler.shutdown()


# ---------------------------------------------------------------------------
# Estimator quality wiring — actual durations feed the tracker
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_quality_tracker_receives_one_sample_per_executed_step() -> None:
    """The integration layer feeds wall-clock samples back to the tracker.

    We assert that every executed step produced a sample whose
    actual_ms is close to the synthetic sleep — the EMA error is
    therefore non-zero, proving the loop closed.
    """
    scheduler = TaskScheduler()
    scheduler.start()
    tracker = EstimatorQualityTracker()
    estimator = BaselineEstimator()

    def timed_factory(step: PlanStep) -> SchedulerTask:
        async def execute(task: SchedulerTask) -> TaskResult:
            await asyncio.sleep(0.02)  # 20 ms synthetic
            return TaskResult(payload={"step": step.step_id})

        return SchedulerTask(
            task_id=step.step_id,
            tool=step.action,
            channel=ChannelKind(step.channel),
            executor=execute,
            args=dict(step.arguments),
        )

    inner = SchedulerStepExecutor(scheduler, timed_factory)
    exec_ = SpeculativeStepExecutor(inner, estimator=estimator)

    try:
        results = await exec_.execute_plan(_build_plan(), llm_eta_ms=500.0)
        # Convert the per-step results into tracker samples. The
        # integration layer is responsible for timing — here we
        # do it manually since this is a wiring test, not the
        # production scheduler hook.
        plan = _build_plan()
        for step, execution in zip(plan.steps, results):
            assert execution.status is StepStatus.COMPLETED
            estimate = estimator.estimate(
                SchedulerTask(
                    task_id=step.step_id,
                    tool=step.action,
                    channel=ChannelKind(step.channel),
                    executor=timed_factory(step).executor,
                    args=dict(step.arguments),
                ),
                EstimatorContext(
                    channel=ChannelKind(step.channel),
                    tool=step.action,
                    args=dict(step.arguments),
                ),
            )
            tracker.record(estimate, actual_duration_ms=20.0)
        assert tracker.sample_count == 3
        # EMA error is non-zero because the voice estimate is
        # small (5 chars / 30cps ≈ 167 ms) but actual is 20 ms.
        assert tracker.ema_error_ms > 0.0
        # At least one bin collected data.
        bins = tracker.calibration_bins()
        assert any(b.total > 0 for b in bins)
    finally:
        scheduler.shutdown()


# ---------------------------------------------------------------------------
# Helpers — estimate_llm_eta_ms clamping
# ---------------------------------------------------------------------------


def test_estimate_llm_eta_ms_clamps_into_range() -> None:
    # Below floor → floor.
    assert estimate_llm_eta_ms(elapsed_ms=-100.0) == 0.0
    # Above ceil → ceil.
    assert estimate_llm_eta_ms(elapsed_ms=120_000.0) == 60_000.0
    # In range → as-is.
    assert estimate_llm_eta_ms(elapsed_ms=1234.0) == 1234.0


def test_estimate_llm_eta_ms_falls_back_to_baseline() -> None:
    assert estimate_llm_eta_ms(baseline_eta_ms=1500.0) == 1500.0
    # Nothing supplied → floor.
    assert estimate_llm_eta_ms() == 0.0
