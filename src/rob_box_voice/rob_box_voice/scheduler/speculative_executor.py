"""Speculative-aware step executor (Phase 3, SCHEDULER_DESIGN.md §6.5).

Bridges :class:`SpeculativePreGenerator` with the Phase 2 two-tier
planner so that, when the LLM ETA is non-trivial, the heavy work
for segments past the freeze boundary starts *before* they reach
the head of their channel. The downstream :class:`TaskScheduler`
still owns FIFO ordering — pre-gen is purely an eager prep step.

Why this lives in its own class
-------------------------------

Three contracts have to align cleanly:

* **Phase 2** — ``DecisionCoordinator`` expects a
  :class:`LowLevelExecutor` (``execute(step) -> StepExecution``).
* **Phase 1** — :class:`SchedulerStepExecutor` already implements
  that contract by adapting ``PlanStep`` → ``SchedulerTask`` and
  submitting via :class:`TaskScheduler`. We do NOT want to fork
  that path — too easy to drift the FIFO/cancel invariants.
* **Phase 3** — :class:`SpeculativePreGenerator` owns the
  ``build_plan`` / ``run`` / ``cancel`` lifecycle for pre-gen
  tasks. It must NOT touch the scheduler directly (§6.5 boundary
  rule from this module's docstring).

This module stitches them together:

1. The integration layer calls :meth:`SpeculativeStepExecutor.prepare`
   once per :class:`DecisionPlan`, with the latest LLM ETA. That
   returns a list of :class:`PreGenCandidate` and kicks off the
   asyncio pre-gen tasks via :meth:`SpeculativePreGenerator.run`.
2. Each :meth:`execute` call still routes through the inner
   :class:`SchedulerStepExecutor` (so cancellation, ordering, and
   status reporting stay identical). When the inner step factory
   is asked to build a :class:`SchedulerTask`, the wrapped
   factory first checks whether the pre-gen cache already holds a
   payload for ``step.step_id`` — if so, the executor coroutine
   short-circuits to that payload instead of recomputing.
3. :meth:`cancel` propagates into the pre-gen layer so a MERGE
   that touches a FROZEN segment (audit #15 / G3) tears down the
   in-flight heavy work, mirroring :class:`SpeculativePreGenerator`'s
   own cancel contract.

The cached-payload short-circuit is deliberately conservative:
the *channel routing* still goes through
:meth:`TaskScheduler.submit`, so FIFO ordering is preserved, and
the §6.4 «voice queue never silent before LLM lands» invariant
holds even when the cache is empty. When the cache *is* full, the
executor still touches the scheduler, so :class:`ChannelStatus`
keeps reporting accurate state (queue depth, current task).
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional, Sequence

from .decision import (
    DecisionPlan,
    LowLevelExecutor,
    PlanStep,
    SchedulerStepExecutor,
    StepExecution,
    StepStatus,
    TaskFactory,
)
from .estimator import (
    EstimatorContext,
    SegmentEstimate,
    SegmentEstimator,
)
from .event_bus import EventBus, EventEnvelope
from .pre_gen import (
    PreGenCandidate,
    PreGenFactory,
    PreGenPlan,
    SpeculativePreGenerator,
)
from .task_scheduler import SchedulerTask, TaskResult, TaskStatus

_LOG = logging.getLogger(__name__)


#: Reason string used when the integration layer cancels pre-gen
#: because of a MERGE that touched a FROZEN segment. Mirrors the
#: §6.5 / audit #15 / G3 wording so downstream log readers can
#: recognise it without parsing free text.
CANCEL_REASON_MERGE_TOUCHED_FROZEN = "merge_touched_frozen"


#: Reason string used when the integration layer cancels pre-gen
#: because the LLM produced a brand-new plan and the previous
#: pre-gen is no longer useful. Distinct from the MERGE reason
#: so log analysis can tell the two apart.
CANCEL_REASON_PLAN_SUPERSEDED = "plan_superseded"


@dataclass(frozen=True)
class SpeculativePlanResult:
    """Outcome of :meth:`SpeculativeStepExecutor.prepare`.

    Attributes:
        plan: The :class:`PreGenPlan` returned by
            :meth:`SpeculativePreGenerator.build_plan` (so callers
            can inspect ``boundary_idx`` / ``skipped_ids`` without
            holding a reference to the generator).
        scheduled_ids: Tuple of task ids actually scheduled into
            the pre-gen runtime. May be empty when ``boundary_idx``
            is ``None`` (e.g. the LLM ETA is huge).
        estimator_qualities: Tuple of :class:`SegmentEstimate` per
            :class:`PlanStep` — useful for the §7.1 feedback block
            so the LLM can see the prediction, not just the
            pre-gen decision.
    """

    plan: PreGenPlan
    scheduled_ids: tuple[str, ...]
    estimator_qualities: tuple[SegmentEstimate, ...]


class SpeculativeStepExecutor(LowLevelExecutor):
    """Phase 2 + Phase 3 bridge executor.

    Wraps a :class:`SchedulerStepExecutor` so the underlying
    scheduler path stays untouched, then layers speculative
    pre-generation on top:

    * :meth:`prepare` — call once per :class:`DecisionPlan` to
      build a :class:`PreGenPlan`, kick off heavy work for past-
      boundary steps, and return a :class:`SpeculativePlanResult`.
    * :meth:`execute` — same signature as
      :class:`SchedulerStepExecutor.execute`. When the inner step
      factory is asked for a :class:`SchedulerTask` whose
      ``task_id`` matches a cached pre-gen payload, the wrapper
      builds an executor coroutine that returns the cached result
      instead of recomputing.
    * :meth:`cancel` — propagates into
      :class:`SpeculativePreGenerator.cancel`. Idempotent.

    Parameters
    ----------
    inner:
        The Phase 2 :class:`SchedulerStepExecutor` that owns the
        scheduler submission path. ``SpeculativeStepExecutor`` does
        NOT mutate it — every execution still flows through
        ``inner.execute`` so FIFO / cancel semantics are preserved.
    estimator:
        :class:`SegmentEstimator` used to predict per-step
        duration / cost / confidence. Required — pre-generation is
        meaningless without a prediction.
    pre_generator:
        Optional pre-built :class:`SpeculativePreGenerator`. When
        omitted, one is constructed with ``event_bus`` and
        ``safety_margin_ms``. Tests usually pass their own so they
        can inspect counters / cached payloads.
    event_bus:
        Optional :class:`EventBus` for telemetry (``pregen.*``).
        Forwarded to the auto-built pre-generator when ``pre_generator``
        is not supplied.
    safety_margin_ms:
        Safety margin forwarded to the auto-built pre-generator.
    task_factory:
        Optional override of the inner step executor's
        ``task_factory``. When supplied, the wrapper uses *this*
        factory instead of ``inner._task_factory`` so the wrapper
        can intercept the cache lookup. When ``None``, the wrapper
        derives the cached-aware factory from ``inner``.
    """

    def __init__(
        self,
        inner: SchedulerStepExecutor,
        *,
        estimator: SegmentEstimator,
        pre_generator: Optional[SpeculativePreGenerator] = None,
        event_bus: Optional[EventBus] = None,
        safety_margin_ms: float = SpeculativePreGenerator.DEFAULT_SAFETY_MARGIN_MS,
        task_factory: Optional[TaskFactory] = None,
    ) -> None:
        if not isinstance(estimator, SegmentEstimator):
            raise TypeError(
                "estimator must satisfy SegmentEstimator protocol, "
                f"got {type(estimator).__name__}"
            )
        self._inner = inner
        self._estimator = estimator
        if pre_generator is None:
            pre_generator = SpeculativePreGenerator(
                estimator=estimator,
                event_bus=event_bus,
                safety_margin_ms=safety_margin_ms,
            )
        self._pre_generator = pre_generator
        # We always wrap the inner factory with the cache-aware
        # version. When the caller supplied their own, we wrap
        # *that* instead. The cache-aware factory is responsible
        # for short-circuiting cached payloads at executor time.
        self._base_task_factory: TaskFactory = (
            task_factory if task_factory is not None else inner._task_factory
        )
        # Build the cache-aware factory once. ``inner.execute``
        # will use *this* factory instead of its own, so the
        # short-circuit actually fires at pump time. We replace
        # ``inner._task_factory`` deliberately — the wrapper owns
        # the bridge seam, and the inner executor's other state
        # (scheduler reference, etc.) is untouched.
        self._cached_task_factory: TaskFactory = self._build_cached_task_factory()
        # The inner executor's ``_task_factory`` is private but
        # the rest of the Phase 2 contract is public; replacing
        # it is the only way to inject the cache-aware path
        # without forking the Phase 2 execute logic.
        inner._task_factory = self._cached_task_factory
        # Maps PlanStep.step_id → SegmentEstimate captured at prepare
        # time. Used by the integration tests so they can assert
        # against the predictions the executor actually used.
        self._estimates_by_step: dict[str, SegmentEstimate] = {}
        self._last_plan: Optional[PreGenPlan] = None
        self._closed: bool = False
        # Diagnostic counters that mirror SpeculativePreGenerator's
        # but are owned by the executor so callers do not have to
        # reach into the pre-generator for the bridge-layer view.
        self.merge_cancellation_count: int = 0

    # ----- accessors ----------------------------------------------------

    @property
    def pre_generator(self) -> SpeculativePreGenerator:
        return self._pre_generator

    @property
    def inner(self) -> SchedulerStepExecutor:
        return self._inner

    @property
    def last_plan(self) -> Optional[PreGenPlan]:
        return self._last_plan

    # ----- prepare (Phase 3 entry point) -------------------------------

    async def prepare(
        self,
        plan: DecisionPlan,
        *,
        llm_eta_ms: float,
        safety_margin_ms: Optional[float] = None,
        extra_factory: Optional[PreGenFactory] = None,
    ) -> SpeculativePlanResult:
        """Build and run a pre-gen plan for *plan*.

        Computes per-step estimates via :attr:`estimator`, runs
        :meth:`SpeculativePreGenerator.build_plan`, and starts the
        asyncio pre-gen tasks. Cached payloads become available
        immediately for any subsequent :meth:`execute` call.

        ``extra_factory`` lets advanced callers wrap the default
        factory with their own logic (e.g. cooperative cancel
        hooks). When supplied, the wrapper invokes the *extra*
        factory instead of the default one — it MUST still return
        ``(task_id, payload)`` tuples.
        """
        if self._closed:
            raise RuntimeError("speculative executor is closed")
        # Reset per-plan diagnostics so the next plan starts clean.
        self._estimates_by_step.clear()

        # Build candidates by running the estimator over each step
        # converted into a synthetic SchedulerTask. The factory is
        # what ``SchedulerStepExecutor`` would have used — we keep
        # the contract identical so the estimator sees the same
        # channel / tool / args it would have seen at execute time.
        candidates: list[PreGenCandidate] = []
        estimates: list[SegmentEstimate] = []
        for step in plan.steps:
            task = self._build_synthetic_task(step)
            estimate = self._estimator.estimate(
                task,
                EstimatorContext(
                    channel=task.channel,
                    tool=task.tool,
                    args=task.args,
                ),
            )
            estimates.append(estimate)
            self._estimates_by_step[step.step_id] = estimate
            candidates.append(
                PreGenCandidate(
                    task_id=step.step_id,
                    estimate=estimate,
                    metadata={
                        "plan_id": plan.plan_id,
                        "action": step.action,
                        "channel": step.channel,
                    },
                )
            )

        pre_plan = self._pre_generator.build_plan(
            candidates,
            llm_eta_ms=llm_eta_ms,
            safety_margin_ms=safety_margin_ms,
        )
        self._last_plan = pre_plan

        factory: PreGenFactory
        if extra_factory is None:
            factory = self._default_pre_factory()
        else:
            factory = self._compose_factory(extra_factory)

        scheduled_ids = await self._pre_generator.run(pre_plan, candidates, factory)
        return SpeculativePlanResult(
            plan=pre_plan,
            scheduled_ids=tuple(scheduled_ids),
            estimator_qualities=tuple(estimates),
        )

    # ----- execute (Phase 2 contract) ----------------------------------

    async def execute(self, step: PlanStep) -> StepExecution:
        """Run a single plan step, short-circuiting when pre-gen cached it.

        The path is identical to ``SchedulerStepExecutor.execute``
        except for one seam: we wrap the inner task factory with a
        cache-aware closure so that, when the scheduler calls the
        executor coroutine, the cached payload (if any) is
        consumed via ``claim`` instead of recomputed.
        """
        return await self._inner.execute(step)

    async def execute_plan(
        self,
        plan: DecisionPlan,
        *,
        llm_eta_ms: float,
        safety_margin_ms: Optional[float] = None,
    ) -> list[StepExecution]:
        """Convenience: prepare + execute every step in order.

        Returns the per-step :class:`StepExecution` list in the
        same order as ``plan.steps``. Useful when the integration
        test wants to assert per-step status without going through
        ``DecisionCoordinator``.
        """
        await self.prepare(
            plan,
            llm_eta_ms=llm_eta_ms,
            safety_margin_ms=safety_margin_ms,
        )
        results: list[StepExecution] = []
        for step in plan.steps:
            results.append(await self.execute(step))
        return results

    # ----- cancel (Phase 3 MERGE seam) ---------------------------------

    async def cancel(
        self,
        reason: str = CANCEL_REASON_MERGE_TOUCHED_FROZEN,
    ) -> int:
        """Cancel any in-flight pre-gen tasks with the given *reason*.

        Returns the number of pre-gen tasks that were cancelled.
        Calling this without an active plan is safe — it forwards
        to :meth:`SpeculativePreGenerator.cancel` which already
        handles the no-active case.

        ``merge_cancellation_count`` tracks *only* the calls made
        with :data:`CANCEL_REASON_MERGE_TOUCHED_FROZEN` so the §6.5
        acceptance criterion («MERGE that touches FROZEN cancels
        pre-gen») is observable from a single counter.
        """
        if reason == CANCEL_REASON_MERGE_TOUCHED_FROZEN:
            self.merge_cancellation_count += 1
        return await self._pre_generator.cancel(reason=reason)

    async def shutdown(self) -> None:
        """Cancel everything and stop accepting new plans."""
        if self._closed:
            return
        await self._pre_generator.shutdown()
        self._closed = True

    # ----- diagnostics --------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        """Return a unified snapshot of pre-gen + executor counters."""
        snap = self._pre_generator.snapshot()
        snap["merge_cancellations"] = self.merge_cancellation_count
        snap["last_plan_boundary_idx"] = (
            self._last_plan.boundary_idx if self._last_plan is not None else None
        )
        snap["last_plan_cumulative_ms"] = (
            self._last_plan.cumulative_ms if self._last_plan is not None else None
        )
        snap["last_plan_skipped_ids"] = (
            list(self._last_plan.skipped_ids)
            if self._last_plan is not None
            else []
        )
        return snap

    # ----- internals ----------------------------------------------------

    def _build_synthetic_task(self, step: PlanStep) -> SchedulerTask:
        """Build a synthetic :class:`SchedulerTask` for estimation.

        The synthetic task has a no-op executor — the estimator only
        inspects ``channel`` / ``tool`` / ``args``. We still wrap
        ``step.arguments`` defensively so callers that pass a
        non-mapping get a clear ``TypeError`` instead of a stack
        trace buried inside the estimator.
        """
        try:
            channel = self._coerce_channel(step)
        except ValueError:
            # Step referenced a non-MVP channel (e.g. ``nav``) — the
            # estimator will report ``unknown_channel`` and the
            # pre-gen layer will skip it. Do NOT raise: the caller
            # might still want the plan to be built.
            channel = step.channel  # type: ignore[assignment]
        return SchedulerTask(
            task_id=step.step_id,
            tool=step.action,
            channel=channel,
            executor=self._noop_executor,
            args=dict(step.arguments),
        )

    @staticmethod
    async def _noop_executor(task: SchedulerTask) -> TaskResult:
        """Synthetic executor for estimator-only paths.

        Should never be invoked — the wrapper short-circuits cached
        steps before the scheduler pumps them. Defensive ``return``
        keeps the protocol honest if a misconfiguration slips
        through (we still want the scheduler to log a clean
        ``COMPLETED`` rather than crash).
        """
        return TaskResult(payload={"synthetic": task.task_id})

    def _coerce_channel(self, step: PlanStep):
        """Convert ``step.channel`` (a string) to :class:`ChannelKind`.

        Defers the import to avoid a circular dependency at module
        load time (``task_scheduler`` imports nothing from
        ``decision``; ``decision`` imports from ``task_scheduler``).
        """
        from .task_scheduler import ChannelKind

        return ChannelKind(step.channel)

    def _default_pre_factory(self) -> PreGenFactory:
        """Default pre-gen factory: run the step's executor eagerly.

        Each pre-gen task builds the corresponding
        :class:`SchedulerTask` via the inner task factory, runs
        the executor coroutine *outside* the scheduler pump
        (the scheduler does not know about pre-gen; doing this on
        the scheduler would defeat the purpose), and returns the
        payload. The scheduler will re-run the same executor at
        schedule time UNLESS the wrapper short-circuits via
        ``claim``.
        """

        async def factory(candidate: PreGenCandidate) -> tuple[str, Any]:
            step = self._candidate_to_step(candidate)
            task = self._base_task_factory(step)
            if hasattr(task, "__await__"):
                task = await task  # type: ignore[misc, assignment]
            if not isinstance(task, SchedulerTask):
                raise TypeError("task_factory must return SchedulerTask")
            # Run the executor outside the scheduler pump. If the
            # executor raises, the pre-gen layer's failure isolation
            # converts that into a ``failed`` event and removes the
            # task from the cache (per SpeculativePreGenerator).
            result = await task.executor(task)
            payload: Any
            if result is None:
                payload = None
            else:
                payload = result.payload
            return candidate.task_id, payload

        return factory

    def _compose_factory(self, extra: PreGenFactory) -> PreGenFactory:
        """Combine the default factory with a caller-supplied wrapper.

        The wrapper sees the (task_id, payload) tuple the default
        factory produced and can transform / log / veto it. Raising
        inside ``extra`` is treated as a normal pre-gen failure by
        the underlying :class:`SpeculativePreGenerator`.
        """

        async def composed(candidate: PreGenCandidate) -> tuple[str, Any]:
            inner = self._default_pre_factory()
            task_id, payload = await inner(candidate)
            return await extra(
                PreGenCandidate(
                    task_id=candidate.task_id,
                    estimate=candidate.estimate,
                    metadata=dict(candidate.metadata),
                )
            )

        return composed

    def _candidate_to_step(self, candidate: PreGenCandidate) -> PlanStep:
        """Rebuild a :class:`PlanStep` from the candidate's metadata.

        The pre-gen layer is metadata-only; we keep the original
        ``PlanStep`` shape so the inner factory's contract is
        unchanged. The factory remains the source of truth for
        ``executor`` and channel routing.
        """
        from .decision import PlanStep

        meta = candidate.metadata
        return PlanStep(
            step_id=candidate.task_id,
            action=str(meta.get("action", "")),
            arguments={},
            channel=str(meta.get("channel", "voice")),
            depends_on=(),
        )

    # ----- cache-aware task factory ------------------------------------

    def _build_cached_task_factory(self) -> TaskFactory:
        """Build the cache-aware factory used by ``inner.execute``.

        The factory first asks the base factory for a
        :class:`SchedulerTask` (so channel / tool / executor wiring
        stays intact), then wraps ``task.executor`` so that, when
        the scheduler pumps the task, the cached payload is
        consumed via ``claim`` if available. When the cache is
        empty, the original executor runs as before.
        """
        base = self._base_task_factory

        def factory(step: PlanStep) -> SchedulerTask:
            task = base(step)
            if hasattr(task, "__await__"):
                # Async factories are out of scope for the cached
                # path — fall back to whatever they produced.
                # Tests should pass sync factories.
                raise TypeError(
                    "cached_task_factory does not support async task factories; "
                    "wrap the async factory in a sync one before passing it in"
                )
            if not isinstance(task, SchedulerTask):
                raise TypeError("task_factory must return SchedulerTask")
            return self._wrap_task_with_cache(task, step)

        return factory

    def cached_task_factory(self) -> TaskFactory:
        """Public alias for :meth:`_build_cached_task_factory`.

        Kept as a thin public accessor so external integration code
        (e.g. tests that bypass this wrapper) can build the same
        factory closure.
        """
        return self._cached_task_factory

    def _wrap_task_with_cache(
        self, task: SchedulerTask, step: PlanStep
    ) -> SchedulerTask:
        """Wrap *task*'s executor so a cached payload is claimed on pump.

        We deliberately do NOT touch ``task.channel`` /
        ``task.tool`` / ``task.args`` — the scheduler's channel
        routing is the only ordering mechanism, and we want to
        preserve it. The executor is replaced with a coroutine
        that checks the cache first and falls back to the
        original executor on a miss.
        """
        original_executor = task.executor
        step_id = step.step_id
        task_id = task.task_id

        async def cached_executor(t: SchedulerTask) -> TaskResult:
            # Bind on the actual task (not the synthetic copy) so
            # ``t.task_id`` matches the value keyed in the cache.
            # ``step_id`` and ``task_id`` are usually equal; we
            # prefer ``task_id`` for the lookup to keep the
            # contract identical to ``SpeculativePreGenerator.run``.
            key = t.task_id or task_id or step_id
            if self._pre_generator.has_payload(key):
                payload = self._pre_generator.claim(key)
                return TaskResult(payload=payload, message="pre-generated")
            # Cache miss — fall back to the original executor.
            t.executor = original_executor  # avoid recursion in error path
            return await original_executor(t)

        # Rebuild the task with the wrapped executor. We must use
        # ``dataclasses.replace`` to keep the dataclass frozen-ish
        # invariants; ``SchedulerTask`` is a regular dataclass so
        # attribute assignment works, but we still prefer to
        # construct a fresh copy to keep the wrapper easy to audit.
        from dataclasses import replace

        return replace(task, executor=cached_executor)


# ---------------------------------------------------------------------------
# Helpers — ETA derivation for the integration layer
# ---------------------------------------------------------------------------


def estimate_llm_eta_ms(
    *,
    elapsed_ms: Optional[float] = None,
    baseline_eta_ms: Optional[float] = None,
    floor_ms: float = 0.0,
    ceil_ms: float = 60_000.0,
) -> float:
    """Return a sane LLM ETA for :meth:`SpeculativeStepExecutor.prepare`.

    The integration tests do not have access to the live
    :class:`LLMEstimator` (Phase 4 wiring). This helper makes the
    tests deterministic by clamping into ``[floor_ms, ceil_ms]``
    and falling back to ``baseline_eta_ms`` when ``elapsed_ms`` is
    not provided.
    """
    raw: float
    if elapsed_ms is not None:
        raw = elapsed_ms
    elif baseline_eta_ms is not None:
        raw = baseline_eta_ms
    else:
        raw = floor_ms
    if raw < floor_ms:
        return floor_ms
    if raw > ceil_ms:
        return ceil_ms
    return raw


__all__ = [
    "CANCEL_REASON_MERGE_TOUCHED_FROZEN",
    "CANCEL_REASON_PLAN_SUPERSEDED",
    "SpeculativePlanResult",
    "SpeculativeStepExecutor",
    "estimate_llm_eta_ms",
]
