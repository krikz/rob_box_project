"""Speculative pre-generation (Phase 3, SCHEDULER_DESIGN.md §6.5).

The §6.4 invariant is «voice queue never goes silent before the
LLM response lands». Without pre-generation the scheduler has to
*wait* for the LLM to react on every turn — even when the LLM is
known to be slow. With pre-generation, the scheduler spins up
the side-effect for a candidate segment **before** it reaches the
head of its channel. By the time the segment becomes ACTIVE, the
heavy work (TTS synthesis, music pattern load, …) is already
done — the audio device can render it without a wait.

Where the boundary lives
------------------------

§6.5 pins the boundary as the first segment whose cumulative
playback duration is **strictly greater** than the LLM's
expected ETA plus a safety margin. Everything to the left of
the boundary is *FROZEN* (the LLM will not have time to rewrite
it before it starts playing); everything to the right is *LIVE*
and a good MERGE candidate. The scheduler pre-generates exactly
the boundary index onward.

Cancellation
------------

A MERGE that touches a FROZEN segment must cancel the
in-flight pre-generation for that segment (G3 from audit #15).
:class:`SpeculativePreGenerator.cancel` walks the active tasks,
sets a sentinel event the user's coroutine is expected to poll,
and cancels the underlying :class:`asyncio.Task`. The
:attr:`cancel_reason` attribute distinguishes between an
explicit cancel (e.g. MERGE) and a normal shutdown.

The pre-gen layer never mutates :class:`SchedulerTask` directly.
The downstream :class:`TaskScheduler` is responsible for
consuming the cached result via :meth:`claim` — keeping the
boundary between «what to do» and «when to run it» explicit.
"""

from __future__ import annotations

import asyncio
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import (
    Any,
    Awaitable,
    Callable,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

from .estimator import (
    EstimatorContext,
    SegmentEstimate,
    SegmentEstimator,
    estimate_total_duration_ms,
)
from .event_bus import EventBus, EventEnvelope


_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Public data structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PreGenCandidate:
    """One segment scheduled for speculative pre-generation.

    Attributes:
        task_id: Stable id used by the :class:`TaskScheduler` to
            ``claim`` the cached result. Must be unique within a
            :class:`SpeculativePreGenerator` instance.
        estimate: The three-axis prediction the candidate was
            ranked by. ``duration_ms=None`` means the estimator
            could not reason about this segment — the pre-gen
            layer skips those automatically.
        metadata: Free-form payload forwarded to the factory so
            downstream tests can pass tool names, channel, etc.
            without subclassing.
    """

    task_id: str
    estimate: SegmentEstimate
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PreGenPlan:
    """Result of :meth:`SpeculativePreGenerator.build_plan`.

    Attributes:
        boundary_idx: Index (into ``candidates``) of the first
            segment to pre-generate. ``None`` when no segment is
            pre-generatable — either every candidate has a ``None``
            duration, or the LLM ETA is so pessimistic that the
            entire queue will play before the LLM responds.
        cumulative_ms: Cumulative duration (in ms) up to and
            including the segment that first crossed the budget
            threshold. Useful for diagnostics.
        skipped_ids: Candidate ids that were skipped (e.g. because
            the estimator could not predict their duration).
    """

    boundary_idx: Optional[int]
    cumulative_ms: float
    skipped_ids: tuple[str, ...]


# ---------------------------------------------------------------------------
# Pre-generation runtime
# ---------------------------------------------------------------------------


#: Signature of a pre-generation factory. It receives the candidate and
#: must return ``(task_id, payload)`` once the heavy work is done.
#: Raising is treated as a normal failure (the pre-gen task simply
#: ends, the result is dropped, and the scheduler will run the real
#: task at schedule time).
PreGenFactory = Callable[[PreGenCandidate], Awaitable[Tuple[str, Any]]]


class PreGenCancelledError(asyncio.CancelledError):
    """Raised inside a pre-gen factory when cancellation is requested.

    Subclasses :class:`asyncio.CancelledError` so coroutines that
    propagate cancellation unchanged behave the same as native
    cancellations. Downstream ``except asyncio.CancelledError``
    branches keep working.
    """

    def __init__(self, task_id: str, reason: str = "superseded") -> None:
        super().__init__(f"pre-generation cancelled for {task_id}: {reason}")
        self.task_id = task_id
        self.reason = reason


class SpeculativePreGenerator:
    """Plans and runs speculative pre-generation per §6.5.

    Parameters
    ----------
    estimator:
        :class:`SegmentEstimator` used to refresh the per-task
        estimates when :meth:`build_plan` is invoked with raw
        candidates. May be omitted if the caller has already
        computed :class:`SegmentEstimate` for every candidate.
    event_bus:
        Optional :class:`EventBus` that will receive
        ``pregen.scheduled``, ``pregen.completed`` and
        ``pregen.cancelled`` events. ``None`` disables telemetry.
    safety_margin_ms:
        Default safety margin in milliseconds when the caller does
        not pass one explicitly. Mirrors the §6.5 default of
        ``500 ms``.
    """

    DEFAULT_SAFETY_MARGIN_MS: float = 500.0

    def __init__(
        self,
        *,
        estimator: SegmentEstimator | None = None,
        event_bus: EventBus | None = None,
        safety_margin_ms: float = DEFAULT_SAFETY_MARGIN_MS,
    ) -> None:
        if safety_margin_ms < 0:
            raise ValueError("safety_margin_ms must be non-negative")
        self._estimator = estimator
        self._event_bus = event_bus
        self._safety_margin_ms = safety_margin_ms
        # task_id → asyncio.Task. Bounded only by the number of
        # in-flight pre-gen tasks; callers should cancel / drain
        # explicitly.
        self._active: dict[str, asyncio.Task[None]] = {}
        # task_id → cached payload produced by the factory.
        # Distinct from the live ``_active`` dict: a completed
        # pre-gen stays here until the scheduler ``claim``s it.
        self._results: dict[str, Any] = {}
        # Event set when cancel() has been invoked — factories
        # that want to support graceful stop can ``await`` it
        # between expensive steps.
        self.cancel_event: asyncio.Event = asyncio.Event()
        # Diagnostic counters — reset on each ``reset()``.
        self.completed_count: int = 0
        self.cancelled_count: int = 0
        self.failed_count: int = 0
        self.last_cancel_reason: Optional[str] = None
        self._closed: bool = False

    # ----- plan construction -------------------------------------------

    def build_plan(
        self,
        candidates: Sequence[PreGenCandidate],
        *,
        llm_eta_ms: float,
        safety_margin_ms: Optional[float] = None,
    ) -> PreGenPlan:
        """Compute the §6.5 boundary without launching anything.

        Pure CPU work — safe to call from synchronous code paths
        and from inside coroutines. ``llm_eta_ms`` is in
        milliseconds and may be ``0`` (means the LLM is
        already-answered, so nothing should be pre-generated).
        Negative ``llm_eta_ms`` is rejected because it would make
        the boundary trivially ``0`` and could mask upstream bugs
        in the LLM ETA feed.
        """
        if llm_eta_ms < 0:
            raise ValueError("llm_eta_ms must be non-negative")
        if not candidates:
            return PreGenPlan(boundary_idx=None, cumulative_ms=0.0, skipped_ids=())
        margin = (
            self._safety_margin_ms
            if safety_margin_ms is None
            else safety_margin_ms
        )
        if margin < 0:
            raise ValueError("safety_margin_ms must be non-negative")

        budget = llm_eta_ms + margin
        boundary_idx: Optional[int] = None
        cumulative_to_boundary: float = 0.0
        total_ms: float = 0.0
        skipped_ids: list[str] = []
        for index, candidate in enumerate(candidates):
            if candidate.estimate.duration_ms is None:
                skipped_ids.append(candidate.task_id)
                continue
            total_ms += candidate.estimate.duration_ms
            if boundary_idx is None:
                cumulative_to_boundary += candidate.estimate.duration_ms
                if cumulative_to_boundary >= budget:
                    boundary_idx = index
        return PreGenPlan(
            boundary_idx=boundary_idx,
            cumulative_ms=cumulative_to_boundary
            if boundary_idx is not None
            else total_ms,
            skipped_ids=tuple(skipped_ids),
        )

    # ----- execution -----------------------------------------------------

    async def run(
        self,
        plan: PreGenPlan,
        candidates: Sequence[PreGenCandidate],
        factory: PreGenFactory,
    ) -> tuple[str, ...]:
        """Execute the pre-generation for every candidate past the boundary.

        Returns the tuple of task_ids that were *scheduled* (not
        necessarily completed — the caller is expected to await
        :meth:`wait_all` before draining). Already-completed tasks
        are not re-scheduled; their cached payloads stay put and
        are returned via :meth:`claim`.

        Raises:
            RuntimeError: If the generator has been closed via
                :meth:`shutdown`.
        """
        if self._closed:
            raise RuntimeError("pre-generator is closed")
        if plan.boundary_idx is None:
            return ()
        scheduled: list[str] = []
        for index, candidate in enumerate(candidates):
            if index < plan.boundary_idx:
                continue
            if not isinstance(candidate, PreGenCandidate):
                raise TypeError(
                    f"candidates[{index}] must be PreGenCandidate, "
                    f"got {type(candidate).__name__}"
                )
            if candidate.task_id in self._results or candidate.task_id in self._active:
                # Already pre-generated; skip to keep the contract
                # that ``claim`` returns the cached payload only
                # once.
                continue
            coro = self._run_one(candidate, factory)
            task = asyncio.create_task(
                coro, name=f"pre-gen-{candidate.task_id}"
            )
            self._active[candidate.task_id] = task
            scheduled.append(candidate.task_id)
            await self._publish(
                "pregen.scheduled",
                {
                    "task_id": candidate.task_id,
                    "estimated_duration_ms": candidate.estimate.duration_ms,
                    "boundary_idx": plan.boundary_idx,
                    "metadata": dict(candidate.metadata),
                },
            )
        return tuple(scheduled)

    async def _run_one(
        self,
        candidate: PreGenCandidate,
        factory: PreGenFactory,
    ) -> None:
        task_id = candidate.task_id
        try:
            _, payload = await factory(candidate)
        except asyncio.CancelledError as exc:
            # Distinguish our explicit cancellation from native
            # asyncio cancellation (e.g. the event loop shutting
            # down) so the counter stays accurate.
            self.cancelled_count += 1
            self._results.pop(task_id, None)
            self._active.pop(task_id, None)
            await self._publish(
                "pregen.cancelled",
                {
                    "task_id": task_id,
                    "reason": getattr(exc, "reason", "superseded"),
                },
            )
            _LOG.info("pre-gen cancelled task=%s reason=%s", task_id, exc)
            raise
        except Exception as exc:  # noqa: BLE001 — boundary isolation
            self.failed_count += 1
            self._results.pop(task_id, None)
            self._active.pop(task_id, None)
            await self._publish(
                "pregen.failed",
                {
                    "task_id": task_id,
                    "error": f"{type(exc).__name__}: {exc}",
                },
            )
            _LOG.warning(
                "pre-gen failed task=%s err=%s", task_id, exc, exc_info=True
            )
            return
        else:
            self._results[task_id] = payload
            self.completed_count += 1
            self._active.pop(task_id, None)
            await self._publish(
                "pregen.completed",
                {
                    "task_id": task_id,
                    "estimated_duration_ms": candidate.estimate.duration_ms,
                },
            )

    # ----- cancellation / drain ----------------------------------------

    async def cancel(self, reason: str = "superseded") -> int:
        """Cancel every in-flight pre-gen task.

        Sets :attr:`cancel_event` so factories that want to stop
        cooperatively can ``await`` it. Returns the number of
        tasks that were cancelled. Cached payloads from
        already-completed pre-gens are kept — the caller can
        still :meth:`claim` them if useful. ``last_cancel_reason``
        is updated even when no tasks were active, so callers
        can rely on it as a single source of truth for «why did
        we stop».
        """
        self.last_cancel_reason = reason
        self.cancel_event.set()
        if not self._active:
            self.cancel_event = asyncio.Event()
            return 0
        tasks = list(self._active.values())
        for task in tasks:
            task.cancel(PreGenCancelledError(task.get_name(), reason))
        # Drain so the cancelled_count is consistent before we
        # return. ``return_exceptions=True`` keeps the gather alive
        # even if some tasks re-raise.
        await asyncio.gather(*tasks, return_exceptions=True)
        self._active.clear()
        # Reset the event so subsequent pre-gen rounds start clean.
        self.cancel_event = asyncio.Event()
        return len(tasks)

    async def wait_all(self, timeout: Optional[float] = None) -> None:
        """Await every active pre-gen task until completion or *timeout*.

        ``timeout`` is in seconds; ``None`` waits forever.
        Cancellation via :meth:`cancel` is independent.
        """
        if not self._active:
            return
        tasks = list(self._active.values())
        if timeout is None:
            await asyncio.gather(*tasks, return_exceptions=True)
        else:
            await asyncio.wait_for(
                asyncio.gather(*tasks, return_exceptions=True),
                timeout=timeout,
            )

    async def shutdown(self) -> None:
        """Cancel everything and stop accepting new plans."""
        if self._closed:
            return
        await self.cancel(reason="shutdown")
        self._closed = True

    def reset(self) -> None:
        """Forget cached payloads and diagnostic counters.

        Does NOT cancel in-flight tasks — callers must invoke
        :meth:`cancel` first if they want a hard stop. Useful for
        tests that re-use a single pre-generator across multiple
        synthetic rounds.
        """
        self._results.clear()
        self.completed_count = 0
        self.cancelled_count = 0
        self.failed_count = 0
        self.last_cancel_reason = None

    # ----- claim / inspect ---------------------------------------------

    def claim(self, task_id: str) -> Any:
        """Return the cached pre-generated payload for *task_id*.

        The payload is removed from the cache on claim — claim is
        an *ownership transfer*, mirroring how the LLM feedback
        loop hands the cached TTS bytes to the audio device.
        Raises :class:`KeyError` when nothing was pre-generated
        for this task id.
        """
        return self._results.pop(task_id)

    def has_payload(self, task_id: str) -> bool:
        return task_id in self._results

    def active_ids(self) -> tuple[str, ...]:
        return tuple(self._active.keys())

    def snapshot(self) -> dict[str, Any]:
        return {
            "completed": self.completed_count,
            "cancelled": self.cancelled_count,
            "failed": self.failed_count,
            "active": len(self._active),
            "cached": len(self._results),
            "last_cancel_reason": self.last_cancel_reason,
        }

    # ----- helpers ------------------------------------------------------

    async def _publish(self, topic: str, payload: dict[str, Any]) -> None:
        if self._event_bus is None:
            return
        try:
            await self._event_bus.publish(
                EventEnvelope(topic, payload, event_id=uuid.uuid4().hex)
            )
        except Exception as exc:  # noqa: BLE001 — telemetry must never break callers
            _LOG.warning("pre-gen telemetry publish failed topic=%s err=%s", topic, exc)


__all__ = [
    "PreGenCandidate",
    "PreGenFactory",
    "PreGenPlan",
    "PreGenCancelledError",
    "SpeculativePreGenerator",
]
