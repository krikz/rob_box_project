"""Pluggable segment estimators (Phase 3, issue #968 §6).

Estimators answer three independent questions about a single
:class:`~rob_box_voice.scheduler.task_scheduler.SchedulerTask` (or
the future ``Segment`` object that W7 LLM integration will use):

* **duration** — how long the side-effect takes (seconds; §6.2);
* **cost** — how heavy the side-effect is (abstract tokens; §11.3
  budget surfaces); and
* **confidence** — how sure the estimator is about its own numbers
  in ``[0.0, 1.0]`` (calibration hook for §6.5 / §7.1).

The estimators are **pure functions** of the task and a
:class:`EstimatorContext`. They never touch the scheduler, the event
bus, or the ROS layer — that keeps them trivially testable and lets
W7 swap in a learned estimator (LLM-based, EMA of historical
duration, …) without rewiring the scheduler.

Contracts (per ``SCHEDULER_DESIGN.md`` §6.2 / §6.3):

* ``estimate_ms`` / ``estimate_cost`` must return a
  **non-negative float**; ``None`` is allowed when the estimator
  cannot reason about the task (e.g. unknown tool).
* ``confidence`` must be a **float in [0.0, 1.0]**. ``1.0`` means
  the estimator is fully calibrated (e.g. we measured the exact
  duration of the previous identical task). ``0.0`` means the
  estimator is guessing — callers should fall back to safer
  defaults.
* All three fields are returned in a single :class:`SegmentEstimate`
  so the LLM feedback loop (§7.1) can render them atomically.

The :class:`SegmentEstimator` Protocol is the integration point.
A :class:`BaselineEstimator` ships with the package — it applies
the §6.2 heuristics (chars/cps for voice, payload duration for
music, preset lookup for anim) and serves both production and the
unit/integration tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Protocol, runtime_checkable

from .task_scheduler import ChannelKind, SchedulerTask


# ---------------------------------------------------------------------------
# Public data structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SegmentEstimate:
    """A single estimator's three-axis prediction for one task.

    Attributes:
        duration_ms: Predicted wall-clock time in milliseconds. Must
            be non-negative; ``None`` when the estimator cannot
            reason about this kind of task.
        cost: Abstract cost units (e.g. token-equivalents for a
            cloud TTS call). Must be non-negative; ``None`` when the
            estimator does not track cost.
        confidence: How much the estimator trusts its own numbers in
            ``[0.0, 1.0]``. The default baseline reports a value
            proportional to how much signal it had (e.g. ``1.0`` for
            an anim preset, ``0.5`` for a text-length heuristic).
        basis: Free-form provenance string — echoed in logs and in
            the §7.1 ``[SEGMENT PLAN]`` block so reviewers can see
            *why* a number was chosen (e.g. ``"chars_per_second=30"``,
            ``"preset=wave"``).
    """

    duration_ms: Optional[float]
    cost: Optional[float]
    confidence: float
    basis: str = ""

    def __post_init__(self) -> None:
        if self.duration_ms is not None and self.duration_ms < 0:
            raise ValueError("duration_ms must be non-negative")
        if self.cost is not None and self.cost < 0:
            raise ValueError("cost must be non-negative")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0.0, 1.0]")


@dataclass(frozen=True)
class EstimatorContext:
    """Read-only context passed to estimators alongside the task.

    Estimators may need a tiny bit of extra information that does
    not live on the task itself (e.g. the historical EMA of TTS
    duration, a calibration curve). ``EstimatorContext`` keeps the
    scheduler contract narrow while leaving room for those signals.

    Attributes:
        channel: The :class:`ChannelKind` the task was enqueued on
            (mirrors :attr:`SchedulerTask.channel` but pinned at
            submit time so estimators cannot disagree with the
            scheduler routing).
        tool: The tool name (mirrors :attr:`SchedulerTask.tool`).
        args: A read-only view of the original tool arguments.
        chars_per_second: Calibrated speech rate for voice tasks.
            Defaults to :attr:`BaselineEstimator.DEFAULT_CHARS_PER_SECOND`.
        preset_durations_ms: Optional preset manifest for anim tasks
            (mapping ``preset_name → duration_ms``).
        llm_eta_ms: Latest LLM ETA estimate (EMA from
            :class:`~rob_box_harness.core.acceptance.LLMEstimator` —
            not present in Phase 3 MVP, the field is here for
            future-proofing). ``None`` when not configured.
    """

    channel: ChannelKind
    tool: str
    args: Mapping[str, Any] = field(default_factory=dict)
    chars_per_second: float = 30.0
    preset_durations_ms: Optional[Mapping[str, float]] = None
    llm_eta_ms: Optional[float] = None


@runtime_checkable
class SegmentEstimator(Protocol):
    """Pluggable estimator interface (Phase 3).

    The scheduler holds one :class:`SegmentEstimator` instance and
    calls :meth:`estimate` for every task that needs a prediction.
    Implementations are expected to be **stateless or thread-safe**;
    the scheduler does not synchronise access. Calibration state
    belongs to :class:`~.quality.EstimatorQualityTracker`, which the
    estimator is free to consult via :meth:`SegmentEstimator.feedback`
    when wiring up the runtime feedback loop.
    """

    def estimate(
        self, task: SchedulerTask, context: EstimatorContext
    ) -> SegmentEstimate:
        """Return a three-axis prediction for *task*."""

    def feedback(
        self,
        task: SchedulerTask,
        estimate: SegmentEstimate,
        actual_duration_ms: float,
        actual_cost: Optional[float] = None,
    ) -> None:
        """Inform the estimator that *task* finished.

        Default no-op. Implementations may update an internal EMA,
        adjust ``chars_per_second``, etc. The scheduler calls this
        once per task after the executor returns.
        """


# ---------------------------------------------------------------------------
# Baseline estimator (per §6.2)
# ---------------------------------------------------------------------------


class BaselineEstimator:
    """Heuristic estimator that mirrors ``SCHEDULER_DESIGN.md`` §6.2.

    Heuristics:

    * ``voice`` (``speak_text`` / narration / sing / rap): duration
      via ``len(text) / chars_per_second``. The default
      ``chars_per_second`` (30) is calibrated against the
      Russian TTS + chipmunk 2x combination used in production
      (see ``estimate_tts_duration`` in
      ``rob_box_mcp_tools.tools.dialogue``). Confidence is
      ``0.5`` — the heuristic is calibrated, but actual rate
      depends on phoneme density.
    * ``music`` (``execute_music_code``): duration from the
      ``duration_sec`` payload the executor returns (or from the
      ``duration_ms`` argument when it is supplied eagerly).
      Confidence is ``0.9`` because the value is a hard contract
      from the music engine.
    * ``anim`` (``play_animation``): duration from a preset
      manifest (the only source-of-truth for clips). Confidence is
      ``1.0`` — we measured the preset.
    * Unknown channel / unknown tool: ``SegmentEstimate(None,
      None, 0.0)`` so callers can recognise the gap.

    Cost heuristics: voice and music cost ``1.0`` token per call
    (cheap, local rendering). Anim clips cost ``0.1`` (lighter than
    voice). Unknown tools report ``None`` cost so budget surfaces
    can fall back to a safe default.

    The estimator keeps **no state** — it is safe to share between
    loops and threads. Calibration feedback flows through
    :meth:`feedback`, which records the absolute error in the
    optional ``self.last_error_ms`` slot for inspection by
    integration tests; long-term calibration belongs to
    :class:`~.quality.EstimatorQualityTracker`.
    """

    #: Default Russian TTS rate, characters/second. Mirrors the
    #: calibration in ``rob_box_mcp_tools.tools.dialogue``'s
    #: ``estimate_tts_duration`` tool (DEFAULT_CHARS_PER_SECOND=30).
    DEFAULT_CHARS_PER_SECOND: float = 30.0

    #: Cost heuristic per call (abstract units).
    COST_VOICE: float = 1.0
    COST_MUSIC: float = 1.0
    COST_ANIM: float = 0.1

    def __init__(
        self,
        *,
        chars_per_second: float = DEFAULT_CHARS_PER_SECOND,
        preset_durations_ms: Optional[Mapping[str, float]] = None,
    ) -> None:
        if chars_per_second <= 0:
            raise ValueError("chars_per_second must be positive")
        self._chars_per_second = chars_per_second
        self._preset_durations_ms: dict[str, float] = dict(
            preset_durations_ms or {}
        )
        # Last recorded absolute error — useful for tests that want
        # to assert that feedback was wired through.
        self.last_error_ms: Optional[float] = None

    @property
    def chars_per_second(self) -> float:
        return self._chars_per_second

    def register_preset(self, name: str, duration_ms: float) -> None:
        """Add or update an anim preset duration (helper for tests)."""
        if duration_ms < 0:
            raise ValueError("duration_ms must be non-negative")
        self._preset_durations_ms[name] = duration_ms

    def estimate(
        self, task: SchedulerTask, context: EstimatorContext
    ) -> SegmentEstimate:
        # Honor the context's chars_per_second override — keeps the
        # estimator composable when a different calibration comes in
        # through the runtime feedback loop.
        cps = context.chars_per_second or self._chars_per_second

        channel = task.channel
        tool = task.tool
        args = task.args or {}

        if channel is ChannelKind.VOICE:
            text = args.get("text") or ""
            duration_ms = (len(text) / cps) * 1000.0
            # Cap confidence below 1.0 because phoneme density
            # matters — the chars/cps heuristic is a strong prior,
            # not a measurement.
            return SegmentEstimate(
                duration_ms=duration_ms,
                cost=self.COST_VOICE,
                confidence=0.5,
                basis=f"chars_per_second={cps:g}",
            )
        if channel is ChannelKind.MUSIC:
            duration_ms_raw = args.get("duration_ms")
            duration_sec = args.get("duration_sec")
            if duration_ms_raw is not None:
                duration_ms = float(duration_ms_raw)
            elif duration_sec is not None:
                duration_ms = float(duration_sec) * 1000.0
            else:
                # Default to a 1-second nominal segment when the
                # music engine has not told us — better than None so
                # the scheduler still has *some* number to budget.
                duration_ms = 1000.0
            return SegmentEstimate(
                duration_ms=duration_ms,
                cost=self.COST_MUSIC,
                confidence=0.9,
                basis="music_payload",
            )
        if channel is ChannelKind.ANIM:
            name = args.get("name") or args.get("preset")
            preset_ms = (
                self._preset_durations_ms.get(name)
                if name is not None
                else None
            )
            if preset_ms is None:
                # Anim without a manifest is the same as unknown —
                # surface a confident-zero rather than guessing.
                return SegmentEstimate(
                    duration_ms=None,
                    cost=self.COST_ANIM,
                    confidence=0.0,
                    basis=f"missing_preset:{name!r}",
                )
            return SegmentEstimate(
                duration_ms=float(preset_ms),
                cost=self.COST_ANIM,
                confidence=1.0,
                basis=f"preset:{name}",
            )
        # Unknown channel — voice / music / anim only for Phase 3.
        _ = tool  # explicitly ignored; tool-level routing is Phase 4
        return SegmentEstimate(
            duration_ms=None,
            cost=None,
            confidence=0.0,
            basis=f"unknown_channel:{channel.value}",
        )

    def feedback(
        self,
        task: SchedulerTask,
        estimate: SegmentEstimate,
        actual_duration_ms: float,
        actual_cost: Optional[float] = None,
    ) -> None:
        """Record the absolute error for inspection.

        Does not mutate ``chars_per_second`` — auto-calibration is
        the responsibility of :class:`~.quality.EstimatorQualityTracker`.
        Keeping this method side-effect-free (apart from the
        ``last_error_ms`` slot) lets the scheduler call it
        unconditionally without worrying about state drift.
        """
        if estimate.duration_ms is None:
            self.last_error_ms = None
            return
        self.last_error_ms = abs(actual_duration_ms - estimate.duration_ms)


# ---------------------------------------------------------------------------
# Helpers shared with the speculative pre-generator
# ---------------------------------------------------------------------------


def estimate_total_duration_ms(estimates: "list[SegmentEstimate]") -> float:
    """Sum non-``None`` durations, treating gaps as zero.

    The §6.4 invariant compares *remaining* voice time against an
    LLM ETA; we cannot pessimistically inflate ``None`` durations
    because that would suppress pre-gen. Instead, we treat gaps as
    zero (no credit, no penalty) and let the caller decide whether
    that is acceptable — usually they will refuse to speculatively
    pre-gen past an unknown segment.
    """
    if not estimates:
        return 0.0
    total = 0.0
    for est in estimates:
        if est.duration_ms is None:
            continue
        total += est.duration_ms
    # Guard against NaN / inf sneaking in from buggy estimators —
    # the pre-gen layer would happily schedule against an infinite
    # budget and never make progress.
    if math.isnan(total) or math.isinf(total):
        raise ValueError("estimator produced non-finite total duration")
    return total


__all__ = [
    "BaselineEstimator",
    "EstimatorContext",
    "SegmentEstimate",
    "SegmentEstimator",
    "estimate_total_duration_ms",
]
