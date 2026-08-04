"""Runtime estimator quality tracker (Phase 3, §6.5 / §7.1).

The estimator contract (see :mod:`rob_box_voice.scheduler.estimator`)
gives the LLM feedback loop three numbers per task — duration,
cost, confidence — but no way to know whether the predictor is
**actually** well-calibrated. This module fills that gap.

What it tracks
--------------

* **EMA error** (exponential moving average of absolute error in
  milliseconds) — the same shape as :class:`LLMEstimator` in
  §6.3, but for the estimator rather than the LLM. Useful as a
  cheap health-check you can log on every turn.
* **MAPE** (mean absolute percentage error) — averaged over the
  last *N* samples. Required by §11.3 acceptance ("метрики
  качества предсказаний") — it answers "how off, in % terms,
  are we on average?".
* **Calibration bins** — counts of predictions bucketed by the
  predicted confidence. After enough samples, the bin for
  ``[0.5, 0.6)`` should hold ~55% accurate predictions; if not,
  the estimator is miscalibrated and the LLM should treat its
  ``confidence`` numbers as a hint, not a contract.
* **Outcome counts** — under / accurate / over. We consider a
  prediction *accurate* when ``|predicted − actual| ≤
  tolerance_ms`` (default 10 % of the actual duration, clamped
  to a sane floor). *Over* means the scheduler over-budgeted
  (predicted too long); *under* means it under-budgeted (the §6.4
  invariant violation that the speculative pre-generator is
  supposed to mask).

Why a separate class
--------------------

The estimator itself stays stateless (§6.2) so it is safe to share
across loops. The tracker, on the other hand, *is* state — it is
attached to the scheduler (one per channel-set) and survives the
lifetime of the dialogue session.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

from .estimator import SegmentEstimate


# ---------------------------------------------------------------------------
# Public outcome enum
# ---------------------------------------------------------------------------


class PredictionOutcome:
    """Outcome classification for a single estimator observation.

    Stored as plain strings (not ``enum.Enum``) so they serialise
    cleanly into the §7.1 ``[SEGMENT PLAN]`` feedback block and
    into the SQLite-backed run history.

    Semantics (per :class:`EstimatorQualityTracker._classify`):

    * ``UNDER`` — the estimator **under-budgeted** the task
      (``actual_ms > estimated_ms``). This is the dangerous case:
      the §6.4 voice-queue invariant is at risk, and the
      speculative pre-generator should prefer to surface this
      metric to the LLM.
    * ``ACCURATE`` — prediction was within ``tolerance`` of actual.
    * ``OVER`` — the estimator **over-budgeted** the task
      (``actual_ms < estimated_ms``). Safe but wasteful: the
      scheduler kept more headroom than it needed.
    * ``UNKNOWN`` — estimate was ``None`` or actual was non-finite.
    """

    UNDER = "under"
    ACCURATE = "accurate"
    OVER = "over"
    UNKNOWN = "unknown"  # estimate was None or actual was non-finite


# ---------------------------------------------------------------------------
# Sample record
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EstimatorSample:
    """One observation fed into :class:`EstimatorQualityTracker`.

    Attributes:
        estimated_ms: What the estimator predicted.
        actual_ms: What the executor reported (always > 0 when the
            sample is meaningful; ``None`` for unknown channels).
        confidence: The estimator's self-reported confidence in
            ``[0.0, 1.0]``. Kept verbatim so calibration bins can
            match what the LLM was told.
        outcome: :class:`PredictionOutcome` computed at intake
            time — recomputing it would be wasteful.
    """

    estimated_ms: Optional[float]
    actual_ms: Optional[float]
    confidence: float
    outcome: str = PredictionOutcome.UNKNOWN


# ---------------------------------------------------------------------------
# Calibration bins
# ---------------------------------------------------------------------------


@dataclass
class CalibrationBin:
    """Histogram bucket for one confidence interval.

    Each bin tracks ``correct`` (samples whose outcome was
    :data:`PredictionOutcome.ACCURATE`) and ``total`` (every sample
    that landed in this bin). The :attr:`observed_accuracy`
    property is the only thing the LLM feedback loop consumes —
    ``predicted=0.7`` bin reporting ``observed_accuracy=0.55``
    means the estimator overstates its own quality.
    """

    lower: float  # inclusive
    upper: float  # exclusive
    correct: int = 0
    total: int = 0

    @property
    def observed_accuracy(self) -> Optional[float]:
        if self.total == 0:
            return None
        return self.correct / self.total

    def matches(self, confidence: float) -> bool:
        return self.lower <= confidence < self.upper


def _default_bins() -> tuple[CalibrationBin, ...]:
    """Standard 10-bin histogram in ``[0.0, 1.0]`` (last bin is closed)."""
    edges = [i / 10.0 for i in range(11)]
    return tuple(
        CalibrationBin(lower=edges[i], upper=edges[i + 1]) for i in range(10)
    )


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------


@dataclass
class EstimatorQualityTracker:
    """EMA + MAPE + calibration metrics for the active estimator.

    Parameters
    ----------
    ema_alpha:
        Smoothing factor for the EMA error. Mirrors
        :class:`LLMEstimator` (§6.3) — the default ``0.3`` is the
        "respond quickly to recent signal" trade-off.
    window:
        How many of the most recent samples to average for MAPE.
        ``None`` keeps the entire history.
    tolerance_fraction:
        Fraction of ``actual_ms`` that counts as "accurate" when
        classifying a prediction. ``0.10`` means a 10 % error
        band, matching the §6.2 ±15 % heuristic budget for voice
        (we are stricter so the metric is meaningful).
    tolerance_floor_ms:
        Minimum tolerance window in milliseconds. Protects tiny
        tasks from being misclassified because of sub-millisecond
        jitter in the executor's `time.monotonic` clock.
    bins:
        Histogram for calibration. Defaults to 10 equal-width bins
        in ``[0.0, 1.0]``; tests can substitute a coarser set.
    """

    ema_alpha: float = 0.3
    window: Optional[int] = 256
    tolerance_fraction: float = 0.10
    tolerance_floor_ms: float = 50.0
    bins: tuple[CalibrationBin, ...] = field(default_factory=_default_bins)
    _ema_error_ms: float = 0.0
    _history: Deque[EstimatorSample] = field(default_factory=deque)
    _under: int = 0
    _accurate: int = 0
    _over: int = 0
    _unknown: int = 0

    def __post_init__(self) -> None:
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0.0, 1.0]")
        if self.window is not None and self.window <= 0:
            raise ValueError("window must be positive or None")
        if not 0.0 < self.tolerance_fraction <= 1.0:
            raise ValueError("tolerance_fraction must be in (0.0, 1.0]")
        if self.tolerance_floor_ms < 0:
            raise ValueError("tolerance_floor_ms must be non-negative")
        # Validate the bin partition. Must be strictly increasing
        # AND non-overlapping (a partition, not just a sequence of
        # intervals). We start from index 1 because index 0 has no
        # predecessor to align with.
        for index, bin_ in enumerate(self.bins):
            if bin_.lower < 0.0 or bin_.upper > 1.0 + 1e-9:
                raise ValueError("bins must cover [0.0, 1.0]")
            if bin_.upper <= bin_.lower:
                raise ValueError("each bin must have lower < upper")
            if index == 0:
                if bin_.lower != 0.0:
                    raise ValueError("first bin must start at 0.0")
                continue
            if bin_.lower != self.bins[index - 1].upper:
                raise ValueError("bins must form a contiguous partition")

    # ----- intake --------------------------------------------------------

    def record(
        self,
        estimate: SegmentEstimate,
        actual_duration_ms: float,
    ) -> EstimatorSample:
        """Feed one observation into the tracker.

        ``actual_duration_ms`` MUST be the wall-clock time the
        executor reported (positive and finite). The
        :class:`SchedulerTask`-side wiring is responsible for
        computing it via ``time.monotonic`` deltas — this class
        stays agnostic to the scheduler internals so it is
        trivially testable.
        """
        if actual_duration_ms is None or actual_duration_ms <= 0:
            raise ValueError("actual_duration_ms must be positive")
        if math.isnan(actual_duration_ms) or math.isinf(actual_duration_ms):
            raise ValueError("actual_duration_ms must be finite")

        outcome = self._classify(estimate.duration_ms, actual_duration_ms)
        sample = EstimatorSample(
            estimated_ms=estimate.duration_ms,
            actual_ms=actual_duration_ms,
            confidence=estimate.confidence,
            outcome=outcome,
        )

        # EMA accumulates over the *full* lifetime — it is a cheap
        # health-check that should not forget ancient history. The
        # window only governs MAPE / outcome counters / calibration
        # bins (the metrics the LLM feedback loop consumes).
        if estimate.duration_ms is not None:
            error = abs(actual_duration_ms - estimate.duration_ms)
            self._ema_error_ms = (
                self.ema_alpha * error
                + (1.0 - self.ema_alpha) * self._ema_error_ms
            )

        self._history.append(sample)
        if self.window is not None:
            while len(self._history) > self.window:
                self._history.popleft()

        # Recompute the window-scoped views. We rebuild the bins
        # from scratch every intake rather than try to subtract
        # the dropped sample — it is O(bins_count + window) and
        # ``window`` defaults to 256, which is cheap. ``bins`` are
        # reset so we never double-count the dropped samples.
        self._refresh_window_views()
        return sample

    def _refresh_window_views(self) -> None:
        """Rebuild the window-scoped counters and bins from ``_history``."""
        # Reset bins first — the dataclass instances are stable
        # (the constructor passes them in once), but we still need
        # to zero the counters that grew inside ``record``.
        for bin_ in self.bins:
            bin_.correct = 0
            bin_.total = 0
        self._under = 0
        self._accurate = 0
        self._over = 0
        self._unknown = 0
        for sample in self._history:
            if sample.outcome is PredictionOutcome.UNDER:
                self._under += 1
            elif sample.outcome is PredictionOutcome.ACCURATE:
                self._accurate += 1
            elif sample.outcome is PredictionOutcome.OVER:
                self._over += 1
            else:
                self._unknown += 1
            if sample.outcome is PredictionOutcome.UNKNOWN:
                continue
            for bin_ in self.bins:
                if bin_.matches(sample.confidence):
                    bin_.total += 1
                    if sample.outcome is PredictionOutcome.ACCURATE:
                        bin_.correct += 1
                    break

    def _classify(
        self,
        estimated_ms: Optional[float],
        actual_ms: float,
    ) -> str:
        if estimated_ms is None:
            return PredictionOutcome.UNKNOWN
        if math.isnan(estimated_ms) or math.isinf(estimated_ms):
            return PredictionOutcome.UNKNOWN
        tolerance = max(actual_ms * self.tolerance_fraction, self.tolerance_floor_ms)
        error = actual_ms - estimated_ms
        if abs(error) <= tolerance:
            return PredictionOutcome.ACCURATE
        return PredictionOutcome.OVER if error < 0 else PredictionOutcome.UNDER

    # ----- read-only views ---------------------------------------------

    @property
    def ema_error_ms(self) -> float:
        """EMA of absolute prediction error in milliseconds."""
        return self._ema_error_ms

    @property
    def sample_count(self) -> int:
        """Total samples ever recorded (counts both window and global)."""
        return len(self._history)

    def mape(self) -> Optional[float]:
        """Mean absolute percentage error over the current window.

        Returns ``None`` when there are no samples *with* a
        prediction — MAPE is undefined for the all-UNKNOWN case.
        """
        if not self._history:
            return None
        total = 0.0
        count = 0
        for sample in self._history:
            if (
                sample.estimated_ms is None
                or sample.actual_ms is None
                or sample.actual_ms <= 0
                or sample.outcome is PredictionOutcome.UNKNOWN
            ):
                continue
            total += abs(sample.estimated_ms - sample.actual_ms) / sample.actual_ms
            count += 1
        if count == 0:
            return None
        return total / count

    def accuracy_ratio(self) -> Optional[float]:
        """Fraction of *recorded* samples that landed in ACCURATE."""
        denom = self._under + self._accurate + self._over
        if denom == 0:
            return None
        return self._accurate / denom

    def outcome_counts(self) -> dict[str, int]:
        """Snapshot of the outcome counters — useful for diagnostics."""
        return {
            PredictionOutcome.UNDER: self._under,
            PredictionOutcome.ACCURATE: self._accurate,
            PredictionOutcome.OVER: self._over,
            PredictionOutcome.UNKNOWN: self._unknown,
        }

    def calibration_bins(self) -> tuple[CalibrationBin, ...]:
        """Return a deep-enough copy of the bins for inspection.

        The list itself is fresh; the :class:`CalibrationBin`
        dataclasses are shared (they are simple counters, so this
        is fine — readers cannot mutate state in a way that would
        break invariants).
        """
        return tuple(self.bins)

    def calibration_gap(self) -> Optional[float]:
        """Average |observed_accuracy − bin_midpoint| over populated bins.

        ``0.0`` means perfectly calibrated (every bin's actual
        hit rate matches the midpoint of the confidence range
        it covers). ``0.5`` means the estimator's confidence is
        essentially noise. ``None`` when no bin has data yet.
        """
        populated = [b for b in self.bins if b.total > 0]
        if not populated:
            return None
        total_gap = 0.0
        for bin_ in populated:
            midpoint = (bin_.lower + bin_.upper) / 2.0
            assert bin_.observed_accuracy is not None
            total_gap += abs(bin_.observed_accuracy - midpoint)
        return total_gap / len(populated)


__all__ = [
    "CalibrationBin",
    "EstimatorQualityTracker",
    "EstimatorSample",
    "PredictionOutcome",
]
