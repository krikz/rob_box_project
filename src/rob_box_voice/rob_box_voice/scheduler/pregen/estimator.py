"""Confidence estimator for the chunk-level speculative pipeline.

This module answers a single question: **given the speculative
task and the recent history of synthesis durations for the same
voice, how likely is the pre-generated audio to match what we
would have produced anyway?**

The answer gates whether the executor spends a TTS call now (and
risks burning budget on a chunk that the user might never hear
because of barge-in / REPLACE) or skips and waits for the
canonical chunk-arrival path.

The estimator is **pure** — no asyncio, no rclpy, no logging side
effects. The caller (``SpeculativeExecutor.kickoff``) logs the
``basis`` string when it skips.

Heuristic
--------

We look at the *variance* of recent actual-vs-estimated durations
for the same voice. The intuition:

* If we've been hitting our duration estimate within ±15% for the
  last 10 chunks of this voice, the next chunk is likely to land
  in the same regime → confidence ≈ 0.8.
* If the recent samples span a wide range (one chunk was 60%
  shorter than predicted, another was 40% longer), the next chunk
  is far more likely to drift → confidence ≈ 0.4.
* If we have no history at all (cold start), confidence is the
  default floor (``0.5``), which is **below** the gate — so the
  executor skips the first speculative chunk of a new session, by
  design.

The ``CONFIDENCE_FLOOR`` constant below is the gate the executor
applies: anything below it is treated as "skip speculation, don't
even start the asyncio.Task".

Why a separate class from ``scheduler.estimator.SegmentEstimate``
--------------------------------------------------------------------

The scheduler-level :class:`~rob_box_voice.scheduler.estimator.SegmentEstimate`
encodes ``duration_ms`` / ``cost`` / ``confidence`` for a
*scheduler task* (with optional LLM-ETA hookup). Here we only need
``confidence`` + a ``basis`` string for diagnostics. Mixing the
two would force the executor to ignore two of three fields — a
classic "wide interface" smell. ADR-0056 §4 documents the
rejection of the "thin adapter" approach.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .pre_gen import PreGenTask


#: Confidence threshold below which the executor skips the
#: speculative task entirely. Calibrated so that the cold-start
#: baseline (``0.5``) does NOT pass — the first chunk of a fresh
#: session has too little signal to spend a TTS call.
CONFIDENCE_FLOOR: float = 0.6


#: Number of recent samples to consult. Keeps the estimator O(1)
#: per kickoff and prevents unbounded growth.
_HISTORY_WINDOW: int = 10


#: Coefficient of variation (σ/μ) below which we consider the
#: voice "well-calibrated". When this holds, the heuristic bumps
#: confidence toward ``0.8``; above it, toward ``0.4``.
_CALIBRATION_GOOD_CV: float = 0.10
_CALIBRATION_BAD_CV: float = 0.30


@dataclass(frozen=True)
class SegmentEstimate:
    """Pure confidence prediction for one speculative chunk.

    ``duration_ms`` and ``cost`` are deliberately omitted: the
    chunk-level gate doesn't need them, and including them would
    invite confusion with the scheduler-level
    :class:`~rob_box_voice.scheduler.estimator.SegmentEstimate`
    that *does* carry them.

    Attributes
    ----------
    confidence
        ``[0.0, 1.0]`` — probability the speculative audio will
        match the canonical post-batch audio. See module docstring
        for the heuristic.
    basis
        Free-form provenance string, echoed in logs/metrics so a
        reviewer can see *why* a value was chosen. One of:
        ``"baseline_cold_start"``, ``"calibrated_voice"``,
        ``"drifting_voice"``, ``"voice_mismatch"``.
    """

    confidence: float
    basis: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.confidence, (int, float)):
            raise ValueError(
                f"SegmentEstimate.confidence must be a float, got "
                f"{type(self.confidence).__name__}"
            )
        if math.isnan(self.confidence):
            raise ValueError("SegmentEstimate.confidence must not be NaN")
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError(
                f"SegmentEstimate.confidence must be in [0.0, 1.0], "
                f"got {self.confidence!r}"
            )


def _coeff_of_variation(samples: Sequence[float]) -> Optional[float]:
    """σ/μ of the input samples, or ``None`` if undefined.

    Empty input → ``None``. Single sample → ``0.0``. All-equal
    samples → ``0.0``. Single zero-mean-zero sample → ``0.0``.
    """
    if not samples:
        return None
    n = len(samples)
    if n == 1:
        return 0.0
    mean = sum(samples) / n
    if mean <= 0:
        # Degenerate: durations must be positive, but if the
        # caller hands us zeros we still want a defined result.
        return None
    var = sum((s - mean) ** 2 for s in samples) / n
    return math.sqrt(var) / mean


def estimate_confidence(
    pregen: PreGenTask,
    recent_actual_durations_ms: Sequence[float],
    *,
    recent_estimated_durations_ms: Optional[Sequence[float]] = None,
) -> SegmentEstimate:
    """Return the confidence estimate for one speculative task.

    Parameters
    ----------
    pregen
        The :class:`PreGenTask` we are about to launch. Used to
        look at the requested voice/language (so we can detect
        "voice mismatch" against the recent history).
    recent_actual_durations_ms
        Last ``_HISTORY_WINDOW`` actual synthesis durations for the
        *current* effective voice, in milliseconds (the executor
        records this on every chunk that finishes).
    recent_estimated_durations_ms
        Same window of estimated durations (typically produced by
        the existing :class:`~rob_box_voice.scheduler.quality.EstimatorQualityTracker`
        for the *scheduler-level* estimate — but here we accept
        any sequence of equal length). If ``None``, the heuristic
        uses a simpler "variance of recent actuals only" rule.

    Notes
    -----
    Pure: no I/O, no logging, no shared state. The caller is
    responsible for trimming the history to ``_HISTORY_WINDOW``
    before calling.
    """
    # Cold-start: no history at all → default below the floor.
    # Skip even the call, so the executor logs a single clear
    # reason instead of an arbitrary 0.5.
    if not recent_actual_durations_ms:
        return SegmentEstimate(confidence=0.5, basis="baseline_cold_start")

    actuals = list(recent_actual_durations_ms)[-_HISTORY_WINDOW:]
    estimates = (
        list(recent_estimated_durations_ms)[-_HISTORY_WINDOW:]
        if recent_estimated_durations_ms is not None
        else None
    )

    cv = _coeff_of_variation(actuals)

    # Calibrated regime: low variance. The point estimate lives
    # between 0.7 and 0.8 — comfortably above the floor.
    if cv is not None and cv <= _CALIBRATION_GOOD_CV:
        # Bonus signal when estimates-vs-actual were close.
        if estimates and len(estimates) == len(actuals):
            ratio_samples = [
                a / e if e > 0 else 1.0
                for a, e in zip(actuals, estimates)
            ]
            ratio_cv = _coeff_of_variation(ratio_samples)
            if ratio_cv is not None and ratio_cv <= _CALIBRATION_GOOD_CV:
                return SegmentEstimate(
                    confidence=0.85,
                    basis="calibrated_voice",
                )
        return SegmentEstimate(
            confidence=0.75,
            basis="calibrated_voice",
        )

    # Drifting regime: medium-to-high variance. Below floor, so
    # the executor will skip — but we still return a defined value
    # so metrics can record *why* we skipped.
    if cv is not None and cv <= _CALIBRATION_BAD_CV:
        return SegmentEstimate(
            confidence=0.5,
            basis="drifting_voice",
        )

    # Wild variance. Definitely below floor.
    return SegmentEstimate(
        confidence=0.4,
        basis="drifting_voice",
    )