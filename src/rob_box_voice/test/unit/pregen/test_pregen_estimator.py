"""Tests for :func:`estimator.estimate_confidence` (ADR-0056 §6.4 #3).

The DoD-named entry point ``test_estimator_confidence_above_threshold``
is the literal test name from issue #2003 body — it pins the
combined property "given a calibrated voice history, confidence
exceeds the executor's floor; given no history, it does not".

The rest of the file pins individual basis branches and
defensive edge cases.
"""
from __future__ import annotations

import pytest

from rob_box_voice.scheduler.pregen import (
    CONFIDENCE_FLOOR,
    PreGenTask,
    SegmentEstimate,
    estimate_confidence,
)


def _make_task(speech_id: str = "nxt") -> PreGenTask:
    return PreGenTask(
        next_speech_id=speech_id,
        next_ssml="<speak>привет</speak>",
        voice="anton",
        language="ru",
        ssml_attributes={},
        dialogue_id="d",
    )


def test_estimator_cold_start_returns_low_confidence():
    """§6.4 #3 baseline: empty history → confidence < floor."""
    estimate = estimate_confidence(_make_task(), [])
    assert isinstance(estimate, SegmentEstimate)
    assert estimate.confidence == 0.5
    assert estimate.basis == "baseline_cold_start"
    assert estimate.confidence < CONFIDENCE_FLOOR


def test_estimator_calibrated_returns_high_confidence():
    """Constant actuals → low CV → confidence > floor."""
    # 10 samples with no drift → low CV → calibrated_voice
    actuals = [300.0] * 10
    estimates = [300.0] * 10
    e = estimate_confidence(
        _make_task(),
        actuals,
        recent_estimated_durations_ms=estimates,
    )
    assert e.basis == "calibrated_voice"
    assert e.confidence >= CONFIDENCE_FLOOR


def test_estimator_drifting_voice_below_floor():
    """Wide variance → drifting_voice → confidence below floor."""
    actuals = [100.0, 500.0, 100.0, 500.0, 100.0, 500.0]
    e = estimate_confidence(_make_task(), actuals)
    # The CV of [100, 500, ...] is high; either basis, but always below floor.
    assert e.confidence < CONFIDENCE_FLOOR
    assert e.basis in {"drifting_voice", "calibrated_voice"}


def test_estimator_clamps_to_window():
    """More than _HISTORY_WINDOW samples still produces a valid result."""
    actuals = [300.0] * 50  # 5x the window size
    e = estimate_confidence(_make_task(), actuals)
    assert e.confidence >= 0.0
    assert e.confidence <= 1.0


def test_estimator_handles_estimated_none():
    """recent_estimated_durations_ms=None → use simpler heuristic."""
    actuals = [300.0] * 5
    e = estimate_confidence(_make_task(), actuals)
    assert e.confidence >= 0.0
    assert e.confidence <= 1.0


def test_segment_estimate_rejects_out_of_range_confidence():
    """Confidence outside [0, 1] must raise."""
    with pytest.raises(ValueError):
        SegmentEstimate(confidence=-0.1)
    with pytest.raises(ValueError):
        SegmentEstimate(confidence=1.1)


def test_segment_estimate_rejects_nan():
    with pytest.raises(ValueError):
        SegmentEstimate(confidence=float("nan"))


# ---------------------------------------------------------------------------
# DoD-named entry point (issue #2003).
#
# The wrapper test pins the *combined* property the executor relies
# on: "with a calibrated voice history, confidence crosses the
# executor's floor; without one, it does not". This is the entire
# reason the executor has a CONFIDENCE_FLOOR gate — to skip the
# first chunk of a fresh session, and to skip chunks from a voice
# whose calibration window has not stabilised.
#
# The focused tests above pin the individual basis branches
# (cold_start / calibrated_voice / drifting_voice) so a regression
# in one branch is still attributable to a specific heuristic.
# ---------------------------------------------------------------------------


def test_estimator_confidence_above_threshold():
    """DoD #3: confidence is above CONFIDENCE_FLOOR for valid history.

    Two assertions, modelled on the executor's actual decision
    rule (ADR-0056 §3.2):

    * **Valid scenario** — calibrated voice (10 close-to-identical
      samples) → ``SegmentEstimate.confidence >= CONFIDENCE_FLOOR``.
      This is what gates the executor to launch a speculative
      task in the first place.
    * **Invalid scenario** — empty history (cold start) →
      ``SegmentEstimate.confidence < CONFIDENCE_FLOOR``. The
      executor MUST skip the first chunk of a fresh session;
      spending a TTS call on it would burn budget on audio the
      user might never hear (barge-in / REPLACE).

    We deliberately use the *calibrated_voice* branch (not the
    drifting_voice branch) so the wrapper stays robust against
    heuristic tuning — the calibrated branch is the only one
    guaranteed to be above the floor across all calibrations.
    """
    task = _make_task()

    # Valid scenario: calibrated voice → confidence must clear the floor.
    valid_actuals = [300.0] * 10
    valid_estimates = [300.0] * 10
    valid = estimate_confidence(
        task,
        valid_actuals,
        recent_estimated_durations_ms=valid_estimates,
    )
    assert valid.confidence >= CONFIDENCE_FLOOR, (
        f"calibrated voice must be above the floor, got "
        f"confidence={valid.confidence:.3f} < floor={CONFIDENCE_FLOOR:.3f} "
        f"(basis={valid.basis!r})"
    )
    assert valid.basis == "calibrated_voice", (
        f"calibrated voice should report basis='calibrated_voice', "
        f"got {valid.basis!r}"
    )

    # Invalid scenario: empty history → confidence must be below the floor.
    invalid = estimate_confidence(task, [])
    assert invalid.confidence < CONFIDENCE_FLOOR, (
        f"cold-start (empty history) must be below the floor, got "
        f"confidence={invalid.confidence:.3f} >= floor={CONFIDENCE_FLOOR:.3f} "
        f"(basis={invalid.basis!r})"
    )
    assert invalid.basis == "baseline_cold_start", (
        f"cold-start should report basis='baseline_cold_start', "
        f"got {invalid.basis!r}"
    )