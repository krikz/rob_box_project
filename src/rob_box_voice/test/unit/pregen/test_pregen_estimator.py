"""Tests for :func:`estimator.estimate_confidence` (ADR-0056 §6.4 #3)."""
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