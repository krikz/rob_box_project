"""Phase 3 unit tests: EstimatorQualityTracker."""
from __future__ import annotations

import pytest

from rob_box_voice.scheduler.quality import (
    CalibrationBin,
    EstimatorQualityTracker,
    PredictionOutcome,
)
from rob_box_voice.scheduler.estimator import SegmentEstimate


def _est(duration_ms: float | None, confidence: float) -> SegmentEstimate:
    return SegmentEstimate(duration_ms=duration_ms, cost=1.0, confidence=confidence)


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------


def test_tracker_rejects_out_of_range_ema_alpha() -> None:
    with pytest.raises(ValueError, match="ema_alpha"):
        EstimatorQualityTracker(ema_alpha=0.0)
    with pytest.raises(ValueError, match="ema_alpha"):
        EstimatorQualityTracker(ema_alpha=1.5)


def test_tracker_rejects_invalid_window_and_tolerance() -> None:
    with pytest.raises(ValueError, match="window"):
        EstimatorQualityTracker(window=0)
    with pytest.raises(ValueError, match="tolerance_fraction"):
        EstimatorQualityTracker(tolerance_fraction=0.0)
    with pytest.raises(ValueError, match="tolerance_fraction"):
        EstimatorQualityTracker(tolerance_fraction=1.5)
    with pytest.raises(ValueError, match="tolerance_floor_ms"):
        EstimatorQualityTracker(tolerance_floor_ms=-1.0)


def test_tracker_rejects_malformed_bins() -> None:
    # Not contiguous — gap from 0.5 to 0.4.
    bad = (
        CalibrationBin(0.0, 0.4),
        CalibrationBin(0.4, 0.7),
    )
    # Adjacent partition is valid; test contiguity by *overlap*.
    overlapping = (
        CalibrationBin(0.0, 0.5),
        CalibrationBin(0.4, 0.8),
        CalibrationBin(0.8, 1.0),
    )
    with pytest.raises(ValueError, match="contiguous"):
        EstimatorQualityTracker(bins=overlapping)

    # Out of [0,1] range.
    bad_oor = (CalibrationBin(-0.1, 0.5), CalibrationBin(0.5, 1.0))
    with pytest.raises(ValueError, match="bins"):
        EstimatorQualityTracker(bins=bad_oor)

    # Empty bin (lower == upper).
    degenerate = (CalibrationBin(0.0, 0.0), CalibrationBin(0.0, 1.0))
    with pytest.raises(ValueError, match="lower < upper"):
        EstimatorQualityTracker(bins=degenerate)

    # Valid adjacent partition must be accepted.
    ok = (CalibrationBin(0.0, 0.5), CalibrationBin(0.5, 1.0))
    EstimatorQualityTracker(bins=ok)
    _ = bad  # silence unused warning; kept for documentation.


# ---------------------------------------------------------------------------
# Classification + EMA
# ---------------------------------------------------------------------------


def test_classify_under_accurate_and_over() -> None:
    tracker = EstimatorQualityTracker(
        tolerance_fraction=0.10, tolerance_floor_ms=10.0
    )
    # Accurate — predicted 1000, actual 1000.
    s1 = tracker.record(_est(1000.0, 0.7), actual_duration_ms=1000.0)
    assert s1.outcome == PredictionOutcome.ACCURATE
    # UNDER — predicted 1000, actual 1200 (estimator under-budgeted,
    # §6.4 invariant is at risk).
    s2 = tracker.record(_est(1000.0, 0.7), actual_duration_ms=1200.0)
    assert s2.outcome == PredictionOutcome.UNDER
    # Way under — predicted 1000, actual 1500.
    s3 = tracker.record(_est(1000.0, 0.7), actual_duration_ms=1500.0)
    assert s3.outcome == PredictionOutcome.UNDER
    # OVER — predicted 1500, actual 1000 (estimator over-budgeted,
    # scheduler kept more headroom than necessary).
    s4 = tracker.record(_est(1500.0, 0.7), actual_duration_ms=1000.0)
    assert s4.outcome == PredictionOutcome.OVER
    # Unknown — estimate is None.
    s5 = tracker.record(_est(None, 0.0), actual_duration_ms=1000.0)
    assert s5.outcome == PredictionOutcome.UNKNOWN


def test_ema_converges_with_repeated_signal() -> None:
    tracker = EstimatorQualityTracker(ema_alpha=0.5)
    # Feed 100 observations all off by +500ms.
    for _ in range(100):
        tracker.record(_est(1000.0, 0.7), actual_duration_ms=1500.0)
    assert tracker.ema_error_ms == pytest.approx(500.0, abs=1e-3)


def test_record_rejects_invalid_actual_duration() -> None:
    tracker = EstimatorQualityTracker()
    with pytest.raises(ValueError, match="positive"):
        tracker.record(_est(1000.0, 0.5), actual_duration_ms=0.0)
    with pytest.raises(ValueError, match="positive"):
        tracker.record(_est(1000.0, 0.5), actual_duration_ms=-100.0)
    with pytest.raises(ValueError, match="finite"):
        tracker.record(_est(1000.0, 0.5), actual_duration_ms=float("inf"))
    with pytest.raises(ValueError, match="finite"):
        tracker.record(_est(1000.0, 0.5), actual_duration_ms=float("nan"))


# ---------------------------------------------------------------------------
# MAPE / accuracy / outcome counters
# ---------------------------------------------------------------------------


def test_mape_and_accuracy_with_mixed_signal() -> None:
    tracker = EstimatorQualityTracker(
        tolerance_fraction=0.10, tolerance_floor_ms=50.0
    )
    # Accurate — 1000→1000, 1000→1050 (within 10% tolerance).
    # Under    — 1000→1500 (estimator under-budgeted).
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=1000.0)
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=1050.0)
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=1500.0)
    # MAPE uses actual_ms as the denominator per-sample:
    #   |0|/1000 + |50|/1050 + |500|/1500 = 0 + 0.047619 + 0.333333 ≈ 0.380952 → /3 ≈ 0.126984.
    assert tracker.mape() == pytest.approx(
        (0.0 / 1000.0 + 50.0 / 1050.0 + 500.0 / 1500.0) / 3.0
    )
    assert tracker.accuracy_ratio() == pytest.approx(2 / 3)
    counts = tracker.outcome_counts()
    assert counts[PredictionOutcome.ACCURATE] == 2
    assert counts[PredictionOutcome.UNDER] == 1
    assert counts[PredictionOutcome.OVER] == 0


def test_mape_returns_none_when_no_meaningful_samples() -> None:
    tracker = EstimatorQualityTracker()
    assert tracker.mape() is None
    # UNKNOWN-only samples must not turn into a 0 / 0 MAPE.
    tracker.record(_est(None, 0.0), actual_duration_ms=1000.0)
    assert tracker.mape() is None
    assert tracker.accuracy_ratio() is None


def test_window_drops_oldest_samples() -> None:
    tracker = EstimatorQualityTracker(window=2, ema_alpha=1.0)
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=1000.0)  # accurate
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=1000.0)  # accurate
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=2000.0)  # over
    tracker.record(_est(1000.0, 0.7), actual_duration_ms=2000.0)  # over
    # The first two should have been dropped — accuracy must be 0.
    assert tracker.accuracy_ratio() == 0.0


# ---------------------------------------------------------------------------
# Calibration bins
# ---------------------------------------------------------------------------


def test_calibration_bins_partition_samples_by_confidence() -> None:
    tracker = EstimatorQualityTracker(tolerance_fraction=0.10, tolerance_floor_ms=50.0)
    # 3 accurate predictions in [0.5, 0.6) bin.
    for _ in range(3):
        tracker.record(_est(1000.0, 0.55), actual_duration_ms=1000.0)
    # 2 inaccurate predictions in [0.9, 1.0) bin.
    for _ in range(2):
        tracker.record(_est(1000.0, 0.95), actual_duration_ms=2000.0)

    bins = {b.lower: b for b in tracker.calibration_bins()}
    assert bins[0.5].total == 3
    assert bins[0.5].correct == 3
    assert bins[0.5].observed_accuracy == pytest.approx(1.0)
    assert bins[0.9].total == 2
    assert bins[0.9].correct == 0
    assert bins[0.9].observed_accuracy == pytest.approx(0.0)
    # Unpopulated bins return None.
    assert bins[0.0].observed_accuracy is None


def test_unknown_outcomes_do_not_populate_bins() -> None:
    tracker = EstimatorQualityTracker()
    for _ in range(5):
        tracker.record(_est(None, 0.55), actual_duration_ms=1000.0)
    bins = {b.lower: b for b in tracker.calibration_bins()}
    for b in bins.values():
        assert b.total == 0


def test_calibration_gap_is_zero_for_perfect_estimator() -> None:
    tracker = EstimatorQualityTracker(tolerance_fraction=0.10, tolerance_floor_ms=50.0)
    # All predictions land in [0.9, 1.0) bin and are all accurate.
    for _ in range(20):
        tracker.record(_est(1000.0, 0.95), actual_duration_ms=1000.0)
    # Bin midpoint is 0.95, observed is 1.0 → gap = 0.05.
    assert tracker.calibration_gap() == pytest.approx(0.05)


def test_calibration_gap_returns_none_when_no_data() -> None:
    tracker = EstimatorQualityTracker()
    assert tracker.calibration_gap() is None
