"""Метрики стенда локализации (#4346): ошибка истина против map, схождение, скачки, похищение."""

from __future__ import annotations

import math

import pytest
from rob_box_sim.loc_metrics import (
    Sample,
    Tolerance,
    converged_at,
    estimate_jumps,
    kidnap_time,
    pose_error,
    run_metrics,
    summary_markdown,
)


def _series(est_fn, truth_fn=lambda t: (0.0, 0.0, 0.0), t_end=30.0, dt=0.1):
    out = []
    t = 0.0
    while t <= t_end + 1e-9:
        out.append(Sample(round(t, 3), truth_fn(t), est_fn(t)))
        t += dt
    return out


def test_pose_error_wraps_yaw():
    d, a = pose_error((0, 0, math.pi - 0.01), (3, 4, -math.pi + 0.01))
    assert d == pytest.approx(5.0)
    assert a == pytest.approx(math.degrees(0.02), abs=1e-6)


def test_correct_from_start_converges_at_once():
    m = run_metrics(_series(lambda t: (0.02, 0.0, 0.01)))
    assert m["err0_m"] == pytest.approx(0.02)
    assert m["converged_s"] == 0.0
    assert m["converged_path_m"] == 0.0
    assert m["jumps"] == 0
    assert m["err_final_m"] == pytest.approx(0.02)


def test_offset_half_meter_then_relocalized_jump():
    """Старт со сдвигом 0.5 м, на 12-й секунде rtabmap «нашёл себя»: один хороший скачок."""
    s = _series(lambda t: (0.5, 0.0, 0.0) if t < 12 else (0.01, 0.0, 0.0))
    m = run_metrics(s)
    assert m["err0_m"] == pytest.approx(0.5)
    assert m["converged_s"] == pytest.approx(12.0, abs=0.11)
    assert (m["jumps"], m["bad_jumps"]) == (1, 0)


def test_never_converges_is_none():
    m = run_metrics(_series(lambda t: (1.0, 0.0, 0.0)))
    assert m["converged_s"] is None and m["converged_path_m"] is None
    assert m["err_final_m"] == pytest.approx(1.0)


def test_false_jump_counts_as_bad():
    """Был прав, прыгнул на 1 м в чужое место (ложное узнавание)."""
    m = run_metrics(_series(lambda t: (0.0, 0.0, 0.0) if t < 10 else (1.0, 0.0, 0.0)))
    assert (m["jumps"], m["bad_jumps"]) == (1, 1)


def test_motion_of_truth_is_not_a_jump():
    """Едет 0.5 м/с, оценка следует за ним — скачков нет."""
    s = _series(lambda t: (0.5 * t, 0.0, 0.0), truth_fn=lambda t: (0.5 * t, 0.0, 0.0))
    m = run_metrics(s)
    assert m["jumps"] == 0
    assert m["path_m"] == pytest.approx(15.0, abs=0.01)


def test_converged_needs_hold():
    """Короткий проход через допуск (2 с при hold 5 с) — не схождение."""
    s = _series(lambda t: (0.0, 0.0, 0.0) if 5 <= t < 7 else (0.6, 0.0, 0.0))
    assert converged_at(s, 0.0, Tolerance()) is None


def test_yaw_outside_tolerance_is_not_converged():
    s = _series(lambda t: (0.0, 0.0, math.radians(10)))
    assert run_metrics(s)["converged_s"] is None


def test_missing_estimate_waits():
    s = _series(lambda t: None if t < 8 else (0.0, 0.0, 0.0))
    m = run_metrics(s)
    assert m["first_estimate_s"] == pytest.approx(8.0, abs=0.11)
    assert m["converged_s"] == pytest.approx(8.0, abs=0.11)


def test_kidnap_time_and_metrics_from_event():
    truth = lambda t: (0.0, 0.0, 0.0) if t < 10 else (0.5, -1.5, 1.57)  # noqa: E731
    est = lambda t: (0.0, 0.0, 0.0) if t < 20 else (0.5, -1.5, 1.57)  # noqa: E731
    s = _series(est, truth)
    tk = kidnap_time(s, Tolerance())
    assert tk == pytest.approx(10.0, abs=0.11)
    m = run_metrics(s, tk)
    assert m["err0_m"] == pytest.approx(math.hypot(0.5, 1.5), abs=1e-3)
    assert m["converged_s"] == pytest.approx(10.0, abs=0.11)
    # сам телепорт (скачок истины) скачком оценки не считается
    assert estimate_jumps(s, tk, Tolerance()) == (1, 0)


def test_no_kidnap_is_none():
    assert kidnap_time(_series(lambda t: (0, 0, 0)), Tolerance()) is None


def test_empty_run():
    assert run_metrics([]) == {"samples": 0}


def test_summary_markdown_dash_for_none():
    md = summary_markdown([{"label": "b1", "err0_m": 0.5, "converged_s": None}])
    assert md.splitlines()[2].startswith("| b1 | 0.5 | — | — |")
