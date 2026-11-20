"""loc_metrics: ошибка локализации против истины Gazebo и вердикт по планке Шифу.

Синтетические замеры, без ROS и без стенда. Речи людей нет: репозиторий публичный.
"""

from __future__ import annotations

import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "hil_loc_metrics", ROOT / "scripts" / "hil" / "diagnostics" / "loc_metrics.py"
)
loc = importlib.util.module_from_spec(_spec)
sys.modules["hil_loc_metrics"] = loc
_spec.loader.exec_module(loc)


def _series(est_fn, truth_fn=lambda t: (0.0, 0.0, 0.0), t_end=30.0, dt=0.1):
    out = []
    t = 0.0
    while t <= t_end + 1e-9:
        out.append(loc.Sample(round(t, 3), truth_fn(t), est_fn(t)))
        t += dt
    return out


def _map(sim, x, y, yaw, gx, gy, gyaw):
    return f"12:00:00.000 MAP sim={sim:.2f} x={x} y={y} yaw={yaw} gt_x={gx:.3f} gt_y={gy:.3f} gt_yaw={gyaw:.1f}"


def _goal(name, result, sim):
    return (
        f"12:00:01.000 GOAL_END {name} result={result} sim={sim:.2f} frame=10 "
        f"dur_sim=5.0 err=0.100 em_episodes=0 recoveries=0 gt=(0, 0, 0)"
    )


def test_pose_error_wraps_yaw_across_pi():
    dist, yaw = loc.pose_error((0.0, 0.0, math.pi - 0.01), (3.0, 4.0, -math.pi + 0.01))
    assert dist == pytest.approx(5.0)
    assert yaw == pytest.approx(0.02, abs=1e-9)


def test_bar_needs_both_on_one_pose():
    assert loc.within_bar(0.32, 0.35)
    assert not loc.within_bar(0.321, 0.0)
    assert not loc.within_bar(0.0, 0.351)


def test_correct_from_start_converges_at_once():
    metrics = loc.run_metrics(_series(lambda t: (0.02, 0.0, 0.01)))
    assert metrics["err0_m"] == pytest.approx(0.02)
    assert metrics["yaw0_rad"] == pytest.approx(0.01)
    assert metrics["converged_s"] == 0.0
    assert metrics["converged_path_m"] == 0.0
    assert metrics["jumps"] == 0
    assert metrics["err_final_m"] == pytest.approx(0.02)


def test_offset_then_one_good_jump():
    series = _series(lambda t: (0.5, 0.0, 0.0) if t < 12 else (0.01, 0.0, 0.0))
    metrics = loc.run_metrics(series)
    assert metrics["err0_m"] == pytest.approx(0.5)
    assert metrics["converged_s"] == pytest.approx(12.0, abs=0.11)
    assert (metrics["jumps"], metrics["bad_jumps"]) == (1, 0)


def test_never_converges_is_none_not_zero():
    metrics = loc.run_metrics(_series(lambda t: (1.0, 0.0, 0.0)))
    assert metrics["converged_s"] is None and metrics["converged_path_m"] is None
    assert metrics["err_final_m"] == pytest.approx(1.0)


def test_false_jump_counts_as_bad():
    metrics = loc.run_metrics(_series(lambda t: (0.0, 0.0, 0.0) if t < 10 else (1.0, 0.0, 0.0)))
    assert (metrics["jumps"], metrics["bad_jumps"]) == (1, 1)


def test_motion_of_truth_is_not_a_jump():
    series = _series(lambda t: (0.5 * t, 0.0, 0.0), truth_fn=lambda t: (0.5 * t, 0.0, 0.0))
    metrics = loc.run_metrics(series)
    assert metrics["jumps"] == 0
    assert metrics["path_m"] == pytest.approx(15.0, abs=0.01)


def test_short_pass_through_tolerance_is_not_convergence():
    series = _series(lambda t: (0.0, 0.0, 0.0) if 5 <= t < 7 else (0.6, 0.0, 0.0))
    assert loc.converged_at(series, 0.0, loc.Tolerance()) is None


def test_yaw_outside_tolerance_is_not_converged():
    series = _series(lambda t: (0.0, 0.0, math.radians(10)))
    assert loc.run_metrics(series)["converged_s"] is None


def test_missing_estimate_waits_and_is_not_zero():
    series = _series(lambda t: None if t < 8 else (0.0, 0.0, 0.0))
    metrics = loc.run_metrics(series)
    assert metrics["first_estimate_s"] == pytest.approx(8.0, abs=0.11)
    assert metrics["err0_m"] == pytest.approx(0.0)
    assert metrics["converged_s"] == pytest.approx(8.0, abs=0.11)


def test_no_pair_at_all_is_none_not_zero():
    metrics = loc.run_metrics(_series(lambda t: None))
    assert metrics["samples"] > 0
    assert metrics["err0_m"] is None
    assert metrics["yaw0_rad"] is None
    assert metrics["err_final_m"] is None
    assert metrics["first_estimate_s"] is None


def test_kidnap_time_and_metrics_from_event():
    truth = lambda t: (0.0, 0.0, 0.0) if t < 10 else (0.5, -1.5, 1.57)  # noqa: E731
    estimate = lambda t: (0.0, 0.0, 0.0) if t < 20 else (0.5, -1.5, 1.57)  # noqa: E731
    series = _series(estimate, truth)
    teleported = loc.kidnap_time(series, loc.Tolerance())
    assert teleported == pytest.approx(10.0, abs=0.11)
    metrics = loc.run_metrics(series, teleported)
    assert metrics["err0_m"] == pytest.approx(math.hypot(0.5, 1.5), abs=1e-3)
    assert metrics["converged_s"] == pytest.approx(10.0, abs=0.11)
    assert loc.estimate_jumps(series, teleported, loc.Tolerance()) == (1, 0)


def test_no_kidnap_is_none():
    assert loc.kidnap_time(_series(lambda t: (0.0, 0.0, 0.0)), loc.Tolerance()) is None


def test_empty_run_is_not_zeros():
    assert loc.run_metrics([]) == {"samples": 0}


def test_summary_markdown_dash_for_none():
    table = loc.summary_markdown([{"label": "b1", "err0_m": 0.5, "converged_s": None}])
    assert table.splitlines()[2].startswith("| b1 | 0.5 | — | — |")


def test_map_line_none_estimate_is_not_zero():
    sample = loc.parse_map_line(_map(1.0, "none", "none", "none", 1.25, -0.5, 90.0))
    assert sample.estimate is None
    assert sample.truth[0] == pytest.approx(1.25)
    assert sample.truth[2] == pytest.approx(math.pi / 2)


def test_map_line_converts_degrees_from_route_log():
    sample = loc.parse_map_line(_map(2.0, "1.000", "2.000", "180.0", 1.100, 2.000, 180.0))
    assert sample.estimate[2] == pytest.approx(math.pi)
    dist, yaw = loc.pose_error(sample.truth, sample.estimate)
    assert dist == pytest.approx(0.1)
    assert yaw == pytest.approx(0.0)


def test_foreign_line_is_not_a_sample():
    assert loc.parse_map_line("12:00:00.000 TRACK sim=1.00 x=0.000 y=0.000 yaw=0.0") is None
    assert loc.parse_goal_end("12:00:00.000 GOAL_START M1 target=(0,0,0) sim=1.00 frame=1") is None


def test_pass_inside_shifu_bar():
    sample = loc.Sample(1.0, (0.0, 0.0, 0.0), (0.2939, 0.0, 0.1348))
    row = loc.goal_verdict("STATUS_SUCCEEDED", sample)
    assert row["verdict"] == loc.PASS
    assert row["err_m"] == pytest.approx(0.294, abs=1e-3)


def test_yaw_over_bar_is_fail_even_if_position_ok():
    sample = loc.Sample(1.0, (0.0, 0.0, 0.0), (0.1, 0.0, 0.351))
    assert loc.goal_verdict("STATUS_SUCCEEDED", sample)["verdict"] == loc.FAIL


def test_position_over_bar_is_fail():
    sample = loc.Sample(1.0, (0.0, 0.0, 0.0), (0.321, 0.0, 0.0))
    assert loc.goal_verdict("STATUS_SUCCEEDED", sample)["verdict"] == loc.FAIL


def test_timeout_is_never_pass_even_inside_bar():
    sample = loc.Sample(1.0, (0.0, 0.0, 0.0), (0.01, 0.0, 0.01))
    row = loc.goal_verdict("TIMEOUT", sample)
    assert row["verdict"] == loc.TIMEOUT
    assert "err_m" not in row


def test_nav_abort_is_fail_not_pass():
    sample = loc.Sample(1.0, (0.0, 0.0, 0.0), (0.01, 0.0, 0.01))
    assert loc.goal_verdict("STATUS_ABORTED", sample)["verdict"] == loc.FAIL


def test_missing_truth_is_no_data_not_zero():
    row = loc.goal_verdict("STATUS_SUCCEEDED", None)
    assert row["verdict"] == loc.NO_DATA
    assert "err_m" not in row
    assert "0.000" not in row["reason"]


def test_missing_estimate_is_no_data_not_zero():
    sample = loc.Sample(1.0, (1.0, 2.0, 0.0), None)
    row = loc.goal_verdict("STATUS_SUCCEEDED", sample)
    assert row["verdict"] == loc.NO_DATA
    assert row["reason"].startswith("нет оценки")


def test_log_verdict_uses_nearest_map_and_keeps_none():
    text = "\n".join(
        [
            _map(9.80, "0.290", "0.000", "7.7", 0.0, 0.0, 0.0),
            _goal("M3", "STATUS_SUCCEEDED", 10.0),
            _map(20.0, "none", "none", "none", 1.0, 0.0, 0.0),
            _goal("M4", "STATUS_SUCCEEDED", 20.0),
            _map(30.0, "0.010", "0.000", "0.0", 0.0, 0.0, 0.0),
            _goal("R5", "TIMEOUT", 30.0),
        ]
    )
    rows = loc.verdicts_from_log(text)
    assert [row["verdict"] for row in rows] == [loc.PASS, loc.NO_DATA, loc.TIMEOUT]
    assert rows[0]["err_m"] == pytest.approx(0.29)
    assert loc.run_verdict(rows) == loc.NO_DATA


def test_run_of_passes_is_pass_and_one_fail_is_fail():
    ok = {"verdict": loc.PASS}
    assert loc.run_verdict([ok, ok]) == loc.PASS
    assert loc.run_verdict([ok, {"verdict": loc.FAIL}]) == loc.FAIL
    assert loc.run_verdict([ok, {"verdict": loc.TIMEOUT}]) == loc.TIMEOUT
    assert loc.run_verdict([]) == loc.NO_DATA


def test_cli_prints_verdict(tmp_path):
    log = tmp_path / "route.log"
    log.write_text(
        _map(1.0, "0.100", "0.000", "0.0", 0.0, 0.0, 0.0) + "\n" + _goal("M1", "STATUS_SUCCEEDED", 1.0) + "\n",
        encoding="utf-8",
    )
    captured = json.loads(
        subprocess.run(
            [sys.executable, str(ROOT / "scripts/hil/diagnostics/loc_metrics.py"), str(log)],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    )
    assert captured["verdict"] == loc.PASS
    assert captured["goals"][0]["name"] == "M1"
