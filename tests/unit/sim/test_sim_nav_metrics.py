"""rob_box_sim.nav_metrics и сценарии стенда: счёт метрик по образцам, без ROS."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from rob_box_sim import worlds_catalog as wc
from rob_box_sim.geometry2d import Box, Pose2D, clearance
from rob_box_sim.nav_bench import load_scenario, timeout_for
from rob_box_sim.nav_metrics import GoalRecord, goal_metrics, sign_changes, summary_markdown
from rob_box_sim.world_sdf import load_world_boxes

REPO = Path(__file__).resolve().parents[3]
SIM = REPO / "src" / "rob_box_sim"
SCENARIOS = SIM / "scenarios" / "baseline.yaml"
WALL = Box("wall", 3.0, 0.0, 0.1, 4.0, 1.0)  # передняя грань x = 2.95


def _straight_run(x_end: float, n: int = 31, est_offset: float = 0.0, w=None) -> GoalRecord:
    rec = GoalRecord("g", Pose2D(2.0, 0.0, 0.0), "SUCCEEDED", 10.0, 25.0)
    for i in range(n):
        t = 10.0 + 15.0 * i / (n - 1)
        x = x_end * i / (n - 1)
        rec.truth.append((t, x, 0.0, 0.0))
        rec.estimate.append((x + est_offset * i / (n - 1), 0.0, 0.0))
        rec.cmd.append((t, 0.15, (w or [0.0])[i % len(w or [0.0])]))
        rec.scan_min.append((t, 2.95 - x))
    return rec


def test_sign_changes_ignore_deadband():
    assert sign_changes([0.3, 0.01, -0.02, 0.4, -0.3, 0.2]) == 2
    assert sign_changes([0.0, 0.04, -0.04]) == 0


def test_straight_run_metrics():
    m = goal_metrics(_straight_run(2.0, est_offset=0.1), [WALL])
    assert m["success"] is True
    assert m["time_s"] == 15.0
    assert m["goal_err_m"] == 0.0
    assert m["path_len_m"] == pytest.approx(2.0)
    assert m["path_ratio"] == 1.0
    assert m["pose_err_max_m"] == pytest.approx(0.1)
    assert m["pose_err_end_m"] == pytest.approx(0.1)
    assert m["min_clearance_m"] == pytest.approx(2.95 - 2.0 - 0.3064, abs=1e-3)
    assert m["collision"] is False and m["collision_s"] == 0.0
    assert m["min_scan_m"] == pytest.approx(0.95)
    assert m["osc_sign_changes"] == 0


def test_collision_detected_by_geometry_even_if_scan_blind():
    rec = _straight_run(2.9)  # нос (x+0.3064) заходит в стену x=2.95
    rec.scan_min = [(t, math.inf) for t, _ in rec.scan_min]  # лидар ничего не видит
    m = goal_metrics(rec, [WALL])
    assert m["collision"] is True
    assert m["min_clearance_m"] < 0
    assert m["collision_s"] > 0
    assert m["min_scan_m"] is None


def test_touching_obstacle_counts_as_collision():
    # физика не пускает корпус внутрь: упёрся с зазором 5 мм (< CONTACT_M)
    m = goal_metrics(_straight_run(2.95 - 0.3064 - 0.005), [WALL])
    assert m["min_clearance_m"] == pytest.approx(0.005, abs=1e-3)
    assert m["collision"] is True
    m = goal_metrics(_straight_run(2.95 - 0.3064 - 0.05), [WALL])
    assert m["collision"] is False


def test_oscillation_counted_from_cmd_vel():
    m = goal_metrics(_straight_run(2.0, w=[0.3, -0.3]), [WALL])
    assert m["osc_sign_changes"] == 30
    assert m["osc_per_m"] == pytest.approx(15.0)


def test_missing_estimate_is_none_not_zero():
    rec = _straight_run(1.0)
    rec.estimate = [None] * len(rec.truth)
    m = goal_metrics(rec, [WALL])
    assert m["pose_err_max_m"] is None and m["pose_err_end_m"] is None


def test_skipped_goal_has_no_numbers():
    m = goal_metrics(GoalRecord("g", Pose2D(1, 0, 0), "SKIPPED", 5.0, 5.0), [WALL])
    assert m["success"] is False
    assert m["min_clearance_m"] is None and m["goal_err_m"] is None
    assert m["collision"] is None and m["recoveries"] is None and m["osc_sign_changes"] is None
    assert "Успех: 0 из 0" in summary_markdown([{"world": "w", **m}])


@pytest.mark.parametrize("world", sorted(wc.CATALOG))
def test_every_catalog_world_has_scenario_with_free_goals(world):
    sc = load_scenario(SCENARIOS, world, SIM / "worlds")
    boxes = load_world_boxes(SIM / "worlds" / f"{world}.sdf")
    for g in sc["goals"]:
        assert clearance(Pose2D(g["x"], g["y"], g.get("yaw", 0.0)), boxes) > 0.05, (world, g["name"])
        assert timeout_for(sc, Pose2D(0, 0, 0), Pose2D(g["x"], g["y"])) >= sc["timeout_base_s"]


def test_polygon_scenario_goals_are_behind_each_gap():
    sc = load_scenario(SCENARIOS, "polygon", SIM / "worlds")
    xs = [g["x"] for g in sc["goals"]]
    for (gap_x, gap), goal, name in zip(wc.POLYGON_GAPS, xs, [g["name"] for g in sc["goals"]]):
        assert gap_x < goal and name == f"gap_{wc.gap_label(gap)}"


def test_local_world_goals_come_from_generator_file(tmp_path):
    (tmp_path / "local").mkdir()
    (tmp_path / "local" / "robot_map.goals.yaml").write_text(
        yaml.safe_dump({"goals": [{"name": "pose_1", "x": 1.0, "y": 2.0, "yaw": 0.0}]})
    )
    sc = load_scenario(SCENARIOS, "local/robot_map", tmp_path)
    assert sc["goals"] == [{"name": "pose_1", "x": 1.0, "y": 2.0, "yaw": 0.0}]
    assert sc["stop_on_failure"] is False


def test_raw_controller_oscillation_counted_separately_from_smoothed():
    rec = _straight_run(2.0, w=[0.0])
    rec.cmd_raw = [(t, 0.15, 0.3 if i % 2 else -0.3) for i, (t, _, _) in enumerate(rec.cmd)]
    m = goal_metrics(rec, [WALL])
    assert m["osc_sign_changes"] == 0 and m["osc_per_m"] == 0.0
    assert m["osc_raw_per_m"] == pytest.approx((len(rec.cmd) - 1) / 2.0, abs=0.01)


def test_raw_oscillation_is_none_without_smoother_chain():
    assert goal_metrics(_straight_run(2.0), [WALL])["osc_raw_per_m"] is None


def test_tilt_reported_in_degrees_and_none_without_pose():
    rec = _straight_run(2.0)
    rec.tilt_max_rad = math.radians(11.0)
    assert goal_metrics(rec, [WALL])["tilt_max_deg"] == pytest.approx(11.0)
    empty = GoalRecord("g", Pose2D(1.0, 0.0, 0.0), "SKIPPED", 0.0, 0.0)
    assert goal_metrics(empty, [WALL])["tilt_max_deg"] is None


def test_trajectory_csv_truth_and_estimate_skip_missing_tf(tmp_path):
    import csv

    from rob_box_sim.nav_bench import write_trajectories

    rec = _straight_run(2.0, n=3)
    rec.estimate[1] = None  # в этот миг TF не было
    paths = write_trajectories(rec, tmp_path, "polygon_gap_1.2")
    assert [p.name for p in paths] == ["polygon_gap_1.2_truth.csv", "polygon_gap_1.2_estimate.csv"]
    truth = list(csv.reader(paths[0].open(encoding="utf-8")))
    est = list(csv.reader(paths[1].open(encoding="utf-8")))
    assert truth[0] == est[0] == ["t", "x", "y", "yaw"]
    assert len(truth) == 4 and len(est) == 3
    assert [float(v) for v in truth[-1]] == pytest.approx([25.0, 2.0, 0.0, 0.0])
    assert write_trajectories(GoalRecord("g", Pose2D(0, 0, 0), "SKIPPED", 0, 0), tmp_path, "x") == []


def test_rtf_is_sim_time_over_wall_time():
    rec = _straight_run(2.0)  # 15 с sim time
    assert goal_metrics(rec, [WALL])["rtf"] is None  # настенное время не записано
    rec.wall_s = 30.0
    assert goal_metrics(rec, [WALL])["rtf"] == 0.5


def test_dead_end_scenario_turns_and_spins_in_place():
    sc = load_scenario(SCENARIOS, "dead_end", SIM / "worlds")
    assert sc["stop_on_failure"] is False  # spin в торце — даже если выезд не удался
    spins = [g for g in sc["goals"] if "spin" in g]
    assert len(spins) == 1 and abs(spins[0]["spin"]) == pytest.approx(math.pi, abs=1e-3)
    assert any(abs(g.get("yaw", 0.0)) == pytest.approx(math.pi, abs=1e-3) for g in sc["goals"] if "spin" not in g)
