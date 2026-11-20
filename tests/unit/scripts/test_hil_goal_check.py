"""The HIL goal-check report uses the decision and pose stamps independently."""

import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location("hil_goal_check", ROOT / "scripts/hil/diagnostics/compare_goal_check.py")
goal_check = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(goal_check)


def test_join_gt_by_sim_stamp_and_compare_to_original_goal():
    marker = (
        "[INFO] [1795105058.692808914] [controller_server]: HIL_GOAL_CHECK_REACHED "
        "checker=general_goal_checker sim_ns=426892000000 robot_ns=426900000000 "
        "robot_frame=odom robot_x=-3.83136 robot_y=3.49448 robot_yaw=0.419526849 "
        "goal_frame=odom goal_x=-4.002149 goal_y=3.588415 goal_yaw=0.617617712"
    )
    wall, fields = goal_check.parse_trace(marker)
    assert wall == 1795105058692808914
    route_goal = goal_check.read_route_goal("16:14:19.289 GOAL_START M1 target=(-3.7,4.0,0) sim=227.50")
    gt = [
        dict(bag_id=9, stamp_ns=426900000000, x=-3.606838, y=3.830239, yaw=-0.197717),
        dict(bag_id=10, stamp_ns=427400000000, x=-3.614403, y=3.833638, yaw=-0.184382),
    ]
    result = goal_check.compare(fields, route_goal, gt, [])
    assert result["nav2_xy_error"] == pytest.approx(0.194917, abs=1e-6)
    assert result["nav2_yaw_error"] == pytest.approx(0.198090863)
    assert result["gt_at_decision_tick"]["delta_ms"] == 8
    assert result["gt_at_nav2_pose_stamp"]["delta_ms"] == 0
    assert result["gt_at_decision_tick"]["xy_error"] == pytest.approx(0.19364, abs=1e-5)
    assert result["gt_500ms_after_decision"]["yaw_error"] == pytest.approx(0.184382)


def test_stale_gt_has_no_invented_nearby_sample():
    assert goal_check.nearest_gt([{"stamp_ns": 201_000_000}], 100_000_000) is None
    assert goal_check.nearest_gt([], 100_000_000) is None
    assert goal_check.angle_error(-math.pi + 0.01, math.pi - 0.01) == pytest.approx(0.02)


ROUTE = """\
16:14:19.289 GOAL_START M1 target=(-3.7,4.0,0) sim=227.50 frame=100 gt=(0,0,0)
16:15:35.100 GOAL_END M1 result=STATUS_SUCCEEDED sim=141.52 frame=1565 dur_sim=75.8 err=0.216 em_episodes=0
16:16:01.000 GOAL_START R4 target=(1.5,-2.0,90) sim=300.00 frame=2000 gt=(1,1,0)
16:17:10.000 GOAL_END R4 result=STATUS_ABORTED sim=310.25 frame=2200 dur_sim=10.2 err=1.004 em_episodes=1
"""

FIELDS = {
    "sim_ns": "426892000000",
    "robot_ns": "426900000000",
    "robot_frame": "odom",
    "goal_frame": "odom",
    "robot_x": "-3.83136",
    "robot_y": "3.49448",
    "robot_yaw": "0.419526849",
    "goal_x": "-4.002149",
    "goal_y": "3.588415",
    "goal_yaw": "0.617617712",
}

OLD_FIELDS = (
    "nav2",
    "robot_pose_age_ms",
    "nav2_xy_error",
    "nav2_yaw_error",
    "gt_at_nav2_pose_stamp",
    "gt_at_decision_tick",
    "gt_500ms_after_decision",
    "gt_1s_after_decision",
    "drive_commands_by_receipt_near_decision",
)


def test_goal_flag_selects_r4_not_m1():
    assert goal_check.read_route_goal(ROUTE, "R4") == (1.5, -2.0, math.radians(90))
    assert goal_check.read_route_goal(ROUTE) == (-3.7, 4.0, 0.0)
    end = goal_check.read_route_goal_end(ROUTE, "R4")
    assert end["result"] == "STATUS_ABORTED"
    assert end["sim_s"] == pytest.approx(310.25)
    assert end["sim_ns"] == 310_250_000_000
    assert end["err"] == pytest.approx(1.004)
    m1 = goal_check.read_route_goal_end(ROUTE)
    assert m1["result"] == "STATUS_SUCCEEDED"
    assert m1["sim_s"] == pytest.approx(141.52)
    assert m1["err"] == pytest.approx(0.216)


def test_goal_end_line_parses_result_and_sim():
    line = (
        "16:15:35.100 GOAL_END M1 result=STATUS_SUCCEEDED sim=141.52 frame=1565 "
        "dur_sim=75.8 err=0.216 em_episodes=0 recoveries=0 gt=(1,2,3)"
    )
    end = goal_check.read_route_goal_end(line, "M1")
    assert end["result"] == "STATUS_SUCCEEDED"
    assert end["sim_s"] == pytest.approx(141.52)
    assert end["sim_ns"] == 141_520_000_000


def test_gt_after_goal_end_is_not_measured_from_decision():
    decision_ns = 100 * goal_check.SECOND_NS
    end_sim_ns = 110 * goal_check.SECOND_NS
    gt = [
        dict(stamp_ns=decision_ns + goal_check.SECOND_NS // 2, x=1.0, y=0.0, yaw=0.0),
        dict(stamp_ns=decision_ns + goal_check.SECOND_NS, x=2.0, y=0.0, yaw=0.0),
        dict(stamp_ns=end_sim_ns + goal_check.SECOND_NS // 2, x=9.0, y=0.0, yaw=0.1),
        dict(stamp_ns=end_sim_ns + (3 * goal_check.SECOND_NS) // 2, x=11.0, y=0.0, yaw=0.2),
    ]
    goal_end = {"result": "STATUS_SUCCEEDED", "sim_s": 110.0, "sim_ns": end_sim_ns, "err": 0.05}
    result = goal_check.compare(FIELDS | {"sim_ns": str(decision_ns)}, (0.0, 0.0, 0.0), gt, [], goal_end, 1_000)
    assert result["gt_500ms_after_decision"]["x"] == pytest.approx(1.0)
    assert result["gt_500ms_after_goal_end"]["x"] == pytest.approx(9.0)
    assert result["gt_1500ms_after_goal_end"]["x"] == pytest.approx(11.0)
    assert result["gt_500ms_after_goal_end"]["stamp_ns"] != result["gt_500ms_after_decision"]["stamp_ns"]
    assert result["goal_end_wall_ns"] == 1_000 + (end_sim_ns - decision_ns)
    assert result["goal_end_wall_precision_ms"] == 10
    assert result["goal_end_result"] == "STATUS_SUCCEEDED"
    assert result["goal_end_err"] == pytest.approx(0.05)


def test_gt_after_goal_end_outside_100ms_is_null():
    decision_ns = 100 * goal_check.SECOND_NS
    end_sim_ns = 110 * goal_check.SECOND_NS
    gt = [dict(stamp_ns=end_sim_ns + goal_check.SECOND_NS // 2 + 101_000_000, x=9.0, y=0.0, yaw=0.0)]
    goal_end = {"result": "STATUS_SUCCEEDED", "sim_s": 110.0, "sim_ns": end_sim_ns, "err": 0.0}
    result = goal_check.compare(FIELDS | {"sim_ns": str(decision_ns)}, (0.0, 0.0, 0.0), gt, [], goal_end, 1_000)
    assert result["gt_500ms_after_goal_end"] is None
    assert result["gt_1500ms_after_goal_end"] is None


def test_missing_goal_end_leaves_null_fields():
    route = "16:14:19.289 GOAL_START M1 target=(-3.7,4.0,0) sim=227.50\n"
    assert goal_check.read_route_goal_end(route, "M1") is None
    result = goal_check.compare(FIELDS, goal_check.read_route_goal(route), [], [], None, 1_000)
    assert result["goal_end_sim"] is None
    assert result["goal_end_result"] is None
    assert result["goal_end_err"] is None
    assert result["goal_end_wall_ns"] is None
    assert result["gt_500ms_after_goal_end"] is None
    assert result["gt_1500ms_after_goal_end"] is None
    for name in OLD_FIELDS:
        assert name in result


def test_default_goal_keeps_m1_and_old_fields(capsys, monkeypatch):
    goal = goal_check.read_route_goal(ROUTE)
    assert goal[0] == pytest.approx(-3.7)
    end = goal_check.read_route_goal_end(ROUTE)
    result = goal_check.compare(FIELDS, goal, [], [], end, 5_000)
    for name in OLD_FIELDS:
        assert name in result
    assert result["goal_end_result"] == "STATUS_SUCCEEDED"
    assert result["goal_end_sim"] == pytest.approx(141.52)
    seen = {}

    def fake_load(artifact, goal_name):
        seen["goal"] = goal_name
        return goal, end, [], []

    monkeypatch.setattr(goal_check, "_load_run", fake_load)
    with pytest.raises(SystemExit) as caught:
        goal_check.main(["/run"])
    assert caught.value.code == 2
    assert seen["goal"] == "M1"
    assert "goal_check_reached.log" in capsys.readouterr().err
    assert goal_check.gt_query_bounds(10 * goal_check.SECOND_NS, 12 * goal_check.SECOND_NS) == (
        6 * goal_check.SECOND_NS,
        18 * goal_check.SECOND_NS,
    )
