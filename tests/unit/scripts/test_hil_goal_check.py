"""The HIL goal-check report uses the decision and pose stamps independently."""

import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "hil_goal_check", ROOT / "scripts/hil/diagnostics/compare_goal_check.py"
)
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
