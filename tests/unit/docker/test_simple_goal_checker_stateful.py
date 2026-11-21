"""Humble 1.1.20 SimpleGoalChecker latches xy and can succeed outside 0.25 m.

Pinned source (docker/main/nav2/Dockerfile commit a0970867, apt 1.1.20):
nav2_controller/plugins/simple_goal_checker.cpp isGoalReached. check_xy_ is set
false inside xy when stateful; leaving xy does not set it true. reset() runs
only from controller_server setPlannerPath (new path), not on xy exit.

Humble declares xy_goal_tolerance, yaw_goal_tolerance, stateful — not
xy_goal_tolerance_buffer / path_length_tolerance (those are later Nav2).

This file does not run Nav2. It locks the yaml and a model of that C++.
HIL is not claimed.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

XY_TOL = 0.25
YAW_TOL = 0.22
# depot4694_m1yaw_20261118T0239Z: entered 0.25 m ~0.6 s, then 0.486 m, yaw 10.7°.
M1_OUT_M = 0.486
M1_YAW_RAD = math.radians(10.7)


class HumbleSimpleGoalChecker:
    """Model of Humble 1.1.20 isGoalReached / reset. Not a live plugin."""

    def __init__(self, *, xy: float = XY_TOL, yaw: float = YAW_TOL, stateful: bool) -> None:
        self._xy_sq = xy * xy
        self._yaw = yaw
        self._stateful = stateful
        self.check_xy = True

    def reset(self) -> None:
        self.check_xy = True

    def is_goal_reached(
        self,
        query_xy: tuple[float, float],
        query_yaw: float,
        goal_xy: tuple[float, float] = (0.0, 0.0),
        goal_yaw: float = 0.0,
    ) -> bool:
        if self.check_xy:
            dx = query_xy[0] - goal_xy[0]
            dy = query_xy[1] - goal_xy[1]
            if dx * dx + dy * dy > self._xy_sq:
                return False
            if self._stateful:
                self.check_xy = False
        dyaw = math.atan2(math.sin(query_yaw - goal_yaw), math.cos(query_yaw - goal_yaw))
        return abs(dyaw) < self._yaw


def _checker() -> dict:
    doc = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    return doc["controller_server"]["ros__parameters"]


def _m1_ticks(stateful: bool) -> tuple[bool, bool]:
    """Inside xy with yaw still out, then 0.486 m with yaw inside tolerance."""
    gc = HumbleSimpleGoalChecker(stateful=stateful)
    inside_bad_yaw = gc.is_goal_reached((0.24, 0.0), 0.50)
    outside_good_yaw = gc.is_goal_reached((M1_OUT_M, 0.0), M1_YAW_RAD)
    return inside_bad_yaw, outside_good_yaw


def test_humble_stateful_succeeds_after_leaving_xy():
    """M1 hole: latch xy on the 0.25 m visit, then succeed at 0.486 m on yaw."""
    assert M1_YAW_RAD < YAW_TOL
    assert M1_OUT_M > XY_TOL
    inside_bad_yaw, outside_good_yaw = _m1_ticks(stateful=True)
    assert inside_bad_yaw is False
    assert outside_good_yaw is True


def test_humble_stateless_rejects_after_leaving_xy():
    inside_bad_yaw, outside_good_yaw = _m1_ticks(stateful=False)
    assert inside_bad_yaw is False
    assert outside_good_yaw is False


def test_humble_stateless_still_succeeds_when_xy_and_yaw_together():
    gc = HumbleSimpleGoalChecker(stateful=False)
    assert gc.is_goal_reached((0.24, 0.0), M1_YAW_RAD) is True
    assert gc.is_goal_reached((0.24, 0.0), 0.50) is False


def test_humble_reset_only_unlatches_xy():
    gc = HumbleSimpleGoalChecker(stateful=True)
    assert gc.is_goal_reached((0.24, 0.0), M1_YAW_RAD) is True
    gc.reset()
    assert gc.check_xy is True
    assert gc.is_goal_reached((M1_OUT_M, 0.0), M1_YAW_RAD) is False


def test_yaml_general_goal_checker_is_stateless_with_expected_tolerances():
    cs = _checker()
    checker = cs["general_goal_checker"]
    follow = cs["FollowPath"]
    assert checker["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == XY_TOL
    assert checker["yaw_goal_tolerance"] == YAW_TOL
    assert "xy_goal_tolerance_buffer" not in checker
    assert "path_length_tolerance" not in checker
    assert follow["xy_goal_tolerance"] == XY_TOL
    assert follow["PathAlign.forward_point_distance"] == 0.20
    # DWB block, not the goal checker. Do not flip every stateful key.
    assert follow["stateful"] is True
