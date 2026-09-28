"""core/nav_goal.py (issue #3151): разбор nav_goal, статусы, nav_status."""

from __future__ import annotations

import math

import pytest

from rob_box_quest.core.nav_goal import (
    NACK_BAD_FRAME,
    NACK_BAD_PAYLOAD,
    NavGoalRequest,
    RateGate,
    nav_status_event,
    normalize_angle,
    parse_nav_goal,
    state_from_goal_status,
    yaw_to_quaternion_zw,
)


def _payload(**over):
    base = {"cmd": "nav_goal", "ts_ms": 1, "seq": 7, "x": 1.0, "y": -2.0, "yaw": 0.5, "frame": "map"}
    base.update(over)
    return base


def test_parse_valid_goal():
    assert parse_nav_goal(_payload()) == NavGoalRequest(seq=7, x=1.0, y=-2.0, yaw=0.5)


def test_parse_accepts_int_coordinates():
    req = parse_nav_goal(_payload(x=3, y=0, yaw=0))
    assert isinstance(req, NavGoalRequest)
    assert (req.x, req.y) == (3.0, 0.0)


@pytest.mark.parametrize("frame", ["odom", "base_link", None, ""])
def test_parse_rejects_non_map_frame(frame):
    assert parse_nav_goal(_payload(frame=frame)) == NACK_BAD_FRAME


@pytest.mark.parametrize(
    "over",
    [
        {"x": float("nan")},
        {"y": float("inf")},
        {"yaw": "1.0"},
        {"x": True},
        {"x": None},
        {"seq": "7"},
        {"seq": True},
        {"seq": None},
    ],
)
def test_parse_rejects_bad_fields(over):
    assert parse_nav_goal(_payload(**over)) == NACK_BAD_PAYLOAD


def test_parse_normalizes_yaw():
    req = parse_nav_goal(_payload(yaw=3 * math.pi / 2))
    assert req.yaw == pytest.approx(-math.pi / 2)


def test_normalize_angle_range():
    assert normalize_angle(-math.pi) == pytest.approx(math.pi)
    assert normalize_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


@pytest.mark.parametrize(
    "code,state",
    [(1, "accepted"), (2, "active"), (3, "active"), (4, "succeeded"), (5, "canceled"), (6, "aborted"), (0, None), (99, None)],
)
def test_goal_status_table(code, state):
    assert state_from_goal_status(code) == state


def test_quaternion_matches_yaw():
    qz, qw = yaw_to_quaternion_zw(math.pi / 2)
    assert math.atan2(2 * qw * qz, 1 - 2 * qz * qz) == pytest.approx(math.pi / 2)


def test_nav_status_optional_fields_only_when_present():
    req = NavGoalRequest(seq=3, x=1.0, y=2.0, yaw=0.0)
    ev = nav_status_event(req, "accepted", ts_ms=5)
    assert ev == {"type": "nav_status", "state": "accepted", "seq": 3, "x": 1.0, "y": 2.0, "yaw": 0.0, "ts_ms": 5}
    ev2 = nav_status_event(req, "active", ts_ms=6, distance_remaining=1.25, reason="x")
    assert ev2["distance_remaining"] == 1.25
    assert ev2["reason"] == "x"
    assert "distance_remaining" not in nav_status_event(req, "active", ts_ms=6, distance_remaining=float("nan"))


def test_rate_gate():
    g = RateGate(0.5)
    assert g.admit(1.0)
    assert not g.admit(1.2)
    assert g.admit(1.6)
    g.reset()
    assert g.admit(1.61)
