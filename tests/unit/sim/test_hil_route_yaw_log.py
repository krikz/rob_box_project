"""Лог route.py: YAW/PLAN/MAP появляются из подписок; GOALS и TIMEOUT не менялись."""

import importlib
import math
import sys
import types
from pathlib import Path

import pytest

VIDEO_DIR = Path(__file__).resolve().parents[3] / "scripts" / "hil" / "video"
ROS_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.duration",
    "rclpy.time",
    "action_msgs",
    "action_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "nav_msgs",
    "nav_msgs.msg",
)


class FakeNode:
    def __init__(self, name="shidi_route", parameter_overrides=None):
        self._subs = []
        self._clock_ns = 12_500_000_000

    def create_subscription(self, typ, topic, cb, qos):
        self._subs.append((topic, cb))
        return None

    def create_timer(self, period, cb):
        return None

    def get_clock(self):
        node = self
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=node._clock_ns))


class FakeParameter:
    class Type:
        BOOL = 1

    def __init__(self, *args, **kwargs):
        pass


def _tf(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    return types.SimpleNamespace(
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y),
            rotation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        )
    )


@pytest.fixture
def route_mod(monkeypatch):
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["rclpy.action"].ActionClient = object
    sys.modules["rclpy.node"].Node = FakeNode
    sys.modules["rclpy.parameter"].Parameter = FakeParameter
    sys.modules["rclpy.duration"].Duration = lambda *a, **k: None
    sys.modules["rclpy.time"].Time = lambda *a, **k: None
    sys.modules["action_msgs.msg"].GoalStatus = type("GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_ABORTED": 6})
    sys.modules["geometry_msgs.msg"].Twist = object
    sys.modules["geometry_msgs.msg"].PoseStamped = object
    sys.modules["nav2_msgs.action"].NavigateToPose = object
    sys.modules["nav2_msgs.action"].NavigateThroughPoses = object
    sys.modules["nav_msgs.msg"].Path = object
    sys.modules["nav_msgs.msg"].Odometry = object
    monkeypatch.syspath_prepend(str(VIDEO_DIR))
    monkeypatch.delitem(sys.modules, "route", raising=False)
    mod = importlib.import_module("route")
    yield mod
    sys.modules.pop("route", None)


def test_mon_subscribes_cmd_vel_yaw_and_plan(route_mod):
    n = route_mod.Mon()
    topics = [topic for topic, _cb in n._subs]
    assert route_mod.YAW_TOPIC == "/cmd_vel_yaw"
    assert route_mod.PLAN_TOPIC == "/plan"
    assert "/cmd_vel_yaw" in topics
    assert "/plan" in topics


def test_tick_writes_yaw_plan_map_when_subscriptions_have_messages(route_mod, capsys):
    n = route_mod.Mon()
    n.on_yaw(types.SimpleNamespace(angular=types.SimpleNamespace(z=-0.42)))
    n.on_plan(
        types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=250_000_000)))
    )
    n.gt = (-3.546, 2.159, 147.1)
    n.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **k: _tf(-3.700, 2.340, 90.0))
    n._clock_ns = 14_700_000_000
    n.tick()
    out = capsys.readouterr().out
    assert "YAW " in out and "wz=-0.420" in out and "age=2.20" in out
    assert "PLAN " in out and "stamp=10.250" in out and "now=14.700" in out
    assert "MAP " in out and "x=-3.700" in out and "y=2.340" in out and "yaw=90.0" in out
    assert "gt_x=-3.546" in out and "gt_y=2.159" in out and "gt_yaw=147.1" in out
    assert "TRACK " in out


def test_goals_and_timeout_unchanged(route_mod):
    assert route_mod.TIMEOUT == 200.0
    by = {g[0]: g for g in route_mod.GOALS}
    assert by["M1"] == ("M1", -3.7, 4.0, 0)
    assert by["M4"] == ("M4", -3.7, -4.5, 0)
    assert by["R3"] == ("R3", -3.7, 2.34, 90)
    assert by["R7"] == ("R7", -3.7, 0.0, 180)
    assert (by["M1"][1], by["M1"][2]) != (-6.0, 4.0)
    assert (by["R3"][1], by["R3"][2]) != (-9.0, 4.0)
    assert (by["R7"][1], by["R7"][2]) != (-10.5, 0.0)
