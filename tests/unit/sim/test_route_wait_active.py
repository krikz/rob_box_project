"""route.py не шлёт цель, пока bt_navigator не active или нет TF odom→base_footprint (#4694).

Humble simple_action_server принимает wait_for_server, как только сервер создан (on_configure),
и отвергает цель без лога, пока lifecycle не active. Costmap2DROS::on_activate ждёт
canTransform(global_frame, robot_base_frame) — у local_costmap это odom ← base_footprint.
Прогон без ROS: rclpy и tf2 — заглушки.
"""

import importlib
import sys
import types
from pathlib import Path

import pytest

VIDEO_DIR = Path(__file__).resolve().parents[3] / "scripts" / "hil" / "video"
NAV2_PARAMS = Path(__file__).resolve().parents[3] / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"


@pytest.fixture
def route(monkeypatch):
    for name in (
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
        "lifecycle_msgs",
        "lifecycle_msgs.srv",
        "tf2_ros",
    ):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["rclpy"].spin_once = lambda *a, **k: None
    sys.modules["rclpy"].spin_until_future_complete = lambda *a, **k: None
    sys.modules["rclpy.action"].ActionClient = object
    sys.modules["rclpy.node"].Node = object
    sys.modules["rclpy.parameter"].Parameter = object
    sys.modules["action_msgs.msg"].GoalStatus = type("GoalStatus", (), {})
    sys.modules["geometry_msgs.msg"].Twist = object
    sys.modules["geometry_msgs.msg"].PoseStamped = object
    sys.modules["nav2_msgs.action"].NavigateToPose = object
    sys.modules["nav2_msgs.action"].NavigateThroughPoses = object
    sys.modules["nav_msgs.msg"].Odometry = object
    sys.modules["lifecycle_msgs.srv"].GetState = type("GetState", (), {"Request": lambda: object()})
    sys.modules["rclpy.duration"].Duration = lambda seconds=0.0: seconds
    sys.modules["rclpy.time"].Time = lambda: 0
    sys.modules["tf2_ros"].Buffer = object
    sys.modules["tf2_ros"].TransformListener = object
    monkeypatch.syspath_prepend(str(VIDEO_DIR))
    monkeypatch.delitem(sys.modules, "route", raising=False)
    mod = importlib.import_module("route")
    yield mod
    sys.modules.pop("route", None)


def test_tf_buffer_s_is_the_costmap_constant(route):
    """30 с — уже принятый tf_buffer_duration local_costmap, не новый таймаут и не TIMEOUT цели."""
    text = NAV2_PARAMS.read_text(encoding="utf-8")
    local = text.split("local_costmap:", 1)[1].split("global_costmap:", 1)[0]
    assert "tf_buffer_duration: 30.0" in local
    assert route.TF_BUFFER_S == 30.0
    assert route.TIMEOUT == 200.0
    assert route.ODOM_FRAME == "odom" and route.BASE_FRAME == "base_footprint"


def test_server_up_but_not_active_and_no_odom_sends_no_goal(route, monkeypatch, capsys):
    """Сервер /navigate_to_pose есть, bt_navigator не active, кадра odom нет — цель не ушла, один FAIL."""
    sent = []
    spins = []

    class Client:
        def wait_for_service(self, timeout_sec=0.0):
            return True

        def call_async(self, request):
            state = types.SimpleNamespace(label="inactive")
            return types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(current_state=state))

    class Tf:
        def can_transform(self, target, source, time, timeout):
            assert (target, source) == (route.ODOM_FRAME, route.BASE_FRAME)
            return False

    def spin_once(n, timeout_sec=0.0):
        spins.append(timeout_sec)

    clock = {"t": 0.0}

    def now():
        clock["t"] += 0.2
        return clock["t"]

    monkeypatch.setattr(route.time, "time", now)
    monkeypatch.setattr(route.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(route.rclpy, "spin_until_future_complete", lambda *a, **k: None)
    monkeypatch.setattr(route, "_send", lambda *a, **k: sent.append(a) or ("REJECTED", 0))

    with pytest.raises(SystemExit) as exc:
        route.wait_nav_active(object(), Tf(), Client(), timeout_s=1.0)

    assert exc.value.code == 1
    assert sent == []
    assert spins  # крутили ноду, но цель не слали
    err = capsys.readouterr().out
    assert err.count("FAIL") == 1
    assert "навигатор не включился" in err
    assert "REJECTED" not in err
    assert "GOAL_START" not in err


def test_active_and_odom_frame_returns_without_sending(route, monkeypatch, capsys):
    class Client:
        def wait_for_service(self, timeout_sec=0.0):
            return True

        def call_async(self, request):
            state = types.SimpleNamespace(label="active")
            return types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(current_state=state))

    class Tf:
        def can_transform(self, target, source, time, timeout):
            return (target, source) == ("odom", "base_footprint")

    monkeypatch.setattr(route.rclpy, "spin_once", lambda *a, **k: None)
    monkeypatch.setattr(route.rclpy, "spin_until_future_complete", lambda *a, **k: None)
    route.wait_nav_active(object(), Tf(), Client(), timeout_s=1.0)
    out = capsys.readouterr().out
    assert "NAV_READY" in out and "FAIL" not in out
