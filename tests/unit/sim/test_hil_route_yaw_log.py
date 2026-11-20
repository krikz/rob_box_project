"""Лог route.py: YAW/CMD/PLAN/MAP появляются из подписок; GOALS и TIMEOUT не менялись."""

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


def _pose(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        )
    )


def _plan_line(out):
    return next(line for line in out.splitlines() if " PLAN " in line)


def _cmd_line(out):
    return next(line for line in out.splitlines() if " CMD " in line)


@pytest.fixture
def route_mod(monkeypatch):
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["rclpy"].spin_until_future_complete = lambda *a, **k: None
    sys.modules["rclpy"].spin_once = lambda *a, **k: None
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
    assert route_mod.CMD_TOPIC == "/cmd_vel"
    assert route_mod.PLAN_TOPIC == "/plan"
    assert "/cmd_vel_yaw" in topics
    assert "/cmd_vel" in topics
    assert "/plan" in topics


def test_tick_writes_yaw_plan_map_when_subscriptions_have_messages(route_mod, capsys):
    n = route_mod.Mon()
    n.on_yaw(types.SimpleNamespace(angular=types.SimpleNamespace(z=-0.42)))
    n.on_cmd(
        types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.15),
            angular=types.SimpleNamespace(z=-0.08),
        )
    )
    n.on_plan(
        types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=250_000_000)))
    )
    n.gt = (-3.546, 2.159, 147.1)
    n.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **k: _tf(-3.700, 2.340, 90.0))
    n._clock_ns = 14_700_000_000
    n.tick()
    out = capsys.readouterr().out
    assert "YAW " in out and "wz=-0.420" in out and "age=2.20" in out
    cmd_line = _cmd_line(out)
    assert "vx=0.150" in cmd_line and "wz=-0.080" in cmd_line and "age=2.20" in cmd_line
    plan_line = _plan_line(out)
    assert "stamp=10.250" in plan_line and "now=14.700" in plan_line
    assert "x=none" in plan_line and "y=none" in plan_line and "yaw=none" in plan_line
    assert "MAP " in out and "x=-3.700" in out and "y=2.340" in out and "yaw=90.0" in out
    assert "gt_x=-3.546" in out and "gt_y=2.159" in out and "gt_yaw=147.1" in out
    assert "TRACK " in out


def test_cmd_line_none_when_no_cmd_vel(route_mod, capsys):
    """Нет /cmd_vel - vx=none wz=none, не выдуманный ноль."""
    n = route_mod.Mon()
    n.tick()
    line = _cmd_line(capsys.readouterr().out)
    assert "vx=none" in line and "wz=none" in line and "age=none" in line


def test_cmd_line_format_when_message(route_mod, capsys):
    """Строка CMD: linear.x, angular.z и age по образцу YAW."""
    n = route_mod.Mon()
    n.on_cmd(
        types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0),
            angular=types.SimpleNamespace(z=0.31),
        )
    )
    n._clock_ns = 13_000_000_000
    n.tick()
    line = _cmd_line(capsys.readouterr().out)
    assert "CMD " in line
    assert "vx=0.000" in line and "wz=0.310" in line and "age=0.50" in line


def test_plan_line_contains_single_pose_xy(route_mod, capsys):
    """Одна поза в /plan — строка PLAN несёт её x,y; без новой подписки."""
    n = route_mod.Mon()
    n.on_plan(
        types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=0)),
            poses=[_pose(-3.214, 3.501, 12.0)],
        )
    )
    n.tick()
    line = _plan_line(capsys.readouterr().out)
    assert "x=-3.214" in line and "y=3.501" in line
    assert "yaw=12.0" in line
    assert "stamp=10.000" in line


def test_plan_line_none_when_no_path(route_mod, capsys):
    """Нет /plan — none, не выдуманная точка."""
    n = route_mod.Mon()
    n.tick()
    line = _plan_line(capsys.readouterr().out)
    assert "stamp=none" in line
    assert "x=none" in line and "y=none" in line and "yaw=none" in line


def test_accept_point_uses_shifu_plank_not_goal_checker(route_mod):
    """Планка приёмки 0.32/0.35 (d-20261120-depot-gt-032-035), не допуск goal checker 0.25/0.30."""
    assert route_mod.ACCEPT_XY_TOLERANCE == pytest.approx(0.32)
    assert route_mod.ACCEPT_YAW_TOLERANCE == pytest.approx(0.35)
    assert route_mod.XY_GOAL_TOLERANCE == 0.25  # goal checker не сдвинут
    assert route_mod.YAW_GOAL_TOLERANCE == pytest.approx(0.30)

    # (a) 0.274 м / 0.179 рад, не таймаут: checker сказал бы OUTSIDE_TOLERANCE, приёмка — PASS.
    verdict, reasons = route_mod.accept_point("OUTSIDE_TOLERANCE", 0.274, 0.179, 40.0)
    assert (verdict, reasons) == ("PASS", [])

    # (b) 0.313 м внутри 0.32, 0.380 рад за 0.35 — провал только по курсу.
    verdict, reasons = route_mod.accept_point("OUTSIDE_TOLERANCE", 0.313, 0.380, 40.0)
    assert verdict == "FAIL"
    assert reasons == ["yaw"]

    # (c) TIMEOUT при 0.24 м / 0.21 рад: таймаут не переименовываем, точка не PASS.
    verdict, reasons = route_mod.accept_point("TIMEOUT", 0.24, 0.21, 200.1)
    assert verdict == "FAIL"
    assert reasons == ["timeout"]
    assert route_mod.hil_result("TIMEOUT", (-3.5, -4.5, 0.0), (-3.5, -4.5, 0.0)) == "TIMEOUT"


def test_acceptance_line_names_failed_points_and_why(route_mod):
    """Итог маршрута: N/M PASS и каждая FAIL-точка с причиной."""
    rows = [
        ("M1", "PASS", []),
        ("M3", "FAIL", ["xy"]),
        ("M4", "FAIL", ["timeout"]),
        ("R4", "FAIL", ["yaw"]),
    ]
    line = route_mod.acceptance_line(rows)
    assert line.startswith("ПРИЁМКА (0.32 м / 0.35 рад): 1/4 PASS")
    assert "M3 FAIL (xy)" in line
    assert "M4 FAIL (timeout)" in line
    assert "R4 FAIL (yaw)" in line
    assert "M1 FAIL" not in line


def test_goal_end_keeps_old_fields_and_appends_accept(route_mod, monkeypatch, capsys):
    """Поле accept= в конце GOAL_END; старые поля и их порядок не тронуты."""
    n = route_mod.Mon()
    n.gt = (-3.50, -4.26, 0.0)  # 0.24 м, курс 0
    n.simt = lambda: 10.0
    monkeypatch.setattr(route_mod, "nframes", lambda: 7)
    monkeypatch.setattr(route_mod, "ts", lambda: "00:00:00.000")

    class Client:
        def send_goal_async(self, goal, feedback_callback=None):
            return types.SimpleNamespace(result=lambda: types.SimpleNamespace(accepted=True))

    monkeypatch.setattr(route_mod, "_wait_result", lambda *a, **k: "STATUS_SUCCEEDED")
    route_mod._send(n, Client(), "M4", object(), "(-3.5,-4.5,0)", (-3.5, -4.5, 0.0), 200.0, None)
    line = next(ln for ln in capsys.readouterr().out.splitlines() if "GOAL_END" in ln)
    assert "result=STATUS_SUCCEEDED" in line  # 0.24 м ≤ допуска checker 0.25; приёмка тоже PASS
    assert line.index("result=") < line.index("accept=")
    assert "accept=PASS" in line
    assert "why=-" in line
    for field in ("sim=", "frame=", "dur_sim=", "err=", "em_episodes=", "recoveries=", "gt="):
        assert field in line


def test_accept_pass_only_on_nav2_success(route_mod):
    """PASS только при успехе Nav2. REJECTED/INTERRUPTED — FAIL, xy/yaw не спасают."""
    verdict, reasons = route_mod.accept_point("REJECTED", 0.10, 0.10, 40.0)
    assert (verdict, reasons) == ("FAIL", ["not_succeeded"])

    verdict, reasons = route_mod.accept_point("INTERRUPTED", 0.10, 0.10, 40.0)
    assert (verdict, reasons) == ("FAIL", ["not_succeeded"])

    verdict, reasons = route_mod.accept_point("STATUS_ABORTED", 0.10, 0.10, 40.0)
    assert (verdict, reasons) == ("FAIL", ["not_succeeded"])

    verdict, reasons = route_mod.accept_point("CANCELED", 0.40, 0.40, 40.0)
    assert (verdict, reasons) == ("FAIL", ["not_succeeded", "xy", "yaw"])

    # STATUS_SUCCEEDED 0.31 м / 0.34 рад — внутри 0.32/0.35, вне чекера 0.25/0.30.
    verdict, reasons = route_mod.accept_point("STATUS_SUCCEEDED", 0.31, 0.34, 40.0)
    assert (verdict, reasons) == ("PASS", [])

    verdict, reasons = route_mod.accept_point("STATUS_SUCCEEDED", 0.33, 0.34, 40.0)
    assert (verdict, reasons) == ("FAIL", ["xy"])

    verdict, reasons = route_mod.accept_point("OUTSIDE_TOLERANCE", 0.31, 0.36, 40.0)
    assert (verdict, reasons) == ("FAIL", ["yaw"])

    # Ровно на планке — ещё PASS; сразу за ней — FAIL.
    assert route_mod.accept_point("STATUS_SUCCEEDED", 0.32, 0.35, 200.0) == ("PASS", [])
    assert route_mod.accept_point("STATUS_SUCCEEDED", 0.32, 0.35, 200.1)[1] == ["timeout"]


def test_goals_and_timeout_unchanged(route_mod):
    assert route_mod.TIMEOUT == 200.0
    by = {g[0]: g for g in route_mod.GOALS}
    assert by["M1"] == ("M1", -3.7, 4.0, 0)
    assert by["M4"] == ("M4", -3.5, -4.5, 0)
    assert by["R3"] == ("R3", -3.7, 2.34, 90)
    assert by["R7"] == ("R7", -3.7, 0.0, 180)
    assert (by["M1"][1], by["M1"][2]) != (-6.0, 4.0)
    assert (by["R3"][1], by["R3"][2]) != (-9.0, 4.0)
    assert (by["R7"][1], by["R7"][2]) != (-10.5, 0.0)
