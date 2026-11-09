"""Исполнитель обхода scripts/hil/video/coverage_run.py (#4560) без ROS: модули rclpy/сообщений — заглушки.

Проверяется решение кода: недостижимые позы пропускаются и считаются, после ``max_fail`` неудач подряд
обход останавливается, оставшиеся позы — NOT_SENT; итоговая строка строится из этих исходов.
"""

import importlib
import sys
import types
from pathlib import Path

import pytest
from rob_box_sim.coverage_plan import CoveragePose

VIDEO_DIR = Path(__file__).resolve().parents[3] / "scripts" / "hil" / "video"
ROS_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.node",
    "rclpy.parameter",
    "action_msgs",
    "action_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "nav_msgs",
    "nav_msgs.msg",
)


@pytest.fixture
def coverage_run(monkeypatch):
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["rclpy.action"].ActionClient = object
    sys.modules["rclpy.node"].Node = object
    sys.modules["rclpy.parameter"].Parameter = object
    sys.modules["action_msgs.msg"].GoalStatus = type("GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_ABORTED": 6})
    sys.modules["geometry_msgs.msg"].Twist = object
    sys.modules["nav2_msgs.action"].NavigateToPose = object
    sys.modules["nav_msgs.msg"].Odometry = object
    monkeypatch.syspath_prepend(str(VIDEO_DIR))
    for name in ("route", "coverage_run"):
        monkeypatch.delitem(sys.modules, name, raising=False)
    mod = importlib.import_module("coverage_run")
    yield mod
    for name in ("route", "coverage_run"):
        sys.modules.pop(name, None)


def _poses(n):
    return [CoveragePose(float(k), 0.0, 0.0, 0, k // 2, "start" if k % 2 == 0 else "end") for k in range(n)]


def test_drive_skips_failures_and_stops_after_max_fail(coverage_run, monkeypatch):
    results = iter(["SUCCEEDED", "ABORTED", "SUCCEEDED", "TIMEOUT", "REJECTED", "ABORTED"])
    sent = []

    def fake_run_goal(n, cl, name, x, y, yaw, timeout):
        sent.append((name, x, yaw, timeout))
        return next(results), 1 if name.startswith("p0001") else 0

    monkeypatch.setattr(coverage_run, "run_goal", fake_run_goal)
    outcomes, stop = coverage_run.drive(None, None, _poses(8), timeout=42.0, max_fail=3)
    assert [o.result for o in outcomes] == [
        "SUCCEEDED", "ABORTED", "SUCCEEDED", "TIMEOUT", "REJECTED", "ABORTED", "NOT_SENT", "NOT_SENT"
    ]  # fmt: skip
    assert stop == "3_failures_in_row"
    assert len(sent) == 6 and sent[0] == ("p0000_c0l0s", 0.0, 0.0, 42.0)
    assert outcomes[1].em_episodes == 1


def test_drive_runs_whole_plan_when_failures_are_sparse(coverage_run, monkeypatch):
    monkeypatch.setattr(
        coverage_run, "run_goal", lambda *a: ("ABORTED" if a[2].startswith("p0002") else "SUCCEEDED", 0)
    )
    outcomes, stop = coverage_run.drive(None, None, _poses(5), timeout=1.0, max_fail=2)
    assert stop == ""
    assert [o.result for o in outcomes].count("SUCCEEDED") == 4


def test_moved_thins_track(coverage_run):
    a = (0.0, 0.0, 0.0, 0.0)
    assert not coverage_run._moved(a, (0.1, 0.01, 0.0, 0.0))
    assert coverage_run._moved(a, (0.1, 0.03, 0.0, 0.0))
    assert coverage_run._moved(a, (0.1, 0.0, 0.0, 0.05))
    assert coverage_run._moved(a, (1.5, 0.0, 0.0, 0.0))
