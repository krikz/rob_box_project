"""Исполнитель обхода scripts/hil/video/coverage_run.py (#4560) без ROS: модули rclpy/сообщений — заглушки.

Проверяется решение кода: недостижимые позы пропускаются и считаются, после ``max_fail`` неудач подряд
обход останавливается, оставшиеся позы — NOT_SENT; итоговая строка строится из этих исходов.
"""

import importlib
import signal
import sys
import types
from pathlib import Path

import pytest
from rob_box_sim.coverage_plan import CoveragePose, coverage_space
from rob_box_sim.map_check import OccupancyGrid

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
    sys.modules["geometry_msgs.msg"].PoseStamped = object
    sys.modules["nav2_msgs.action"].NavigateToPose = object
    sys.modules["nav2_msgs.action"].NavigateThroughPoses = object
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

    def fake_run_goal(n, cl, name, x, y, yaw, timeout, stop=None):
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
        coverage_run, "run_goal", lambda *a, **kw: ("ABORTED" if a[2].startswith("p0002") else "SUCCEEDED", 0)
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


def test_drive_counts_real_route_statuses(coverage_run, monkeypatch):
    """HIL Depot 09.11: run_goal отдаёт имена GoalStatus (STATUS_SUCCEEDED), а сравнение было с SUCCEEDED —
    20 успехов подряд засчитались неудачами и обход встал по 20_failures_in_row."""
    results = iter(["STATUS_SUCCEEDED"] * 4 + ["STATUS_ABORTED", "TIMEOUT", "STATUS_SUCCEEDED"])
    monkeypatch.setattr(coverage_run, "run_goal", lambda *a, **kw: (next(results), 0))
    outcomes, stop = coverage_run.drive(None, None, _poses(7), timeout=1.0, max_fail=3)
    assert stop == ""
    assert [o.result for o in outcomes] == ["SUCCEEDED"] * 4 + ["ABORTED", "TIMEOUT", "SUCCEEDED"]
    assert sum(o.succeeded for o in outcomes) == 5


def test_route_status_names_are_goal_status_names(coverage_run):
    """Исход run_goal для завершённой цели — имя из GoalStatus; нормализация снимает ровно этот префикс."""
    route = sys.modules["route"]
    assert route.STATUS[4] == "STATUS_SUCCEEDED" and route.STATUS[6] == "STATUS_ABORTED"
    assert coverage_run.GoalOutcome.from_route("g", route.STATUS[4]).result == "SUCCEEDED"


def test_signal_cancels_current_goal_and_marks_the_rest_not_sent(coverage_run, monkeypatch):
    stop = coverage_run.StopSignal()
    asked = []

    def fake_run_goal(n, cl, name, x, y, yaw, timeout, stop=None):
        asked.append(stop())
        if name.startswith("p0001"):
            sig(signal.SIGINT)  # оператор нажал Ctrl+C посреди второй цели
            assert stop(), "run_goal должен видеть флаг и отменить цель"
            return "INTERRUPTED", 0
        return "STATUS_SUCCEEDED", 0

    sig = stop
    monkeypatch.setattr(coverage_run, "run_goal", fake_run_goal)
    outcomes, why = coverage_run.drive(None, None, _poses(5), timeout=1.0, max_fail=3, stop=stop)
    assert why == "interrupted_SIGINT"
    assert [o.result for o in outcomes] == ["SUCCEEDED", "INTERRUPTED", "NOT_SENT", "NOT_SENT", "NOT_SENT"]
    assert asked == [False, False]


def test_second_signal_exits_at_once(coverage_run):
    stop = coverage_run.StopSignal()
    stop(signal.SIGTERM)
    assert stop.name == "SIGTERM"
    with pytest.raises(KeyboardInterrupt):
        stop(signal.SIGINT)


def test_signal_before_first_goal_sends_nothing(coverage_run, monkeypatch):
    stop = coverage_run.StopSignal()
    stop(signal.SIGTERM)
    monkeypatch.setattr(coverage_run, "run_goal", lambda *a, **kw: pytest.fail("цель после сигнала"))
    outcomes, why = coverage_run.drive(None, None, _poses(3), timeout=1.0, max_fail=3, stop=stop)
    assert why == "interrupted_SIGTERM" and {o.result for o in outcomes} == {"NOT_SENT"}


def test_finish_writes_summary_for_what_was_driven(coverage_run, tmp_path):
    grid = OccupancyGrid(80, 60, 0.05, 0.0, 0.0, [0] * 4800)
    space = coverage_space(grid, (1.0, 1.0))
    node = types.SimpleNamespace(track=[(0.0, 1.0, 1.0, 0.0), (1.0, 1.5, 1.0, 0.0)], simt=lambda: 12.5)
    outcomes = [coverage_run.GoalOutcome("p0000", "SUCCEEDED"), coverage_run.GoalOutcome("p0001", "INTERRUPTED")]
    line = coverage_run.finish(tmp_path / "cov", node, grid, space, outcomes, "interrupted_SIGINT", 2.5)
    assert "COVERAGE_DONE goals=2 succeeded=1 skipped=1 (INTERRUPTED=1)" in line
    assert "stop=interrupted_SIGINT interrupted=yes" in line and "sim_s=10.0" in line
    assert "coverage_free=" in line
    assert (tmp_path / "cov" / "track.json").exists()
    assert any(p.name.startswith("coverage.") for p in (tmp_path / "cov").iterdir())


def test_plan_knobs_reach_the_planner(coverage_run):
    args = coverage_run.parse_args(["--turn-clearance", "0.69", "--no-cross", "--plan-only"])
    assert args.turn_clearance == 0.69 and args.no_cross and args.plan_only
    assert coverage_run.parse_args([]).turn_clearance is None  # умолчание — из coverage_plan


# ── полоса одной целью NavigateThroughPoses (попытка 3 на Depot: 0.06 м/с позами по одной) ────────────


def _lanes(*sizes):
    """Позы нескольких полос: размеры полос → CoveragePose подряд, x растёт на 1 м."""
    out, x = [], 0.0
    for lane, n in enumerate(sizes):
        for k in range(n):
            kind = "start" if k == 0 else "end" if k == n - 1 else "mid"
            out.append(CoveragePose(x, float(lane), 0.0, 0, lane, kind))
            x += 1.0
    return out


def test_lanes_of_groups_consecutive_poses_of_one_lane(coverage_run):
    groups = coverage_run.lanes_of(_lanes(2, 3, 2))
    assert [len(g) for g in groups] == [2, 3, 2]
    assert {(p.cell, p.lane) for p in groups[1]} == {(0, 1)}


def test_drive_sends_each_lane_as_one_goal_and_counts_poses_honestly(coverage_run, monkeypatch):
    sent = []
    # настоящие исходы route: имя GoalStatus, TIMEOUT; третье — сколько точек осталось по feedback Nav2
    results = iter(
        [("STATUS_SUCCEEDED", 0, 1), ("STATUS_ABORTED", 2, 2), ("TIMEOUT", 0, 3), ("STATUS_SUCCEEDED", 1, 1)]
    )

    def fake_run_lane(n, cl, name, poses, timeout, stop=None):
        sent.append((name, len(poses), round(timeout, 3)))
        return next(results)

    monkeypatch.setattr(coverage_run, "run_lane", fake_run_lane)
    monkeypatch.setattr(coverage_run, "run_goal", lambda *a, **kw: pytest.fail("полоса не должна идти позами"))
    outcomes, stop = coverage_run.drive(None, None, _lanes(2, 4, 3, 2), timeout=45.0, max_fail=3, lanes=True)
    assert stop == ""
    assert [o.name for o in outcomes] == ["L0000_c0l0", "L0001_c0l1", "L0002_c0l2", "L0003_c0l3"]
    assert [o.result for o in outcomes] == ["SUCCEEDED", "ABORTED", "TIMEOUT", "SUCCEEDED"]
    assert [(o.poses, o.passed) for o in outcomes] == [(2, 2), (4, 2), (3, 0), (2, 2)]
    assert [o.em_episodes for o in outcomes] == [0, 2, 0, 1]
    # таймаут полосы: 45 с на позу + путь на потолке Nav2 0.15 м/с; у первой полосы пути «от» нет
    speed = coverage_run.SPEED_MPS
    assert sent[0] == ("L0000_c0l0", 2, round(45.0 * 2 + 1.0 / speed, 3))
    assert sent[1] == ("L0001_c0l1", 4, round(45.0 * 4 + (2**0.5 + 3.0) / speed, 3))


def test_lane_failures_in_row_stop_and_not_sent_keeps_pose_count(coverage_run, monkeypatch):
    monkeypatch.setattr(coverage_run, "run_lane", lambda *a, **kw: ("STATUS_ABORTED", 0, 2))
    outcomes, stop = coverage_run.drive(None, None, _lanes(2, 2, 3, 2), timeout=1.0, max_fail=2, lanes=True)
    assert stop == "2_failures_in_row"
    assert [o.result for o in outcomes] == ["ABORTED", "ABORTED", "NOT_SENT", "NOT_SENT"]
    assert sum(o.poses for o in outcomes) == 9 and sum(o.passed for o in outcomes) == 0


def test_signal_during_lane_cancels_it(coverage_run, monkeypatch):
    stop = coverage_run.StopSignal()

    def fake_run_lane(n, cl, name, poses, timeout, stop=None):
        sig(signal.SIGTERM)
        assert stop()
        return "INTERRUPTED", 0, 1

    sig = stop
    monkeypatch.setattr(coverage_run, "run_lane", fake_run_lane)
    outcomes, why = coverage_run.drive(None, None, _lanes(3, 2), timeout=1.0, max_fail=3, stop=stop, lanes=True)
    assert why == "interrupted_SIGTERM"
    assert [(o.result, o.poses, o.passed) for o in outcomes] == [("INTERRUPTED", 3, 2), ("NOT_SENT", 2, 0)]


def test_default_timeout_is_45_s_and_lanes_by_default(coverage_run):
    args = coverage_run.parse_args([])
    assert args.timeout == 45.0 and not args.per_pose
    assert coverage_run.parse_args(["--per-pose"]).per_pose


def test_finish_reports_motion(coverage_run, tmp_path):
    grid = OccupancyGrid(80, 60, 0.05, 0.0, 0.0, [0] * 4800)
    space = coverage_space(grid, (1.0, 1.0))
    # 10 с едет 0.1 м/с, потом 10 с стоит (точка раз в секунду, как прореживает CoverageMon)
    track = [(float(t), 1.0 + 0.1 * t, 1.0, 0.0) for t in range(11)] + [(10.0 + t, 2.0, 1.0, 0.0) for t in range(1, 11)]
    node = types.SimpleNamespace(track=track, simt=lambda: 20.0)
    outcomes = [coverage_run.GoalOutcome.from_route("L0000", "STATUS_SUCCEEDED", poses=2, remaining=1)]
    line = coverage_run.finish(tmp_path, node, grid, space, outcomes, "", 0.0)
    assert "goals=1 succeeded=1" in line and "poses=2 poses_passed=2" in line
    assert "moving_s=10.0 moving_pct=50.0% moved_m=1.0 avg_mps=0.050 moving_mps=0.100" in line
