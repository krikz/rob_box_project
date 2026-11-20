"""Разворот у цели (#4694): xy внутри допуска, yaw вне — крутим; допуск yaw не выдумываем."""

import math
import types
from pathlib import Path

import pytest
import yaml

from rob_box_core.follow_path_halt import NAV_CMD_VEL_TOPIC, YAW_AT_GOAL_CMD_VEL_TOPIC
from rob_box_teleop import yaw_at_goal as yag
from rob_box_teleop.followpath_watchdog_node import FollowPathWatchdogNode
from rob_box_teleop.yaw_at_goal_node import goal_from_plan

REPO = Path(__file__).resolve().parents[3]
NAV2 = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

# Depot 17.11 17:06Z, истина Gazebo на GOAL_END (комментарий #4694).
M2_GOAL = (-1.0, 4.5, 0.0)
M2_POSE = (-1.188, 4.366, math.radians(28.5))
R3_GOAL = (-3.7, 2.34, math.radians(90.0))
R3_POSE = (-3.546, 2.159, math.radians(147.1))
# Depot M1 18.11 00:24Z: план stamp=42.792, now=209.60, map внутри 0.25 м, yaw ~113°.
M1_GOAL = (-3.7, 4.0, 0.0)
M1_POSE = (-3.484, 3.951, math.radians(113.4))
M1_FAR = (-3.7 + 0.5, 4.0, math.radians(113.4))
M1_PLAN_STAMP_S = 42.792
M1_NOW_S = 209.60


def _params():
    return yaml.safe_load(NAV2.read_text(encoding="utf-8"))


def test_tolerances_come_from_general_goal_checker_not_a_made_up_degree():
    xy, yaw, omega = yag.tolerances_from_nav2(_params())
    checker = _params()["controller_server"]["ros__parameters"]["general_goal_checker"]
    follow = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert xy == checker["xy_goal_tolerance"] == 0.25
    assert yaw == checker["yaw_goal_tolerance"] == pytest.approx(0.30)
    assert omega == follow["max_vel_theta"] == pytest.approx(0.6)
    # 0.30 рад ≈ 17.2°. M2 28.5° и R3 57.1° вне допуска — checker честно не принял бы цель.
    assert yag.yaw_outside(M2_POSE[2], M2_GOAL[2], yaw)
    assert yag.yaw_outside(R3_POSE[2], R3_GOAL[2], yaw)
    assert not yag.yaw_outside(math.radians(16.0), 0.0, yaw)


def test_depot_timeouts_turn_toward_goal_yaw_inside_xy_window():
    xy, yaw, omega = yag.tolerances_from_nav2(_params())
    m2 = yag.turn_at_goal(M2_POSE, M2_GOAL, xy, yaw, omega)
    r3 = yag.turn_at_goal(R3_POSE, R3_GOAL, xy, yaw, omega)
    assert m2 is not None and not m2.done and m2.omega < 0.0
    assert r3 is not None and not r3.done and r3.omega < 0.0
    assert abs(m2.omega) == pytest.approx(min(abs(yag.shortest_yaw(M2_POSE[2], 0.0)), omega))
    assert abs(r3.omega) == pytest.approx(omega)  # 57° больше max_vel_theta за 1 с


def test_outside_xy_window_is_not_our_turn():
    xy, yaw, omega = yag.tolerances_from_nav2(_params())
    far = (M2_GOAL[0] + 0.4, M2_GOAL[1], math.radians(40.0))
    assert yag.turn_at_goal(far, M2_GOAL, xy, yaw, omega) is None


def test_xy_and_yaw_inside_tolerance_is_done_not_another_spin():
    xy, yaw, omega = yag.tolerances_from_nav2(_params())
    pose = (M2_GOAL[0] + 0.1, M2_GOAL[1], math.radians(5.0))
    turn = yag.turn_at_goal(pose, M2_GOAL, xy, yaw, omega)
    assert turn == yag.Turn(0.0, True, "yaw внутри допуска")


def test_shortest_turn_crosses_the_branch_cut():
    assert yag.shortest_yaw(math.radians(170.0), math.radians(-170.0)) == pytest.approx(math.radians(20.0), abs=1e-9)
    turn = yag.turn_at_goal((0.0, 0.0, math.radians(170.0)), (0.0, 0.0, math.radians(-170.0)), 0.25, 0.20, 0.6)
    assert turn is not None and turn.omega > 0.0


def test_missing_checker_is_a_loud_failure_not_a_default_degree():
    doc = {"controller_server": {"ros__parameters": {}}}
    with pytest.raises(ValueError):
        yag.tolerances_from_nav2(doc)


def _tf(pose):
    """Ответ lookup_transform: translation + rotation, как ждёт Tf2Lookup."""
    x, y, yaw = pose
    rot = _quat_msg(yaw)
    return types.SimpleNamespace(
        transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=x, y=y, z=0.0), rotation=rot)
    )


def _quat_msg(yaw: float):
    z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return types.SimpleNamespace(x=0.0, y=0.0, z=z, w=w)


def _plan(x, y, yaw, stamp_s=1.0, frame="map"):
    pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0), orientation=_quat_msg(yaw))
    sec, nsec = divmod(int(round(stamp_s * 1e9)), 1_000_000_000)
    header = types.SimpleNamespace(frame_id=frame, stamp=types.SimpleNamespace(sec=sec, nanosec=nsec))
    return types.SimpleNamespace(header=header, poses=[types.SimpleNamespace(pose=pose)])


def _status(code: int):
    return types.SimpleNamespace(status=code)


def test_goal_from_plan_is_the_tail_the_checker_compares():
    path = _plan(-1.0, 4.5, 0.0, stamp_s=10.0)
    assert goal_from_plan(path, 10.5) == pytest.approx((-1.0, 4.5, 0.0))
    assert goal_from_plan(path, 13.0) is None  # старше 2 с, позы нет — не цель
    assert goal_from_plan(_plan(0.0, 0.0, 0.0, frame="odom"), 1.0) is None
    m1 = _plan(*M1_GOAL, stamp_s=M1_PLAN_STAMP_S)
    assert goal_from_plan(m1, M1_NOW_S, pose=M1_POSE, xy_tolerance_m=0.25) == pytest.approx(M1_GOAL)
    assert goal_from_plan(m1, M1_NOW_S, pose=M1_FAR, xy_tolerance_m=0.25) is None


def test_node_turns_m2_on_its_own_topic_and_does_not_touch_controller_cmd_vel(node_params):
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M2_POSE)
    yaw._now = lambda: 100.0
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*M2_GOAL, stamp_s=100.0))
    yaw._tick()
    published = {topic: pub.messages for topic, pub in node.publishers}
    assert published[NAV_CMD_VEL_TOPIC] == []
    twists = published[YAW_AT_GOAL_CMD_VEL_TOPIC]
    assert len(twists) == 1 and twists[0].linear.x == 0.0 and twists[0].angular.z < 0.0
    assert abs(twists[0].angular.z) == pytest.approx(min(abs(yag.shortest_yaw(M2_POSE[2], 0.0)), 0.6))


def test_tf_lookup_passes_time_objects_not_floats(node_params):
    """#4788: to_time=lambda s: s → lookup_transform(float) → TypeError, /cmd_vel_yaw молчит."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    calls = []

    def capture(target, source, time, timeout):
        calls.append((time, timeout))
        return _tf(M2_POSE)

    yaw._lookup._buffer.lookup_transform = capture
    yaw._now = lambda: 100.0
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*M2_GOAL, stamp_s=100.0))
    yaw._tick()
    assert calls, "latest() должен звать lookup_transform"
    time_arg, timeout_arg = calls[0]
    assert not isinstance(time_arg, float)
    assert not isinstance(timeout_arg, float)
    published = {topic: pub.messages for topic, pub in node.publishers}
    assert published[YAW_AT_GOAL_CMD_VEL_TOPIC]


def test_node_turns_at_goal_when_plan_is_older_than_two_seconds(node_params):
    """M1: xy внутри 0.25 м, yaw вне допуска, /plan старше 2 с — крутим."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M1_POSE)
    yaw._now = lambda: M1_NOW_S
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*M1_GOAL, stamp_s=M1_PLAN_STAMP_S))
    yaw._tick()
    published = {topic: pub.messages for topic, pub in node.publishers}
    assert published[NAV_CMD_VEL_TOPIC] == []
    twists = published[YAW_AT_GOAL_CMD_VEL_TOPIC]
    assert len(twists) == 1 and twists[0].linear.x == 0.0
    assert twists[0].angular.z < 0.0
    assert abs(twists[0].angular.z) == pytest.approx(min(abs(yag.shortest_yaw(M1_POSE[2], 0.0)), 0.6))


def test_node_does_not_invent_path_from_stale_plan_when_far(node_params):
    """Далеко от хвоста старый /plan не цель: FollowPath, не выдуманный разворот."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M1_FAR)
    yaw._now = lambda: M1_NOW_S
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*M1_GOAL, stamp_s=M1_PLAN_STAMP_S))
    yaw._tick()
    assert all(pub.messages == [] for _, pub in node.publishers)


def test_node_is_silent_outside_the_xy_window_and_when_nav_is_not_live(node_params):
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf((-1.5, 4.5, math.radians(40.0)))
    yaw._now = lambda: 100.0
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*M2_GOAL, stamp_s=100.0))
    yaw._tick()
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(6)]))  # ABORTED — не наша забота
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M2_POSE)
    yaw._tick()
    assert all(pub.messages == [] for _, pub in node.publishers)


def _yaw_twists(node):
    return {topic: pub.messages for topic, pub in node.publishers}[YAW_AT_GOAL_CMD_VEL_TOPIC]


def _arm(node, pose, goal, now_s=100.0):
    """Живой NavigateThroughPoses + хвост /plan + поза. Как окна стоянки Gate 2."""
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(pose)
    yaw._now = lambda: now_s
    yaw._on_nav(types.SimpleNamespace(status_list=[_status(2)]))
    yaw._on_plan(_plan(*goal, stamp_s=now_s))


# Gate 2 Depot 20.11, окна стоянки 2 с: /cmd_vel_yaw = wz 0.0 ровно 20/20 на 10 Гц
# (M2/M5/R1/R2/R3), mux /diff_drive_controller/cmd_vel_unstamped совпал с нулями,
# а не с ненулевым /cmd_vel DWB (mean angular.z 0.150/0.485/0.600/−0.128/0.512).
# Причина — _tick публиковал Turn(0, done) каждый тик: yaw уже в допуске, xy внутри,
# план свежий. timeout 0.2 с канала priority 15 не истекал.
_STALL_TICKS = 20  # 2 с × 10 Гц, как семплы окон bag


def test_inside_yaw_tolerance_does_not_hold_cmd_vel_yaw_at_zero(node_params):
    """Ветка бага: xy и yaw внутри допуска, план свежий — тишина, не нули."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    inside = (M2_GOAL[0] + 0.1, M2_GOAL[1], math.radians(5.0))
    _arm(node, inside, M2_GOAL)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    assert twists == []
    assert node._yaw._turning is False


def test_active_turn_still_publishes_nonzero_omega(node_params):
    """Пока разворот идёт, команда как раньше: свой топик, знак к цели, не нули."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    _arm(node, M2_POSE, M2_GOAL)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    assert len(twists) == _STALL_TICKS
    assert all(t.linear.x == 0.0 and t.angular.z < 0.0 for t in twists)
    assert abs(twists[0].angular.z) == pytest.approx(min(abs(yag.shortest_yaw(M2_POSE[2], 0.0)), 0.6))


def test_finished_turn_publishes_one_zero_then_silence(node_params):
    """Yaw вошёл в допуск: один нулевой Twist, дальше ни одного — timeout 0.2 отдаёт пол Nav2."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    _arm(node, M2_POSE, M2_GOAL)
    node._yaw._tick()
    assert len(_yaw_twists(node)) == 1 and _yaw_twists(node)[0].angular.z != 0.0

    inside = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(2.0))
    node._yaw._lookup._buffer.lookup_transform = lambda *args: _tf(inside)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    zeros = [t for t in twists if t.angular.z == 0.0 and t.linear.x == 0.0]
    assert len(zeros) == 1
    assert twists[-1] is zeros[0]
    assert node._yaw._turning is False


def test_aborted_turn_publishes_one_zero_then_silence(node_params):
    """Отказ: nav больше не жив / вышли из xy — тоже один ноль, не поток нулей."""
    node_params["nav2_params_file"] = str(NAV2)
    node = FollowPathWatchdogNode()
    _arm(node, M2_POSE, M2_GOAL)
    node._yaw._tick()
    node._yaw._on_nav(types.SimpleNamespace(status_list=[_status(6)]))  # ABORTED
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    zeros = [t for t in twists if t.angular.z == 0.0]
    assert len(zeros) == 1
    assert len(twists) == 2  # команда разворота + один ноль
    assert node._yaw._turning is False
