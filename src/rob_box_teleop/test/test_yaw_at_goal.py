"""Финальный подъезд к цели (#4694): держим цель, доворачиваем, едем, один ноль.

Допуск yaw не выдумываем — читаем goal checker. Старый ``turn_at_goal`` заменён на
``approach_at_goal``: «разворот на месте» по истине идёт по окружности 0.088 м и сам
выносит позу из окна 0.25 м (Gate 2 21.11 M3), а защёлкнутый DWB везти не может.
"""

import math
import types
from pathlib import Path

import pytest
import yaml

from rob_box_core.follow_path_halt import NAV_CMD_VEL_TOPIC, YAW_AT_GOAL_CMD_VEL_TOPIC
from rob_box_teleop import followpath_orphan as fo
from rob_box_teleop import yaw_at_goal as yag
from rob_box_teleop.followpath_watchdog_node import FOLLOW_STATUS_TOPIC, PARENT_STATUS_TOPICS, FollowPathWatchdogNode
from rob_box_teleop.yaw_at_goal_node import goal_from_plan

REPO = Path(__file__).resolve().parents[3]
NAV2 = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
_TO_POSE, _THROUGH_POSES = (topic for _, topic in PARENT_STATUS_TOPICS)

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
# Gate 2 Depot 21.11, проход 1, M3: цель (−1, −1, yaw 0°). Захват на 0.24 м при yaw −79°,
# затем «разворот» вынес позу на 0.396 м при vx=0, план заморожен.
M3_GOAL = (-1.0, -1.0, 0.0)
M3_CAPTURE = (-1.0, -1.0 + 0.24, math.radians(-79.0))
M3_THROWN = (-1.0, -1.0 + 0.396, math.radians(-79.0))


def _params():
    return yaml.safe_load(NAV2.read_text(encoding="utf-8"))


def _tols():
    return yag.tolerances_from_nav2(_params())


def test_tolerances_come_from_general_goal_checker_not_a_made_up_degree():
    xy, yaw, omega, speed = _tols()
    checker = _params()["controller_server"]["ros__parameters"]["general_goal_checker"]
    follow = _params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert xy == checker["xy_goal_tolerance"] == 0.25
    assert yaw == checker["yaw_goal_tolerance"] == pytest.approx(0.22)
    assert omega == follow["max_vel_theta"] == pytest.approx(0.6)
    assert speed == follow["max_speed_xy"] == pytest.approx(0.15)
    # 0.22 рад ≈ 12.6°. M2 28.5° и R3 57.1° вне допуска — checker честно не принял бы цель.
    assert yag.yaw_outside(M2_POSE[2], M2_GOAL[2], yaw)
    assert yag.yaw_outside(R3_POSE[2], R3_GOAL[2], yaw)
    assert not yag.yaw_outside(math.radians(12.0), 0.0, yaw)


def test_approach_target_survives_the_measured_rotation_sweep():
    """0.10 + 0.088 + 0.03 ≤ xy_goal_tolerance боевого yaml. Сдвинут допуск — тест падает."""
    xy, _, _, _ = _tols()
    assert yag.APPROACH_TARGET_M + yag.ROTATION_SWEEP_M + 0.03 <= xy
    assert yag.ROTATION_SWEEP_M == pytest.approx(0.088)
    assert yag.APPROACH_TARGET_M == pytest.approx(0.10)


def test_far_from_point_turns_nose_before_driving():
    """R3: нос на 16.7° от точки, дальше 0.15 рад — чистый доворот, без хода."""
    xy, yaw, omega, speed = _tols()
    step = yag.approach_at_goal(R3_POSE, R3_GOAL, xy, yaw, speed, omega)
    bearing = yag.bearing_to(R3_POSE, R3_GOAL)
    assert abs(bearing) > yag.BEARING_ALIGN_RAD
    expected = math.copysign(min(abs(bearing), omega), bearing)
    assert step == yag.Approach(0.0, expected, False, "доворот на точку")


def test_aligned_nose_drives_no_faster_than_our_cap_or_the_controller():
    """M2: нос на точку уже смотрит (bearing 7° < 0.15) — едем, не крутимся.

    Раньше ``turn_at_goal`` здесь крутил к курсу цели (dyaw −28.5°) на месте. Это и выносило
    позу из окна по окружности 0.088 м. Теперь сначала ход: потолок — наш 0.10 и FollowPath.
    """
    xy, yaw, omega, speed = _tols()
    assert abs(yag.bearing_to(M2_POSE, M2_GOAL)) < yag.BEARING_ALIGN_RAD
    step = yag.approach_at_goal(M2_POSE, M2_GOAL, xy, yaw, speed, omega)
    assert step is not None and not step.done and step.reason == "подъезд к точке"
    assert step.vx == pytest.approx(min(speed, yag.APPROACH_MAX_VX_M_S))
    assert step.vx <= 0.10
    slower = yag.approach_at_goal(M2_POSE, M2_GOAL, xy, yaw, 0.05, omega)
    assert slower is not None and slower.vx == pytest.approx(0.05)


def test_inside_approach_target_turns_to_the_goal_yaw():
    xy, yaw, omega, speed = _tols()
    pose = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(40.0))
    assert yag.pose_distance(pose, M2_GOAL) <= yag.APPROACH_TARGET_M
    step = yag.approach_at_goal(pose, M2_GOAL, xy, yaw, speed, omega)
    dyaw = yag.shortest_yaw(pose[2], M2_GOAL[2])
    expected = math.copysign(min(abs(dyaw), omega), dyaw)
    assert step == yag.Approach(0.0, expected, False, "доворот к курсу цели")


def test_both_inside_tolerance_is_done_not_another_spin():
    """У точки и курс глубже допуска (запас 0.05) — стоп, не ещё один круг."""
    xy, yaw, omega, speed = _tols()
    pose = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(5.0))
    step = yag.approach_at_goal(pose, M2_GOAL, xy, yaw, speed, omega)
    assert step == yag.Approach(0.0, 0.0, True, "цель в допуске")


def test_bearing_crosses_the_branch_cut_by_the_short_way():
    pose = (0.0, 0.0, math.radians(170.0))
    goal = (1.0, 0.0, 0.0)  # точка строго по +x, курс робота почти назад
    bearing = yag.bearing_to(pose, goal)
    assert bearing == pytest.approx(math.radians(-170.0), abs=1e-9)
    assert abs(bearing) <= math.pi
    step = yag.approach_at_goal(pose, goal, 0.25, 0.22, 0.15, 0.6)
    assert step is not None and step.omega < 0.0 and step.vx == 0.0


def test_missing_pose_or_nonpositive_speed_is_none():
    assert yag.approach_at_goal(None, M2_GOAL, 0.25, 0.22, 0.15, 0.6) is None
    assert yag.approach_at_goal(M2_POSE, None, 0.25, 0.22, 0.15, 0.6) is None
    assert yag.approach_at_goal(M2_POSE, M2_GOAL, 0.25, 0.22, 0.0, 0.6) is None
    assert yag.approach_at_goal(M2_POSE, M2_GOAL, 0.25, 0.22, 0.15, 0.0) is None


def test_goal_differs_ignores_a_frozen_tail_and_sees_a_new_one():
    same = (M2_GOAL[0] + 0.004, M2_GOAL[1], M2_GOAL[2] + 0.009)
    assert not yag.goal_differs(M2_GOAL, same, yag.GOAL_CHANGE_EPS_M, yag.GOAL_CHANGE_EPS_RAD)
    moved = (M2_GOAL[0] + 0.02, M2_GOAL[1], M2_GOAL[2])
    assert yag.goal_differs(M2_GOAL, moved, yag.GOAL_CHANGE_EPS_M, yag.GOAL_CHANGE_EPS_RAD)
    turned = (M2_GOAL[0], M2_GOAL[1], M2_GOAL[2] + 0.05)
    assert yag.goal_differs(M2_GOAL, turned, yag.GOAL_CHANGE_EPS_M, yag.GOAL_CHANGE_EPS_RAD)
    assert yag.goal_differs(None, M2_GOAL, 0.005, 0.01)
    assert not yag.goal_differs(None, None, 0.005, 0.01)


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


def _action_status(node, topic, *goals):
    status_list = []
    for goal_id, code in goals:
        goal_info = types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=bytes([goal_id]) * 16),
            stamp=types.SimpleNamespace(sec=100, nanosec=0),
        )
        status_list.append(types.SimpleNamespace(goal_info=goal_info, status=code))
    msg = types.SimpleNamespace(status_list=status_list)
    for subscribed_topic, callback in node.subscriptions:
        if subscribed_topic == topic:
            callback(msg)


def test_goal_from_plan_is_the_tail_the_checker_compares():
    path = _plan(-1.0, 4.5, 0.0, stamp_s=10.0)
    assert goal_from_plan(path, 10.5) == pytest.approx((-1.0, 4.5, 0.0))
    assert goal_from_plan(path, 13.0) is None  # старше 2 с, позы нет — не цель
    assert goal_from_plan(_plan(0.0, 0.0, 0.0, frame="odom"), 1.0) is None
    m1 = _plan(*M1_GOAL, stamp_s=M1_PLAN_STAMP_S)
    assert goal_from_plan(m1, M1_NOW_S, pose=M1_POSE, xy_tolerance_m=0.25) == pytest.approx(M1_GOAL)
    assert goal_from_plan(m1, M1_NOW_S, pose=M1_FAR, xy_tolerance_m=0.25) is None


def _yaw_twists(node):
    return {topic: pub.messages for topic, pub in node.publishers}[YAW_AT_GOAL_CMD_VEL_TOPIC]


def _arm(node, pose, goal, now_s=100.0, stamp_s=None):
    """Живые родитель и FollowPath, хвост /plan и поза."""
    yaw = node._yaw
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(pose)
    yaw._now = lambda: now_s
    _action_status(node, _TO_POSE, (1, fo.STATUS_EXECUTING))
    _action_status(node, FOLLOW_STATUS_TOPIC, (1, fo.STATUS_EXECUTING))
    yaw._on_plan(_plan(*goal, stamp_s=now_s if stamp_s is None else stamp_s))


def _node(node_params):
    node_params["nav2_params_file"] = str(NAV2)
    return FollowPathWatchdogNode()


def test_node_drives_m2_on_its_own_topic_and_does_not_touch_controller_cmd_vel(node_params):
    """M2: нос на точку уже смотрит — публикуем ход, не разворот. /cmd_vel контроллера не трогаем."""
    node = _node(node_params)
    _arm(node, M2_POSE, M2_GOAL)
    node._yaw._tick()
    published = {topic: pub.messages for topic, pub in node.publishers}
    assert published[NAV_CMD_VEL_TOPIC] == []
    twists = published[YAW_AT_GOAL_CMD_VEL_TOPIC]
    assert len(twists) == 1 and twists[0].linear.x == pytest.approx(0.10) and twists[0].angular.z != 0.0


def test_tf_lookup_passes_time_objects_not_floats(node_params):
    """#4788: to_time=lambda s: s → lookup_transform(float) → TypeError, /cmd_vel_yaw молчит."""
    node = _node(node_params)
    yaw = node._yaw
    calls = []

    def capture(target, source, time, timeout):
        calls.append((time, timeout))
        return _tf(M2_POSE)

    _arm(node, M2_POSE, M2_GOAL)
    yaw._lookup._buffer.lookup_transform = capture  # после _arm: он ставит свою lambda
    yaw._tick()
    assert calls, "latest() должен звать lookup_transform"
    time_arg, timeout_arg = calls[0]
    assert not isinstance(time_arg, float)
    assert not isinstance(timeout_arg, float)
    assert _yaw_twists(node)


def test_node_holds_goal_when_plan_is_older_than_two_seconds(node_params):
    """M1: xy внутри 0.25 м, /plan старше 2 с — захват есть, подъезд идёт (нос не на точку)."""
    node = _node(node_params)
    _arm(node, M1_POSE, M1_GOAL, now_s=M1_NOW_S, stamp_s=M1_PLAN_STAMP_S)
    node._yaw._tick()
    published = {topic: pub.messages for topic, pub in node.publishers}
    assert published[NAV_CMD_VEL_TOPIC] == []
    twists = published[YAW_AT_GOAL_CMD_VEL_TOPIC]
    assert len(twists) == 1 and twists[0].linear.x == 0.0 and twists[0].angular.z != 0.0
    assert node._yaw._hold == pytest.approx(M1_GOAL)


def test_node_does_not_invent_path_from_stale_plan_when_far(node_params):
    """Далеко от хвоста старый /plan не цель: FollowPath, не выдуманный подъезд."""
    node = _node(node_params)
    _arm(node, M1_FAR, M1_GOAL, now_s=M1_NOW_S, stamp_s=M1_PLAN_STAMP_S)
    node._yaw._tick()
    assert all(pub.messages == [] for _, pub in node.publishers)
    assert node._yaw._hold is None


def test_node_is_silent_outside_the_xy_window_and_when_nav_is_not_live(node_params):
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, (-1.5, 4.5, math.radians(40.0)), M2_GOAL)
    yaw._tick()
    _action_status(node, _TO_POSE, (1, fo.STATUS_ABORTED))
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M2_POSE)
    yaw._tick()
    assert all(pub.messages == [] for _, pub in node.publishers)


# Gate 2 Depot 20.11, окна стоянки 2 с: /cmd_vel_yaw = wz 0.0 ровно 20/20 на 10 Гц
# (M2/M5/R1/R2/R3), mux совпал с нулями, а не с ненулевым /cmd_vel DWB.
# Причина — _tick публиковал нулевой Twist каждый тик. timeout 0.2 с канала priority 15
# не истекал. Нулевого потока быть не должно: ни вне окна, ни внутри допуска.
_STALL_TICKS = 20  # 2 с × 10 Гц, как семплы окон bag


def test_inside_tolerance_at_capture_publishes_nothing(node_params):
    """Захват уже в допуске обоих: done сразу, ни одной публикации, нулей тоже."""
    node = _node(node_params)
    inside = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(5.0))
    _arm(node, inside, M2_GOAL)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    assert _yaw_twists(node) == []
    assert node._yaw._floor is False
    assert node._yaw._hold is None


def test_active_approach_keeps_publishing(node_params):
    """Пока подъезд идёт, команда каждый тик: свой топик, оба поля, не нули."""
    node = _node(node_params)
    _arm(node, M2_POSE, M2_GOAL)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    assert len(twists) == _STALL_TICKS
    assert all(t.linear.x > 0.0 for t in twists)
    assert node._yaw._floor is True


def test_finished_approach_publishes_one_zero_then_silence(node_params):
    """Доехали и довернули: один нулевой Twist, дальше ни одного — timeout 0.2 отдаёт пол Nav2."""
    node = _node(node_params)
    _arm(node, M2_POSE, M2_GOAL)
    node._yaw._tick()
    assert len(_yaw_twists(node)) == 1 and _yaw_twists(node)[0].linear.x != 0.0

    inside = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(2.0))
    node._yaw._lookup._buffer.lookup_transform = lambda *args: _tf(inside)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    zeros = [t for t in twists if t.angular.z == 0.0 and t.linear.x == 0.0]
    assert len(zeros) == 1
    assert twists[-1] is zeros[0]
    assert node._yaw._floor is False
    assert node._yaw._hold is None


def test_nav_succeeded_publishes_one_zero_then_silence(node_params):
    """Nav больше не жив (SUCCEEDED=4): один ноль и тишина, цель забыта."""
    node = _node(node_params)
    _arm(node, M2_POSE, M2_GOAL)
    node._yaw._tick()
    _action_status(node, _TO_POSE, (1, fo.STATUS_SUCCEEDED))
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    twists = _yaw_twists(node)
    zeros = [t for t in twists if t.angular.z == 0.0 and t.linear.x == 0.0]
    assert len(zeros) == 1
    assert len(twists) == 2  # команда подъезда + один ноль
    assert node._yaw._floor is False
    assert node._yaw._hold is None


@pytest.mark.parametrize("status", [fo.STATUS_ABORTED, fo.STATUS_CANCELING])
def test_recovery_gets_control_until_a_new_follow_path_starts(node_params, status):
    node = _node(node_params)
    yaw = node._yaw
    pose = (M2_GOAL[0] + 0.05, M2_GOAL[1], math.radians(45.0))
    _arm(node, pose, M2_GOAL)
    yaw._tick()
    twists = _yaw_twists(node)
    assert len(twists) == 1 and twists[0].angular.z < 0.0
    assert yaw._stage.finishing

    # Родительский NavigateToPose всё ещё выполняется и /plan не менялся.
    _action_status(node, FOLLOW_STATUS_TOPIC, (1, status))
    for _ in range(_STALL_TICKS):
        yaw._tick()
    assert len(twists) == 2
    assert (twists[-1].linear.x, twists[-1].angular.z) == (0.0, 0.0)
    assert yaw._hold is None
    assert not yaw._stage.finishing

    _action_status(node, FOLLOW_STATUS_TOPIC, (1, status), (2, fo.STATUS_EXECUTING))
    yaw._tick()
    assert len(twists) == 3 and twists[-1].angular.z < 0.0


def test_no_yaw_command_until_follow_path_is_active(node_params):
    node = _node(node_params)
    _arm(node, M2_POSE, M2_GOAL)
    _action_status(node, FOLLOW_STATUS_TOPIC)
    node._yaw._tick()
    assert _yaw_twists(node) == []


@pytest.mark.parametrize("status", [fo.STATUS_ABORTED, fo.STATUS_CANCELING])
def test_old_parent_status_does_not_suppress_other_live_parent(node_params, status):
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M2_POSE, M2_GOAL)
    _action_status(node, _THROUGH_POSES, (2, fo.STATUS_EXECUTING))
    yaw._tick()
    twists = _yaw_twists(node)
    assert len(twists) == 1 and twists[-1].linear.x > 0.0

    _action_status(node, _TO_POSE, (1, status))
    yaw._tick()
    assert len(twists) == 2 and twists[-1].linear.x > 0.0

    _action_status(node, _THROUGH_POSES, (2, status))
    for _ in range(_STALL_TICKS):
        yaw._tick()
    assert len(twists) == 3
    assert (twists[-1].linear.x, twists[-1].angular.z) == (0.0, 0.0)


def test_each_action_status_is_subscribed_once(node_params):
    node = _node(node_params)
    topics = [topic for topic, _ in node.subscriptions]
    assert all(topics.count(topic) == 1 for topic in (_TO_POSE, _THROUGH_POSES, FOLLOW_STATUS_TOPIC))


def test_m3_keeps_publishing_after_rotation_throws_it_out_of_the_window(node_params):
    """M3: захват на 0.24 м, разворот вынес на 0.396 м, план заморожен — не молчим, доезжаем.

    Старая нода здесь отпускала пол: goal_from_plan за окном 2 с возвращал None.
    Удержанная цель этого не позволяет. У точки и на курсе — ровно один ноль.
    """
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M3_CAPTURE, M3_GOAL, now_s=180.0, stamp_s=180.0)
    yaw._tick()
    assert yaw._hold == pytest.approx(M3_GOAL)
    assert _yaw_twists(node)[-1].linear.x == 0.0  # нос на −79°, сначала доворот

    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(M3_THROWN)
    yaw._now = lambda: 190.0  # план старше 2 с, поза вне 0.25 м
    for _ in range(5):
        yaw._tick()
    thrown = _yaw_twists(node)[-5:]
    assert all(t.linear.x != 0.0 or t.angular.z != 0.0 for t in thrown)
    assert yaw._hold == pytest.approx(M3_GOAL)

    arrived = (M3_GOAL[0], M3_GOAL[1] + 0.05, math.radians(2.0))
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(arrived)
    before = len(_yaw_twists(node))
    for _ in range(_STALL_TICKS):
        yaw._tick()
    after = _yaw_twists(node)[before:]
    zeros = [t for t in after if t.linear.x == 0.0 and t.angular.z == 0.0]
    assert zeros == [after[0]]
    assert len(after) == 1
    assert yaw._floor is False


def test_escape_beyond_radius_publishes_one_zero_then_silence(node_params):
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M2_POSE, M2_GOAL)
    yaw._tick()
    escaped = (M2_GOAL[0] + 0.70, M2_GOAL[1], 0.0)
    assert yag.pose_distance(escaped, M2_GOAL) > yag.ESCAPE_RADIUS_M
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf(escaped)
    for _ in range(_STALL_TICKS):
        yaw._tick()
    twists = _yaw_twists(node)
    zeros = [t for t in twists if t.linear.x == 0.0 and t.angular.z == 0.0]
    assert len(zeros) == 1 and twists[-1] is zeros[0]
    assert len(twists) == 2
    assert yaw._hold is None and yaw._floor is False


def test_fresh_plan_with_a_different_tail_moves_the_hold(node_params):
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M2_POSE, M2_GOAL, now_s=100.0)
    yaw._tick()
    assert yaw._hold == pytest.approx(M2_GOAL)
    new_goal = (M2_GOAL[0] + 0.10, M2_GOAL[1], math.radians(20.0))
    yaw._on_plan(_plan(*new_goal, stamp_s=100.0))
    yaw._lookup._buffer.lookup_transform = lambda *args: _tf((new_goal[0] + 0.05, new_goal[1], 0.0))
    yaw._tick()
    assert yaw._hold == pytest.approx(new_goal)
    assert yaw._engaged_at == pytest.approx(100.0)


def test_time_cap_publishes_one_zero_then_blocks_reengage(node_params):
    """Потолок 25 с: один ноль и 5 с тишины, даже если поза снова в окне."""
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M2_POSE, M2_GOAL, now_s=100.0)
    yaw._tick()
    yaw._now = lambda: 100.0 + yag.ENGAGE_TIME_CAP_S + 0.1
    yaw._tick()
    assert yaw._hold is None and yaw._floor is False
    cap_at = 100.0 + yag.ENGAGE_TIME_CAP_S + 0.1
    assert yaw._block_until == pytest.approx(cap_at + yag.REENGAGE_COOLDOWN_S)
    twists_at_cap = len(_yaw_twists(node))
    yaw._now = lambda: yaw._block_until - 0.5
    for _ in range(5):
        yaw._tick()
    assert len(_yaw_twists(node)) == twists_at_cap  # тишина, нулей больше нет
    assert yaw._hold is None
    yaw._now = lambda: yaw._block_until + 0.1
    yaw._tick()
    assert yaw._hold == pytest.approx(M2_GOAL)


def test_frozen_tail_does_not_reset_the_time_cap(node_params):
    """Тот же замороженный хвост часы не сбрасывает: иначе потолок никогда не сработает."""
    node = _node(node_params)
    yaw = node._yaw
    _arm(node, M2_POSE, M2_GOAL, now_s=100.0, stamp_s=100.0)
    yaw._tick()
    yaw._now = lambda: 130.0
    yaw._on_plan(_plan(*M2_GOAL, stamp_s=130.0))  # план освежили, хвост тот же
    yaw._tick()
    assert yaw._engaged_at == pytest.approx(100.0)
    assert yaw._hold is None  # 30 с > 25 с, цель забыта


def test_outside_window_without_capture_publishes_nothing(node_params):
    """Гард 20.11: вне окна и без захвата — ни одного сообщения, нулевого потока нет."""
    node = _node(node_params)
    far = (M2_GOAL[0] + 0.40, M2_GOAL[1], math.radians(40.0))
    _arm(node, far, M2_GOAL)
    for _ in range(_STALL_TICKS):
        node._yaw._tick()
    assert _yaw_twists(node) == []
    assert node._yaw._floor is False and node._yaw._hold is None
