#!/usr/bin/env python3
"""Маршрут из целей NavigateToPose по одной; лог: sim time, номер кадра камеры, поза gt, стопы защиты.

``run_lane`` — несколько точек одной целью NavigateThroughPoses (полоса обхода покрытием, #4560).
Тик ещё пишет YAW (/cmd_vel_yaw), PLAN (штамп /plan и now), MAP (map←base_footprint рядом с GT).
"""
import datetime
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

GOALS = [  # Depot; M — картография (карта пуста), R — маршрут по построенной карте
    ("M1", -3.7, 4.0, 0),
    ("M2", -1.0, 4.5, 0),
    ("M3", -1.0, -1.0, 0),
    ("M4", -3.7, -4.5, 0),
    ("M5", 3.0, -0.9, 0),
    ("M6", 9.0, -0.9, 0),
    ("R1", 4.25, 3.5, 90),
    ("R2", -2.0, 0.0, 180),
    ("R3", -3.7, 2.34, 90),
    ("R4", -3.2, -2.61, 270),
    ("R5", -4.0, -3.0, 0),
    ("R6", 3.0, -0.9, 0),
    ("R7", -3.7, 0.0, 180),
]
TIMEOUT = 200.0
# local_costmap.tf_buffer_duration (nav2_params.yaml): сколько costmap держит историю TF.
# Тот же предел — ждать кадр odom→base_footprint и active навигатора до первой цели, не TIMEOUT цели.
TF_BUFFER_S = 30.0
NAV_NODE = "bt_navigator"
ODOM_FRAME = "odom"  # local_costmap.global_frame: costmap ждёт global_frame ← robot_base_frame
MAP_FRAME = "map"
BASE_FRAME = "base_footprint"  # local_costmap.robot_base_frame
YAW_TOPIC = "/cmd_vel_yaw"  # twist_mux yaw_at_goal, тот же вход что у YawAtGoal
PLAN_TOPIC = "/plan"  # хвост пути Nav2; штамп vs now — видно, старше ли план 2 с


def load_goals(path):
    """Цели из файла точек мира (``goals:`` с name/x/y/yaw в радианах, напр. building.goals.yaml)."""
    import yaml

    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    return [
        (g["name"], float(g["x"]), float(g["y"]), round(math.degrees(float(g.get("yaw", 0.0))), 1))
        for g in doc["goals"]
    ]


STATUS = {v: k for k, v in GoalStatus.__dict__.items() if k.startswith("STATUS_")}


def ts():
    return datetime.datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]


def nframes():
    try:
        names = os.listdir("/sim_video/frames/video_top")
        return max(int(n.rsplit("_", 1)[1].split(".")[0]) for n in names)
    except Exception:
        return -1


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Mon(Node):
    def __init__(self, name="shidi_route"):
        super().__init__(name, parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.gt = None
        self.em_last = 0.0
        self.em_on = False
        self.em_episodes = 0
        self.recoveries = 0
        self.poses_remaining = 0
        self.yaw_wz = None
        self.yaw_sim = None
        self.plan_stamp = None
        self.tf_buffer = None
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self.on_gt, 10)
        self.create_subscription(Twist, "/cmd_vel_emergency", self.on_em, 10)
        self.create_subscription(Twist, YAW_TOPIC, self.on_yaw, 10)
        self.create_subscription(Path, PLAN_TOPIC, self.on_plan, 10)
        self.create_timer(0.2, self.tick)

    def simt(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, math.degrees(yaw_of(p.orientation)))

    def on_em(self, m):
        self.em_last = time.time()
        if not self.em_on:
            self.em_on = True
            self.em_episodes += 1
            print(f"{ts()} EM_ON sim={self.simt():.2f}", flush=True)

    def on_yaw(self, m):
        self.yaw_wz = float(m.angular.z)
        self.yaw_sim = self.simt()

    def on_plan(self, m):
        stamp = m.header.stamp
        self.plan_stamp = stamp.sec + stamp.nanosec * 1e-9

    def map_pose(self):
        """Поза map←base_footprint из того же TF-буфера, что ждёт wait_ready."""
        buf = self.tf_buffer
        if buf is None:
            return None
        try:
            from rclpy.duration import Duration
            from rclpy.time import Time

            tf = buf.lookup_transform(MAP_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, math.degrees(yaw_of(q)))

    def tick(self):
        if self.em_on and time.time() - self.em_last > 0.3:
            self.em_on = False
            print(f"{ts()} EM_OFF sim={self.simt():.2f}", flush=True)
        sim = self.simt()
        if self.yaw_wz is None or self.yaw_sim is None:
            print(f"{ts()} YAW sim={sim:.2f} wz=none age=none", flush=True)
        else:
            print(f"{ts()} YAW sim={sim:.2f} wz={self.yaw_wz:.3f} age={sim - self.yaw_sim:.2f}", flush=True)
        if self.plan_stamp is None:
            print(f"{ts()} PLAN sim={sim:.2f} stamp=none now={sim:.3f}", flush=True)
        else:
            print(f"{ts()} PLAN sim={sim:.2f} stamp={self.plan_stamp:.3f} now={sim:.3f}", flush=True)
        if self.gt:
            print(
                f"{ts()} TRACK sim={sim:.2f} x={self.gt[0]:.3f} y={self.gt[1]:.3f} yaw={self.gt[2]:.1f}",
                flush=True,
            )
            pose = self.map_pose()
            gt = self.gt
            if pose is None:
                print(
                    f"{ts()} MAP sim={sim:.2f} x=none y=none yaw=none gt_x={gt[0]:.3f} gt_y={gt[1]:.3f} gt_yaw={gt[2]:.1f}",
                    flush=True,
                )
            else:
                print(
                    f"{ts()} MAP sim={sim:.2f} x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.1f} "
                    f"gt_x={gt[0]:.3f} gt_y={gt[1]:.3f} gt_yaw={gt[2]:.1f}",
                    flush=True,
                )

    def on_fb(self, fb):
        r = getattr(fb.feedback, "number_of_recoveries", None)
        if r is not None and r != self.recoveries:
            self.recoveries = r
            print(f"{ts()} RECOVERY n={r} sim={self.simt():.2f}", flush=True)
        left = getattr(fb.feedback, "number_of_poses_remaining", None)
        if left is not None:
            self.poses_remaining = left


def _nav_active(n, cli, timeout_sec):
    """GetState bt_navigator == active. Сервис ещё не поднят или ответ не пришёл — False, не исключение."""
    from lifecycle_msgs.srv import GetState

    if not cli.wait_for_service(timeout_sec=0.0):
        return False
    fut = cli.call_async(GetState.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_sec)
    res = fut.result() if fut.done() else None
    return res is not None and res.current_state.label == "active"


def _odom_tf(tf):
    """Кадр, который ждёт Costmap2DROS::on_activate: canTransform(global_frame, robot_base_frame, TimePointZero)."""
    from rclpy.duration import Duration
    from rclpy.time import Time

    return tf.can_transform(ODOM_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=0.0))


def wait_nav_active(n, tf, get_state, timeout_s=TF_BUFFER_S):
    """Навигатор active и есть TF odom→base_footprint. Иначе одна строка FAIL и SystemExit, без целей.

    ``wait_for_server`` истинен, когда action-сервер создан (on_configure), даже если lifecycle ещё
    activating: Humble simple_action_server отвергает цель без лога, пока сервер не active. controller_server
    не выходит из on_activate, пока local costmap не увидит этот кадр — тогда bt_navigator тоже не active.
    """
    end = time.time() + timeout_s
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        left = max(0.0, end - time.time())
        if _nav_active(n, get_state, min(1.0, left)) and _odom_tf(tf):
            print(f"{ts()} NAV_READY {NAV_NODE}=active tf={ODOM_FRAME}->{BASE_FRAME}", flush=True)
            return
    print(
        f"{ts()} FAIL навигатор не включился: нет active {NAV_NODE} "
        f"или TF {ODOM_FRAME}->{BASE_FRAME} за {timeout_s:.0f} с",
        flush=True,
    )
    raise SystemExit(1)


def wait_ready(n):
    """Ждать истинную позу и /clock (до 15 с), сервер /navigate_to_pose (до 60 с), затем active и TF odom.

    Сервер создан — ещё не повод слать цель: пока навигатор не active или нет кадра costmap, цель не уходит.
    """
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener

    t_end = time.time() + 15
    while (n.gt is None or n.simt() == 0) and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    cl = ActionClient(n, NavigateToPose, "/navigate_to_pose")
    assert cl.wait_for_server(timeout_sec=60), "нет /navigate_to_pose"
    tf = Buffer()
    n.tf_buffer = tf
    n.tf_listener = TransformListener(tf, n, spin_thread=False)  # держим слушателя, пока жива нода
    get_state = n.create_client(GetState, f"/{NAV_NODE}/get_state")
    wait_nav_active(n, tf, get_state)
    return cl


def through_poses_client(n):
    """Клиент /navigate_through_poses (до 60 с)."""
    cl = ActionClient(n, NavigateThroughPoses, "/navigate_through_poses")
    assert cl.wait_for_server(timeout_sec=60), "нет /navigate_through_poses"
    return cl


def _cancel(n, gh, res, why):
    print(f"{ts()} {why} cancel", flush=True)
    c = gh.cancel_goal_async()
    rclpy.spin_until_future_complete(n, c, timeout_sec=5)
    t1 = time.time()
    while not res.done() and time.time() - t1 < 5:
        rclpy.spin_once(n, timeout_sec=0.05)
    return why


def _wait_result(n, gh, timeout, stop=None):
    res = gh.get_result_async()
    w0 = time.time()
    while not res.done():
        rclpy.spin_once(n, timeout_sec=0.05)
        if stop is not None and stop():
            return _cancel(n, gh, res, "INTERRUPTED")
        if time.time() - w0 > timeout:
            return _cancel(n, gh, res, "TIMEOUT")
    return STATUS.get(res.result().status, res.result().status)


def _pose(x, y, yaw):
    """Поза в map (yaw в градусах)."""
    p = PoseStamped()
    p.header.frame_id = MAP_FRAME
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(math.radians(yaw) / 2)
    p.pose.orientation.w = math.cos(math.radians(yaw) / 2)
    return p


def _send(n, cl, name, goal, target, last_xy, timeout, stop):
    """Отправить цель, дождаться итога, GOAL_START/GOAL_END в лог; вернуть (итог, эпизоды защиты)."""
    n.recoveries = 0
    em0 = n.em_episodes
    s_sim, s_fr = n.simt(), nframes()
    print(f"{ts()} GOAL_START {name} target={target} sim={s_sim:.2f} frame={s_fr} gt={n.gt}", flush=True)
    fut = cl.send_goal_async(goal, feedback_callback=n.on_fb)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    result = "REJECTED" if gh is None or not gh.accepted else _wait_result(n, gh, timeout, stop)
    e_sim, e_fr = n.simt(), nframes()
    err = math.dist(n.gt[:2], last_xy) if n.gt else float("nan")
    em = n.em_episodes - em0
    print(
        f"{ts()} GOAL_END {name} result={result} sim={e_sim:.2f} frame={e_fr} dur_sim={e_sim - s_sim:.1f} "
        f"err={err:.3f} em_episodes={em} recoveries={n.recoveries} gt={n.gt}",
        flush=True,
    )
    return result, em


def run_goal(n, cl, name, x, y, yaw, timeout=TIMEOUT, stop=None):
    """Одна цель NavigateToPose (yaw в градусах): GOAL_START/GOAL_END в лог; вернуть (итог, эпизоды защиты).

    ``stop()`` → True (сигнал оператора) — цель отменяется, итог ``INTERRUPTED``.
    """
    g = NavigateToPose.Goal()
    g.pose = _pose(x, y, yaw)
    return _send(n, cl, name, g, f"({x},{y},{yaw})", (x, y), timeout, stop)


def run_lane(n, cl, name, poses, timeout, stop=None):
    """Точки ``poses`` [(x, y, yaw°)] одной целью NavigateThroughPoses: робот не тормозит в каждой.

    Вернуть (итог, эпизоды защиты, осталось точек). Осталось — последний feedback Nav2
    ``number_of_poses_remaining``: пройденную точку убирает RemovePassedGoals (navigate_through_poses.xml,
    радиус 0.35 м), последнюю — только успех. Сколько пройдено, считает ``GoalOutcome.from_route``.
    """
    g = NavigateThroughPoses.Goal()
    g.poses = [_pose(*p) for p in poses]
    n.poses_remaining = len(poses)
    target = "[" + " ".join(f"({x},{y},{yaw})" for x, y, yaw in poses) + "]"
    result, em = _send(n, cl, name, g, target, poses[-1][:2], timeout, stop)
    print(f"{ts()} LANE_FEEDBACK {name} poses={len(poses)} remaining={n.poses_remaining}", flush=True)
    return result, em, n.poses_remaining


def main():
    rclpy.init()
    n = Mon()
    cl = wait_ready(n)
    # ROUTE_GOALS_FILE=/sim_worlds/building.goals.yaml — точки мира вместо GOALS (Depot); порядок — как в argv.
    goals = load_goals(os.environ["ROUTE_GOALS_FILE"]) if os.environ.get("ROUTE_GOALS_FILE") else GOALS
    by_name = {g[0]: g for g in goals}
    order = sys.argv[1:] or [g[0] for g in goals]
    unknown = [name for name in order if name not in by_name]
    assert not unknown, f"нет целей {unknown}; есть {sorted(by_name)}"
    for name, x, y, yaw in [by_name[name] for name in order]:
        run_goal(n, cl, name, x, y, yaw)
        t1 = time.time()
        while time.time() - t1 < 1.5:
            rclpy.spin_once(n, timeout_sec=0.05)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
