#!/usr/bin/env python3
"""Маршрут из целей NavigateToPose по одной; лог: sim time, номер кадра камеры, поза gt, стопы защиты."""
import datetime
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

GOALS = [  # Depot; M — картография (карта пуста), R — маршрут по построенной карте
    ("M1", -6.0, 4.0, 0),
    ("M2", -1.0, 4.5, 0),
    ("M3", -1.0, -1.0, 0),
    ("M4", -6.0, -4.5, 180),
    ("M5", 3.0, -0.9, 0),
    ("M6", 9.0, -0.9, 0),
    ("R1", 4.25, 3.5, 90),
    ("R2", -2.0, 0.0, 180),
    ("R3", -9.0, 4.0, 90),
    ("R4", -9.0, -4.5, 270),
    ("R5", -4.0, -3.0, 0),
    ("R6", 3.0, -0.9, 0),
    ("R7", -10.5, 0.0, 180),
]
TIMEOUT = 200.0


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
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self.on_gt, 10)
        self.create_subscription(Twist, "/cmd_vel_emergency", self.on_em, 10)
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

    def tick(self):
        if self.em_on and time.time() - self.em_last > 0.3:
            self.em_on = False
            print(f"{ts()} EM_OFF sim={self.simt():.2f}", flush=True)
        if self.gt:
            print(
                f"{ts()} TRACK sim={self.simt():.2f} x={self.gt[0]:.3f} y={self.gt[1]:.3f} yaw={self.gt[2]:.1f}",
                flush=True,
            )

    def on_fb(self, fb):
        r = getattr(fb.feedback, "number_of_recoveries", None)
        if r is not None and r != self.recoveries:
            self.recoveries = r
            print(f"{ts()} RECOVERY n={r} sim={self.simt():.2f}", flush=True)


def wait_ready(n):
    """Ждать истинную позу и /clock (до 15 с), затем сервер /navigate_to_pose (до 60 с)."""
    t_end = time.time() + 15
    while (n.gt is None or n.simt() == 0) and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    cl = ActionClient(n, NavigateToPose, "/navigate_to_pose")
    assert cl.wait_for_server(timeout_sec=60), "нет /navigate_to_pose"
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


def run_goal(n, cl, name, x, y, yaw, timeout=TIMEOUT, stop=None):
    """Одна цель NavigateToPose (yaw в градусах): GOAL_START/GOAL_END в лог; вернуть (итог, эпизоды защиты).

    ``stop()`` → True (сигнал оператора) — цель отменяется, итог ``INTERRUPTED``.
    """
    n.recoveries = 0
    em0 = n.em_episodes
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = x, y
    g.pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
    g.pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
    s_sim, s_fr = n.simt(), nframes()
    print(f"{ts()} GOAL_START {name} target=({x},{y},{yaw}) sim={s_sim:.2f} frame={s_fr} gt={n.gt}", flush=True)
    fut = cl.send_goal_async(g, feedback_callback=n.on_fb)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    result = "REJECTED" if gh is None or not gh.accepted else _wait_result(n, gh, timeout, stop)
    e_sim, e_fr = n.simt(), nframes()
    err = math.dist(n.gt[:2], (x, y)) if n.gt else float("nan")
    em = n.em_episodes - em0
    print(
        f"{ts()} GOAL_END {name} result={result} sim={e_sim:.2f} frame={e_fr} dur_sim={e_sim - s_sim:.1f} "
        f"err={err:.3f} em_episodes={em} recoveries={n.recoveries} gt={n.gt}",
        flush=True,
    )
    return result, em


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
