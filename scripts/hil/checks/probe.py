#!/usr/bin/env python3
"""Проба HIL-проверок группы А (``scripts/hil/checks/run_checks.sh``): rclpy внутри ``rob-box-hil-gazebo`` на песочнице.

Только пишет raw-строки ``<эпоха UTC> <KIND> sim=<время sim> k=v ...`` в stdout; вердикт строит ``verdicts.py``.
Слушает всегда: статусы ``/navigate_to_pose`` и ``/follow_path`` (NAV_STATUS, FP_STATUS при смене), выход twist_mux
(MUX: ненулевые все, нулевые не чаще 1 Гц), аварийный вход (EM_ON/EM_OFF), правду Gazebo (TRACK, 5 Гц).

    watch   --secs N                          только слушать
    orphan  [--len 1.5] [--timeout 10]        сирота нарочно (#4458): цель NavigateToPose → отмена → своя FollowPath
    nav     (--goal x,y,yaw° | --ahead M) [--timeout 180]  цель NavigateToPose из скрипта (3b из #4365, #4435)
    wall    [--speed 0.1] [--max-s 40] [--push 4]  телеоп /cmd_vel_web к ближайшему препятствию (#4365, #4470)
    backoff [--speed 0.1] [--secs 4]          отъехать назад

Направление выбирает скан (свободное — для orphan/nav --ahead, ближайшее препятствие — для wall), поза в ``map`` —
из TF. Моторов в HIL нет: всё едет в Gazebo.
"""

import argparse
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

NAV_STATUS_TOPIC = "/navigate_to_pose/_action/status"
FP_STATUS_TOPIC = "/follow_path/_action/status"
MUX_OUT_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"  # выход twist_mux (start_twist_mux.sh)
EMERGENCY_TOPIC = "/cmd_vel_emergency"
TELEOP_TOPIC = "/cmd_vel_web"  # вход twist_mux web_ui, priority 50
GT_TOPIC = "/sim/ground_truth/odom"
SCAN_TOPIC = "/scan"
MAP_FRAME, BASE_FRAME = "map", "base_footprint"
CONTROLLER_ID, GOAL_CHECKER_ID = "FollowPath", "general_goal_checker"  # nav2_params.yaml controller_server
STATUS = {v: k.replace("STATUS_", "", 1) for k, v in GoalStatus.__dict__.items() if k.startswith("STATUS_")}
CMD_HZ = 10.0
#: Файл-сигнал «хватит слушать» для ``watch`` (в контейнере; старый удаляется на старте пробы).
STOP_FILE = "/tmp/hil_probe.stop"
#: Сектор «впереди» и окно сглаживания скана, рад.
FRONT_HALF = math.radians(10)
WINDOW_HALF = math.radians(5)


def out(kind, node=None, **kv):
    sim = f" sim={node.simt():.3f}" if node is not None else ""
    rest = "".join(f" {k}={v}" for k, v in kv.items())
    print(f"{time.time():.3f} {kind}{sim}{rest}", flush=True)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def uuid_hex(goal_id):
    return bytes(bytearray(int(b) for b in goal_id.uuid)).hex()


class Probe(Node):
    def __init__(self):
        super().__init__("hil_checks_probe", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.gt = None
        self.scan = None
        self.em_last, self.em_on = 0.0, False
        self.mux_zero_at = 0.0
        self.last_status = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, NAV_STATUS_TOPIC, lambda m: self.on_status("NAV_STATUS", m), qos)
        self.create_subscription(GoalStatusArray, FP_STATUS_TOPIC, lambda m: self.on_status("FP_STATUS", m), qos)
        self.create_subscription(Twist, MUX_OUT_TOPIC, self.on_mux, 10)
        self.create_subscription(Twist, EMERGENCY_TOPIC, self.on_em, 10)
        self.create_subscription(Odometry, GT_TOPIC, self.on_gt, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self.teleop = self.create_publisher(Twist, TELEOP_TOPIC, 10)
        self.create_timer(0.2, self.tick)
        import tf2_ros  # noqa: PLC0415

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)

    def simt(self):
        return self.get_clock().now().nanoseconds / 1e9

    # -- слушатели --------------------------------------------------------------------------------
    def on_status(self, kind, msg):
        for s in msg.status_list:
            gid = uuid_hex(s.goal_info.goal_id)
            name = STATUS.get(s.status, str(s.status))
            if self.last_status.get((kind, gid)) != name:
                self.last_status[(kind, gid)] = name
                out(kind, self, goal=gid, status=name)

    def on_mux(self, m):
        moving = abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3
        if moving or time.time() - self.mux_zero_at >= 1.0:
            if not moving:
                self.mux_zero_at = time.time()
            out("MUX", self, v=f"{m.linear.x:+.3f}", w=f"{m.angular.z:+.3f}")

    def on_em(self, _m):
        self.em_last = time.time()
        if not self.em_on:
            self.em_on = True
            out("EM_ON", self, gt=self.gt_str())

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z)

    def on_scan(self, m):
        self.scan = m

    def tick(self):
        if self.em_on and time.time() - self.em_last > 0.3:
            self.em_on = False
            out("EM_OFF", self, gt=self.gt_str())
        if self.gt:
            x, y, yaw, v, w = self.gt
            out("TRACK", self, x=f"{x:.3f}", y=f"{y:.3f}", yaw=f"{math.degrees(yaw):.1f}", v=f"{v:+.3f}", w=f"{w:+.3f}")

    def gt_str(self):
        return "none" if not self.gt else f"({self.gt[0]:.3f},{self.gt[1]:.3f})"

    # -- скан и поза --------------------------------------------------------------------------------
    def ranges(self):
        """(угол, дальность) годных лучей последнего скана в кадре лидара (крепление лидара yaw=0)."""
        s = self.scan
        res = []
        for i, r in enumerate(s.ranges):
            if s.range_min <= r <= s.range_max and math.isfinite(r):
                res.append((wrap(s.angle_min + i * s.angle_increment), r))
        return res

    def sector(self, center, half):
        rs = [r for a, r in self.ranges() if abs(wrap(a - center)) <= half]
        return min(rs) if rs else math.inf

    def free_direction(self):
        """Угол, где ближайшее препятствие дальше всего (минимум в окне ±5°), и эта дальность."""
        cands = [(self.sector(math.radians(d), WINDOW_HALF), math.radians(d)) for d in range(-180, 180, 10)]
        best = max(cands)
        return best[1], best[0]

    def nearest_obstacle(self, max_dist):
        """Впереди препятствие ближе ``max_dist`` — едем прямо; иначе угол ближайшего препятствия."""
        front = self.sector(0.0, FRONT_HALF)
        if front <= max_dist:
            return 0.0, front
        cands = [(self.sector(math.radians(d), WINDOW_HALF), math.radians(d)) for d in range(-180, 180, 10)]
        dist, ang = min(cands)
        return (ang, dist) if dist <= max_dist else (None, dist)

    def pose_in_map(self):
        import rclpy.time  # noqa: PLC0415

        tf = self.tf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_of(q)

    def pose_stamped(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = MAP_FRAME
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def send_cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.teleop.publish(t)


def spin_for(n, secs, every=None, fn=None, until=None):
    """Крутить ноду ``secs`` с; ``fn`` раз в ``every`` с; ``until()`` → True — выйти раньше."""
    t_end, t_next = time.time() + secs, 0.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.02)
        if fn and time.time() >= t_next:
            fn()
            t_next = time.time() + every
        if until and until():
            return True
    return False


def wait_ready(n, need_tf):
    ok = spin_for(n, 20, until=lambda: n.gt is not None and n.scan is not None and n.simt() > 0)
    if not ok:
        out("ERROR", n, what="нет_правды_Gazebo_скана_или_clock")
        return False
    if need_tf:

        def has_tf():
            try:
                n.pose_in_map()
                return True
            except Exception:  # noqa: BLE001 - tf2 бросает несколько типов
                return False

        if not spin_for(n, 15, until=has_tf):
            out("ERROR", n, what=f"нет_TF_{MAP_FRAME}_{BASE_FRAME}")
            return False
    return True


def wait_future(n, fut, secs):
    spin_for(n, secs, until=fut.done)
    return fut.result() if fut.done() else None


def run_nav_goal(n, client, x, y, yaw, timeout, cancel_after=None):
    """Цель NavigateToPose; ``cancel_after`` — отменить через столько с после EXECUTING. Итог — в NAV_RESULT."""
    out("NAV_SENT", n, x=f"{x:.2f}", y=f"{y:.2f}", yaw=f"{math.degrees(yaw):.0f}")
    goal = NavigateToPose.Goal()
    goal.pose = n.pose_stamped(x, y, yaw)
    gh = wait_future(n, client.send_goal_async(goal), 10)
    if gh is None or not gh.accepted:
        out("NAV_RESULT", n, status="REJECTED")
        return "REJECTED"
    res = gh.get_result_async()
    if cancel_after is not None:
        spin_for(n, cancel_after, until=res.done)
        if not res.done():
            out("NAV_SELF_CANCEL", n, why="orphan_setup")
            wait_future(n, gh.cancel_goal_async(), 5)
    if not spin_for(n, timeout, until=res.done):
        out("NAV_SELF_CANCEL", n, why="timeout")
        wait_future(n, gh.cancel_goal_async(), 5)
        spin_for(n, 5, until=res.done)
    status = STATUS.get(res.result().status, "?") if res.done() else "NO_RESULT"
    out("NAV_RESULT", n, status=status)
    return status


def ahead(n, dist_m, margin=0.9):
    """Точка в ``map`` на ``dist_m`` (не дальше свободного минус ``margin``) в самом свободном направлении."""
    ang, free = n.free_direction()
    x, y, yaw = n.pose_in_map()
    d = min(dist_m, free - margin)
    heading = wrap(yaw + ang)
    out("AHEAD", n, rel_deg=f"{math.degrees(ang):.0f}", free=f"{free:.2f}", dist=f"{d:.2f}")
    return x, y, heading, d


def cmd_watch(n, a):
    """Слушать ``secs`` с или до файла ``STOP_FILE`` (его создаёт run_checks.sh, когда сценарий кончился)."""
    spin_for(n, a.secs, until=lambda: os.path.exists(STOP_FILE))


def cmd_nav(n, a):
    client = ActionClient(n, NavigateToPose, "/navigate_to_pose")
    if not wait_ready(n, need_tf=a.goal is None) or not client.wait_for_server(timeout_sec=30):
        out("ERROR", n, what="нет_/navigate_to_pose")
        return
    if a.goal:
        gx, gy, gyaw = (float(v) for v in a.goal.split(","))
        gyaw = math.radians(gyaw)
    else:
        x, y, heading, d = ahead(n, a.ahead)
        gx, gy, gyaw = x + d * math.cos(heading), y + d * math.sin(heading), heading
    run_nav_goal(n, client, gx, gy, gyaw, a.timeout)
    spin_for(n, 5)


def cmd_orphan(n, a):
    """NavigateToPose → отмена (последняя цель терминальна) → своя FollowPath по прямой: её должен погасить сторож."""
    nav = ActionClient(n, NavigateToPose, "/navigate_to_pose")
    fp = ActionClient(n, FollowPath, "/follow_path")
    if not wait_ready(n, need_tf=True):
        return
    if not nav.wait_for_server(timeout_sec=30) or not fp.wait_for_server(timeout_sec=30):
        out("ERROR", n, what="нет_/navigate_to_pose_или_/follow_path")
        return
    x, y, heading, d = ahead(n, a.len)
    if d < 0.5:
        out("ORPHAN_SKIP", n, why="мало_места", free=f"{d:.2f}")
        return
    run_nav_goal(
        n, nav, x + 0.5 * d * math.cos(heading), y + 0.5 * d * math.sin(heading), heading, 10, cancel_after=2.0
    )
    spin_for(n, 1.0)
    x, y, _ = n.pose_in_map()
    goal = FollowPath.Goal()
    goal.path = Path()
    goal.path.header.frame_id = MAP_FRAME
    goal.path.header.stamp = n.get_clock().now().to_msg()
    steps = max(2, int(d / 0.05))
    goal.path.poses = [
        n.pose_stamped(x + d * i / steps * math.cos(heading), y + d * i / steps * math.sin(heading), heading)
        for i in range(steps + 1)
    ]
    goal.controller_id, goal.goal_checker_id = CONTROLLER_ID, GOAL_CHECKER_ID
    out("FP_SENT", n, len=f"{d:.2f}")
    gh = wait_future(n, fp.send_goal_async(goal), 10)
    if gh is None or not gh.accepted:
        out("ORPHAN_SKIP", n, why="FollowPath_не_принят")
        return
    gid = uuid_hex(gh.goal_id)
    out("FP_ACCEPTED", n, goal=gid)
    res = gh.get_result_async()
    if not spin_for(n, a.timeout, until=res.done):
        out("FP_SELF_CANCEL", n, goal=gid, why="сторож_не_погасил")
        wait_future(n, gh.cancel_goal_async(), 5)
        spin_for(n, 5, until=res.done)
    out("FP_RESULT", n, goal=gid, status=STATUS.get(res.result().status, "?") if res.done() else "NO_RESULT")
    spin_for(n, 3)


def cmd_wall(n, a):
    """Повернуться к ближайшему препятствию и ехать на него ``speed``; после EM_ON толкать ещё ``push`` с."""
    if not wait_ready(n, need_tf=False):
        return
    ang, dist = n.nearest_obstacle(a.max_dist)
    if ang is None:
        out("WALL_NOT_FOUND", n, nearest=f"{dist:.2f}", max=a.max_dist)
        return
    out("WALL", n, rel_deg=f"{math.degrees(ang):.0f}", dist=f"{dist:.2f}")
    if abs(ang) > math.radians(5) and not rotate(n, wrap(n.gt[2] + ang)):
        return
    out("SCAN_START", n, front=f"{n.sector(0.0, FRONT_HALF):.3f}")
    out("CMD", n, v=f"{a.speed:+.2f}", w="+0.00")
    t0 = time.time()
    em_seen = []

    def push():
        n.send_cmd(a.speed, 0.0)
        if n.em_on and not em_seen:
            em_seen.append(time.time())

    spin_for(n, a.max_s, 1.0 / CMD_HZ, push, until=lambda: em_seen and time.time() - em_seen[0] >= a.push)
    for _ in range(5):
        n.send_cmd(0.0, 0.0)
    out("PUSH_END", n, secs=f"{time.time() - t0:.1f}", em="yes" if em_seen else "no")
    spin_for(n, 2)
    out("SCAN_END", n, front=f"{n.sector(0.0, FRONT_HALF):.3f}")


def rotate(n, target_yaw, w=0.4, tol=math.radians(4), secs=30):
    """Разворот на месте по правде Gazebo (скид-стир в симе проскальзывает — по команде не считаем)."""

    def turn():
        err = wrap(target_yaw - n.gt[2])
        n.send_cmd(0.0, math.copysign(w, err))

    done = spin_for(n, secs, 1.0 / CMD_HZ, turn, until=lambda: abs(wrap(target_yaw - n.gt[2])) < tol)
    for _ in range(5):
        n.send_cmd(0.0, 0.0)
    err = math.degrees(wrap(target_yaw - n.gt[2]))
    out("ROTATE_DONE" if done else "ROTATE_FAIL", n, err_deg=f"{err:.1f}")
    spin_for(n, 1.0)
    return done


def cmd_backoff(n, a):
    if not wait_ready(n, need_tf=False):
        return
    out("CMD", n, v=f"{-a.speed:+.2f}", w="+0.00")
    spin_for(n, a.secs, 1.0 / CMD_HZ, lambda: n.send_cmd(-a.speed, 0.0))
    for _ in range(5):
        n.send_cmd(0.0, 0.0)
    out("PUSH_END", n, secs=f"{a.secs:.1f}", em="-")
    spin_for(n, 1.5)


def parse_args(argv):
    ap = argparse.ArgumentParser(description="проба HIL-проверок группы А")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("watch").add_argument("--secs", type=float, default=60)
    p = sub.add_parser("orphan")
    p.add_argument("--len", type=float, default=1.5)
    p.add_argument("--timeout", type=float, default=10)
    p = sub.add_parser("nav")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--goal", help="x,y,yaw° в map")
    g.add_argument("--ahead", type=float)
    p.add_argument("--timeout", type=float, default=180)
    p = sub.add_parser("wall")
    p.add_argument("--speed", type=float, default=0.1)
    p.add_argument("--max-s", dest="max_s", type=float, default=40)
    p.add_argument("--max-dist", dest="max_dist", type=float, default=3.5)
    p.add_argument("--push", type=float, default=4.0)
    p = sub.add_parser("backoff")
    p.add_argument("--speed", type=float, default=0.1)
    p.add_argument("--secs", type=float, default=4)
    return ap.parse_args(argv)


COMMANDS = {"watch": cmd_watch, "nav": cmd_nav, "orphan": cmd_orphan, "wall": cmd_wall, "backoff": cmd_backoff}


def main(argv=None):
    a = parse_args(sys.argv[1:] if argv is None else argv)
    if a.cmd == "watch" and os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)
    rclpy.init()
    n = Probe()
    out("PROBE_START", n, cmd=a.cmd)
    try:
        COMMANDS[a.cmd](n, a)
    finally:
        out("PROBE_DONE", n)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
