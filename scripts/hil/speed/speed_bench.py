#!/usr/bin/env python3
"""speed_bench — один прогон сцены набора «предельная скорость» в HIL (решение Шифу 09.11).

Запуск в контейнере gazebo песочницы (окружение zenoh робота уже есть), рядом ``speed_verdict.py``::

    docker exec rob-box-hil-gazebo /entrypoint.sh python3 /tmp/speed/speed_bench.py \
        --world polygon --spawn 0 0 0 --vmax 0.5 --tasks 'g:E1:3,0,0@150;o:O1:-10.5,0,3.1416@120' --out /tmp/sp/x

Задачи (система мира; ``map`` rtabmap = поза спавна, цель пересчитывается spawn⁻¹ ∘ цель):
  ``g:<имя>:x,y,yaw@таймаут``                          — NavigateToPose;
  ``o:<имя>:x,y,yaw@таймаут[,после_м[,впереди_м]]``     — едем к цели; когда пройдено ``после_м`` (3.0) и
      скорость ≥ 0.9·vmax (или пройдено ещё 3 м), перед носом в ``впереди_м`` (1.5) появляется стенка
      3 × 0.2 × 1 м (``ign service /world/<w>/create``); после остановки цель отменяется, стенка убирается.
После первой проваленной цели ``g`` остальные цели — SKIPPED (робот уже не там, где ждёт набор).

Метрики считает ``speed_verdict`` (там же пороги). Пишет ``<out>/result.json`` (список задач) и ``track.csv``.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import speed_verdict as sv  # noqa: E402
from rob_box_sim.geometry2d import Pose2D, clearance  # noqa: E402
from rob_box_sim.world_sdf import load_world_boxes  # noqa: E402

ODOM_LOST_COV = 9999.0  # как rob_box_mcp_tools.core.localization_check
POSE_LAG_S = 0.6  # поза сверяется с правдой с задержкой: TF на этот штамп уже пришёл
WALL_NAME = "speed_wall"


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder:
    """Правда Gazebo, ``/odom`` (ICP), ``/cmd_vel``, ``/cmd_vel_emergency`` и синхронная ошибка позы."""

    def __init__(self, nav, spawn: Pose2D):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_ros import Buffer, TransformListener

        self.nav = nav
        self.spawn = spawn
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self._tfl = TransformListener(self.tf, nav, spin_thread=False)
        self.truth: list[tuple[float, float, float, float]] = []  # t, x, y, yaw (мир)
        self.truth_t: list[float] = []
        self.odom: list[tuple[float, bool]] = []  # t, lost
        self.cmds: list[tuple[float, float]] = []  # wall t, v (контроллер на Pi крутится по стене)
        self.estop: list[float] = []
        self.pose_err: list[tuple[float, float]] = []  # t, ошибка м
        self.pose_miss: list[float] = []  # t кадров правды без TF на штамп
        self._next_pose_t = 0.0
        self._pose_i = 0
        nav.create_subscription(Odometry, "/sim/ground_truth/odom", self._on_truth, 50)
        nav.create_subscription(Odometry, "/odom", self._on_odom, 50)
        nav.create_subscription(Twist, "/cmd_vel", lambda m: self.cmds.append((time.monotonic(), m.linear.x)), 50)
        nav.create_subscription(Twist, "/cmd_vel_emergency", lambda m: self.estop.append(self.now()), 20)

    def now(self) -> float:
        return self.nav.get_clock().now().nanoseconds / 1e9

    def _on_truth(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.truth_t.append(t)
        self.truth.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        c = msg.pose.covariance[0]
        self.odom.append((t, not math.isfinite(c) or c >= ODOM_LOST_COV))

    def pose_step(self) -> None:
        """Сверить правду с TF ``map → base_footprint`` на тот же штамп (раз в 0.1 с sim)."""
        horizon = self.now() - POSE_LAG_S
        while self._pose_i < len(self.truth) and self.truth[self._pose_i][0] <= horizon:
            t, x, y, _ = self.truth[self._pose_i]
            self._pose_i += 1
            if t < self._next_pose_t:
                continue
            self._next_pose_t = t + 0.1
            m = self.map_pose_at(t)
            if m is None:
                self.pose_miss.append(t)
                continue
            self.pose_err.append((t, sv.pose_error((x, y), self.spawn, m)))

    def map_pose_at(self, t: float):
        """``map → base_footprint`` на штамп ``t``. rtabmap шлёт ``map → icp_odom`` со штампом своего кадра
        (реже скана), и цепочка на точный штамп часто не находится: тогда ``icp_odom → base_footprint`` на ``t``
        (ICP 10 Гц + колёса 50 Гц) и поправка карты ``map → icp_odom`` последняя (она меняется медленно)."""
        m = self.lookup("map", "base_footprint", t)
        if m is not None:
            return m
        ib = self.lookup("icp_odom", "base_footprint", t)
        mi = self.lookup("map", "icp_odom", None)
        if ib is None or mi is None:
            return None
        p = Pose2D(*mi).compose(Pose2D(*ib))
        return (p.x, p.y, p.yaw)

    def lookup(self, parent: str, child: str, t: float | None):
        stamp = Time() if t is None else Time(nanoseconds=int(t * 1e9))
        try:
            tr = self.tf.lookup_transform(parent, child, stamp)
        except Exception:  # noqa: BLE001 — нет TF на штамп: считаем промахом
            return None
        q = tr.transform
        return (q.translation.x, q.translation.y, yaw_of(q.rotation))

    def slice_truth(self, t0: float, t1: float):
        i = bisect.bisect_left(self.truth_t, t0)
        j = bisect.bisect_right(self.truth_t, t1)
        return self.truth[i:j]


def spin_for(nav, rec: Recorder, s: float) -> None:
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.05)
        rec.pose_step()


def pose_stamped(nav, g: Pose2D):
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y = g.x, g.y
    ps.pose.orientation.z, ps.pose.orientation.w = math.sin(g.yaw / 2), math.cos(g.yaw / 2)
    return ps


def parse_tasks(spec: str) -> list[dict]:
    """'g:E1:3,0,0@150;o:O1:-10.5,0,3.14@120,3.0,1.5' → [{kind, name, goal, timeout, after_m, ahead_m}]."""
    out = []
    for part in (p.strip() for p in spec.split(";") if p.strip()):
        kind, name, rest = part.split(":", 2)
        xyz, _, tail = rest.partition("@")
        extra = [float(v) for v in tail.split(",")] if tail else []
        out.append(
            {
                "kind": {"g": "goal", "o": "obstacle"}[kind],
                "name": name,
                "goal": [float(v) for v in xyz.split(",")],
                "timeout": extra[0] if extra else 150.0,
                "after_m": extra[1] if len(extra) > 1 else 3.0,
                "ahead_m": extra[2] if len(extra) > 2 else 1.5,
            }
        )
    return out


# ── стенка в Gazebo ─────────────────────────────────────────────────────────


def gz_world() -> str:
    out = subprocess.run(["ign", "service", "-l"], capture_output=True, text=True, timeout=20).stdout
    for line in out.splitlines():
        if line.startswith("/world/") and line.endswith("/create"):
            return line.split("/")[2]
    raise RuntimeError("нет сервиса /world/<w>/create")


def wall_sdf(b) -> str:
    size = f"{b.sx} {b.sy} {b.height}"
    geom = f"<geometry><box><size>{size}</size></box></geometry>"
    return (
        f"<sdf version='1.8'><model name='{WALL_NAME}'><static>true</static>"
        f"<pose>{b.cx:.4f} {b.cy:.4f} {b.height / 2:.3f} 0 0 {b.yaw:.4f}</pose><link name='l'>"
        f"<collision name='c'>{geom}</collision><visual name='v'>{geom}"
        "<material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>"
        "</visual></link></model></sdf>"
    )


def gz_call(world: str, service: str, reqtype: str, req: str) -> bool:
    r = subprocess.run(
        [
            "ign",
            "service",
            "-s",
            f"/world/{world}/{service}",
            "--reqtype",
            reqtype,
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ],
        capture_output=True,
        text=True,
        timeout=15,
    )
    return "true" in r.stdout


# ── задачи ──────────────────────────────────────────────────────────────────


class Runner:
    def __init__(self, nav, rec: Recorder, boxes, vmax: float, world: str):
        self.nav, self.rec, self.boxes, self.vmax, self.world = nav, rec, boxes, vmax, world

    def send(self, goal_world: list[float]) -> bool:
        gm = self.rec.spawn.inverse().compose(Pose2D(*goal_world))
        return self.nav.goToPose(pose_stamped(self.nav, gm))

    def wait(self, t0: float, timeout: float, hook=None) -> tuple[str, int]:
        from nav2_simple_commander.robot_navigator import TaskResult

        rec_n = 0
        while not self.nav.isTaskComplete():
            self.rec.pose_step()
            fb = self.nav.getFeedback()
            if fb is not None:
                rec_n = max(rec_n, int(fb.number_of_recoveries))
            if hook is not None and hook():
                self.nav.cancelTask()
                spin_for(self.nav, self.rec, 1.0)
                return "CANCELED_BY_BENCH", rec_n
            if self.rec.now() - t0 > timeout:
                self.nav.cancelTask()
                spin_for(self.nav, self.rec, 1.0)
                return "TIMEOUT", rec_n
        r = self.nav.getResult()
        return {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED", TaskResult.FAILED: "FAILED"}.get(
            r, f"UNKNOWN({r})"
        ), rec_n

    def goal(self, task: dict) -> dict:
        t0, w0 = self.rec.now(), time.monotonic()
        ok = self.send(task["goal"])
        status, recov = self.wait(t0, task["timeout"]) if ok else ("REJECTED", 0)
        return self.metrics(task, t0, w0, status, recov, extra_boxes=())

    def obstacle(self, task: dict) -> dict:
        t0, w0 = self.rec.now(), time.monotonic()
        st = ObstacleState(self, task, t0)
        ok = self.send(task["goal"])
        status, recov = self.wait(t0, task["timeout"], hook=st.step) if ok else ("REJECTED", 0)
        if st.wall is not None:
            st.removed = gz_call(self.world, "remove", "ignition.msgs.Entity", f'name: "{WALL_NAME}" type: MODEL')
            spin_for(self.nav, self.rec, 2.0)
        res = self.metrics(task, t0, w0, status, recov, extra_boxes=(st.wall,) if st.wall else ())
        res["obstacle"] = st.report()
        return res

    def metrics(self, task: dict, t0: float, w0: float, status: str, recov: int, extra_boxes) -> dict:
        t1, w1 = self.rec.now(), time.monotonic()
        spin_for(self.nav, self.rec, POSE_LAG_S + 0.4)  # догнать сверку позы до t1
        tr = self.rec.slice_truth(t0, t1)
        boxes = list(self.boxes) + list(extra_boxes)
        clr = min((clearance(Pose2D(x, y, yw), boxes) for (_, x, y, yw) in tr[::5]), default=math.inf)
        vs = [v for _, v in sv.speeds([(t, x, y) for t, x, y, _ in tr])]
        ev = [t for t in self.rec.estop if t0 <= t <= t1]
        lost = [f for t, f in self.rec.odom if t0 <= t <= t1]
        total, longest = sv.lost_runs(lost)
        gw = Pose2D(*task["goal"])
        end = tr[-1] if tr else (t1, math.nan, math.nan, math.nan)
        return {
            "name": task["name"],
            "kind": task["kind"],
            "goal_world": task["goal"],
            "status": status,
            "t_start": round(t0, 2),
            "sim_s": round(t1 - t0, 1),
            "wall_s": round(w1 - w0, 1),
            "recoveries": recov,
            "guard_stops": sum(1 for i, t in enumerate(ev) if i == 0 or t - ev[i - 1] > 0.5),
            "err_xy_m": round(math.hypot(end[1] - gw.x, end[2] - gw.y), 3),
            "truth_end": [round(v, 3) for v in end[1:]],
            "path_m": round(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(tr, tr[1:])), 2),
            "v_max": sv._r(max(vs, default=None)),
            "v_p95": sv._r(sv.percentile(vs, 95)),
            "min_clearance_m": None if math.isinf(clr) else round(clr, 3),
            "pose_err": {
                **sv.spread_stats([e for t, e in self.rec.pose_err if t0 <= t <= t1]),
                "miss": sum(1 for t in self.rec.pose_miss if t0 <= t <= t1),
            },
            "icp": {"frames": len(lost), "lost": total, "longest_lost_run": longest},
            "cmd_interval": sv.cmd_interval_stats([c for c in self.rec.cmds if w0 <= c[0] <= w1]),
        }


class ObstacleState:
    """Когда ставить стенку и когда робот встал перед ней."""

    def __init__(self, runner: Runner, task: dict, t0: float):
        self.r, self.task, self.t0 = runner, task, t0
        self.wall = None
        self.t_spawn = None
        self.removed = False
        self.spawned = False
        self.wall_ahead_m = None

    def _travelled(self) -> float:
        tr = self.r.rec.slice_truth(self.t0, math.inf)
        return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(tr, tr[1:]))

    def _speed(self) -> float:
        tr = self.r.rec.truth[-40:]
        vs = sv.speeds([(t, x, y) for t, x, y, _ in tr])
        return vs[-1][1] if vs else 0.0

    def step(self) -> bool:
        """Хук ожидания цели: True — отменить цель (робот встал у стенки или 15 с после неё)."""
        if self.wall is None:
            d = self._travelled()
            if d >= self.task["after_m"] and (self._speed() >= 0.9 * self.r.vmax or d >= self.task["after_m"] + 3.0):
                self._spawn()
            return False
        tr = self.r.rec.slice_truth(self.t_spawn, math.inf)
        if self.r.rec.now() - self.t_spawn > 15.0:
            return True
        ob = sv.obstacle_metrics(tr, self.wall, self.t_spawn)
        return bool(ob.get("stopped")) and self.r.rec.now() - self.t_spawn > 3.0

    def _spawn(self) -> None:
        _, x, y, yaw = self.r.rec.truth[-1]
        wall = sv.wall_ahead(Pose2D(x, y, yaw), self.task["ahead_m"])
        self.spawned = gz_call(self.r.world, "create", "ignition.msgs.EntityFactory", f'sdf: "{wall_sdf(wall)}"')
        self.wall = wall
        self.wall_ahead_m = self.task["ahead_m"]
        self.t_spawn = self.r.rec.truth[-1][0]  # штамп правды после ответа сервиса
        print(f"WALL spawned={self.spawned} t={self.t_spawn:.2f} at=({wall.cx:.2f},{wall.cy:.2f})", flush=True)

    def report(self) -> dict:
        if self.wall is None:
            return {"spawned": False}
        tr = self.r.rec.slice_truth(self.t_spawn, math.inf)
        ob = sv.obstacle_metrics(tr, self.wall, self.t_spawn)
        ob.update(
            {
                "spawned": self.spawned,
                "removed": self.removed,
                "ahead_m": self.wall_ahead_m,
                "t_spawn": round(self.t_spawn, 2),
                "wall": [round(self.wall.cx, 3), round(self.wall.cy, 3), round(self.wall.yaw, 3)],
            }
        )
        return ob


def _wait_nav2(nav, timeout_s: float = 180.0) -> None:
    """Как ``waitUntilNav2Active``, но с пределом: сбой discovery не должен висеть до таймаута прогона (09.11:
    19 мин «bt_navigator/get_state service not available» при активном nav2 на Main Pi)."""
    from lifecycle_msgs.srv import GetState

    cli = nav.create_client(GetState, "/bt_navigator/get_state")
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if cli.wait_for_service(timeout_sec=2.0):
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=5.0)
            if fut.result() is not None and fut.result().current_state.label == "active":
                return
    print(f"SPEEDBENCH_NO_NAV2 bt_navigator не active за {timeout_s:.0f} с", flush=True)
    raise SystemExit(3)


def controller_params(nav) -> dict:
    from rcl_interfaces.srv import GetParameters

    names = [
        "FollowPath.max_vel_x",
        "FollowPath.max_speed_xy",
        "FollowPath.acc_lim_x",
        "FollowPath.decel_lim_x",
        "FollowPath.max_vel_theta",
    ]
    cli = nav.create_client(GetParameters, "/controller_server/get_parameters")
    if not cli.wait_for_service(timeout_sec=10.0):
        return {"error": "нет /controller_server/get_parameters"}
    fut = cli.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(nav, fut, timeout_sec=10.0)
    if fut.result() is None:
        return {"error": "нет ответа"}
    return {n: v.double_value for n, v in zip(names, fut.result().values)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, help="мир для коробок зазора: polygon, turn90; depot — без коробок")
    ap.add_argument("--spawn", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    ap.add_argument("--vmax", type=float, required=True)
    ap.add_argument("--tasks", required=True)
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.parameter import Parameter

    sdf = Path(f"/sim_worlds/{args.world}.sdf")
    boxes = load_world_boxes(sdf) if sdf.exists() else []
    spawn = Pose2D(*args.spawn)
    rclpy.init()
    nav = BasicNavigator(node_name="shidi_speedbench")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    rec = Recorder(nav, spawn)
    w0 = time.monotonic()
    _wait_nav2(nav)
    dl = time.monotonic() + 120
    while time.monotonic() < dl and not (rec.truth and rec.lookup("map", "base_footprint", None)):
        rclpy.spin_once(nav, timeout_sec=0.1)
    spin_for(nav, rec, args.settle)
    head = {
        "label": args.label,
        "world": args.world,
        "spawn": list(args.spawn),
        "vmax": args.vmax,
        "ready_wall_s": round(time.monotonic() - w0, 1),
        "params": controller_params(nav),
    }
    print("SPEEDBENCH_HEAD " + json.dumps(head, ensure_ascii=False), flush=True)
    runner = Runner(nav, rec, boxes, args.vmax, gz_world())
    args.out.mkdir(parents=True, exist_ok=True)
    results, failed = [], False
    for task in parse_tasks(args.tasks):
        if failed and task["kind"] == "goal":
            r = {"name": task["name"], "kind": "goal", "goal_world": task["goal"], "status": "SKIPPED"}
        else:
            r = runner.obstacle(task) if task["kind"] == "obstacle" else runner.goal(task)
            failed = failed or (task["kind"] == "goal" and bool(sv.task_failures(r)))
        r = {**head, **r}
        results.append(r)
        print("SPEEDBENCH " + json.dumps(r, ensure_ascii=False, default=float), flush=True)
        (args.out / "result.json").write_text(
            json.dumps(results, ensure_ascii=False, indent=1, default=float), encoding="utf-8"
        )
    with open(args.out / "track.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "yaw"])
        w.writerows(rec.truth[::5])
    with open(args.out / "pose_err.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "err"])
        w.writerows(rec.pose_err)
    nav.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
