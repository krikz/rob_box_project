"""Стенд сценариев навигации в симе: цели мира → NavigateToPose → метрики.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §6 шаг 4.

Запускается в контейнере ``bench`` профиля docker/sim (сеть zenoh-router, sim time),
когда Gazebo с миром ``--world`` и боевой Nav2/rtabmap уже подняты::

    python3 -m rob_box_sim.nav_bench --world polygon --out /results

Порядок:
  1. ждёт Nav2 (bt_navigator active);
  2. переводит rtabmap в картирование тем же сервисом, что на роботе
     (``/rtabmap/rtabmap/set_mode_mapping``): карта мира с нуля;
  3. по очереди шлёт цели сценария (система ``map`` = мир: спавн в (0, 0, 0)),
     пишет истинную позу, TF ``map → base_footprint``, ``/cmd_vel``, ``/scan``,
     число recovery; по таймауту — отмена и статус TIMEOUT;
  4. после провала при ``stop_on_failure`` остальные цели — SKIPPED;
  5. в конце снимает ``/rtabmap/map`` и сверяет с миром (map_check);
  6. пишет ``<out>/<мир>.json``, ``.csv``, ``.md`` и ``<мир>_map.pgm``; после каждой цели —
     позы ``<мир>_<цель>_truth.csv`` / ``_estimate.csv`` (t, x, y, yaw) для анимации.

Сравнение контроллеров и правка параметров — шаг 6; здесь только замер.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import yaml

from rob_box_sim import map_check
from rob_box_sim.geometry2d import Pose2D
from rob_box_sim.nav_metrics import GoalRecord, goal_metrics, summary_markdown
from rob_box_sim.world_sdf import load_world_boxes

DEFAULT_SCENARIOS = "/ws/install/rob_box_sim/share/rob_box_sim/scenarios/baseline.yaml"
SET_MAPPING_SERVICE = "/rtabmap/rtabmap/set_mode_mapping"
GLOBAL_COSTMAP_TOPIC = "/global_costmap/costmap"


def load_scenario(path: Path, world: str, worlds_dir: Path) -> dict:
    """Цели и настройки мира из yaml сценариев; ``goals_from`` — файл целей рядом с миром."""
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    defaults = doc.get("defaults", {})
    if world not in doc["worlds"]:
        raise SystemExit(f"мира {world!r} нет в {path}: есть {sorted(doc['worlds'])}")
    sc = {**defaults, **doc["worlds"][world]}
    if "goals_from" in sc:
        extra = yaml.safe_load((worlds_dir / sc["goals_from"]).read_text(encoding="utf-8"))
        sc["goals"] = extra["goals"]
    if not sc.get("goals"):
        raise SystemExit(f"у мира {world!r} нет целей")
    return sc


def timeout_for(sc: dict, start: Pose2D, goal: Pose2D) -> float:
    return float(sc["timeout_base_s"]) + start.distance_to(goal) / float(sc["timeout_speed_mps"])


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _tilt(q) -> float:
    """Наклон корпуса от вертикали, рад (крен и тангаж вместе): угол оси z тела с мировой z."""
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.acos(max(-1.0, min(1.0, zz)))


class Recorder:
    """Подписки стенда на узле навигатора: колбэки крутит сам BasicNavigator."""

    def __init__(self, node):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.node = node
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, node, spin_thread=False)
        self.rec: GoalRecord | None = None
        self.last_truth: tuple[float, float, float] | None = None
        node.create_subscription(Odometry, "/sim/ground_truth/odom", self._on_truth, 20)
        node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        node.create_subscription(Twist, "/cmd_vel_nav", self._on_cmd_raw, 20)
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def estimate(self):
        from rclpy.time import Time

        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:  # noqa: BLE001 — нет TF = нет оценки, это и пишем
            return None
        tr = t.transform
        return (tr.translation.x, tr.translation.y, _yaw(tr.rotation))

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.last_truth = (p.position.x, p.position.y, _yaw(p.orientation))
        if self.rec is not None:
            self.rec.truth.append((self.now(), *self.last_truth))
            self.rec.estimate.append(self.estimate())
            self.rec.tilt_max_rad = max(self.rec.tilt_max_rad, _tilt(p.orientation))

    def _on_cmd(self, msg):
        if self.rec is not None:
            self.rec.cmd.append((self.now(), msg.linear.x, msg.angular.z))

    def _on_cmd_raw(self, msg):
        if self.rec is not None:
            self.rec.cmd_raw.append((self.now(), msg.linear.x, msg.angular.z))

    def _on_scan(self, msg):
        if self.rec is not None:
            finite = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            self.rec.scan_min.append((self.now(), min(finite) if finite else math.inf))


def _spin_for(node, seconds: float) -> None:
    import rclpy

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def set_mapping(node, timeout_s: float = 60.0) -> bool:
    import rclpy
    from std_srvs.srv import Empty

    cli = node.create_client(Empty, SET_MAPPING_SERVICE)
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    fut = cli.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
    return fut.done()


def _pose_stamped(nav, goal: Pose2D):
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x = goal.x
    ps.pose.position.y = goal.y
    ps.pose.orientation.z = math.sin(goal.yaw / 2)
    ps.pose.orientation.w = math.cos(goal.yaw / 2)
    return ps


def run_goal(nav, recorder: Recorder, name: str, goal: Pose2D, timeout_s: float) -> GoalRecord:
    from nav2_simple_commander.robot_navigator import TaskResult

    rec = GoalRecord(name, goal, "RUNNING", recorder.now(), recorder.now())
    recorder.rec = rec
    if not nav.goToPose(_pose_stamped(nav, goal)):
        rec.status = "REJECTED"
    else:
        timed_out = False
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                rec.recoveries = max(rec.recoveries, int(fb.number_of_recoveries))
            if recorder.now() - rec.t_start > timeout_s:
                timed_out = True
                nav.cancelTask()
                break
        if timed_out:
            rec.status = "TIMEOUT"
            _spin_for(nav, 1.0)
        else:
            res = nav.getResult()
            rec.status = {
                TaskResult.SUCCEEDED: "SUCCEEDED",
                TaskResult.CANCELED: "CANCELED",
                TaskResult.FAILED: "FAILED",
            }.get(res, f"UNKNOWN({res})")
    rec.t_end = recorder.now()
    recorder.rec = None
    return rec


def dump_costmap(nav, recorder: Recorder, out: Path, stem: str) -> int | None:
    """Глобальная costmap в миг провала: значения клеток (pgm) и стоимость под роботом.

    Стоимость под роботом 99/100 — центр робота в зоне inscribed/lethal: планировщик
    не выводит путь из такой клетки, контроллер не находит допустимой траектории.
    """
    grid = map_check.grab_map(nav, GLOBAL_COSTMAP_TOPIC, timeout_s=10.0)
    if grid is None:
        print(f"[bench] COSTMAP {stem}: {GLOBAL_COSTMAP_TOPIC} не пришла", flush=True)
        return None
    out.mkdir(parents=True, exist_ok=True)
    map_check.write_raw_pgm(grid, out / f"{stem}_costmap.pgm")
    est = recorder.estimate()
    under = grid.value_at(est[0], est[1]) if est else None
    print(f"[bench] COSTMAP {stem} {grid.width}x{grid.height} res={grid.resolution:.3f} "
          f"origin=({grid.origin_x:.2f},{grid.origin_y:.2f}) robot={est} cost_under_robot={under}", flush=True)
    return under


def write_trajectories(rec: GoalRecord, out: Path, stem: str) -> list[Path]:
    """Позы цели для анимации: ``<stem>_truth.csv`` и ``<stem>_estimate.csv`` (t, x, y, yaw).

    truth — истинная поза Gazebo, estimate — TF ``map → base_footprint`` в те же моменты
    (момент без TF пропускается). Цель без поз (SKIPPED) файлов не даёт.
    """
    if not rec.truth:
        return []
    out.mkdir(parents=True, exist_ok=True)
    est = [(t, *e) for (t, *_), e in zip(rec.truth, rec.estimate) if e is not None]
    paths = []
    for kind, samples in (("truth", rec.truth), ("estimate", est)):
        path = out / f"{stem}_{kind}.csv"
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw"])
            w.writerows([[f"{v:.4f}" for v in row] for row in samples])
        paths.append(path)
    return paths


def write_outputs(out: Path, world: str, rows: list[dict], extra: dict) -> None:
    out.mkdir(parents=True, exist_ok=True)
    stem = world.replace("/", "_")
    (out / f"{stem}.json").write_text(
        json.dumps({"world": world, "goals": rows, **extra}, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with open(out / f"{stem}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (out / f"{stem}.md").write_text(summary_markdown(rows) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Стенд сценариев навигации (сим)")
    ap.add_argument("--world", required=True, help="имя мира как SIM_WORLD: polygon | local/robot_map ...")
    ap.add_argument("--scenarios", type=Path, default=Path(DEFAULT_SCENARIOS))
    ap.add_argument("--worlds-dir", type=Path, default=Path("/sim_worlds"))
    ap.add_argument("--out", type=Path, default=Path("/results"))
    ap.add_argument("--label", default="baseline", help="метка прогона в JSON (например dwb)")
    args = ap.parse_args(argv)

    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.parameter import Parameter

    sc = load_scenario(args.scenarios, args.world, args.worlds_dir)
    boxes = load_world_boxes(args.worlds_dir / f"{args.world}.sdf")

    rclpy.init()
    nav = BasicNavigator(node_name="sim_nav_bench")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    recorder = Recorder(nav)

    t0 = time.monotonic()
    # amcl нет (локализация — rtabmap); Humble ждёт localizer всегда, отдаём ему тот же bt_navigator
    nav.waitUntilNav2Active(navigator="bt_navigator", localizer="bt_navigator")
    mapping = set_mapping(nav)
    print(f"[bench] Nav2 active, set_mode_mapping={'ok' if mapping else 'НЕТ ОТВЕТА'}", flush=True)
    # дать rtabmap принять первый скан в режиме картирования и опубликовать map→odom
    deadline = time.monotonic() + 60
    while (recorder.estimate() is None or recorder.last_truth is None) and time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.2)
    _spin_for(nav, 3.0)

    rows: list[dict] = []
    failed = False
    for g in sc["goals"]:
        goal = Pose2D(float(g["x"]), float(g["y"]), float(g.get("yaw", 0.0)))
        if failed and sc.get("stop_on_failure", True):
            rec = GoalRecord(g["name"], goal, "SKIPPED", recorder.now(), recorder.now())
        else:
            start = Pose2D(*recorder.last_truth) if recorder.last_truth else Pose2D(0, 0, 0)
            timeout = timeout_for(sc, start, goal)
            print(f"[bench] {g['name']}: цель ({goal.x}, {goal.y}, {goal.yaw}) таймаут {timeout:.0f} с", flush=True)
            rec = run_goal(nav, recorder, g["name"], goal, timeout)
            failed = failed or rec.status != "SUCCEEDED"
        under = None
        if rec.status not in ("SUCCEEDED", "SKIPPED"):
            under = dump_costmap(nav, recorder, args.out, f"{args.world.replace('/', '_')}_{g['name']}")
        row = {"world": args.world, "label": args.label, **goal_metrics(rec, boxes), "fail_cost_under_robot": under}
        print("[bench] " + json.dumps(row, ensure_ascii=False), flush=True)
        rows.append(row)
        write_trajectories(rec, args.out, f"{args.world.replace('/', '_')}_{g['name']}")

    grid = map_check.grab_map(nav)
    map_stats = None
    if grid is not None:
        map_stats = map_check.compare(grid, boxes)
        args.out.mkdir(parents=True, exist_ok=True)
        map_check.write_pgm(grid, args.out / f"{args.world.replace('/', '_')}_map.pgm")
        print(f"[bench] MAP {grid.width}x{grid.height} res={grid.resolution:.3f} "
              f"origin=({grid.origin_x:.2f},{grid.origin_y:.2f}) {json.dumps(map_stats)}", flush=True)
        for line in map_check.ascii_map(grid, step=max(1, int(0.2 / grid.resolution))):
            print("[bench] MAP " + line, flush=True)
    else:
        print("[bench] MAP: /rtabmap/map не пришла", flush=True)

    write_outputs(args.out, args.world, rows, {
        "label": args.label,
        "set_mode_mapping": mapping,
        "map": map_stats,
        "wall_time_s": round(time.monotonic() - t0, 1),
        "scenario": {k: v for k, v in sc.items() if k != "goals"},
    })
    print("\n" + summary_markdown(rows), flush=True)
    nav.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
