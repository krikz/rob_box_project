"""Стенд локализации в симе (#4346, вариант D): пишет истину и оценку позы, крутит робота «пробой».

Запускается в контейнере ``locbench`` (docker/sim/compose.loc.yaml) сразу после старта стека,
то есть с момента «перезапуска» робота::

    python3 -m rob_box_sim.loc_bench --scenario restart_same --label a1 --out /results

Что делает:
  1. ждёт /clock; с этого момента 10 Гц sim time пишет истинную позу Gazebo
     (``/sim/ground_truth/odom``) и оценку TF ``map → base_footprint``; считает детекции
     ``/detections`` по id;
  2. ждёт первую оценку (rtabmap поднялся и опубликовал map → odom), не дольше ``--wait-estimate-s``;
  3. выполняет шаги «пробы» сценария из scenarios/localization.yaml: ``wait`` (стоять),
     ``spin`` (поворот на месте через twist_mux, вход ``cmd_vel_web``), ``kidnap`` (запросить
     телепорт: файл ``<out>/<label>.kidnap``; телепорт делает docker/sim/run_loc_bench.sh через
     ``ign service set_pose`` и стенд ждёт скачка истины);
  4. пишет ``<label>_poses.csv`` (t, истина, оценка) и ``<label>.json`` с метриками
     rob_box_sim.loc_metrics (от старта или от телепорта), RTF и детекциями.

Сам стенд ничего не решает за rtabmap: только едет по сценарию и записывает.
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
from rob_box_sim.loc_metrics import Sample, Tolerance, kidnap_time, run_metrics

DEFAULT_SCENARIOS = "/ws/install/rob_box_sim/share/rob_box_sim/scenarios/localization.yaml"


def load_scenario(path: Path, name: str) -> dict:
    """Сценарий ``name``: поля ``defaults`` + свои; шаги пробы — ``probe`` (или общий ``probes[<имя>]``)."""
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    if name not in doc["scenarios"]:
        raise SystemExit(f"сценария {name!r} нет в {path}: есть {sorted(doc['scenarios'])}")
    sc = {**doc.get("defaults", {}), **doc["scenarios"][name]}
    probe = sc.get("probe")
    if isinstance(probe, str):
        sc["probe"] = doc["probes"][probe]
    for step in sc["probe"]:
        if len(step) != 1 or next(iter(step)) not in ("wait", "spin", "kidnap"):
            raise SystemExit(f"шаг пробы {step!r}: ждём ровно один из wait | spin | kidnap")
    return sc


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LocRecorder:
    """Подписки и запись; колбэки крутит spin_once из main."""

    def __init__(self, node, rate_hz: float):
        from apriltag_msgs.msg import AprilTagDetectionArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_ros import Buffer, TransformListener

        self.node = node
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, node, spin_thread=False)
        self.truth = None
        self.samples: list[Sample] = []
        self.detections: dict[int, int] = {}
        self.period = 1.0 / rate_hz
        self._next = 0.0
        self.cmd = node.create_publisher(Twist, "/cmd_vel_web", 10)
        self._twist = Twist
        node.create_subscription(Odometry, "/sim/ground_truth/odom", self._on_truth, 20)
        node.create_subscription(AprilTagDetectionArray, "/detections", self._on_det, 10)

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def estimate(self):
        from rclpy.time import Time

        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except Exception:  # noqa: BLE001 — нет TF = нет оценки, это и пишем
            return None
        return (tr.translation.x, tr.translation.y, _yaw(tr.rotation))

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_det(self, msg):
        for d in msg.detections:
            self.detections[int(d.id)] = self.detections.get(int(d.id), 0) + 1

    def tick(self, timeout: float = 0.02) -> None:
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=timeout)
        t = self.now()
        if t >= self._next and (self.truth is not None):
            self._next = t + self.period
            self.samples.append(Sample(t, self.truth, self.estimate()))

    def drive(self, wz: float) -> None:
        msg = self._twist()
        msg.angular.z = wz
        self.cmd.publish(msg)


def run_probe(rec: LocRecorder, steps: list[dict], out: Path, label: str, kidnap_wait_wall_s: float) -> list[str]:
    """Шаги пробы по sim time. Возвращает журнал шагов (для json)."""
    log = []
    for step in steps:
        kind, val = next(iter(step.items()))
        t0 = rec.now()
        log.append(f"{t0:.1f} {kind} {val}")
        print(f"[loc] {t0:.1f} {kind} {val}", flush=True)
        if kind == "wait":
            while rec.now() - t0 < float(val):
                rec.tick()
        elif kind == "spin":
            angle, wz = float(val[0]), float(val[1])
            dur = abs(angle) / abs(wz)
            nxt = 0.0
            while rec.now() - t0 < dur:
                if rec.now() >= nxt:
                    rec.drive(math.copysign(abs(wz), angle))
                    nxt = rec.now() + 0.1
                rec.tick()
            for _ in range(5):
                rec.drive(0.0)
                rec.tick(0.05)
        elif kind == "kidnap":
            marker = out / f"{label}.kidnap"
            n0 = len(rec.samples)
            # «x y yaw» одной строкой: её читает bash (docker/sim/run_loc_bench.sh)
            marker.write_text(" ".join(f"{float(v):.4f}" for v in val) + "\n", encoding="utf-8")
            print(f"[loc] KIDNAP_NOW {json.dumps(val)}", flush=True)
            wall0 = time.monotonic()
            while time.monotonic() - wall0 < kidnap_wait_wall_s:
                rec.tick()
                if kidnap_time(rec.samples[n0:], Tolerance()) is not None:
                    break
            else:
                print("[loc] KIDNAP: скачка истины нет — телепорт не случился", flush=True)
                log.append("kidnap_missing")
    return log


def write_poses(samples: list[Sample], path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "tx", "ty", "tyaw", "ex", "ey", "eyaw"])
        for s in samples:
            cells = [f"{v:.4f}" for v in s.truth] if s.truth else ["", "", ""]
            cells += [f"{v:.4f}" for v in s.estimate] if s.estimate else ["", "", ""]
            w.writerow([f"{s.t:.3f}", *cells])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Стенд локализации после перезапуска (сим, #4346)")
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--label", required=True, help="метка прогона: имя файлов результатов")
    ap.add_argument("--scenarios", type=Path, default=Path(DEFAULT_SCENARIOS))
    ap.add_argument("--out", type=Path, default=Path("/results"))
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--wait-estimate-s", type=float, default=120.0, help="sim-секунд ждать первую оценку")
    ap.add_argument("--kidnap-wait-wall-s", type=float, default=90.0)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.parameter import Parameter

    sc = load_scenario(args.scenarios, args.scenario)
    tol = Tolerance(**sc.get("tolerance", {}))
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node(
        "sim_loc_bench", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    rec = LocRecorder(node, args.rate)
    while rec.now() <= 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    t_start, wall_start = rec.now(), time.monotonic()
    print(f"[loc] start sim t={t_start:.1f} scenario={args.scenario} label={args.label}", flush=True)
    while rec.now() - t_start < args.wait_estimate_s and not any(s.estimate for s in rec.samples[-1:]):
        rec.tick()
    print(f"[loc] first estimate after {rec.now() - t_start:.1f} s sim: {rec.estimate()}", flush=True)
    log = run_probe(rec, sc["probe"], args.out, args.label, args.kidnap_wait_wall_s)
    t_end, wall = rec.now(), time.monotonic() - wall_start
    for _ in range(3):
        rec.drive(0.0)

    t_kid = kidnap_time(rec.samples, tol)
    t_event = t_kid if sc.get("event") == "kidnap" else t_start
    metrics = run_metrics(rec.samples, t_event, tol)
    if sc.get("event") == "kidnap" and t_kid is None:
        metrics["kidnap_missing"] = True
    result = {
        "label": args.label,
        "scenario": args.scenario,
        "event": sc.get("event", "restart"),
        "t_event_from_start_s": round(t_event - t_start, 1),
        **metrics,
        "rtf": round((t_end - t_start) / wall, 3) if wall > 0 else None,
        "sim_s": round(t_end - t_start, 1),
        "wall_s": round(wall, 1),
        "detections": {str(k): v for k, v in sorted(rec.detections.items())},
        "steps": log,
        "tolerance": tol.__dict__,
    }
    write_poses(rec.samples, args.out / f"{args.label}_poses.csv")
    (args.out / f"{args.label}.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("[loc] RESULT " + json.dumps(result, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
