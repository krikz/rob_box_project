"""Рекордер картографии в симе: снимки карты rtabmap и две позы робота по sim time.

Запускается рядом со стендом (сервис ``maprec`` в docker/sim/compose.mapping.yaml),
пишет в ``--out``:

  * ``maps/map_<t_ms>.npz`` — каждое сообщение ``/rtabmap/map`` (t — sim time приёма,
    resolution, origin, data[r, c] как в OccupancyGrid);
  * ``poses.csv`` — t, истинная поза Gazebo (``/sim/ground_truth/odom``) и оценка
    TF ``map → base_footprint`` с частотой ``--rate`` по sim time;
  * ``meta.json`` — топик, число снимков, sim time и настенное время записи, RTF.

Останавливается по SIGINT/SIGTERM (``docker stop``) или по ``--duration`` sim-секунд.
Кадры видео из записи строит ``rob_box_sim.map_video``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
from pathlib import Path


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _cells(pose) -> list[str]:
    """Три ячейки csv позы; нет позы — три пустые (map_video.load_poses их пропускает)."""
    return [f"{v:.4f}" for v in pose] if pose is not None else ["", "", ""]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Рекордер карты rtabmap и поз (сим)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--map-topic", default="/rtabmap/map")
    ap.add_argument("--rate", type=float, default=10.0, help="частота записи поз, Гц sim time")
    ap.add_argument("--duration", type=float, default=0.0, help="sim-секунд записи (0 — до сигнала)")
    args = ap.parse_args(argv)

    import numpy as np
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    maps_dir = args.out / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("sim_map_recorder", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node, spin_thread=False)
    state = {"truth": None, "maps": 0, "stop": False}

    def now() -> float:
        return node.get_clock().now().nanoseconds / 1e9

    def on_truth(msg):
        p = msg.pose.pose
        state["truth"] = (p.position.x, p.position.y, _yaw(p.orientation))

    def on_map(msg):
        t = now()
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        np.savez_compressed(
            maps_dir / f"map_{int(round(t * 1000)):09d}.npz", t=t, resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x, origin_y=msg.info.origin.position.y, data=data,
        )
        state["maps"] += 1

    node.create_subscription(Odometry, "/sim/ground_truth/odom", on_truth, 20)
    node.create_subscription(OccupancyGrid, args.map_topic, on_map, QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def stop(*_):
        state["stop"] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    # Ждём /clock: до него sim time = 0.
    while now() <= 0 and not state["stop"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    t_start, wall_start = now(), time.monotonic()
    period = 1.0 / args.rate
    next_t = t_start
    rows = 0
    with open(args.out / "poses.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "tx", "ty", "tyaw", "ex", "ey", "eyaw"])
        while not state["stop"]:
            rclpy.spin_once(node, timeout_sec=0.02)
            t = now()
            if args.duration and t - t_start >= args.duration:
                break
            if t < next_t:
                continue
            next_t = t + period
            truth = state["truth"]
            try:
                tr = tf.lookup_transform("map", "base_footprint", Time()).transform
                est = (tr.translation.x, tr.translation.y, _yaw(tr.rotation))
            except Exception:  # noqa: BLE001 — нет TF = нет оценки, пишем пусто
                est = None
            if truth is None and est is None:
                continue
            w.writerow([f"{t:.3f}", *_cells(truth), *_cells(est)])
            rows += 1
            if rows % 50 == 0:
                f.flush()
    t_end, wall = now(), time.monotonic() - wall_start
    meta = {
        "map_topic": args.map_topic,
        "map_snapshots": state["maps"],
        "pose_rows": rows,
        "sim_s": round(t_end - t_start, 2),
        "wall_s": round(wall, 2),
        "rtf": round((t_end - t_start) / wall, 3) if wall > 0 else None,
    }
    (args.out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print("[maprec] " + json.dumps(meta, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
