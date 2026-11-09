"""Тонкая нода варианта B (#4346): видимый тег с известной позой → ``/rtabmap/initialpose``.

ПРОТОТИП, ВЫКЛЮЧЕН ПО УМОЛЧАНИЮ (``enabled: false``): боевое поведение робота без решения
Шифу не меняется. На роботе ноду никто не запускает; в симе — сервис ``tagreloc``
docker/sim/compose.loc.yaml (стенд локализации).

Вход: ``/detections`` (apriltag_msgs/AprilTagDetectionArray) и TF ``base_footprint → tag36h11:<id>``
(его публикует apriltag_node), текущая оценка — TF ``map → base_footprint``.
Решение — rob_box_perception.tag_relocalization.TagRelocalizer (чистое ядро):
``correct`` → PoseWithCovarianceStamped в ``initialpose_topic``; ``agree`` — только лог.

Параметры:
  enabled            bool   (false)
  tags               double[] плоско: id, x, y, z, normal_yaw, id, x, ... (центр тега и куда смотрит лицо)
  family             str    (tag36h11) — префикс кадров тегов apriltag_ros
  base_frame/map_frame       (base_footprint / map)
  initialpose_topic  str    (/rtabmap/initialpose)
  max_range_m, min_consistent, correct_if_m, correct_if_deg, cooldown_s — см. RelocalizerConfig
"""

from __future__ import annotations

import math
import sys

from rob_box_perception.tag_relocalization import RelocalizerConfig, TagRelocalizer, Transform, parse_tags


def _tags_from_flat(flat) -> dict:
    flat = list(flat)
    if len(flat) % 5:
        raise ValueError(f"tags: длина {len(flat)} не кратна 5 (id, x, y, z, normal_yaw)")
    return parse_tags([flat[i:i + 5] for i in range(0, len(flat), 5)])


def main(argv=None) -> int:
    import rclpy
    from apriltag_msgs.msg import AprilTagDetectionArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=argv)
    node = Node("tag_relocalization")
    p = {
        "enabled": node.declare_parameter("enabled", False).value,
        "tags": node.declare_parameter("tags", [0.0]).value,
        "family": node.declare_parameter("family", "tag36h11").value,
        "base_frame": node.declare_parameter("base_frame", "base_footprint").value,
        "map_frame": node.declare_parameter("map_frame", "map").value,
        "topic": node.declare_parameter("initialpose_topic", "/rtabmap/initialpose").value,
        "max_range_m": node.declare_parameter("max_range_m", 2.5).value,
        "min_consistent": node.declare_parameter("min_consistent", 5).value,
        "correct_if_m": node.declare_parameter("correct_if_m", 0.2).value,
        "correct_if_deg": node.declare_parameter("correct_if_deg", 8.0).value,
        "cooldown_s": node.declare_parameter("cooldown_s", 10.0).value,
    }
    log = node.get_logger()
    if not p["enabled"]:
        log.info("tag_relocalization выключен (enabled=false): ничего не публикую")
        node.destroy_node()
        rclpy.shutdown()
        return 0
    reloc = TagRelocalizer(RelocalizerConfig(
        tags=_tags_from_flat(p["tags"]), max_range_m=float(p["max_range_m"]),
        min_consistent=int(p["min_consistent"]), correct_if_m=float(p["correct_if_m"]),
        correct_if_deg=float(p["correct_if_deg"]), cooldown_s=float(p["cooldown_s"]),
    ))
    tf = Buffer()
    TransformListener(tf, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, p["topic"], 10)
    log.info(f"tag_relocalization: теги {sorted(reloc.config.tags)} → {p['topic']}")

    def lookup(parent, child, stamp):
        try:
            tr = tf.lookup_transform(parent, child, stamp, timeout=Duration(seconds=0.1)).transform
        except Exception:  # noqa: BLE001 — нет TF = нет наблюдения
            return None
        r = tr.rotation
        return Transform(tr.translation.x, tr.translation.y, tr.translation.z, (r.x, r.y, r.z, r.w))

    def on_detections(msg):
        now = node.get_clock().now()
        est_tf = lookup(p["map_frame"], p["base_frame"], Time())
        estimate = (est_tf.x, est_tf.y, est_tf.yaw()) if est_tf else None
        for det in msg.detections:
            if int(det.id) not in reloc.config.tags:
                continue
            base_tag = lookup(p["base_frame"], f"{p['family']}:{int(det.id)}", Time.from_msg(msg.header.stamp))
            if base_tag is None:
                continue
            d = reloc.observe(now.nanoseconds / 1e9, int(det.id), base_tag, estimate)
            if d is None:
                continue
            if d.kind == "agree":
                log.debug(f"тег {d.tag_id}: rtabmap согласен (расхождение {d.error_m:.2f} м)")
                continue
            out = PoseWithCovarianceStamped()
            out.header.frame_id = p["map_frame"]
            out.header.stamp = now.to_msg()
            out.pose.pose.position.x, out.pose.pose.position.y = d.pose[0], d.pose[1]
            out.pose.pose.orientation.z = math.sin(d.pose[2] / 2)
            out.pose.pose.orientation.w = math.cos(d.pose[2] / 2)
            out.pose.covariance[0] = out.pose.covariance[7] = 0.05 ** 2
            out.pose.covariance[35] = math.radians(3.0) ** 2
            pub.publish(out)
            err = "нет оценки" if d.error_m is None else f"{d.error_m:.2f} м / {d.error_deg:.1f}°"
            log.info(f"тег {d.tag_id}: поза по тегу ({d.pose[0]:.2f}, {d.pose[1]:.2f}, "
                     f"{math.degrees(d.pose[2]):.0f}°), расхождение с rtabmap {err} → initialpose")

    node.create_subscription(AprilTagDetectionArray, "/detections", on_detections, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
