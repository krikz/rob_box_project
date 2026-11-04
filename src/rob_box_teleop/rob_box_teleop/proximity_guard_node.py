"""proximity_guard_node — зона безопасности вокруг корпуса (#4365): мигалка, писк, голос, аварийный стоп.

Тонкая нода на Main Pi (контейнер ``proximity-guard`` на образе teleop). Решает всё ``rob_box_teleop.proximity``;
нода подаёт туда скан и скорость и публикует решённое через готовые механизмы:

* ``/scan`` (``LaserScan``) -> точки в ``base_footprint`` (поза лидара — из TF, статическая);
* ``/diff_drive_controller/odom`` (``Odometry``, 50 Гц) -> «в движении» и куда едет корпус. Колёсная одометрия,
  а не ``/odom``: тот публикует rtabmap icp_odometry (10 Гц, живёт вместе с rtabmap), см. controller_manager.yaml;
* стоп: нулевой ``Twist`` в ``cmd_vel_emergency`` — вход twist_mux с priority 255 и timeout 0.1 с,
  поэтому пока держится стоп, нода шлёт его таймером ``rate_hz`` (умолчание 20 Гц);
* мигалка: ``ExpressionRequest.gesture`` в ``/voice/expression/request`` (``expression_client``) —
  её показывает ``face_node`` на Vision Pi;
* писк: имя триггера sound_pack в ``/voice/sound/trigger`` — его играет ``sound_node`` на Sound Pi;
* голос: фраза кода (``Utterance``) в ``SpeechRequest`` через ``speech_client``, автор ``system`` — как у
  joystick_control_node; озвучивает владелец голоса на Sound Pi.

Какие каналы, когда и на сколько — решает ``proximity.AlarmScheduler`` (одно событие тревоги на зону).

Габарит корпуса — footprint local_costmap из того же ``nav2_params.yaml``, что у Nav2 (один источник).
Включение: ``COMPOSE_PROFILES=proximity-guard`` (по умолчанию сервиса нет).
"""

from __future__ import annotations

import json
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rob_box_core import expression_client, speech_client
from rob_box_core.speech_client import SYSTEM_AGENT
from rob_box_core.utterance import Utterance

from rob_box_teleop import proximity as px

EMERGENCY_TOPIC = "cmd_vel_emergency"
SOUND_TRIGGER_TOPIC = "/voice/sound/trigger"
#: Скан или колёсная одометрия старше этого — данных нет (угрозы не видно, это пишется в журнал).
STALE_S = 0.5

_DEFAULTS = {
    "footprint_file": "/config/nav2/nav2_params.yaml",
    "base_frame": "base_footprint",
    "scan_topic": "/scan",
    "odom_topic": "/diff_drive_controller/odom",
    "rate_hz": 20.0,
    "warn_m": px.Thresholds.warn_m,
    "stop_m": px.Thresholds.stop_m,
    "release_margin_m": px.Thresholds.release_margin_m,
    "release_s": px.Thresholds.release_s,
    "stop_hold_s": px.Thresholds.stop_hold_s,
    "moving_v": px.Thresholds.moving_v,
    "moving_w": px.Thresholds.moving_w,
    "closing_mps": px.Thresholds.closing_mps,
    "min_points": px.Thresholds.min_points,
}
_THRESHOLD_KEYS = ("warn_m", "stop_m", "release_margin_m", "release_s", "stop_hold_s", "moving_v", "moving_w",
                   "closing_mps")


def load_body(path: str) -> px.Body:
    """Корпус из ``nav2_params.yaml``."""
    import yaml  # noqa: PLC0415 - нужен только на старте ноды

    with open(path, encoding="utf-8") as fh:
        return px.body_from_nav2_params(yaml.safe_load(fh))


class ProximityGuardNode(Node):
    """Скан + одометрия -> зона -> стоп, мигалка, звук."""

    def __init__(self) -> None:
        super().__init__("proximity_guard")
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)
        self.th = px.Thresholds(min_points=int(self._p("min_points")),
                                **{k: float(self._p(k)) for k in _THRESHOLD_KEYS})
        self.body = load_body(str(self._p("footprint_file")))
        self.base_frame = str(self._p("base_frame"))
        self.tracker = px.ZoneTracker(self.th)
        self.alarms = px.AlarmScheduler()
        self._mount: Optional[px.Mount] = None
        self._points: List[px.Point] = []
        self._scan_at = -math.inf
        self._vel = (0.0, 0.0)
        self._odom_at = -math.inf
        self._blind_logged = False

        import tf2_ros  # noqa: PLC0415 - TF нужен только живой ноде

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._stop_pub = self.create_publisher(Twist, EMERGENCY_TOPIC, 10)
        self._sound_pub = self.create_publisher(String, SOUND_TRIGGER_TOPIC, 10)
        expression_client.prepare(self, "request")
        speech_client.prepare(self, "request")
        self.create_subscription(LaserScan, str(self._p("scan_topic")), self._on_scan, 10)
        self.create_subscription(Odometry, str(self._p("odom_topic")), self._on_odom, 10)
        self.create_timer(1.0 / float(self._p("rate_hz")), self._on_tick)
        self.get_logger().info(
            f"🛡️ зона корпуса: x[{self.body.x_min:.3f}, {self.body.x_max:.3f}] y[{self.body.y_min:.3f}, "
            f"{self.body.y_max:.3f}]; внимание < {self.th.warn_m:.2f} м, стоп < {self.th.stop_m:.2f} м, "
            f"только в движении (|v| > {self.th.moving_v}, |w| > {self.th.moving_w})")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- входы ------------------------------------------------------------------------------------
    def _on_scan(self, msg) -> None:
        mount = self._mount_for(msg.header.frame_id)
        if mount is None:
            return
        self._points = px.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                      msg.range_max, mount)
        self._scan_at = self._now()

    def _on_odom(self, msg) -> None:
        self._vel = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
        self._odom_at = self._now()

    def _mount_for(self, frame: str) -> Optional[px.Mount]:
        """Поза лидара в ``base_frame`` из TF (статическая — берётся один раз)."""
        if self._mount is not None:
            return self._mount
        import rclpy.time  # noqa: PLC0415

        try:
            tf = self._tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - tf2 бросает несколько своих типов
            self.get_logger().warn(f"нет TF {self.base_frame} ← {frame}: {exc}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        self._mount = px.Mount(t.x, t.y, px.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.get_logger().info(f"лидар {frame} в {self.base_frame}: x={t.x:.3f} y={t.y:.3f} "
                               f"yaw={math.degrees(self._mount.yaw):.1f}°")
        return self._mount

    # -- решение и выходы ---------------------------------------------------------------------------
    def _on_tick(self) -> None:
        now = self._now()
        threat = self._threat(now)
        tr = self.tracker.update(now, threat.gap)
        if tr is not None:
            self._log_transition(tr, threat)
        if self.tracker.stopping:
            self._stop_pub.publish(Twist())
        for effect in self.alarms.tick(now, self.tracker.zone):
            self._apply(effect)

    def _log_transition(self, tr: px.Transition, threat: px.Threat) -> None:
        where = "" if threat.point is None else f" в точке ({threat.point[0]:+.2f}, {threat.point[1]:+.2f})"
        line = f"🛡️ {px.describe(tr)}{where}; v={self._vel[0]:+.2f} м/с w={self._vel[1]:+.2f} рад/с"
        # Два вызова, а не выбор метода: rclpy запрещает менять уровень у одного места вызова
        # («Logger severity cannot be changed between calls», поймано в симе).
        if px.severity(tr.zone):
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    def _threat(self, now: float) -> px.Threat:
        blind = now - self._scan_at > STALE_S or now - self._odom_at > STALE_S
        if blind != self._blind_logged:
            self._blind_logged = blind
            if blind:
                self.get_logger().error(f"🛡️ нет свежих скана или одометрии (> {STALE_S} с) — зона НЕ контролируется")
            else:
                self.get_logger().info("🛡️ скан и одометрия идут — зона под контролем")
        if blind:
            return px.NO_THREAT
        return px.nearest_threat(self.body, self._points, self._vel[0], self._vel[1], self.th)

    def _apply(self, effect: px.Effect) -> None:
        if effect.kind == px.EFFECT_LIGHTS:
            expression_client.express(self, turn_id="", epoch=0, agent=SYSTEM_AGENT, gesture=effect.name,
                                      ttl_ms=effect.ttl_ms)
        elif effect.kind == px.EFFECT_SOUND:
            msg = String()
            msg.data = json.dumps({"sound": effect.name, "request_id": ""}, ensure_ascii=False)
            self._sound_pub.publish(msg)
        else:
            self._say(effect.name)

    def _say(self, text: str) -> None:
        """Фраза тревоги голосом; сбой отправки — в журнал, стоп и мигалку он не трогает (как у джойстика)."""
        try:
            speech_client.say(self, Utterance(text=text), agent=SYSTEM_AGENT)
        except Exception as exc:  # noqa: BLE001 - речь — обратная связь, не управление
            self.get_logger().error(f"🛡️ фраза тревоги не отправлена ({text!r}): {exc!r}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProximityGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
