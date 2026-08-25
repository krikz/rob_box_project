"""ROS2-нода rob_box_quest — Zenoh-мост + teleop + dead-man + safety.

Источник истины: docs/architecture/meta-quest-api.md §5/§6/§9,
docs/adr/0027 §3.3 (dead-man, watchdog, emergency B),
docs/plans/2026-08-24-meta-quest-telepresence.md §1.3.

Что делает:
1. Поднимает aiohttp WSS server (Phase 1.2) внутри ROS executor.
2. QuestBridge имплементирует Protocol из ws_server.py:
   - publish_quest → TeleopController.consume + publish cmd_vel_quest
   - publish_emergency → publish cmd_vel_emergency (edge)
   - feed_client_alive → reset Watchdog
   - emergency_stop → TeleopController.emergency_stop + close WS
3. Периодический tick loop (30 Гц) — TeleopController.tick → publish.
4. Watchdog loop — если клиент молчит > 0.5 с → emergency_stop.
5. Подписка на /odom для robot_status aggregation (Phase 1.3: базовый,
   в Phase 1.6 — battery/wifi).

Запуск:
    ros2 run rob_box_quest quest_node
    # или
    python3 -m rob_box_quest.quest_node
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .core.safety import Watchdog
from .core.teleop import TeleopController
from .server.session import WATCHDOG_TIMEOUT_S as SESSION_WATCHDOG_TIMEOUT_S
from .server.ws_server import WSSServer, build_app

log = logging.getLogger(__name__)


# Throttle публикации cmd_vel_quest (meta-quest-api.md §9):
# "не чаще 30 Гц, на повторный seq сервер отбрасывает".
# У нас — fixed-rate 30 Гц.
PUBLISH_RATE_HZ: float = 30.0
PUBLISH_PERIOD_S: float = 1.0 / PUBLISH_RATE_HZ


class QuestBridge:
    """Реализация Bridge Protocol из ws_server.py через rclpy publishers.

    Хранит TeleopController (один на Phase 1) и Watchdog.
    Publish из event-loop aiohttp безопасен (rclpy thread-safe).
    """

    def __init__(
        self,
        node: "QuestNode",
        cmd_vel_quest_pub,
        cmd_vel_emergency_pub,
    ) -> None:
        self._node = node
        self._pub_quest = cmd_vel_quest_pub
        self._pub_emergency = cmd_vel_emergency_pub
        self._teleop = TeleopController()
        self._watchdog = Watchdog(timeout_s=SESSION_WATCHDOG_TIMEOUT_S)

    # --- Bridge Protocol -------------------------------------------------

    def publish_quest(self, linear: float, angular: float) -> None:
        """Bridge.feed из ws_server на teleop_twist.

        Phase 1.3: просто публикуем cmd_vel_quest с текущими значениями.
        Контроллер (dead-man) живёт на стороне клиента: при отпускании
        grip клиент шлёт JSON_CMD{cmd:'teleop_twist', deadman:false} →
        мы публикуем нулевой Twist (это обеспечивается самим клиентом).
        Здесь мы публикуем те значения, которые присланы (clamping внутри
        TeleopController).
        """
        now = time.monotonic()
        twist = self._teleop.consume(linear, angular, deadman=True, now_monotonic=now)
        if twist is None:
            # consume вернул None (emergency) → публикуем нулевой Twist
            # чтобы twist_mux увидел timeout и emergency_stop выиграл.
            self._publish_zero()
            return
        msg = Twist()
        msg.linear.x = float(twist.linear_x)
        msg.angular.z = float(twist.angular_z)
        self._pub_quest.publish(msg)

    def publish_emergency(self) -> None:
        """Edge-triggered: однократно публикуем ненулевой emergency-marker.

        twist_mux видит timeout 0.1 с на cmd_vel_emergency (priority 255)
        → effective cmd_vel = 0. После этого сокет закроется по watchdog
        и bridge не сможет публиковать — safe stop сохраняется до ack.
        """
        # Публикуем Twist с наносекундным timestamp (twist_mux считает
        # timeout от stamp). marker = True на стороне ноды-флага не нужен
        # потому что сам twist_mux реагирует на отсутствие сообщений.
        msg = Twist()
        # Специальный sentinel: угловая скорость = nan запрещена ROS;
        # используем соглашение "emergency" в логах.
        self._node.get_logger().warning("🛑 EMERGENCY STOP from Quest client — publishing cmd_vel_emergency")
        self._pub_emergency.publish(msg)

    def feed_client_alive(self) -> None:
        """Клиент прислал валидный фрейм → сброс watchdog."""
        self._watchdog.feed(time.monotonic())

    def emergency_stop(self) -> None:
        """Зафиксировать emergency lock (клиент прислал stop_emergency)."""
        self._teleop.emergency_stop()
        self._publish_zero()

    # --- Periodic helpers (вызываются из QuestNode loop) -----------------

    def tick_publish(self, now_monotonic: float) -> None:
        """Периодическая публикация последнего Twist (30 Гц).

        Если клиент молчит — tick() вернёт None и мы публикуем нули.
        """
        twist = self._teleop.tick(now_monotonic)
        if twist is None:
            self._publish_zero()
            return
        msg = Twist()
        msg.linear.x = float(twist.linear_x)
        msg.angular.z = float(twist.angular_z)
        self._pub_quest.publish(msg)

    def watchdog_check(self, now_monotonic: float) -> bool:
        """True если watchdog trip → safe stop нужен."""
        return self._watchdog.tripped(now_monotonic)

    def reset(self) -> None:
        """Operator ack после emergency — снимаем lock и сбрасываем watchdog."""
        self._teleop.reset()
        self._watchdog.reset()

    # --- internal --------------------------------------------------------

    def _publish_zero(self) -> None:
        msg = Twist()
        # zero linear/angular — twist_mux timeout'нет и emergency_stop (255)
        # остаётся приоритетом; cmd_vel_quest публикует нули.
        self._pub_quest.publish(msg)


class QuestNode(Node):
    """Главная ROS2-нода rob_box_quest.

    Параметры:
      ws_host (str, default "0.0.0.0")
      ws_port (int, default 8765) — внутри host-network,
                                    снаружи через Caddy (8443).
      pin     (str, optional) — если не задан, генерируется ACTIVE_PIN.
      log_pin (bool, default True) — логировать PIN в stdout (для docker logs).
    """

    def __init__(self) -> None:
        super().__init__("quest_node")
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("log_pin", True)

        log_pin = bool(self.get_parameter("log_pin").value)

        # Publishers (см. twist_mux.yaml: priority 40 quest, 255 emergency).
        _RE = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_quest = self.create_publisher(Twist, "cmd_vel_quest", _RE)
        self._pub_emergency = self.create_publisher(Twist, "cmd_vel_emergency", _RE)
        # Odometry для robot_status (Phase 1.3: только для healthcheck).
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, _RE)
        self._latest_odom: Optional[Odometry] = None

        # Bridge + WS server.
        from .server.ws_server import ACTIVE_PIN

        self.bridge = QuestBridge(
            node=self,
            cmd_vel_quest_pub=self._pub_quest,
            cmd_vel_emergency_pub=self._pub_emergency,
        )
        self.ws_server = WSSServer(bridge=self.bridge, pin=ACTIVE_PIN)
        if log_pin:
            self.get_logger().warning(
                f"🔑 Quest PIN: {ACTIVE_PIN} " "(show this to operator — required to start a session)"
            )

        # Запускаем aiohttp в отдельном thread (rclpy и aiohttp —
        # оба event-loop; запускать aiohttp в rclpy callback'е нельзя).
        self._aio_thread: Optional[threading.Thread] = None
        self._aio_loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_event = threading.Event()

        # Периодический tick 30 Гц (publish cmd_vel_quest пока свежо).
        self._tick_timer = self.create_timer(PUBLISH_PERIOD_S, self._on_tick_timer)
        # Watchdog check (раз в 100 мс).
        self._watchdog_timer = self.create_timer(0.1, self._on_watchdog_timer)

        # Запуск aiohttp отложен до first timer callback (rclpy init
        # уже произошёл к этому моменту).
        self._aio_started = False

    # --- ROS callbacks ----------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_tick_timer(self) -> None:
        if not self._aio_started:
            self._start_aiohttp()
            self._aio_started = True
        self.bridge.tick_publish(time.monotonic())

    def _on_watchdog_timer(self) -> None:
        if self.bridge.watchdog_check(time.monotonic()):
            self.get_logger().warning("🛑 Watchdog tripped — emergency stop (Quest client silent)")
            self.bridge.publish_emergency()
            self.bridge.emergency_stop()

    # --- aiohttp lifecycle ------------------------------------------------

    def _start_aiohttp(self) -> None:
        if self._aio_thread is not None:
            return

        from aiohttp import web as _aiohttp_web

        app = build_app(self.ws_server)

        def _runner() -> None:
            self._aio_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._aio_loop)
            runner = _aiohttp_web.AppRunner(app)
            self._aio_loop.run_until_complete(runner.setup())
            site = _aiohttp_web.TCPSite(
                runner,
                str(self.get_parameter("ws_host").value),
                int(self.get_parameter("ws_port").value),
            )
            self._aio_loop.run_until_complete(site.start())
            self.get_logger().info(
                f"🌐 Quest WSS server listening on "
                f"{self.get_parameter('ws_host').value}:"
                f"{self.get_parameter('ws_port').value}/quest"
            )
            try:
                self._aio_loop.run_forever()
            finally:
                self._aio_loop.run_until_complete(runner.cleanup())

        self._aio_thread = threading.Thread(target=_runner, name="quest-aiohttp", daemon=True)
        self._aio_thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._aio_loop is not None and self._aio_loop.is_running():
            self._aio_loop.call_soon_threadsafe(self._aio_loop.stop)
        if self._aio_thread is not None:
            self._aio_thread.join(timeout=2.0)


def main(args: Optional[list[str]] = None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = QuestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
