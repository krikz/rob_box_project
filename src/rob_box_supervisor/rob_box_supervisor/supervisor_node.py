"""AvatarSupervisor — ROS 2 нода-координатор аватара (Phase 1 monitor, AV-6).

Дизайн:
- Параметр ``mode`` (default ``"monitor"``). В ``monitor``-режиме нода
  публикует ``/avatar/state`` из агрегатора и **отвечает** на сервисы
  ``AcquireFloor`` / ``ReleaseFloor`` / ``SetAvatarMode`` сообщением
  ``success=true, applied=false, reason="supervisor_in_monitor_mode"``,
  **не** меняя ``twist_mux`` inputs и ``dialogue_node`` параметры
  (ADR-0028 §4.5). Это минимизирует blast radius: нода задеплоена и
  наблюдает, реальное влияние — после явного ``mode:=active``
  (после AV-7+AV-8+AV-10).
- Phase 2 (``active``-режим) появится в отдельных карточках — здесь
  параметр читается, проверяется, и при попытке ``active`` нода
  логирует ``NOT_IMPLEMENTED`` и фактически остаётся в monitor.

Источники истины:
- ADR-0028 §4.3 (ROS 2 API)
- ADR-0028 §4.5 (monitor-режим)
- ADR-0028 §6 Q4 (``dead_man_trips_total{client_id}``)
- docs/architecture/SYSTEM_OVERVIEW.md §5.4
- docs/plans/2026-08-24-avatar-decomposition.md §AV-6

Zenoh: ``ZENOH_SESSION_CONFIG_URI`` env-переменная подхватывается
rmw_zenoh_cpp автоматически — нам читать её вручную не нужно, только
залогировать на старте для диагностики (см. :py:meth:`__init__`).
"""

from __future__ import annotations

import json
import os
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

# msgpack — Phase 1 wire-format для /avatar/state (см. ADR-0028 §4.3).
# AV-5 даст полный IDL; пока Phase 1 совместим через dict→msgpack.
try:  # pragma: no cover — на CI msgpack гарантированно есть (см. exec_depend)
    import msgpack

    _HAS_MSGPACK = True
except ImportError:  # pragma: no cover — defensive: нода не падает на импорте
    msgpack = None  # type: ignore[assignment]
    _HAS_MSGPACK = False


# Default monitor-mode reason, который нода возвращает клиентам в Phase 1.
# Зафиксирован строкой, чтобы логи и e2e-тесты могли матчить без магических
# литералов по всему коду (ADR-0028 §4.5).
MONITOR_MODE_REASON = "supervisor_in_monitor_mode"


class AvatarSupervisor(Node):
    """ROS 2 нода ``avatar_supervisor`` (Vision Pi, Phase 1 monitor)."""

    # ── topic / service constants (ADR-0028 §4.3) ─────────────────────
    AVATAR_STATE_TOPIC = "/avatar/state"

    ODOM_TOPIC = "/odom"
    DEVICE_SNAPSHOT_TOPIC = "/device/snapshot"
    VOICE_DIALOGUE_STATE_TOPIC = (
        "/voice/dialogue/state"  # НЕ /voice/state (ADR-0027 #2)
    )

    ACQUIRE_FLOOR_SERVICE = "acquire_floor"
    RELEASE_FLOOR_SERVICE = "release_floor"
    SET_AVATAR_MODE_SERVICE = "set_avatar_mode"

    def __init__(self) -> None:
        super().__init__("avatar_supervisor")

        # Параметр mode (default monitor). В monitor — наблюдаем, не
        # вмешиваемся. В active — Phase 2 (NOT_IMPLEMENTED в AV-6).
        self.declare_parameter("mode", "monitor")
        self._mode: str = str(self.get_parameter("mode").value or "monitor")

        # Логгер ROS (не stdlib logging — для unified rclpy logging).
        self._log = self.get_logger()

        # Aggregator + dead-man counter (pure-Python, тестируются отдельно).
        # Импортируем лениво: в mock-rclpy окружении (CI) эти модули не
        # зависят от rclpy и импорт всегда безопасен.
        from rob_box_supervisor.core import DeadManCounter, StateAggregator

        self._aggregator = StateAggregator()
        self._dead_man = DeadManCounter()

        # Publisher /avatar/state (latched, transient_local — ADR-0028 §4.3).
        # QoSProfile(depth=1, transient_local=True) эмулирует "latched".
        from rclpy.qos import DurabilityPolicy, QoSProfile

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(
            RosString, self.AVATAR_STATE_TOPIC, state_qos
        )

        # Subscriptions — на Phase 1 только регистрируем callback-и и
        # обновляем aggregator. Полный IDL / парсинг msg — Phase 2.
        self.create_subscription(RosString, self.ODOM_TOPIC, self._on_odom_msg, 10)
        self.create_subscription(
            RosString, self.DEVICE_SNAPSHOT_TOPIC, self._on_device_snapshot_msg, 10
        )
        self.create_subscription(
            RosString, self.VOICE_DIALOGUE_STATE_TOPIC, self._on_voice_state_msg, 10
        )

        # Services (Phase 1 — monitor: принимаем, логируем, отвечаем
        # success=true/applied=false/reason=monitor). Используем
        # std_srvs/Trigger как переносимый контракт Phase 1; AV-5 даст
        # кастомный IDL с полями client_id/floor/mode.
        from std_srvs.srv import Trigger

        self._srv_acquire = self.create_service(
            Trigger,
            self.ACQUIRE_FLOOR_SERVICE,
            self._on_acquire_floor,
        )
        self._srv_release = self.create_service(
            Trigger,
            self.RELEASE_FLOOR_SERVICE,
            self._on_release_floor,
        )
        self._srv_set_mode = self.create_service(
            Trigger,
            self.SET_AVATAR_MODE_SERVICE,
            self._on_set_avatar_mode,
        )

        # Периодическая публикация /avatar/state — 1 Hz достаточно для
        # monitor (Phase 2 увеличит частоту / сделает event-driven).
        self._timer = self.create_timer(1.0, self._publish_avatar_state)

        self._log_startup_diagnostics()

    # ── helpers ──────────────────────────────────────────────────────
    def _log_startup_diagnostics(self) -> None:
        """Залогировать env и mode на старте — помогает e2e/postmortem."""
        zenoh = os.environ.get("ZENOH_SESSION_CONFIG_URI", "<unset>")
        msgpack_state = "ok" if _HAS_MSGPACK else "MISSING"
        self._log.info(
            "avatar_supervisor started: mode=%s, zenoh=%s, msgpack=%s",
            self._mode,
            zenoh,
            msgpack_state,
        )

    def _monitor_response(self) -> dict:
        """Стандартный ответ для всех сервисов в monitor-режиме."""
        return {
            "success": True,
            "applied": False,
            "reason": MONITOR_MODE_REASON,
        }

    def _publish_avatar_state(self) -> None:
        """Timer-callback: публикует свежий snapshot агрегатора в /avatar/state."""
        snapshot = self._aggregator.snapshot()
        if _HAS_MSGPACK:
            try:
                payload = msgpack.packb(snapshot.to_msgpack_dict(), use_bin_type=True)
            except Exception as exc:  # noqa: BLE001
                self._log.warning("avatar_supervisor: msgpack encode failed: %s", exc)
                payload = json.dumps(snapshot.to_msgpack_dict()).encode("utf-8")
        else:  # pragma: no cover — defensive
            payload = json.dumps(snapshot.to_msgpack_dict()).encode("utf-8")
        msg = RosString()
        msg.data = payload.decode(
            "latin-1"
        )  # ROS String — UTF-8 safe для bytes-as-text
        self._state_pub.publish(msg)

    # ── subscription callbacks (Phase 1: best-effort parse) ──────────
    def _on_odom_msg(self, msg: RosString) -> None:
        """Обработать ``/odom``. Phase 1 — парсим минимум (x, y) из JSON."""
        data = self._try_parse_json(msg.data)
        if not isinstance(data, dict):
            return
        x = data.get("x")
        y = data.get("y")
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            self._aggregator.update_odom(x, y)

    def _on_device_snapshot_msg(self, msg: RosString) -> None:
        """Обработать ``/device/snapshot``. Phase 1 — battery_pct."""
        data = self._try_parse_json(msg.data)
        if not isinstance(data, dict):
            return
        self._aggregator.update_device_snapshot(battery_pct=data.get("battery_pct"))

    def _on_voice_state_msg(self, msg: RosString) -> None:
        """Обработать ``/voice/dialogue/state`` (НЕ /voice/state — ADR-0027 #2)."""
        data = self._try_parse_json(msg.data)
        if isinstance(data, dict):
            self._aggregator.update_voice_state(data.get("state"))
        elif isinstance(data, str):
            self._aggregator.update_voice_state(data)
        else:
            self._aggregator.update_voice_state(msg.data or None)

    @staticmethod
    def _try_parse_json(text: Optional[str]) -> Any:
        if not text:
            return None
        try:
            return json.loads(text)
        except (ValueError, TypeError):
            return None

    # ── service callbacks (Phase 1: monitor → log + answer) ──────────
    def _on_acquire_floor(self, _request: Any, response: Any) -> Any:
        """``AcquireFloor`` — Phase 1 monitor: лог + monitor response."""
        self._log.info("AcquireFloor received (mode=%s) — phase 1 monitor", self._mode)
        return self._fill_monitor_response(response)

    def _on_release_floor(self, _request: Any, response: Any) -> Any:
        """``ReleaseFloor`` — Phase 1 monitor."""
        self._log.info("ReleaseFloor received (mode=%s) — phase 1 monitor", self._mode)
        return self._fill_monitor_response(response)

    def _on_set_avatar_mode(self, _request: Any, response: Any) -> Any:
        """``SetAvatarMode`` — Phase 1 monitor (NOT_IMPLEMENTED для active)."""
        if self._mode != "monitor":
            # Phase 2: реальный FSM. Пока — refuse и остаёмся в monitor.
            self._log.warning(
                "SetAvatarMode: mode=%s запрошен, но Phase 2 не реализован (AV-6 = monitor-only). "
                "Отвечаю success=true/applied=false/reason=%s",
                self._mode,
                MONITOR_MODE_REASON,
            )
        else:
            self._log.info(
                "SetAvatarMode received (mode=%s) — phase 1 monitor", self._mode
            )
        return self._fill_monitor_response(response)

    def _fill_monitor_response(self, response: Any) -> Any:
        """Заполнить std_srvs/Trigger.response (success/message) монитор-ответом.

        Trigger.response имеет поля ``success: bool`` и ``message: string``.
        Кладём в ``message`` JSON-строку с полями ``applied`` и ``reason``
        (по ADR-0028 §4.5 клиенты должны видеть все три поля).
        """
        body = self._monitor_response()
        response.success = bool(body["success"])
        response.message = json.dumps(
            {"applied": body["applied"], "reason": body["reason"]}
        )
        return response


def main(args: Optional[list] = None) -> None:
    """Console-script entry point: ``ros2 run rob_box_supervisor supervisor_node``."""
    if not rclpy.ok():
        rclpy.init(args=args)
    node = AvatarSupervisor()
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
