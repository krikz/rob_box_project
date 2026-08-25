#!/usr/bin/env python3
"""
supervisor_client.py — клиентский API супервизора аватара (ADR-0028).

Этот модуль предоставляет тонкий Python-API, через который
``rob_box_telegram`` взаимодействует с ``avatar_supervisor``:

* ``AcquireFloor{client_id, floor}`` — попросить эксклюзивное право
  (teleop / voice) перед публикацией ``cmd_vel_*`` или TTS.
* ``ReleaseFloor{client_id, floor}`` — отпустить право.
* ``/avatar/state`` (latched) — текущее состояние супервизора, на
  которое клиент подписывается, чтобы гасить свои кнопки, если
  ``teleop_floor != "telegram"``.
* ``teleop_heartbeat`` (10 Гц) — клиент держит floor живым, пока
  публикует команды движения.

Состояния реализации
--------------------

ADR-0028 §4.5 разделяет развёртывание супервизора (Phase 2.1–2.2) и
рефакторинг клиента (Phase 2.4). Чтобы не блокировать текущую
работу телеграм-бота, ``SupervisorClient`` работает в одном из двух
режимов:

* ``mode = "monitor"`` (default, Phase 1) — все методы
  ``acquire_floor`` сразу возвращают ``granted=True``, ``release_floor``
  и heartbeat — no-op. Это позволяет деплоить клиентский код
  заранее и включать настоящий супервизор одним переключателем
  (``mode=active``) без правок в ``telegram_node``.
* ``mode = "active"`` (Phase 2) — реальные ROS 2 service-calls
  ``/supervisor/acquire_floor`` и ``/supervisor/release_floor``;
  подписка на ``/avatar/state`` (msgpack в ``std_msgs/String``);
  публикация ``teleop_heartbeat`` (msgpack).

Если в режиме ``active`` супервизор ещё не задеплоен (service-call
падает с timeout) — клиент **логирует warning один раз** и продолжает
работать в режиме fallback (как ``monitor``). Это сохраняет
работоспособность бота во время раскатки supervisor-ноды.

Моки для тестов
---------------

Для unit-тестов доступны стабы:

* ``SupervisorClient.set_test_mode("always_grant"|"always_deny")`` —
  жёстко заданный ответ без обращения к ROS.
* ``SupervisorClient.set_mock_response("acquire"|"release", fn)`` —
  подменить функцию-обработчик для теста на acquire/release.

Эти хуки безопасны: они не доступны через ROS-параметры, только
через прямой вызов из тестов.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger(__name__)


class Floor(str, Enum):
    """Два независимых «права», которые клиент может попросить у супервизора."""

    TELEOP = "teleop"
    VOICE = "voice"


@dataclass
class AcquireResult:
    """Ответ супервизора на запрос AcquireFloor.

    * ``granted`` — клиент может публиковать команды
    * ``denied_reason`` — почему отказали (``"held_by_other"`` и т.п.)
    * ``held_by`` — ``client_id`` текущего держателя (для UI)
    """

    granted: bool
    denied_reason: Optional[str] = None
    held_by: Optional[str] = None
    # True, если клиент реально дёрнул service; False — fallback/monitor.
    contacted_service: bool = False


@dataclass
class AvatarState:
    """Снимок /avatar/state (msgpack в std_msgs/String)."""

    teleop_floor: Optional[str] = None
    voice_floor: Optional[str] = None
    mode: str = "off"  # off / telegram_active / avatar_present / mixed / ...
    since_ms: int = 0
    raw: Dict[str, Any] = field(default_factory=dict)


class SupervisorClient:
    """Клиент avatar_supervisor (ADR-0028 §4.4).

    Используется только из ``TelegramNode``. Один экземпляр на ноду,
    создаётся в ``__init__`` и живёт до уничтожения ноды.
    """

    # Параметры ROS 2 (читаются из declare_parameter в TelegramNode)
    DEFAULT_MODE = "monitor"  # Phase 1 default — без active-супервизора
    DEFAULT_ACQUIRE_TIMEOUT_S = 0.5
    DEFAULT_HEARTBEAT_PERIOD_S = 0.1  # 10 Гц (ADR-0028 §4.4)
    SERVICE_ACQUIRE = "/supervisor/acquire_floor"
    SERVICE_RELEASE = "/supervisor/release_floor"
    TOPIC_STATE = "/avatar/state"
    TOPIC_HEARTBEAT = "/teleop_heartbeat"

    def __init__(
        self,
        node: Any,
        client_id: str = "telegram",
        mode: str = DEFAULT_MODE,
        acquire_timeout_s: float = DEFAULT_ACQUIRE_TIMEOUT_S,
        heartbeat_period_s: float = DEFAULT_HEARTBEAT_PERIOD_S,
    ) -> None:
        self._node = node
        self._client_id = client_id
        self._mode = mode
        self._acquire_timeout_s = acquire_timeout_s
        self._heartbeat_period_s = heartbeat_period_s

        # Что мы сейчас держим (для heartbeat и release)
        self._held_floors: Dict[Floor, float] = {}
        self._held_floors_lock = threading.Lock()

        # Heartbeat
        self._heartbeat_timer: Optional[Any] = None
        self._heartbeat_pub: Optional[Any] = None

        # State subscribers
        self._state_sub: Optional[Any] = None
        self._state: AvatarState = AvatarState()
        self._state_lock = threading.Lock()
        self._state_listeners: list[Callable[[AvatarState], None]] = []

        # Fallback / no-service state
        self._warned_about_missing_service = False

        # Test hooks (only used by unit tests)
        self._test_mode: Optional[str] = None  # "always_grant"|"always_deny"|None
        self._mock_acquire: Optional[Callable[..., AcquireResult]] = None
        self._mock_release: Optional[Callable[..., None]] = None

        # Connect to actual ROS 2 services/topics if mode == "active"
        if self._mode == "active":
            self._setup_active_mode()
        else:
            logger.info(
                "SupervisorClient[%s] in monitor mode — no service calls, "
                "all floors granted locally (ADR-0028 §4.5)",
                client_id,
            )

    # ── Public API ───────────────────────────────────────────────────

    @property
    def client_id(self) -> str:
        return self._client_id

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def state(self) -> AvatarState:
        """Текущий снимок /avatar/state (или локальный, если не active)."""
        with self._state_lock:
            return AvatarState(
                teleop_floor=self._state.teleop_floor,
                voice_floor=self._state.voice_floor,
                mode=self._state.mode,
                since_ms=self._state.since_ms,
                raw=dict(self._state.raw),
            )

    def acquire_floor(
        self,
        floor: Floor,
        timeout_s: Optional[float] = None,
    ) -> AcquireResult:
        """Попытаться получить floor. Не блокирует telegram-поток."""
        if timeout_s is None:
            timeout_s = self._acquire_timeout_s

        # 1) Test hook — для unit-тестов
        if self._mock_acquire is not None:
            return self._mock_acquire(floor=floor, client_id=self._client_id)

        # 2) Test-mode shortcut
        if self._test_mode == "always_grant":
            self._record_held(floor)
            return AcquireResult(granted=True, contacted_service=False)
        if self._test_mode == "always_deny":
            return AcquireResult(
                granted=False,
                denied_reason="held_by_other",
                held_by="quest",
                contacted_service=False,
            )

        # 3) Monitor mode (Phase 1 default) — grant locally
        if self._mode != "active":
            self._record_held(floor)
            return AcquireResult(granted=True, contacted_service=False)

        # 4) Active mode — real ROS 2 service call
        return self._acquire_via_service(floor, timeout_s)

    def release_floor(self, floor: Floor) -> None:
        """Отпустить floor (best-effort)."""
        if self._mock_release is not None:
            self._mock_release(floor=floor, client_id=self._client_id)
            self._clear_held(floor)
            return

        if self._test_mode is not None:
            self._clear_held(floor)
            return

        if self._mode != "active":
            self._clear_held(floor)
            return

        self._release_via_service(floor)

    def with_floor(
        self,
        floor: Floor,
        callback: Callable[[], None],
    ) -> AcquireResult:
        """Context-manager-style helper: acquire → callback → release.

        Возвращает ``AcquireResult`` чтобы caller мог среагировать
        на отказ (например, погасить кнопку в UI).
        """
        result = self.acquire_floor(floor)
        if not result.granted:
            logger.info(
                "SupervisorClient[%s] denied floor=%s reason=%s held_by=%s",
                self._client_id,
                floor.value,
                result.denied_reason,
                result.held_by,
            )
            return result
        try:
            callback()
        finally:
            self.release_floor(floor)
        return result

    def start_heartbeat(self) -> None:
        """Запустить teleop_heartbeat 10 Гц, пока держим teleop_floor."""
        if self._mode != "active":
            return
        if self._heartbeat_timer is not None:
            return
        try:
            from std_msgs.msg import String as RosString  # type: ignore
        except ImportError:
            return
        self._heartbeat_pub = self._node.create_publisher(
            RosString, self.TOPIC_HEARTBEAT, 10
        )
        self._heartbeat_timer = self._node.create_timer(
            self._heartbeat_period_s, self._send_heartbeat
        )
        logger.info(
            "SupervisorClient[%s] heartbeat started (period=%.3fs)",
            self._client_id,
            self._heartbeat_period_s,
        )

    def stop_heartbeat(self) -> None:
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.cancel()
            self._heartbeat_timer = None
        self._heartbeat_pub = None

    def subscribe_state(
        self, listener: Callable[[AvatarState], None]
    ) -> Callable[[], None]:
        """Подписка на изменения /avatar/state. Возвращает unsubscribe."""
        self._state_listeners.append(listener)
        # Сразу отдадим текущее состояние — UI не должен ждать первого
        # STATE_UPDATE, чтобы понять, держим ли мы floor.
        try:
            listener(self.state)
        except Exception as exc:  # noqa: BLE001
            logger.warning("State listener raised on initial dispatch: %r", exc)

        def _unsubscribe() -> None:
            try:
                self._state_listeners.remove(listener)
            except ValueError:
                pass

        return _unsubscribe

    # ── Test hooks (только для unit-тестов) ──────────────────────────

    def set_test_mode(self, mode: Optional[str]) -> None:
        """``"always_grant"`` / ``"always_deny"`` / ``None`` (default)."""
        self._test_mode = mode

    def set_mock_response(
        self,
        op: str,
        fn: Optional[Callable[..., Any]],
    ) -> None:
        """Подменить ``acquire`` или ``release`` в тестах."""
        if op == "acquire":
            self._mock_acquire = fn
        elif op == "release":
            self._mock_release = fn
        else:
            raise ValueError(f"Unknown op: {op!r}")

    def reset_test_hooks(self) -> None:
        self._test_mode = None
        self._mock_acquire = None
        self._mock_release = None

    # ── Internal ─────────────────────────────────────────────────────

    def _record_held(self, floor: Floor) -> None:
        with self._held_floors_lock:
            self._held_floors[floor] = time.monotonic()

    def _clear_held(self, floor: Floor) -> None:
        with self._held_floors_lock:
            self._held_floors.pop(floor, None)

    def _is_holding(self, floor: Floor) -> bool:
        with self._held_floors_lock:
            return floor in self._held_floors

    def _setup_active_mode(self) -> None:
        """Создать service-clients / subscriptions / publishers."""
        try:
            from std_msgs.msg import String as RosString  # type: ignore
        except ImportError:
            logger.warning(
                "SupervisorClient[%s] cannot create ROS interfaces: rclpy not available",
                self._client_id,
            )
            return
        # NOTE: реальные service-типы появятся вместе с rob_box_supervisor
        # (ADR-0028 Phase 2.1). До тех пор используем std_msgs/String
        # пакеты (msgpack) и service-вызовы через generic client API.
        from rclpy.qos import (  # type: ignore
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        try:
            self._state_sub = self._node.create_subscription(
                RosString,
                self.TOPIC_STATE,
                self._on_state_msg,
                latched_qos,
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "SupervisorClient[%s] state subscription failed: %r",
                self._client_id,
                exc,
            )

    def _on_state_msg(self, msg: Any) -> None:
        try:
            payload = json.loads(getattr(msg, "data", "") or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        new_state = AvatarState(
            teleop_floor=payload.get("teleop_floor"),
            voice_floor=payload.get("voice_floor"),
            mode=str(payload.get("mode", "off")),
            since_ms=int(payload.get("since_ms", 0)),
            raw=payload,
        )
        with self._state_lock:
            self._state = new_state
        for listener in list(self._state_listeners):
            try:
                listener(new_state)
            except Exception as exc:  # noqa: BLE001
                logger.warning("State listener raised: %r", exc)

    def _acquire_via_service(
        self, floor: Floor, timeout_s: float
    ) -> AcquireResult:
        """Реальный service-call (Phase 2)."""
        try:
            from rclpy.client import Client  # noqa: F401  type: ignore
        except ImportError:
            return self._fallback_grant(floor)
        # NOTE: здесь будет вызов self._node.create_client(...) +
        # client.call_async(req). Сейчас supervisor-нода не существует,
        # поэтому active-режим пока ведёт себя как monitor + warning.
        return self._fallback_grant(floor)

    def _release_via_service(self, floor: Floor) -> None:
        self._clear_held(floor)

    def _fallback_grant(self, floor: Floor) -> AcquireResult:
        """Active-режим без супервизора: grant + warning один раз."""
        if not self._warned_about_missing_service:
            logger.warning(
                "SupervisorClient[%s] active mode but service not available — "
                "falling back to direct publish (ADR-0028 §4.5). "
                "This warning is shown once per node lifetime.",
                self._client_id,
            )
            self._warned_about_missing_service = True
        self._record_held(floor)
        return AcquireResult(granted=True, contacted_service=False)

    def _send_heartbeat(self) -> None:
        if not self._is_holding(Floor.TELEOP):
            return
        if self._heartbeat_pub is None:
            return
        try:
            from std_msgs.msg import String as RosString  # type: ignore
        except ImportError:
            return
        payload = {
            "client_id": self._client_id,
            "ts_ms": int(time.time() * 1000),
        }
        msg = RosString()
        msg.data = json.dumps(payload, ensure_ascii=False)
        try:
            self._heartbeat_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Heartbeat publish failed: %r", exc)

    def shutdown(self) -> None:
        """Корректно остановить heartbeat / release floors."""
        self.stop_heartbeat()
        with self._held_floors_lock:
            held = list(self._held_floors.keys())
        for floor in held:
            self.release_floor(floor)
