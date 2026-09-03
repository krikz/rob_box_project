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
  публикация ``teleop_heartbeat`` (msgpack, см. § heartbeat).

Если в режиме ``active`` супервизор ещё не задеплоен (service-call
падает с timeout) — клиент **честно деградирует** (ADR-0018 §honesty,
ADR-0028 §4.5): поведение зависит от параметра ``supervisor_required``
(default ``False`` чтобы не уронить бот на роботе без супервизора):

* ``supervisor_required=False`` (default) — продолжаем работать
  в fallback: ``granted=True``, ``contacted_service=False``, WARN
  один раз в 60 секунд + счётчик ``supervisor_service_unavailable_total``.
  Это сохраняет работоспособность бота во время раскатки supervisor-ноды.
* ``supervisor_required=True`` — возвращаем ``granted=False``,
  ``denied_reason="supervisor_unavailable"``; handler в чате
  показывает «Супервизор недоступен, команда отклонена».

Wire format ``/avatar/state`` (AV-14, issue #1906)
--------------------------------------------------

До AV-14 этот модуль пытался декодировать ``/avatar/state`` через
``json.loads`` — что было **тихо неправильно** (издатель
сериализует msgpack в latin-1 строку, потребитель ждал JSON).
Результат: каждый ``/avatar/state`` падал в ``JSONDecodeError``,
``except`` молча проглатывал ошибку, и Telegram-бот **никогда** не
видел состояния супервизора. UI-gate (``_handle_move``) не блокировал
кнопки даже когда другой оператор держал ``teleop_floor`` — это
и был баг #1906.

AV-14 переносит единственный кодек в :mod:`rob_box_supervisor.core.state`:
``encode_for_ros_string`` / ``decode_from_ros_string``. Эта сторона
**обязана** использовать его и **не** пытаться парсить payload
самостоятельно. Если импорт кодека невозможен (например, на минимальном
CI без ``rob_box_supervisor``) — мы возвращаемся к прежнему
``AvatarState()``-default и логируем rate-limited WARN, **не**
``json.loads``-fallback (молчаливый fallback ровно то, что спрятал
баг, не повторяем).

Моки для тестов
---------------

Для unit-тестов доступны стабы:

* ``SupervisorClient.set_test_mode("always_grant"|"always_deny")`` —
  жёстко заданный ответ без обращения к ROS.
* ``SupervisorClient.set_mock_response("acquire"|"release", fn)`` —
  подменить функцию-обработчик для теста на acquire/release.

Эти хуки безопасны: они не доступны через ROS-параметры, только
через прямой вызов из тестов.

Wire-контракт (техдолг, AV-5)
------------------------------

Серверная сторона сегодня использует ``std_srvs/srv/Trigger`` с
переходным контрактом (ADR-0028 §4.5 W3-2, ADR-0028 §4.3): ``Trigger.Request``
пустой, ``client_id`` и ``floor`` читаются либо из атрибутов
``request.client_id``/``request.floor`` напрямую (симметрично будущему
кастомному IDL из AV-5), либо из JSON в ``request.data`` как fallback.
Ответ — ``response.success: bool`` + ``response.message: JSON({
applied, granted, reason, conflict_with, ...})``.

Этот модуль следует этому же контракту: в активном режиме шлёт
``client_id``/``floor`` как атрибуты + JSON-строкой в ``request.data``
(чтобы выжить и на патченных, и на непрятченных серверах). Когда AV-5
принесёт кастомный IDL, достаточно заменить ``Trigger`` на
``AcquireFloor.srv`` — клиентский код уже мапит ответ по полям body.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Optional, TYPE_CHECKING

from .observability import record_avatar_state_decode_error

# AV-14: codec lives in rob_box_supervisor.core.state. The runtime
# import is done lazily inside ``_on_state_msg`` so the bot stays
# importable in minimal CI envs without the supervisor package
# installed. The runtime fallback on import failure is "log + skip",
# NOT "fall back to json.loads" — silent fallback is exactly the bug
# #1906 we are closing.

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    pass  # только для typing-импортов, фактические импорты rclpy — ленивые


# AV-14: rate-limited WARN для декодирования /avatar/state. Не чаще
# раза в 10 секунд — иначе на 1 Гц топике с битым payload залогируем
# лишних ~10 строк/сек, и нужный сигнал утонет.
_DECODE_WARN_PERIOD_S: float = 10.0
_decode_warn_last_ts: float = 0.0
_decode_warn_lock = threading.Lock()


def _maybe_warn_decode(reason: str, exc: BaseException) -> None:
    """Записать ошибку декодирования в счётчик и (rate-limited) в лог."""
    global _decode_warn_last_ts
    try:
        record_avatar_state_decode_error(reason=reason)
    except Exception:  # noqa: BLE001 — observability никогда не валит hot path
        pass
    now = time.monotonic()
    with _decode_warn_lock:
        last = _decode_warn_last_ts
        if now - last < _DECODE_WARN_PERIOD_S:
            return
        _decode_warn_last_ts = now
    logger.warning(
        "SupervisorClient: /avatar/state decode failed (%s): %r " "(rate-limited: 1 WARN per %.0fs)",
        reason,
        exc,
        _DECODE_WARN_PERIOD_S,
    )


class Floor(str, Enum):
    """Два независимых «права», которые клиент может попросить у супервизора."""

    TELEOP = "teleop"
    VOICE = "voice"


# Имена сервисов/топиков. Должны совпадать с константами в rob_box_supervisor.
# Сервисы объявлены без ведущего "/", потому что нода supervisor сидит в
# namespace "/supervisor" (см. conftest supervisor-пакета + AV-12).
SERVICE_ACQUIRE = "/supervisor/acquire_floor"
SERVICE_RELEASE = "/supervisor/release_floor"
TOPIC_STATE = "/avatar/state"
TOPIC_HEARTBEAT = "/teleop_heartbeat"


# JSON-ключи ответа сервиса (W3-2 fix-shape, см. _acquire_floor_logic в
# rob_box_supervisor/rob_box_supervisor/supervisor_node.py).
_RESP_GRANTED = "granted"
_RESP_APPLIED = "applied"
_RESP_REASON = "reason"
_RESP_HELD_BY = "held_by"

# Специальные коды причин для деградации
REASON_GRANTED = "granted"
REASON_CONFLICT_PREFIX = "conflict:"
REASON_MONITOR = "supervisor_in_monitor_mode"
REASON_INVALID_REQUEST_PREFIX = "invalid_request:"
REASON_SUPERVISOR_UNAVAILABLE = "supervisor_unavailable"

# Параметры деградации и rate-limit
DEFAULT_SUPERVISOR_REQUIRED = False
DEFAULT_HEARTBEAT_ENABLED = True  # Публикуется только пока _is_holding(TELEOP)
WARN_RATE_LIMIT_S = 60.0
DEFAULT_RELEASE_TIMEOUT_S = 0.2


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
    # True, если клиент реально дёрнул service; False — fallback/monitor/degrad.
    contacted_service: bool = False


@dataclass
class AvatarState:
    """Снимок /avatar/state (msgpack в std_msgs/String)."""

    teleop_floor: Optional[str] = None
    voice_floor: Optional[str] = None
    mode: str = "off"  # off / telegram_active / avatar_present / mixed / ...
    since_ms: int = 0
    raw: Dict[str, Any] = field(default_factory=dict)


# Импортный хелпер: даёт None если rclpy недоступен (юнит-тесты).
def _try_import_rclpy():
    try:
        import rclpy  # noqa: F401  type: ignore
        from std_msgs.msg import String as _StdString  # type: ignore

        return _StdString
    except ImportError:
        return None


def _try_import_trigger():
    try:
        from std_srvs.srv import Trigger  # type: ignore

        return Trigger
    except ImportError:
        return None


# ── Метрики (опциональные, см. observability.py) ──────────────────────
def _record_metric(metric_id: str, **labels: str) -> None:
    """Один счётчик = одна функция-обёртка в observability.

    Без prometheus_client — no-op (как ``_NoopMetric``).
    """
    try:
        from .observability import _get_counter  # type: ignore[attr-defined]
    except (ImportError, AttributeError):
        return

    labelnames = tuple(sorted(labels.keys()))
    counter = _get_counter(metric_id, metric_id, labelnames)
    if labels:
        counter.labels(**labels).inc()
    else:
        counter.inc()


class SupervisorClient:
    """Клиент avatar_supervisor (ADR-0028 §4.4).

    Используется только из ``TelegramNode``. Один экземпляр на ноду,
    создаётся в ``__init__`` и живёт до уничтожения ноды.
    """

    # Параметры ROS 2 (читаются из declare_parameter в TelegramNode)
    DEFAULT_MODE = "monitor"  # Phase 1 default — без active-супервизора
    DEFAULT_ACQUIRE_TIMEOUT_S = 0.5
    DEFAULT_HEARTBEAT_PERIOD_S = 0.1  # 10 Гц (ADR-0028 §4.4)
    # Алиасы для обратной совместимости (SupervisorClient.SERVICE_ACQUIRE и т.п.)
    SERVICE_ACQUIRE = SERVICE_ACQUIRE
    SERVICE_RELEASE = SERVICE_RELEASE
    TOPIC_STATE = TOPIC_STATE
    TOPIC_HEARTBEAT = TOPIC_HEARTBEAT
    SUPERVISOR_REQUIRED_DEFAULT = DEFAULT_SUPERVISOR_REQUIRED

    def __init__(
        self,
        node: Any,
        client_id: str = "telegram",
        mode: str = DEFAULT_MODE,
        acquire_timeout_s: float = DEFAULT_ACQUIRE_TIMEOUT_S,
        heartbeat_period_s: float = DEFAULT_HEARTBEAT_PERIOD_S,
        supervisor_required: bool = DEFAULT_SUPERVISOR_REQUIRED,
        release_timeout_s: float = DEFAULT_RELEASE_TIMEOUT_S,
        now_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        self._node = node
        self._client_id = client_id
        self._mode = mode
        self._acquire_timeout_s = max(0.0, float(acquire_timeout_s))
        self._heartbeat_period_s = max(0.0, float(heartbeat_period_s))
        self._supervisor_required = bool(supervisor_required)
        self._release_timeout_s = max(0.0, float(release_timeout_s))
        # now_fn инжектится в тестах для fake-clock rate-limit (ADR-0018).
        self._now_fn: Callable[[], float] = now_fn or time.monotonic

        # Что мы сейчас держим (для heartbeat и release)
        self._held_floors: Dict[Floor, float] = {}
        self._held_floors_lock = threading.Lock()

        # Heartbeat
        self._heartbeat_timer: Optional[Any] = None
        self._heartbeat_pub: Optional[Any] = None

        # ROS-клиенты сервисов (создаются лениво в active-режиме).
        self._acquire_client: Optional[Any] = None
        self._release_client: Optional[Any] = None
        self._client_lock = threading.Lock()

        # State subscribers
        self._state_sub: Optional[Any] = None
        self._state: AvatarState = AvatarState()
        self._state_lock = threading.Lock()
        self._state_listeners: list[Callable[[AvatarState], None]] = []

        # Деградация / warn rate-limit
        self._last_warn_ts: float = 0.0

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

    @property
    def supervisor_required(self) -> bool:
        """Флаг деградации (ADR-0028 §4.5, ADR-0018 §honesty).

        ``False`` (default) — fallback grant + WARN при недоступности
        супервизора, чтобы не уронить бот на роботе без supervisor-ноды.
        ``True`` — строгий режим, denial при недоступности.
        """
        return self._supervisor_required

    def acquire_floor(
        self,
        floor: Floor,
        timeout_s: Optional[float] = None,
    ) -> AcquireResult:
        """Попытаться получить floor. Не блокирует telegram-поток."""
        if timeout_s is None:
            timeout_s = self._acquire_timeout_s

        # 1) Test hook — для unit-тестов (см. set_mock_response)
        if self._mock_acquire is not None:
            result = self._mock_acquire(floor=floor, client_id=self._client_id)
            if result.granted:
                self._record_held(floor)
                self._ensure_heartbeat_for_teleop(floor)
            return result

        # 2) Test-mode shortcut
        if self._test_mode == "always_grant":
            self._record_held(floor)
            self._ensure_heartbeat_for_teleop(floor)
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
            self._stop_heartbeat_if_not_holding(Floor.TELEOP)
            return

        if self._test_mode is not None:
            self._clear_held(floor)
            self._stop_heartbeat_if_not_holding(Floor.TELEOP)
            return

        if self._mode != "active":
            self._clear_held(floor)
            self._stop_heartbeat_if_not_holding(Floor.TELEOP)
            return

        self._release_via_service(floor)
        self._clear_held(floor)
        self._stop_heartbeat_if_not_holding(Floor.TELEOP)

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
        """Запустить teleop_heartbeat 10 Гц, пока держим teleop_floor.

        Idempotent: повторный вызов — no-op (return). Если публикация
        уже идёт, ничего не делаем. В monitor-режиме — no-op.
        """
        if self._mode != "active":
            return
        if self._heartbeat_timer is not None:
            return
        RosString = _try_import_rclpy()
        if RosString is None:
            return
        try:
            self._heartbeat_pub = self._node.create_publisher(
                RosString, self.TOPIC_HEARTBEAT, 10
            )
            self._heartbeat_timer = self._node.create_timer(
                self._heartbeat_period_s, self._send_heartbeat
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "SupervisorClient[%s] heartbeat start failed: %r", self._client_id, exc
            )
            self._heartbeat_pub = None
            self._heartbeat_timer = None
            return
        logger.info(
            "SupervisorClient[%s] heartbeat started (period=%.3fs)",
            self._client_id,
            self._heartbeat_period_s,
        )

    def stop_heartbeat(self) -> None:
        """Остановить heartbeat. Безопасно вызывать, даже если не запущен."""
        if self._heartbeat_timer is not None:
            try:
                self._heartbeat_timer.cancel()
            except Exception:  # noqa: BLE001
                pass
            self._heartbeat_timer = None
        self._heartbeat_pub = None

    def subscribe_state(self, listener: Callable[[AvatarState], None]) -> Callable[[], None]:
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

    def set_supervisor_required(self, value: bool) -> None:
        """Подменить политику деградации в тестах (не для прод-кода)."""
        self._supervisor_required = bool(value)

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
            self._held_floors[floor] = self._now_fn()

    def _clear_held(self, floor: Floor) -> None:
        with self._held_floors_lock:
            self._held_floors.pop(floor, None)

    def _is_holding(self, floor: Floor) -> bool:
        with self._held_floors_lock:
            return floor in self._held_floors

    def _ensure_heartbeat_for_teleop(self, floor: Floor) -> None:
        """Стартует heartbeat если держим teleop. Иначе — no-op.

        Heartbeat стартует **только** когда клиент реально держит
        teleop_floor (AV-15 acceptance, ADR-0028 §4.4 S10):
        без активного флора публиковать нечего — это лишний шум и
        пустые heartbeat-ы делают супервизорную FSM-сематику
        двусмысленной.
        """
        if floor == Floor.TELEOP:
            self.start_heartbeat()

    def _stop_heartbeat_if_not_holding(self, floor: Floor) -> None:
        if floor == Floor.TELEOP and not self._is_holding(Floor.TELEOP):
            self.stop_heartbeat()

    def _setup_active_mode(self) -> None:
        """Создать service-clients / subscriptions / publishers.

        В активном режиме клиент должен перестать быть «monitor-only»
        заглушкой: реально ходить в ``/supervisor/acquire_floor`` и
        ``/supervisor/release_floor``. Импорты rclpy — ленивые (см.
        ADR-0021 — lazy-import ceiling) и безопасные: если rclpy
        отсутствует (юнит-тесты без ROS, CI без колкона), клиент
        остаётся в «деградированном» поведении — это та же честная
        отметка «не работаем с реальным ROS», что и ADR-0018 §honesty.
        """
        RosString = _try_import_rclpy()
        if RosString is None:
            logger.warning(
                "SupervisorClient[%s] cannot create ROS interfaces: rclpy not available",
                self._client_id,
            )
            return

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
        # Подписка на /avatar/state — это безопасно: state-инфо нужно
        # даже когда service ещё не поднят (read-only).
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

        # service-clients создаём лениво (первый acquire/release),
        # потому что в момент декларации нода-нода supervisor-а может
        # ещё не успеть зарегистрировать сервис. См. _ensure_clients.

        # Heartbeat создаётся только когда держим teleop (см. start_heartbeat).

    def _ensure_clients(self) -> bool:
        """Ленивое создание service-clients. Возвращает True если готовы.

        Если клиенты не удалось создать (нет rclpy, нет ``create_client``
        у ноды, нет Trigger-типа) — возвращает ``False`` и caller должен
        выбрать fallback/monitor-reжим через ``_supervisor_required``.
        """
        with self._client_lock:
            if (
                self._acquire_client is not None
                and self._release_client is not None
            ):
                return True

            Trigger = _try_import_trigger()
            if Trigger is None:
                # std_srvs недоступен — без Trigger-типа клиент
                # не сможет сделать service-call. Это типичная ситуация
                # для юнит-тестов вне CI-образа; в прод-коде std_srvs
                # есть всегда.
                return False

            try:
                if self._acquire_client is None:
                    self._acquire_client = self._node.create_client(
                        Trigger, self.SERVICE_ACQUIRE
                    )
                if self._release_client is None:
                    self._release_client = self._node.create_client(
                        Trigger, self.SERVICE_RELEASE
                    )
            except Exception as exc:  # noqa: BLE001
                logger.warning(
                    "SupervisorClient[%s] create_client failed: %r",
                    self._client_id,
                    exc,
                )
                return False
        return True

    def _wait_for_services(self, timeout_s: float) -> bool:
        """Подождать готовности обоих сервисов. Не блокирует executor.

        Использует ``wait_for_service(timeout)`` — это polling-loop
        внутри rclpy, который проверяет discovery-graph, **но не
        вызывает** ``spin_until_future_complete`` на чужом executor.
        Это та же стратегия, что в ``supervisor_node._set_dialogue_param``
        (см. ADR-0028 §4.4 для wire-контракта).
        """
        try:
            acquire_ok = bool(
                self._acquire_client
                and self._acquire_client.wait_for_service(timeout_s)
            )
            release_ok = bool(
                self._release_client
                and self._release_client.wait_for_service(timeout_s)
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug(
                "SupervisorClient[%s] wait_for_service raised: %r", self._client_id, exc
            )
            return False
        return acquire_ok and release_ok

    def _on_state_msg(self, msg: Any) -> None:
        """Decode ``/avatar/state`` via the single codec in rob_box_supervisor.

        AV-14 (issue #1906): the publisher and this consumer MUST speak the
        same wire format, defined in :mod:`rob_box_supervisor.core.state`.
        We deliberately do **not** fall back to ``json.loads`` here — that
        silent fallback is exactly the bug we are closing.

        On any decode failure we (a) bump the
        ``avatar_state_decode_errors_total`` counter and (b) log a
        rate-limited WARN, but we do NOT update ``self._state`` (UI-gate
        would silently keep its previous value, which is preferable to
        resetting to a default-constructed ``AvatarState()`` that lies
        about "no other operator" — that lie is also a safety bug).
        """
        data = getattr(msg, "data", None)
        if not isinstance(data, str) or not data:
            _maybe_warn_decode("empty", ValueError("empty msg.data"))
            return

        try:
            from rob_box_supervisor.core.state import (  # noqa: PLC0415
                StateTransportError,
                StateVersionError,
                decode_from_ros_string,
            )
        except ImportError as exc:
            # Codec unavailable (e.g. minimal CI without rob_box_supervisor
            # installed). Same contract: log + skip, never silently default.
            _maybe_warn_decode("missing_codec", exc)
            return

        try:
            decoded = decode_from_ros_string(data)
        except (StateTransportError, StateVersionError) as exc:
            _maybe_warn_decode("transport_or_version", exc)
            return
        except Exception as exc:  # noqa: BLE001 — не валить подписку
            _maybe_warn_decode("other", exc)
            return

        # Bridge supervisor schema (FloorState dataclass with client_id +
        # since_ms + last_heartbeat_ms) → Telegram UI contract
        # (teleop_floor/voice_floor = Optional[str] client_id). Callbacks
        # (_handle_move, _on_avatar_state) and existing tests only read
        # ``.client_id``-shaped values; ``since_ms`` and the event
        # become raw fallback fields for now.
        teleop_holder: Optional[str] = decoded.teleop_floor.client_id if decoded.teleop_floor else None
        voice_holder: Optional[str] = decoded.voice_floor.client_id if decoded.voice_floor else None
        new_state = AvatarState(
            teleop_floor=teleop_holder,
            voice_floor=voice_holder,
            mode=str(decoded.mode or "off"),
            since_ms=int(decoded.since_ms or 0),
            raw={
                "mode": decoded.mode,
                "teleop_floor": (
                    {
                        "client_id": teleop_holder,
                        "since_ms": decoded.teleop_floor.since_ms,
                        "last_heartbeat_ms": decoded.teleop_floor.last_heartbeat_ms,
                    }
                    if decoded.teleop_floor
                    else None
                ),
                "voice_floor": (
                    {
                        "client_id": voice_holder,
                        "since_ms": decoded.voice_floor.since_ms,
                        "last_heartbeat_ms": decoded.voice_floor.last_heartbeat_ms,
                    }
                    if decoded.voice_floor
                    else None
                ),
                "last_event": (
                    {
                        "timestamp_ms": decoded.last_event.timestamp_ms,
                        "client_id": decoded.last_event.client_id,
                        "kind": decoded.last_event.kind,
                        "args": dict(decoded.last_event.args),
                    }
                    if decoded.last_event
                    else None
                ),
                "since_ms": decoded.since_ms,
                "version": decoded.version,
            },
        )
        with self._state_lock:
            self._state = new_state
        for listener in list(self._state_listeners):
            try:
                listener(new_state)
            except Exception as exc:  # noqa: BLE001
                logger.warning("State listener raised: %r", exc)

    # ── Service-call: acquire (Phase 2 / AV-15) ──────────────────────

    def _acquire_via_service(
        self, floor: Floor, timeout_s: float
    ) -> AcquireResult:
        """Реальный service-call ``AcquireFloor``.

        Контракт вызова (W3-2, техдолг AV-5):
        * ``Trigger.Request`` — пустой по стандарту, но мы кладём
          ``client_id``/``floor`` в атрибуты **и** дублируем JSON-строкой
          в ``request.data`` как fallback.
        * ``Trigger.Response`` — ``success: bool``, ``message: JSON({...})``
          с полями ``applied``, ``granted``, ``reason``, ``held_by``.

        Возврат соответствует ``AcquireResult`` (см. dataclass).
        """
        # Если rclpy недоступен — fallback по политике (ADR-0018 honesty).
        if _try_import_rclpy() is None:
            return self._degrade_on_unavailable(floor, reason="rclpy_unavailable")

        if not self._ensure_clients():
            return self._degrade_on_unavailable(floor, reason="client_init_failed")

        if not self._wait_for_services(timeout_s):
            _record_metric(
                "supervisor_service_unavailable_total",
                floor=floor.value,
                reason="wait_for_service_timeout",
            )
            return self._degrade_on_unavailable(floor, reason="wait_for_service_timeout")

        Trigger = _try_import_trigger()
        assert Trigger is not None  # иначе _ensure_clients вернул бы False

        request = Trigger.Request()
        # Атрибуты — для будущего IDL (AV-5).
        request.client_id = self._client_id  # type: ignore[attr-defined]
        request.floor = floor.value  # type: ignore[attr-defined]
        # Fallback JSON в request.data — для актуального сервера Trigger.
        request.data = json.dumps(  # type: ignore[attr-defined]
            {"client_id": self._client_id, "floor": floor.value},
            ensure_ascii=False,
        )

        # Делаем call_async и ждём через Event.wait — handler-поток
        # (asyncio-loop telegram) не блокирует rclpy executor другой ноды,
        # потому что callback-и rclpy ставят Event из своего потока, а
        # мы ждём в asyncio-loop, не в rclpy. Это та же стратегия,
        # что supervisor_node._set_dialogue_param (ADR-0028 §4.4).
        event = threading.Event()

        try:
            future = self._acquire_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "SupervisorClient[%s] call_async(acquire) raised: %r",
                self._client_id,
                exc,
            )
            return self._degrade_on_unavailable(floor, reason="call_async_failed")

        local_result: Dict[str, Any] = {}

        def _done(fut: Any) -> None:
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                local_result["exc"] = exc
            else:
                local_result["resp"] = resp
            finally:
                event.set()

        future.add_done_callback(_done)

        if not event.wait(timeout=timeout_s):
            _record_metric(
                "supervisor_service_unavailable_total",
                floor=floor.value,
                reason="service_timeout",
            )
            return self._degrade_on_unavailable(floor, reason="service_timeout")

        if "exc" in local_result:
            _record_metric(
                "supervisor_service_unavailable_total",
                floor=floor.value,
                reason="service_exception",
            )
            return self._degrade_on_unavailable(
                floor, reason="service_exception"
            )

        resp = local_result["resp"]
        result, granted = self._parse_acquire_response(resp, floor)
        # В ответе сервиса есть granted → обновляем метрику и состояние.
        if granted:
            self._record_held(floor)
            self._ensure_heartbeat_for_teleop(floor)
            _record_metric(
                "supervisor_acquire_total",
                floor=floor.value,
                result=REASON_GRANTED,
            )
        else:
            _record_metric(
                "supervisor_acquire_total",
                floor=floor.value,
                result=result.denied_reason or "denied",
            )
        return result

    def _parse_acquire_response(
        self, resp: Any, floor: Floor
    ) -> tuple[AcquireResult, bool]:
        """Парсит Trigger.Response в ``AcquireResult``. Возвращает (result, granted).

        Совместим со всеми вариантами ответа серверной стороны
        (см. rob_box_supervisor.supervisor_node._on_acquire_floor /
        _fill_floor_response):
        * ``response.success=False`` — service вообще не пришёл / упал;
        * ``response.message`` — JSON со всеми или частью ключей
          ``granted``/``applied``/``reason``/``held_by``.
        """
        success = bool(getattr(resp, "success", False))
        raw_message = getattr(resp, "message", "") or ""

        body: Dict[str, Any] = {}
        if raw_message:
            try:
                parsed = json.loads(raw_message)
                if isinstance(parsed, dict):
                    body = parsed
            except (json.JSONDecodeError, TypeError):
                # Серверная сторона может положить в message что угодно
                # (например, упрощённый Trigger-ответ для совместимости);
                # не валим, продолжаем с пустым body.
                pass

        granted = bool(body.get(_RESP_GRANTED, success))
        applied = body.get(_RESP_APPLIED, granted)
        reason = str(body.get(_RESP_REASON, "") or "")
        held_by = body.get(_RESP_HELD_BY)

        if not applied and not success:
            # Сервис сообщил «не применил» (например, supervisor в monitor-режиме).
            return (
                AcquireResult(
                    granted=False,
                    denied_reason=REASON_MONITOR,
                    contacted_service=True,
                ),
                False,
            )

        if granted:
            return (
                AcquireResult(
                    granted=True,
                    contacted_service=True,
                ),
                True,
            )

        # Сервис применил запрос, но отказал. Обычно это "conflict:
        # held_by=quest" — для UI передаём held_by.
        denied_reason = "held_by_other"
        if reason.startswith(REASON_CONFLICT_PREFIX) and held_by:
            denied_reason = "held_by_other"
        elif reason.startswith(REASON_INVALID_REQUEST_PREFIX):
            denied_reason = "invalid_request"

        return (
            AcquireResult(
                granted=False,
                denied_reason=denied_reason,
                held_by=str(held_by) if held_by else None,
                contacted_service=True,
            ),
            False,
        )

    def _release_via_service(self, floor: Floor) -> None:
        """Реальный release через service. Best-effort, ошибку логируем."""
        if _try_import_rclpy() is None or not self._ensure_clients():
            return

        Trigger = _try_import_trigger()
        assert Trigger is not None

        request = Trigger.Request()
        request.client_id = self._client_id  # type: ignore[attr-defined]
        request.floor = floor.value  # type: ignore[attr-defined]
        request.data = json.dumps(  # type: ignore[attr-defined]
            {"client_id": self._client_id, "floor": floor.value},
            ensure_ascii=False,
        )

        try:
            future = self._release_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "SupervisorClient[%s] release call_async failed: %r",
                self._client_id,
                exc,
            )
            return

        event = threading.Event()

        def _done(fut: Any) -> None:
            try:
                fut.result()
            except Exception as exc:  # noqa: BLE001
                logger.debug(
                    "SupervisorClient[%s] release future error: %r", self._client_id, exc
                )
            finally:
                event.set()

        future.add_done_callback(_done)
        # Release не должен «залипнуть» — ждём только короткий таймаут.
        event.wait(timeout=self._release_timeout_s)

    def _degrade_on_unavailable(
        self, floor: Floor, reason: str
    ) -> AcquireResult:
        """Честная деградация при недоступности сервиса (ADR-0018 §honesty).

        ``supervisor_required=False`` (default): grant local + WARN rate-limit.
        ``supervisor_required=True``: deny с reason="supervisor_unavailable".
        """
        if not self._supervisor_required:
            self._maybe_warn_unavailable(reason)
            self._record_held(floor)
            self._ensure_heartbeat_for_teleop(floor)
            return AcquireResult(
                granted=True,
                denied_reason=reason,
                contacted_service=False,
            )

        return AcquireResult(
            granted=False,
            denied_reason=REASON_SUPERVISOR_UNAVAILABLE,
            contacted_service=False,
        )

    def _maybe_warn_unavailable(self, reason: str) -> None:
        """WARN один раз в WARN_RATE_LIMIT_S (защита от лог-spam)."""
        now = self._now_fn()
        if now - self._last_warn_ts < WARN_RATE_LIMIT_S:
            return
        self._last_warn_ts = now
        logger.warning(
            "SupervisorClient[%s] supervisor unavailable "
            "(reason=%s, supervisor_required=false → fallback grant). "
            "WARN rate-limited to once per %.0fs — see ADR-0028 §4.5.",
            self._client_id,
            reason,
            WARN_RATE_LIMIT_S,
        )

    def _send_heartbeat(self) -> None:
        if not self._is_holding(Floor.TELEOP):
            self.stop_heartbeat()
            return
        if self._heartbeat_pub is None:
            return
        RosString = _try_import_rclpy()
        if RosString is None:
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
