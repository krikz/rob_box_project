"""AvatarSupervisor — ROS 2 нода-координатор аватара (Phase 1+2, AV-6/AV-X).

Дизайн:
- Параметр ``mode`` (default ``"monitor"``). В ``monitor``-режиме нода
  публикует ``/avatar/state`` из агрегатора и **отвечает** на сервисы
  ``AcquireFloor`` / ``ReleaseFloor`` / ``SetAvatarMode`` сообщением
  ``success=true, applied=false, reason="supervisor_in_monitor_mode"``,
  **не** меняя ``twist_mux`` inputs и ``dialogue_node`` параметры
  (ADR-0028 §4.5). Это минимизирует blast radius: нода задеплоена и
  наблюдает, реальное влияние — после явного ``mode:=active``.
- Phase 2 (``active``-режим, AV-X = эта карточка) подключает pure-Python
  ``ModeManager`` (AV-3) и ``LockManager`` (AV-4) к ROS 2 service
  handlers: AcquireFloor/ReleaseFloor делегируют floor-логику, FSM
  переводит mode, ``/avatar/state`` публикуется через ``core/state``
  msgpack-схему. В Phase 1 поведение backward-compat: нода в
  ``mode=monitor`` НЕ создаёт ``_mode_manager/_lock_manager`` и НЕ
  трогает ничего вне себя (S12, ADR-0028 §4.5).

Transport для service-контракта (ВАЖНО — trade-off, см. commit msg):
- В этой карточке используем ``std_srvs/Trigger`` для backward-compat с
  Phase 1 e2e (AV-11 делает ``ros2 service call /acquire_floor
  std_srvs/srv/Trigger``). ``client_id``/``floor`` передаются через
  JSON-конвенцию в ``Trigger.Request.data`` (расширение convention).
  В проде rclpy ``Trigger.Request`` пустой, поэтому client_id и floor
  берутся из default-значений ``("unknown", "voice_floor")`` если
  data не парсится. Канонический IDL сохранён в
  ``srv/AcquireFloor.srv`` / ``srv/ReleaseFloor.srv`` (SOT) — миграция
  на ament_cmake + rosidl = отдельная карточка Phase 2.1.

Источники истины:
- ADR-0028 §4.1 (FSM)
- ADR-0028 §4.2 (LockManager)
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
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool as RosBool
from std_msgs.msg import String as RosString

# Phase 2 (AV-X) — pure-Python компоненты из core/. Импортируем явно
# на module-уровне, чтобы type hints (``_build_avatar_state`` return
# ``AvatarState``) работали в runtime + чтобы fail-fast при нарушении
# контракта ``core/__init__.py``.
from rob_box_supervisor.core import (  # noqa: F401,E402
    AvatarState,
    FLOOR_TELEOP,
    FLOOR_VOICE,
    FloorState,
    LockManager,
    ModeManager,
    pack,
)
from rob_box_supervisor.core.fsm import EVENT_FORCE_OFF  # noqa: E402
from rob_box_supervisor.core.locks import DEAD_MAN_TIMEOUT_MS  # noqa: E402

# msgpack — Phase 1+2 wire-format для /avatar/state (см. ADR-0028 §4.3).
# core/state.py даёт полный IDL (dataclasses + pack/unpack) — мы их
# переиспользуем в Phase 2, чтобы не плодить два msgpack-сериализатора.
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

# ADR-0027 §3.4 — валидные значения ``voice_input_mode`` на dialogue_node.
# Супервизор — единственная точка, которая имеет право их менять (ADR-0028 S5).
VOICE_INPUT_MODES: tuple[str, ...] = (
    "respeaker",
    "quest_passthrough",
    "quest_ttts",
    "quest_stt",
    "quest_llm_formalize",
)

# Phase 1 транспорт запроса смены режима голоса. Phase 2 заменит на
# ``SetVoiceMode``-сервис с кастомным IDL (ADR-0028 §4.3) — здесь топик
# достаточен, чтобы не плодить rosidl-интерфейсы ради monitor-фазы.
SET_VOICE_MODE_TOPIC: str = "/avatar/set_voice_mode"

# Топик для heartbeat-сообщений от клиентов (msgpack или JSON:
# ``{"client_id": str, "ts_ms": int, "floor": str}``). Контракт:
# - Клиент шлёт ≥ 10 Гц пока держит ``teleop_floor`` / ``voice_floor``.
# - Без heartbeat > DEAD_MAN_TIMEOUT_MS — watchdog снимает floor
#   (ADR-0028 §4.4 S10 + §6 Q4).
# Phase 1/Phase 2 default: один общий топик (мульти-floor); per-floor
# топики = Phase 2.1 (отдельная карточка).
HEARTBEAT_TOPIC: str = "/avatar/heartbeat"

# Boolean-топик для twist_mux lock (ADR-0028 §3 / §4.2): когда
# ``teleop_floor`` держит клиент, супервизор публикует ``True``,
# блокируя nav2/web/voice priority (см. ``docker/main/config/twist_mux/twist_mux.yaml``).
# Когда floor отпущен (release или dead-man) — ``False``.
# S12: В monitor-режиме НЕ публикуем (минимизируем blast radius).
TELEOP_LOCK_TOPIC: str = "/teleop_lock"

# Dead-man watchdog period. Должно быть <= DEAD_MAN_TIMEOUT_MS / 2,
# чтобы trip-реакция укладывалась в < 500 мс. ADR-0028 §6 Q4 Q&A.
DEAD_MAN_TICK_PERIOD_S: float = 0.1  # 10 Hz — достаточно для 500 ms порога.


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
        # вмешиваемся. В active — Phase 2 wire-up (FSM + LockManager).
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

        # Phase 2 (AV-X) — ModeManager+LockManager+AvatarState импортированы
        # на module-уровне (см. начало файла). ModeManager+LockManager
        # создаются лениво при первом active-вызове (``_ensure_active_handlers``);
        # в monitor-mode нода не должна их создавать (S12, ADR-0028 §4.5
        # — минимизируем blast radius).
        self._mode_manager: Optional[ModeManager] = None
        self._lock_manager: Optional[LockManager] = None
        # ``_active_inited`` — флаг "первый active-вызов обработан", чтобы
        # не пересоздавать менеджеры (и не терять state) на каждом запросе.
        self._active_inited: bool = False
        # ``_floor_acquired_at_ms[floor]`` — для FloorState.since_ms.
        self._floor_acquired_at_ms: Dict[str, int] = {}
        # ``_last_heartbeat_ms[(client_id, floor)]`` — для FloorState.last_heartbeat_ms.
        self._last_heartbeat_ms: Dict[Tuple[str, str], int] = {}

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
        # ADR-0028 S5 — супервизор единственный, кто меняет voice_input_mode
        # на dialogue_node. Phase 1 транспорт — топик (см. SET_VOICE_MODE_TOPIC).
        self.create_subscription(
            RosString, SET_VOICE_MODE_TOPIC, self._on_set_voice_mode, 10
        )
        # Heartbeat от клиентов (Phase 2 active). Параметр-клиент к
        # dialogue_node создаётся лениво в active-режиме (в monitor
        # супервизор НЕ трогает чужие параметры — S12).
        self._dialogue_param_client = None

        # Heartbeat subscription (Phase 2 active, ADR-0028 §4.4 S10).
        # В monitor-режиме подписка зарегистрирована, но callback no-op
        # (S12: minimize blast radius; consumers still see consistent
        # ``/avatar/state`` via aggregator).
        self.create_subscription(RosString, HEARTBEAT_TOPIC, self._on_heartbeat_msg, 10)

        # Twist_mux teleop_lock publisher (Phase 2 active, ADR-0028 §4.2).
        # В monitor нода не вмешивается в twist_mux (S12). Publisher
        # создаётся лениво на первом active-acquire teleop_floor или
        # явно в ``_ensure_active_handlers`` — НЕ здесь, чтобы
        # monitor-нода не держала лишний topic на стороне twist_mux
        # (вдруг ``/teleop_lock`` ещё не зарегистрирован в twist_mux.yaml
        # Phase 1.3).
        self._teleop_lock_pub: Optional[Any] = None

        # Dead-man watchdog (Phase 2 active). Периодический tick
        # проверяет ``LockManager.holder()`` для каждого floor-а и при
        # expired — снимает floor + FSM EVENT_FORCE_OFF (escape hatch).
        # В monitor-режиме таймер ВСЁ РАВНО тикает (rclpy создаёт timer в
        # __init__), но callback делает no-op (``_mode_manager is None``),
        # чтобы сохранить прозрачный fallback для тестов, где нода
        # переключается в active динамически.
        self._dead_man_timer = self.create_timer(
            DEAD_MAN_TICK_PERIOD_S, self._dead_man_tick
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
        """Залогировать env и mode на старте — помогает e2e/postmortem.

        Используем f-string + одиночный ``msg`` вместо ``info(fmt, *args)``:
        rclpy ``RcutilsLogger.info`` принимает ``(msg, *args)``, где *args — это
        позиционные параметры для ``%``-форматирования msg, а не самостоятельные
        поля. Вызов с 3+ args (например, ``info(fmt, a, b, c)``) ломает рантайм
        ``TypeError: RcutilsLogger.info() takes 2 positional arguments but N were given``
        (issue #1644, run #32892615440). Тестировано в карточке t_369751b4.
        """
        zenoh = os.environ.get("ZENOH_SESSION_CONFIG_URI", "<unset>")
        msgpack_state = "ok" if _HAS_MSGPACK else "MISSING"
        self._log.info(
            f"avatar_supervisor started: mode={self._mode}, "
            f"zenoh={zenoh}, msgpack={msgpack_state}"
        )

    def _monitor_response(self) -> dict:
        """Стандартный ответ для всех сервисов в monitor-режиме."""
        return {
            "success": True,
            "applied": False,
            "reason": MONITOR_MODE_REASON,
        }

    def _publish_avatar_state(self) -> None:
        """Timer-callback: публикует свежий snapshot в /avatar/state.

        Phase 1: msgpack-encoded dict из ``StateAggregator.snapshot()``.
        Phase 2: в active-режиме публикуем ``AvatarState`` через
        ``core/state.pack`` (AV-5 IDL), чтобы клиенты (Quest, Telegram)
        видели ``mode``/``floors``/``last_event`` через единый msgpack-
        контракт. В monitor-mode остаётся Phase 1 dict-payload
        (backward-compat с существующими e2e).
        """
        if self._mode == "active" and self._mode_manager is not None:
            # Phase 2: canonical AvatarState msgpack (см. core/state.py).
            state = self._build_avatar_state()
            if _HAS_MSGPACK:
                payload = pack(state)
            else:  # pragma: no cover — defensive
                # Fallback на JSON, если msgpack недоступен (теоретически).
                from dataclasses import asdict

                payload = json.dumps(asdict(state)).encode("utf-8")
            msg = RosString()
            # std_msgs/String — UTF-8 safe для bytes-as-text.
            msg.data = payload.decode("latin-1")
            self._state_pub.publish(msg)
            return

        # Phase 1 (monitor mode): legacy dict-payload.
        snapshot = self._aggregator.snapshot()
        if _HAS_MSGPACK:
            try:
                payload = msgpack.packb(snapshot.to_msgpack_dict(), use_bin_type=True)
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"avatar_supervisor: msgpack encode failed: {exc}")
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

    # ── voice mode (ADR-0028 S5) ─────────────────────────────────────
    def _on_set_voice_mode(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_mode`` — запрос сменить режим голоса.

        Единственная точка, которая имеет право менять ``voice_input_mode``
        на ``dialogue_node`` (ADR-0028 S5). В monitor-режиме принимаем и
        логируем, но НЕ применяем (S12); в active — выставляем параметр.
        """
        mode = (msg.data or "").strip()
        applied, reason = self._apply_voice_mode(mode)
        # f-string: RcutilsLogger принимает ОДИН msg (issue #1644).
        self._log.info(f"SetVoiceMode: mode={mode} applied={applied} reason={reason}")

    def _apply_voice_mode(self, mode: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_input_mode`` (тестируется без rclpy).

        Возвращает ``(applied, reason)``. Не валит ноду на битом входе.
        """
        if mode not in VOICE_INPUT_MODES:
            return False, f"invalid_voice_mode: {mode!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        try:
            self._set_dialogue_param("voice_input_mode", mode)
        except Exception as exc:  # noqa: BLE001 — отказ не должен валить ноду
            self._log.warning(f"SetVoiceMode: failed to set dialogue param: {exc}")
            return False, f"param_set_failed: {exc}"
        return True, "applied"

    def _set_dialogue_param(self, name: str, value: str) -> None:
        """Выставить string-параметр на ``dialogue_node`` через SetParameters.

        Клиент создаётся лениво (первый вызов в active-режиме). Вызов
        асинхронный (rclpy client), результат логируем в done-callback —
        в monitor-режиме метод не вызывается вовсе (S12).
        """
        if self._dialogue_param_client is None:
            from rcl_interfaces.srv import SetParameters  # noqa: PLC0415

            self._dialogue_param_client = self.create_client(
                SetParameters, "/dialogue_node/set_parameters"
            )
        from rcl_interfaces.msg import (  # noqa: PLC0415
            Parameter,
            ParameterType,
            ParameterValue,
        )
        from rcl_interfaces.srv import SetParameters  # noqa: PLC0415

        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = value
        req.parameters = [param]

        future = self._dialogue_param_client.call_async(req)

        def _done(fut) -> None:
            try:
                res = fut.result()
                ok = bool(res and res.results and res.results[0].successful)
                if not ok:
                    self._log.warning(
                        "SetVoiceMode: dialogue_node rejected parameter set"
                    )
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"SetVoiceMode: parameter set failed: {exc}")

        future.add_done_callback(_done)

    # ── service callbacks (Phase 1: monitor → log + answer) ──────────
    def _on_acquire_floor(self, request: Any, response: Any) -> Any:
        """``AcquireFloor`` — Phase 1+2.

        - ``mode=monitor`` (default): backward-compat monitor response.
        - ``mode=active``: Phase 2 wire-up — LockManager.acquire +
          ModeManager.transition + публикация AvatarState в response.
          ``client_id``/``floor`` берутся из ``request.data`` (JSON) или
          из defaults ``("unknown", "voice_floor")`` если data не парсится.
        """
        if self._mode != "active":
            self._log.info(
                f"AcquireFloor received (mode={self._mode}) — phase 1 monitor"
            )
            return self._fill_monitor_response(response)

        # Phase 2 active-путь.
        self._ensure_active_handlers()
        client_id, floor = self._parse_floor_request(
            request, default_floor="voice_floor"
        )
        try:
            body = self._acquire_floor_active(client_id=client_id, floor=floor)
        except Exception as exc:  # noqa: BLE001 — фатальные ошибки отдаём клиенту
            self._log.warning(f"AcquireFloor: unexpected error: {exc!r}")
            return self._fill_error_response(
                response, reason=f"internal_error: {exc!r}"
            )
        return self._fill_active_response(response, body)

    def _on_release_floor(self, request: Any, response: Any) -> Any:
        """``ReleaseFloor`` — Phase 1+2 (симметрично ``_on_acquire_floor``)."""
        if self._mode != "active":
            self._log.info(
                f"ReleaseFloor received (mode={self._mode}) — phase 1 monitor"
            )
            return self._fill_monitor_response(response)

        self._ensure_active_handlers()
        client_id, floor = self._parse_floor_request(
            request, default_floor="voice_floor"
        )
        try:
            body = self._release_floor_active(client_id=client_id, floor=floor)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"ReleaseFloor: unexpected error: {exc!r}")
            return self._fill_error_response(
                response, reason=f"internal_error: {exc!r}"
            )
        return self._fill_active_response(response, body)

    def _on_set_avatar_mode(self, _request: Any, response: Any) -> Any:
        """``SetAvatarMode`` — Phase 1 monitor (NOT_IMPLEMENTED для active)."""
        if self._mode != "monitor":
            # Phase 2: SetAvatarMode пока NOT_IMPLEMENTED в Active.
            # Расширение (event-driven FSM-управление) — отдельная карточка.
            self._log.warning(
                f"SetAvatarMode: mode={self._mode} запрошен, но SetAvatarMode "
                f"не реализован в Phase 2 (AV-X = Acquire/ReleaseFloor only). "
                f"Отвечаю success=true/applied=false/reason={MONITOR_MODE_REASON}"
            )
        else:
            self._log.info(
                f"SetAvatarMode received (mode={self._mode}) — phase 1 monitor"
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

    # ── Phase 2 wire-up helpers (AV-X) ───────────────────────────────

    def _ensure_active_handlers(self) -> None:
        """Ленивая инициализация ``ModeManager`` + ``LockManager``.

        Создаются ОДИН раз при первом active-вызове. Повторные вызовы —
        no-op. Гарантирует, что monitor-режим не держит FSM/Lock
        (S12, ADR-0028 §4.5).

        Side-effects (Phase 2 dead-man, t_6fa213cc):
          - ``self._teleop_lock_pub`` — eager-создаётся здесь, чтобы
            ``_dead_man_tick`` мог publish ``False`` при trip-е без
            отдельной lazy-логики. Twist_mux lock-topic живёт
            *только* в active-режиме (monitor нода не вмешивается,
            S12).
        """
        if self._active_inited:
            return
        from rob_box_supervisor.core import LockManager, ModeManager

        self._mode_manager = ModeManager()
        self._lock_manager = LockManager()
        # Teleop-lock publisher — eager в active (см. docstring).
        if self._teleop_lock_pub is None:
            self._teleop_lock_pub = self.create_publisher(
                RosBool, TELEOP_LOCK_TOPIC, 10
            )
        self._active_inited = True
        self._log.info("Phase 2 active: ModeManager + LockManager initialized")

    @staticmethod
    def _parse_floor_request(
        request: Any, default_floor: str = "voice_floor"
    ) -> Tuple[str, str]:
        """Извлечь ``(client_id, floor)`` из ``Trigger.Request``.

        Конвенция: ``request.data`` = JSON ``{"client_id": "...", "floor": "..."}``.
        Если data пустой / не парсится / нет полей — возвращаем дефолты
        ``("unknown", default_floor)``. В rclpy ``Trigger.Request`` пустой,
        поэтому в проде ``data`` всегда None → defaults. Это сознательный
        trade-off (см. docstring модуля); миграция на ament_cmake + rosidl
        = Phase 2.1.
        """
        client_id = "unknown"
        floor = default_floor
        data = getattr(request, "data", None)
        if not data:
            return client_id, floor
        try:
            payload = json.loads(data)
        except (ValueError, TypeError):
            return client_id, floor
        if isinstance(payload, dict):
            client_id = str(payload.get("client_id") or client_id)
            floor = str(payload.get("floor") or floor)
        return client_id, floor

    def _acquire_floor_active(self, client_id: str, floor: str) -> Dict[str, Any]:
        """Phase 2 acquire: ``LockManager.acquire`` + ``ModeManager.transition``.

        Возвращает dict для ``_fill_active_response``:
          ``{"success": True, "applied": bool, "reason": str, "mode": str,
            "state": {<floors>}, "__state_bytes__": <msgpack>}``

        Бросает ``LockManager.ConflictError`` или ``ModeManager.ConflictError``
        при коллизии; обработчик-обёртка ``_on_acquire_floor`` ловит их и
        возвращает ``applied=False, reason=conflict``.
        """
        # Сначала пробуем LockManager.acquire — он бросит ConflictError
        # если floor уже занят другим client.
        try:
            self._lock_manager.acquire(client_id, floor)  # type: ignore[union-attr]
        except Exception as exc:  # noqa: BLE001
            return {
                "success": True,
                "applied": False,
                "reason": f"lock_conflict: {exc}",
                "mode": self._mode_manager.mode.value,  # type: ignore[union-attr]
            }

        # LockManager принял → переводим FSM.
        now_ms = self._now_ms()
        self._floor_acquired_at_ms[floor] = now_ms
        self._last_heartbeat_ms[(client_id, floor)] = now_ms

        try:
            event = self._pick_acquire_event(client_id=client_id, floor=floor)
            new_mode = self._mode_manager.transition(event, client_id=client_id)  # type: ignore[union-attr]
        except Exception as exc:  # noqa: BLE001 — FSM ConflictError → откат
            # Откатываем LockManager, чтобы state остался консистентным.
            try:
                self._lock_manager.release(client_id, floor)  # type: ignore[union-attr]
            except Exception:  # noqa: BLE001
                pass
            return {
                "success": True,
                "applied": False,
                "reason": f"fsm_conflict: {exc}",
                "mode": self._mode_manager.mode.value,  # type: ignore[union-attr]
            }

        # Side-effect: twist_mux lock для teleop_floor (ADR-0028 §4.2 S5).
        # voice_floor НЕ блокирует twist_mux — пропускаем.
        if floor == FLOOR_TELEOP:
            try:
                self._apply_teleop_lock(acquire=True)
            except Exception as exc:  # noqa: BLE001 — не валим acquire на side-effect
                self._log.warning(f"AcquireFloor: teleop_lock publish failed: {exc!r}")

        body: Dict[str, Any] = {
            "success": True,
            "applied": True,
            "reason": "ok",
            "mode": new_mode.value,
        }
        return body

    def _release_floor_active(self, client_id: str, floor: str) -> Dict[str, Any]:
        """Phase 2 release: ``LockManager.release`` + ``ModeManager.transition``.

        ``LockManager.release`` бросает ``PermissionError`` если
        client_id != holder; ``ModeManager.transition`` бросает
        ``ConflictError`` в редких race-сценариях. Оба отдаём клиенту
        как ``applied=False, reason=conflict``.
        """
        # LockManager.release (idempotent для already-released).
        try:
            self._lock_manager.release(client_id, floor)  # type: ignore[union-attr]
        except PermissionError as exc:
            return {
                "success": True,
                "applied": False,
                "reason": f"not_holder: {exc}",
                "mode": self._mode_manager.mode.value,  # type: ignore[union-attr]
            }
        # FSM: release-event в зависимости от client_id.
        event = self._pick_release_event(client_id=client_id)
        new_mode = self._mode_manager.transition(event, client_id=client_id)  # type: ignore[union-attr]

        # Side-effect: снять twist_mux lock для teleop_floor (ADR-0028 §4.2).
        # voice_floor не блокировал twist_mux — пропускаем.
        if floor == FLOOR_TELEOP:
            try:
                self._apply_teleop_lock(acquire=False)
            except Exception as exc:  # noqa: BLE001 — не валим release на side-effect
                self._log.warning(f"ReleaseFloor: teleop_lock publish failed: {exc!r}")

        # Чистим bookkeeping.
        self._floor_acquired_at_ms.pop(floor, None)
        self._last_heartbeat_ms.pop((client_id, floor), None)

        return {
            "success": True,
            "applied": True,
            "reason": "ok",
            "mode": new_mode.value,
        }

    def _pick_acquire_event(self, client_id: str, floor: str) -> str:
        """Выбрать FSM-событие для ``ModeManager.transition`` под (client, floor).

        Маппинг:
          - telegram, voice_floor, mode=off → EVENT_TELEGRAM_ACQUIRE_FLOOR
          - telegram, voice_floor, mode=avatar_present → EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR
          - quest, teleop_floor, mode=off → EVENT_QUEST_ACQUIRE_FLOOR
          - quest, teleop_floor, mode=telegram_active → EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY
          - quest, voice_floor, mode=off/telegram_active → EVENT_QUEST_ACQUIRE_FLOOR
          - telegram, teleop_floor, mode=off → EVENT_TELEGRAM_ACQUIRE_FLOOR
        """
        from rob_box_supervisor.core.fsm import (
            EVENT_QUEST_ACQUIRE_FLOOR,
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
            EVENT_TELEGRAM_ACQUIRE_FLOOR,
            EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
            Mode,
        )

        current = self._mode_manager.mode
        # type: ignore[union-attr]
        if client_id == "telegram":
            if floor == "voice_floor":
                if current == Mode.AVATAR_PRESENT:
                    return EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR
                return EVENT_TELEGRAM_ACQUIRE_FLOOR
            if floor == "teleop_floor":
                return EVENT_TELEGRAM_ACQUIRE_FLOOR
        if client_id == "quest":
            if floor == "teleop_floor":
                if current == Mode.TELEGRAM_ACTIVE:
                    return EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY
                return EVENT_QUEST_ACQUIRE_FLOOR
            if floor == "voice_floor":
                return EVENT_QUEST_ACQUIRE_FLOOR
        raise ValueError(
            f"unsupported (client_id={client_id!r}, floor={floor!r}) "
            f"in mode={current.value!r}"
        )

    @staticmethod
    def _pick_release_event(client_id: str) -> str:
        from rob_box_supervisor.core.fsm import (
            EVENT_QUEST_RELEASE,
            EVENT_TELEGRAM_RELEASE,
        )

        if client_id == "telegram":
            return EVENT_TELEGRAM_RELEASE
        if client_id == "quest":
            return EVENT_QUEST_RELEASE
        raise ValueError(f"unknown client_id={client_id!r} for release")

    def _now_ms(self) -> int:
        """Текущее время в миллисекундах (для ``FloorState.since_ms``)."""
        return int(time.time() * 1000)

    def _build_avatar_state(self) -> AvatarState:
        """Собрать ``AvatarState`` из текущего FSM + LockManager + aggregator.

        Контракт: ``floors{teleop, voice}`` отражают holder-ов LockManager
        (с учётом dead-man auto-release); ``mode`` — текущий FSM.mode.
        """
        # AvatarState/FloorState уже импортированы в __init__ (см. начало
        # метода) — используем их как глобальные имена модуля.
        teleop_holder = self._lock_manager.holder("teleop_floor")  # type: ignore[union-attr]
        voice_holder = self._lock_manager.holder("voice_floor")  # type: ignore[union-attr]
        now_ms = self._now_ms()

        teleop_state = (
            FloorState(
                client_id=teleop_holder,
                since_ms=self._floor_acquired_at_ms.get("teleop_floor", now_ms),
                last_heartbeat_ms=self._last_heartbeat_ms.get(
                    (teleop_holder, "teleop_floor"), now_ms
                ),
            )
            if teleop_holder
            else None
        )
        voice_state = (
            FloorState(
                client_id=voice_holder,
                since_ms=self._floor_acquired_at_ms.get("voice_floor", now_ms),
                last_heartbeat_ms=self._last_heartbeat_ms.get(
                    (voice_holder, "voice_floor"), now_ms
                ),
            )
            if voice_holder
            else None
        )
        return AvatarState(
            mode=self._mode_manager.mode.value,  # type: ignore[union-attr]
            teleop_floor=teleop_state,
            voice_floor=voice_state,
            last_event=None,
            since_ms=now_ms,
        )

    def _fill_active_response(self, response: Any, body: Dict[str, Any]) -> Any:
        """Заполнить Trigger.response из Phase 2 active-обработчика.

        Конвенция: ``Trigger.success`` = True (мы ответили), ``Trigger.message`` =
        JSON с полями ``applied``, ``reason``, ``mode``, ``state`` (msgpack
        round-trip, см. ``core/state.pack``).
        """
        # ``state`` сериализуем отдельно (msgpack) и кладём base64, чтобы
        # message оставался UTF-8 safe (Trigger.message — std_msgs/String).
        import base64

        state_bytes = body.pop("__state_bytes__", b"")
        if state_bytes:
            body["state_b64"] = base64.b64encode(state_bytes).decode("ascii")
        response.success = bool(body.get("success", True))
        response.message = json.dumps(body)
        return response

    def _fill_error_response(self, response: Any, reason: str) -> Any:
        """Ответ при internal error — клиент видит applied=False + reason."""
        response.success = True  # service call дошёл; applied=False — отказ.
        response.message = json.dumps({"applied": False, "reason": reason})
        return response

    # ── Phase 2 active dead-man watchdog (AV-6 Phase 2, t_6fa213cc) ──
    #
    # ADR-0028 §4.4 S10: «Клиент, держащий teleop_floor, шлёт heartbeat
    # ≥ 10 Гц; без heartbeat > DEAD_MAN_TIMEOUT_MS — снимаем floor».
    # Реализовано в трёх местах:
    #   1. ``_on_heartbeat_msg`` — обновляет ``last_heartbeat_ms`` через
    #      ``LockManager.heartbeat()`` для каждого (client_id, floor)
    #      в сообщении. Если клиент не держит этот floor — no-op
    #      (race с release; без шума).
    #   2. ``_dead_man_tick`` — периодический watchdog (10 Hz). Проходит
    #      по всем (client_id, floor) в ``LockManager`` и при expired
    #      снимает floor через ``EVENT_FORCE_OFF`` + трип счётчика.
    #   3. ``_apply_teleop_lock`` — публикует ``True``/``False`` в
    #      ``/teleop_lock`` при acquire/release teleop_floor (S5: нода —
    #      единственный владелец twist_mux-side-effect).
    def _on_heartbeat_msg(self, msg: RosString) -> None:
        """Обработать heartbeat от клиента.

        Конвенция payload (JSON): ``{"client_id": str, "ts_ms": int,
        "floor": "teleop_floor"|"voice_floor"}``. Невалидный JSON или
        отсутствующие поля → log + drop (heartbeat не должен валить
        ноду). В monitor-режиме callback пропускает работу (S12).
        """
        if self._mode != "active":
            return
        data = self._try_parse_json(msg.data)
        if not isinstance(data, dict):
            return
        client_id = data.get("client_id")
        floor = data.get("floor")
        if not isinstance(client_id, str) or not client_id:
            return
        if floor not in (FLOOR_TELEOP, FLOOR_VOICE):
            return
        if self._lock_manager is None:
            return  # monitor / pre-init
        now_ms = self._now_ms()
        # ``heartbeat`` бросает PermissionError, если client_id != holder
        # (или floor free). Это нормальная ситуация после release/deadman —
        # просто игнорируем (ADR-0028 §4.2).
        try:
            self._lock_manager.heartbeat(client_id, floor, now_ms=now_ms)
            # ``heartbeat`` обновляет только ``_last_heartbeat_ms`` в
            # LockManager; параллельный bookkeeping в supervisor_node
            # (_last_heartbeat_ms[(client_id, floor)]) тоже обновим,
            # чтобы публикация ``/avatar/state`` сразу отражала свежий ts.
            self._last_heartbeat_ms[(client_id, floor)] = now_ms
        except Exception:  # noqa: BLE001 — heartbeat не валит ноду
            pass

    def _apply_teleop_lock(self, acquire: bool) -> None:
        """Опубликовать ``acquire`` (True/False) в ``/teleop_lock``.

        Вызывается из ``_acquire_floor_active`` / ``_release_floor_active``
        только для ``floor="teleop_floor"`` (voice_floor не блокирует
        twist_mux). В monitor-режиме метод не вызывается (выше по
        call-stack стоит ``if self._mode != "active"`` early-return).

        Publisher создаётся лениво (при первом acquire teleop_floor),
        чтобы monitor-нода не держала лишний ROS 2 publisher, который
        мог бы конфликтовать с twist_mux.yaml Phase 1 (где lock-topic
        ``joystick_lock``, а не ``/teleop_lock``).
        """
        if self._teleop_lock_pub is None:
            self._teleop_lock_pub = self.create_publisher(
                RosBool, TELEOP_LOCK_TOPIC, 10
            )
        out = RosBool()
        out.data = bool(acquire)
        self._teleop_lock_pub.publish(out)
        # f-string: RcutilsLogger принимает ОДИН msg (issue #1644).
        self._log.info(f"teleop_lock publish: {out.data}")

    def _dead_man_tick(self) -> None:
        """Периодический watchdog: снять expired floor-ы.

        Алгоритм (ADR-0028 §4.4 S10):
          1. Для каждого floor-а вызвать ``LockManager.holder(floor)``.
          2. ``holder()`` возвращает ``None`` если floor свободен ИЛИ
             expired (LockManager сам авто-чистит expired при чтении).
          3. Если мы ЗНАЕМ (через ``_last_heartbeat_ms`` / FSM), что у
             floor был holder, а теперь ``None`` — это dead-man trip.
          4. При trip: ``EVENT_FORCE_OFF`` (escape hatch ``* → off``,
             ADR-0028 §4.1 §6 Q1), чистим bookkeeping,
             ``DeadManCounter.trip(client_id)``, публикуем
             ``/avatar/state`` со свежим mode=off.

        В monitor-режиме ``_lock_manager is None`` — early return.
        """
        if self._lock_manager is None or self._mode_manager is None:
            return
        if self._mode != "active":
            return  # S12: monitor не вмешивается

        tripped = False
        for floor in (FLOOR_TELEOP, FLOOR_VOICE):
            # ``holder(floor)`` автоматически вычистит expired floor.
            current_holder = self._lock_manager.holder(floor)
            if current_holder is not None:
                continue
            # Floor освобождён. Проверим, был ли это dead-man trip
            # (т.е. в нашем bookkeeping осталась запись с expired
            # last_heartbeat_ms).
            expired_clients = []
            now_ms = self._now_ms()
            for (cid, f), ts in list(self._last_heartbeat_ms.items()):
                if f != floor:
                    continue
                if (now_ms - ts) <= DEAD_MAN_TIMEOUT_MS:
                    continue  # Не expired — просто release до тика.
                expired_clients.append((cid, ts))
            if not expired_clients:
                continue  # Это был явный release, не deadman.
            # Dead-man trip!
            tripped = True
            # Берём первого expired клиента для счётчика (обычно один).
            # Если их несколько — логируем, но FSM-event один (escape hatch).
            for cid, ts in expired_clients:
                self._dead_man.trip(cid)
                # f-string: RcutilsLogger принимает ОДИН msg (issue #1644).
                self._log.warning(
                    f"dead_man trip: client={cid} floor={floor} "
                    f"last_hb_ms={ts} now_ms={now_ms}"
                )
            # FSM: ``* → off`` через escape hatch. Очищаем bookkeeping,
            # держим state согласованным.
            try:
                self._mode_manager.transition(EVENT_FORCE_OFF)
            except Exception as exc:  # noqa: BLE001
                # EVENT_FORCE_OFF не должен бросать (escape hatch),
                # но мы защищаемся от теоретических регрессий в FSM.
                self._log.warning(f"dead_man: FSM force_off failed: {exc!r}")
            # Чистим bookkeeping floor-а.
            self._floor_acquired_at_ms.pop(floor, None)
            for cid, _ts in expired_clients:
                self._last_heartbeat_ms.pop((cid, floor), None)
            # Twist_mux lock: если был teleop-floor, снимаем lock.
            if floor == FLOOR_TELEOP:
                self._apply_teleop_lock(acquire=False)

        if tripped:
            # Публикуем свежий /avatar/state (mode=off, floors=None).
            try:
                self._publish_avatar_state()
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"dead_man: publish_avatar_state failed: {exc!r}")


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
