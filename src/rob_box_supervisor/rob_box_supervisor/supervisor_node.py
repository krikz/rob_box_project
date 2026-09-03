"""AvatarSupervisor — ROS 2 нода-координатор аватара (Phase 1 monitor, AV-6).

Дизайн:
- Параметр ``mode`` (default ``"monitor"``). В ``monitor``-режиме нода
  публикует ``/avatar/state`` из агрегатора и **отвечает** на сервисы
  ``AcquireFloor`` / ``ReleaseFloor`` / ``SetAvatarMode`` типизированным
  ответом ``success=true, applied=false, reason=supervisor_in_monitor_mode``,
  **не** меняя ``twist_mux`` inputs и ``dialogue_node`` параметры
  (ADR-0028 §4.5). Это минимизирует blast radius: нода задеплоена и
  наблюдает, реальное влияние — после явного ``mode:=active``
  (после AV-7+AV-8+AV-10).
- Phase 2 (``active``-режим) появится в отдельных карточках — здесь
  параметр читается, проверяется, и при попытке ``active`` нода
  логирует ``NOT_IMPLEMENTED`` и фактически остаётся в monitor.

Типизированный wire-контракт (ADR-0028 §4.3):
Сервисы ``acquire_floor`` / ``release_floor`` / ``set_avatar_mode``
объявлены на типизированных IDL из пакета ``rob_box_supervisor_msgs``
(см. docs/adr/0028-avatar-supervisor.md §4.3 и карточку AV-12,
issue #1904). Поля запроса и ответа больше НЕ проходят через JSON в
``std_msgs/String.message`` — клиент шлёт ``{client_id, floor}`` /
``{client_id, mode}`` напрямую, сервер отвечает
``{granted, held_by, applied, reason}`` / ``{success, mode, reason,
applied}`` как typed fields.

Раньше (W3-2/W3-4, issue #968 wave2) для acquire/release/mode
использовался ``std_srvs/Trigger`` и ``client_id``/``floor``/``event``
сморгались через ``request.data`` как JSON-строка —
`std_srvs/Trigger.Request` в реальном ROS 2 не имеет полей (пусто
перед ``---`` в .srv), так что JSON-вариант работал только внутри
процесса супервизора (поэтому весь W3-2 контракт был ЧЕСТНО
задокументированным техдолгом, см. ADR-0028 §4.2). Эта карточка
(AV-12) окончательно выводит три сервиса на типизированный wire —
теперь они работают и через процессы / по сети.

W3-2 (issue #968 wave2, провалы G2/G3) — ``acquire_floor``/
``release_floor`` безусловные заглушки заменены: в ``active``-режиме
реально захватывают/отпускают floor через
:class:`~rob_box_supervisor.core.locks.LockManager` (dead-man 500 мс,
ADR-0028 §6 Q4). Контракт теперь — типизированный IDL (AV-12).

W3-4 (issue #968 wave2) — ``set_avatar_mode`` тоже больше не
заглушка: в ``active``-режиме реально прогоняет FSM-событие через
:class:`~rob_box_supervisor.core.fsm.ModeManager` (переходы —
ADR-0028 §4.1) и отвечает ``applied=true`` + текущим avatar-режимом
в типизированном поле ``mode``. ``ModeManager`` остаётся событийным
автоматом (ADR-0028 §4.1), поэтому ``request.mode`` — это имя
FSM-события (``core.fsm.EVENT_*``), а не целевой режим напрямую.
При уходе из активного avatar-режима (``*_release``/``force_off``/
``both_release``) floor-ы, которые ``ModeManager`` перестал считать
занятыми, зеркально освобождаются и в ``LockManager`` — иначе
остаётся висячий holder, до которого никто больше не может достучаться
через ``ReleaseFloor`` (см. :py:meth:`_set_avatar_mode_logic`).

Источники истины:
- ADR-0028 §4.3 (ROS 2 API)
- ADR-0028 §4.5 (monitor-режим)
- ADR-0028 §6 Q4 (``dead_man_trips_total{client_id}``)
- docs/architecture/SYSTEM_OVERVIEW.md §5.4
- docs/architecture/meta-quest-api.md §3, §5.1 (wire-контракт клиентов)
- docs/plans/2026-08-24-avatar-decomposition.md §AV-6

Zenoh: ``ZENOH_SESSION_CONFIG_URI`` env-переменная подхватывается
rmw_zenoh_cpp автоматически — нам читать её вручную не нужно, только
залогировать на старте для диагностики (см. :py:meth:`__init__`).

Импорт IDL — ленивый/через try-import (ADR-0028 §4.5 fail-safe):
``rob_box_supervisor_msgs`` может отсутствовать (например, mock-rclpy
CI-окружение без собранного IDL). В этом случае нода логирует WARN на
старте и остаётся в monitor-режиме с честным fallback: сервисы
объявляются как ``std_srvs/Trigger``, ответы кладутся в ``message``
JSON-ом (прежний W3-2 контракт). Активный режим в fallback отключён,
чтобы не оставлять лазейку к LockManager через невалидный wire.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

from rob_box_supervisor.core.state import (
    AvatarEvent,
    AvatarState,
    StateTransportError,
    StateVersionError,
    encode_for_ros_string,
)


# AV-14 (issue #1906) — ``/avatar/state`` wire format lives in
# :mod:`rob_box_supervisor.core.state`. The supervisor here ONLY calls
# :func:`encode_for_ros_string`; no local msgpack, no JSON fallback.
# See ``docs/plans/2026-09-02-avatar-epic-state-audit.md`` §1.2 G3 for the
# silent-fail bug this prevents (msgpack publisher + JSON consumer = Telegram
# never saw any state). The codec helper also handles the
# ``forward-compat /msgpack absent`` defensive case via
# :func:`is_ros_string_safe`, so this module does not need its own try/except
# + JSON fallback — the bug we are closing lived precisely in that branch.


# Default monitor-mode reason, который нода возвращает клиентам в Phase 1.
# Зафиксирован строкой, чтобы логи и e2e-тесты могли матчить без магических
# литералов по всему коду (ADR-0028 §4.5).
MONITOR_MODE_REASON = "supervisor_in_monitor_mode"

# Reason-коды для типизированных ответов. Экспортируются константами, чтобы
# клиенты/тесты могли матчить без магических литералов (S14 ADR-0028).
REASON_OK = "ok"
REASON_GRANTED = "granted"
REASON_RELEASED = "released"
REASON_HELD_BY_OTHER = "held_by_other"
REASON_BAD_REQUEST = "bad_request"
REASON_CONFLICT = "conflict"
REASON_PERMISSION_DENIED = "permission_denied"
REASON_INVALID_EVENT = "invalid_event"
REASON_INVALID_REQUEST = "invalid_request"
REASON_APPLIED = "applied"
# По ADR-0028 §4.5 — в monitor-режиме сервис НЕ вмешивается, но и НЕ
# отказывает клиенту. Стандартное behaviour: applied=false, reason=MONITOR_MODE_REASON.
REASON_MONITOR = MONITOR_MODE_REASON

# ADR-0027 §3.4 — валидные значения ``voice_input_mode`` на dialogue_node.
# Супервизор — единственная точка, которая имеет право их менять (ADR-0028 S5).
# "off" (W3-1, §3.5 docs/design/dialogue-mode-spec-2026-08-28.md) —
# «диалог off»: блокирует диалоговую ноду ТОЛЬКО для обычных людей у
# ReSpeaker-микрофона; вход оператора (Telegram/Quest) продолжает работать.
VOICE_INPUT_MODES: tuple[str, ...] = (
    "respeaker",
    "quest_passthrough",
    "quest_ttts",
    "quest_stt",
    "quest_llm_formalize",
    "off",
)

# Phase 1 транспорт запроса смены режима голоса. Phase 2 заменит на
# ``SetVoiceMode``-сервис с кастомным IDL (ADR-0028 §4.3) — здесь топик
# достаточен, чтобы не плодить rosidl-интерфейсы ради monitor-фазы.
SET_VOICE_MODE_TOPIC: str = "/avatar/set_voice_mode"


# FSM-событийные имена (ADR-0028 §4.1 mermaid). Документируем здесь полный
# реестр, чтобы клиентские сервисы/тесты могли импортировать тот же набор,
# что и core.fsm — и не расходились имена.
EVENT_TELEGRAM_ACQUIRE_FLOOR = "telegram_acquire_floor"
EVENT_QUEST_ACQUIRE_FLOOR = "quest_acquire_floor"
EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY = "quest_acquire_floor_teleop_only"
EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR = "telegram_acquire_voice_floor"
EVENT_QUEST_ACQUIRE_FULL_FLOOR = "quest_acquire_full_floor"
EVENT_TELEGRAM_RELEASE = "telegram_release"
EVENT_QUEST_RELEASE = "quest_release"
EVENT_QUEST_RELEASE_TELEOP = "quest_release_teleop"
EVENT_TELEGRAM_RELEASE_VOICE = "telegram_release_voice"
EVENT_BOTH_RELEASE = "both_release"
EVENT_FORCE_OFF = "force_off"


# Wire-уровневые имена floor-ов (поле ``floor`` в AcquireFloor/ReleaseFloor
# request/response и в ``meta-quest-api.md`` §3 / §5.1): короткие, для
# провода и клиентских фрейм-типов 0x31/0x32.
WIRE_FLOOR_TELEOP = "teleop"
WIRE_FLOOR_VOICE = "voice"

# Доменные имена floor-ов внутри supervisor (LockManager — источник истины,
# ADR-0028 §4.2): "teleop_floor"/"voice_floor". Сюда маппим wire-имена
# через ``_wire_to_lock_floor``.
LOCK_FLOOR_TELEOP = "teleop_floor"
LOCK_FLOOR_VOICE = "voice_floor"


def _wire_to_lock_floor(wire: Optional[str]) -> Optional[str]:
    """Маппинг wire-floor (``teleop``/``voice``) → LockManager-floor.

    Нужен потому, что клиентский API (см. ``meta-quest-api.md`` §3) и
    IDL-пакет ``rob_box_supervisor_msgs`` используют короткие wire-имена,
    а LockManager (ADR-0028 §4.2) держит floor-ы под именами с суффиксом.
    Возвращает ``None`` если значение wire-floor не распознано —
    вызывающий код классифицирует как ``bad_request``.
    """
    if wire == WIRE_FLOOR_TELEOP:
        return LOCK_FLOOR_TELEOP
    if wire == WIRE_FLOOR_VOICE:
        return LOCK_FLOOR_VOICE
    return None


def _lock_to_wire_floor(lock: Optional[str]) -> str:
    """Маппинг LockManager-floor → wire-floor для response-полей ``held_by``.

    Если ``lock`` — None (нет holder) или неожиданное значение — возвращаем
    пустую строку (``held_by=""`` в response).
    """
    if lock == LOCK_FLOOR_TELEOP:
        return WIRE_FLOOR_TELEOP
    if lock == LOCK_FLOOR_VOICE:
        return WIRE_FLOOR_VOICE
    return ""


def _try_load_supervisor_msgs() -> dict:
    """Ленивая загрузка IDL-типов из ``rob_box_supervisor_msgs``.

    Делается на старте ноды (а не на module-import), потому что в CI с
    mock-rclpy этого пакета нет, и статический импорт уронит весь test
    suite. Паттерн взят из ``quest_node.py`` (``vesc_msgs`` try-import).

    Возвращает dict с ключами ``Acq``/``AcqReq``/``AcqResp``/``Rel``/``RelReq``/
    ``RelResp``/``Set``/``SetReq``/``SetResp``/``TeleopHeartbeat``/``FloorState``/
    ``AvatarState`` (Python-классы из сгенерированных
    ``rob_box_supervisor_msgs.srv`` / ``.msg``).

    ``Acq``/``Rel``/``Set`` — ПОЛНЫЕ srv-классы (``AcquireFloor`` /
    ``ReleaseFloor`` / ``SetAvatarMode``) с nested ``.Request``/``.Response``.
    Они используются в :py:meth:`AvatarSupervisor._register_services` для
    ``create_service()`` — rclpy требует полный srv-класс, иначе бросает
    ``RuntimeError: The service type provided is not valid``
    (см. issue #1904, раунды e2e 337..341, карточка t_979f0cb2). До этого
    фикса нода передавала ``AcquireFloor.Request`` и падала в
    ``__init__`` ещё до публикации сервисов → docker restart-policy →
    crash-loop, который блокировал все пять подряд e2e-раундов.

    ``*Req``/``*Resp`` — вложенные ``.Request``/``.Response``-классы.
    Нужны для построения объектов запросов/ответов из логики ноды
    (LockManager/StateAggregator — см. callback-и сервисов и
    ``_fill_*_response``). Сами по себе они НЕ валидный аргумент
    ``create_service()`` (см. выше).

    Если модуль недоступен — каждый ключ равен ``None``; вызывающий код
    обязан это обработать (см. ``__init__`` и сервисные callback-и).
    """
    types_map: dict = {  # noqa: WPS234 — инициализация «пустого» dict'a перед try
        "Acq": None,
        "AcqReq": None,
        "AcqResp": None,
        "Rel": None,
        "RelReq": None,
        "RelResp": None,
        "Set": None,
        "SetReq": None,
        "SetResp": None,
        "AvatarState": None,
        "FloorState": None,
        "TeleopHeartbeat": None,
    }
    try:
        from rob_box_supervisor_msgs.srv import (  # noqa: WPS433
            AcquireFloor,
            ReleaseFloor,
            SetAvatarMode,
        )

        # Полные srv-классы — ОБЯЗАТЕЛЬНО для create_service() (см. docstring).
        types_map["Acq"] = AcquireFloor
        types_map["Rel"] = ReleaseFloor
        types_map["Set"] = SetAvatarMode
        # Вложенные Request/Response — для построения объектов в callback-ах.
        types_map["AcqReq"] = AcquireFloor.Request
        types_map["AcqResp"] = AcquireFloor.Response
        types_map["RelReq"] = ReleaseFloor.Request
        types_map["RelResp"] = ReleaseFloor.Response
        types_map["SetReq"] = SetAvatarMode.Request
        types_map["SetResp"] = SetAvatarMode.Response
    except ImportError:
        return types_map
    try:
        from rob_box_supervisor_msgs.msg import (  # noqa: WPS433
            AvatarStateMsg as AvatarState,
            FloorState as FloorStateMsg,
            TeleopHeartbeat as TeleopHeartbeatMsg,
        )

        types_map["AvatarState"] = AvatarState
        types_map["FloorState"] = FloorStateMsg
        types_map["TeleopHeartbeat"] = TeleopHeartbeatMsg
    except ImportError:  # pragma: no cover — srv и msg собираются вместе
        pass
    return types_map


class AvatarSupervisor(Node):
    """ROS 2 нода ``avatar_supervisor`` (Vision Pi, Phase 1 monitor)."""

    # ── topic / service constants (ADR-0028 §4.3) ─────────────────────
    AVATAR_STATE_TOPIC = "/avatar/state"

    ODOM_TOPIC = "/odom"
    DEVICE_SNAPSHOT_TOPIC = "/device/snapshot"
    VOICE_DIALOGUE_STATE_TOPIC = "/voice/dialogue/state"  # НЕ /voice/state (ADR-0027 #2)

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
        from rob_box_supervisor.core import (
            DeadManCounter,
            Floor,
            LockManager,
            ModeManager,
            StateAggregator,
        )

        self._aggregator = StateAggregator()
        self._dead_man = DeadManCounter()
        # LockManager — источник истины по voice_floor/teleop_floor (W3-2,
        # ADR-0028 §4.2) для сервисов AcquireFloor/ReleaseFloor.
        self._lock_manager = LockManager()
        # ModeManager — FSM avatar-режимов (off/telegram_active/
        # avatar_present/mixed, ADR-0028 §4.1), подключён в W3-4 под
        # SetAvatarMode. Его voice_held_by/teleop_held_by — ТОЛЬКО вход
        # для собственных решений о переходах (см. core/__init__.py
        # docstring), НЕ источник истины по floor-ам — им остаётся
        # LockManager. _set_avatar_mode_logic зеркалит releases между
        # ними best-effort (см. её docstring).
        self._mode_manager = ModeManager()
        # Снимок последних известных holder-ов — нужен только чтобы отличить
        # "floor освободился через dead-man" от "floor и так был свободен"
        # при периодической проверке в _publish_avatar_state (метрика
        # dead_man_trips_total, ADR-0028 §6 Q4).
        self._known_floor_holders: dict = {Floor.TELEOP: None, Floor.VOICE: None}

        # Publisher /avatar/state (latched, transient_local — ADR-0028 §4.3).
        # QoSProfile(depth=1, transient_local=True) эмулирует "latched".
        from rclpy.qos import DurabilityPolicy, QoSProfile

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(RosString, self.AVATAR_STATE_TOPIC, state_qos)

        # Subscriptions — на Phase 1 только регистрируем callback-и и
        # обновляем aggregator. Полный IDL / парсинг msg — Phase 2.
        self.create_subscription(RosString, self.ODOM_TOPIC, self._on_odom_msg, 10)
        self.create_subscription(RosString, self.DEVICE_SNAPSHOT_TOPIC, self._on_device_snapshot_msg, 10)
        self.create_subscription(RosString, self.VOICE_DIALOGUE_STATE_TOPIC, self._on_voice_state_msg, 10)
        # ADR-0028 S5 — супервизор единственный, кто меняет voice_input_mode
        # на dialogue_node. Phase 1 транспорт — топик (см. SET_VOICE_MODE_TOPIC).
        self.create_subscription(RosString, SET_VOICE_MODE_TOPIC, self._on_set_voice_mode, 10)
        # Параметр-клиент к dialogue_node создаётся лениво в active-режиме
        # (в monitor супервизор НЕ трогает чужие параметры — S12).
        self._dialogue_param_client = None

        # ── IDL-типы для типизированных сервисов (AV-12, ADR-0028 §4.3) ──
        # Пытаемся загрузить rob_box_supervisor_msgs. Если недоступен
        # (CI mock-rclpy, битая сборка), фолбэк на std_srvs/Trigger с
        # JSON-в-message — прежний W3-2 контракт, и нода остаётся в monitor.
        self._msgs_types = _try_load_supervisor_msgs()
        # Проверяем именно ПОЛНЫЕ srv-классы (Acq/Set), а не вложенные
        # Request/Response — create_service() требует полный srv-класс
        # (см. ``_try_load_supervisor_msgs`` docstring и карточку t_979f0cb2).
        # Если IDL загружен частично (например, только .msg, но не .srv),
        # typed-путь недоступен и мы уходим в std_srvs/Trigger fallback.
        self._use_typed_floor_services: bool = (
            self._msgs_types["Acq"] is not None and self._msgs_types["Set"] is not None
        )

        if not self._use_typed_floor_services:
            # Не fatal — это fail-safe (ADR-0028 §4.5). Нода остаётся
            # наблюдателем, в monitor-режиме, и сервисы объявляются на
            # std_srvs/Trigger со старым JSON-контрактом (W3-2/W3-4).
            self._log.warn(
                "rob_box_supervisor_msgs IDL недоступен — floor-сервисы на "
                "std_srvs/Trigger fallback, нода остаётся в monitor (ADR-0028 §4.5)."
            )

        self._register_services()

        # Периодическая публикация /avatar/state — 1 Hz достаточно для
        # monitor (Phase 2 увеличит частоту / сделает event-driven).
        self._timer = self.create_timer(1.0, self._publish_avatar_state)

        self._log_startup_diagnostics()

    # ── service registration (типизированный IDL или fallback) ────────
    def _register_services(self) -> None:
        """Объявить ``acquire_floor`` / ``release_floor`` / ``set_avatar_mode``.

        С типизированным IDL (``rob_box_supervisor_msgs``) — поля запроса/
        ответа становятся typed attributes; клиенты могут не сериализовать
        JSON и не парсить ``message``. Без IDL (CI / недосборка) — fallback
        на ``std_srvs/Trigger`` + JSON-в-``data``/``message``, как было в
        W3-2/W3-4 (документированный техдолг). Этот dual-mode — fail-safe по
        ADR-0028 §4.5, чтобы нода могла быть задеплоена и без пересборки
        workspace, оставаясь безопасным наблюдателем.
        """
        if self._use_typed_floor_services:
            # ВАЖНО: create_service() требует ПОЛНЫЙ srv-класс (с nested
            # .Request/.Response), а не отдельный .Request/.Response-класс.
            # Передача ``AcquireFloor.Request`` приводит к
            # ``RuntimeError: The service type provided is not valid``
            # ещё в ``__init__`` ноды → docker restart policy → crash-loop
            # (issue #1904, e2e-раунды 337..341, карточка t_979f0cb2).
            self._srv_acquire = self.create_service(
                self._msgs_types["Acq"],
                self.ACQUIRE_FLOOR_SERVICE,
                self._on_acquire_floor,
            )
            self._srv_release = self.create_service(
                self._msgs_types["Rel"],
                self.RELEASE_FLOOR_SERVICE,
                self._on_release_floor,
            )
            self._srv_set_mode = self.create_service(
                self._msgs_types["Set"],
                self.SET_AVATAR_MODE_SERVICE,
                self._on_set_avatar_mode,
            )
        else:
            from std_srvs.srv import Trigger

            self._srv_acquire = self.create_service(Trigger, self.ACQUIRE_FLOOR_SERVICE, self._on_acquire_floor_fb)
            self._srv_release = self.create_service(Trigger, self.RELEASE_FLOOR_SERVICE, self._on_release_floor_fb)
            self._srv_set_mode = self.create_service(Trigger, self.SET_AVATAR_MODE_SERVICE, self._on_set_avatar_mode_fb)

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
        # AV-14: msgpack is a hard dep (declared in package.xml). Its
        # availability is verified at import time by core.state; if the
        # codec raises here it means the environment is genuinely broken
        # (and we want a single loud failure, not a silent fallback to JSON).
        self._log.info(
            f"avatar_supervisor started: mode={self._mode}, "
            f"zenoh={zenoh}, typed_services={self._use_typed_floor_services}"
        )


    def _monitor_response(self) -> dict:
        """Стандартный ответ для всех сервисов в monitor-режиме.

        Возвращает dict — адаптер (``_fill_floor_response`` /
        ``_fill_monitor_response``) раскладывает его по полям response.
        Поля под типизированный IDL и под fallback (`std_srvs/Trigger` +
        JSON-в-`message`) одни и те же: ``success``, ``applied``,
        ``reason``.
        """
        return {
            "success": True,
            "applied": False,
            "reason": MONITOR_MODE_REASON,
        }

    def _check_dead_man_trips(self) -> None:
        """Периодически (1 Hz, из таймера) засечь dead-man авто-release floor-ов.

        ``LockManager.holder()`` сам лениво чистит истёкшие floor-ы
        (ADR-0028 §6 Q4, 500 мс), но не сообщает наружу, что именно
        произошло. Здесь сравниваем со снимком прошлого тика: если
        floor был занят client_id, а теперь свободен без явного
        ``ReleaseFloor`` (мы бы уже обнулили ``_known_floor_holders`` в
        :py:meth:`_release_floor_logic`) — это dead-man trip, инкрементируем
        метрику ``dead_man_trips_total`` через агрегатор. Разрешение —
        до 1с (период таймера), это метрика, а не enforcement (сам floor
        снимается лениво и мгновенно при следующем acquire/holder()).
        """
        from rob_box_supervisor.core import Floor  # noqa: PLC0415

        for floor in (Floor.TELEOP, Floor.VOICE):
            prev = self._known_floor_holders.get(floor)
            current = self._lock_manager.holder(floor)
            if prev is not None and current is None:
                new_count = self._aggregator.record_dead_man_trip(prev)
                self._log.warning(f"dead_man_trip: client_id={prev} floor={floor} count={new_count}")
            self._known_floor_holders[floor] = current

    def _build_published_avatar_state(self) -> AvatarState:
        """Build the wire-format :class:`AvatarState` from the canonical sources.

        Sources of truth (per ADR-0028 §4.2, see audit §1.2 G3):
          - avatar ``mode`` ← :py:attr:`ModeManager.mode`
          - floors ownership ← :class:`LockManager` (dead-man aware)
          - ``last_event`` ← last dead-man trip recorded on the local
            :class:`StateAggregator` (Phase 1 metric) — None if no trip yet.

        This deliberately does NOT pull ``pose_xy`` / ``battery_pct`` /
        ``voice_state`` from :class:`StateAggregator`: those inputs are
        Phase 1 telemetry (and remain in the aggregator for the local
        metric path), but they do not belong in the ``/avatar/state``
        schema (ADR-0028 §4.3). Including them would either force a
        schema change (forbidden by AV-14 acceptance criterion #1:
        "не менять схему AvatarState") or hand-roll a second codec
        forbidden by criterion #2. Future consumers that need pose/battery
        should subscribe to ``/odom`` / ``/device/snapshot`` directly.
        """
        from rob_box_supervisor.core import (  # noqa: PLC0415
            Floor as FloorConst,
            FloorState,
        )

        now_ms = int(time.time() * 1000)

        teleop_holder = self._lock_manager.holder(FloorConst.TELEOP)
        voice_holder = self._lock_manager.holder(FloorConst.VOICE)
        teleop_floor = (
            FloorState(
                client_id=str(teleop_holder),
                since_ms=now_ms,
                last_heartbeat_ms=now_ms,
            )
            if teleop_holder is not None
            else None
        )
        voice_floor = (
            FloorState(
                client_id=str(voice_holder),
                since_ms=now_ms,
                last_heartbeat_ms=now_ms,
            )
            if voice_holder is not None
            else None
        )

        last_event = self._aggregator.last_event_as_avatar_event(now_ms)

        return AvatarState(
            mode=str(self._mode_manager.mode.value),
            teleop_floor=teleop_floor,
            voice_floor=voice_floor,
            last_event=last_event,
            since_ms=now_ms,
        )

    def _publish_avatar_state(self) -> None:
        """Timer-callback: публикует свежий snapshot в /avatar/state.

        AV-14: единственный кодек — :func:`encode_for_ros_string` из
        :mod:`core.state`. Никакого локального ``msgpack.packb``, никакого
        JSON-fallback'a — последний был именно тем путём, по которому
        издатель (msgpack-as-latin-1) и потребитель (``json.loads`` в
        ``supervisor_client``) разошлись. Если codec raise (битый state
        или отсутствующий msgpack) — пропускаем тик и шумим
        rate-limited WARN, чтобы не молча проглатывать (issue #1906).
        """
        self._check_dead_man_trips()
        try:
            state = self._build_published_avatar_state()
            payload_str = encode_for_ros_string(state)
        except (StateTransportError, StateVersionError) as exc:
            # Rate-limit: один WARN на тик максимум, чтобы не засорять лог
            # при циклической ошибке (publisher 1 Hz, log-flooding = bad).
            self._log.warning(f"avatar_supervisor: /avatar/state publish skipped: {exc}")
            return
        except Exception as exc:  # noqa: BLE001 — не валить таймер
            self._log.warning(f"avatar_supervisor: unexpected encode failure: " f"{type(exc).__name__}: {exc}")
            return

        msg = RosString()
        msg.data = payload_str
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

            self._dialogue_param_client = self.create_client(SetParameters, "/dialogue_node/set_parameters")
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
                    self._log.warning("SetVoiceMode: dialogue_node rejected parameter set")
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"SetVoiceMode: parameter set failed: {exc}")

        future.add_done_callback(_done)

    # ── typed service callbacks (ADR-0028 §4.3, AV-12) ────────────────
    @staticmethod
    def _extract_floor_request(request: Any) -> tuple[Optional[str], Optional[str]]:
        """Достать ``client_id``/``floor`` из типизированного запроса.

        AV-12: ``AcquireFloor.Request`` / ``ReleaseFloor.Request`` из
        ``rob_box_supervisor_msgs`` имеют typed-поля ``client_id: string``
        и ``floor: string`` (см. ``src/rob_box_supervisor_msgs/srv/``).
        Прошлый JSON-в-`data` fallback УБРАН (карточка AV-12, W3-2/R13):
        именно он маскировал иллюзию «всё работает», хотя
        ``std_srvs/Trigger.Request`` в реальном ROS 2 не имел полей вообще
        (ADR-0028 §4.2). Теперь контракт честный — типизированные поля
        запроса, никакого JSON, никакого ``request.data``.

        Возвращает ``(client_id, floor)``: ``(None, None)`` если поля
        пустые/не заданы, и caller дальше классифицирует как bad_request.
        """
        client_id = getattr(request, "client_id", None)
        floor = getattr(request, "floor", None)
        if not client_id:
            return None, None
        return str(client_id), (str(floor) if floor else None)

    def _acquire_floor_logic(self, client_id: Optional[str], floor: Optional[str]) -> dict:
        """Чистая логика ``AcquireFloor`` (тестируется без rclpy).

        В ``monitor`` — как раньше: ``applied=false``, floor не трогаем
        (S12). В ``active`` — реально дёргаем :class:`LockManager`.

        Возвращает dict, который адаптер ``_fill_floor_response`` /
        ``_fill_acquire_floor_response`` разливает по полям response.

        Поля dict-а (типизированный IDL / Trigger fallback одинаковы):

        - ``success``: bool — для Trigger.response.success
        - ``granted``: bool — флаг, дошёл ли запрос до LockManager и
          выдан ли этот floor этому client_id
        - ``held_by``: str — client_id текущего держателя (для клиента,
          чтобы отобразить «Floor held by X»); пустая строка если floor
          свободен
        - ``applied``: bool — supervisor НЕ вмешивался (false в monitor)
        - ``reason``: str — машино-читаемая причина (``granted`` /
          ``held_by_other`` / ``bad_request`` / ``conflict`` /
          ``monitor``); пустая строка при ошибках, которые считаются
          невозможными
        """
        if self._mode != "active":
            return {
                "success": True,
                "granted": False,
                "held_by": "",
                "applied": False,
                "reason": MONITOR_MODE_REASON,
            }
        from rob_box_supervisor.core import LockConflictError  # noqa: PLC0415

        if not client_id:
            return {
                "success": True,
                "granted": False,
                "held_by": "",
                "applied": False,
                "reason": REASON_BAD_REQUEST,
            }
        lock_floor = _wire_to_lock_floor(floor)
        if lock_floor is None:
            return {
                "success": True,
                "granted": False,
                "held_by": "",
                "applied": False,
                "reason": REASON_BAD_REQUEST,
            }

        # LockManager.acquire сам решает, что делать с уже-держащим-другим.
        # До вызова захватим текущего держателя (если есть) для ответа.
        current_holder_lock = self._lock_manager.holder(lock_floor)
        if current_holder_lock and current_holder_lock != client_id:
            return {
                "success": True,
                "granted": False,
                "held_by": str(current_holder_lock),
                "applied": True,
                "reason": REASON_HELD_BY_OTHER,
            }

        try:
            self._lock_manager.acquire(client_id, lock_floor)
        except LockConflictError as exc:
            return {
                "success": True,
                "granted": False,
                "held_by": str(exc.held_by),
                "applied": True,
                "reason": REASON_HELD_BY_OTHER,
            }
        self._known_floor_holders[lock_floor] = client_id
        return {
            "success": True,
            "granted": True,
            "held_by": client_id,
            "applied": True,
            "reason": REASON_GRANTED,
        }

    def _release_floor_logic(self, client_id: Optional[str], floor: Optional[str]) -> dict:
        """Чистая логика ``ReleaseFloor`` (тестируется без rclpy).

        Возвращает dict для адаптера ``_fill_release_floor_response``.

        Поля (типизированный IDL / Trigger fallback одинаковы):

        - ``success``: bool
        - ``applied``: bool (false в monitor / когда запрос отвергнут)
        - ``reason``: str (``released`` / ``permission_denied`` /
          ``bad_request`` / ``monitor``)
        """
        if self._mode != "active":
            return {
                "success": True,
                "applied": False,
                "reason": MONITOR_MODE_REASON,
            }
        if not client_id:
            return {
                "success": True,
                "applied": False,
                "reason": REASON_BAD_REQUEST,
            }
        lock_floor = _wire_to_lock_floor(floor)
        if lock_floor is None:
            return {
                "success": True,
                "applied": False,
                "reason": REASON_BAD_REQUEST,
            }

        try:
            self._lock_manager.release(client_id, lock_floor)
        except PermissionError:
            return {
                "success": True,
                "applied": False,
                "reason": REASON_PERMISSION_DENIED,
            }
        if self._known_floor_holders.get(lock_floor) == client_id:
            self._known_floor_holders[lock_floor] = None
        return {
            "success": True,
            "applied": True,
            "reason": REASON_RELEASED,
        }

    def _on_acquire_floor(self, request: Any, response: Any) -> Any:
        """``AcquireFloor`` (типизированный IDL, ADR-0028 §4.3, AV-12)."""
        client_id, floor = self._extract_floor_request(request)
        body = self._acquire_floor_logic(client_id, floor)
        self._log.info(
            f"AcquireFloor: client_id={client_id} floor={floor} mode={self._mode} "
            f"granted={body['granted']} held_by={body['held_by']!r} reason={body['reason']}"
        )
        return self._fill_acquire_floor_response(response, body)

    def _on_release_floor(self, request: Any, response: Any) -> Any:
        """``ReleaseFloor`` (типизированный IDL, ADR-0028 §4.3, AV-12)."""
        client_id, floor = self._extract_floor_request(request)
        body = self._release_floor_logic(client_id, floor)
        self._log.info(
            f"ReleaseFloor: client_id={client_id} floor={floor} mode={self._mode} "
            f"applied={body['applied']} reason={body['reason']}"
        )
        return self._fill_release_floor_response(response, body)

    @staticmethod
    def _extract_avatar_mode_request(request: Any) -> tuple[Optional[str], Optional[str]]:
        """Достать ``mode``/``client_id`` из типизированного SetAvatarMode-запроса.

        AV-12: ``SetAvatarMode.Request`` из ``rob_box_supervisor_msgs``
        имеет typed-поля ``client_id: string`` и ``mode: string``.
        ``mode`` — имя FSM-события (``core.fsm.EVENT_*`` из ADR-0028 §4.1
        mermaid), а не целевой avatar-режим напрямую — ``ModeManager``
        это событийный автомат, не setter состояния. Прошлый JSON-в-
        ``data`` fallback УБРАН (карточка AV-12, W3-2/R13) по тем же
        причинам, что и для floor-ов.

        Возвращает ``(event, client_id)``.
        """
        event = getattr(request, "mode", None)
        client_id = getattr(request, "client_id", None)
        if not event:
            return None, (str(client_id) if client_id else None)
        return str(event), (str(client_id) if client_id else None)

    def _set_avatar_mode_logic(self, event: Optional[str], client_id: Optional[str]) -> dict:
        """Чистая логика ``SetAvatarMode`` через ModeManager (W3-4, тестируется без rclpy).

        В ``monitor`` — как раньше: ``applied=false``, avatar-режим не
        трогаем (ADR-0028 §4.5/S12). В ``active`` — реально прогоняем
        событие через :class:`~rob_box_supervisor.core.fsm.ModeManager`;
        валидация переходов (``ConflictError``/``ValueError`` для
        неизвестного события) уже внутри самого ModeManager, здесь только
        маппинг в ``success``/``applied``/``reason``.

        Floor-синхронизация (решение W3-4, продолжение W3-2/ADR-0028 §4.2):
        ``ModeManager`` хранит ``voice_held_by``/``teleop_held_by`` ТОЛЬКО
        как вход для собственных решений о переходах — источник истины по
        floor-ам для клиентов остаётся :class:`LockManager`. Но если
        переход СНИМАЕТ holder-а в ``ModeManager`` (уход из активного
        avatar-режима — ``*_release``/``force_off``/``both_release``), а
        тот же floor всё ещё числится за тем же ``client_id`` в
        ``LockManager`` — оставлять его висеть нельзя: FSM уже решила, что
        клиент вышел, и достучаться до floor-а через ``ReleaseFloor``
        больше некому. Поэтому здесь же best-effort зеркально освобождаем
        такие floor-ы и в ``LockManager`` (idempotent — no-op, если там и
        так уже свободно; не валит переход, если floor неожиданно занят
        другим client_id — состояния разошлись, это отдельный инцидент).

        Возвращает dict для адаптера ``_fill_set_avatar_mode_response``:

        - ``success``: bool
        - ``applied``: bool (false в monitor / при отказе)
        - ``mode``: str — текущий avatar-режим после запроса (для
          клиента, который не подписан на ``/avatar/state``)
        - ``reason``: str — машино-читаемая причина (``applied`` /
          ``conflict`` / ``invalid_event`` / ``bad_request`` /
          ``monitor``)
        """
        if self._mode != "active":
            return {
                "success": True,
                "applied": False,
                "mode": self._mode_manager.mode.value,
                "reason": MONITOR_MODE_REASON,
            }
        if not event:
            return {
                "success": True,
                "applied": False,
                "mode": self._mode_manager.mode.value,
                "reason": REASON_BAD_REQUEST,
            }

        from rob_box_supervisor.core import Floor, FSMConflictError  # noqa: PLC0415

        prev_voice = self._mode_manager.voice_held_by()
        prev_teleop = self._mode_manager.teleop_held_by()

        try:
            new_mode = self._mode_manager.transition(event, client_id)
        except ValueError:
            return {
                "success": True,
                "applied": False,
                "mode": self._mode_manager.mode.value,
                "reason": REASON_INVALID_EVENT,
            }
        except FSMConflictError:
            return {
                "success": True,
                "applied": False,
                "mode": self._mode_manager.mode.value,
                "reason": REASON_CONFLICT,
            }

        if prev_voice is not None and self._mode_manager.voice_held_by() is None:
            self._release_lock_manager_floor(prev_voice, Floor.VOICE)
        if prev_teleop is not None and self._mode_manager.teleop_held_by() is None:
            self._release_lock_manager_floor(prev_teleop, Floor.TELEOP)

        return {
            "success": True,
            "applied": True,
            "mode": new_mode.value,
            "reason": REASON_APPLIED,
        }

    def _release_lock_manager_floor(self, client_id: str, floor: str) -> None:
        """Зеркально отпустить ``floor`` в ``LockManager`` вслед за ModeManager (W3-4).

        ``PermissionError`` (floor в LockManager уже держит другой
        client_id — состояния разошлись) не валит переход режима:
        ModeManager уже принял решение, floor-синхронизация — best-effort
        диагностика, а не транзакция.
        """
        try:
            self._lock_manager.release(client_id, floor)
        except PermissionError as exc:
            self._log.warning(f"SetAvatarMode: floor sync release failed: {exc}")
        if self._known_floor_holders.get(floor) == client_id:
            self._known_floor_holders[floor] = None

    def _on_set_avatar_mode(self, request: Any, response: Any) -> Any:
        """``SetAvatarMode`` (типизированный IDL, ADR-0028 §4.3, AV-12)."""
        event, client_id = self._extract_avatar_mode_request(request)
        body = self._set_avatar_mode_logic(event, client_id)
        self._log.info(
            f"SetAvatarMode: event={event} client_id={client_id} mode={self._mode} "
            f"applied={body['applied']} avatar_mode={body['mode']} reason={body['reason']}"
        )
        return self._fill_set_avatar_mode_response(response, body)

    # ── typed response adapters (типизированный IDL) ─────────────────
    def _fill_acquire_floor_response(self, response: Any, body: dict) -> Any:
        """Заполнить типизированный ответ ``AcquireFloorResponse``.

        Поля ``success`` (bool), ``granted`` (bool), ``held_by`` (string),
        ``reason`` (string), ``applied`` (bool) копируются как есть.
        """
        response.success = bool(body["success"])
        response.granted = bool(body["granted"])
        response.held_by = str(body.get("held_by", ""))
        response.reason = str(body.get("reason", ""))
        response.applied = bool(body.get("applied", False))
        return response

    def _fill_release_floor_response(self, response: Any, body: dict) -> Any:
        """Заполнить типизированный ответ ``ReleaseFloorResponse``."""
        response.success = bool(body["success"])
        response.reason = str(body.get("reason", ""))
        response.applied = bool(body.get("applied", False))
        return response

    def _fill_set_avatar_mode_response(self, response: Any, body: dict) -> Any:
        """Заполнить типизированный ответ ``SetAvatarModeResponse``."""
        response.success = bool(body["success"])
        response.mode = str(body.get("mode", ""))
        response.reason = str(body.get("reason", ""))
        response.applied = bool(body.get("applied", False))
        return response

    # ── fallback (std_srvs/Trigger) callbacks (W3-2/W3-4 legacy) ────
    # Используются только если rob_box_supervisor_msgs недоступен (CI mock-rclpy
    # ИЛИ недосборка workspace). В этом случае нода остаётся в monitor и
    # сервисы возвращают тот же W3-2/W3-4 JSON-в-message контракт.
    def _on_acquire_floor_fb(self, request: Any, response: Any) -> Any:
        """Fallback ``acquire_floor`` на ``std_srvs/Trigger`` (монитор-only).

        Контракт: ``request`` — пустой Trigger.Request (нет полей
        ``client_id``/``floor``); клиент, который хочет floor, должен
        использовать типизированный IDL (AV-12). Этот callback существует
        только для обратной совместимости с чужими тестовыми клиентами,
        которые ещё не перешли на AV-12.
        """
        body = self._monitor_response()
        return self._fill_monitor_response(response, body)

    def _on_release_floor_fb(self, request: Any, response: Any) -> Any:
        """Fallback ``release_floor`` на ``std_srvs/Trigger``."""
        body = self._monitor_response()
        return self._fill_monitor_response(response, body)

    def _on_set_avatar_mode_fb(self, request: Any, response: Any) -> Any:
        """Fallback ``set_avatar_mode`` на ``std_srvs/Trigger``."""
        body = self._monitor_response()
        return self._fill_monitor_response(response, body)

    def _fill_monitor_response(self, response: Any, body: Optional[dict] = None) -> Any:
        """Заполнить ``std_srvs/Trigger.response`` (success/message) монитор-ответом.

        Trigger.response имеет поля ``success: bool`` и ``message: string``.
        Кладём в ``message`` JSON-строку с полями ``applied``, ``granted``,
        ``held_by``, ``mode`` и ``reason`` (по ADR-0028 §4.5 клиенты
        должны видеть всё то же, что и в типизированном ответе).
        """
        if body is None:
            body = self._monitor_response()
        response.success = bool(body["success"])
        payload = {k: v for k, v in body.items() if k != "success"}
        response.message = json.dumps(payload)
        return response

    @staticmethod
    def _fill_floor_response(response: Any, body: dict) -> Any:
        """Симметричный adapter: заполнить ``std_srvs/Trigger.response`` для
        acquire/release_floor и set_avatar_mode (W3-2 legacy). Все три
        fallback-callback-а используют этот метод для единого JSON-конверта.

        В коде не вызывается на типизированном пути (там используются
        ``_fill_acquire_floor_response`` и т.д.) — оставлен для симметрии
        API и для backwards-compatible тестов, которые могут ещё
        использовать ``Trigger`` mock.
        """
        response.success = bool(body["success"])
        payload = {k: v for k, v in body.items() if k != "success"}
        response.message = json.dumps(payload)
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
