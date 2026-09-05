"""AvatarArbiter — ROS 2 нода-арбитр floor/FSM без LLM (ADR-0051 §2.2, #1987).

Вынесена из ``avatar_supervisor`` (supervisor_node.py) арбитражная часть:
единственный владелец ``LockManager`` + FSM режимов + агрегация
``/avatar/state``. Сервисы ``acquire_floor`` / ``release_floor`` /
``set_avatar_mode`` отвечают по адресу ``/avatar_arbiter/*`` с жёстким
latency-бюджетом (< 100 мс): в этом процессе НЕТ LLM / агентского цикла /
voice-параметров (ADR-0051 §2.2, целевая архитектура §4).

Дизайн:
- Параметр ``mode`` (default ``"monitor"``). В ``monitor``-режиме нода
  публикует ``/avatar/state`` из агрегатора и **отвечает** на сервисы
  ``AcquireFloor`` / ``ReleaseFloor`` / ``SetAvatarMode`` типизированным
  ответом ``success=true, applied=false, reason=supervisor_in_monitor_mode``,
  **не** меняя ``twist_mux`` inputs (ADR-0028 §4.5). Это минимизирует
  blast radius: нода задеплоена и наблюдает, реальное влияние — после
  явного ``mode:=active``.
- В ``active``-режиме реально арбитрирует floor-ы (см. W3-2/W3-4 ниже).

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
заглушка: в ``active``-режиме реально меняет режим через
:class:`~rob_box_supervisor.core.fsm.ModeManager` (переходы —
ADR-0028 §4.1) и отвечает ``applied=true`` + текущим avatar-режимом
в типизированном поле ``mode``.

``request.mode`` — это ЦЕЛЕВОЙ режим (``off``/``telegram_active``/
``avatar_present``/``mixed``), а не имя FSM-события: так требует
wire-контракт ``meta-quest-api.md`` §3 (фрейм ``0x30 SET_MODE``) и
ADR-0028 §4.3, где поле ответа называется ``actual_mode``.
``ModeManager`` при этом остаётся событийным автоматом — целевой
режим превращается в событие таблицей :data:`MODE_TRANSITIONS`,
и это единственное место, где такое превращение происходит.
Клиенты имён событий не знают: одно событие значит разные переходы
из разных режимов (``quest_release`` — это ``avatar_present → off``,
но ``mixed → telegram_active``), поэтому событие на проводе
неоднозначно без клиентской копии FSM, а копия разъедется.
При уходе из активного avatar-режима (``*_release``/``force_off``/
``both_release``) floor-ы, которые ``ModeManager`` перестал считать
занятыми, зеркально освобождаются и в ``LockManager`` — иначе
остаётся висячий holder, до которого никто больше не может достучаться
через ``ReleaseFloor`` (см. :py:meth:`_set_avatar_mode_logic`).

Источники истины:
- ADR-0028 §4.3 (ROS 2 API)
- ADR-0028 §4.5 (monitor-режим)
- ADR-0028 §6 Q4 (``dead_man_trips_total{client_id}``)
- ADR-0051 §2.2 + целевая архитектура §4 (вынос арбитра из супервизора)
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
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

from rob_box_supervisor.core.state import (
    AvatarState,
    StateTransportError,
    StateVersionError,
    encode_for_ros_string,
)


# AV-14 (issue #1906) — ``/avatar/state`` wire format lives in
# :mod:`rob_box_supervisor.core.state`. The arbiter here ONLY calls
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


# FSM-событийные имена (ADR-0028 §4.1 mermaid). Это ВНУТРЕННИЙ словарь
# арбитра: ровно ``core.fsm._ALL_EVENTS``, ни одного лишнего. Раньше
# здесь лежали ещё ``quest_acquire_full_floor``/``quest_release_teleop``/
# ``telegram_release_voice`` — имена рёбер из mermaid-диаграммы, которых
# в ``core.fsm`` не существует; ``ModeManager.transition()`` отвечал на
# них ``ValueError``. Держим список синхронным с автоматом, а не с
# картинкой.
#
# На провод события НЕ выходят: клиент шлёт целевой режим (см.
# ``MODE_TRANSITIONS`` ниже и ``SetAvatarMode.srv``).
EVENT_TELEGRAM_ACQUIRE_FLOOR = "telegram_acquire_floor"
EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR = "telegram_acquire_voice_floor"
EVENT_TELEGRAM_RELEASE = "telegram_release"
EVENT_QUEST_ACQUIRE_FLOOR = "quest_acquire_floor"
EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY = "quest_acquire_floor_teleop_only"
EVENT_QUEST_RELEASE = "quest_release"
EVENT_BOTH_RELEASE = "both_release"
EVENT_FORCE_OFF = "force_off"


# Wire-уровневые имена режимов (поле ``mode`` в SetAvatarMode и во фрейме
# ``0x30 SET_MODE``, meta-quest-api.md §3). Совпадают со значениями
# ``core.fsm.Mode``.
WIRE_MODE_OFF = "off"
WIRE_MODE_TELEGRAM_ACTIVE = "telegram_active"
WIRE_MODE_AVATAR_PRESENT = "avatar_present"
WIRE_MODE_MIXED = "mixed"

WIRE_MODES: tuple = (
    WIRE_MODE_OFF,
    WIRE_MODE_TELEGRAM_ACTIVE,
    WIRE_MODE_AVATAR_PRESENT,
    WIRE_MODE_MIXED,
)

# (текущий режим, целевой режим) → FSM-событие, которым ModeManager этот
# переход делает. Единственное место, где целевой режим превращается в
# событие: клиенты этой таблицы не знают и знать не должны.
#
# Почему таблица, а не «событие на проводе»: одно и то же событие значит
# разные переходы в зависимости от текущего режима (``quest_release`` —
# это ``avatar_present → off``, но ``mixed → telegram_active``). Событие
# на проводе поэтому неоднозначно, если клиент не держит у себя копию
# FSM. Копия неизбежно разъедется — см. ADR-0028 §4.1.
#
# Отсутствие пары в таблице = переход недостижим за один шаг (например
# ``off → mixed``: mixed требует ДВУХ клиентов). Такой запрос отклоняется
# c ``reason="conflict"``; сами через промежуточное состояние не ходим.
# Пара «режим сам в себя» отсутствует намеренно — идемпотентный no-op
# обрабатывается до обращения к таблице.
MODE_TRANSITIONS: dict = {
    # * → off: escape hatch, всегда разрешён (ADR-0028 §4.1 + §6 Q1).
    (WIRE_MODE_TELEGRAM_ACTIVE, WIRE_MODE_OFF): EVENT_FORCE_OFF,
    (WIRE_MODE_AVATAR_PRESENT, WIRE_MODE_OFF): EVENT_FORCE_OFF,
    (WIRE_MODE_MIXED, WIRE_MODE_OFF): EVENT_FORCE_OFF,
    # off → *
    (WIRE_MODE_OFF, WIRE_MODE_TELEGRAM_ACTIVE): EVENT_TELEGRAM_ACQUIRE_FLOOR,
    (WIRE_MODE_OFF, WIRE_MODE_AVATAR_PRESENT): EVENT_QUEST_ACQUIRE_FLOOR,
    # telegram_active → *
    (WIRE_MODE_TELEGRAM_ACTIVE, WIRE_MODE_AVATAR_PRESENT): EVENT_QUEST_ACQUIRE_FLOOR,
    (
        WIRE_MODE_TELEGRAM_ACTIVE,
        WIRE_MODE_MIXED,
    ): EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
    # avatar_present → *
    (WIRE_MODE_AVATAR_PRESENT, WIRE_MODE_MIXED): EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
    # mixed → *
    (WIRE_MODE_MIXED, WIRE_MODE_TELEGRAM_ACTIVE): EVENT_QUEST_RELEASE,
    (WIRE_MODE_MIXED, WIRE_MODE_AVATAR_PRESENT): EVENT_TELEGRAM_RELEASE,
}


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
    Они используются в :py:meth:`AvatarArbiter._register_services` для
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


class AvatarArbiter(Node):
    """ROS 2 нода ``avatar_arbiter`` — арбитр floor/FSM без LLM (Vision Pi)."""

    # ── topic / service constants (ADR-0028 §4.3) ─────────────────────
    AVATAR_STATE_TOPIC = "/avatar/state"

    ODOM_TOPIC = "/odom"
    DEVICE_SNAPSHOT_TOPIC = "/device/snapshot"
    VOICE_DIALOGUE_STATE_TOPIC = (
        "/voice/dialogue/state"  # НЕ /voice/state (ADR-0027 #2)
    )

    # Сервисы объявляются с АБСОЛЮТНЫМИ именами под префиксом
    # ``/avatar_arbiter/`` (ADR-0051 §2.2, issue #1987): клиенты
    # (quest_node, ТАРС) ходят на /avatar_arbiter/{acquire_floor,
    # release_floor,set_avatar_mode}. Раньше супервизор регистрировал
    # эти сервисы относительными именами в корне (/acquire_floor и т.д.).
    ACQUIRE_FLOOR_SERVICE = "/avatar_arbiter/acquire_floor"
    RELEASE_FLOOR_SERVICE = "/avatar_arbiter/release_floor"
    SET_AVATAR_MODE_SERVICE = "/avatar_arbiter/set_avatar_mode"

    # ── heartbeat (AV-13, ADR-0028 §4.4 S10) ──────────────────────────
    TELEOP_HEARTBEAT_TOPIC = "/teleop_heartbeat"
    # Best-effort: heartbeat терпит потери — важна свежесть, не доставка.
    HEARTBEAT_QOS_DEPTH = 10
    # Период проверки протухания floor-ов (10 Гц — половина от желаемой
    # 10 Гц частоты heartbeat, чтобы пограничный кейс 500 мс не проскакивал
    # мимо). ADR-0028 §4.4 S10 требует «не реже 10 Гц».
    FLOOR_EXPIRY_CHECK_PERIOD_S = 0.1

    def __init__(self) -> None:
        super().__init__("avatar_arbiter")

        # Параметр mode (default monitor). В monitor — наблюдаем, не
        # вмешиваемся. В active — Phase 2 (NOT_IMPLEMENTED в AV-6).
        self.declare_parameter("mode", "monitor")
        self._mode: str = str(self.get_parameter("mode").value or "monitor")

        # AV-13: параметр dead_man_timeout_ms (default 500, ADR-0028 §6 Q4).
        # Вынесен в параметр, чтобы можно было подкрутить на железе без пересборки
        # (Phase 1 метрика — собрать dead_man_trips_total и посмотреть на
        # практике, см. ADR-0028 §6 Q4).
        self.declare_parameter("dead_man_timeout_ms", 500)
        self._dead_man_timeout_ms: int = int(
            self.get_parameter("dead_man_timeout_ms").value or 500
        )

        # AV-13 / Issue #1968: единый источник времени ноды (для
        # тестируемости). LockManager и watcher используют self._now_ms()
        # вместо time.time() — тесты подменяют на fake clock через
        # ``AvatarArbiter._now_ms``, нода в проде — оставляет default
        # (time.monotonic).
        #
        # ВАЖНО: имя ``_clock`` было зарезервировано rclpy.Node для clock-
        # объекта (rclpy.clock.Clock), у которого есть ``.handle`` атрибут.
        # rclpy.timer.Timer.__init__ читает ``self._clock.handle`` и
        # ``self._context.handle`` — перезапись ``self._clock`` лямбдой
        # приводит к ``AttributeError: 'function' object has no attribute
        # 'handle'`` на ``create_timer()`` (issue #1968, deploy run 33751147006
        # от 2026-09-03). Храним call'able только в ``self._now_ms``,
        # ``self._clock`` остаётся за rclpy.
        self._now_ms: Callable[[], int] = lambda: int(time.monotonic() * 1000)

        # AV-13: подписка на /teleop_heartbeat. В CI (mock-rclpy) пакета
        # сообщений нет — импорт делаем лениво, при неудаче логируем WARN и
        # остаёмся в monitor (dead-man enforcement недоступен, ADR-0028 §4.5).
        # Сам факт отсутствия подписки — failure, который нужно задетектить
        # сразу на старте, поэтому вызываем из __init__ (а не лениво в
        # callback), и сохраняем тип в self._heartbeat_msg_type для тестов.
        self._heartbeat_msg_type: Optional[Any] = self._try_import_heartbeat_msg()

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
        # ADR-0028 §4.2) для сервисов AcquireFloor/ReleaseFloor. AV-13:
        # пробрасываем ``dead_man_timeout_ms`` (ROS-параметр) как
        # конструкторский override, чтобы можно было тюнить порог на железе
        # без пересборки. ``clock`` тоже инжектируем — для тестов с fake-clock.
        self._lock_manager = LockManager(
            clock=self._now_ms, timeout_ms=self._dead_man_timeout_ms
        )
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

        # AV-13: подписка на /teleop_heartbeat (ADR-0028 §4.4 S10).
        # QoS best-effort, depth=10 (heartbeat терпит потери — важна
        # свежесть, не надёжная доставка). Если IDL-пакет не собран
        # (CI, fresh clone без colcon build) — пропускаем подписку,
        # логируем WARN, dead-man enforcement недоступен. Уже было
        # сделано в self._try_import_heartbeat_msg() выше.
        if self._heartbeat_msg_type is not None:
            from rclpy.qos import ReliabilityPolicy  # noqa: PLC0415

            heartbeat_qos = QoSProfile(
                depth=self.HEARTBEAT_QOS_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(
                self._heartbeat_msg_type,
                self.TELEOP_HEARTBEAT_TOPIC,
                self._on_teleop_heartbeat,
                heartbeat_qos,
            )
        else:
            self._log.warning(
                "avatar_arbiter: TeleopHeartbeat IDL not importable; "
                "/teleop_heartbeat subscription skipped, dead-man enforcement "
                "disabled (monitor-safe, ADR-0028 §4.5)"
            )

        # AV-13: watcher-таймер для периодической проверки протухания floor-ов
        # (ADR-0028 §4.4 S10, не реже 10 Гц). Этот таймер — отдельный от
        # 1 Гц-таймера публикации /avatar/state, потому что при dead-man trip
        # мы публикуем /avatar/state ВНЕОЧЕРЕДНО (см. _check_floor_expiry),
        # а не ждём следующего 1 Гц-тика.
        self._expiry_timer = self.create_timer(
            self.FLOOR_EXPIRY_CHECK_PERIOD_S, self._check_floor_expiry
        )

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

    # ── heartbeat / dead-man (AV-13, ADR-0028 §4.4 S10) ────────────────
    def _try_import_heartbeat_msg(self) -> Optional[Any]:
        """Ленивый try-import :class:`TeleopHeartbeat` из IDL-пакета AV-12.

        В CI (mock-rclpy) и fresh-clone без ``colcon build`` пакета
        ``rob_box_supervisor_msgs`` нет — импорт падает, мы возвращаем
        ``None``. В этом случае :py:meth:`__init__` пропускает
        ``create_subscription``, логирует WARN и остаётся в monitor —
        dead-man enforcement недоступен (ADR-0028 §4.5).

        В проде после AV-12 (issue #1904, ветка
        ``z-{agent}/1904-av-12-rob-box-supervisor-msgs-idl-floor-``)
        IDL будет собран, импорт успешен, подписка зарегистрирована.
        """
        try:
            from rob_box_supervisor_msgs.msg import TeleopHeartbeat  # noqa: PLC0415

            return TeleopHeartbeat
        except ImportError:
            return None

    def _on_teleop_heartbeat(self, msg: Any) -> None:
        """Обработать ``/teleop_heartbeat`` от teleop-клиента (ADR-0028 §4.4 S10).

        Поведение по :data:`_mode`:

        - ``monitor`` — только считаем статистику (счётчик пришедших
          heartbeat-ов), floor **не** трогаем (S12: monitor не вмешивается).
          Если супервизор случайно держит floor по лени от прошлой
          active-сессии — здесь не сбрасываем, иначе теряем state для
          диагностики.
        - ``active`` — зовём ``LockManager.heartbeat(...)``, который
          продлевает ``last_heartbeat_ms`` для текущего holder-а
          ``teleop_floor``. Heartbeat от клиента, который floor **не**
          держит (state пуст или держит другой client_id) —
          игнорируем, чтобы случайный клиент не смог «оживить» floor
          за другого (LockManager бросает :class:`PermissionError` —
          ловим и логируем WARN).

        ``now_ms`` берём из :data:`_now_ms` (а не ``time.time()``) —
        единственный источник времени ноды, чтобы тесты с fake-clock
        могли контролировать часы. Это контрактное требование карточки
        (acceptance «часы подменяемы в тестах»).
        """
        client_id = getattr(msg, "client_id", None) or ""
        if not client_id:
            self._log.warning("teleop_heartbeat: empty client_id, ignored")
            return

        if self._mode != "active":
            # Monitor: считаем статистику, не вмешиваемся (S12).
            self._aggregator.record_heartbeat_seen(client_id)
            return

        from rob_box_supervisor.core import Floor  # noqa: PLC0415

        now_ms = self._now_ms()
        try:
            self._lock_manager.heartbeat(client_id, Floor.TELEOP, now_ms=now_ms)
        except PermissionError as exc:
            # Heartbeat от клиента, который НЕ держит teleop_floor (или
            # держит другой) — это НЕ трип, это попытка heartbeat-а не туда.
            # Контракт LockManager.heartbeat: PermissionError если нет
            # holder-а или чужой client_id. Не валим ноду, логируем WARN —
            # типичный случай race с release.
            self._log.warning(
                f"teleop_heartbeat: rejected client_id={client_id} reason={exc}"
            )
            return
        # Продлили — обновим снимок holder-ов, чтобы метрика trip
        # (см. _check_floor_expiry) корректно отличала «клиент вышел»
        # от «клиент тикает» в следующем тике.
        self._known_floor_holders[Floor.TELEOP] = client_id

    def _check_floor_expiry(self) -> None:
        """Watcher-таймер 10 Гц: снимаем expired floor-ы + внеочередной publish.

        AV-13, ADR-0028 §4.4 S10. Период 100 мс (см.
        :data:`FLOOR_EXPIRY_CHECK_PERIOD_S`) — половина от желаемой
        10 Гц частоты heartbeat, чтобы пограничный кейс 500 мс не
        проскакивал мимо.

        Алгоритм:

        1. Для каждого floor-а (:data:`Floor.TELEOP`, :data:`Floor.VOICE`)
           вызываем :py:meth:`LockManager.force_expire` — он активно
           переводит expired floor в ``None`` (в отличие от ленивого
           :py:meth:`LockManager.holder`).
        2. Если floor действительно был снят (``expired_holder`` не
           ``None``) — инкрементируем :class:`DeadManCounter` /
           агрегатор-метрику, логируем WARN с фактическим возрастом
           heartbeat-а в мс и **немедленно** публикуем ``/avatar/state``
           вне очереди — клиенты должны узнать о снятии floor-а сразу,
           а не через секунду (1 Гц-таймер ``_publish_avatar_state``).

        При протухании нескольких floor-ов за один тик публикуем ОДИН
        раз в конце (дешевле, чем публиковать на каждый trip).

        Этот метод ЗАМЕНЯЕТ прежний ``_check_dead_man_trips`` (1 Гц,
        ленивый, только метрика): watcher теперь единственный путь и для
        enforcement, и для метрики (acceptance «двух путей снятия floor
        быть не должно»).
        """
        from rob_box_supervisor.core import Floor  # noqa: PLC0415

        now_ms = self._now_ms()
        any_tripped = False

        for floor in (Floor.TELEOP, Floor.VOICE):
            expired_holder = self._lock_manager.force_expire(floor, now_ms=now_ms)
            if expired_holder is None:
                # Floor ещё живой / уже был свободен — обновим снимок
                # для следующего тика (prev → current), чтобы метрика
                # была согласована с LockManager.
                self._known_floor_holders[floor] = self._lock_manager.holder(floor)
                continue

            # Trip: floor реально был снят.
            new_count = self._aggregator.record_dead_man_trip(expired_holder)
            self._known_floor_holders[floor] = None
            self._log.warning(
                f"dead_man_trip: client_id={expired_holder} floor={floor} "
                f"timeout_ms={self._dead_man_timeout_ms} count={new_count}"
            )
            any_tripped = True

        if any_tripped:
            # Внеочередной publish — клиенты узнают о снятии floor-а сразу.
            # Запускаем только в active (в monitor метрика trip-ов всё равно
            # собирается, но /avatar/state и так публикуется каждую секунду
            # — внеочередной там избыточен и шумит в логе).
            if self._mode == "active":
                self._publish_avatar_state_inline()

    def _publish_avatar_state_inline(self) -> None:
        """Внеочередная публикация ``/avatar/state`` после dead-man trip.

        Отличие от 1 Гц-таймера только в том, кто её инициировал: тик
        публикует по расписанию, а сюда приходят из
        :py:meth:`_check_floor_expiry`, когда floor реально сняли и
        клиентам надо узнать об этом сразу, а не через секунду.

        Кодек — тот же :func:`encode_for_ros_string` (AV-14 #1906):
        второй путь сериализации здесь недопустим, ровно на нём
        издатель и потребитель однажды разошлись.
        """
        try:
            state = self._build_published_avatar_state()
            payload_str = encode_for_ros_string(state)
        except (StateTransportError, StateVersionError) as exc:
            self._log.warning(
                f"avatar_arbiter: внеочередной publish пропущен: {exc}"
            )
            return
        except Exception as exc:  # noqa: BLE001 — не валить watcher
            self._log.warning(
                f"avatar_arbiter: unexpected encode failure: {type(exc).__name__}: {exc}"
            )
            return
        msg = RosString()
        msg.data = payload_str
        self._state_pub.publish(msg)

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

            self._srv_acquire = self.create_service(
                Trigger, self.ACQUIRE_FLOOR_SERVICE, self._on_acquire_floor_fb
            )
            self._srv_release = self.create_service(
                Trigger, self.RELEASE_FLOOR_SERVICE, self._on_release_floor_fb
            )
            self._srv_set_mode = self.create_service(
                Trigger, self.SET_AVATAR_MODE_SERVICE, self._on_set_avatar_mode_fb
            )

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
            f"avatar_arbiter started: mode={self._mode}, "
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
        try:
            state = self._build_published_avatar_state()
            payload_str = encode_for_ros_string(state)
        except (StateTransportError, StateVersionError) as exc:
            # Rate-limit: один WARN на тик максимум, чтобы не засорять лог
            # при циклической ошибке (publisher 1 Hz, log-flooding = bad).
            self._log.warning(
                f"avatar_arbiter: /avatar/state publish skipped: {exc}"
            )
            return
        except Exception as exc:  # noqa: BLE001 — не валить таймер
            self._log.warning(
                f"avatar_arbiter: unexpected encode failure: "
                f"{type(exc).__name__}: {exc}"
            )
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

    # ── service callbacks (W3-2: active → LockManager, monitor → как было) ──
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

    def _acquire_floor_logic(
        self, client_id: Optional[str], floor: Optional[str]
    ) -> dict:
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

    def _release_floor_logic(
        self, client_id: Optional[str], floor: Optional[str]
    ) -> dict:
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
    def _extract_avatar_mode_request(
        request: Any,
    ) -> tuple[Optional[str], Optional[str]]:
        """Достать ``mode``/``client_id`` из типизированного SetAvatarMode-запроса.

        AV-12: ``SetAvatarMode.Request`` из ``rob_box_supervisor_msgs``
        имеет typed-поля ``client_id: string`` и ``mode: string``.
        ``mode`` — ЦЕЛЕВОЙ avatar-режим (``off``/``telegram_active``/
        ``avatar_present``/``mixed``), как требует wire-контракт
        ``meta-quest-api.md`` §3 (фрейм ``0x30 SET_MODE``) и ADR-0028 §4.3
        (поле ответа там называется ``actual_mode`` — значит в запросе
        желаемый). Имя FSM-события на провод НЕ выходит: превращение
        режима в событие живёт в ``MODE_TRANSITIONS``.

        Прошлый JSON-в-``data`` fallback УБРАН (карточка AV-12, W3-2/R13)
        по тем же причинам, что и для floor-ов.

        Возвращает ``(target_mode, client_id)``.
        """
        target_mode = getattr(request, "mode", None)
        client_id = getattr(request, "client_id", None)
        if not target_mode:
            return None, (str(client_id) if client_id else None)
        return str(target_mode), (str(client_id) if client_id else None)

    @staticmethod
    def _target_mode_to_event(current_mode: str, target_mode: str) -> Optional[str]:
        """``(текущий режим, целевой режим)`` → имя FSM-события.

        Чистая функция над :data:`MODE_TRANSITIONS` — тестируется без
        rclpy и без ноды. Возвращает ``None``, если переход недостижим за
        один шаг; вызывающий превращает это в ``reason="conflict"``.

        Запрос текущего же режима сюда не доходит — идемпотентный no-op
        разбирается выше по стеку (см. :py:meth:`_set_avatar_mode_logic`).
        """
        return MODE_TRANSITIONS.get((current_mode, target_mode))

    def _set_avatar_mode_logic(
        self, target_mode: Optional[str], client_id: Optional[str]
    ) -> dict:
        """Чистая логика ``SetAvatarMode`` через ModeManager (W3-4, тестируется без rclpy).

        Принимает ЦЕЛЕВОЙ режим (wire-контракт, см.
        :py:meth:`_extract_avatar_mode_request`), переводит его в
        FSM-событие через :data:`MODE_TRANSITIONS` и уже событие отдаёт
        ``ModeManager``.

        В ``monitor`` — как раньше: ``applied=false``, avatar-режим не
        трогаем (ADR-0028 §4.5/S12). В ``active`` — реально прогоняем
        событие через :class:`~rob_box_supervisor.core.fsm.ModeManager`;
        валидация самого перехода (``ConflictError``) остаётся внутри
        ModeManager, здесь только маппинг в ``success``/``applied``/
        ``reason``.

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
        current = self._mode_manager.mode.value

        # Пустой или неизвестный режим — bad_request. Проверяем ДО
        # идемпотентной ветки, иначе опечатка молча выглядела бы как
        # успешный no-op.
        if not target_mode or target_mode not in WIRE_MODES:
            return {
                "success": True,
                "applied": False,
                "mode": current,
                "reason": REASON_BAD_REQUEST,
            }

        # Запросили режим, в котором уже находимся — идемпотентный no-op.
        # Клиенты пере-отправляют SET_MODE на реконнекте, и падать на
        # этом нельзя (ADR-0028 §4.1: повторный acquire — no-op).
        if target_mode == current:
            return {
                "success": True,
                "applied": True,
                "mode": current,
                "reason": REASON_APPLIED,
            }

        event = self._target_mode_to_event(current, target_mode)
        if event is None:
            # Переход недостижим за один шаг (``off → mixed`` требует двух
            # клиентов). Через промежуточное состояние сами не ходим.
            return {
                "success": True,
                "applied": False,
                "mode": current,
                "reason": REASON_CONFLICT,
            }

        from rob_box_supervisor.core import Floor, FSMConflictError  # noqa: PLC0415

        prev_voice = self._mode_manager.voice_held_by()
        prev_teleop = self._mode_manager.teleop_held_by()

        try:
            new_mode = self._mode_manager.transition(event, client_id)
        except ValueError:
            # Сюда попасть можно, только если MODE_TRANSITIONS разъехался
            # с ``core.fsm._ALL_EVENTS`` — это баг супервизора, а не
            # клиента, поэтому success=False, а не тихий отказ.
            self._log.error(
                f"SetAvatarMode: MODE_TRANSITIONS отдал неизвестное событие "
                f"{event!r} для перехода {current!r} -> {target_mode!r}"
            )
            return {
                "success": False,
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
        target_mode, client_id = self._extract_avatar_mode_request(request)
        body = self._set_avatar_mode_logic(target_mode, client_id)
        self._log.info(
            f"SetAvatarMode: target={target_mode} client_id={client_id} mode={self._mode} "
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
    """Console-script entry point: ``ros2 run rob_box_supervisor avatar_arbiter``."""
    if not rclpy.ok():
        rclpy.init(args=args)
    node = AvatarArbiter()
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
