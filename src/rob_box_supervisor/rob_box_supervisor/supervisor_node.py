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

W3-2 (issue #968 wave2, провалы G2/G3) — ``acquire_floor``/
``release_floor`` перестали быть безусловными заглушками: в
``active``-режиме реально захватывают/отпускают floor через
:class:`~rob_box_supervisor.core.locks.LockManager` (dead-man 500 мс,
ADR-0028 §6 Q4). Контракт запроса — ЗАВЕДОМО переходный (см.
:py:meth:`_extract_floor_request`): ``std_srvs/Trigger.Request`` в
реальном ROS 2 не имеет полей вообще (пустой message перед ``---``),
поэтому JSON/атрибуты ``client_id``/``floor`` — временное решение до
кастомного IDL (AV-5, ADR-0028 §4.3). Это ЧЕСТНО задокументированный
технический долг, а не полноценный wire-контракт — см. ADR-0028 §4.2.

W3-4 (issue #968 wave2) — ``set_avatar_mode`` перестал быть заглушкой:
в ``active``-режиме реально прогоняет FSM-событие через
:class:`~rob_box_supervisor.core.fsm.ModeManager` (переходы —
ADR-0028 §4.1) и отвечает ``applied=true`` + текущим avatar-режимом
(``actual_mode``, поле по ADR-0028 §4.3). Контракт запроса — тот же
переходный техдолг, что и floor-ы (W3-2): ``event``/``client_id``
вместо честного IDL (см. :py:meth:`_extract_avatar_mode_request`).
При уходе из активного avatar-режима (``*_release``/``force_off``/
``both_release``) floor-ы, которые ``ModeManager`` перестал считать
занятыми, зеркально освобождаются и в ``LockManager`` — иначе остаётся
висячий holder, до которого никто больше не может достучаться через
``ReleaseFloor`` (см. :py:meth:`_set_avatar_mode_logic`).

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
import time
from typing import Any, Callable, Optional

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

    # ── heartbeat (AV-13, ADR-0028 §4.4 S10) ──────────────────────────
    TELEOP_HEARTBEAT_TOPIC = "/teleop_heartbeat"
    # Best-effort: heartbeat терпит потери — важна свежесть, не доставка.
    HEARTBEAT_QOS_DEPTH = 10
    # Период проверки протухания floor-ов (10 Гц — половина от желаемой
    # 10 Гц частоты heartbeat, чтобы пограничный кейс 500 мс не проскакивал
    # мимо). ADR-0028 §4.4 S10 требует «не реже 10 Гц».
    FLOOR_EXPIRY_CHECK_PERIOD_S = 0.1

    def __init__(self) -> None:
        super().__init__("avatar_supervisor")

        # Параметр mode (default monitor). В monitor — наблюдаем, не
        # вмешиваемся. В active — Phase 2 (NOT_IMPLEMENTED в AV-6).
        self.declare_parameter("mode", "monitor")
        self._mode: str = str(self.get_parameter("mode").value or "monitor")

        # AV-13: параметр dead_man_timeout_ms (default 500, ADR-0028 §6 Q4).
        # Вынесен в параметр, чтобы можно было подкрутить на железе без пересборки
        # (Phase 1 метрика — собрать dead_man_trips_total и посмотреть на
        # практике, см. ADR-0028 §6 Q4).
        self.declare_parameter("dead_man_timeout_ms", 500)
        self._dead_man_timeout_ms: int = int(self.get_parameter("dead_man_timeout_ms").value or 500)

        # AV-13: единственный источник времени ноды (для тестируемости).
        # LockManager и watcher используют self._now_ms() вместо time.time() —
        # тесты подменяют на fake clock через self._clock, нода в проде —
        # оставляет default (time.monotonic).
        self._clock: Callable[[], int] = lambda: int(time.monotonic() * 1000)
        self._now_ms = self._clock

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
        self._lock_manager = LockManager(clock=self._now_ms, timeout_ms=self._dead_man_timeout_ms)
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
                "avatar_supervisor: TeleopHeartbeat IDL not importable; "
                "/teleop_heartbeat subscription skipped, dead-man enforcement "
                "disabled (monitor-safe, ADR-0028 §4.5)"
            )

        # AV-13: watcher-таймер для периодической проверки протухания floor-ов
        # (ADR-0028 §4.4 S10, не реже 10 Гц). Этот таймер — отдельный от
        # 1 Гц-таймера публикации /avatar/state, потому что при dead-man trip
        # мы публикуем /avatar/state ВНЕОЧЕРЕДНО (см. _check_floor_expiry),
        # а не ждём следующего 1 Гц-тика.
        self._expiry_timer = self.create_timer(self.FLOOR_EXPIRY_CHECK_PERIOD_S, self._check_floor_expiry)

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
            self._log.warning(f"teleop_heartbeat: rejected client_id={client_id} reason={exc}")
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
        """Лёгкая копия :py:meth:`_publish_avatar_state` для внеочередных trip-ов.

        Отличия от 1 Гц-таймера: НЕ зовёт ``_check_dead_man_trips`` (это
        и есть watcher — вызывающий код), используется только когда
        watcher уже решил «нужен внеочередной publish».
        """
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
        msg.data = payload.decode("latin-1")
        self._state_pub.publish(msg)

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
        self._log.info(f"avatar_supervisor started: mode={self._mode}, " f"zenoh={zenoh}, msgpack={msgpack_state}")

    def _monitor_response(self) -> dict:
        """Стандартный ответ для всех сервисов в monitor-режиме."""
        return {
            "success": True,
            "applied": False,
            "reason": MONITOR_MODE_REASON,
        }

    def _publish_avatar_state(self) -> None:
        """Timer-callback (1 Hz): публикует свежий snapshot агрегатора в /avatar/state.

        AV-13: dead-man trip-метрика теперь собирается в
        :py:meth:`_check_floor_expiry` (10 Гц watcher, активный снос
        expired floor-ов) — НЕ здесь. Этот метод только публикует,
        чтобы не было двух путей снятия floor (acceptance «двух путей
        снятия floor быть не должно»).
        """
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
        msg.data = payload.decode("latin-1")  # ROS String — UTF-8 safe для bytes-as-text
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

    # ── service callbacks (W3-2: active → LockManager, monitor → как было) ──
    @staticmethod
    def _extract_floor_request(request: Any) -> tuple[Optional[str], Optional[str]]:
        """Достать ``client_id``/``floor`` из запроса — ЗАВЕДОМО переходный код.

        ``std_srvs/Trigger.Request`` в реальном ROS 2 не имеет полей вообще
        (пустой message перед ``---``) — это НЕ полноценный wire-контракт,
        а временное решение до кастомного IDL (AV-5, ADR-0028 §4.3). Пока
        поддерживаем два пути, оба задокументированы как техдолг:

        1. Атрибуты ``request.client_id``/``request.floor`` напрямую —
           так будущий IDL (AV-5) подключится без изменений в этом методе.
        2. JSON-строка в ``request.data`` (или ``request.message``) —
           «быстрый» переходный вариант, симметричный тому, как
           :py:meth:`_fill_monitor_response` уже пакует JSON в
           ``response.message``.
        """
        client_id = getattr(request, "client_id", None)
        floor = getattr(request, "floor", None)
        if client_id and floor:
            return str(client_id), str(floor)

        raw = getattr(request, "data", None) or getattr(request, "message", None)
        data = AvatarSupervisor._try_parse_json(raw)
        if isinstance(data, dict):
            cid = data.get("client_id")
            fl = data.get("floor")
            if cid and fl:
                return str(cid), str(fl)
        return None, None

    def _acquire_floor_logic(self, client_id: Optional[str], floor: Optional[str]) -> dict:
        """Чистая логика ``AcquireFloor`` (тестируется без rclpy).

        В ``monitor`` — как раньше: ``applied=false``, floor не трогаем
        (S12). В ``active`` — реально дёргаем :class:`LockManager`.
        """
        if self._mode != "active":
            return {"success": True, "applied": False, "granted": False, "reason": MONITOR_MODE_REASON}
        from rob_box_supervisor.core import Floor, LockConflictError  # noqa: PLC0415

        if not client_id or floor not in Floor.values():
            return {
                "success": True,
                "applied": False,
                "granted": False,
                "reason": f"invalid_request: client_id={client_id!r} floor={floor!r}",
            }

        try:
            self._lock_manager.acquire(client_id, floor)
        except LockConflictError as exc:
            return {
                "success": True,
                "applied": True,
                "granted": False,
                "reason": f"conflict: held_by={exc.held_by}",
            }
        self._known_floor_holders[floor] = client_id
        return {"success": True, "applied": True, "granted": True, "reason": "granted"}

    def _release_floor_logic(self, client_id: Optional[str], floor: Optional[str]) -> dict:
        """Чистая логика ``ReleaseFloor`` (тестируется без rclpy)."""
        if self._mode != "active":
            return {"success": True, "applied": False, "reason": MONITOR_MODE_REASON}
        from rob_box_supervisor.core import Floor  # noqa: PLC0415

        if not client_id or floor not in Floor.values():
            return {
                "success": True,
                "applied": False,
                "reason": f"invalid_request: client_id={client_id!r} floor={floor!r}",
            }
        try:
            self._lock_manager.release(client_id, floor)
        except PermissionError as exc:
            return {"success": True, "applied": False, "reason": f"permission_denied: {exc}"}
        if self._known_floor_holders.get(floor) == client_id:
            self._known_floor_holders[floor] = None
        return {"success": True, "applied": True, "reason": "released"}

    def _on_acquire_floor(self, request: Any, response: Any) -> Any:
        """``AcquireFloor`` — monitor: лог + monitor response; active: LockManager."""
        client_id, floor = self._extract_floor_request(request)
        body = self._acquire_floor_logic(client_id, floor)
        self._log.info(
            f"AcquireFloor: client_id={client_id} floor={floor} mode={self._mode} "
            f"granted={body['granted']} reason={body['reason']}"
        )
        return self._fill_floor_response(response, body)

    def _on_release_floor(self, request: Any, response: Any) -> Any:
        """``ReleaseFloor`` — monitor: лог + monitor response; active: LockManager."""
        client_id, floor = self._extract_floor_request(request)
        body = self._release_floor_logic(client_id, floor)
        self._log.info(
            f"ReleaseFloor: client_id={client_id} floor={floor} mode={self._mode} "
            f"applied={body['applied']} reason={body['reason']}"
        )
        return self._fill_floor_response(response, body)

    @staticmethod
    def _extract_avatar_mode_request(request: Any) -> tuple[Optional[str], Optional[str]]:
        """Достать ``event``/``client_id`` из SetAvatarMode-запроса (W3-4).

        Тот же переходный контракт, что и :py:meth:`_extract_floor_request`
        (W3-2): атрибуты запроса напрямую ИЛИ JSON в ``request.data``/
        ``request.message`` — вместо честного IDL (AV-5, ADR-0028 §4.3).
        ADR-0028 §4.3 документирует поле ``mode`` в контракте сервиса, но
        :class:`~rob_box_supervisor.core.fsm.ModeManager` — событийный
        автомат (переходы по ``EVENT_*`` из ADR-0028 §4.1 mermaid), а не
        setter состояния, поэтому здесь и в клиентском payload ожидается
        ``event`` — имя ребра диаграммы, а не целевой режим напрямую.
        """
        event = getattr(request, "event", None)
        client_id = getattr(request, "client_id", None)
        if event:
            return str(event), (str(client_id) if client_id else None)

        raw = getattr(request, "data", None) or getattr(request, "message", None)
        data = AvatarSupervisor._try_parse_json(raw)
        if isinstance(data, dict):
            ev = data.get("event")
            cid = data.get("client_id")
            if ev:
                return str(ev), (str(cid) if cid else None)
        return None, None

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
        """
        if self._mode != "active":
            return {
                "success": True,
                "applied": False,
                "actual_mode": self._mode_manager.mode.value,
                "reason": MONITOR_MODE_REASON,
            }
        if not event:
            return {
                "success": True,
                "applied": False,
                "actual_mode": self._mode_manager.mode.value,
                "reason": f"invalid_request: event={event!r}",
            }

        from rob_box_supervisor.core import Floor, FSMConflictError  # noqa: PLC0415

        prev_voice = self._mode_manager.voice_held_by()
        prev_teleop = self._mode_manager.teleop_held_by()

        try:
            new_mode = self._mode_manager.transition(event, client_id)
        except ValueError as exc:
            return {
                "success": True,
                "applied": False,
                "actual_mode": self._mode_manager.mode.value,
                "reason": f"invalid_event: {exc}",
            }
        except FSMConflictError as exc:
            return {
                "success": True,
                "applied": False,
                "actual_mode": self._mode_manager.mode.value,
                "reason": f"conflict: floor={exc.floor} held_by={exc.held_by}",
            }

        if prev_voice is not None and self._mode_manager.voice_held_by() is None:
            self._release_lock_manager_floor(prev_voice, Floor.VOICE)
        if prev_teleop is not None and self._mode_manager.teleop_held_by() is None:
            self._release_lock_manager_floor(prev_teleop, Floor.TELEOP)

        return {"success": True, "applied": True, "actual_mode": new_mode.value, "reason": "applied"}

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
        """``SetAvatarMode`` — active: реальный переход через ModeManager
        (W3-4); monitor: лог + monitor response (ADR-0028 §4.5), как раньше."""
        event, client_id = self._extract_avatar_mode_request(request)
        body = self._set_avatar_mode_logic(event, client_id)
        self._log.info(
            f"SetAvatarMode: event={event} client_id={client_id} mode={self._mode} "
            f"applied={body['applied']} actual_mode={body['actual_mode']} reason={body['reason']}"
        )
        return self._fill_floor_response(response, body)

    def _fill_monitor_response(self, response: Any) -> Any:
        """Заполнить std_srvs/Trigger.response (success/message) монитор-ответом.

        Trigger.response имеет поля ``success: bool`` и ``message: string``.
        Кладём в ``message`` JSON-строку с полями ``applied`` и ``reason``
        (по ADR-0028 §4.5 клиенты должны видеть все три поля).
        """
        body = self._monitor_response()
        response.success = bool(body["success"])
        response.message = json.dumps({"applied": body["applied"], "reason": body["reason"]})
        return response

    @staticmethod
    def _fill_floor_response(response: Any, body: dict) -> Any:
        """Заполнить std_srvs/Trigger.response для acquire/release_floor (W3-2)
        и set_avatar_mode (W3-4) — все три сервиса разделяют один и тот же
        JSON-конверт для ``response.message``.

        Симметрично :py:meth:`_fill_monitor_response`: ``response.message``
        несёт JSON со всеми полями кроме ``success`` (``applied``,
        ``granted`` для acquire, ``actual_mode`` для set_avatar_mode,
        ``reason``) — отсутствующие в конкретном body ключи просто не
        попадают в JSON.
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
