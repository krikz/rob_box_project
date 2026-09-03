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

import asyncio
import contextlib
import json
import os
import time
import uuid
from typing import Any, Iterator, Mapping, Optional

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

# AV-21 (issue #1913) — супервизор-агент «мозг оператора» (ADR-0028 §1.1).
# Вход: ``/avatar/command`` (std_msgs/String, JSON), выход:
# ``/avatar/command_result``. Полные JSON-схемы — в
# ``docs/architecture/avatar-supervisor-agent.md``. Наполнять вход
# будут карточки-после (AV-22: Quest STT, Telegram-текст).
AVATAR_COMMAND_TOPIC: str = "/avatar/command"
AVATAR_COMMAND_RESULT_TOPIC: str = "/avatar/command_result"

# Какой ``voice_input_mode`` выставлять на ``dialogue_node`` пока супервизор
# обрабатывает команду оператора. ``"off"`` — «диалог off» (W3-1, полное
# управление оператора; см. dialogue-mode-spec-2026-08-28.md §3.5).
# Переопределяется параметром ``agent_during_voice_mode``.
AGENT_DURING_VOICE_MODE_DEFAULT: str = "off"

# Валидные ``source``-поля в ``/avatar/command``. Используется только для
# метрик (label) и валидации payload-а — НЕ для роутинга (это работа
# AV-22). Неизвестные источники НЕ отбрасываем, лишь логируем warning
# (расширяемость — future: ``"web"``, ``"admin"``).
AGENT_COMMAND_SOURCES: tuple[str, ...] = ("quest", "telegram")

# Timeout ожидания ответа от OperatorHarness при обработке команды
# (защита от зависшего LLM, который отправил запрос и не вернул
# ответ). В текущем PR — без жёсткого таймаута, но константа здесь,
# чтобы Phase 2 не пришлось переписывать.
AGENT_COMMAND_TIMEOUT_S: float = 30.0


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

    # AV-21 (issue #1913) — супервизор-агент «мозг оператора».
    # ``agent_enabled`` гейт всего agent-прохода (default false — без
    # включения нода ведёт себя как сегодня). ``system_prompt_file`` —
    # имя файла в ``rob_box_harness/prompts/`` (см. _load_system_prompt).
    # ``agent_during_voice_mode`` — какой voice_input_mode выставлять на
    # dialogue_node на время обработки команды (default "off" —
    # "диалог off", полное управление оператора).
    AGENT_ENABLED_PARAM = "agent_enabled"
    SYSTEM_PROMPT_FILE_PARAM = "system_prompt_file"
    AGENT_DURING_VOICE_MODE_PARAM = "agent_during_voice_mode"

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

        # ── AV-21: супервизор-агент «мозг оператора» ────────────────
        # Параметры (см. ``AGENT_*_PARAM`` выше). ``agent_enabled`` —
        # мастер-гейт: при false нода ведёт себя как сегодня, никаких
        # tool-каллов и свапов голоса. ``system_prompt_file`` —
        # override имени файла в rob_box_harness/prompts/. ``agent_during_voice_mode``
        # — какой voice_input_mode ставить на время обработки команды
        # (default "off" — полное управление оператора).
        self.declare_parameter(self.AGENT_ENABLED_PARAM, False)
        self.declare_parameter(self.SYSTEM_PROMPT_FILE_PARAM, "operator_system_prompt.txt")
        self.declare_parameter(self.AGENT_DURING_VOICE_MODE_PARAM, AGENT_DURING_VOICE_MODE_DEFAULT)
        self._agent_enabled: bool = bool(self.get_parameter(self.AGENT_ENABLED_PARAM).value)
        self._system_prompt_file: str = str(
            self.get_parameter(self.SYSTEM_PROMPT_FILE_PARAM).value or "operator_system_prompt.txt"
        )
        self._agent_during_voice_mode: str = str(
            self.get_parameter(self.AGENT_DURING_VOICE_MODE_PARAM).value or AGENT_DURING_VOICE_MODE_DEFAULT
        )

        # OperatorHarness создаётся ЛЕНИВО (см. _ensure_agent_harness):
        # при ``agent_enabled=false`` мы не должны инстанцировать LLM /
        # Tools / Memory, чтобы нода в monitor-режиме не делала лишнего
        # ввода-вывода. Инвариант тестов: enabled=false → harness не
        # создаётся.
        self._agent_harness: Any = None
        # Текущий ``voice_input_mode`` dialogue_node — нужен для
        # _voice_mode_swap (восстановить прежнее значение в finally).
        # ``None`` = мы не знаем (первый swap после старта) → в finally
        # НЕ делаем restore, только логируем warning, чтобы не сбросить
        # режим в дефолт по своей инициативе.
        self._voice_input_mode_before_swap: Optional[str] = None

        # Publisher /avatar/command_result (для супервизор-агента).
        # QoS — default reliable (depth=10). Не latched: результаты
        # привязаны к конкретной команде, late joiner их НЕ получит
        # (это поведение «command-response», не «state-broadcast»).
        self._agent_result_pub = self.create_publisher(RosString, AVATAR_COMMAND_RESULT_TOPIC, 10)
        # Подписка /avatar/command — JSON с командой оператора.
        self.create_subscription(RosString, AVATAR_COMMAND_TOPIC, self._on_avatar_command, 10)

        # Метрики (см. rob_box_voice.observability.metrics). Регистрируются
        # лениво через get_metric — если prometheus_client недоступен,
        # это no-op (см. там же is_metrics_enabled). Метрики-объекты
        # кладём в self один раз (lazy init в __init__, не в каждом
        # callback — иначе на каждое сообщение новый Counter).
        self._agent_metrics = self._build_agent_metrics()

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
        # AV-14: msgpack is a hard dep (declared in package.xml). Its
        # availability is verified at import time by core.state; if the
        # codec raises here it means the environment is genuinely broken
        # (and we want a single loud failure, not a silent fallback to JSON).
        self._log.info(f"avatar_supervisor started: mode={self._mode}, " f"zenoh={zenoh}")

    def _monitor_response(self) -> dict:
        """Стандартный ответ для всех сервисов в monitor-режиме."""
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

    # ── AV-21 (issue #1913): супервизор-агент «мозг оператора» ─────
    # Скелет: топики /avatar/command → /avatar/command_result, гейт
    # ``agent_enabled``, voice-mode swap с try/finally, метрики через
    # rob_box_voice.observability.metrics. Дизайн / acceptance —
    # docs/plans/2026-09-02-avatar-supervisor-agent-design.md §5.

    # ── helpers: metrics ─────────────────────────────────────────────────────────────────

    def _build_agent_metrics(self) -> dict[str, Any]:
        """Зарегистрировать / достать метрики супервизор-агента.

        Используем ``get_metric`` из rob_box_voice.observability.metrics —
        он no-op'ит если prometheus_client нет (CI / unit-тесты), и
        идемпотентно регистрирует счётчик/гистограмму (повторный вызов
        с тем же именем = тот же объект). Метрики, как и в voice-слое,
        живут в process-global prometheus REGISTRY (cross-node в проде
        не пересекаются, т.к. каждая ROS-нода = отдельный процесс).
        """
        try:
            from rob_box_voice.observability.metrics import get_metric  # noqa: PLC0415
        except ImportError:
            # Если rob_box_voice недоступен (минимальный CI-env без
            # voice-пакета) — все методы record_* станут no-op через
            # ``MetricsDisabled``. Это идома проекта, см. metrics.py.
            get_metric = None  # type: ignore[assignment]
        if get_metric is None:
            # Подменяем на заглушки, чтобы ``self._agent_metrics`` всегда
            # был dict-ом и все .inc/.observe/.labels были no-op.
            return {
                "commands": _NoopLabelCounter(),
                "tool_calls": _NoopLabelCounter(),
                "latency": _NoopHistogram(),
                "enabled": False,
            }
        return {
            "commands": get_metric(
                "counter",
                "avatar_agent_commands_total",
                "Supervisor-agent command outcomes, labelled by source and result.",
                labelnames=("source", "result"),
            ),
            "tool_calls": get_metric(
                "counter",
                "avatar_agent_tool_calls_total",
                "Supervisor-agent tool invocations, labelled by tool name.",
                labelnames=("tool",),
            ),
            "latency": get_metric(
                "histogram",
                "avatar_agent_latency_seconds",
                "Supervisor-agent end-to-end command latency (seconds).",
            ),
            "enabled": True,
        }

    def _record_agent_command(self, source: str, result: str, latency_s: Optional[float] = None) -> None:
        """Инкрементить ``avatar_agent_commands_total`` и засечь гистограмму."""
        metrics = self._agent_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["commands"].labels(source=source or "unknown", result=result).inc()
            if latency_s is not None:
                metrics["latency"].observe(latency_s)
        except Exception as exc:  # noqa: BLE001 — метрики best-effort
            self._log.warning(f"agent_metrics: command record failed: {exc}")

    def _record_agent_tool_call(self, tool_name: str) -> None:
        """Инкрементить ``avatar_agent_tool_calls_total``."""
        metrics = self._agent_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["tool_calls"].labels(tool=tool_name).inc()
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"agent_metrics: tool_call record failed: {exc}")

    # ── helpers: voice-mode swap (ADR-0028 S5) ─────────────────────────────────────

    @contextlib.contextmanager
    def _voice_mode_swap(self) -> Iterator[None]:
        """Контекст-менеджер «пока оператор работает, личность молчит».

        ADR-0028 S5: единственная точка, которая имеет право менять
        ``voice_input_mode`` на ``dialogue_node`` — супервизор (через
        ``_apply_voice_mode`` / ``_set_dialogue_param``). При входе
        ставим ``agent_during_voice_mode`` (default "off"), при выходе
        (в ``finally``, включая путь с исключением) — восстанавливаем
        предыдущее значение. Если предыдущее неизвестно (``None`` —
        см. :py:meth:`_capture_current_voice_mode`), в finally НЕ делаем
        restore и только логируем — иначе свапнём режим в дефолт по
        своей инициативе.

        Контекст-менеджер намеренно вызывает ``_apply_voice_mode`` (а
        не пишет в dialogue_node напрямую) — это и есть «через
        супервизор», а не side-door (карточка §4). В monitor-режиме
        ``_apply_voice_mode`` возвращает ``(False, MONITOR_MODE_REASON)``,
        и свап фактически не применяется — но try/finally дисциплина
        сохраняется (тест-инвариант AC #8: «voice_input_mode
        восстановлен даже если LLM упал»).
        """
        prev_mode = self._voice_input_mode_before_swap
        # Входим в режим «оператор работает».
        applied_in, reason_in = self._apply_voice_mode(self._agent_during_voice_mode)
        if not applied_in:
            # В monitor — это ожидаемо; в active — повод для warn (но
            # НЕ ошибка, чтобы не валить обработку команды).
            self._log.debug(
                f"voice_mode_swap.enter: mode={self._agent_during_voice_mode} "
                f"applied={applied_in} reason={reason_in}"
            )
        try:
            yield
        finally:
            # Восстанавливаем ТОЛЬКО если знаем предыдущее значение И
            # оно отличается от текущего. ``None`` = «не знаем»
            # (см. _capture_current_voice_mode) — НЕ делаем restore,
            # иначе свапнём dialogue_node в дефолт по своей инициативе.
            if prev_mode is not None and prev_mode != self._agent_during_voice_mode:
                applied_out, reason_out = self._apply_voice_mode(prev_mode)
                if not applied_out:
                    self._log.warning(
                        f"voice_mode_swap.exit: failed to restore mode={prev_mode} " f"reason={reason_out}"
                    )
            elif prev_mode is None:
                self._log.debug("voice_mode_swap.exit: previous mode unknown, no restore")
            # Сбрасываем snapshot: следующий swap начнёт с чистого
            # состояния (``prev_mode`` будет снова захвачен в
            # _on_avatar_command ДО входа в swap).
            self._voice_input_mode_before_swap = None

    def _capture_current_voice_mode(self) -> Optional[str]:
        """Захватить текущий ``voice_input_mode`` dialogue_node для swap.

        Phase 1 (монитор): у нас нет гарантированного способа узнать
        текущее значение (нет GetParameters клиента к dialogue_node).
        Возвращаем ``None`` — swap использует его как «не пытаться
        restore в finally». Phase 2 (active-режим) заменит это на
        настоящий GetParameters.ack.

        Тест-инвариант (AC #8): если LLM бросит исключение после
        capture, finally-ветка _voice_mode_swap НЕ пытается
        восстанавливать ``None — иначе свапнём dialogue_node в дефолт
        по своей инициативе.
        """
        return None

    # ── helpers: harness ─────────────────────────────────────────────────────────────

    def _build_agent_harness_sync(self) -> Any:
        """Создать и ``init()``-нуть OperatorHarness.

        Импорт ленивый: ``rob_box_harness`` — opt dep для
        ``rob_box_supervisor``, в минимальном CI-env может отсутствовать.
        При отсутствии — публикуем ``ok=false, summary="harness_unavailable"``
        в /avatar/command_result (а не валим ноду).

        ``init()`` асинхронный, дрок-вызываем в новом asyncio-цикле
        (см. _run_harness_sync). Это безопасно для тестов с FakeLLMProvider
        (он sync-friendly) и для прод-вызовов в ROS callback (rclpy.spin
        блокирует основной поток; asyncio.run в callback создаёт свой
        loop, отрабатывает, закрывает — но конфликта с rclpy-loop нет,
        т.к. rclpy использует свой собственный event loop в C).
        """
        try:
            from rob_box_harness.config import HarnessConfig  # noqa: PLC0415
            from rob_box_harness.harnesses.operator import OperatorHarness  # noqa: PLC0415
        except ImportError as exc:
            self._log.warning(f"_build_agent_harness_sync: import failed: {exc}")
            return None

        config = HarnessConfig(harness="operator", name="operator_agent")
        harness = OperatorHarness(config, prompt_file=self._system_prompt_file)
        try:
            asyncio.run(harness.init())
        except Exception as exc:  # noqa: BLE001 — ленивый init падать не должен
            self._log.warning(f"_build_agent_harness_sync: harness.init() failed: {exc}")
            return None
        return harness

    def _ensure_agent_harness(self) -> Any:
        """Ленивая инициализация harness (один раз на ноду).

        Тест-инвариант: ``agent_enabled=false`` → harness НЕ создаётся
        (см. _on_avatar_command, где _ensure_agent_harness не
        вызывается). Высжим ``self._agent_harness`` кешем, чтобы на
        каждый /avatar/command не пересоздавать OperatorHarness (это
        дорого — там Tools/LLM/Memory init).
        """
        if self._agent_harness is None:
            self._agent_harness = self._build_agent_harness_sync()
        return self._agent_harness

    def _run_harness_sync(self, harness: Any, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Дрок-вызов async ``harness.step(payload)`` в новом loop.

        Возвращает mapping из ``OperatorHarness.step``. Ошибки harness-а
        (например, LLM exception) превращаем в ``ok=False`` с внятным
        ``summary`` — ADR-0018 в рантайме: лучше честный FAIL, чем
        придуманное действие.

        ``step()`` сам по себе async — оператор HARNESS.step
        (``rob_box_harness/harnesses/operator.py:183``) уже ловит
        LLM-исключения и возвращает ``ok=False, summary="llm_error: ..."``.
        Здесь ловим только исключения самого ``asyncio.run`` (не
        запустился loop) или init-ошибки выше по стеку.
        """
        try:
            result = asyncio.run(harness.run(payload))
        except Exception as exc:  # noqa: BLE001
            return {
                "ok": False,
                "summary": f"llm_error: {type(exc).__name__}",
                "tool_calls": [],
                "error": str(exc),
                "source": payload.get("source", ""),
                "client_id": payload.get("client_id", ""),
            }
        # HarnessRunResult.output — это mapping из step.
        return result.output if hasattr(result, "output") else result

    # ── /avatar/command processing ─────────────────────────────────────────────────

    def _parse_command_payload(self, raw: str) -> dict[str, Any]:
        """Распарсить JSON из ``/avatar/command``.

        Возвращает ``{ok, payload, error}`` — pure-функция для
        тестируемости. ``ok=False`` если JSON битый ИЛИ
        отсутствуют обязательные поля ``source``/``client_id``/``text``.
        Допускаем ``ts_ms`` опциональным (генерируем client-side).
        """
        if not raw:
            return {"ok": False, "error": "empty payload"}
        data = self._try_parse_json(raw)
        if not isinstance(data, dict):
            return {"ok": False, "error": "malformed_input: not a JSON object"}
        text = data.get("text")
        if not isinstance(text, str) or not text.strip():
            return {"ok": False, "error": "malformed_input: missing 'text' field"}
        source = data.get("source")
        if not isinstance(source, str) or not source.strip():
            return {"ok": False, "error": "malformed_input: missing 'source' field"}
        client_id = data.get("client_id", "")
        if not isinstance(client_id, str):
            client_id = str(client_id) if client_id is not None else ""
        ts_ms = data.get("ts_ms")
        return {
            "ok": True,
            "payload": {
                "source": source,
                "client_id": client_id,
                "text": text.strip(),
                "ts_ms": ts_ms,
            },
        }

    def _generate_request_id(self, payload: Mapping[str, Any]) -> str:
        """Сгенерировать ``request_id`` для /avatar/command_result.

        Детерминированно из ``client_id + ts_ms`` если есть (повторная
        обработка той же команды = тот же request_id — удобно для
        дедупликации на клиенте). Иначе — UUID4.
        """
        cid = str(payload.get("client_id", "") or "")
        ts = payload.get("ts_ms")
        if cid and ts is not None:
            return f"{cid}:{ts}"
        return uuid.uuid4().hex

    def _publish_command_result(self, request_id: str, body: Mapping[str, Any]) -> None:
        """Опубликовать результат в /avatar/command_result (std_msgs/String JSON).

        Схема — docs/architecture/avatar-supervisor-agent.md §3.
        ``body`` содержит ``ok``/``summary``/``tool_calls``/опц.
        ``latency_ms``. Невалидный JSON невозможен (мы сами собираем),
        но защищаемся от битых типов: всё приводится к примитивам.
        """
        result = {
            "request_id": request_id,
            "ok": bool(body.get("ok", False)),
            "summary": str(body.get("summary", "")),
            "tool_calls": list(body.get("tool_calls", []) or []),
        }
        if "latency_ms" in body:
            result["latency_ms"] = int(body["latency_ms"])
        try:
            payload = json.dumps(result, ensure_ascii=False)
        except (TypeError, ValueError) as exc:
            self._log.warning(f"_publish_command_result: json.dumps failed: {exc}")
            payload = json.dumps({"request_id": request_id, "ok": False, "summary": "publish_error"})
        msg = RosString()
        msg.data = payload
        self._agent_result_pub.publish(msg)

    # Функция by-design ветвится на: malformed_input / agent_disabled /
    # harness_unavailable / ok / no_tool / llm_error / outer_error.
    # Каждая ветка — короткий блок с побочкой (publish + metric).
    # Извлечение в helper-ы увеличит indirection без выигрыша по
    # читаемости (флоу плоский, не цикл).
    def _on_avatar_command(self, msg: RosString) -> None:  # noqa: C901
        """ROS-callback ``/avatar/command`` — главная точка входа супервизор-агента.

        Поток (см. design.md §5.3):
          1. Парсинг JSON. Битый → публикуем ``malformed_input``, выходим.
          2. Гейт ``agent_enabled``. False → публикуем ``agent_disabled``.
          3. Ленивая инициализация harness (один раз).
          4. ``_voice_mode_swap()`` (try/finally) — личность молчит
             пока мы работаем.
          5. ``harness.step(payload)`` → результат.
          6. Публикация результата в ``/avatar/command_result``.
          7. Метрики.
        """
        started_ns = time.monotonic_ns()
        raw = msg.data or ""
        parsed = self._parse_command_payload(raw)
        if not parsed["ok"]:
            self._log.warning(f"_on_avatar_command: {parsed['error']}")
            self._publish_command_result(
                request_id=uuid.uuid4().hex,
                body={"ok": False, "summary": "malformed_input", "tool_calls": []},
            )
            self._record_agent_command(source="unknown", result="malformed_input")
            return

        payload = parsed["payload"]
        source = payload["source"]
        request_id = self._generate_request_id(payload)

        if not self._agent_enabled:
            self._publish_command_result(
                request_id=request_id,
                body={"ok": False, "summary": "agent_disabled", "tool_calls": []},
            )
            self._record_agent_command(source=source, result="agent_disabled")
            return

        if source not in AGENT_COMMAND_SOURCES:
            # Не блокируем (расширяемость — AV-22 добавит "web"/"admin"),
            # но логируем — чтобы оператор видел «незнакомый источник».
            self._log.warning(
                f"_on_avatar_command: unknown source={source!r} (expected one of "
                f"{AGENT_COMMAND_SOURCES}); processing anyway"
            )

        harness = self._ensure_agent_harness()
        if harness is None:
            self._publish_command_result(
                request_id=request_id,
                body={"ok": False, "summary": "harness_unavailable", "tool_calls": []},
            )
            self._record_agent_command(source=source, result="harness_unavailable")
            return

        # Снимок «текущего» voice_input_mode ДО swap.apply — нужно
        # для finally-восстановления. В Phase 1 это всегда "unknown"
        # (см. _capture_current_voice_mode), в active-режиме Phase 2
        # заменит на настоящий GetParameters.
        self._voice_input_mode_before_swap = self._capture_current_voice_mode()

        try:
            with self._voice_mode_swap():
                result = self._run_harness_sync(harness, payload)
        except Exception as exc:  # noqa: BLE001 — НЕ ДОЛЖНО сбежать из swap
            # ``_voice_mode_swap`` имеет try/finally, но защищаемся от
            # ошибок ВНЕ swap (publish, метрики). Сам swap уже
            # восстановил voice_input_mode.
            self._log.warning(f"_on_avatar_command: outer exception: {exc}")
            result = {
                "ok": False,
                "summary": f"outer_error: {type(exc).__name__}",
                "tool_calls": [],
            }

        # Нормализуем tool_calls (OperatorHarness.step уже возвращает
        # dict-ы; дополнительно — записываем метрики на каждый tool).
        tool_calls = result.get("tool_calls", []) or []
        for tc in tool_calls:
            name = tc.get("name") if isinstance(tc, dict) else None
            if isinstance(name, str) and name:
                self._record_agent_tool_call(name)

        latency_ms = int((time.monotonic_ns() - started_ns) / 1_000_000)
        self._publish_command_result(
            request_id=request_id,
            body={
                "ok": bool(result.get("ok", False)),
                "summary": str(result.get("summary", "")),
                "tool_calls": tool_calls,
                "latency_ms": latency_ms,
            },
        )

        result_label = "ok" if result.get("ok", False) else "error"
        if not result.get("ok", False):
            # Различаем «no_tool» от «error» — иначе в метрике сольются
            # два разных operational-сигнала.
            summary = str(result.get("summary", ""))
            if summary.startswith("no_tool"):
                result_label = "no_tool"
            elif summary.startswith("llm_error"):
                result_label = "llm_error"
        self._record_agent_command(
            source=source,
            result=result_label,
            latency_s=(time.monotonic_ns() - started_ns) / 1_000_000_000.0,
        )

    # ── /avatar/command: pure-logic wrapper (тестируется без rclpy) ─────────

    def _handle_avatar_command_logic(
        self,
        payload: Mapping[str, Any],
        *,
        harness_factory: Any = None,
    ) -> dict[str, Any]:
        """Pure-логика «получили payload → вернули result».

        Без side-effects (не публикует в ROS, не меняет voice_mode,
        не дёргает метрики — это делает _on_avatar_command в
        try/finally-disciplined fashion). Используется для unit-тестов
        с подменой harness через ``harness_factory``.

        ``harness_factory`` — callable(payload) → mapping c
        ``ok``/``summary``/``tool_calls``. По умолчанию используется
        ``self._agent_harness`` (если уже создан) или возвращается
        ``ok=False, summary="harness_unavailable"``.

        Тест-инвариант: если harness.step бросит исключение —
        возвращаем ``ok=False, summary="llm_error: <type>"`` вместо
        propagate (см. AC #7 — не выдумываем действий).
        """
        if not self._agent_enabled:
            return {"ok": False, "summary": "agent_disabled", "tool_calls": []}
        if harness_factory is not None:
            try:
                return harness_factory(payload)
            except Exception as exc:  # noqa: BLE001
                return {
                    "ok": False,
                    "summary": f"llm_error: {type(exc).__name__}",
                    "tool_calls": [],
                    "error": str(exc),
                }
        harness = self._agent_harness
        if harness is None:
            return {"ok": False, "summary": "harness_unavailable", "tool_calls": []}
        return self._run_harness_sync(harness, payload)


class _NoopLabelCounter:
    """Заглушка для метрик при отсутствии ``rob_box_voice`` (минимальный CI-env).

    Имеет те же ``labels(...).inc()``, что и prometheus Counter, но
    ничего не считает. Это позволяет ``self._agent_metrics`` быть
    всегда dict-ом, без ``if rob_box_voice is not None`` в каждом
    методе record_*.

    ``labels`` — обычный атрибут (не слот), чтобы unit-тесты могли
    подменить его на spy и перехватить вызовы ``.inc()``.
    """

    def labels(self, *args: Any, **kwargs: Any) -> "_NoopLabelCounter":
        return self

    def inc(self, amount: float = 1.0) -> None:
        return None


class _NoopHistogram:
    """Заглушка для гистограммы latency при отсутствии ``rob_box_voice``.

    Аналогично ``_NoopLabelCounter`` — ``labels`` не слот, чтобы тесты
    могли его подменить.
    """

    def observe(self, amount: float) -> None:
        return None

    def labels(self, *args: Any, **kwargs: Any) -> "_NoopHistogram":
        return self


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
