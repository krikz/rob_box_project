"""Проводка ``dialog_node`` v2 для трёх функций v1 (#3912, решение Шифу 23.10 D1, карта PR-15 §3/§7).

Логика — в ``rob_box_dialog`` (чистый Python): ``meeting`` (приветствие по встрече лицом), ``startup`` (стартовая
фраза), ``status_feed`` (статус LLM/вейков для ТАРС). Здесь — только ROS: подписка, таймеры, latched-паблишеры,
речь через владельца речи (``V2Ports.say`` → ``SpeechRequest``). v1 ``dialogue_node`` берёт отсюда же
``start_status_feeds`` и ``catalog_model`` — реализация одна. Функции, а не методы: ``DialogNode`` — тонкий хост.

Окно ответа на приветствие (#3776) — тот же ``core.greeting_window``, что у v1: кадр ``GreetingReply`` сессии.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional, Tuple

from rob_box_core import expression_client
from rob_box_dialog import knowledge as kn
from rob_box_dialog.meeting import MeetingGreeter, MeetingMarker, greet_on_meeting, parse_meeting_marker
from rob_box_dialog.meeting import unprompted_quiet
from rob_box_dialog.shift_log import FACE
from rob_box_dialog.startup import NodeReport, Readiness, ReadinessWatch, assess, llm_usable, pick_startup_phrase
from rob_box_dialog.status_feed import StatusFeed, describe_chain, describe_wake

from .greeting_window import open_window

_AGENT = kn.AGENTS[0]


# -- фид статуса ТАРС ----------------------------------------------------------------------------------------------

def catalog_model(name: str) -> Optional[str]:
    """Модель, с которой ``build_provider(name)`` собирает провайдера; каталога нет — ``None`` (ключа не будет)."""
    try:
        from rob_box_harness.providers.catalog import LLM_PROVIDER_REGISTRY  # noqa: PLC0415
    except ImportError:
        return None
    model = (LLM_PROVIDER_REGISTRY.get(name) or {}).get("default_model")
    return model if isinstance(model, str) and model else None


def start_status_feeds(node: Any, string_cls: Any, llm_payload: Optional[dict],
                       wake_payload: Optional[dict]) -> Tuple[StatusFeed, StatusFeed]:
    """Latched-паблишеры ``/voice/llm_status`` и ``/voice/wake_words`` (TRANSIENT_LOCAL, depth 1); сразу отдать
    значения — поздний подписчик (quest_node → шлем) их получит."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

    def make_msg(data: str) -> Any:
        msg = string_cls()
        msg.data = data
        return msg

    llm_feed = StatusFeed(node.create_publisher(string_cls, "/voice/llm_status", qos), make_msg)
    wake_feed = StatusFeed(node.create_publisher(string_cls, "/voice/wake_words", qos), make_msg)
    llm_feed.publish(llm_payload)
    wake_feed.publish(wake_payload)
    return llm_feed, wake_feed


def start_v2_status_feeds(node: Any, string_cls: Any) -> Tuple[StatusFeed, StatusFeed]:
    """v2: настроенная цепочка Личности (параметр ``llm_providers``) и вейки Личности (``address.wake_words``)."""
    chain = str(node.get_parameter("llm_providers").value or "").split(",")
    return start_status_feeds(node, string_cls, describe_chain(chain, catalog_model), describe_wake(node._wake))


# -- встреча лицом ---------------------------------------------------------------------------------------------------

def turn_busy(node: Any) -> bool:
    """Идёт ход Личности (в полёте или ждёт очереди)."""
    return len(node._turns) > 0 or node._turn_lock.locked()


def subscribe_meeting(node: Any) -> bool:
    """Подписка на ``/vision/hailo/events`` — источник Встречи. Импорт сообщения защищён, как в v1: без лицевых
    сообщений в образе диалог работает, робот просто не здоровается первым."""
    node._meeting_greeter = MeetingGreeter()
    try:
        from rob_box_perception_msgs.msg import VisionEvent  # noqa: PLC0415

        node.create_subscription(VisionEvent, "/vision/hailo/events", lambda msg: on_vision_event(node, msg), 10)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f"⚠️ [встреча] лицевой повод недоступен ({exc!r}): робот не здоровается первым")
        return False
    node.get_logger().info("👁 [встреча] подписка на /vision/hailo/events — приветствие знакомого после разлуки")
    return True


def on_vision_event(node: Any, msg: Any) -> Optional[str]:
    """Маркер начала Встречи → приход в журнал смены (#4076: имя лица или незнакомое) и не больше одной фразы (правила
    — ``rob_box_dialog.meeting``). Колбэк горячий (~5 событий в секунду на человека): всё, кроме маркера, отсеивается
    разбором. Ошибка не роняет колбэк."""
    marker = parse_meeting_marker(event_type=getattr(msg, "event_type", "") or "",
                                  attributes_json=getattr(msg, "attributes_json", "") or "",
                                  source_camera=getattr(msg, "source_camera", "") or "")
    if marker is None:
        return None
    node._shift.seen(marker.name if marker.is_named else None, FACE)
    try:
        return _greet(node, marker)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f"⚠️ [встреча] приветствие упало: {exc!r}")
        return None


def _greet(node: Any, marker: MeetingMarker) -> Optional[str]:
    spoken = {}

    def speak(text: str) -> None:
        spoken["speech_id"] = node._ports.say("", node._session.snapshot.epoch, text, emotion="happy")

    quiet = unprompted_quiet(node._session.snapshot, turn_busy(node))
    phrase = greet_on_meeting(node._meeting_greeter, marker, idle=not quiet, allowed=lambda _m: True, speak=speak)
    if phrase:
        # #3776: после приветствия окно 15 с — «привет» в ответ без «робот» (кадр ждёт конца речи).
        open_window(node, marker.person_id, marker.name.strip(), str(spoken.get("speech_id") or ""))
        node.get_logger().info(f"🗣 [встреча] {marker.name} (person={marker.person_id[:8]}): {phrase!r}")
    return phrase


# -- готовность узлов (#3913) --------------------------------------------------------------------------------------

#: Топик отчёта монитора узлов контекстного агрегатора (Main Pi) - единственный источник «что поднялось».
READINESS_TOPIC = "/perception/context_update"


def subscribe_readiness(node: Any) -> ReadinessWatch:
    """Подписка на отчёт монитора узлов; импорт сообщения защищён, как у встречи: нет сообщения - отчёта не будет, и
    стартовая фраза честно скажет «не знаю», а не «всё в норме»."""
    watch = ReadinessWatch()
    try:
        from rob_box_perception_msgs.msg import PerceptionEvent  # noqa: PLC0415

        node.create_subscription(PerceptionEvent, READINESS_TOPIC, lambda msg: on_node_report(watch, msg), 10)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f"⚠️ [старт] отчёт о готовности узлов недоступен ({exc!r}): фраза скажет «не знаю»")
    return watch


def on_node_report(watch: ReadinessWatch, msg: Any, clock: Callable[[], float] = time.monotonic) -> None:
    """Сообщение агрегатора -> последний отчёт (колбэк горячий: только три списка, остальное в сообщении не читаем)."""
    watch.observe(NodeReport.from_lists(getattr(msg, "active_nodes", ()) or (), getattr(msg, "failed_nodes", ()) or (),
                                        getattr(msg, "missing_nodes", ()) or ()), clock())


# -- стартовая фраза -----------------------------------------------------------------------------------------------

class StartupGreeting:
    """Одноразовая фраза после старта (#1003, порядок v1 + #3913): ждём ``startup_greeting_sec`` → ждём готовности узлов
    (опрос раз в ``STARTUP_READY_POLL_S``, не дольше ``STARTUP_READY_TIMEOUT_S`` от старта) → если робот не занят —
    earcon ``heard`` → 2 с → earcon ``done`` → 1,5 с → фраза по вердикту готовности (``rob_box_dialog.startup``).
    Занят (ход, тишина, удержание) - пропуск, без повтора."""

    def __init__(self, node: Any, delay_s: float, text: str, quiet: Callable[[], bool],
                 readiness: Callable[[], Readiness], clock: Callable[[], float] = time.monotonic) -> None:
        self._node, self._text, self._quiet, self._readiness, self._clock = node, text, quiet, readiness, clock
        self._deadline = clock() + max(delay_s, kn.STARTUP_READY_TIMEOUT_S)
        self._fired = False
        self.phrase: Optional[str] = None
        self.verdict: Optional[str] = None
        self._timer = node.create_timer(delay_s, self._start)

    def _next(self, delay_s: float, step: Callable[[], None]) -> None:
        self._timer.cancel()
        self._timer = self._node.create_timer(delay_s, step)

    def _earcon(self, earcon: str) -> None:
        expression_client.express(self._node, turn_id="", epoch=self._node._session.snapshot.epoch, agent=_AGENT,
                                  earcon=earcon)

    def _start(self) -> None:
        self._timer.cancel()
        if self._fired:
            return
        self._fired = True
        self._poll()

    def _poll(self) -> None:
        """Тихо - пропуск; всё поднялось или время вышло - озвучиваем; иначе ещё секунду ждём."""
        if self._quiet():
            self._node.get_logger().info("Startup greeting: диалог активен — пропускаю приветствие")
            return
        ready = assess(self._readiness()).kind == "ready"
        if not ready and self._clock() < self._deadline:
            self._next(kn.STARTUP_READY_POLL_S, self._poll_tick)
            return
        self._earcon("heard")
        self._next(kn.STARTUP_GREETING_STEPS_S[0], self._done)

    def _poll_tick(self) -> None:
        self._timer.cancel()
        self._poll()

    def _done(self) -> None:
        self._earcon("done")
        self._next(kn.STARTUP_GREETING_STEPS_S[1], self._speak)

    def _speak(self) -> None:
        self._timer.cancel()
        self.phrase, verdict = pick_startup_phrase(self._readiness(), self._text)
        self.verdict = verdict.kind
        self._node.get_logger().info(f"🗣 Startup greeting [{verdict.kind}, не поднялось={list(verdict.down)}]: "
                                     f"{self.phrase!r}")
        self._node._ports.say("", self._node._session.snapshot.epoch, self.phrase)


def arm_startup_greeting(node: Any) -> Optional[StartupGreeting]:
    """Параметры ``startup_greeting_sec`` (0 — выключено) и ``startup_greeting_text`` (пусто — фраза по готовности)."""
    delay = float(node.get_parameter("startup_greeting_sec").value or 0.0)
    if delay <= 0:
        return None
    node.get_logger().info(f"🗣 Startup greeting через {delay:.0f}s (после готовности узлов, не позже "
                           f"{kn.STARTUP_READY_TIMEOUT_S:.0f}s)")
    watch = subscribe_readiness(node)
    return StartupGreeting(node, delay, str(node.get_parameter("startup_greeting_text").value or ""),
                           lambda: unprompted_quiet(node._session.snapshot, turn_busy(node)),
                           lambda: watch.snapshot(time.monotonic(), llm_usable(node._reporter.all_states())))


__all__ = ["READINESS_TOPIC", "StartupGreeting", "arm_startup_greeting", "catalog_model", "on_node_report",
           "on_vision_event", "start_status_feeds", "start_v2_status_feeds", "subscribe_meeting", "subscribe_readiness",
           "turn_busy"]
