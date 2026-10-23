"""dialog_node — тонкий ROS-хост диалога v2 (ADR-0150 §2.2, §3; PR-8 ч.8e2 #3702).

Стартует только при ``dialog_engine: v2`` (launch, ``rob_box_core.dialog_engine``); при ``v1`` работает
``dialogue_node``. Вся логика хода — ``rob_box_dialog`` (``address → grammar → context → LLM → command → Executor →
respond``, :class:`rob_box_dialog.turn.TurnEngine`); транспорт — ``core.dialog_v2_ports`` (дорога исполнения v1:
LLM-цепочка с мостом команд, MCP-тулы под планировщиком ресурсов). Здесь — только ROS:

* подписки: ``/voice/stt/utterance`` (реплика → адресация → ход), ``/voice/speaker/result`` (собеседник),
  ``/voice/speech/event`` (окно озвученного, план, «говорю»), ``/voice/sound/event`` (план), ``/voice/music/state`` и
  ``/voice/music/event`` (снимок плеера, подложка выступления), ``/voice/providers/state`` (доска провайдеров),
  ``/dialog/interrupt_request`` (ясный стоп рвёт речь эпохи и ход в полёте), сервис ``/dialog/control``
  (hold/resume/reset), ``/vision/hailo/events`` (встреча лицом, ``core.dialog_v2_parity``, #3912);
* публикации: latched ``/dialog/session``, ``/dialog/turn_event`` и рефлексы, ``/dialog/intent``, ``/dialog/plan_event``
  (через ``plan_host``), речь — ``speech_client``; ``ActionRequest`` не публикуется (подписчика нет, см. ports);
* latched ``/voice/llm_status`` и ``/voice/wake_words`` (экран ТАРС, #3912);
* таймеры: стартовая фраза (один раз), TTL кадров сцен (1 с), переиздание состояния LLM-провайдеров (2 с);
  дедлайн хода — в ``turn.py``.

Телеграм (PR-12a #3706, ADR-0150 §9.2): реплика ``Utterance source=telegram`` — не вход Личности, а ход
телеграм-агента своего чата (``rob_box_dialog.chats``: сессия ``telegram:<chat_id>``, своя история и LLM-цепочка
``telegram_*``); ответ и фразы об исходе — ``DialogOutput`` в ``/dialog/output`` (его читает ``telegram_node``).

Перебивание (PR-9 #3703, ``rob_box_dialog.interrupt``): ясный стоп — с ``/dialog/interrupt_request`` (ранний
сегмент STT) или с адресованной реплики — отменяет речь эпохи (граница по слову) и все хода в полёте: отмена задачи
хода доходит до ``await`` LLM-провайдера, HTTP-запрос закрывает его клиент. Ход стопа (Tier-1) кладёт эпоху + 1.
Не стоп во время хода — новый ход ждёт конца текущего (APPEND), речь не рвётся.

Текст ходов на диск — только в e2e-БД вне ``/data`` (непустой ``e2e_db_path``, ``rob_box_dialog.e2e_events``; #3904).
"""

from __future__ import annotations

import asyncio
import functools
import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from rob_box_core import dialog_engine, speech_client
from rob_box_core.e2e_mode import E2EModeFollower
from rob_box_core.provider_publisher import make_publisher, publish_provider_states
from rob_box_core.speaker_wire import speaker_result_to_payload
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import BARE_WAKE, HELD, Addressed, NotAddressed, decide_in_session, heard_from
from rob_box_dialog.address import log_label
from rob_box_dialog.address import person_ref_from
from rob_box_dialog.address import wake_words as load_wake_words
from rob_box_dialog.backlog import Backlog
from rob_box_dialog.chats import ChatPorts, ChatTurns, channel_ref
from rob_box_dialog.e2e_events import open_events
from rob_box_dialog.interrupt import TurnsInFlight, on_interrupt
from rob_box_dialog.memory import profiled_person
from rob_box_dialog.scenes import HOLD, RESET, RESUME, SceneDirector, apply_control
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.speaker_adapter import speaker_ref
from rob_box_dialog.turn import TurnEngine, host_sections, say_late
from rob_box_dialog_msgs.msg import DialogOutput
from rob_box_dialog_msgs.msg import Intent as IntentMsg
from rob_box_dialog_msgs.msg import InterruptRequest, ProviderStateArray, SessionState, SoundEvent, SpeakerResult
from rob_box_dialog_msgs.msg import SpeechEvent
from rob_box_dialog_msgs.msg import Utterance as UtteranceMsg
from rob_box_dialog_msgs.srv import DialogControl
from rob_box_llm.provider import LLMSettings
from rob_box_mcp_tools.voice_request import REQUEST_TOPIC as VOICE_REQUEST_TOPIC

from .core import greeting_window, identity_question
from .core import dialog_v2_parity as parity
from .core import plan_host
from .core.dialog_v2_ports import CallCounter, V2Ports, assemble_llm
from .core.music_player_state import MUSIC_EVENT_TOPIC, MUSIC_STATE_TOPIC, parse_music_state
from .core.provider_reporter import ProviderStateReporter, on_provider_states, reporter_context
from .core.speaker_memory import SpeakerMemory
from .core.speaker_turn import note_backlog_voice
from .core.utterance_speaker import UtteranceSpeakerRegistry
from .tts_voice_registry import robot_default_voice
from rob_box_dialog.music_state import MusicStateMemory

#: ``SpeechEvent.kind`` конца реплики: finished / cancelled / rejected.
_ENDED = (SpeechEvent.KIND_FINISHED, SpeechEvent.KIND_CANCELLED, SpeechEvent.KIND_REJECTED)
#: ``DialogControl.Request.ACTION_*`` -> действие :func:`rob_box_dialog.scenes.apply_control`.
_CONTROL_ACTIONS = {DialogControl.Request.ACTION_HOLD: HOLD, DialogControl.Request.ACTION_RESUME: RESUME,
                    DialogControl.Request.ACTION_RESET: RESET}
_LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def declare_params(node: Any) -> None:
    """Параметры ноды (``dialog_node.yaml``; ключи API — из окружения, как у ``dialogue_node``)."""
    node.declare_parameter("llm_providers", "deepseek,minimax")
    node.declare_parameter("system_prompt_file", "master_prompt_compact.txt")
    node.declare_parameter("wake_words_path", "/config/wake_words.yaml")
    node.declare_parameter("temperature", 0.0)
    node.declare_parameter("max_tokens", 0)
    # Номинальный провайдер TTS, пока tts_node не сказал фактический (паузы MiniMax выступления, #3901).
    node.declare_parameter("tts_provider", "minimax")
    node.declare_parameter("e2e_db_path", "")
    # Телеграм-агент (§9.2, PR-12a): своя цепочка и персона; значения — бывшие мёртвые llm_* telegram_bot.yaml (V27).
    node.declare_parameter("telegram_llm_providers", "deepseek,minimax")
    node.declare_parameter("telegram_system_prompt_file", "telegram_chat_prompt.txt")
    node.declare_parameter("telegram_temperature", 0.7)
    node.declare_parameter("telegram_max_tokens", 0)
    # Стартовая фраза (#1003, перенос v1 #3912): через N с после старта; 0 — выкл; пустой текст — случайная фраза.
    node.declare_parameter("startup_greeting_sec", kn.STARTUP_GREETING_DELAY_S)
    node.declare_parameter("startup_greeting_text", "")
    # «Робот помнит собеседника» (#3919, core/speaker_memory.py): профиль - harness_voice.db (шов note_seen, как у v1),
    # факты - voice_facts из voice_memory.db (куда пишет memory_save); в e2e - e2e-БД фактов (как e2e_db_path MCP).
    # Пустой путь - этой части нет.
    node.declare_parameter("sqlite_db_path", "")
    node.declare_parameter("voice_facts_db_path", "")
    node.declare_parameter("e2e_voice_facts_db_path", "/data/voice_memory.e2e.db")


def _prefix(agent: str) -> str:
    """Префикс параметров агента: у Личности его нет (``llm_providers``), у телеграма — ``telegram_``."""
    return "" if agent == kn.AGENTS[0] else f"{agent}_"


def _settings(node: Node, prefix: str = "") -> Optional[LLMSettings]:
    temperature = float(node.get_parameter(f"{prefix}temperature").value or 0.0)
    max_tokens = int(node.get_parameter(f"{prefix}max_tokens").value or 0)
    if temperature <= 0 and max_tokens <= 0:
        return None
    return LLMSettings(temperature=temperature or None, max_tokens=max_tokens or None)


def _prompt_dir() -> Path:
    from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

    return Path(get_package_share_directory("rob_box_voice")) / "prompts"


def build_llm(node: Any, counter: CallCounter, agent: str = kn.AGENTS[0]) -> tuple:
    """LLM-цепочка v2 агента: тот же ``build_llm_chain``, все провайдеры — в режиме команд моста (валидатор знает
    агента), вызовы считаются. У каждого агента своя цепочка — своё здоровье провайдеров."""
    prefix = _prefix(agent)
    raw = str(node.get_parameter(f"{prefix}llm_providers").value)
    return assemble_llm(raw.split(","), counter, prompt_dir=_prompt_dir(),
                        system_prompt_file=str(node.get_parameter(f"{prefix}system_prompt_file").value),
                        settings=_settings(node, prefix), agent=agent, log=node.get_logger().info,
                        busy=lambda: plan_host.perform_busy(node))


def _mcp_provider(node: Any, sender: str, for_telegram: bool = False) -> Any:
    """Провайдер MCP-тулов под подписью ``sender`` (``LLMToolCallAdapter`` → ``ROSMCPToolProvider``)."""
    from rob_box_harness.core.tool_registry import ToolRegistry  # noqa: PLC0415
    from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider  # noqa: PLC0415
    from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: PLC0415

    adapter = LLMToolCallAdapter(node, sender=sender, turn_context=lambda: {"utterance_id": node.utterance_id})
    provider = ROSMCPToolProvider(adapter)
    provider.update_tools([{"type": "function", "function": {"name": s.name, "description": s.description,
                                                             "parameters": dict(s.parameters)}}
                           for s in ToolRegistry(for_telegram=for_telegram).list_tools()])
    if not provider.list_tools():
        raise RuntimeError("dialog_node: каталог MCP-тулов пуст — команды молча не исполнялись бы")
    return adapt_tool_provider(provider)


def build_tools(node: Any) -> Any:
    """MCP-тулы Личности под планировщиком ресурсов — как у v1. Подпись и срез — личности, а не по имени ROS-ноды
    ``dialog_node`` (#3853)."""
    from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER  # noqa: PLC0415

    return plan_host.make_tool_executor(node, _mcp_provider(node, PERSONALITY_SENDER), lambda *_: None)


def build_chat_tools(node: Any, personality_tools: Any) -> Any:
    """MCP-тулы агента чата: своя подпись (срез ``telegram`` с ``say_aloud``), планировщик ресурсов — общий с
    Личностью (одни владельцы, одна занятость); хост выступления остаётся у Личности."""
    from rob_box_mcp_tools.mcp_auth import TELEGRAM_SENDER  # noqa: PLC0415
    from rob_box_voice.core.plan_tools import PlanToolExecutor  # noqa: PLC0415

    return PlanToolExecutor(_mcp_provider(node, TELEGRAM_SENDER, True), personality_tools.planner)


def build_chats(node: Any, tools: Any, intent_pub: Any, wake: Any, hold: Any = lambda: False) -> ChatTurns:
    """Телеграм-агент: своя LLM-цепочка, исполнитель — свой под общим планировщиком Личности (те же владельцы),
    ответ — ``DialogOutput``; ``hold`` — держит ли оператор Личность (отказ ``say_aloud``, §9.2.3)."""
    counter = CallCounter()
    llm, system = build_llm(node, counter, kn.TEXT_CHANNEL_AGENT)
    pub = node.create_publisher(DialogOutput, "/dialog/output", 10)
    log = node.get_logger().info

    def send(ref: str, text: str, speech_id: str) -> None:
        msg = DialogOutput()
        msg.agent, msg.channel_ref, msg.text, msg.speech_id = kn.TEXT_CHANNEL_AGENT, ref, text, speech_id
        pub.publish(msg)

    chat_tools = build_chat_tools(node, tools)

    def make_engine(chat_id: int) -> TurnEngine:
        session = Session(kn.TEXT_CHANNEL_AGENT, clock=time.time, session_id=channel_ref(chat_id))
        base = V2Ports(node, llm, counter, chat_tools, agent=kn.TEXT_CHANNEL_AGENT, intent_pub=intent_pub, log=log,
                       epoch=lambda: session.snapshot.epoch)
        ports = ChatPorts(base, chat_id, send, log, hold)
        base.on_late = ports.late
        chats.adopt(base.executor)
        return TurnEngine(session, ports, system_prompt=system, deadline_s=kn.CHAT_TURN_DEADLINE_S,
                          log=_RosLog(node.get_logger()))

    chats = ChatTurns(make_engine, wake, log)
    return chats


class DialogNode(Node):
    """ROS-хост хода v2: подписки → ``TurnEngine``, порты → топики."""

    def __init__(self) -> None:
        super().__init__("dialog_node")
        declare_params(self)
        self.engine_flag = dialog_engine.declare(self)
        self.utterance_id: Optional[str] = None
        self.last_turn: Any = None  # concurrent.futures.Future последнего хода (журнал, тесты)
        self.last_chat_turn: Any = None  # то же для хода телеграм-агента
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, name="dialog_v2_loop")
        self._turn_lock = asyncio.Lock()
        self._turns = TurnsInFlight()
        self._session_pub = self.create_publisher(SessionState, identity_question.SESSION_TOPIC, _LATCHED)
        self._session = Session("personality", clock=time.time)
        self._session.subscribe(functools.partial(identity_question.publish, self))
        self._voice_pub = self.create_publisher(String, VOICE_REQUEST_TOPIC, 10)
        self._scenes = SceneDirector(self._session, self._restore_voice, robot_default_voice())
        self._wake = load_wake_words(str(self.get_parameter("wake_words_path").value))[0]
        self._backlog = Backlog()
        self._speakers = UtteranceSpeakerRegistry()
        self._music = MusicStateMemory()
        self._speaker_memory = SpeakerMemory(
            str(self.get_parameter("sqlite_db_path").value), str(self.get_parameter("voice_facts_db_path").value),
            e2e_facts_db=str(self.get_parameter("e2e_voice_facts_db_path").value),
            is_e2e=lambda: self._e2e_follower.applied, log=self.get_logger())
        # #3904: режим e2e один - владельца /speaker_id_node; профиль собеседника в e2e не идёт в боевую БД (#3919).
        self._e2e_follower = E2EModeFollower(self, self._apply_e2e_mode)
        self._turn_memory: Any = (None, None)  # (person_ref, раздел speaker_memory) хода в работе
        self._music_player_state: Any = None
        self._actual_tts_provider: Optional[str] = None  # из /voice/tts/provider_state (plan_host)
        counter = CallCounter()
        llm, system = build_llm(self, counter)
        self._tool_executor = build_tools(self)
        intent_pub = self.create_publisher(IntentMsg, "/dialog/intent", 10)
        self._chats = build_chats(self, self._tool_executor, intent_pub, self._wake,
                                  hold=lambda: self._session.snapshot.top("OperatorHold") is not None)
        self._ports = V2Ports(self, llm, counter, self._tool_executor, intent_pub=intent_pub,
                              log=self.get_logger().info, epoch=lambda: self._session.snapshot.epoch,
                              performer=functools.partial(plan_host.performer, self))
        self._reporter = ProviderStateReporter(llm, functools.partial(publish_provider_states, make_publisher(self)),
                                               self._speak_direct)
        self._engine = TurnEngine(self._session, self._ports, system_prompt=system, sections=self._sections,
                                  events=self._open_events(), log=_RosLog(self.get_logger()))
        self._ports.on_late = functools.partial(say_late, self._engine)  # поздний старт музыки - фраза кода (#3960)
        self._subscribe()
        # Паритет с v1 (#3912): статус LLM/вейков для ТАРС, приветствие по встрече лицом, стартовая фраза.
        self._tars_status_feeds = parity.start_v2_status_feeds(self, String)
        parity.subscribe_meeting(self)
        self._startup_greeting = parity.arm_startup_greeting(self)
        self.create_service(DialogControl, "/dialog/control", self._on_control)
        self.create_timer(1.0, self._session.expire)
        self.create_timer(5.0, self._chats.tick)
        self.create_timer(2.0, self._reporter.tick)
        self.add_on_set_parameters_callback(self._on_params)
        self._thread.start()
        self.get_logger().info(f"✅ dialog_node v2: {kn.AGENTS[0]}, dialog_engine={self.engine_flag}, "
                               f"wake={self._wake[:3]}…, e2e={self._engine_events_state()}")

    def _subscribe(self) -> None:
        self.create_subscription(UtteranceMsg, "/voice/stt/utterance", self._on_utterance, 10)
        self.create_subscription(SpeakerResult, "/voice/speaker/result", self._on_speaker, 10)
        self.create_subscription(SpeechEvent, "/voice/speech/event", self._on_speech_event, 10)
        self.create_subscription(SoundEvent, "/voice/sound/event", functools.partial(plan_host.on_sound, self), 10)
        self.create_subscription(String, MUSIC_STATE_TOPIC, self._on_music_state, _LATCHED)
        self.create_subscription(String, MUSIC_EVENT_TOPIC, functools.partial(plan_host.on_music_event, self), 10)
        self.create_subscription(String, plan_host.TTS_PROVIDER_STATE_TOPIC,
                                 functools.partial(plan_host.on_tts_provider_state, self), 10)
        self.create_subscription(ProviderStateArray, "/voice/providers/state",
                                 functools.partial(on_provider_states, self._reporter.board, time.time), _LATCHED)
        self.create_subscription(InterruptRequest, "/dialog/interrupt_request", self._on_interrupt, 10)

    # -- вход ---------------------------------------------------------------------------------------------------

    def _on_utterance(self, msg: Any) -> None:
        """Реплика: адресация кодом (вейк, бэклог, окна ответа); адресованная — ход в asyncio-loop ноды."""
        now = time.time()
        self.get_logger().info(f"🎤 STT: {log_label(msg)}")
        if msg.source == UtteranceMsg.SOURCE_TELEGRAM and msg.chat_id:
            self._on_chat(int(msg.chat_id), msg.text, now)
            return
        uid = str(getattr(msg, "utterance_id", "") or "")
        heard = heard_from(msg, now=now, person_ref=person_ref_from(self._speakers, uid))
        snap = self._session.snapshot
        decision = decide_in_session(heard, self._backlog, self._wake, snap, greeting=greeting_window.window(self))
        if isinstance(decision, NotAddressed) and decision.reason == HELD:
            self.get_logger().info("[hold] оператор держит личность — реплика не слушается")
            return
        if isinstance(decision, NotAddressed):
            greeting_window.not_a_reply(self, heard)  # окно после приветствия открыто, а фраза не ответ (#3776)
            self._backlog.add(heard)
            voice = self._speakers.peek(uid) if uid else None
            if voice is not None:  # биометрия успела раньше STT: автор фразы — её голос, не нынешний собеседник
                note_backlog_voice(self, uid, voice)
            self.get_logger().info(f"[backlog] accumulated reason={decision.reason} entries="
                                   f"{self._backlog.background_phrases(now)}")
            return
        greeting_window.take_reply(self, decision.source, heard)  # окно после приветствия снимает первая фраза
        self.utterance_id = uid or None
        self._interrupt(decision.text, uid, "utterance")
        self.last_turn = self._turns.add(asyncio.run_coroutine_threadsafe(self._turn(decision), self._loop), uid)

    def _on_chat(self, chat_id: int, text: str, now: float) -> None:
        """Реплика чата — ход телеграм-агента этого чата (не Личности); «Клод …» — никому (координатору)."""
        decision = self._chats.decide(chat_id, text, now)
        if isinstance(decision, Addressed):
            self.last_chat_turn = asyncio.run_coroutine_threadsafe(self._chats.run(chat_id, decision), self._loop)

    async def _turn(self, decision: Addressed) -> None:
        if self._turn_lock.locked():  # не стоп во время хода — APPEND: новый ждёт конца текущего, речь не рвётся
            self.get_logger().info("🧭 [turn-v2] ход уже идёт — новый ждёт его конца")
        async with self._turn_lock:
            try:
                await self._note_speaker(decision)
                await self._engine.run(decision)
            except Exception as exc:  # noqa: BLE001 — упавший ход не роняет ноду, но громко
                self.get_logger().error(f"🧭 [turn-v2] ход упал: {type(exc).__name__}: {exc}")

    def _apply_e2e_mode(self, enabled: bool) -> bool:
        """Режим владельца применён: на входе в e2e профиль e2e-собеседников начинается с нуля (счёт диалогов акта)."""
        if enabled:
            self._speaker_memory.reset_e2e()
        return True

    async def _note_speaker(self, decision: Addressed) -> None:
        """Начало хода: профиль подтверждённого собеседника ведёт код, раздел ``speaker_memory`` - в снимок (#3919)."""
        speaker = self._session.snapshot.speaker
        counts = decision.source != BARE_WAKE and bool(decision.text.strip())  # пустой ход - не диалог (как у v1)
        section = await self._speaker_memory.section(speaker) if counts else None
        self._turn_memory = (profiled_person(speaker), section)

    def _on_speaker(self, msg: Any) -> None:
        """Биометрия фразы: реестр по ``utterance_id`` и собеседник сессии (имя — только при ``confirmed``)."""
        data = speaker_result_to_payload(msg)
        if data.get("event"):
            return
        if msg.utterance_id:
            self._speakers.submit(msg.utterance_id, data)
            note_backlog_voice(self, msg.utterance_id, data)  # фраза в бэклоге: кто сказал её (#3767, #3892)
        self._session.set_speaker(speaker_ref(data, now=time.time(), prev=self._session.snapshot.speaker))

    def _on_speech_event(self, msg: Any) -> None:
        plan_host.on_speech(self, msg)
        greeting_window.on_speech_event(self, int(msg.kind), msg.speech_id, msg.agent)  # окно ответа (#3776)
        if msg.kind == SpeechEvent.KIND_VOICE_CHANGED:  # голос хранит tts_node; сессия помнит, что он применил
            self._session.set_voice(msg.voice_requested or None, msg.voice_applied or None, msg.provider or None)
            return
        if msg.agent == kn.TEXT_CHANNEL_AGENT:  # итог say_aloud чата (PR-12b): «Озвучил (N с)» по событию
            self._chats.on_speech(msg)
            return
        if msg.agent != kn.AGENTS[0]:
            return
        if msg.kind == SpeechEvent.KIND_STARTED:
            self._engine.speech_started(msg.speech_id)
        elif msg.kind in _ENDED:
            self._engine.speech_ended(msg.speech_id, int(msg.spoken_chars))

    def _on_music_state(self, msg: Any) -> None:
        snap = parse_music_state(msg.data)
        if snap is None:
            return
        self._music_player_state = snap
        self._music.observe_state(snap)
        plan_host.on_music(self, snap)
        title = str((snap.dj_info or {}).get("title") or "") or None
        self._session.set_music(MusicStateRef("playing" if snap.is_playing() else "idle", track_id=snap.track_id,
                                              title=title))
        self._scenes.on_music(bool(snap.dj), snap.dj_info)

    def _on_interrupt(self, msg: Any) -> None:
        """Сегмент фразы с вейком во время речи (stt_node): ясный стоп — перебивание; спорный решит ход (Tier-1/LLM)."""
        self._interrupt(msg.text_partial or "", msg.utterance_id or "", "interrupt_request")

    def _interrupt(self, text: str, uid: str, via: str) -> bool:
        """Ясный стоп (``interrupt.on_interrupt``): речь эпохи — отмена по границе слова, хода в полёте — отмена до
        HTTP-клиента LLM. Эпоху +1 кладёт ход стопа (Tier-1). Не стоп — ничего (речь не рвётся)."""
        started = time.monotonic()
        snap = self._session.snapshot
        stop = on_interrupt(text, snap, self._wake)
        if not stop.interrupts:
            return False
        speech_client.cancel(self, agent=kn.AGENTS[0], epoch=snap.epoch, reason=f"interrupt:{uid or '-'}",
                             boundary=stop.speech_boundary)
        cancelled = self._turns.cancel(keep_key=uid)
        self.get_logger().info(f"🛑 [interrupt] {stop.policy} via={via} {text[:60]!r} epoch={snap.epoch} "
                               f"boundary={stop.speech_boundary} turns_cancelled={cancelled} "
                               f"за {(time.monotonic() - started) * 1000:.1f} мс utterance={uid or '-'}")
        return True

    def _on_control(self, request: Any, response: Any) -> Any:
        """``DialogControl``: hold — кадр ``OperatorHold`` (TTL), resume — снять его, reset — новая сессия.
        Чужой ``set_by`` или неизвестное действие — ``ok=false`` (кадры не тронуты)."""
        action = _CONTROL_ACTIONS.get(int(request.action), "")
        response.ok = apply_control(self._session, action, request.set_by or "operator", float(request.ttl_s))
        if response.ok and action == RESET:
            self._engine.log.reset()
        snap = self._session.snapshot
        response.epoch = int(snap.epoch)
        response.scenes = identity_question.session_state(snap).scenes
        return response

    def _restore_voice(self, provider: str, voice: str) -> None:
        """Вернуть голос (конец персоны сета, ``reset``): просьба владельцу голоса, тот ответит ``voice_changed``."""
        body = String()
        body.data = json.dumps({"request_id": uuid.uuid4().hex[:12], "voice": voice, "provider": provider},
                               ensure_ascii=False)
        self._voice_pub.publish(body)
        self.get_logger().info(f"🎭 [scene] голос возвращается: {provider}:{voice}")

    # -- выход и служебное ---------------------------------------------------------------------------------------

    def _sections(self, snap: Any, _text: str) -> Dict[str, Any]:
        """Разделы снимка хода из владельцев: собеседник, провайдеры, музыка (данные, не правила)."""
        return host_sections(snap, self._music, reporter_context(self._reporter), self._turn_memory)

    def _speak_direct(self, text: str) -> None:
        """Фраза кода вне хода (смена провайдера): речь личности текущей эпохи."""
        self._ports.say("", self._session.snapshot.epoch, text)

    def _open_events(self) -> Any:
        try:
            return open_events(str(self.get_parameter("e2e_db_path").value), self._session.snapshot.session_id)
        except ValueError as exc:
            self.get_logger().error(f"🧪 [e2e] e2e-БД не открыта: {exc}")
            return None

    def _engine_events_state(self) -> str:
        events = getattr(self._engine, "_events", None)
        return getattr(events, "path", "off") if events is not None else "off"

    def _on_params(self, params: List[Any]) -> SetParametersResult:
        """Харнесс ставит/очищает ``e2e_db_path`` на лету (``ros2 param set``): e2e-БД открывается/закрывается сразу."""
        if any(p.name == "e2e_db_path" for p in params):
            self._loop.call_soon_threadsafe(self._reopen_events)
        return SetParametersResult(successful=True)

    def _reopen_events(self) -> None:
        old = getattr(self._engine, "_events", None)
        if old is not None:
            old.close()
        self._engine._events = self._open_events()
        self.get_logger().info(f"🧪 [e2e] e2e-БД: {self._engine_events_state()}")

    def shutdown(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)


class _RosLog:
    """``logging.Logger``-подобная обёртка логгера ROS для ``TurnEngine``."""

    def __init__(self, logger: Any) -> None:
        self.info, self.warning, self.error = logger.info, logger.warning, logger.error


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DialogNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
