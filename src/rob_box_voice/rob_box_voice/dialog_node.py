"""dialog_node — тонкий ROS-хост диалога v2 (ADR-0150 §2.2, §3; PR-8 ч.8e2 #3702).

Стартует только при ``dialog_engine: v2`` (launch, ``rob_box_core.dialog_engine``); при ``v1`` работает
``dialogue_node``. Вся логика хода — ``rob_box_dialog`` (``address → grammar → context → LLM → command → Executor →
respond``, :class:`rob_box_dialog.turn.TurnEngine`); транспорт — ``core.dialog_v2_ports`` (дорога исполнения v1:
LLM-цепочка с мостом команд, MCP-тулы под планировщиком ресурсов). Здесь — только ROS:

* подписки: ``/voice/stt/utterance`` (реплика → адресация → ход), ``/voice/speaker/result`` (собеседник),
  ``/voice/speech/event`` (окно озвученного, план, «говорю»), ``/voice/sound/event`` (план), ``/voice/music/state`` и
  ``/voice/music/event`` (снимок плеера, подложка выступления), ``/voice/providers/state`` (доска провайдеров),
  ``/dialog/interrupt_request`` (ясный стоп рвёт речь эпохи и ход в полёте), сервис ``/dialog/control``
  (hold/resume/reset);
* публикации: latched ``/dialog/session``, ``/dialog/turn_event`` и рефлексы, ``/dialog/intent``, ``/dialog/plan_event``
  (через ``plan_host``), речь — ``speech_client``; ``ActionRequest`` не публикуется (подписчика нет, см. ports);
* таймеры: TTL кадров сцен (1 с), переиздание состояния LLM-провайдеров (2 с); дедлайн хода — в ``turn.py``.

Перебивание (PR-9 #3703, ``rob_box_dialog.interrupt``): ясный стоп — с ``/dialog/interrupt_request`` (ранний
сегмент STT) или с адресованной реплики — отменяет речь эпохи (граница по слову) и все хода в полёте: отмена задачи
хода доходит до ``await`` LLM-провайдера, HTTP-запрос закрывает его клиент. Ход стопа (Tier-1) кладёт эпоху + 1.
Не стоп во время хода — новый ход ждёт конца текущего (APPEND), речь не рвётся.

Текст ходов на диск — только в e2e-БД вне ``/data`` (``e2e_mode`` + ``e2e_db_path``, ``rob_box_dialog.e2e_events``).
"""

from __future__ import annotations

import asyncio
import functools
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from rob_box_core import dialog_engine, speech_client
from rob_box_core.provider_publisher import make_publisher, publish_provider_states
from rob_box_core.speaker_wire import speaker_result_to_payload
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import Addressed, NotAddressed, decide, heard_from, log_label, person_ref_from
from rob_box_dialog.address import wake_words as load_wake_words
from rob_box_dialog.backlog import Backlog
from rob_box_dialog.e2e_events import open_events
from rob_box_dialog.interrupt import TurnsInFlight, on_interrupt
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.speaker_adapter import speaker_block, speaker_ref
from rob_box_dialog.turn import TurnEngine
from rob_box_dialog_msgs.msg import Intent as IntentMsg
from rob_box_dialog_msgs.msg import InterruptRequest, ProviderStateArray, SessionState, SoundEvent, SpeakerResult
from rob_box_dialog_msgs.msg import SpeechEvent
from rob_box_dialog_msgs.msg import Utterance as UtteranceMsg
from rob_box_dialog_msgs.srv import DialogControl
from rob_box_harness.core.assembly import AgentSpec, build_llm_chain, load_system_prompt
from rob_box_llm.provider import LLMSettings

from .core import command_bridge, identity_question
from .core import plan_host
from .core.dialog_v2_ports import CallCounter, V2Ports, command_providers, counted_wrapper
from .core.music_player_state import MUSIC_EVENT_TOPIC, MUSIC_STATE_TOPIC, parse_music_state
from .core.provider_reporter import ProviderStateReporter, on_provider_states, reporter_context
from .core.utterance_speaker import UtteranceSpeakerRegistry
from rob_box_dialog.music_state import MusicStateMemory

#: ``SpeechEvent.kind`` конца реплики: finished / cancelled / rejected.
_ENDED = (SpeechEvent.KIND_FINISHED, SpeechEvent.KIND_CANCELLED, SpeechEvent.KIND_REJECTED)
_LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def declare_params(node: Any) -> None:
    """Параметры ноды (``dialog_node.yaml``; ключи API — из окружения, как у ``dialogue_node``)."""
    node.declare_parameter("llm_providers", "deepseek,minimax")
    node.declare_parameter("system_prompt_file", "master_prompt_compact.txt")
    node.declare_parameter("wake_words_path", "/config/wake_words.yaml")
    node.declare_parameter("temperature", 0.0)
    node.declare_parameter("max_tokens", 0)
    node.declare_parameter("e2e_mode", False)
    node.declare_parameter("e2e_db_path", "")


def _settings(node: Node) -> Optional[LLMSettings]:
    temperature = float(node.get_parameter("temperature").value or 0.0)
    max_tokens = int(node.get_parameter("max_tokens").value or 0)
    if temperature <= 0 and max_tokens <= 0:
        return None
    return LLMSettings(temperature=temperature or None, max_tokens=max_tokens or None)


def _prompt_dir() -> Path:
    from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

    return Path(get_package_share_directory("rob_box_voice")) / "prompts"


def build_llm(node: Any, counter: CallCounter) -> tuple:
    """LLM-цепочка v2: тот же ``build_llm_chain``, все провайдеры — в режиме команд моста, вызовы считаются."""
    chain = tuple(p.strip().lower() for p in str(node.get_parameter("llm_providers").value).split(",") if p.strip())
    bridge = command_bridge.wrapper_for(command_providers(chain), log=node.get_logger().info,
                                        busy=lambda: plan_host.perform_busy(node))
    spec = AgentSpec(name="personality", prompt_dir=_prompt_dir(),
                     system_prompt_file=str(node.get_parameter("system_prompt_file").value),
                     provider_chain=chain, settings=_settings(node), wrap_provider=counted_wrapper(bridge, counter))
    return build_llm_chain(spec), load_system_prompt(spec)


def build_tools(node: Any) -> Any:
    """MCP-тулы под планировщиком ресурсов — как у v1 (``LLMToolCallAdapter`` → ``ROSMCPToolProvider``)."""
    from rob_box_harness.core.tool_registry import ToolRegistry  # noqa: PLC0415
    from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider  # noqa: PLC0415
    from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: PLC0415

    provider = ROSMCPToolProvider(LLMToolCallAdapter(node, turn_context=lambda: {"utterance_id": node.utterance_id}))
    provider.update_tools([{"type": "function", "function": {"name": s.name, "description": s.description,
                                                             "parameters": dict(s.parameters)}}
                           for s in ToolRegistry().list_tools()])
    if not provider.list_tools():
        raise RuntimeError("dialog_node: каталог MCP-тулов пуст — команды молча не исполнялись бы")
    return plan_host.make_tool_executor(node, adapt_tool_provider(provider), lambda *_: None)


class DialogNode(Node):
    """ROS-хост хода v2: подписки → ``TurnEngine``, порты → топики."""

    def __init__(self) -> None:
        super().__init__("dialog_node")
        declare_params(self)
        self.engine_flag = dialog_engine.declare(self)
        self.utterance_id: Optional[str] = None
        self.last_turn: Any = None  # concurrent.futures.Future последнего хода (журнал, тесты)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, name="dialog_v2_loop")
        self._turn_lock = asyncio.Lock()
        self._turns = TurnsInFlight()
        self._session_pub = self.create_publisher(SessionState, identity_question.SESSION_TOPIC, _LATCHED)
        self._session = Session("personality", clock=time.time)
        self._session.subscribe(functools.partial(identity_question.publish, self))
        self._wake = load_wake_words(str(self.get_parameter("wake_words_path").value))[0]
        self._backlog = Backlog()
        self._speakers = UtteranceSpeakerRegistry()
        self._music = MusicStateMemory()
        self._music_player_state: Any = None
        counter = CallCounter()
        llm, system = build_llm(self, counter)
        self._tool_executor = build_tools(self)
        self._ports = V2Ports(self, llm, counter, self._tool_executor,
                              intent_pub=self.create_publisher(IntentMsg, "/dialog/intent", 10),
                              log=self.get_logger().info, epoch=lambda: self._session.snapshot.epoch,
                              performer=functools.partial(plan_host.performer, self))
        self._reporter = ProviderStateReporter(llm, functools.partial(publish_provider_states, make_publisher(self)),
                                               self._speak_direct)
        self._engine = TurnEngine(self._session, self._ports, system_prompt=system, sections=self._sections,
                                  events=self._open_events(), log=_RosLog(self.get_logger()))
        self._subscribe()
        self.create_service(DialogControl, "/dialog/control", self._on_control)
        self.create_timer(1.0, self._session.expire)
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
        self.create_subscription(ProviderStateArray, "/voice/providers/state",
                                 functools.partial(on_provider_states, self._reporter.board, time.time), _LATCHED)
        self.create_subscription(InterruptRequest, "/dialog/interrupt_request", self._on_interrupt, 10)

    # -- вход ---------------------------------------------------------------------------------------------------

    def _on_utterance(self, msg: Any) -> None:
        """Реплика: адресация кодом (вейк, бэклог, окна ответа); адресованная — ход в asyncio-loop ноды."""
        now = time.time()
        self.get_logger().info(f"🎤 STT: {log_label(msg)}")
        uid = str(getattr(msg, "utterance_id", "") or "")
        heard = heard_from(msg, now=now, person_ref=person_ref_from(self._speakers, uid))
        snap = self._session.snapshot
        decision = decide(heard, self._backlog, self._wake, question=snap.top("Identity"),
                          ask=snap.top("PerformAsk"))
        if isinstance(decision, NotAddressed):
            self._backlog.add(heard)
            self.get_logger().info(f"[backlog] accumulated reason={decision.reason} entries="
                                   f"{self._backlog.background_phrases(now)}")
            return
        self.utterance_id = uid or None
        self._interrupt(decision.text, uid, "utterance")
        self.last_turn = self._turns.add(asyncio.run_coroutine_threadsafe(self._turn(decision), self._loop), uid)

    async def _turn(self, decision: Addressed) -> None:
        if self._turn_lock.locked():  # не стоп во время хода — APPEND: новый ждёт конца текущего, речь не рвётся
            self.get_logger().info("🧭 [turn-v2] ход уже идёт — новый ждёт его конца")
        async with self._turn_lock:
            try:
                await self._engine.run(decision)
            except Exception as exc:  # noqa: BLE001 — упавший ход не роняет ноду, но громко
                self.get_logger().error(f"🧭 [turn-v2] ход упал: {type(exc).__name__}: {exc}")

    def _on_speaker(self, msg: Any) -> None:
        """Биометрия фразы: реестр по ``utterance_id`` и собеседник сессии (имя — только при ``confirmed``)."""
        data = speaker_result_to_payload(msg)
        if data.get("event"):
            return
        if msg.utterance_id:
            self._speakers.submit(msg.utterance_id, data)
        self._session.set_speaker(speaker_ref(data, now=time.time(), prev=self._session.snapshot.speaker))

    def _on_speech_event(self, msg: Any) -> None:
        plan_host.on_speech(self, msg)
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
                                              title=title, dj=bool(snap.dj)))

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
        """``DialogControl``: hold — кадр ``OperatorHold`` (TTL), resume — снять его, reset — новая сессия."""
        set_by = request.set_by or "operator"
        if request.action == DialogControl.Request.ACTION_HOLD:
            self._session.push_scene("OperatorHold", set_by, ttl_s=float(request.ttl_s) or None)
        elif request.action == DialogControl.Request.ACTION_RESUME:
            self._session.pop_scene("OperatorHold", set_by)
        elif request.action == DialogControl.Request.ACTION_RESET:
            self._session.reset()
            self._engine.log.reset()
        snap = self._session.snapshot
        response.ok, response.epoch = True, int(snap.epoch)
        response.scenes = identity_question.session_state(snap).scenes
        return response

    # -- выход и служебное ---------------------------------------------------------------------------------------

    def _sections(self, snap: Any, _text: str) -> Dict[str, Any]:
        """Разделы снимка хода из владельцев: собеседник, провайдеры, музыка (данные, не правила)."""
        return {"speaker": speaker_block(snap.speaker), "providers": reporter_context(self._reporter),
                "music_state": self._music.section()}

    def _speak_direct(self, text: str) -> None:
        """Фраза кода вне хода (смена провайдера): речь личности текущей эпохи."""
        self._ports.say("", self._session.snapshot.epoch, text)

    def _open_events(self) -> Any:
        try:
            return open_events(bool(self.get_parameter("e2e_mode").value), str(self.get_parameter("e2e_db_path").value),
                               self._session.snapshot.session_id)
        except ValueError as exc:
            self.get_logger().error(f"🧪 [e2e] e2e-БД не открыта: {exc}")
            return None

    def _engine_events_state(self) -> str:
        events = getattr(self._engine, "_events", None)
        return getattr(events, "path", "off") if events is not None else "off"

    def _on_params(self, params: List[Any]) -> SetParametersResult:
        """Харнесс включает ``e2e_mode`` на лету (``ros2 param set``): e2e-БД открывается/закрывается сразу."""
        if any(p.name in ("e2e_mode", "e2e_db_path") for p in params):
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
