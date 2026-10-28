"""Сухой стенд хода v2: настоящий ``TurnEngine`` без ROS и без робота (эпик #3683, карточка #3906).

Робот — узкое место: акт марафона идёт 5–15 мин, деплой выбивает стенд на 15–40 мин. Этот модуль прогоняет текст
реплики теми же функциями, что на роботе: ``address.decide_in_session`` (вейк, бэклог) → ``TurnEngine.run``
(Tier-1 / команда LLM → исполнение → фраза кода) → окно озвученного. Настоящими остаются: грамматика и адресация,
сессия, бэклог с авторством, срез тулов, цепочка LLM с мостом команд (``dialog_v2_ports.assemble_llm``), исполнитель
действий (``V2Ports`` + ``Executor``), тулы памяти (``rob_box_mcp_tools.tools.memory`` поверх настоящего
``VoiceMemory`` во временной e2e-БД).

Фейковые владельцы, которые отвечают событием сразу (:class:`FakeTools`, :class:`FakeSpeech`, :class:`FakeSpeakers`):
речь, музыка, голос/темп/громкость, лицо, биометрия диктора, статус робота. Что они знают о мире — таблицей здесь; в
отчёт каждого шага попадает, каких владельцев ход коснулся (``DryStand.touched``). Лог шага (:class:`StepLog`) — в
форматах строк живого робота (источники указаны у каждой строки): по нему работает та же
``e2e_tool_match.step_verdict``, что у ``check_acceptance`` на стенде.

Чего стенд не видит по построению: STT (опечатки распознавания, «Робби»), настоящая биометрия, звук, тайминги
перебивания, гонки ROS. Такие шаги помечает ``dry_marathon.classify_step``.

Вариант понимания (карточка #4163, :mod:`understanding_variants`): ``DryStand(variant="A"|"B"|"C")`` на время реплики
подменяет в движке Tier-1 и срез тулов - кто понимает фразу, код по словам (A) или модель (B, C). Боевой код не тронут.
"""

from __future__ import annotations

import importlib
import json
import re
import sys
import tempfile
import types
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Set, Tuple

REPO = Path(__file__).resolve().parents[2]
#: Пакеты ``src/<пакет>/<пакет>``: свои, а не editable-установка основного чекаута (в worktree она старая).
_SRC = REPO / "src"


def ensure_paths() -> None:
    """Положить пакеты репозитория первыми в ``sys.path`` (без ROS: сообщения ``rob_box_dialog_msgs`` не нужны)."""
    for pkg in sorted(_SRC.iterdir()) if _SRC.is_dir() else ():
        if (pkg / pkg.name).is_dir() and str(pkg) not in sys.path:
            sys.path.insert(0, str(pkg))


ensure_paths()
sys.path.insert(0, str(Path(__file__).resolve().parent))

import e2e_db  # noqa: E402
import understanding_variants as uv  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import (  # noqa: E402
    Addressed, NotAddressed, decide_in_session, heard_from, log_label, wake_words,
)
from rob_box_dialog.backlog import Backlog  # noqa: E402
from rob_box_dialog.e2e_events import inside_robot_data, open_events  # noqa: E402
from rob_box_dialog.music_state import MusicStateMemory  # noqa: E402
from rob_box_dialog.session import MusicStateRef, Session  # noqa: E402
from rob_box_dialog.speaker_adapter import speaker_ref, voice_tag  # noqa: E402
from rob_box_dialog.turn import ASK_MARK, LOG_MARK, TurnEngine, host_sections  # noqa: E402
from rob_box_llm.provider import ToolResult  # noqa: E402
from rob_box_mcp_tools.voice_request import VoiceOutcome, outcome_data  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports  # noqa: E402
from rob_box_voice.core.music_player_state import build_music_state_payload, parse_music_state  # noqa: E402
from rob_box_voice.core.voice_memory import VoiceMemory  # noqa: E402
from rob_box_voice.tts_voice_registry import (  # noqa: E402
    ROBOT_DEFAULT_PROVIDER, default_voice_for, providers_overview, robot_default_voice, voice_id_by_name, voices_for,
)

WAKE_WORDS_YAML = REPO / "docker" / "sound" / "config" / "wake_words.yaml"
#: Скорость речи человека/робота для модели времени, символов в секунду (живой замер ≈ 13–16).
SPEECH_CPS = 14.0
#: Пауза харнесса между шагами, с: шаг ждёт конца речи робота + паузу. Фраза в бэклог ждёт только маркер бэклога: по
#: живому прогону акта 3 (run 39910805388) шаги ``expect: backlog`` идут с периодом ≈ 18 с при 6–8 с звука фразы.
STEP_GAP_S = 3.0
BACKLOG_GAP_S = 8.0
#: Ночь, мастерская: точка отсчёта фейковых часов (местное время робота).
NIGHT_START = datetime(2026, 10, 24, 1, 40, 0)


class FakeClock:
    """Часы сессии и бэклога: время идёт по модели шагов (речь, паузы), а не по стене — TTL и окна честны."""

    def __init__(self, start: datetime = NIGHT_START) -> None:
        self._t = start.timestamp()

    def now(self) -> float:
        return self._t

    def advance(self, seconds: float) -> float:
        self._t += max(0.0, seconds)
        return self._t

    def wall(self) -> datetime:
        return datetime.fromtimestamp(self._t)


class LiveClock(FakeClock):
    """Часы консоли: время суток для человека - настоящее (``get_current_time`` не врёт про «час сорок ночи»), а
    сессия и бэклог идут по той же модели времени, что и в марафоне (речь и паузы), от настоящего «сейчас»."""

    def __init__(self) -> None:
        super().__init__(datetime.now())

    def wall(self) -> datetime:
        return datetime.now().astimezone()


@dataclass
class StepLog:
    """Лог одного шага в форматах строк живого робота; харнесс сверяет шаг по нему."""

    lines: List[str] = field(default_factory=list)

    def add(self, line: str) -> None:
        self.lines.extend(str(line).splitlines() or [""])

    @property
    def text(self) -> str:
        return "\n".join(self.lines)


class _EngineLog:
    """``logging.Logger``-подобный приёмник для ``TurnEngine`` и моста команд: строки идут в лог текущего шага."""

    def __init__(self, sink: Callable[[str], None]) -> None:
        self.info = self.warning = self.error = sink


class _NodeLogger:
    def __init__(self, sink: Callable[[str], None]) -> None:
        self.info = self.warn = self.warning = self.error = self.debug = sink


class _FakeRosNode:
    """Минимум ноды для тулов MCP: логгер и ``voice_memory`` (tools/memory.py читает только их)."""

    def __init__(self, sink: Callable[[str], None], memory: VoiceMemory) -> None:
        self._logger = _NodeLogger(sink)
        self.voice_memory = memory

    def get_logger(self) -> _NodeLogger:
        return self._logger


def load_tool_module(module: str) -> types.ModuleType:
    """``rob_box_mcp_tools.tools.<module>`` без ``tools/__init__`` (тот тянет rclpy через navigation): пакет-заглушка с
    ``__path__``, сам модуль — настоящий."""
    name = "rob_box_mcp_tools.tools"
    if name not in sys.modules:
        stub = types.ModuleType(name)
        stub.__path__ = [str(_SRC / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "tools")]
        sys.modules[name] = stub
    return importlib.import_module(f"{name}.{module}")


def load_memory_tools() -> types.ModuleType:
    return load_tool_module("memory")


# ── Биометрия диктора (фейк) ──────────────────────────────────────────────────────────────────────────────────────


class FakeSpeakers:
    """Владелец биометрии: голос синтеза (``anton``, ``ermil`` …) → результат в форме ``rob_box_core.speaker_wire``.

    Незарегистрированный голос — ``unknown_voice``; после ``register_speaker`` (тул вызывается во время хода) голос
    узнан с уверенным именем уже на СЛЕДУЮЩЕЙ реплике (как биометрия, которая приходит после хода). Настоящий порог
    0.72 и прогрев профиля на роботе — акустика, здесь их нет."""

    def __init__(self) -> None:
        self._known: Dict[str, Tuple[str, str]] = {}

    def register(self, voice_id: str, name: str) -> str:
        sid = self._known.get(voice_id, (uuid.uuid4().hex[:8], name))[0]
        self._known[voice_id] = (sid, name)
        return sid

    def result_for(self, voice_id: str) -> Dict[str, Any]:
        known = self._known.get(voice_id)
        if known is None:
            return {"is_known": False, "inconclusive": False, "speaker_id": None, "name": None, "confidence": 0.0}
        sid, name = known
        return {"is_known": True, "inconclusive": False, "speaker_id": sid, "name": name, "confidence": 0.88}


# ── Речь (фейк) ───────────────────────────────────────────────────────────────────────────────────────────────────


@dataclass
class Said:
    speech_id: str
    text: str
    voice: str
    turn_id: str
    epoch: int


class FakeSpeech:
    """Владелец речи: принимает реплику, пишет строки ``tts_node`` и ``speech_owner``, «озвучивает» сразу."""

    def __init__(self, emit: Callable[[str], None]) -> None:
        self._emit = emit
        self.pending: List[Said] = []
        self.all: List[Said] = []

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str]) -> str:
        speech_id = str(uuid.uuid4())
        said = Said(speech_id, text, voice or "", turn_id, epoch)
        self.pending.append(said)
        self.all.append(said)
        # tts_node.py: f"🔊 TTS: speech_id={speech_id[:8]}, dialogue_id=…, batch=… i/n, voice=…, lang=…, text={text!r}"
        self._emit(f"🔊 TTS: speech_id={speech_id[:8]}, dialogue_id=None, batch=None 1/1, "
                   f'voice={voice or "default"}, lang=default, text={text!r}')
        return speech_id

    def settle(self, engine: TurnEngine, agent: str) -> float:
        """Реплики отзвучали: событие владельца в движок (окно озвученного — только отзвучавшее). Возвращает секунды."""
        total = 0.0
        for said in self.pending:
            engine.speech_started(said.speech_id)
            # speech_owner.py: f"🗣️ SpeechEvent {kind} speech_id={…[:8]} agent=… epoch=… text_len=… {extra}"
            self._emit(f"🗣️ SpeechEvent started speech_id={said.speech_id[:8]} agent={agent} "
                       f"epoch={said.epoch} text_len={len(said.text)} ")
            engine.speech_ended(said.speech_id, len(said.text))
            self._emit(f"🗣️ SpeechEvent finished speech_id={said.speech_id[:8]} agent={agent} "
                       f"epoch={said.epoch} text_len={len(said.text)} spoken_chars={len(said.text)}")
            total += len(said.text) / SPEECH_CPS
        self.pending.clear()
        return total


# ── Тулы (фейковые владельцы + настоящая память) ──────────────────────────────────────────────────────────────────

#: Тулы, которых у стенда нет: честный отказ ``no_fake_owner`` (а не выдуманный успех).
NO_FAKE = "no_fake_owner"
#: Какой фейковый владелец ответил на тул (для отчёта шага: на чём держится вердикт).
TOOL_OWNERS: Mapping[str, str] = {
    "get_current_time": "clock", "get_current_pose": "robot_status", "get_battery_level": "robot_status",
    "get_robot_status": "robot_status",
    "get_perception_context": "face", "play_animation": "face", "get_sound_info": "sound", "play_sound": "sound",
    "list_tts_voices": "voice", "set_voice": "voice", "set_tts_provider": "voice", "set_speed": "voice",
    "set_pitch": "voice", "set_volume": "volume", "register_speaker": "speaker",
    "get_music_state": "music", "set_music_volume": "music", "request_music": "music", "generate_music": "music",
    "dj_set": "music", "music_library": "music", "stop_music": "music",
}


class FakeTools:
    """Исполнитель тулов хода (``V2Ports._send`` зовёт ``execute``): владельцы отвечают сразу, данные — как у настоящих
    тулов (ключи ``data`` сверены с ``rob_box_mcp_tools/tools/*``). Память — настоящие MemorySave/Search/ContextTool."""

    def __init__(self, stand: "DryStand") -> None:
        self._stand = stand
        self.calls: List[Tuple[str, Mapping[str, Any]]] = []
        self._memory_tools: Dict[str, Any] = {}

    def begin_turn(self) -> None:
        return None

    def _memory(self, name: str) -> Any:
        if not self._memory_tools:
            mod = load_memory_tools()
            node = self._stand.ros_node
            self._memory_tools = {"memory_save": mod.MemorySaveTool(node), "memory_search": mod.MemorySearchTool(node),
                                  "memory_context": mod.MemoryContextTool(node)}
        return self._memory_tools[name]

    async def execute(self, call: Any) -> ToolResult:
        args = dict(call.arguments)
        self.calls.append((call.name, args))
        request_id = uuid.uuid4().hex[:8]
        # llm_adapter.py: f"📤 Отправлен запрос {request_id[:8]}: {tool_name}"
        self._stand.emit(f"📤 Отправлен запрос {request_id}: {call.name}")
        # mcp_server.py: f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}"
        self._stand.emit(f"📥 Запрос выполнения: {call.name} с параметрами {args}")
        ok, data = self._dispatch(call.name, args)
        # mcp_server.py: «Публикую результат» - и при отказе тула; затем «выполнен успешно» либо «завершился с ошибкой»
        self._stand.emit(f"📤 Публикую результат для {call.name} (request_id: {request_id})")
        self._stand.emit(f"✅ Инструмент {call.name} выполнен успешно" if ok else
                         f"❌ Инструмент {call.name} завершился с ошибкой: {data.get('reason')}")
        return ToolResult(tool_call_id=call.id, content=json.dumps(data, ensure_ascii=False, default=str),
                          is_error=not ok, data=data)

    def _dispatch(self, name: str, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        if name in ("memory_save", "memory_search", "memory_context"):
            self._stand.touch("memory")
            result = self._memory(name).execute(**args)
            data = dict(result.data or {})
            if not result.success:
                data = {**data, "status": "rejected", "reason": (result.message or result.error or "error")[:120]}
            return bool(result.success), data
        handler = getattr(self, f"_t_{name}", None)
        if handler is None:
            return False, {"status": "rejected", "reason": NO_FAKE, "tool": name}
        self._stand.touch(TOOL_OWNERS.get(name, "owner"))
        return handler(args)

    # -- чтение состояния робота (query) --------------------------------------------------------------------------

    def _t_get_current_time(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """Те же поля и та же пропись времени, что у настоящего тула (``system.GetCurrentTimeTool``: ``format_time_ru``,
        время суток по часу), от часов стенда: марафон - ночь мастерской, консоль - настоящее время."""
        system = load_tool_module("system")
        now = self._stand.clock.wall()
        weekdays = ["понедельник", "вторник", "среда", "четверг", "пятница", "суббота", "воскресенье"]
        months = ["января", "февраля", "марта", "апреля", "мая", "июня", "июля", "августа", "сентября", "октября",
                  "ноября", "декабря"]
        return True, {"time": now.strftime("%H:%M"), "date": f"{now.day} {months[now.month - 1]} {now.year}",
                      "weekday": weekdays[now.weekday()], "period": system._hour_period(now.hour),  # noqa: SLF001
                      "iso": now.isoformat(timespec="seconds"), "timezone": str(now.tzinfo or "Europe/Moscow"),
                      "formatted_time": system.format_time_ru(now)}

    def _t_get_battery_level(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"battery_level": 87.0, "warning": False}

    def _t_get_robot_status(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"position": {"x": 1.2, "y": 0.4, "yaw": 0.1}, "battery_level": 87.0, "systems": {}}

    def _t_get_perception_context(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"faces": [], "objects": [], "people_count": 0, "summary": "никого в кадре, мастерская, ночь"}

    def _t_get_sound_info(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        triggers = ["beep", "chime", "fanfare", "whoosh", "click", "alarm", "robot_hello", "robot_sad", "laugh"]
        return True, {"total_sounds": len(triggers), "categories": {"base": 3, "ui": 3, "robot": 3, "other": 0},
                      "all_triggers": triggers}

    def _t_list_tts_voices(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        provider = str(args.get("provider") or "")
        active = provider or self._stand.voice_provider
        if not provider:  # как у настоящего тула (#3883): обзор по провайдерам, без полного списка ids
            return True, {"provider": active, "default_voice": default_voice_for(active),
                          "providers": providers_overview(active)}
        return True, {"provider": provider, "voices": voices_for(provider),
                      "default_voice": default_voice_for(provider)}

    def _t_get_music_state(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        deck = self._stand.player.deck()
        return True, {"deck": deck, "playing": bool(deck.get("playing"))}

    def _t_play_animation(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"status": "done", "animation": args.get("name") or args.get("animation") or "idle"}

    # -- действия владельцев ------------------------------------------------------------------------------------

    def _t_register_speaker(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        name = str(args.get("name") or "").strip()
        if not name:
            return False, {"status": "rejected", "reason": "name_empty", "ask_required": True, "name": None}
        voice = self._stand.current_voice_id
        sid = self._stand.speakers.register(voice, name) if voice else ""
        # speaker_id_node.py: f"✅ Speaker '{name}' registered (id={sid[:8]})" - исход регистрации у владельца биометрии
        self._stand.emit(f"✅ Speaker '{name}' registered (id={sid[:8]})")
        return True, {"name": name, "status": "done", "reason": None, "sent_by_robot": False, "speaker_id": sid}

    def _t_get_current_pose(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"x": 1.2, "y": 0.4, "yaw": 0.1, "frame": "map"}

    def _voice_outcome(self, status: str, requested: str, applied: str, provider: str, reason: str = "",
                       ok: bool = True) -> Tuple[bool, Dict[str, Any]]:
        """Исход смены голоса, как у тула: строка лога и данные исхода (``voice_request.outcome_data``); тул не говорит -
        фразу строит диалог по этим данным (``knowledge.VOICE_DONE``)."""
        stand = self._stand
        # dialogue.py (SetVoiceTool): f"[set_voice] voice={…!r} provider={…} default={…} switched=… status=…"
        stand.emit(f"[set_voice] voice={applied!r} provider={provider} default={default_voice_for(provider)} "
                   f"switched={provider != stand.voice_provider} status={status}")
        data = {**outcome_data(VoiceOutcome(status, requested, applied, provider, reason)),
                "available": voices_for(provider)}
        return ok, data

    def _t_set_voice(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        stand = self._stand
        voice = str(args.get("voice") or "").strip()
        if not voice:
            return False, {"status": "rejected", "reason": "voice_empty"}
        provider = str(args.get("provider") or stand.voice_provider)
        if voice.lower() == kn.VOICE_DEFAULT_ARG:  # «обычный голос» - голос робота по умолчанию (#3803)
            provider, voice = robot_default_voice()
            if (stand.voice_provider, stand.session.snapshot.voice.applied) in ((provider, voice), (provider, None)):
                return self._voice_outcome("already_default", voice, voice, provider)
        current = stand.session.snapshot.voice.applied or default_voice_for(stand.voice_provider)
        voice = voice_id_by_name(provider, voice) or voice  # имя каталога («Весельчак») -> id, как у владельца (#4086)
        if voice not in voices_for(provider):
            return self._voice_outcome("fallback", voice, current, stand.voice_provider, "voice_unavailable", False)
        outcome = self._voice_outcome("applied", voice, voice, provider)
        stand.voice_provider = provider
        stand.session.set_voice(voice, voice, provider)
        return outcome

    def _t_set_tts_provider(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        stand = self._stand
        provider = str(args.get("provider") or "").strip()
        if provider not in ("minimax", "yandex", "silero"):
            return False, {"status": "rejected", "reason": "provider_unknown", "requested": provider}
        stand.voice_provider = provider
        voice = default_voice_for(provider)
        stand.session.set_voice(voice, voice, provider)
        return True, outcome_data(VoiceOutcome("applied", "", voice, provider))

    def _t_set_volume(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"old_volume": 70, "new_volume": 80 if str(args.get("action")) == "louder" else 60}

    def _t_set_speed(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"old_speed": 1.0, "new_speed": 0.85 if "slow" in str(args) else 1.2}

    def _t_set_pitch(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"old_pitch": 1.0, "new_pitch": 0.8}

    def _t_set_music_volume(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"status": "done", "action": args.get("action")}

    def _t_play_sound(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return True, {"status": "done", "sound": args.get("sound") or args.get("name")}

    def _music_start(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        title = str(args.get("title") or args.get("query") or args.get("prompt") or args.get("theme") or "трек")
        track = self._stand.player.start(title)
        return True, {"status": "done", "event": "started", "track_id": track, "title": title}

    def _t_request_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return self._music_start(args)

    def _t_generate_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return self._music_start(args)

    def _t_dj_set(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return self._music_start(args)

    def _t_music_library(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        return self._music_start(args)

    def _t_stop_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        stopped = self._stand.player.stop()
        return True, {"status": "done", "stopped": stopped}


class FakePlayer:
    """Владелец плеера: играет/молчит; каждое изменение — снимок ``/voice/music/state`` в сессию и память музыки."""

    def __init__(self, stand: "DryStand") -> None:
        self._stand = stand
        self._track: Optional[str] = None
        self._title: Optional[str] = None
        self._n = 0

    def deck(self) -> Dict[str, Any]:
        return {"playing": self._track is not None, "track_id": self._track, "title": self._title}

    def start(self, title: str) -> str:
        self._n += 1
        self._track, self._title = f"dry-track-{self._n}", title
        self._publish()
        return self._track

    def stop(self) -> bool:
        was = self._track is not None
        self._track = self._title = None
        self._publish()
        return was

    def _publish(self) -> None:
        stand = self._stand
        payload = build_music_state_payload(playing=self._track is not None, track_id=self._track, ts=stand.clock.now())
        snap = parse_music_state(payload)
        stand.music_mem.observe_state(snap, now=stand.clock.now())
        stand.music_mem.observe_track_name(self._title)
        stand.session.set_music(MusicStateRef("playing" if self._track else "idle", track_id=self._track,
                                              title=self._title))


# ── Порты хода ────────────────────────────────────────────────────────────────────────────────────────────────────


class DryPorts(V2Ports):
    """``V2Ports`` без ROS: LLM-вызов и исполнитель — настоящие (родитель), речь/стадии/интент — фейковые владельцы."""

    def __init__(self, stand: "DryStand", llm: Any, counter: CallCounter, tools: FakeTools) -> None:
        super().__init__(None, llm, counter, tools, log=stand.emit, epoch=lambda: stand.session.snapshot.epoch)
        self._stand = stand
        self.stages: List[Tuple[str, str, str]] = []
        self.intents: List[str] = []

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        return self._stand.speech.say(turn_id, epoch, text, voice)

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "") -> None:
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        self._stand.speech.pending.clear()

    def stop_perform(self, how: str) -> bool:
        return False

    def intent(self, turn_id: str, epoch: int, kind: str) -> None:
        self.intents.append(kind)


# ── Стенд ─────────────────────────────────────────────────────────────────────────────────────────────────────────


@dataclass
class HearResult:
    """Итог одной реплики на стенде: адресована ли, агрегат хода, что сказано, что вызвано."""

    text: str
    addressed: bool
    reason: str = ""
    source: str = ""
    aggregate: Any = None
    said: List[str] = field(default_factory=list)
    tools: List[str] = field(default_factory=list)
    llm_calls: int = 0
    log: str = ""
    touched: Set[str] = field(default_factory=set)
    #: Исходы действий хода из e2e-БД: ``(тул, статус, причина)`` — «вызван» ещё не «исполнен» (``rejected{…}``).
    outcomes: List[Tuple[str, str, Optional[str]]] = field(default_factory=list)
    #: Вид Tier-1, которым ход закрыт кодом без LLM (``knowledge.TIER1_KINDS``); ``""`` - ход шёл к модели.
    tier1: str = ""
    #: Время хода по движку (реплика → итог, с ожиданием LLM и исполнителей), мс; у варианта C плюс ответ классификатора.
    latency_ms: int = 0
    #: Токены вызовов LLM хода (от провайдера, если прислал ``usage``; фейк - 0).
    tokens: int = 0
    #: Тулы, которые увидела модель (срез хода); ход без LLM - пусто. Тулы, добавленные мостом позже, - в ``widened``.
    offered: List[str] = field(default_factory=list)
    #: Тулы вне среза, которые мост добавил в запрос после ``tool_not_available`` (``tools_view.widen_for``, #3960).
    widened: List[str] = field(default_factory=list)
    #: Вариант понимания, на котором шёл ход (``understanding_variants``), и выбор классификатора (вариант C).
    variant: str = uv.A
    route: Any = None
    #: Вызовы тулов хода с аргументами (``tools`` - только имена) и ответ человека «да» на вопрос тула (подтверждение).
    calls: List[Tuple[str, Dict[str, Any]]] = field(default_factory=list)
    asked_yes: bool = False


_TIER1_LINE = re.compile(re.escape(LOG_MARK) + r" tier1 kind=(\w+)")
_ASKED_YES_LINE = re.compile(re.escape(LOG_MARK) + " " + re.escape(ASK_MARK) + " ответ да")
_WIDENED_LINE = re.compile(r"tool_not_available → в запрос добавлены тулы \[(.*?)\]")


def tier1_kind(log: str) -> str:
    """Вид Tier-1 хода по строке движка ``🧭 [turn-v2] tier1 kind=…`` (та же строка, что читает живой харнесс)."""
    found = _TIER1_LINE.search(log)
    return found.group(1) if found else ""


def widened_tools(log: str) -> List[str]:
    """Тулы, которые мост добавил в запрос ``tool_not_available → …`` (строки моста команд в логе хода)."""
    return [name.strip(" '\"") for line in _WIDENED_LINE.findall(log) for name in line.split(",") if name.strip(" '\"")]


class DryStand:
    """Один «робот» без железа: сессия, бэклог, память, движок хода. ``llm`` — уже собранная цепочка с мостом команд
    (``assemble_llm``) или фейковый провайдер в обёртке моста; ``counter`` считает её настоящие вызовы.

    ``variant`` - кто понимает фразу (``understanding_variants``: ``A`` как на роботе, ``B`` модель, ``C`` классификатор
    ``router``); подмена Tier-1 и среза тулов живёт только на время реплики. ``memory_path`` - файл e2e-памяти, который
    переживает прогон (консоль ``--db``); вне боевой ``/data``. ``clock`` - часы: по умолчанию ночь мастерской
    (:class:`FakeClock`, марафон), у консоли - настоящее время (:class:`LiveClock`)."""

    def __init__(self, llm: Any, counter: CallCounter, system_prompt: str, workdir: Path,
                 wake: Optional[Sequence[str]] = None, *, variant: str = uv.A, router: Any = None,
                 memory_path: Optional[Path] = None, clock: Optional[FakeClock] = None) -> None:
        self.workdir = Path(workdir)
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.variant = uv.check(variant)
        if self.variant == uv.C and router is None:
            raise ValueError("вариант C без классификатора (router): нечем выбирать группу тулов")
        self.router = router
        if memory_path is not None and inside_robot_data(str(memory_path)):
            raise ValueError(f"память {memory_path} внутри боевой /data: она общая с живыми людьми, e2e-память - вне неё")
        self._memory_path = Path(memory_path) if memory_path is not None else None
        self.clock = clock or FakeClock()
        self.counter = counter
        self.llm = llm
        self.system_prompt = system_prompt
        self.wake = list(wake) if wake is not None else wake_words(str(WAKE_WORDS_YAML))[0]
        self.speakers = FakeSpeakers()
        self.phrase_voices: Dict[str, Mapping[str, Any]] = {}  # биометрия фразы по её id (реестр хоста, #4141)
        self.voice_provider = ROBOT_DEFAULT_PROVIDER
        self.current_voice_id = ""
        self.touched: Set[str] = set()
        self._log = StepLog()
        self._utterances = 0
        self._memory_gen = 0
        self.memory = self._new_memory()
        self.ros_node = _FakeRosNode(self.emit, self.memory)
        self.speech = FakeSpeech(self.emit)
        self.session = Session(kn.AGENTS[0], clock=self.clock.now)
        self.backlog = Backlog()
        self.music_mem = MusicStateMemory()
        self.player = FakePlayer(self)
        self.player.stop()  # плеер при старте публикует latched-снимок «idle»: раздел music_state не «unknown»
        self.tools = FakeTools(self)
        self.ports = DryPorts(self, llm, counter, self.tools)
        self.events = open_events(str(self.workdir / "e2e_dialog.db"), "dry-marathon")
        self.engine = TurnEngine(self.session, self.ports, system_prompt=system_prompt, sections=self._sections,
                                 events=self.events, log=_EngineLog(self.emit), phrase_voice=self._phrase_voice)

    # -- служебное ------------------------------------------------------------------------------------------------

    def emit(self, line: str) -> None:
        self._log.add(line)

    def touch(self, owner: str) -> None:
        self.touched.add(owner)

    def _sections(self, snap: Any, _text: str) -> Mapping[str, Any]:
        return host_sections(snap, self.music_mem)

    async def _phrase_voice(self, uid: str, _wait_s: float) -> Optional[Mapping[str, Any]]:
        """Голос фразы по её id, как ``UtteranceSpeakerRegistry.resolve`` хоста: фейковая биометрия здесь приходит до
        STT, ждать нечего."""
        return self.phrase_voices.get(uid)

    def _new_memory(self) -> VoiceMemory:
        self._memory_gen += 1
        path = self.workdir / f"voice_memory_{self._memory_gen}.e2e.db"
        if self._memory_path is not None and self._memory_gen == 1:  # консоль ``--db``: память переживает запуск
            self._memory_path.parent.mkdir(parents=True, exist_ok=True)
            path = self._memory_path
        # Ollama в сухом прогоне нет: векторный поиск выключен (FTS5 работает), порт закрыт — отказ мгновенный
        return VoiceMemory(str(path), ollama_base_url="http://127.0.0.1:9", ollama_timeout=0.2)

    def reset_memory(self) -> None:
        """Акт, который пишет в память, начинается с чистой e2e-памяти (``activate_e2e_memory_db``, #2781, #2890)."""
        self.memory.close()
        self.memory = self._new_memory()
        self.ros_node.voice_memory = self.memory

    def reset_speakers(self) -> None:
        """Акт, который регистрирует дикторов, стирает каст (``activate_e2e_speaker_db``, #2750)."""
        self.speakers = FakeSpeakers()

    def reset_session(self) -> None:
        """``/dialog/control reset`` перед актом (``reset_dialog_v2_session_for_act``): эпоха + 1, окно ходов пусто."""
        self.session.reset()
        self.engine.log.reset()

    def close(self) -> None:
        if self.events is not None:
            self.events.close()
        self.memory.close()

    # -- вход -----------------------------------------------------------------------------------------------------

    def _utterance_msg(self, text: str, started: float, ended: float, uid: str, source: str = "respeaker") -> Any:
        def stamp(t: float) -> Any:
            return types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9))

        return types.SimpleNamespace(text=text, source=source, started_at=stamp(started), ended_at=stamp(ended),
                                     utterance_id=uid, chat_id=0)

    async def hear(self, text: str, voice_id: str = "", *, gap_s: float = STEP_GAP_S,
                   source: str = "respeaker") -> HearResult:
        """Реплика человека голосом ``voice_id``: STT → биометрия → адресация → (ход | бэклог). Как ``_on_utterance``
        и ``_on_speaker`` хоста ``dialog_node``, только с фейковой биометрией и моделью времени. ``source`` - откуда
        реплика (``address.SOURCE_*``): ``operator`` адресован без вейка - так говорит консоль ``scripts/dialogue``."""
        self._log = StepLog()
        self.touched = set()
        calls_before = self.counter.calls
        self.clock.advance(gap_s)
        started = self.clock.now()
        ended = self.clock.advance(len(text) / SPEECH_CPS)
        self._utterances += 1
        uid = f"dry-u{self._utterances}"
        self.current_voice_id = voice_id
        msg = self._utterance_msg(text, started, ended, uid, source)
        self.emit(f"🎤 STT: {log_label(msg)}")
        bio = self.speakers.result_for(voice_id) if voice_id else {"is_known": False}
        if bio.get("is_known"):
            self.touch("speaker")
            # speaker_id_node.py: f"👤 Speaker: '{match.name}' confidence={…:.3f} ({elapsed:.0f} ms)"
            self.emit(f"👤 Speaker: '{bio['name']}' confidence={bio['confidence']:.3f} (400 ms)")
        else:
            self.emit("👤 Speaker: unknown (400 ms)")  # speaker_id_node.py: f"👤 Speaker: unknown ({elapsed:.0f} ms)"
        person = str(bio.get("speaker_id") or "") if bio.get("is_known") else ""
        heard = heard_from(msg, now=ended, person_ref=person)
        self.phrase_voices[uid] = {**bio, "utterance_id": uid}
        self.session.set_speaker(speaker_ref(bio, now=ended, prev=self.session.snapshot.speaker))
        decision = decide_in_session(heard, self.backlog, self.wake, self.session.snapshot)
        if isinstance(decision, NotAddressed):
            return self._not_addressed(heard, decision, uid, bio)
        return await self._turn(decision, text, calls_before)

    def _not_addressed(self, heard: Any, decision: NotAddressed, uid: str, bio: Mapping[str, Any]) -> HearResult:
        self.backlog.add(heard)
        self.backlog.note_voice(uid, voice_tag(bio))
        # dialog_node.py: f"[backlog] accumulated reason={decision.reason} entries={…}"
        self.emit(f"[backlog] accumulated reason={decision.reason} entries="
                  f"{self.backlog.background_phrases(self.clock.now())}")
        self.clock.advance(BACKLOG_GAP_S)
        return HearResult(heard.text, False, decision.reason, log=self._log.text, touched=set(self.touched))

    async def _route(self, decision: Addressed) -> Any:
        """Выбор группы тулов классификатором до хода (вариант C); у других вариантов выбора нет. Ход, который закроет
        Tier-1 безопасности (стоп, тишина - код решает первым, до модели), Jev не зовёт."""
        if self.variant != uv.C:
            return None
        snap, safety = self.session.snapshot, uv.safety_match()
        for said in (decision.text, decision.command):
            command = safety(said, snap) if said else None
            if command is not None and not command.disputed:
                return None
        return await self.router.choose(decision.text, snap.music_playing)

    async def _turn(self, decision: Addressed, text: str, calls_before: int) -> HearResult:
        self.tools.calls.clear()
        self.speech.all.clear()
        route = await self._route(decision)
        offered: List[Tuple[str, ...]] = []
        with uv.installed(self.variant, offered, lambda: route.as_pair() if route is not None else None):
            agg = await self.engine.run(decision)
        spoken_s = self.speech.settle(self.engine, kn.AGENTS[0])
        self.clock.advance(spoken_s)
        log = self._log.text
        return HearResult(text, True, source=decision.source, aggregate=agg,
                          said=[s.text for s in self.speech.all], tools=[name for name, _ in self.tools.calls],
                          llm_calls=self.counter.calls - calls_before, log=log,
                          touched=set(self.touched), outcomes=self._outcomes(agg.turn_id), tier1=tier1_kind(log),
                          latency_ms=agg.latency_ms + int(getattr(route, "latency_ms", 0) or 0), tokens=agg.tokens,
                          offered=list(offered[0]) if offered else [], widened=widened_tools(log),
                          variant=self.variant, route=route, calls=[(n, dict(a)) for n, a in self.tools.calls],
                          asked_yes=_ASKED_YES_LINE.search(log) is not None)

    def _outcomes(self, turn_id: str) -> List[Tuple[str, str, Optional[str]]]:
        """Исходы действий хода — читателем e2e-БД, которым пользуются ``turn_metrics`` и ``honesty_audit``."""
        if self.events is None:
            return []
        turn = next((t for t in reversed(e2e_db.load(Path(self.events.path))) if t.turn_id == turn_id), None)
        return [(a.get("tool", ""), a.get("status", ""), a.get("reason")) for a in (turn.actions if turn else [])]


def new_workdir(prefix: str = "dry_marathon_") -> Path:
    """Временный каталог e2e-БД: вне боевой ``/data`` (``open_events`` откажет иначе)."""
    return Path(tempfile.mkdtemp(prefix=prefix))


__all__ = ["DryPorts", "DryStand", "FakeClock", "LiveClock", "FakeSpeakers", "FakeSpeech", "FakeTools", "HearResult", "NO_FAKE",
           "StepLog", "TOOL_OWNERS", "ensure_paths", "load_memory_tools", "load_tool_module", "new_workdir",
           "tier1_kind",
           "widened_tools"]
