"""Сухой стенд хода v2: настоящий ``TurnEngine`` без ROS и без робота (эпик #3683, карточка #3906).

Робот — узкое место: акт марафона идёт 5–15 мин, деплой выбивает стенд на 15–40 мин. Этот модуль прогоняет текст
реплики теми же функциями, что на роботе: ``address.decide_in_session`` (вейк, бэклог) → ``TurnEngine.run``
(Tier-1 / команда LLM → исполнение → фраза кода) → окно озвученного. Настоящими остаются: грамматика и адресация,
сессия, бэклог с авторством, срез тулов, цепочка LLM с мостом команд (``dialog_v2_ports.assemble_llm``), исполнитель
действий (``V2Ports`` + ``Executor``), тулы памяти (``rob_box_mcp_tools.tools.memory`` поверх настоящего
``VoiceMemory`` во временной e2e-БД), раздел ``speaker_memory`` хода (настоящий ``SpeakerMemory`` хоста, #3919).

Фейковые владельцы, которые отвечают событием сразу (:class:`FakeTools`, :class:`FakeSpeech`, :class:`FakeSpeakers`,
:class:`FakePlayer`, :class:`FakeLibrary`): речь, музыка (дека, сет, библиотека), голос/темп/громкость, лицо,
биометрия диктора, статус робота. Что они знают о мире — таблицей здесь; в
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
    BARE_WAKE, Addressed, NotAddressed, decide_in_session, heard_from, log_label, wake_words,
)
from rob_box_dialog.backlog import Backlog  # noqa: E402
from rob_box_dialog.e2e_events import inside_robot_data, open_events  # noqa: E402
from rob_box_dialog.memory import profiled_person  # noqa: E402
from rob_box_dialog.music_state import MusicStateMemory  # noqa: E402
from rob_box_dialog.session import MusicStateRef, Session  # noqa: E402
from rob_box_dialog.speaker_adapter import speaker_ref, voice_tag  # noqa: E402
from rob_box_dialog.turn import ASK_MARK, LOG_MARK, TurnEngine, host_sections  # noqa: E402
from rob_box_llm.provider import ToolResult  # noqa: E402
from rob_box_mcp_tools.voice_request import VoiceOutcome, outcome_data  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports  # noqa: E402
from rob_box_voice.core.music_player_state import build_music_state_payload, parse_music_state  # noqa: E402
from rob_box_voice.core.speaker_memory import SpeakerMemory  # noqa: E402
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

    @staticmethod
    def _music_title(args: Dict[str, Any]) -> str:
        return str(args.get("title") or args.get("text") or args.get("query") or args.get("prompt")
                   or args.get("theme") or args.get("style") or "трек")

    def _t_request_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        title = self._music_title(args)
        return True, {"status": "done", "event": "started", "track_id": self._stand.player.request(title),
                      "title": title}

    def _t_generate_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """Генерация: трек ложится в библиотеку и играет (мост ожидания на роботе играет сразу, песня - следом)."""
        title = self._music_title(args)
        track = self._stand.library.add(title, kind=str(args.get("kind") or "instrumental"),
                                        genre=str(args.get("style") or ""))
        deck = self._stand.player.library(track["id"], title)
        return True, {"status": "done", "event": "started", "track_id": deck, "library_id": track["id"],
                      "title": title}

    def _t_dj_set(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """``DjSetTool``: ``stop`` - сет, а без сета - дека (``was_playing`` - была ли музыка, по нему фраза
        ``knowledge.DJ_DONE``); ``start`` - новый сет снимает идущий."""
        player, action = self._stand.player, str(args.get("action") or "start")
        if action == "stop":
            if not player.set_running:
                result = player.stop("user_stop")
                return True, {**result, "was_playing": result["track_id"] is not None}
            return True, {**player.close_set("user_stop"), "was_playing": True}
        if action != "start":
            return False, {"status": "rejected", "reason": f"action={action!r}: есть только start и stop"}
        theme = str(args.get("theme") or "")
        return True, {**player.start_set(theme), "theme": theme}

    def _t_music_library(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """``MusicLibraryTool`` над :class:`FakeLibrary`: list / search / info / play. Играет только ``play`` (у
        живого ``starts_music`` - флаг одного действия). Правки библиотеки стенд не ведёт - честный ``no_fake_owner``."""
        library, action = self._stand.library, str(args.get("action") or "")
        track_id = str(args.get("track_id") or "")
        if action == "list":
            tracks = library.list_tracks(int(args.get("limit") or 20))
            return True, {"tracks": tracks, "total": library.count, "shown": len(tracks),
                          "sort_by": args.get("sort_by") or "recent"}
        if action == "search":
            query = str(args.get("query") or "")
            tracks = library.search_tracks(query, int(args.get("limit") or 5))
            return True, {"query": query, "tracks": tracks, "total": len(tracks)}
        if action in ("info", "play") and not track_id:
            return False, {"status": "rejected", "reason": f"для '{action}' нужен track_id (из list / search)"}
        if action == "info":
            info = library.info(track_id)
            return (True, info) if info else (False, {"status": "rejected", "reason": "unknown_track",
                                                      "track_id": track_id})
        if action == "play":
            track = library.get(track_id)
            if track is None:
                return False, {"status": "rejected", "reason": "track_not_found", "track_id": track_id}
            deck = self._stand.player.library(track_id, track["title"])
            return True, {"ok": True, "track_id": deck, "library_id": track_id, "title": track["title"],
                          "duration_s": track["duration_ms"] / 1000.0}
        return False, {"status": "rejected", "reason": NO_FAKE, "tool": "music_library", "action": action}

    def _t_stop_music(self, args: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """``StopMusicTool``: с ``track_id`` - снять только его; иначе сет и дека через владельцев
        (``MCPServer._stop_deck``), а в тишине - явный стоп деки всё равно (строка ``track_id=None``, как у живого).
        ``was_playing`` - играло ли до стопа: по нему диалог выбирает фразу (``knowledge.TOOL_PHRASES``)."""
        player = self._stand.player
        track_id = str(args.get("track_id") or "")
        if track_id:
            if track_id != player.track_id:
                return True, {"was_playing": False, "stopped": False, "track_id": track_id}
            player.stop("user_stop")
            return True, {"was_playing": True, "stopped": True, "track_id": track_id}
        was_playing = player.playing or player.set_running
        player.close_set("user_stop")
        if player.playing:
            player.stop("user_stop")
        if not was_playing:
            player.stop("user_stop")
        return True, {"was_playing": was_playing}


#: Темп и форма трека на деке стенда: поля строки ``started`` (звука нет, значения - только форма строки).
STAND_BPM = 100.0
STAND_FORM_BEATS = 64.0


class FakePlayer:
    """Владелец плеера v2 (``engine/player_owner.py`` ``PlayerOwner`` и сет ``engine/session.py`` ``SetSession``):
    одна дека, что на ней и идёт ли сет. Строки - в форматах живого и в тех же случаях: ``started`` - трек встал на
    деку; ``stop`` - на КАЖДЫЙ стоп владельца, в тишине тоже (``track_id=None``, живой 26.10 20:57:50); стоп сета -
    строка сета, затем стоп деки. id треков - как у живого: сет ``setNNNNN:01:A``, заказ ``reqNNNNN:01:A``, трек
    библиотеки ``gen_<id8>_<время>``. Каждое изменение - снимок ``/voice/music/state`` в сессию и память музыки."""

    def __init__(self, stand: "DryStand") -> None:
        self._stand = stand
        self._track: Optional[str] = None
        self._title: Optional[str] = None
        self._dj: Dict[str, Any] = {"enabled": False}
        self._set_id: Optional[str] = None
        self._n = 0

    @property
    def track_id(self) -> Optional[str]:
        return self._track

    @property
    def playing(self) -> bool:
        return self._track is not None

    @property
    def set_running(self) -> bool:
        return self._set_id is not None

    def busy(self) -> bool:
        """Играет трек или включён сет - как «busy» предусловия акта харнесса (``_e2e_music_stop_py``)."""
        return self.playing or bool(self._dj.get("enabled"))

    def deck(self) -> Dict[str, Any]:
        return {"playing": self.playing, "track_id": self._track, "title": self._title, "dj": dict(self._dj)}

    def _seq(self) -> int:
        self._n += 1
        return self._n

    def play(self, track_id: str, title: str, dj: Optional[Mapping[str, Any]] = None) -> str:
        """``PlayerOwner.play`` и его ``started``: трек на деке, строка старта, снимок."""
        self._track, self._title = track_id, title
        self._dj = {**(dj or {}), "enabled": bool((dj or {}).get("enabled"))}
        # player_owner.py: f"🎵 [music v2] started track_id={track_id} " + поля старта ``k=v`` через пробел
        self._stand.emit(f"🎵 [music v2] started track_id={track_id} clock_beat=0.0 start_beat=0.0 phase_in_form=0 "
                         f"late_beats=0.0 bpm={STAND_BPM} deck=A form_beats={STAND_FORM_BEATS} players_aligned=True "
                         f"latency_s=0.0")
        self.publish()
        return track_id

    def start(self, title: str) -> str:
        """Трек на деке без сета (как заказ ``request_music``)."""
        return self.request(title)

    def request(self, title: str) -> str:
        """``RequestMusicTool``: идущий сет закрывается (``close_set("request_music")``), заказ - на деку."""
        self.close_set("request_music")
        return self.play(f"req{self._seq():05d}:01:A", title, {"title": title, "source": "synth"})

    def library(self, library_id: str, title: str) -> str:
        """``engine/gen/library_play.py``: идущий сет закрывается (``music_library``), трек кэша - на деку."""
        self.close_set("music_library")
        self._seq()
        return self.play(f"gen_{library_id[:8]}_{int(self._stand.clock.now())}", title,
                         {"title": title, "source": "generated", "library_id": library_id})

    def start_set(self, theme: str) -> Dict[str, Any]:
        """``DjSetTool._start``: идущий сет снимается (``new_set``), новый ставит свой первый трек."""
        self.close_set("new_set")
        set_id = f"set{self._seq():05d}"
        self._set_id = set_id
        track = self.play(f"{set_id}:01:A", f"сет «{theme}»" if theme else "диджей-сет",
                          {"enabled": True, "theme": theme, "source": "synth"})
        return {"ok": True, "track_id": track, "set_id": set_id}

    def close_set(self, reason: str) -> Optional[Dict[str, Any]]:
        """``DjSetTool.close_set`` → ``SetSession.stop``: строка сета, затем стоп деки. Сета нет - ничего."""
        if self._set_id is None:
            return None
        set_id, self._set_id = self._set_id, None
        # session.py: f"🎧 [set v2] {self.set_id} stop reason={reason}"
        self._stand.emit(f"🎧 [set v2] {set_id} stop reason={reason}")
        return {**self.stop(reason), "set_id": set_id}

    def stop(self, reason: str = "user_stop") -> Dict[str, Any]:
        """``PlayerOwner.stop``: дека пуста, строка ``stop`` всегда (в тишине - ``track_id=None``), снимок ``idle``."""
        last = self._track
        self._track = self._title = None
        self._dj = {"enabled": False}
        # player_owner.py: f"🎵 [music v2] stop track_id={last} reason={reason}"
        self._stand.emit(f"🎵 [music v2] stop track_id={last} reason={reason}")
        self.publish()
        return {"ok": True, "track_id": last}

    def publish(self) -> None:
        """Latched-снимок ``/voice/music/state`` (после старта, стопа и при запуске ноды)."""
        stand = self._stand
        payload = build_music_state_payload(playing=self.playing, track_id=self._track, dj=dict(self._dj),
                                            ts=stand.clock.now())
        snap = parse_music_state(payload)
        stand.music_mem.observe_state(snap, now=stand.clock.now())
        stand.music_mem.observe_track_name(self._title)
        stand.session.set_music(MusicStateRef("playing" if self._track else "idle", track_id=self._track,
                                              title=self._title))


#: Что лежит в библиотеке сгенерированной музыки робота до прогона (``/data/music_library``): знание фейка о мире.
LIBRARY_SEED: Tuple[Mapping[str, Any], ...] = (
    {"title": "Дождь над мастерской", "kind": "instrumental", "genre": "ambient", "mood": "спокойное",
     "tags": ("дождь", "ночь")},
    {"title": "Ночная пайка", "kind": "instrumental", "genre": "lofi", "mood": "уютное", "tags": ("мастерская",)},
)
#: Поля строки списка, как ``gen_library._brief`` (без текстов песен и служебных полей).
LIBRARY_BRIEF = ("id", "title", "kind", "genre", "mood", "tags", "rating", "play_count", "duration_ms", "created_at",
                 "last_played_at")


class FakeLibrary:
    """Владелец библиотеки сгенерированной музыки (``tools/gen_library.py``): список, поиск, подробности трека.
    Генерация кладёт трек сюда. ``info`` - только по id, который библиотека сама назвала в ответе list / search
    (``NamedTracks``, #4025); иначе ``unknown_track``."""

    def __init__(self, clock: FakeClock) -> None:
        self._clock = clock
        self._tracks: List[Dict[str, Any]] = []
        self._named: Set[str] = set()
        for seed in LIBRARY_SEED:
            self.add(**seed)

    @property
    def count(self) -> int:
        return len(self._tracks)

    def add(self, title: str, kind: str = "instrumental", genre: str = "", mood: str = "",
            tags: Sequence[str] = ()) -> Dict[str, Any]:
        track = {"id": str(uuid.uuid4()), "title": title, "kind": kind, "genre": genre, "mood": mood,
                 "tags": list(tags), "rating": 0, "play_count": 0, "duration_ms": 150000,
                 "created_at": self._clock.now(), "last_played_at": None}
        self._tracks.insert(0, track)  # recent - свежие первыми
        return track

    def get(self, track_id: str) -> Optional[Dict[str, Any]]:
        return next((t for t in self._tracks if t["id"] == track_id), None)

    def _named_brief(self, tracks: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        self._named.update(t["id"] for t in tracks)
        return [{k: t.get(k) for k in LIBRARY_BRIEF} for t in tracks]

    def list_tracks(self, limit: int = 20) -> List[Dict[str, Any]]:
        return self._named_brief(self._tracks[:max(1, min(100, limit))])

    def search_tracks(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Подстрока без регистра по названию, жанру, настроению, тегам (у живого - и по prompt / lyrics / notes)."""
        needle = query.strip().lower()
        found = [t for t in self._tracks if needle and needle in " ".join(
            [t["title"], t["genre"], t["mood"], *t["tags"]]).lower()]
        return self._named_brief(found[:max(1, min(20, limit))])

    def info(self, track_id: str) -> Optional[Dict[str, Any]]:
        track = self.get(track_id) if track_id in self._named else None
        return {**track, "success": True} if track else None


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
    #: Голос каждой реплики ``said`` (``""`` - голос робота/рассказчика; иначе голос роли, его дал код).
    voices: List[str] = field(default_factory=list)
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
        self.speaker_memory = self._new_speaker_memory()
        self._turn_memory: Tuple[Optional[str], Optional[Mapping[str, Any]]] = (None, None)
        self.ros_node = _FakeRosNode(self.emit, self.memory)
        self.speech = FakeSpeech(self.emit)
        self.session = Session(kn.AGENTS[0], clock=self.clock.now)
        self.backlog = Backlog()
        self.music_mem = MusicStateMemory()
        self.library = FakeLibrary(self.clock)
        self.player = FakePlayer(self)
        self.player.publish()  # плеер при старте публикует latched-снимок «idle»: раздел music_state не «unknown»
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
        """Разделы снимка, как ``dialog_node._sections``: собеседник, музыка и ``speaker_memory`` хода (#3919)."""
        return host_sections(snap, self.music_mem, (), self._turn_memory)

    async def _note_speaker(self, decision: Addressed) -> None:
        """Начало хода, как ``dialog_node._note_speaker`` (#3919): подтверждённому собеседнику - его профиль и факты
        из e2e-памяти в раздел ``speaker_memory`` снимка (настоящий ``SpeakerMemory``, строка ``🧠 [speaker-memory]``)."""
        speaker = self.session.snapshot.speaker
        counts = decision.source != BARE_WAKE and bool(decision.text.strip())  # пустой ход - не диалог (как у v1)
        section = await self.speaker_memory.section(speaker) if counts else None
        self._turn_memory = (profiled_person(speaker), section)

    def _new_speaker_memory(self) -> SpeakerMemory:
        """Память собеседника хоста в режиме e2e: профиль - в памяти процесса, факты - из e2e-памяти стенда."""
        return SpeakerMemory("", "", e2e_facts_db=str(self.memory.db_path), is_e2e=lambda: True,
                             log=_EngineLog(self.emit))

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
        """Чистая e2e-память фактов (``enter_e2e_mode`` харнесса стирает e2e-БД, #2781, #2890); профили собеседников
        хоста в e2e - тоже заново (``dialog_node._apply_e2e_mode`` → ``SpeakerMemory.reset_e2e``)."""
        self.memory.close()
        self.memory = self._new_memory()
        self.ros_node.voice_memory = self.memory
        self.speaker_memory = self._new_speaker_memory()

    def reset_speakers(self) -> None:
        """Чистая e2e-БД дикторов: каст стёрт (``enter_e2e_mode`` харнесса, #2750)."""
        self.speakers = FakeSpeakers()

    def stop_music_for_act(self) -> str:
        """Предусловие акта харнесса (``ensure_music_stopped_for_act``, #3871/#4027): играет трек или сет - стоп теми
        же тулами, что у харнесса (``dj_set stop``, затем ``stop_music``). Строки стопа - в лог харнесса, не шага.
        Вердикт - как у харнесса: ``was_idle`` | ``stopped``."""
        self._log = StepLog()
        if not self.player.busy():
            return "was_idle"
        self.tools._t_dj_set({"action": "stop"})  # noqa: SLF001 - вызов тула мимо хода, как /mcp/execute харнесса
        self.tools._t_stop_music({})  # noqa: SLF001
        return "stopped"

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
        await self._note_speaker(decision)
        with uv.installed(self.variant, offered, lambda: route.as_pair() if route is not None else None):
            agg = await self.engine.run(decision)
        spoken_s = self.speech.settle(self.engine, kn.AGENTS[0])
        self.clock.advance(spoken_s)
        log = self._log.text
        return HearResult(text, True, source=decision.source, aggregate=agg,
                          said=[s.text for s in self.speech.all], voices=[s.voice for s in self.speech.all],
                          tools=[name for name, _ in self.tools.calls],
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


__all__ = ["DryPorts", "DryStand", "FakeClock", "LiveClock", "FakeLibrary", "FakePlayer", "FakeSpeakers", "FakeSpeech",
           "FakeTools", "HearResult", "NO_FAKE",
           "StepLog", "TOOL_OWNERS", "ensure_paths", "load_memory_tools", "load_tool_module", "new_workdir",
           "tier1_kind",
           "widened_tools"]
