#!/usr/bin/env python3
"""
mcp_server.py - MCP Server для предоставления инструментов LLM

Эта нода:
1. Регистрирует все доступные MCP инструменты
2. Публикует список инструментов в OpenAI Tool Calls формате (совместимо с DeepSeek, Qwen, и др.)
3. Принимает запросы на выполнение инструментов
4. Возвращает результаты выполнения

ROS 2 интерфейс:
- Публикует: /mcp/tools (String) - JSON список доступных инструментов
- Подписывается на: /mcp/execute (String) - JSON запросы на выполнение
- Публикует: /mcp/result (String) - JSON результаты выполнения

Режим e2e (#3904) — своего переключателя у ноды НЕТ: она последователь
владельца ``/speaker_id_node e2e_mode`` (``rob_box_core.e2e_mode``) и
переключает активную БД долгосрочной памяти (voice_facts, ``memory_save``)
боевая ↔ e2e_db_path по latched ``/e2e/mode``. Применённый режим — в
параметре ``e2e_mode_applied`` (только чтение, его сверяет харнесс). См.
``_apply_e2e_mode``. Реализовано только для legacy VoiceMemory
(``/data/voice_memory.db``, активный бэкенд в проде на 22.09.2026) — при
``MCP_USE_HARNESS_VOICE_MEMORY=1`` переключение отказывает (fail-fast),
``e2e_mode_applied`` остаётся False и харнесс не прогоняет акт.

Parameters:
    e2e_db_path (str) — изолированная БД памяти для E2E-режима
        [/data/voice_memory.e2e.db]
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from rob_box_dialog import knowledge as _dialog_kn
from rob_box_dialog_msgs.msg import SpeakerResult
from rob_box_core.e2e_mode import E2EModeFollower
from rob_box_core.speaker_wire import NAME_CONFIRMED, name_state, speaker_result_to_payload
import asyncio
import json
import math
import os
import threading
from typing import Any, Callable, Dict, Optional, Tuple

# Issue #2442 — единый шов «Встреча» вместо самостоятельного
# ``current_speaker_id``. См. ``_on_speaker_result`` ниже и ADR-0105 §3.
from rob_box_harness.encounter import EncounterSeam, EncounterChannel
from rob_box_harness.identity import Acquaintance, MemoryIdentitySeam
from rob_box_harness.memory import InMemoryStore
from rob_box_music import knowledge as music_kn
from rob_box_music.diversity import MusicHistory

from rob_box_voice.core.music_player_state import (
    MUSIC_EVENT_TOPIC,
    MUSIC_STATE_TOPIC,
    MusicEventLog,
)

from .engine.backing_deck import BackingDeck
from .engine.chooser_model import DEFAULT_MODE as CHOOSER_DEFAULT_MODE
from .engine.gen.kie_suno import KieSunoProvider
from .engine.gen.song import STARTED_WAIT_S, SongStudio, current_voice, harness_lyrics_ask
from .engine.player_owner import PlayerOwner
from .engine.renardo_adapter import V2_CLOCK_LATENCY_S, RenardoAdapter
from .engine.renardo_runtime import RenardoRuntime
from .engine.speech_duck import SpeechDuck, duck_controls
from .engine.reasoner import DEADLINE_S as V2_REASONER_DEADLINE_S, SetReasoner
from .engine.request_slots import harness_ask
from .engine.tools_v2 import DjSetTool, GenerateMusicTool, RequestMusicTool, music_busy
from .registry import MCPToolRegistry
from .tools import (
    NavigateToWaypointTool,
    NavigateToCoordinatesTool,
    MoveDirectionTool,
    StopNavigationTool,
    ListWaypointsTool,
    SaveWaypointTool,
    DeleteWaypointTool,
    ClearWaypointsTool,
    GetCurrentPoseTool,
    SetVolumeTool,
    SetPitchTool,
    SetSpeedTool,
    GetRobotStatusTool,
    GetCurrentTimeTool,
    GetPerceptionContextTool,
    GetBatteryLevelTool,
    StartMappingTool,
    ContinueMappingTool,
    FinishMappingTool,
    OptimizeMapTool,
    LoadMapTool,
    PlayAnimationTool,
    PlaySoundTool,
    GetSoundInfoTool,
    SpeakTextTool,
    ListenForResponseTool,
    EstimateTtsDurationTool,
    RegisterSpeakerTool,
    SetVoiceTool,
    # Issue #1765 — cross-provider TTS switching.
    SetTtsProviderTool,
    ListTtsVoicesTool,
    MemorySaveTool,
    MemorySearchTool,
    MemoryContextTool,
    RtttlLibrary,
    StopMusicTool,
    GetMusicStateTool,
    SetMusicVolumeTool,
    LookupMelodyTool,
    SearchMelodyTool,
    AddMusicMaterialTool,
    FaqSearchTool,
    SearchWebTool,
    # Issue #2113 — TARS 2 metrics panel (operator.admin). Публикует
    # запрос в /avatar/tars/panel_request; URL собирает TarsPanelDispatcher
    # в rob_box_supervisor и публикует в /avatar/tars/panel_url.
    ShowMetricsTool,
    # PR-13b #3708 (решение Шифу 23.10 «регаем, чтобы из ТАРС было доступно»): диагностика operator.admin.
    ContainerStatusTool,
    ReadLogsTool,
    Ros2NodeStatusTool,
    SayAloudTool,
    SayTool,
)

# Issue #1392 / ADR-0158 — library of generated tracks (cache) + music_library tool.
# Imported explicitly (not via wildcard) so the heavy ``tools.music``
# dependency tree stays opt-in.
try:
    from .engine.gen.library import GeneratedMusicLibrary
    from .engine.gen.library_play import play_library_track
    from .engine.gen.quota import MIB, enforce_quota
    from .tools.gen_library import MusicLibraryTool
    _GEN_LIBRARY_AVAILABLE = True
except ImportError as _exc:  # noqa: BLE001
    GeneratedMusicLibrary = None  # type: ignore[assignment,misc]
    MusicLibraryTool = play_library_track = enforce_quota = MIB = None  # type: ignore[assignment,misc]
    _GEN_LIBRARY_AVAILABLE = False
    _GEN_LIBRARY_IMPORT_ERROR = str(_exc)
from .mcp_auth import RequestAuthenticator
from .slice_authority import ToolSliceAuthority, load_default_authority
from .waypoint_store import WaypointStore
from .waypoint_adapter import WaypointAdapter
from .mapping_state import MappingState
from rob_box_core.provider_publisher import ProviderStatePublisher
from .engine.gen.provider import gen_provider_states
from .perception_projection import project_perception_event

try:
    from rob_box_voice.core.voice_memory import VoiceMemory as _VoiceMemory
    from rob_box_voice.core.faq_store import FAQStore as _FAQStore
    from rob_box_voice.core.event_profile import load_event_profile as _load_event_profile
except ImportError:
    _VoiceMemory = None  # type: ignore[assignment,misc]
    _FAQStore = None
    _load_event_profile = None

# Issue #2000 / ADR-0055 — Phase 1 path consolidation. ONLY ``self.voice_memory``
# (long-term facts, see ``_init_voice_memory`` below) can share
# /data/harness_voice.db with the dialogue node, via VoiceMemoryAdapter (sync
# facade over SQLiteVoiceMemory) — gated by this env var so production can
# roll back to the legacy VoiceMemory without code edits.
#
# WaypointStore and FAQStore are NOT part of
# this switch and stay on ``VOICE_MEMORY_DB_PATH`` (default
# /data/voice_memory.db) regardless of this flag. Investigated for Phase 2
# (issue #2000) and rejected as a plain path swap: SQLiteVoiceMemory already
# defines its own ``waypoints`` (name TEXT PRIMARY KEY) and ``faq_items``
# (created_at, no FTS5) tables in harness_voice.db, which collide by name
# with incompatible schemas against WaypointStore's ``waypoints`` (id,
# map_id NOT NULL FK -> maps) and the legacy FAQStore's ``faq_items``
# (indexed_at, FTS5 triggers). ``CREATE TABLE IF NOT EXISTS`` would silently
# keep whichever schema got created first and every write from the other
# store would then fail with "no such column". See the comment on
# ``sqlite_db_path`` in docker/sound/config/voice_assistant/dialogue_node.yaml
# and docs/adr/0055-voice-memory-db-unify-with-harness.md (§1.3, §3) for the
# full analysis. Merging them needs a schema-reconciling adapter (like this
# one, but for waypoints/faq), not a default-value edit.
_USE_HARNESS_VOICE_MEMORY = os.getenv("MCP_USE_HARNESS_VOICE_MEMORY", "0").strip().lower() in (
    "1",
    "true",
    "yes",
)
try:
    if _USE_HARNESS_VOICE_MEMORY:
        from rob_box_harness.memory.voice_memory_adapter import (
            VoiceMemoryAdapter as _VoiceMemoryAdapter,
        )
    else:
        _VoiceMemoryAdapter = None  # type: ignore[assignment,misc]
except ImportError:
    _VoiceMemoryAdapter = None  # type: ignore[assignment,misc]


def _string_publisher(publisher: Any) -> Any:
    """``text -> publisher.publish(String(data=text))`` — публикация для ``PlayerOwner``."""
    def publish(text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)
    return publish


#: Сколько тул v2 ждёт ``started`` своего трека, прежде чем ответить ``ok`` (A2 p100 ≤ 6 с, A14). Одно знание с
#: дедлайном действия музыки в диалоге (``rob_box_dialog.knowledge.MUSIC_ACTION_DEADLINE_S`` = это + подготовка).
V2_STARTED_WAIT_S = _dialog_kn.MUSIC_START_WAIT_S


def _teed_events(publish: Callable[[str], None], log: MusicEventLog) -> Callable[[str], None]:
    """Публикация ``/voice/music/event`` + тот же поток в процессе: тулы v2 ждут ``started`` (ADR-0149 PR-6)."""
    def publish_event(text: str) -> None:
        publish(text)
        log.observe_json(text)
    return publish_event


def _set_reasoner(node: Any) -> SetReasoner:
    """PR-10: ризонер плана сета по параметрам ноды; ``music_v2_reasoner: false`` — ноль вызовов LLM."""
    return SetReasoner(enabled=bool(node.get_parameter("music_v2_reasoner").value),
                       deadline_s=float(node.get_parameter("music_v2_reasoner_deadline_s").value),
                       logger=node.get_logger())


def _speaker(node: Any) -> Callable[[str], None]:
    """Реплика диджея на переходе (§12 В2) — тем же ``speak_text``, что у LLM: один путь в TTS (SSML строит тул)."""
    return lambda line: node.registry.execute("speak_text", text=line)


def _tts_provider(node: Any) -> str:
    """Провайдер TTS, как у ``speak_text``: фактический от tts_node, пока его нет — параметр ``tts_provider``."""
    from .tools.dialogue import _active_tts_provider

    return _active_tts_provider(node)


def _song_studio(node: Any, events: Any) -> Optional[SongStudio]:
    """Песня голосом робота (ADR-0158 PR-7): нужны ключ KIE и LLM для текстов; нет — ``None`` (``not_wired`` во фразе).
    Голос - провайдер и голос из ``/voice/tts/provider_state`` (то, что реально звучит; читаем лениво)."""
    provider, lyrics = KieSunoProvider(), harness_lyrics_ask(log=node.get_logger)
    if lyrics is None or not provider.configured:
        node.get_logger().warning("🎤 песня голосом робота выключена: нужны KIE_API и ключ llm_harness")
        return None
    return SongStudio(provider, lambda: getattr(node, "_generated_music_library", None), lyrics,
                      lambda: current_voice(_tts_provider(node), getattr(node, "actual_tts_voice", None)),
                      play=_play_cached(node),
                      speak=_speaker(node), started=lambda track_id: events.wait(track_id, STARTED_WAIT_S), log=node.get_logger)


def _play_cached(node: Any) -> Any:
    """Шов «сыграть трек кэша» для внутренних исполнителей (путь, песня, кавер, сет; ADR-0158 PR-9): функция
    ``play_library_track`` на кэше ноды, а не тул по реестру. Кэш поднимается позже движка — читаем лениво.
    ``hold`` — мост ожидания песни (петля с кроссфейдом через ``BridgeHold``, ``tools_v2.hold_start``)."""
    def play(track_id: str, hold: Any = None) -> Any:
        library = getattr(node, "_generated_music_library", None)
        if library is None:
            from .base import MCPToolResult

            return MCPToolResult(success=False, error="библиотека генераций недоступна — трек не заиграл")
        return play_library_track(node, library, track_id, hold=hold)

    return play


def _cover_maker(node: Any) -> Any:
    """Кавер пьесы (ADR-0158 PR-4): Suno через KIE (ключ ``KIE_API`` окружения), кэш — ``/data/music_library``
    (поднимается позже движка — лениво). Ключа нет — исполнитель есть, но не готов: кавер-кандидатов нет."""
    from .engine.gen.cover import CoverMaker
    from .engine.gen.kie_suno import KieSunoProvider

    return CoverMaker(KieSunoProvider(), lambda: getattr(node, "_generated_music_library", None),
                      log=node.get_logger)


def _instrumental_maker(node: Any, cover: Any) -> Any:
    """Одиночный инструментал («сгенерируй рок»): тот же KIE и кэш, баланс — последний известный у кавера. Ключа
    нет — исполнитель есть, но не готов: кандидата генерации нет."""
    from .engine.gen.instrumental import InstrumentalMaker

    return InstrumentalMaker(KieSunoProvider(), lambda: getattr(node, "_generated_music_library", None),
                             balance=cover.balance, log=node.get_logger)


def _gen_set_maker(node: Any, cover: Any) -> Any:
    """Сет из генерации (ADR-0158 PR-8): тот же KIE, тот же кэш, баланс — последний известный у кавера; путь сета
    выбирает тот же чойсер (``music_chooser``). Ключа нет — исполнитель есть, но не готов: генерации в сете нет."""
    from .engine.chooser_model import chooser_from_env
    from .engine.gen.gen_session import GenSetMaker

    chooser = chooser_from_env(str(node.get_parameter("music_chooser").value), log=node.get_logger)
    return GenSetMaker(KieSunoProvider(), lambda: getattr(node, "_generated_music_library", None), chooser,
                       balance=cover.balance, log=node.get_logger, cover=cover)


def _attach_player_owner_v2(node: Any, runtime: Any) -> Optional[PlayerOwner]:
    """ADR-0149 §9, PR-4b — владелец плеера v2 (с PR-13b — единственный музыкальный путь).

    ``PlayerOwner`` — единственный писатель ``/voice/music/state`` и
    ``/voice/music/event``: latched-публикатор снимка уходит владельцу, у ноды
    ``music_state_pub`` становится ``None``. Renardo поднимает ``RenardoRuntime`` (#3614):
    адаптер берёт его контекст, палитру, подтверждённую сервером, ``send_osc``, а
    ``/fail`` scsynth уходит владельцу. Старый путь удалён (PR-13, #3614), движок один.
    """
    if runtime is None:
        node.get_logger().error("❌ RenardoRuntime не создан — музыкальный движок v2 выключен")
        return None
    latency = float(node.get_parameter("music_v2_clock_latency").value)
    event_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    node.music_event_pub = node.create_publisher(String, MUSIC_EVENT_TOPIC, event_qos)
    adapter = RenardoAdapter(runtime.context, runtime.known_synth_names, runtime.send_osc, latency_s=latency)
    events = MusicEventLog()
    owner = PlayerOwner(adapter, _string_publisher(node.music_state_pub),
                        _teed_events(_string_publisher(node.music_event_pub), events), logger=node.get_logger())
    runtime.osc_fail_listener = owner.on_server_fail
    node.music_state_pub = None  # писатель снимка — только владелец

    def confirm(track_id: Optional[str]) -> Any:
        return events.wait(track_id, V2_STARTED_WAIT_S)

    node._music_confirm = confirm  # play_library_track: трек кэша на деке, ok — по started (ADR-0158 PR-2)

    history = MusicHistory()  # music_history: хуки и оси недавних сетов переживают перезапуск (I17, #3399)
    history.announce(node.get_logger())
    dj_set = DjSetTool(node, owner, confirm=confirm, reasoner=_set_reasoner(node), speak=_speaker(node),
                       history=history, tracks_dir=str(node.get_parameter("music_v2_tracks_dir").value),
                       lines=bool(node.get_parameter("music_v2_dj_lines").value))
    node.registry.register(dj_set)  # PR-5: сет v2 — SetSession поверх владельца
    node._dj_set_tool = dj_set  # стоп деки закрывает и сет (MCPServer._stop_deck)
    cover = _cover_maker(node)  # PR-4: кавер пьесы; его баланс KIE — и сету из генерации
    dj_set.gen_sets = _gen_set_maker(node, cover)  # ADR-0158 PR-8: рок/гранж-сет — кэш и генерация
    # PR-6: одиночный club-трек v2; PR-11: classic-песня v2 (мелодия по названию)
    # ADR-0157 PR-2: одиночная пьеса читает и пишет ту же music_history (свежесть пьес между заказами)
    request = RequestMusicTool(node, owner, dj_set, confirm=confirm, history=history,
                               tracks_dir=str(node.get_parameter("music_v2_tracks_dir").value),
                               chooser_mode=str(node.get_parameter("music_chooser").value),
                               slots_ask=harness_ask(log=node.get_logger),  # #3609: LLM-разбор фразы в слоты
                               # ADR-0158 PR-3: кэш генераций поднимается позже (_register_gen_library_tools) —
                               # читаем лениво; трек кэша играет play_library_track на деке (PR-2)
                               gen_library=lambda: getattr(node, "_generated_music_library", None),
                               play_cached=_play_cached(node),
                               cover=cover, speak=_speaker(node),  # PR-4: кавер пьесы
                               songs=_song_studio(node, events),  # PR-7: песня голосом робота
                               instrumental=_instrumental_maker(node, cover))  # «сгенерируй рок»
    # ADR-0150 PR-7c: подложка выступления — request_music(intent=backing); outro — stop_music(boundary=bar)
    request.backing = BackingDeck(owner, adapter, lambda mood: request._stage_club("", mood),  # noqa: SLF001
                                  confirm, lambda: music_busy(owner, dj_set), log=node.get_logger())
    node._backing_deck = request.backing
    node.registry.register(request)
    node.registry.register(GenerateMusicTool(node, request))  # ADR-0158 §3.3: просьба, тот же путь
    request.warm()  # #3577: индекс партитур и каталог — в фоне до первого заказа (бюджет старта 2.5 с)
    owner.publish_state()
    node.get_logger().info(
        f"🎵 движок v2: владелец плеера — единственный писатель {MUSIC_STATE_TOPIC} и "
        f"{MUSIC_EVENT_TOPIC}; Clock.latency={latency} (ADR-0149 PR-4)")
    return owner


def _attach_speech_duck(node: Any, runtime: Any) -> Optional[SpeechDuck]:
    """ADR-0150 О8 (а), PR-7d — музыка приглушается, пока робот говорит в динамики.

    ``/voice/speech/event`` (владелец речи ``tts_node``) → :class:`SpeechDuck` → ``/n_set 999 duck 0|1``; таймер
    снимает реплики с потерянным финалом. Без звукового сервера — нечего приглушать, подписки нет.
    """
    if runtime is None:
        return None
    from rob_box_dialog_msgs.msg import SpeechEvent  # noqa: PLC0415 — как voice_request.VoiceRequests

    controls = duck_controls(*(float(node.get_parameter(name).value) for name in (
        "music_duck_db", "music_duck_attack_ms", "music_duck_release_ms")))
    duck = SpeechDuck(runtime.send_osc, controls, logger=node.get_logger())
    kinds = {getattr(SpeechEvent, f"KIND_{name.upper()}"): name for name in ("started", "finished", "cancelled",
                                                                             "rejected")}
    node.create_subscription(SpeechEvent, "/voice/speech/event",
                             lambda msg: duck.on_event(kinds.get(int(msg.kind), ""), msg.speech_id,
                                                       int(msg.duration_ms)), 20)
    node.create_timer(1.0, duck.tick)
    node.get_logger().info("🔉 приглушение музыки под речь: /voice/speech/event → /n_set 999 duck "
                           + " ".join(f"{k}={v:g}" for k, v in controls.items()))
    return duck


def _speaker_signal(data: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
    """``/voice/speaker/result`` → ``(меняет ли «кто сейчас», новый speaker_id)``.

    Issue #2863 — «не знаю» ≠ «это другой человек». Отказ регистрации
    (``event=register_error``, поля is_known в нём нет вовсе) и фраза,
    которую биометрия не смогла оценить (``inconclusive``: нет эмбеддинга /
    мало речи), раньше читались как is_known=false и сбрасывали узнанного
    диктора в ∅. Теперь они ``(False, None)`` — не сигнал. Сбрасывает
    только оценённая фраза (``{"is_known": false}`` без пометки, #2829).

    Issue #4023 — «кто сейчас» становится профилем ТОЛЬКО при опубликованном
    имени (``speaker_wire.name_state == confirmed``) или по ack регистрации.
    Узнанный голос без подтверждённого имени (``contested``: похож на двоих с
    разными именами; гипотеза ``tentative``; профиль без имени) сбрасывает
    «кто сейчас» в ∅, как незнакомец: иначе memory-тулы без ``speaker_id``
    читали и писали факты ЧУЖОГО профиля (марафон 23.10, 2b n709).
    Модульная функция — чтобы держать CC ``_on_speaker_result`` в бюджете
    ADR-0021 и чтобы её можно было звать со стаба ноды в тестах.
    """
    event = data.get("event")
    if (event and event != "registered") or data.get("inconclusive"):
        return False, None
    raw_sid = data.get("speaker_id")
    # ``registered`` событие несёт speaker_id даже без is_known; ``and``
    # отфильтровывает пустые строки и None.
    if raw_sid and (event == "registered" or name_state(data) == NAME_CONFIRMED):
        return True, str(raw_sid)
    return True, None


class MCPServer(Node):
    """
    MCP Server - центральная нода для управления инструментами

    Предоставляет инструменты для LLM и обрабатывает запросы на их выполнение.
    """

    def __init__(self):
        super().__init__("mcp_server")

        # Параметры ноды
        # music_master_gain — ЕДИНСТВЕННАЯ ручка уровня музыки относительно
        # речи: мастер-фейдер ПОСЛЕ лимитера (/n_set 999 gain <v>).
        # Внутренняя динамика микса при этом сохраняется.
        self.declare_parameter("music_master_gain", 0.5)
        # ADR-0150 О8 (а), В16 (PR-7d): приглушение музыки под речь — отдельный множитель после
        # лимитера, gain и trim (masterfilter.scd duck*); числа — rob_box_music.knowledge.SPEECH_DUCK.
        self.declare_parameter("music_duck_db", music_kn.SPEECH_DUCK["duckDb"])
        self.declare_parameter("music_duck_attack_ms", music_kn.SPEECH_DUCK["duckAttack"] * 1000.0)
        self.declare_parameter("music_duck_release_ms", music_kn.SPEECH_DUCK["duckRelease"] * 1000.0)
        # ADR-0149: владелец плеера v2 (engine/player_owner.py) — единственный писатель
        # /voice/music/state и /voice/music/event.
        # Clock.latency v2 (#3328): 0.5 с — late-бандлов 250/мин → 0 (PR-2/PR-4 замеры).
        self.declare_parameter("music_v2_clock_latency", V2_CLOCK_LATENCY_S)
        # ADR-0149 PR-10: ризонер плана сета (В1 MiniMax). §12 В2 (решение Шифу 06.10): реплика диджея на
        # каждом переходе — факты из плана, LLM ризонера раскрашивает, иначе шаблон; музыку не трогает.
        # Старый ключ music_v2_hype_line (выкрик один раз за сет) больше не читается.
        self.declare_parameter("music_v2_reasoner", True)
        self.declare_parameter("music_v2_reasoner_deadline_s", V2_REASONER_DEADLINE_S)
        self.declare_parameter("music_v2_dj_lines", True)
        # ADR-0152 §2.3: полная модель каждого трека сета — <track_id>.json (ротация); "" — не писать.
        self.declare_parameter("music_v2_tracks_dir", "/data/music_v2/tracks")
        # ADR-0157 PR-7 (#3564): кто выбирает пьесу при неоднозначном заказе - режим как у провайдеров LLM/STT/TTS
        # (параметр ноды, не env): deterministic | jev | chain (Jev 900 мс → LLM 2 с → кубик кода; решение Шифу 13.10).
        self.declare_parameter("music_chooser", CHOOSER_DEFAULT_MODE)
        # Issue #1219 — активный TTS-провайдер для валидации голосов в
        # speak_text/set_voice. Должен совпадать с tts_node.yaml provider
        # (minimax). Используется для выбора списка голосов (Q4).
        self.declare_parameter("tts_provider", "minimax")

        # Issue #2781 — E2E-изоляция долгосрочной памяти (voice_facts),
        # по образцу speaker_id_node.e2e_mode (issue #2750/#2763). memory_save
        # пишет факты в self.voice_memory (legacy VoiceMemory →
        # /data/voice_memory.db), и до этой правки изоляция дикторов НЕ
        # накрывала этот писатель: акт «Знакомство» ночного марафона
        # регистрирует голоса через speaker_id_node (изолирован) И называет
        # LLM факты через memory_save (НЕ изолирован) — 59 из 116 фактов
        # боевой /data/voice_memory.db на 22.09.2026 оказались синтезированным
        # кастом марафона ("Саша"/"Борис"). Режим — не свой параметр, а
        # владельца /speaker_id_node (#3904, см. E2EModeFollower ниже).
        self.declare_parameter("e2e_db_path", "/data/voice_memory.e2e.db")

        # Реестр инструментов
        self.registry = MCPToolRegistry()

        # Issue #2781 — состояние переключателя e2e_mode для voice_memory
        # (см. docstring declare_parameter выше и _apply_e2e_mode). Боевой
        # путь запоминается в _init_voice_memory() (None, если активен
        # harness-адаптер MCP_USE_HARNESS_VOICE_MEMORY=1 — для него
        # переключение не реализовано, см. _apply_e2e_mode).
        self._voice_memory_prod_db_path: "str | None" = None
        self._voice_memory_ollama_url: "str | None" = None
        self._voice_memory_e2e_db_path: str = str(
            self.get_parameter("e2e_db_path").value or "/data/voice_memory.e2e.db"
        )
        self._voice_memory_e2e_mode_active: bool = False
        self._voice_memory_lock = threading.Lock()

        # Долгосрочная память (VoiceMemory) — инициализировать ДО регистрации инструментов
        self.voice_memory = None
        self._init_voice_memory()
        # #3904 — режим e2e приходит от владельца (/e2e/mode). Подписка
        # здесь (после _init_voice_memory), чтобы ``self.voice_memory`` /
        # ``self._voice_memory_prod_db_path`` уже существовали к первому
        # сообщению.
        self._e2e_follower = E2EModeFollower(self, self._apply_e2e_mode)

        # FAQ store + event profile (event mode)
        self.faq_store = None
        self.event_profile = None
        self._init_faq_store()

        # WaypointStore — SQLite CRUD для вейпоинтов (одна БД с VoiceMemory)
        self.waypoint_store = self._init_waypoint_store()

        # MappingState — FSM персистентное состояние (localization / mapping)
        self.mapping_state = MappingState()
        _ms = self.mapping_state.get()
        self.get_logger().info(
            f"🗺️  MappingState: mode={_ms['mode']}, map='{_ms.get('map_name') or 'none'}'"
        )

        # Лёгкий снимок позиции из /odom (вместо тяжёлого TF2 Listener на /tf 110 Гц)
        self._pose_snapshot: "Dict[str, float] | None" = None
        self._init_pose_subscription()

        # Регистрация инструментов
        self._register_tools()

        # QoS RELIABLE для гарантированной доставки результатов через Zenoh
        # BEST_EFFORT терял сообщения в сетевом окружении!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._qos_profile = qos_profile

        # Publisher для списка инструментов.
        #
        # TRANSIENT_LOCAL (latched): каталог инструментов — это статическое
        # объявление, а не поток данных. Раньше здесь висел таймер на 10с,
        # который каждые десять секунд сериализовал ~50 схем с indent=2 и
        # публиковал их в топик, у которого в проде не было ни одного
        # подписчика. У этой ноды уже была история CPU-петли
        # (mcp-server-cpu-loop-2026-08-22), так что периодическая рассылка
        # мегабайтного JSON в никуда — не мелочь. Теперь публикуем один раз
        # при старте, а late joiner'ы (dialogue_node, `ros2 topic echo`)
        # получают последнее сообщение из durability-кэша.
        tools_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.tools_pub = self.create_publisher(String, "/mcp/tools", tools_qos)

        # Publisher для результатов
        self.result_pub = self.create_publisher(String, "/mcp/result", qos_profile)

        # Issue #3133 / ADR-0141: плеер — владелец состояния музыки.
        # /voice/music/state — JSON-снимок {state, track_id, form_ends_at,
        # stops_at, dj, finished_track_id, ts}, контракт — модуль
        # rob_box_voice.core.music_player_state. Latched (TRANSIENT_LOCAL,
        # KEEP_LAST 1): перезапущенный audio_node/dialogue_node сразу
        # получает последний снимок, а не «ничего не играет» до тика.
        # Подписчики обязаны подписываться тоже TRANSIENT_LOCAL.
        self.music_state_pub = self.create_publisher(String, MUSIC_STATE_TOPIC, tools_qos)
        self._player_owner = _attach_player_owner_v2(self, getattr(self, "_renardo_runtime", None))
        self._speech_duck = _attach_speech_duck(self, getattr(self, "_renardo_runtime", None))

        # Subscriber для запросов на выполнение
        # ReentrantCallbackGroup — критически важно!
        # on_execute_request блокируется ожидая action result.
        # Если он в дефолтной MutuallyExclusiveCallbackGroup,
        # ActionClient response callbacks НЕ МОГУТ выполниться
        # (та же группа "залочена") → deadlock → "Nav2 не ответил".
        self._execute_cb_group = ReentrantCallbackGroup()
        self.execute_sub = self.create_subscription(
            String, "/mcp/execute", self.on_execute_request, qos_profile,
            callback_group=self._execute_cb_group
        )

        # Аутентификация отправителя /mcp/execute. Топик открыт всему
        # ROS2/Zenoh-графу, а за ним сразу registry.execute() — без этой
        # проверки любой пир исполняет инструменты в обход LLM и
        # confirmation gate. См. mcp_auth.py.
        self.authenticator = RequestAuthenticator.from_env(logger=self.get_logger())

        # Issue #1998 §6.2 — sender → slice → tool allowlist (ADR-0052).
        # Стоит после auth (HMAC подпись) и до FSM-гарда картографирования:
        # каждый гард отвечает за свой инвариант и не знает про остальные,
        # чтобы лог отказа был однозначным. Политика загружается из
        # bundled YAML (data/slice_policy.yaml); тесты могут подменить
        # через ``self.slice_authority = ...`` (атрибут публичный).
        try:
            self.slice_authority: ToolSliceAuthority = load_default_authority()
        except Exception as exc:  # ConfigError / yaml YAMLError / etc.
            # ADR-0018 (честный отказ): поведение здесь — **fail-closed**.
            # Пустая политика (senders={}) означает, что is_allowed() откажет
            # ЛЮБОМУ sender'у на ЛЮБОЙ инструмент — голосовой стек
            # останется без инструментов целиком. Это сознательный выбор
            # (срез — граница безопасности, открывать её при сломанном конфиге
            # нельзя), но диагностически это тихая смерть: снаружи выглядит
            # как «агент перестал уметь всё», поэтому лог обязан называть
            # и причину, и самую вероятную поломку (упаковка).
            self.get_logger().error(
                f"❌ Не удалось загрузить slice_policy.yaml: {exc}. "
                "mcp_server стартует с ПУСТОЙ политикой — это FAIL-CLOSED: "
                "НИ ОДИН sender не сможет вызвать НИ ОДИН инструмент, "
                "весь tool-слой агента мёртв. Самая частая причина — сломанная "
                "упаковка: rob_box_mcp_tools/setup.py обязан объявлять "
                "package_data={'rob_box_mcp_tools.data': ['*.yaml']}, иначе YAML не "
                "попадает в install-дерево (прод-образ собирается без "
                "--symlink-install). Проверка на роботе: python3 -c "
                "'from rob_box_mcp_tools.slice_authority import load_default_authority; "
                "load_default_authority()'"
            )
            self.slice_authority = ToolSliceAuthority.from_mapping(
                {"senders": {}, "slices": {"_noop": []}}
            )

        # Подписка на perception context для обновления инструментов
        try:
            from rob_box_perception_msgs.msg import PerceptionEvent

            self.perception_sub = self.create_subscription(PerceptionEvent, "/perception/context_update", self.on_perception_update, 10)
            self.get_logger().info("✅ Подписан на /perception/context_update")
        except ImportError:
            self.get_logger().warning("⚠️ PerceptionEvent не найден, мониторинг контекста отключен")

        # Issue #1770 — подписка на /voice/speaker/result, чтобы memory
        # tools могли фильтровать facts/turns по speaker_id без явной передачи
        # от LLM (fallback: см. ``tools/memory.py:_current_encounter_speaker_id``).
        # speaker_id_node публикует ``{"is_known": true, "speaker_id": "...",
        # "name": "...", "confidence": 0.93}`` после каждой распознанной
        # реплики; нам нужен только ``speaker_id`` (UUID), всё остальное —
        # для логов.
        #
        # Issue #2442 — раньше это был отдельный ``self.current_speaker_id:
        # Optional[str]``, продублированный с ``dialogue_node._current_speaker``
        # и тремя фоллбэками в ``tools/memory.py``. Теперь сигнал кормит
        # ``EncounterSeam`` (``rob_box_harness.encounter``, тот же шов, что
        # использует голосовой адаптер) — единственный источник «кто сейчас»
        # для этого процесса. Идентичность здесь намеренно эфемерна
        # (``InMemoryStore`` — ничего не пишет на диск): mcp_server и раньше
        # не считал ``since_last_seen`` для локального кэша спикера, так что
        # это не регрессия, а явное сохранение прежнего поведения при
        # переходе на общий контракт шва (ADR-0105 §3.1, §2.4).
        self._encounter_seam = EncounterSeam(MemoryIdentitySeam(InMemoryStore()))
        try:
            self._speaker_result_sub = self.create_subscription(
                SpeakerResult,
                "/voice/speaker/result",
                self._on_speaker_result,
                10,
            )
            self.get_logger().info("🎙️ Подписан на /voice/speaker/result (issue #1770)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось подписаться на /voice/speaker/result: {exc}"
            )

        # Issue #1229 — фактический провайдер TTS (после фолбека) от tts_node.
        # tts_node публикует JSON {"provider": str, "voice": str, ...} после
        # старта/фолбека/синтеза. SpeakTextTool/SetVoiceTool валидируют голоса
        # по РЕАЛЬНОМУ провайдеру (а не номинальному tts_provider из YAML):
        # иначе LLM выбирает minimax-голоса, которых нет у yandex после
        # фолбека, и слышит один и тот же дефолтный голос.
        self.actual_tts_provider: str | None = None
        self.actual_tts_voice: str | None = None
        # ADR-0150 §8 (PR-4 ч.2): здоровье генерации музыки (kind=music_gen) - переиздание раз в 2 с.
        self._providers_publisher = ProviderStatePublisher(self, gen_provider_states)
        try:
            self._provider_state_sub = self.create_subscription(
                String, "/voice/tts/provider_state", self._on_tts_provider_state, 10
            )
            self.get_logger().info("🎙️ Подписан на /voice/tts/provider_state (issue #1229)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось подписаться на /voice/tts/provider_state: {exc}"
            )

        # Публикуем каталог инструментов один раз — он latched (см. tools_qos).
        self.publish_tools()

        self.get_logger().info(f"🛠️ MCP Server запущен с {len(self.registry)} инструментами")
        self.get_logger().info(f"   Инструменты: {', '.join(self.registry.list_tools())}")

    # ------------------------------------------------------------------
    # Music cleanup hooks — safety-net (issue #935)
    # ------------------------------------------------------------------
    def _on_tts_provider_state(self, msg: "String") -> None:
        """Issue #1229 — запомнить фактического провайдера TTS от tts_node.

        tts_node публикует JSON {"provider": str, "voice": str, ...} после
        старта/фолбека/синтеза. SpeakTextTool/SetVoiceTool читают
        ``self.actual_tts_provider`` через ``node.actual_tts_provider`` и
        валидируют голоса по РЕАЛЬНОМУ провайдеру (Q4/Q6).
        """
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (TypeError, ValueError):
            return
        provider = payload.get("provider") if isinstance(payload, dict) else None
        if not provider:
            return
        self.actual_tts_provider = str(provider)
        self.actual_tts_voice = payload.get("voice")  # голос, который реально звучит (песня голосом робота, PR-7)
        self.get_logger().info(
            f"🎙️ [issue 1229] actual TTS provider → '{self.actual_tts_provider}' "
            f"(reason: {payload.get('reason')})"
        )

    def _on_speaker_result(self, msg: "SpeakerResult") -> None:
        """Issue #1770 / #2442 — учесть сигнал голосовой биометрии в ``EncounterSeam``.

        Формат: ``{"is_known": true, "speaker_id": "<uuid>",
        "name": "Денчик", "confidence": 0.93}`` или
        ``{"is_known": false}``. ``speaker_id`` есть только при
        ``is_known=True``; в этом случае мы кормим сигнал в
        ``EncounterSeam.observe()`` — тот же шов, что используют
        ``tools/memory.py`` (fallback ``speaker_id``) через
        ``_current_encounter_speaker_id``.

        На событие ``{"event": "registered", "speaker_id": "..."}`` мы
        тоже обновляем шов — это значит, что прямо сейчас
        зарегистрировали нового юзера и следующая реплика отнесётся к
        нему. Это осознанное расхождение с
        ``rob_box_harness.encounter.voice_adapter.VoiceEncounterAdapter``
        (тот трактует ``registered`` как no-op) — поведение mcp_server
        здесь старше и продиктовано issue #1770 («следующая реплика
        должна найти профиль немедленно после регистрации»), сохраняем
        его 1:1, а не подменяем общим адаптером.

        ``EncounterSeam.observe`` — асинхронный метод (может дойти до
        ``IdentitySeam.since_last_seen``). Мостик sync ROS callback →
        async вызов — ``asyncio.run`` за один вызов: тот же приём, что
        ``VoiceMemoryAdapter`` уже использует в этом сервисе (см.
        ``rob_box_harness/memory/voice_memory_adapter.py``, докстрока
        «Why a per-call asyncio.run»). Идентичность шва здесь —
        ``InMemoryStore`` (без диска), поэтому вызов не блокируется на
        реальном I/O и безопасен внутри ROS executor'а.
        """
        data = speaker_result_to_payload(msg)
        # Issue #2863 — служебные ack и неоценённые фразы не сигнал.
        relevant, new_speaker_id = _speaker_signal(data)
        if not relevant:
            return

        old = self._current_encounter_speaker_id()
        if new_speaker_id == old:
            return  # без изменений — тихий return, не спамим лог

        who = Acquaintance(id=new_speaker_id, name=data.get("name")) if new_speaker_id else None
        try:
            confidence = float(data.get("confidence") or 0.0)
        except (TypeError, ValueError):
            confidence = 0.0
        asyncio.run(
            self._encounter_seam.observe(EncounterChannel.VOICE, who, confidence)
        )
        if new_speaker_id:
            self.get_logger().info(
                f"👤 [issue 1770] current_speaker_id: {old or '∅'} → "
                f"{new_speaker_id[:12]}… (name={data.get('name')!r})"
            )
        else:
            # #4023: почему ∅ — голос не узнан (unknown) или узнан без подтверждённого имени (contested/…)
            self.get_logger().info(
                f"👤 [issue 1770] current_speaker_id: {old or '∅'} → ∅ "
                f"(name_state={name_state(data)})"
            )

    def _current_encounter_speaker_id(self) -> Optional[str]:
        """Issue #2442 — ``speaker_id`` текущей Встречи, или ``None``.

        Единственная точка чтения ``EncounterSeam.current()`` внутри
        mcp_server. ``tools/memory.py`` не импортирует этот метод напрямую
        (``mcp_server`` зависит от ``tools`` — обратный импорт дал бы
        цикл), поэтому там та же логика продублирована как модульная
        функция ``_current_encounter_speaker_id(node)`` — оба места читают
        ровно один и тот же атрибут узла, ``node._encounter_seam``.
        """
        current = self._encounter_seam.current()
        if current is None or current.who is None:
            return None
        return current.who.id

    def _stop_deck(self, reason: str) -> None:
        """Жёсткий cleanup: закрыть сет и снять деку через их владельцев.

        Одна точка стопа для ``stop_music``: глушить
        Renardo за спиной ``PlayerOwner`` нельзя — снимок остался бы
        ``playing``, а ``SetSession`` поставила бы следующий трек.
        """
        set_tool = getattr(self, "_dj_set_tool", None)
        if set_tool is not None:
            set_tool.close_set(reason)
        owner = getattr(self, "_player_owner", None)
        if owner is not None and owner.is_playing():
            owner.stop(reason)

    def _init_waypoint_store(self) -> WaypointAdapter:
        """Инициализация адаптера для вейпоинтов.

        ADR-0055 Phase 2 v2 (issue #2000): ``WaypointAdapter`` —
        schema-reconciling facade over ``WaypointStore``. Остаётся на
        ``VOICE_MEMORY_DB_PATH`` / ``/data/voice_memory.db``; DDL
        ``harness_voice.db.waypoints`` НЕ трогается (ADR-0055 §3).
        Адаптер отдаёт:

        * MCP-вью (pass-through): ``list_waypoints`` / ``get_waypoint``
          / ``save_waypoint`` / ``delete_waypoint`` / ``clear_waypoints``
          / ``get_active_map`` / ``list_maps`` / ``create_map``.
          Все эти методы удовлетворяют ранее подписанному контракту
          ``WaypointStore``, поэтому ``tools/navigation.py``
          (NavigateToWaypointTool, ListWaypointsTool, …) продолжают
          работать без правок кода, принимая ``WaypointAdapter``
          (duck-type = ``WaypointStore``).
        * Harness-вью (для AgentCore step 03): ``*_harness``-методы,
          ``map_id`` по умолчанию ``"default"``.

        Обёртка ``WaypointAdapter`` — единственная точка контакта
        для всех MCP-инструментов по вейпоинтам, чтобы Phase 2 v3
        (data-migration) могла сменить бэкенд «под ковром» без
        затрагивания вызывающего кода.
        """
        import os

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        try:
            store = WaypointStore(db_path=db_path)
            adapter = WaypointAdapter(store=store)
            active = adapter.get_active_map()
            if active:
                wp_count = len(adapter.list_waypoints())
                self.get_logger().info(
                    f"📍 WaypointAdapter: карта '{active['name'] or active['map_id'][:8]}', "
                    f"{wp_count} точек"
                )
            else:
                self.get_logger().info("📍 WaypointAdapter: активная карта не задана (будет создана при первом сохранении)")
            return adapter
        except Exception as exc:
            self.get_logger().error(f"❌ Ошибка инициализации WaypointAdapter: {exc}")
            # Fallback — in-memory adapter so tools don't crash
            fallback_store = WaypointStore(db_path=":memory:")
            return WaypointAdapter(store=fallback_store)

    def _init_pose_subscription(self) -> None:
        """Подписка на /odom для лёгкого снимка позиции (без tf2_ros.Buffer).

        Раньше здесь был ``tf2_ros.TransformListener`` → подписка на /tf (~110 Гц
        с камеры), которая частыми wake-up'ами экзекутора и дорогой пересборкой
        WaitSet на rmw_zenoh жгла ~45% CPU (см. mcp-server-cpu-loop-2026-08-22).
        /odom (~10 Гц) даёт позицию робота напрямую; инструменты читают снимок,
        а не делают blocking TF lookup.
        """
        try:
            from nav_msgs.msg import Odometry

            self.create_subscription(Odometry, "/odom", self._on_odom_snapshot, 10)
            self.get_logger().info("📍 Подписан на /odom (лёгкий снимок позиции)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ Не удалось подписаться на /odom: {exc}")

    def _on_odom_snapshot(self, msg) -> None:
        """Обновить снимок позиции {x, y, theta} из nav_msgs/Odometry."""
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._pose_snapshot = {
                "x": float(pos.x),
                "y": float(pos.y),
                "theta": math.atan2(siny_cosp, cosy_cosp),
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ Ошибка обработки /odom: {exc}")

    def get_current_pose_snapshot(self) -> "Dict[str, float] | None":
        """Последний снимок позиции робота ({x, y, theta}) или None, если данных нет."""
        return self._pose_snapshot

    def _register_tools(self):
        """Регистрация всех доступных инструментов."""
        # Navigation tools (require waypoint_store and/or pose snapshot)
        self.registry.register(NavigateToWaypointTool(self, self.waypoint_store))
        self.registry.register(NavigateToCoordinatesTool(self))
        self.registry.register(MoveDirectionTool(self))
        self.registry.register(StopNavigationTool(self))
        self.registry.register(ListWaypointsTool(self, self.waypoint_store))
        self.registry.register(
            SaveWaypointTool(self, self.waypoint_store, self.get_current_pose_snapshot, self.mapping_state)
        )
        self.registry.register(DeleteWaypointTool(self, self.waypoint_store))
        self.registry.register(ClearWaypointsTool(self, self.waypoint_store))
        self.registry.register(GetCurrentPoseTool(self, self.get_current_pose_snapshot))

        # System tools
        self.registry.register(SetVolumeTool(self))
        self.registry.register(SetPitchTool(self))
        self.registry.register(SetSpeedTool(self))
        self.registry.register(GetRobotStatusTool(self))
        self.registry.register(GetCurrentTimeTool(self))

        # Perception tools
        self.perception_context_tool = GetPerceptionContextTool(self)
        self.battery_tool = GetBatteryLevelTool(self)
        self.registry.register(self.perception_context_tool)
        self.registry.register(self.battery_tool)

        # Mapping tools
        self.registry.register(StartMappingTool(self, self.waypoint_store, self.mapping_state))
        self.registry.register(ContinueMappingTool(self))
        self.registry.register(FinishMappingTool(self, self.waypoint_store, self.mapping_state))
        self.registry.register(OptimizeMapTool(self))
        self.registry.register(LoadMapTool(self, self.waypoint_store, self.mapping_state))

        # Animation tools
        self.registry.register(PlayAnimationTool(self))

        # Sound tools
        self.registry.register(PlaySoundTool(self))
        self.registry.register(GetSoundInfoTool(self))

        # FAQ / Event tools
        self.registry.register(FaqSearchTool(self))

        # Dialogue tools (критично для агентного диалога!)
        # Голос хранит один владелец - tts_node (VoiceOwner, PR-4 ч.3): speak_text без voice= говорит текущим голосом,
        # set_voice/set_tts_provider просят через /voice/tts/voice_request и ждут SpeechEvent voice_changed.
        self.registry.register(SpeakTextTool(self))
        self.registry.register(EstimateTtsDurationTool(self))
        self.registry.register(ListenForResponseTool(self))
        self.registry.register(SetVoiceTool(self))
        # Issue #1765 — переключение TTS-провайдера + список голосов
        # по провайдеру (кросс-провайдерный кейс: «Яндекс Артём» при
        # активном minimax). Оба tool'а публикуют /voice/tts/set_provider,
        # tts_node подписан и пересобирает provider_chain.
        self.registry.register(SetTtsProviderTool(self))
        self.registry.register(ListTtsVoicesTool(self))
        # Issue #1101 — LLM-driven speaker registration (replaces regex NLU).
        # LLM extracts name from user_input and calls register_speaker(name=X)
        # via MCP. speaker_id_node binds d-vector to name in /data/speakers.db.
        self.registry.register(RegisterSpeakerTool(self))
        self._search_web_tool = SearchWebTool(self)
        self.registry.register(self._search_web_tool)
        # Issue #2113 — TARS 2 metrics panel. ``show_metrics`` публикует
        # запрос в /avatar/tars/panel_request; TarsPanelDispatcher
        # (rob_box_supervisor.tars_panel) подписан на этот топик, парсит
        # запрос и публикует собранный URL Grafana-панели в
        # /avatar/tars/panel_url (на этот топик уже подписан Quest-клиент).
        # Без этой регистрации mcp_server отвечал бы «unknown tool» на
        # show_metrics — даже если срез-гард пропускал avatar_supervisor.
        self.registry.register(ShowMetricsTool(self))
        # PR-13b #3708 — диагностика ТАРС (срез operator.admin): статус нод по rclpy, логи контейнеров из Loki,
        # метрики контейнеров из Prometheus. Без регистрации тулы были в политике, но вызов давал «unknown tool».
        self.registry.register(Ros2NodeStatusTool(self))
        self.registry.register(ReadLogsTool(self))
        self.registry.register(ContainerStatusTool(self))
        # Issue #3305 — ``say``: голос оператора (ТАРС). Публикует SSML в
        # /voice/tts/request (нужен только node.create_publisher). Без
        # регистрации /mcp/execute отвечал «unknown tool».
        self.registry.register(SayTool(self))
        # PR-12b (#3707) — ``say_aloud``: озвучка из чата Telegram, итог по SpeechEvent (срез ``telegram``).
        self.registry.register(SayAloudTool(self))

        # Memory tools (долгосрочная память + семантический поиск)
        self.registry.register(MemorySaveTool(self))
        self.registry.register(MemorySearchTool(self))
        self.registry.register(MemoryContextTool(self))

        self._register_music_tools()

    def _register_music_tools(self) -> None:
        """Регистрирует music tools, не роняя весь MCP server при частичной деградации."""
        music_master_gain = self.get_parameter("music_master_gain").value
        self.get_logger().info(f"🎵 Music master_gain: {music_master_gain:.2f}")
        # Renardo и scsynth для движка v2 (#3614): контекст, палитра, raw OSC, /fail, мастер-фейдер.
        # Не поднялся — нода живёт, get_music_state честно скажет «движок недоступен».
        runtime: Optional[RenardoRuntime]
        try:
            runtime = RenardoRuntime(master_gain=music_master_gain)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"❌ Music subsystem disabled: RenardoRuntime init failed: {exc}")
            runtime = None
        if runtime is not None:
            runtime.attach_logger(self.get_logger())
        self._renardo_runtime = runtime

        # RTTTL-библиотека (архив data/rtttl_melodies.jsonl.gz) — независима от
        # SQLite. Поиск по имени/жанру; играет мелодию v2 (``request_music``).
        rtttl_library: Optional[RtttlLibrary] = None
        try:
            rtttl_library = RtttlLibrary()
            self.registry.register(SearchMelodyTool(self, rtttl_library))
            self.registry.register(AddMusicMaterialTool(self, rtttl_library))  # #3227
            self.get_logger().info(f"🎵 RTTTL library: {rtttl_library.total()} мелодий")
        except Exception as exc:
            self.get_logger().error(f"❌ RTTTL library disabled: {exc}")

        self.registry.register(StopMusicTool(self))
        self.registry.register(GetMusicStateTool(self, runtime))
        if runtime is not None:
            # Issue #3125 — громкость МУЗЫКИ (мастер-фейдер), не голоса.
            self.registry.register(SetMusicVolumeTool(self, runtime))

        if rtttl_library is not None:
            self.registry.register(LookupMelodyTool(self, rtttl_library))

        # Issue #1392 / ADR-0158 — библиотека сгенерированных треков (кэш).
        # Graceful degradation: любой сбой (нет /data, ImportError) выключает
        # только music_library — Renardo-тулы продолжают работать.
        self._register_gen_library_tools()

    # ------------------------------------------------------------------
    # Issue #1392 / ADR-0158 — библиотека сгенерированных треков.
    # ------------------------------------------------------------------

    def _register_gen_library_tools(self) -> None:
        """Регистрирует ``music_library`` — один узкий тул над кэшем генераций (graceful degradation).

        Новый узкий ``generate_music`` (ADR-0158 PR-3) регистрируется рядом с
        ``request_music`` в ``_attach_player_owner_v2``: он просьба к чойсеру пути.

        Failure modes (each disables only music_library, Renardo keeps working):
            * Module import failed                              → log + return
            * /data volume not writable (GeneratedMusicLibrary) → log + skip
        """
        if not _GEN_LIBRARY_AVAILABLE:
            self.get_logger().warning(
                f"⚠️ music_library tool disabled: import failed "
                f"({_GEN_LIBRARY_IMPORT_ERROR})"
            )
            return

        # Type-narrow for Pyright: _GEN_LIBRARY_AVAILABLE gates the rest.
        assert GeneratedMusicLibrary is not None
        assert MusicLibraryTool is not None

        try:
            music_library = GeneratedMusicLibrary()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Generated music library disabled: init failed ({exc}). "
                "music_library will NOT be registered."
            )
            return
        self._generated_music_library = music_library
        self.get_logger().info(
            f"🎼 Generated music library: {music_library.count} трек(ов) "
            f"в {music_library.root_dir}"
        )

        self.registry.register(MusicLibraryTool(self, music_library))
        quota = enforce_quota(music_library)  # ADR-0158 PR-9: квота диска проверяется и на старте
        self.get_logger().info(f"🎼 Квота кэша: занято {quota['used'] // MIB} МиБ из {quota['quota'] // MIB}, "
                               f"вычищено {len(quota['removed'])}")

    def _init_voice_memory(self) -> None:
        """Инициализация VoiceMemory (долгосрочная память). Не падает при ошибках.

        ADR-0055 Phase 1 — when ``MCP_USE_HARNESS_VOICE_MEMORY=1`` the
        adapter over ``SQLiteVoiceMemory`` is used so writes go to
        ``/data/harness_voice.db`` (same file the dialogue node uses).
        Otherwise we fall back to the legacy ``VoiceMemory`` →
        ``/data/voice_memory.db``.

        This flag only affects ``self.voice_memory`` (facts). It does
        NOT unify ``self.waypoint_store`` or ``self.faq_store`` — those
        stay on ``/data/voice_memory.db`` either way (see the module-level
        comment above ``_USE_HARNESS_VOICE_MEMORY`` for why: their table
        names collide with incompatible schemas already living in
        harness_voice.db).

        Issue #2781 — ``self._voice_memory_prod_db_path`` stays ``None``
        when the harness adapter branch below is taken: ``_apply_e2e_mode``
        only knows how to re-point the legacy ``_VoiceMemory`` class at a
        different SQLite file, and treats ``None`` as "no E2E target for
        this backend" (fails the ``ros2 param set`` instead of silently
        no-op'ing — see its docstring).
        """
        if _USE_HARNESS_VOICE_MEMORY and _VoiceMemoryAdapter is not None:
            import os

            # Default to the unified DB on /data/ but allow override for
            # staging / test environments.
            db_path = os.getenv(
                "HARNESS_VOICE_DB", "/data/harness_voice.db"
            )
            try:
                self.voice_memory = _VoiceMemoryAdapter(db_path=db_path)
                stats = self.voice_memory.get_stats()
                self.get_logger().info(
                    f"🧠 VoiceMemoryAdapter (ADR-0055 Phase 1): {db_path} "
                    f"(facts={stats.get('fact_count', 0)})"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"❌ Ошибка инициализации VoiceMemoryAdapter: {exc}"
                )
                self.voice_memory = None
            return

        if _VoiceMemory is None:
            self.get_logger().warning(
                "⚠️ rob_box_voice не найден — VoiceMemory отключена. "
                "Memory MCP tools не будут работать."
            )
            return

        import os

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        ollama_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")

        try:
            self.voice_memory = _VoiceMemory(db_path=db_path, ollama_base_url=ollama_url)
            # Issue #2781 — запоминаем боевой путь/ollama_url, чтобы
            # _apply_e2e_mode мог переоткрыть VoiceMemory на e2e_db_path и
            # вернуться обратно, не завися от переменных окружения второй раз.
            self._voice_memory_prod_db_path = db_path
            self._voice_memory_ollama_url = ollama_url
            stats = self.voice_memory.get_stats()
            self.get_logger().info(
                f"🧠 VoiceMemory инициализирована: {db_path} "
                f"(turns={stats['turn_count']}, sessions={stats['session_count']}, "
                f"facts={stats['fact_count']}, vec={stats['vec_enabled']})"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ Ошибка инициализации VoiceMemory: {exc}")
            self.voice_memory = None

    # ── E2E isolation for voice_facts (issue #2781) ─────────────────────────
    #
    # #3904: режим — владельца /speaker_id_node (rob_box_core.e2e_mode);
    # эта нода только применяет его (E2EModeFollower → _apply_e2e_mode).

    def _wipe_e2e_voice_memory_db_file(self) -> None:
        """Стереть файл ``e2e_db_path`` (и -wal/-shm/-journal) перед каждым ON.

        По образцу ``speaker_id_node._wipe_e2e_db_file``. НИКОГДА не трогает
        ``self._voice_memory_prod_db_path`` — путь для удаления берётся из
        ``self._voice_memory_e2e_db_path`` напрямую (константа узла, не то,
        что можно подменить через ``ros2 param set``).
        """
        import os as _os

        # Issue #2890 — боевая voice_memory.db не стирается НИКОГДА, даже
        # если e2e_db_path в конфиге указал на неё же (опечатка в YAML,
        # симлинк): исключение прерывает переключение в _apply_e2e_mode,
        # харнесс получает отказ e2e_mode и фейлит акт.
        prod = self._voice_memory_prod_db_path
        if prod and _os.path.realpath(prod) == _os.path.realpath(
            self._voice_memory_e2e_db_path
        ):
            raise RuntimeError(
                f"e2e_db_path {self._voice_memory_e2e_db_path!r} совпадает с "
                f"боевой {prod!r} — стирать отказываюсь"
            )
        removed = []
        for suffix in ("", "-wal", "-shm", "-journal"):
            path = f"{self._voice_memory_e2e_db_path}{suffix}"
            try:
                if _os.path.exists(path):
                    _os.remove(path)
                    removed.append(path)
            except OSError as exc:
                self.get_logger().warning(
                    f"⚠️ не удалось удалить {path!r} перед E2E-прогоном "
                    f"({type(exc).__name__}: {exc}) — e2e-база памяти может "
                    "унаследовать факты с прошлого марафона"
                )
        # Issue #2890 — явная строка сброса e2e-памяти фактов в логе робота.
        self.get_logger().warning(
            f"🧹 mcp_server: e2e-память фактов сброшена перед актом — "
            f"удалено {len(removed)} файл(ов) {self._voice_memory_e2e_db_path!r}"
        )

    def _apply_e2e_mode(self, enabled: bool) -> bool:
        """Переключить voice_memory (voice_facts) боевая ↔ E2E. ``True`` — успех.

        Issue #2781 — до этой правки изоляция БД дикторов (#2750/#2763)
        накрывала только ``speakers.db``: акт «Знакомство» ночного марафона
        и регистрирует голоса (изолировано), и называет LLM факты через
        ``memory_save`` (НЕ изолировано) — замер на Vision Pi 22.09.2026
        нашёл 59 из 116 фактов боевой ``/data/voice_memory.db``,
        упоминающих синтезированный каст марафона ("Саша не ест лук",
        "Борис болеет за Спартак").

        Переход False→True стирает прежний файл ``e2e_db_path`` (см.
        ``_wipe_e2e_voice_memory_db_file``) и открывает НОВЫЙ экземпляр
        ``VoiceMemory`` на нём — гарантия пустой e2e-базы держится узлом,
        не вызывающим скриптом. Переход True→True (повторный
        ``ros2 param set ... true``) — no-op, не чистит базу второй раз
        (не обнулять уже накопленные за акт факты). Боевой
        ``_voice_memory_prod_db_path`` никогда не удаляется и не изменяется.

        ``False`` — отказ (переключение НЕ применено, активная БД осталась
        прежней): либо E2E-изоляция не реализована для активного бэкенда
        (``self._voice_memory_prod_db_path is None`` — harness-адаптер
        ``MCP_USE_HARNESS_VOICE_MEMORY=1`` или ``rob_box_voice`` недоступен,
        ``self.voice_memory is None``), либо переоткрытие файла бросило
        исключение. ``E2EModeFollower`` тогда не меняет ``e2e_mode_applied``,
        и харнесс (``enter_e2e_mode``) фейлит акт: лучше не прогнать акт,
        чем засорить боевую память.
        """
        if enabled == self._voice_memory_e2e_mode_active:
            self.get_logger().info(
                f"🧪 e2e_mode (voice_memory): уже "
                f"{'включён' if enabled else 'выключен'} — no-op"
            )
            return True

        if self._voice_memory_prod_db_path is None or _VoiceMemory is None:
            self.get_logger().error(
                "❌ e2e_mode (voice_memory): нет боевого пути для переключения — "
                "либо активен harness-адаптер (MCP_USE_HARNESS_VOICE_MEMORY=1, "
                "E2E-изоляция для него не реализована, issue #2781 накрывает "
                "только legacy VoiceMemory/voice_memory.db), либо "
                "rob_box_voice недоступен и self.voice_memory не инициализирована. "
                "Переключение отклонено."
            )
            return False

        target_path = (
            self._voice_memory_e2e_db_path
            if enabled
            else self._voice_memory_prod_db_path
        )
        try:
            with self._voice_memory_lock:
                old_memory = self.voice_memory
                if enabled:
                    # Гарантия чистой базы — только на переходе в E2E-режим.
                    self._wipe_e2e_voice_memory_db_file()
                self.voice_memory = _VoiceMemory(
                    db_path=target_path,
                    ollama_base_url=self._voice_memory_ollama_url,
                )
                self._voice_memory_e2e_mode_active = enabled
                if old_memory is not None:
                    old_memory.close()
        except Exception as exc:  # noqa: BLE001 — переключение не должно ронять ноду
            self.get_logger().error(
                f"❌ e2e_mode (voice_memory)={'ON' if enabled else 'OFF'} "
                f"переключение провалилось: {type(exc).__name__}: {exc} — "
                f"активная БД памяти НЕ изменена (осталась "
                f"{'E2E' if self._voice_memory_e2e_mode_active else 'боевая'})"
            )
            return False

        # WARNING, не info: смена активной БД памяти обязана быть видна в
        # `docker logs voice-assistant` без фильтров (issue #2750 — то же
        # решение у speaker_id_node._apply_e2e_mode).
        self.get_logger().warning(
            f"🧪 mcp_server: e2e_mode (voice_memory)={'ON' if enabled else 'OFF'} — "
            f"voice_facts (memory_save) теперь пишутся в {target_path!r} "
            f"({'боевая voice_memory.db НЕ используется, пока режим включён' if enabled else 'вернулись на боевую'})"
        )
        return True

    def _init_faq_store(self) -> None:
        """Инициализация FAQStore и загрузка event profile (режим мероприятия).

        Остаётся на ``VOICE_MEMORY_DB_PATH`` / ``/data/voice_memory.db``:
        ``faq_items`` в ``/data/harness_voice.db`` уже создаётся
        ``SQLiteVoiceMemory`` с другой схемой (``created_at`` вместо
        ``indexed_at``, без FTS5-триггеров) — см. ADR-0055.
        """
        if _FAQStore is None or _load_event_profile is None:
            self.get_logger().info(
                "ℹ️ FAQ-модуль не загружен — faq_search будет возвращать 'недоступен'."
            )
            return

        import os

        faq_mode_enabled = os.getenv("FAQ_MODE_ENABLED", "0").strip().lower() in ("1", "true", "yes")
        faq_config_file = os.getenv("FAQ_EVENT_CONFIG_FILE", "/config/event.yaml")

        if not faq_mode_enabled:
            self.get_logger().info("ℹ️ FAQ mode disabled (FAQ_MODE_ENABLED != 1)")
            return

        try:
            profile = _load_event_profile(enabled=True, config_file=faq_config_file)
        except Exception as exc:
            self.get_logger().warning(f"⚠️ Не удалось загрузить event profile: {exc}")
            return

        if profile is None:
            self.get_logger().info("ℹ️ Event profile not loaded — faq_search disabled")
            return

        self.event_profile = profile

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        try:
            self.faq_store = _FAQStore(db_path=db_path)
            self.get_logger().info(
                f"📚 FAQStore инициализирован: event={profile.event_id} "
                f"({profile.name}), faq_file={profile.faq_file}"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ Ошибка инициализации FAQStore: {exc}")
            self.faq_store = None

    def publish_tools(self):
        """Публикация списка доступных инструментов в OpenAI Tool Calls формате."""
        tools = self.registry.get_openai_tools()
        msg = String()
        # Без indent: сообщение читает машина, а отступы удваивали payload.
        msg.data = json.dumps(tools, ensure_ascii=False)
        self.tools_pub.publish(msg)
        self.get_logger().info(f"📤 Опубликован каталог из {len(tools)} инструментов (latched)")

    def on_execute_request(self, msg: String):
        """
        Обработка запроса на выполнение инструмента

        Формат запроса (JSON):
        {
            "tool_name": "navigate_to_waypoint",
            "parameters": {
                "waypoint": "кухня"
            },
            "request_id": "optional_unique_id"
        }
        """
        try:
            request = json.loads(msg.data)
            tool_name = request.get("tool_name")
            parameters = request.get("parameters", {})
            request_id = request.get("request_id", "")

            self.get_logger().info(f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}")

            # ── Auth Guard: запрос должен быть подписан общим секретом ──
            # Стоит перед FSM-гардом и перед любым обращением к registry:
            # неаутентифицированный запрос не должен даже влиять на
            # replay-кэш имён инструментов в логах.
            is_authentic, auth_error = self.authenticator.verify(request)
            if not is_authentic:
                self.get_logger().error(
                    f"🚫 Отклонён неаутентифицированный /mcp/execute "
                    f"'{tool_name}': {auth_error}"
                )
                self._publish_error(f"Запрос отклонён: {auth_error}", request_id)
                return
            # ────────────────────────────────────────────────────────────

            if not tool_name:
                self._publish_error("Не указано имя инструмента", request_id)
                return

            # ── Slice Guard: sender → slice → tool (ADR-0052, issue #1998 §6.2) ──
            # Стоит после auth (нам нужен подписанный ``auth.sender``) и
            # до FSM-гарда картографирования: каждый гард отвечает за свой
            # инвариант и не знает про остальные, чтобы лог отказа был
            # однозначным. Sender берём из подписанного блока auth —
            # провалидированного HMAC'ом, так что подменить нельзя.
            sender = (
                (request.get("auth") or {}).get("sender")
                if isinstance(request.get("auth"), dict)
                else None
            ) or "unknown"
            slice_decision = self.slice_authority.is_allowed(sender, tool_name)
            if not slice_decision.allowed:
                self.get_logger().warning(
                    f"🚫 Slice blocked '{tool_name}' для sender='{sender}': "
                    f"{slice_decision.reason}"
                )
                from .base import MCPToolResult
                _slice_msg = (
                    f"Инструмент '{tool_name}' недоступен: {slice_decision.reason}"
                )
                _result = MCPToolResult(success=False, error=_slice_msg)
                _resp = {"tool_name": tool_name, "request_id": request_id, "result": _result.to_dict()}
                _msg_out = String()
                _msg_out.data = json.dumps(_resp, ensure_ascii=False)
                self.result_pub.publish(_msg_out)
                return
            # ────────────────────────────────────────────────────────────

            # ── FSM Guard: block disallowed tools during active mapping ──
            if not self.mapping_state.is_tool_allowed(tool_name):
                _ms = self.mapping_state.get()
                _map_label = f" '{_ms.get('map_name')}'" if _ms.get("map_name") else ""
                _block_msg = (
                    f"Идёт картографирование{_map_label}. "
                    "Сначала скажи 'завершить картографирование' — "
                    "тогда смогу помочь с навигацией и другими командами."
                )
                self.get_logger().warning(f"🚫 FSM blocked '{tool_name}' during mapping")
                from .base import MCPToolResult
                _result = MCPToolResult(success=False, error=_block_msg)
                _resp = {"tool_name": tool_name, "request_id": request_id, "result": _result.to_dict()}
                _msg_out = String()
                _msg_out.data = json.dumps(_resp, ensure_ascii=False)
                self.result_pub.publish(_msg_out)
                return
            # ────────────────────────────────────────────────────────────

            # Выполнить инструмент
            result = self.registry.execute(tool_name, **parameters)

            # Опубликовать результат
            response = {"tool_name": tool_name, "request_id": request_id, "result": result.to_dict()}

            msg_out = String()
            msg_out.data = json.dumps(response, ensure_ascii=False)

            # Логируем ПЕРЕД публикацией
            self.get_logger().info(f"📤 Публикую результат для {tool_name} (request_id: {request_id[:8]})")
            self.result_pub.publish(msg_out)
            self.get_logger().info(f"✅ Результат опубликован на /mcp/result")

            if result.success:
                self.get_logger().info(f"✅ Инструмент {tool_name} выполнен успешно")
            else:
                self.get_logger().warning(f"❌ Инструмент {tool_name} завершился с ошибкой: {result.error}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга JSON запроса: {e}")
            self._publish_error(f"Ошибка парсинга JSON: {str(e)}", "")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка выполнения запроса: {e}")
            self._publish_error(f"Внутренняя ошибка: {str(e)}", "")

    def on_perception_update(self, msg):
        """Обработка обновления контекста восприятия.

        Проекция (issue #2532) живёт в ``perception_projection`` — что именно
        из 27 полей ``PerceptionEvent`` видит Личность, решается там, один раз
        и под тестом паритета с IDL. Здесь остаётся тонкий адаптер.

        ``hasattr``-гвардов тут больше нет намеренно: прежде они превращали
        расхождение имён полей с IDL в тихий ноль (батарея не доходила до LLM
        вообще, timestamp всегда был 0). Теперь дрейф IDL — это громкая ошибка
        в логе, а не молчаливая потеря данных.
        """
        try:
            context = project_perception_event(msg)

            # battery_percent = None означает «данных о батарее ещё не было»
            # (агрегатор пишет 0.0 В по умолчанию). Не путаем с «разряжена».
            battery_percent = context["battery_percent"]
            if battery_percent is not None:
                self.battery_tool.update_battery(battery_percent)

            self.perception_context_tool.update_context(context)

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка обновления контекста: {e}")

    def _publish_error(self, error_message: str, request_id: str = ""):
        """Опубликовать сообщение об ошибке."""
        response = {"tool_name": None, "request_id": request_id, "result": {"success": False, "error": error_message}}

        msg = String()
        msg.data = json.dumps(response, ensure_ascii=False)
        self.result_pub.publish(msg)


def _recommended_executor_threads() -> int:
    """Return a safe worker count for ROS callbacks in containerized runtime.

    Mapping/navigation tools synchronously wait for action/service futures from inside
    subscription callbacks. If the executor ends up with a single worker thread, the
    waiting callback can starve the service response callback and create a false timeout.
    """
    try:
        affinity_count = len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        affinity_count = os.cpu_count() or 0
    return max(2, affinity_count or 0)


def _make_executor(node: "MCPServer"):
    """Build the executor for the MCP server.

    Uses ``MultiThreadedExecutor`` so subscription callbacks (e.g.
    ``/voice/tts/finished``) can run while a tool call blocks in ``registry.execute``.

    When ``MCP_SPIN_DIAG=1`` is set, wraps it with a diagnostic that logs — every ~10s —
    how many times the executor woke up and which entity kinds were ready. This is used
    to pinpoint the idle CPU spin (see ``/memories/repo/mcp-server-cpu-loop-2026-08-22.md``):
    the ready-entity distribution shows whether action-client waitables keep the WaitSet
    perpetually ready over ``rmw_zenoh_cpp``.
    """
    from rclpy.executors import MultiThreadedExecutor

    if os.environ.get("MCP_SPIN_DIAG", "0").lower() not in ("1", "true", "yes"):
        return MultiThreadedExecutor(num_threads=_recommended_executor_threads())

    import time
    from collections import Counter

    class _SpinDiagExecutor(MultiThreadedExecutor):
        _DIAG_WINDOW_S = 10.0

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._diag_logger = node.get_logger()
            self._diag_wakeups = 0
            self._diag_ready = Counter()
            self._diag_last = time.monotonic()
            self._diag_cpu = time.process_time()

        def wait_for_ready_callbacks(self, *args, **kwargs):
            handler, entity, _node = super().wait_for_ready_callbacks(*args, **kwargs)
            self._diag_wakeups += 1
            self._diag_ready[self._diag_entity_name(entity)] += 1
            now = time.monotonic()
            if now - self._diag_last >= self._DIAG_WINDOW_S:
                cpu_s = time.process_time() - self._diag_cpu
                top = ", ".join(
                    f"{name}={count}" for name, count in self._diag_ready.most_common(10)
                )
                self._diag_logger.info(
                    f"🔬 [spin-diag] {self._DIAG_WINDOW_S:.0f}s: wakeups={self._diag_wakeups} "
                    f"cpu_s={cpu_s:.1f} ready=[{top}]"
                )
                self._diag_wakeups = 0
                self._diag_ready.clear()
                self._diag_last = now
                self._diag_cpu = time.process_time()
            return handler, entity, _node

        @staticmethod
        def _diag_entity_name(entity) -> str:
            if hasattr(entity, "topic_name"):
                return f"sub:{entity.topic_name}"
            if hasattr(entity, "srv_name"):
                return f"srv:{entity.srv_name}"
            if hasattr(entity, "timer_period_ns"):
                return "timer"
            try:
                from rclpy.guard_condition import GuardCondition

                if isinstance(entity, GuardCondition):
                    return "guard"
            except Exception:  # noqa: BLE001
                pass
            return f"waitable:{type(entity).__name__}"

    return _SpinDiagExecutor(num_threads=_recommended_executor_threads())


def main(args=None):
    rclpy.init(args=args)
    node = MCPServer()

    # Используем MultiThreadedExecutor для параллельной обработки callbacks
    # Это позволяет получать /voice/tts/finished пока execute() блокируется
    executor = _make_executor(node)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
