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
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import json
import math
import os
import traceback
from typing import Any, Callable, Dict, Optional

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
    TaskDeltaTool,
    MemorySaveTool,
    MemorySearchTool,
    MemoryContextTool,
    MusicManager,
    TrackLibrary,
    ExecuteMusicCodeTool,
    ComposeMusicTool,
    StopMusicTool,
    SetVibePresetTool,
    GetMusicStateTool,
    SaveTrackTool,
    ListTracksTool,
    LoadTrackTool,
    DeleteTrackTool,
    SetDjModeTool,
    SearchSamplesTool,
    FaqSearchTool,
    SearchWebTool,
)

# Issue #1392 — MiniMax music generation + generated-music library tools.
# Imported explicitly (not via wildcard) so the heavy ``tools.music``
# dependency tree stays opt-in.
try:
    from .core.minimax_music_client import MinimaxMusicClient
    from .core.generated_music_library import GeneratedMusicLibrary
    from .tools.minimax_music import (
        GenerateMusicTool,
        GenListLibraryTool,
        GenSearchLibraryTool,
        GenSaveToLibraryTool,
        GenPlayFromLibraryTool,
        GenDeleteFromLibraryTool,
        GenGetTrackInfoTool,
    )
    _MINIMAX_MUSIC_AVAILABLE = True
except ImportError as _exc:  # noqa: BLE001
    MinimaxMusicClient = None  # type: ignore[assignment,misc]
    GeneratedMusicLibrary = None  # type: ignore[assignment,misc]
    GenerateMusicTool = GenListLibraryTool = GenSearchLibraryTool = None  # type: ignore[assignment,misc]
    GenSaveToLibraryTool = GenPlayFromLibraryTool = None  # type: ignore[assignment,misc]
    GenDeleteFromLibraryTool = GenGetTrackInfoTool = None  # type: ignore[assignment,misc]
    _MINIMAX_MUSIC_AVAILABLE = False
    _MINIMAX_MUSIC_IMPORT_ERROR = str(_exc)
from .mcp_auth import RequestAuthenticator
from .waypoint_store import WaypointStore
from .mapping_state import MappingState
from .voice_state import VoiceStateStore

try:
    from rob_box_voice.core.voice_memory import VoiceMemory as _VoiceMemory
    from rob_box_voice.core.faq_store import FAQStore as _FAQStore
    from rob_box_voice.core.event_profile import load_event_profile as _load_event_profile
except ImportError:
    _VoiceMemory = None  # type: ignore[assignment,misc]
    _FAQStore = None
    _load_event_profile = None


class MCPServer(Node):
    """
    MCP Server - центральная нода для управления инструментами

    Предоставляет инструменты для LLM и обрабатывает запросы на их выполнение.
    """

    def __init__(self):
        super().__init__("mcp_server")

        # Параметры ноды
        # Громкость музыки — ДВА разных параметра, см.
        # docs/analysis/2026-08-30-music-quality-audit.md (RC1).
        #
        # music_max_amp — санитарный потолок ОДНОГО слоя, не регулятор
        # громкости. Issue 986 («музыка орала, голос не был слышен») чинили
        # понижением до 0.42, но это выравнивало все слои по одному потолку:
        # микс становился плоским, а клиппинг оставался (капается каждый amp,
        # а не их сумма — 4 слоя * 0.42 = 1.68 на шине). Теперь сумму держит
        # синт masterlimiter в scsynth, поэтому потолок поднят.
        self.declare_parameter("music_max_amp", 0.85)
        # music_master_gain — ЕДИНСТВЕННАЯ ручка уровня музыки относительно
        # речи: мастер-фейдер ПОСЛЕ лимитера (/n_set 999 gain <v>).
        # Внутренняя динамика микса при этом сохраняется.
        self.declare_parameter("music_master_gain", 0.5)
        # Issue #1219 — активный TTS-провайдер для валидации голосов в
        # speak_text/set_voice. Должен совпадать с tts_node.yaml provider
        # (minimax). Используется для выбора списка голосов (Q4).
        self.declare_parameter("tts_provider", "minimax")

        # Реестр инструментов
        self.registry = MCPToolRegistry()

        # Каждый блок инициализации оборачивается в ``_init_step`` — это
        # fail-fast wrapper, который:
        #   * логирует FATAL через ROS2-логгер (видно в ``head -50`` workflow'а),
        #   * поднимает RuntimeError, чтобы процесс упал с traceback, а не
        #     тихо деградировал (как было в инциденте #1736 29.08: mcp_server
        #     exit 1 без traceback, потому что ``__init__`` ROS2-ноды не
        #     оборачивался).
        # Issue #1736: логи ``L-Deploy and Verify`` режут ``head -50`` + ``tail -150``,
        # middle logs SKIPPED — exit без явного stack trace теряется в
        # "SKIPPED MIDDLE LOGS". Теперь любой необработанный exception во
        # время ``__init__`` виден ДО 50-й строки (fatal) И сохраняется
        # в stacktrace через RuntimeError.

        # Долгосрочная память (VoiceMemory) — инициализировать ДО регистрации инструментов
        self.voice_memory = None
        self._init_step("voice_memory", self._init_voice_memory)

        # FAQ store + event profile (event mode)
        self.faq_store = None
        self.event_profile = None
        self._init_step("faq_store", self._init_faq_store)

        # WaypointStore — SQLite CRUD для вейпоинтов (одна БД с VoiceMemory)
        self.waypoint_store = self._init_step("waypoint_store", self._init_waypoint_store)

        # MappingState — FSM персистентное состояние (localization / mapping)
        self.mapping_state = MappingState()
        _ms = self.mapping_state.get()
        self.get_logger().info(
            f"🗺️  MappingState: mode={_ms['mode']}, map='{_ms.get('map_name') or 'none'}'"
        )

        # Лёгкий снимок позиции из /odom (вместо тяжёлого TF2 Listener на /tf 110 Гц)
        self._pose_snapshot: "Dict[str, float] | None" = None
        self._init_pose_subscription()

        # Регистрация инструментов — критична, без неё нода бесполезна
        self._init_step("register_tools", self._register_tools)


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

        # Issue 989 Fix C: публикуем состояние музыки для audio_node, чтобы
        # тот поднимал VAD threshold при активной музыке (strict mode).
        # audio_node подписывается на /voice/music/state ("playing"/"idle").
        self.music_state_pub = self.create_publisher(String, "/voice/music/state", qos_profile)

        # 🔴 FIX (live 30.08, vision-pi 12:33): mp3-трек из
        # ``gen_play_from_library`` играет в ``sound_node``, а не в Renardo.
        # ``MusicManager.stop_all()`` про него ничего не знает, поэтому и
        # ``music_cleanup``, и watchdog его не гасили: юзер сказал «останови
        # музыку», робот ответил «Музыка выключена.», а трек доиграл до
        # конца. Единственным местом, которое реально его останавливало,
        # был ``StopMusicTool``. Публикуем те же два топика здесь, а тул
        # теперь делегирует сюда (одна точка правды).
        self.sound_stop_pub = self.create_publisher(String, "/voice/sound/stop", qos_profile)
        self.generated_music_state_pub = self.create_publisher(
            String, "/voice/generated_music/state", qos_profile
        )

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
        #
        # Issue #1736: при гонке за первый запуск ``_read_or_create_token_file``
        # может вернуть None — раньше это была SILENT DEGRADATION (authenticator
        # enabled=False, все execute() отклоняются, но процесс жив). Теперь,
        # если allow_unauthenticated=False и токен не получен, _init_step
        # фейлит __init__ с FATAL-логом, который виден в head -50 workflow'а
        # (раньше exit 1 без stack trace попадал в SKIPPED MIDDLE LOGS).
        self.authenticator = self._init_step(
            "authenticator",
            lambda: RequestAuthenticator.from_env(logger=self.get_logger()),
        )
        if self.authenticator is None or not self.authenticator.enabled:
            # Если ROB_BOX_MCP_ALLOW_UNAUTHENTICATED=1 явно — пропускаем (это
            # документированный escape hatch и логируется как ERROR в
            # from_env). Иначе — fail-fast.
            allow_unauth = os.getenv(
                "ROB_BOX_MCP_ALLOW_UNAUTHENTICATED", ""
            ).strip().lower() in ("1", "true")
            if not allow_unauth:
                raise RuntimeError(
                    "mcp_auth: секрет недоступен и ROB_BOX_MCP_ALLOW_UNAUTHENTICATED "
                    "не выставлен — mcp_server не может безопасно принимать "
                    "/mcp/execute. Задай ROB_BOX_MCP_TOKEN или выдай доступ "
                    "на запись к /data/.mcp_token."
                )
            self.get_logger().error(
                "⚠️ mcp_auth: токен недоступен + ROB_BOX_MCP_ALLOW_UNAUTHENTICATED=1 "
                "— принимаем все /mcp/execute без подписи (security risk)."
            )

        # Подписка на perception context для обновления инструментов
        try:
            from rob_box_perception_msgs.msg import PerceptionEvent

            self.perception_sub = self.create_subscription(PerceptionEvent, "/perception/context_update", self.on_perception_update, 10)
            self.get_logger().info("✅ Подписан на /perception/context_update")
        except ImportError:
            self.get_logger().warning("⚠️ PerceptionEvent не найден, мониторинг контекста отключен")

        # Issue #1229 — фактический провайдер TTS (после фолбека) от tts_node.
        # tts_node публикует JSON {"provider": str, "voice": str, ...} после
        # старта/фолбека/синтеза. SpeakTextTool/SetVoiceTool валидируют голоса
        # по РЕАЛЬНОМУ провайдеру (а не номинальному tts_provider из YAML):
        # иначе LLM выбирает minimax-голоса, которых нет у yandex после
        # фолбека, и слышит один и тот же дефолтный голос.
        self.actual_tts_provider: str | None = None
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

        # --------------------------------------------------------------
        # Music session safety-net — issue #935
        # --------------------------------------------------------------
        # Two redundant mechanisms so the robot stops playing music when
        # the LLM forgets (e.g. the tool-loop is exhausted at
        # ``_MAX_TOOL_ITERATIONS=5``).  Either of them is sufficient on
        # its own; both together = belt-and-braces.
        #
        # (1) Periodic watchdog timer runs ``auto_stop_idle_music`` every
        #     ``_MUSIC_WATCHDOG_PERIOD_S`` seconds (default 5s).
        # (2) ``/mcp/music_cleanup`` topic subscription lets the
        #     dialogue_node push a single-shot "dialogue ended" message
        #     that triggers ``stop_music_on_session_end`` immediately.
        try:
            self._music_watchdog_period_s: float = float(
                os.environ.get("MUSIC_WATCHDOG_PERIOD_S", "5.0")
            )
        except (TypeError, ValueError):
            self._music_watchdog_period_s = 5.0
        self._music_watchdog_enabled: bool = (
            os.environ.get("MUSIC_WATCHDOG_ENABLED", "true").lower()
            in ("1", "true", "yes", "on")
        )
        # Subscribe to /mcp/music_cleanup — payload is JSON like
        # {"reason": "dialogue_end"} or {"reason": "shutdown"}. Empty
        # payload defaults to dialogue_end.
        try:
            self._music_cleanup_sub = self.create_subscription(
                String,
                "/mcp/music_cleanup",
                self._on_music_cleanup,
                qos_profile,
            )
            self.get_logger().info("🎵 Подписан на /mcp/music_cleanup (issue #935)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось подписаться на /mcp/music_cleanup: {exc}"
            )

        # 🔴 FIX (live 10:13 DJ): подписка на /voice/dj_mode — watchdog
        # должен знать, что DJ-режим активен (непрерывный сет с
        # переходами каждые 30-120с). Без этого segments-дедлайн #990
        # (~30с при segments:16) убивал музыку посреди DJ-сета:
        # «чуть музыки потом замолкает».
        try:
            self._dj_active = False
            self._dj_mode_sub = self.create_subscription(
                String,
                "/voice/dj_mode",
                self._on_dj_mode,
                qos_profile,
            )
            self.get_logger().info("🎧 Подписан на /voice/dj_mode (DJ watchdog)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось подписаться на /voice/dj_mode: {exc}"
            )

        if self._music_watchdog_enabled:
            period = max(0.1, self._music_watchdog_period_s)
            try:
                self._music_watchdog_timer = self.create_timer(
                    period, self._run_music_watchdog
                )
                self.get_logger().info(
                    f"🎵 Music watchdog timer запущен (period={period}s, "
                    f"ttl={self._music_manager._auto_stop_ttl_seconds if self._music_manager else '?'}s)"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"⚠️ Не удалось запустить watchdog timer: {exc}"
                )

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
        self.get_logger().info(
            f"🎙️ [issue 1229] actual TTS provider → '{self.actual_tts_provider}' "
            f"(reason: {payload.get('reason')})"
        )

    def _on_dj_mode(self, msg: "String") -> None:
        """Track DJ-mode state so the watchdog doesn't kill DJ sets.

        DJ-режим = непрерывный сет с переходами каждые 30-120с.
        segments-дедлайн (#990) ставится на каждый execute_music_code
        (~30с при segments:16) — если DJ активен и мы его соблюдаем,
        музыка умирает посреди сета. Пока DJ включён — дедлайн
        игнорируется; музыка живёт по idle-TTL (300с), а каждый
        переход обновляет активность.
        """
        try:
            data = json.loads(msg.data) if msg.data else {}
            self._dj_active = bool(data.get("enabled", False))
            state = "ON" if self._dj_active else "OFF"
            self.get_logger().info(f"🎧 [DJ watchdog] DJ mode: {state}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ DJ mode parse failed: {exc}")

    def _on_music_cleanup(self, msg: "String") -> None:
        """Topic callback: force-stop music on dialogue/shutdown events.

        Triggered by messages on ``/mcp/music_cleanup`` published by
        :class:`dialogue_node` when the dialogue ends. Payload is JSON;
        an empty/non-JSON string defaults to ``reason="dialogue_end"``.
        """
        if not getattr(self, "_music_manager", None):
            return
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (TypeError, ValueError):
            payload = {}
        reason = str(payload.get("reason", "dialogue_end")) if isinstance(payload, dict) else "dialogue_end"
        result = self._music_manager.stop_music_on_session_end()
        if result.get("was_active"):
            self.get_logger().warning(
                f"🎵 [{reason}] Авто-стоп {len(result.get('stopped_patterns', []))} "
                f"активных паттернов (issue #935). msg={result.get('message')}"
            )
        else:
            self.get_logger().info(
                f"🎵 [{reason}] Cleanup: активной музыки не обнаружено "
                f"(stop_all вызван профилактически). msg={result.get('message')}"
            )
        # Renardo погашен — гасим и mp3-трек в sound_node (см. комментарий
        # у ``sound_stop_pub``).
        self.stop_generated_track_playback()

    def stop_generated_track_playback(self) -> None:
        """Остановить mp3 из библиотеки сгенерированной музыки.

        ``gen_play_from_library`` публикует путь в ``sound_node``; ни
        ``MusicManager.stop_all()``, ни ``/g_freeAll`` до него не достают.
        Одна точка правды для ``StopMusicTool``, ``music_cleanup`` и
        watchdog — issue #1392 follow-up.
        """
        try:
            msg = String()
            msg.data = "STOP"
            self.sound_stop_pub.publish(msg)
            state = String()
            state.data = json.dumps({"status": "idle"})
            self.generated_music_state_pub.publish(state)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось остановить mp3 в sound_node: {exc}"
            )

    def _on_music_fallback(self, msg: "String") -> None:
        """Issue #1016 — play the top-rated library track when the LLM
        returned an empty reply to a music request.

        Triggered by messages on ``/mcp/music_fallback`` published by
        :class:`dialogue_node` (empty-response branch). The library query
        is already ordered ``rating DESC, name ASC`` (:meth:`TrackLibrary.
        list_tracks`), so the first result is the best human track we have.

        Best-effort: if the music stack is unavailable, or the library is
        empty, this is a silent no-op (the robot already said "Принял.").
        """
        manager = getattr(self, "_music_manager", None)
        library = getattr(self, "_track_library", None)
        if manager is None or library is None:
            self.get_logger().warning(
                "🎵 [music_fallback] music manager/library unavailable — skip"
            )
            return
        try:
            reason = ""
            if msg.data:
                try:
                    payload = json.loads(msg.data)
                    if isinstance(payload, dict):
                        reason = f" ({payload.get('reason', '')})"
                except (TypeError, ValueError):
                    pass
            listing = library.list_tracks(min_rating=0)
            tracks = listing.get("tracks", [])
            if not tracks:
                self.get_logger().warning(
                    "🎵 [music_fallback] библиотека пуста — нечего играть"
                )
                return
            top = tracks[0]
            loaded = library.load_track(top["name"])
            if not loaded.get("success"):
                self.get_logger().warning(
                    f"🎵 [music_fallback] не удалось загрузить "
                    f"'{top['name']}': {loaded.get('error')}"
                )
                return
            result = manager.execute_code(
                loaded["code"], pattern_name=top["name"]
            )
            if result.get("success"):
                self.get_logger().info(
                    f"🎵 [music_fallback{reason}] играю топ-трек "
                    f"'{top['name']}' (rating={top.get('rating')})"
                )
            else:
                self.get_logger().warning(
                    f"🎵 [music_fallback] топ-трек '{top['name']}' "
                    f"не запустился: {result.get('error')}"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [music_fallback] обработчик упал: {exc}"
            )

    def _run_music_watchdog(self) -> None:
        """Timer callback: auto-stop idle music when TTL is exceeded."""
        manager = getattr(self, "_music_manager", None)
        if manager is None:
            return
        try:
            # 🔴 FIX (live 10:13 DJ): проброс DJ-флага в MusicManager —
            # watchdog не должен убивать непрерывный DJ-сет по
            # segments-дедлайну #990.
            if hasattr(self, "_dj_active"):
                manager._dj_active = bool(self._dj_active)
            result = manager.auto_stop_idle_music()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Music watchdog failed: {exc}"
            )
            return
        if result.get("stopped"):
            patterns = result.get("active_patterns", [])
            idle = result.get("idle_seconds", "?")
            ttl = result.get("ttl_seconds", "?")
            # 🔴 FIX (live 30.08): без ``stop_reason`` строка врала — писала
            # «после 20.1s (ttl=300s)», хотя музыку убил segments-дедлайн,
            # а не idle-TTL. Из лога было не понять, почему бит прожил 20
            # секунд при ttl=300.
            reason = result.get("stop_reason", "idle_ttl")
            deadline_segments = result.get("deadline_segments")
            self.get_logger().warning(
                f"🎵 [watchdog] Авто-стоп {len(patterns)} паттернов: "
                f"reason={reason} idle={idle:.1f}s ttl={ttl:.0f}s"
                + (f" segments={deadline_segments}" if deadline_segments else "")
                + ". Issue #935."
            )
            self.stop_generated_track_playback()
        # Issue 989 Fix C: синхронизируем состояние музыки для audio_node
        # (поднятие VAD threshold при активной музыке). Watchdog тикает
        # каждые ~5s — достаточно для strict mode; tool-вызовы публикуют
        # состояние немедленно (см. publish_music_state).
        self.publish_music_state()

    def publish_music_state(self) -> None:
        """Опубликовать /voice/music/state: "playing" если музыка активна, иначе "idle".

        Issue 989 Fix C: audio_node слушает этот топик и поднимает порог VAD
        при активной музыке, чтобы бит не триггерил «речь» (эхо-петля).
        Музыка считается активной, если у MusicManager есть открытая сессия
        (``music_session_active_since`` не None) или именованные паттерны.
        """
        manager = getattr(self, "_music_manager", None)
        pub = getattr(self, "music_state_pub", None)
        if manager is None or pub is None:
            return
        try:
            state = manager.get_state()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"⚠️ publish_music_state: get_state failed: {exc}")
            return
        playing = bool(state.get("active_patterns")) or state.get("music_session_active_since") is not None
        msg = String()
        msg.data = "playing" if playing else "idle"
        pub.publish(msg)

    def _init_waypoint_store(self) -> WaypointStore:
        """Инициализация WaypointStore (SQLite для вейпоинтов)."""
        import os

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        try:
            store = WaypointStore(db_path=db_path)
            active = store.get_active_map()
            if active:
                wp_count = len(store.list_waypoints())
                self.get_logger().info(
                    f"📍 WaypointStore: карта '{active['name'] or active['map_id'][:8]}', "
                    f"{wp_count} точек"
                )
            else:
                self.get_logger().info("📍 WaypointStore: активная карта не задана (будет создана при первом сохранении)")
            return store
        except Exception as exc:
            self.get_logger().error(f"❌ Ошибка инициализации WaypointStore: {exc}")
            # Fallback — create in-memory so tools don't crash
            return WaypointStore(db_path=":memory:")

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

    def _init_step(self, name: str, fn: Callable[[], Any]):
        """Запустить блок инициализации ``fn`` под ``try/except`` с FATAL-логом.

        Проблема (issue #1736, 29.08.2026 staging run 33260223956): mcp_server
        падал с ``exit code 1`` в первые 5-10 секунд после старта launchsystem.
        В логах workflow'а (``L-Deploy and Verify.yml`` режет ``head -50`` +
        ``tail -150``) не было ни traceback, ни ERROR до 50-й строки — падение
        попадало в "SKIPPED MIDDLE LOGS". Причина: ``__init__`` ROS2-ноды
        выполняется в MainThread, и любое необработанное исключение роняет
        процесс без явного stacktrace в ROS2-логгер.

        Решение: любой блок инициализации, который ДОЛЖЕН работать, оборачивается
        в ``_init_step``. Это:
          1. Логирует шаг через ``info`` (видно в ``head -50`` при crash'е),
          2. На исключение логирует FATAL с полным traceback (``logger.fatal``
             в ROS2 пишет в stderr с поднятым уровнем — CI-deploy gate
             ловит его надёжно),
          3. Поднимает ``RuntimeError`` с контекстом — теперь crash ВСЕГДА
             идёт с явным stacktrace, который сохранится в ``tail -150``.

        Args:
            name: Человекочитаемое имя шага (для логов).
            fn:   Callable без аргументов, возвращает значение или None.

        Returns:
            Результат ``fn()``, либо ``None`` если она вернула ``None``.
        """
        self.get_logger().info(f"🔧 mcp_server init: {name}…")
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001
            # ``logger.fatal`` — это ROS2-специфичный уровень, который пишет
            # в stderr СРАЗУ, до любых буферов. Это критично, потому что
            # workflow ``L-Deploy and Verify`` смотрит только ``head -50`` + ``tail -150``,
            # и без немедленного вывода crash теряется в SKIPPED MIDDLE LOGS.
            tb = traceback.format_exc()
            self.get_logger().fatal(
                f"❌ mcp_server init failed at step '{name}': {exc}\n{tb}"
            )
            raise RuntimeError(
                f"mcp_server.__init__ failed at step '{name}': {exc}"
            ) from exc

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
        # Issue #1219 — SpeakTextTool и SetVoiceTool делят VoiceStateStore:
        # set_voice персистентно меняет голос на диалог, speak_text без
        # voice= говорит установленным голосом (Q7).
        voice_store = VoiceStateStore()
        self.registry.register(SpeakTextTool(self, voice_store=voice_store))
        self.registry.register(EstimateTtsDurationTool(self))
        self.registry.register(ListenForResponseTool(self))
        self.registry.register(SetVoiceTool(self, voice_store=voice_store))
        # Issue #1101 — LLM-driven speaker registration (replaces regex NLU).
        # LLM extracts name from user_input and calls register_speaker(name=X)
        # via MCP. speaker_id_node binds d-vector to name in /data/speakers.db.
        self.registry.register(RegisterSpeakerTool(self))
        self.registry.register(SearchWebTool(self))
        # Issue #968 (S6) — task_delta: schema-only registration so the
        # LLM sees the tool. Real execution is intercepted in-process by
        # SchedulerToolExecutor (rob_box_voice, S6.2) before it ever
        # reaches mcp_server — see TaskDeltaTool's docstring.
        self.registry.register(TaskDeltaTool(self))

        # Memory tools (долгосрочная память + семантический поиск)
        self.registry.register(MemorySaveTool(self))
        self.registry.register(MemorySearchTool(self))
        self.registry.register(MemoryContextTool(self))

        self._register_music_tools()

    def _register_music_tools(self) -> None:
        """Регистрирует music tools, не роняя весь MCP server при частичной деградации."""
        music_max_amp = self.get_parameter("music_max_amp").value
        music_master_gain = self.get_parameter("music_master_gain").value
        self.get_logger().info(
            f"🎵 Music max_amp: {music_max_amp:.2f}, "
            f"master_gain: {music_master_gain:.2f}"
        )

        try:
            music_manager = MusicManager(
                max_amp=music_max_amp, master_gain=music_master_gain
            )
        except Exception as exc:
            self.get_logger().error(
                f"❌ Music subsystem disabled: MusicManager init failed: {exc}"
            )
            return

        # Expose the manager so the watchdog / DialogueNode can hook into
        # session cleanup. Issue #935: when the dialog finishes without
        # ``stop_music``, an external safety-net (the dialog-end hook below)
        # calls ``music_manager.stop_music_on_session_end()`` to stop
        # playback automatically.
        self._music_manager: Optional[MusicManager] = music_manager
        self.registry.register(ExecuteMusicCodeTool(self, music_manager))
        # Форма трека строится кодом, а не LLM (RC4 в
        # docs/analysis/2026-08-30-music-quality-audit.md).
        self.registry.register(ComposeMusicTool(self, music_manager))
        self.registry.register(StopMusicTool(self, music_manager))
        self.registry.register(SetVibePresetTool(self, music_manager))
        self.registry.register(GetMusicStateTool(self, music_manager))
        self.registry.register(SetDjModeTool(self))
        self.registry.register(SearchSamplesTool(self))

        try:
            track_library = TrackLibrary()
        except Exception as exc:
            self.get_logger().error(
                f"❌ Music library disabled: TrackLibrary init failed: {exc}"
            )
            return

        self._track_library = track_library
        self.get_logger().info(f"🎵 Track library: {track_library.list_tracks()['total']} трек(ов)")
        self.registry.register(SaveTrackTool(self, track_library, music_manager))
        self.registry.register(ListTracksTool(self, track_library))
        self.registry.register(LoadTrackTool(self, track_library, music_manager))
        self.registry.register(DeleteTrackTool(self, track_library))

        # Issue #1392 — MiniMax music generation + persistent library.
        # Graceful degradation: any failure (no API key, no /data volume,
        # import error) only disables the new tools — Renardo tools keep
        # working. This mirrors the same try/except pattern used above.
        self._register_minimax_music_tools()

        # Issue #1016 — empty-response music fallback. When the LLM returns
        # an empty reply to a music request ("поставь что-нибудь", "сыграй
        # классику"), dialogue_node publishes /mcp/music_fallback and we
        # play the top-rated human track from the library instead of
        # leaving the user in silence.
        # 🔴 FIX (live 13.08): _register_music_tools() выполняется в
        # _register_tools() РАНЬШЕ, чем __init__ присваивает
        # self._qos_profile → AttributeError глотался try/except'ом, и
        # подписка на /mcp/music_fallback никогда не создавалась.
        qos = getattr(self, "_qos_profile", None)
        if qos is None:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._qos_profile = qos
        try:
            self._music_fallback_sub = self.create_subscription(
                String,
                "/mcp/music_fallback",
                self._on_music_fallback,
                qos,
            )
            self.get_logger().info("🎵 Подписан на /mcp/music_fallback (issue #1016)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось подписаться на /mcp/music_fallback: {exc}"
            )

    # ------------------------------------------------------------------
    # Issue #1392 — MiniMax music generation + persistent library tools.
    # ------------------------------------------------------------------

    def _register_minimax_music_tools(self) -> None:
        """Регистрирует 6 gen_* библиотечных тулзов (graceful degradation).

        Tools registered on success:
            gen_list_library, gen_search_library, gen_save_to_library,
            gen_play_from_library, gen_delete_from_library, gen_get_track_info

        ``generate_music`` НЕ регистрируется с 20.08.2026: MiniMax Music API
        отключён для новых юзеров (410 Gone, status_code 2153).

        Failure modes (each disables only the new tools, Renardo keeps working):
            * Module import failed (e.g. missing package)         → log + return
            * /data volume not writable (GeneratedMusicLibrary)   → log + skip lib tools
        """
        if not _MINIMAX_MUSIC_AVAILABLE:
            self.get_logger().warning(
                f"⚠️ MiniMax music tools disabled: import failed "
                f"({_MINIMAX_MUSIC_IMPORT_ERROR})"
            )
            return

        # Type-narrow for Pyright: _MINIMAX_MUSIC_AVAILABLE gates the rest.
        assert MinimaxMusicClient is not None
        assert GeneratedMusicLibrary is not None
        assert GenerateMusicTool is not None
        assert GenListLibraryTool is not None
        assert GenSearchLibraryTool is not None
        assert GenSaveToLibraryTool is not None
        assert GenPlayFromLibraryTool is not None
        assert GenDeleteFromLibraryTool is not None
        assert GenGetTrackInfoTool is not None

        # Library — initialised first; required by ALL gen_* tools.
        try:
            music_library = GeneratedMusicLibrary()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ MiniMax music library disabled: init failed ({exc}). "
                "gen_* tools will NOT be registered."
            )
            return
        self._generated_music_library = music_library
        self.get_logger().info(
            f"🎼 Generated music library: {music_library.count} трек(ов) "
            f"в {music_library.root_dir}"
        )

        # Register library-only tools (they don't need the API client).
        self.registry.register(GenListLibraryTool(self, music_library))
        self.registry.register(GenSearchLibraryTool(self, music_library))
        self.registry.register(GenSaveToLibraryTool(self, music_library))
        self.registry.register(GenDeleteFromLibraryTool(self, music_library))
        self.registry.register(GenGetTrackInfoTool(self, music_library))
        # play tool is registered even without client — uses library only
        self.registry.register(GenPlayFromLibraryTool(self, music_library))

        # 20.08.2026 — MiniMax Music API отключён для новых юзеров (410 Gone,
        # status_code 2153, проверено вживую). ``generate_music`` больше НЕ
        # регистрируется: LLM не должен видеть/вызывать мёртвый инструмент
        # (e2e regression: dj01 «сыграй renardo бит» → forbidden tool call).
        # gen_* библиотечные тулзы (list/search/save/play/delete/get_info)
        # остаются — они работают с локальной /data/music_library без API.
        self.get_logger().info(
            "🎼 MiniMax music generation disabled (API discontinued 410 Gone). "
            "gen_* library tools only."
        )

    def _init_voice_memory(self) -> None:
        """Инициализация VoiceMemory (долгосрочная память). Не падает при ошибках."""
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
            stats = self.voice_memory.get_stats()
            self.get_logger().info(
                f"🧠 VoiceMemory инициализирована: {db_path} "
                f"(turns={stats['turn_count']}, sessions={stats['session_count']}, "
                f"facts={stats['fact_count']}, vec={stats['vec_enabled']})"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ Ошибка инициализации VoiceMemory: {exc}")
            self.voice_memory = None

    def _init_faq_store(self) -> None:
        """Инициализация FAQStore и загрузка event profile (режим мероприятия)."""
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
        """Обработка обновления контекста восприятия."""
        try:
            # Обновляем battery tool
            if hasattr(msg, "battery_percentage"):
                self.battery_tool.update_battery(msg.battery_percentage)

            # Обновляем perception context tool
            # Конвертируем PerceptionEvent в dict для хранения
            context = {
                "timestamp": msg.timestamp if hasattr(msg, "timestamp") else 0.0,
                "internet_available": msg.internet_available if hasattr(msg, "internet_available") else False,
                "battery_percentage": msg.battery_percentage if hasattr(msg, "battery_percentage") else 0.0,
                "mapping_mode": msg.mapping_mode if hasattr(msg, "mapping_mode") else "unknown",
            }
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
