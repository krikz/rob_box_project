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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import os
from typing import Dict, Any

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
    MemorySaveTool,
    MemorySearchTool,
    MemoryContextTool,
    MusicManager,
    TrackLibrary,
    ExecuteMusicCodeTool,
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
)
from .waypoint_store import WaypointStore
from .mapping_state import MappingState

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
        # Issue 986: музыка орала, голос не был слышен — понизили max_amp с 0.7 до 0.42
        self.declare_parameter("music_max_amp", 0.42)

        # Реестр инструментов
        self.registry = MCPToolRegistry()

        # Долгосрочная память (VoiceMemory) — инициализировать ДО регистрации инструментов
        self.voice_memory = None
        self._init_voice_memory()

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

        # TF Buffer для определения текущей позиции робота
        self.tf_buffer = self._init_tf_buffer()

        # Регистрация инструментов
        self._register_tools()

        # QoS RELIABLE для гарантированной доставки результатов через Zenoh
        # BEST_EFFORT терял сообщения в сетевом окружении!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher для списка инструментов
        self.tools_pub = self.create_publisher(String, "/mcp/tools", qos_profile)

        # Publisher для результатов
        self.result_pub = self.create_publisher(String, "/mcp/result", qos_profile)

        # Issue 989 Fix C: публикуем состояние музыки для audio_node, чтобы
        # тот поднимал VAD threshold при активной музыке (strict mode).
        # audio_node подписывается на /voice/music/state ("playing"/"idle").
        self.music_state_pub = self.create_publisher(String, "/voice/music/state", qos_profile)

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

        # Подписка на perception context для обновления инструментов
        try:
            from rob_box_perception_msgs.msg import PerceptionEvent

            self.perception_sub = self.create_subscription(PerceptionEvent, "/perception/context_update", self.on_perception_update, 10)
            self.get_logger().info("✅ Подписан на /perception/context_update")
        except ImportError:
            self.get_logger().warning("⚠️ PerceptionEvent не найден, мониторинг контекста отключен")

        # Таймер для периодической публикации списка инструментов
        self.tools_timer = self.create_timer(10.0, self.publish_tools)

        # Публикуем список инструментов сразу при старте
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
            self.get_logger().warning(
                f"🎵 [watchdog] Авто-стоп {len(patterns)} паттернов после "
                f"{idle:.1f}s (ttl={ttl:.0f}s). Issue #935."
            )
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

    def _init_tf_buffer(self):
        """Инициализация TF2 Buffer + Listener для определения позиции робота."""
        try:
            import tf2_ros

            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer, self)
            self.get_logger().info("🗺️  TF2 Buffer + Listener инициализированы")
            return tf_buffer
        except Exception as exc:
            self.get_logger().error(f"❌ TF2 init failed: {exc}")
            return None

    def _register_tools(self):
        """Регистрация всех доступных инструментов."""
        # Navigation tools (require waypoint_store and/or tf_buffer)
        self.registry.register(NavigateToWaypointTool(self, self.waypoint_store))
        self.registry.register(NavigateToCoordinatesTool(self))
        self.registry.register(MoveDirectionTool(self))
        self.registry.register(StopNavigationTool(self))
        self.registry.register(ListWaypointsTool(self, self.waypoint_store))
        self.registry.register(SaveWaypointTool(self, self.waypoint_store, self.tf_buffer, self.mapping_state))
        self.registry.register(DeleteWaypointTool(self, self.waypoint_store))
        self.registry.register(ClearWaypointsTool(self, self.waypoint_store))
        self.registry.register(GetCurrentPoseTool(self, self.tf_buffer))

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
        self.registry.register(SpeakTextTool(self))
        self.registry.register(EstimateTtsDurationTool(self))
        self.registry.register(ListenForResponseTool(self))

        # Memory tools (долгосрочная память + семантический поиск)
        self.registry.register(MemorySaveTool(self))
        self.registry.register(MemorySearchTool(self))
        self.registry.register(MemoryContextTool(self))

        self._register_music_tools()

    def _register_music_tools(self) -> None:
        """Регистрирует music tools, не роняя весь MCP server при частичной деградации."""
        music_max_amp = self.get_parameter("music_max_amp").value
        self.get_logger().info(f"🎵 Music max_amp: {music_max_amp:.2f}")

        try:
            music_manager = MusicManager(max_amp=music_max_amp)
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

        self.get_logger().info(f"🎵 Track library: {track_library.list_tracks()['total']} трек(ов)")
        self.registry.register(SaveTrackTool(self, track_library, music_manager))
        self.registry.register(ListTracksTool(self, track_library))
        self.registry.register(LoadTrackTool(self, track_library, music_manager))
        self.registry.register(DeleteTrackTool(self, track_library))

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
        msg.data = json.dumps(tools, ensure_ascii=False, indent=2)
        self.tools_pub.publish(msg)
        self.get_logger().debug(f"📤 Опубликован список {len(tools)} инструментов")

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


def main(args=None):
    rclpy.init(args=args)
    node = MCPServer()

    # Используем MultiThreadedExecutor для параллельной обработки callbacks
    # Это позволяет получать /voice/tts/finished пока execute() блокируется
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=_recommended_executor_threads())
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
