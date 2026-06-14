#!/usr/bin/env python3
"""
dialogue_node.py — Voice dialogue agent (OpenAI Agents SDK + DeepSeek/MiMo)

Subscribes:
    /voice/stt/result (String) — recognised speech
    /audio/vad        (Bool)   — VAD for barge-in

Publishes:
    /voice/dialogue/response (String) — JSON response with SSML
    /voice/dialogue/state    (String) — current state (IDLE / LISTENING / DIALOGUE / SILENCED)
"""

import asyncio
import json
import os
import re
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from agents import Agent, Runner, function_tool
from agents.exceptions import MaxTurnsExceeded
from agents.items import ToolCallItem
from agents.model_settings import ModelSettings
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from httpx import Timeout as HttpxTimeout
from openai import (
    APIConnectionError,
    APITimeoutError,
    APIStatusError,
    RateLimitError,
    AsyncOpenAI,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

from .core.dialogue_manager import DialogueManager, DialogueState

try:
    from rob_box_voice.core.voice_memory import VoiceMemory as _VoiceMemory

    _VOICE_MEMORY_AVAILABLE = True
except ImportError:
    _VoiceMemory = None  # type: ignore[assignment,misc]
    _VOICE_MEMORY_AVAILABLE = False

try:
    from .core.faq_loader import load_faq_items as _load_faq_items
    from .core.faq_store import FAQStore as _FAQStore

    _FAQ_MODE_AVAILABLE = True
except ImportError:
    _load_faq_items = None  # type: ignore[assignment]
    _FAQStore = None  # type: ignore[assignment,misc]
    _FAQ_MODE_AVAILABLE = False

try:
    from .skills import (FAQSkill, MemorySkill, MusicSkill, NavigationSkill,
                         StatusSkill)

    _SKILLS_AVAILABLE = True
except ImportError:
    _SKILLS_AVAILABLE = False  # skills module not yet installed

# ── DuckDuckGo search (free, no API key) ──────────────────────────────────────
try:
    from ddgs import DDGS
    _DDGS_AVAILABLE = True
except ImportError:
    try:
        from duckduckgo_search import DDGS  # legacy name
        _DDGS_AVAILABLE = True
    except ImportError:
        _DDGS_AVAILABLE = False

# ── Feature flag — set USE_SKILLS=false to fall back to flat tool mode ────────
USE_SKILLS: bool = os.getenv("USE_SKILLS", "true").lower() == "true"


class DialogueNode(Node):
    """Voice dialogue agent built on OpenAI Agents SDK."""

    # ── Provider configs ────────────────────────────────────────────
    PROVIDERS = {
        "deepseek": {
            "base_url": "https://api.deepseek.com/v1",
            "model": "deepseek-v4-flash",
            "fallback_model": "deepseek-v4-flash",  # lighter model on timeout/errors
            "env_vars": ["DEEPSEEK_API_KEY", "LLM_API_KEY"],
        },
        "mimo": {
            "base_url": "https://api.xiaomimimo.com/v1",
            "model": "mimo-v2.5-pro",
            "fallback_model": "mimo-v2.5",  # lighter model on timeout/errors
            "env_vars": ["MIMO_API_KEY", "LLM_API_KEY"],
        },
    }

    def __init__(self):
        super().__init__("dialogue_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("provider", "deepseek")
        self.declare_parameter("api_key", "")
        self.declare_parameter("base_url", "")
        self.declare_parameter("model", "")
        self.declare_parameter("temperature", 0.7)
        self.declare_parameter("max_tokens", 500)
        self.declare_parameter("system_prompt_file", "master_prompt_compact.txt")
        self.declare_parameter("history_max_turns", 20)
        self.declare_parameter("agent_max_turns", 20)
        self.declare_parameter("dialogue_timeout", 300.0)
        self.declare_parameter("wake_words", ["робок", "робот", "роббокс"])
        self.declare_parameter("enable_mcp_tools", True)
        self.declare_parameter("enable_fallback", False)
        self.declare_parameter("fallback_model", "")  # empty = use PROVIDERS default
        self.declare_parameter("llm_timeout_sec", 90.0)
        self.declare_parameter("verbose_llm", True)
        self.declare_parameter("faq_mode_enabled", False)
        self.declare_parameter("faq_event_config_file", "")
        # Tool names whose turns are excluded from conversation history.
        # Prevents DeepSeek V3 multi-turn FC pattern-completion bug where the
        # model sees e.g. save_waypoint(X)->"X saved!" in history and skips
        # tool calls for the next similar request.
        # Configure via voice_assistant.yaml: history_excluded_tools: ["handle_navigation"]
        self.declare_parameter("history_excluded_tools", ["handle_navigation"])

        self._provider: str = self.get_parameter("provider").value
        self._temperature: float = self.get_parameter("temperature").value
        self._max_tokens: int = self.get_parameter("max_tokens").value
        self._max_turns: int = self.get_parameter("history_max_turns").value
        self._agent_max_turns: int = self.get_parameter("agent_max_turns").value
        self._llm_timeout: float = self.get_parameter("llm_timeout_sec").value
        self._verbose_llm: bool = self.get_parameter("verbose_llm").value
        self._history_excluded_tools: set = set(
            self.get_parameter("history_excluded_tools").value
        )

        # ── System prompt ────────────────────────────────────────────
        self._system_prompt: str = self._load_system_prompt()

        # ── Dialogue state ───────────────────────────────────────────
        self.dialogue_manager = DialogueManager(
            wake_words=self.get_parameter("wake_words").value,
            dialogue_timeout=self.get_parameter("dialogue_timeout").value,
        )
        self._vad_speech_detected: bool = False

        # ── Conversation history (SDK input-list format) ─────────────
        self._conversation: List[dict] = []
        self._conv_lock = threading.Lock()

        # ── asyncio loop in daemon thread ────────────────────────────
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        # ── Current agent task (cancelled on barge-in) ──────────────
        self._run_task: Optional[asyncio.Task] = None
        self._task_lock = threading.Lock()
        # Flag set by _cancel_run: speak_text checks this before acquiring the
        # output lock so that a still-running old speak_text coroutine (that
        # was unblocked by the forced TTS-event flush) does not race with the
        # new run's speak_text calls.
        self._run_cancelled: bool = False

        # ── VAD speech flag (no barge-in) ────────────────────────────
        # VAD only tracks whether speech is happening; barge-in requires
        # wake word via STT, so background noise never cancels a run.

        # ── TTS completion tracking ──────────────────────────────────
        # speak_text tool awaits these events so agent calls are sequential
        self._tts_events: dict = {}  # speech_id -> asyncio.Event
        self._tts_events_lock = threading.Lock()

        # ── Sound completion tracking ─────────────────────────────────
        # play_sound tool awaits this event until sound_node publishes "ready".
        # Avoids relying on catalog duration (which is shorter than real playback
        # due to audio startup latency + cleanup_playback_noise 0.1s delay).
        self._sound_done_event: Optional[asyncio.Event] = None
        self._sound_event_lock = threading.Lock()

        # ── spoken texts accumulator ──────────────────────────────────
        # Collects all speak_text calls for building clean history.
        self._spoken_texts: List[str] = []

        # ── speak_text dedup ─────────────────────────────────────────
        # LLM sometimes retries the same speak_text call multiple times.
        # Track recent texts with timestamps; skip duplicates within 5s.
        self._recent_speak: deque = deque(maxlen=10)
        self._recent_speak_lock = threading.Lock()

        # ── Tool call tracker ─────────────────────────────────────────
        # Tracks which MCP tools were called during an agent turn.
        # Used to add [tools: X] markers to conversation history so that
        # the LLM doesn't hallucinate actions without calling tools.
        self._tools_called: List[str] = []

        # ── Output serialisation lock ────────────────────────────────
        # speak_text / play_sound / play_animation must not run in parallel.
        # DeepSeek ignores parallel_tool_calls=False, so we enforce ordering
        # here: each output tool acquires the lock before executing and holds
        # it until the action is fully complete (TTS finished / sound done).
        self._output_lock = asyncio.Lock()

        # ── ROS2 pub/sub ─────────────────────────────────────────────
        cbg = ReentrantCallbackGroup()
        qos_r = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._response_pub = self.create_publisher(
            String, "/voice/dialogue/response", 10
        )
        self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
        self._sound_trigger_pub = self.create_publisher(
            String, "/voice/sound/trigger", 10
        )
        self._tts_control_pub = self.create_publisher(String, "/voice/tts/control", 10)

        self.create_subscription(
            String, "/voice/stt/result", self._on_stt, qos_r, callback_group=cbg
        )
        self.create_subscription(
            Bool, "/audio/vad", self._on_vad, 10, callback_group=cbg
        )
        self.create_subscription(
            String,
            "/voice/tts/finished",
            self._on_tts_finished_dlg,
            10,
            callback_group=cbg,
        )
        self.create_subscription(
            String, "/voice/sound/state", self._on_sound_state, 10, callback_group=cbg
        )
        self.create_subscription(
            String, "/voice/dj_mode", self._on_dj_mode_msg, 10, callback_group=cbg
        )

        # ── MCP Adapter ──────────────────────────────────────────────
        self._mcp: Optional[LLMToolCallAdapter] = None
        if self.get_parameter("enable_mcp_tools").value:
            try:
                self._mcp = LLMToolCallAdapter(self)
                self.get_logger().info("✅ MCP adapter ready")
            except Exception as exc:
                self.get_logger().error(f"❌ MCP adapter failed: {exc}")

        # ── Long-term memory (voice_turns DB) ─────────────────────────
        self._voice_memory = None
        self._init_voice_memory()

        # ── Event FAQ mode ──────────────────────────────────────────────
        self._event_profile: Optional[Dict[str, Any]] = self._load_event_profile()
        self._faq_store = None
        self._init_faq_store()

        # ── Build agent ──────────────────────────────────────────────
        self._agent: Optional[Agent] = None
        self._build_agent()

        # ── Timeout timer ────────────────────────────────────────────
        self.create_timer(5.0, self._on_timeout_check)

        # ── DJ mode state ─────────────────────────────────────────────
        self._dj_mode_enabled: bool = False
        self._dj_next_transition_at: float = 0.0
        self._dj_transition_count: int = 0
        self._dj_theme: str = ""  # party theme/context set at DJ mode activation
        self._dj_set_plan: str = ""  # full set plan generated by agent on transition #1
        self._dj_persona: str = ""  # DJ persona/name set by user (e.g. 'Демогорган')
        self._dj_stop_at: float = 0.0  # timestamp to auto-stop music after DJ disabled
        self.create_timer(5.0, self._on_dj_tick_check)

        self._log_config()
        self.get_logger().info("✅ DialogueNode (OpenAI Agents SDK) ready")

    # ────────────────────────────────────────────────────────────────
    # Agent construction
    # ────────────────────────────────────────────────────────────────

    def _load_system_prompt(self) -> str:
        prompt_file = self.get_parameter("system_prompt_file").value
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg = get_package_share_directory("rob_box_voice")
            path = os.path.join(pkg, "prompts", prompt_file)
            with open(path, "r", encoding="utf-8") as fh:
                prompt = fh.read()
            self.get_logger().info(
                f"✅ Prompt loaded: {prompt_file} ({len(prompt)} bytes)"
            )
            return prompt
        except Exception as exc:
            self.get_logger().warning(f"⚠️ Prompt not found ({exc}) — using default")
            return "Ты ROBBOX — умный робот-ассистент. Отвечай кратко и по делу."

    def _load_prompt_file(self, filename: str) -> str:
        """Load a prompt file by relative path under the package prompts/ directory.

        Args:
            filename: Relative path under prompts/, e.g. "compositor_prompt.txt"
                      or "skills/music_skill_prompt.txt".

        Returns:
            File contents, or empty string on failure.
        """
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg = get_package_share_directory("rob_box_voice")
            path = os.path.join(pkg, "prompts", filename)
            with open(path, "r", encoding="utf-8") as fh:
                content = fh.read()
            self.get_logger().info(
                f"✅ Prompt loaded: {filename} ({len(content)} bytes)"
            )
            return content
        except Exception as exc:
            self.get_logger().warning(f"⚠️ Prompt '{filename}' not found: {exc}")
            return ""

    def _resolve_api_key(self) -> str:
        key = self.get_parameter("api_key").value
        if key:
            return key
        for env in self.PROVIDERS.get(self._provider, {}).get("env_vars", []):
            val = os.environ.get(env, "")
            if val:
                return val
        raise RuntimeError(
            f"API key not found for provider '{self._provider}'. "
            f"Set one of: {self.PROVIDERS.get(self._provider, {}).get('env_vars', [])}"
        )

    def _resolve_base_url(self) -> str:
        val = self.get_parameter("base_url").value
        return val or self.PROVIDERS.get(self._provider, {}).get("base_url", "")

    def _resolve_model(self) -> str:
        val = self.get_parameter("model").value
        return val or self.PROVIDERS.get(self._provider, {}).get(
            "model", "deepseek-v4-flash"
        )

    def _resolve_fallback_model(self) -> str:
        """Resolve fallback model (lighter/faster) for timeout/error recovery."""
        val = self.get_parameter("fallback_model").value
        return val or self.PROVIDERS.get(self._provider, {}).get(
            "fallback_model", "deepseek-v4-flash"
        )

    def _init_voice_memory(self) -> None:
        """Init VoiceMemory for persistent turn logging. Fails silently."""
        if not _VOICE_MEMORY_AVAILABLE:
            self.get_logger().warning(
                "⚠️ VoiceMemory unavailable — turn logging disabled"
            )
            return
        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        ollama_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
        try:
            self._voice_memory = _VoiceMemory(
                db_path=db_path, ollama_base_url=ollama_url
            )
            stats = self._voice_memory.get_stats()
            self.get_logger().info(
                f"🧠 DialogueNode VoiceMemory: {db_path} "
                f"(turns={stats['turn_count']}, sessions={stats['session_count']})"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ VoiceMemory init failed: {exc}")
            self._voice_memory = None

    def _load_event_profile(self) -> Optional[Dict[str, Any]]:
        """Load event FAQ mode configuration from YAML when enabled."""
        if not self.get_parameter("faq_mode_enabled").value:
            return None

        config_file = self.get_parameter("faq_event_config_file").value
        if not config_file:
            self.get_logger().warning(
                "⚠️ FAQ mode enabled but faq_event_config_file is empty"
            )
            return None

        config_path = Path(config_file).expanduser()
        if not config_path.is_absolute():
            config_path = config_path.resolve()
        if not config_path.exists():
            self.get_logger().warning(f"⚠️ FAQ event config not found: {config_path}")
            return None

        try:
            payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            self.get_logger().error(f"❌ Failed to read FAQ event config: {exc}")
            return None

        event = payload.get("event", payload) if isinstance(payload, dict) else None
        if not isinstance(event, dict):
            self.get_logger().warning(
                "⚠️ FAQ event config must be a mapping or contain 'event:' block"
            )
            return None

        name = str(event.get("name", "")).strip()
        faq_file = str(event.get("faq_file", "")).strip()
        if not name or not faq_file:
            self.get_logger().warning(
                "⚠️ FAQ event config requires 'name' and 'faq_file'"
            )
            return None

        faq_path = Path(faq_file).expanduser()
        if not faq_path.is_absolute():
            faq_path = (config_path.parent / faq_path).resolve()

        date = str(event.get("date", "")).strip()
        event_id = str(event.get("id", "")).strip() or self._slugify_event_id(
            f"{name}-{date or faq_path.stem}"
        )

        return {
            "event_id": event_id,
            "name": name,
            "organization": str(event.get("organization", "")).strip(),
            "location": str(event.get("location", "")).strip(),
            "date": date,
            "description": str(event.get("description", "")).strip(),
            "robot_role": str(
                event.get("robot_role", "РОББОКС — ровер-помощник")
            ).strip(),
            "intro_identity": str(event.get("intro_identity", "")).strip(),
            "faq_file": str(faq_path),
        }

    def _slugify_event_id(self, value: str) -> str:
        slug = re.sub(r"[^a-zA-Z0-9]+", "-", value.lower()).strip("-")
        return slug or "faq-event"

    def _init_faq_store(self) -> None:
        """Load FAQ knowledge for the active event when FAQ mode is enabled."""
        if not self._event_profile:
            return
        if not _FAQ_MODE_AVAILABLE:
            self.get_logger().warning(
                "⚠️ FAQ mode modules unavailable — FAQ retrieval disabled"
            )
            return

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        ollama_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
        try:
            self._faq_store = _FAQStore(db_path=db_path, ollama_base_url=ollama_url)
            items = _load_faq_items(self._event_profile["faq_file"])
            loaded_count = self._faq_store.replace_items(
                event_id=self._event_profile["event_id"],
                items=items,
            )
            self.get_logger().info(
                f"📚 FAQ mode ready: event={self._event_profile['event_id']} rows={loaded_count}"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ FAQ mode init failed: {exc}")
            self._faq_store = None

    def _render_event_instructions(self, base_instructions: str) -> str:
        """Prepend active event context to agent instructions."""
        if not self._event_profile:
            return base_instructions

        profile = self._event_profile
        lines = [
            "[EVENT MODE]",
            f"Ты работаешь на мероприятии как {profile['robot_role']}.",
            f"Мероприятие: {profile['name']}.",
        ]
        if profile.get("organization"):
            lines.append(f"Организация: {profile['organization']}.")
        if profile.get("location"):
            lines.append(f"Локация: {profile['location']}.")
        if profile.get("date"):
            lines.append(f"Дата: {profile['date']}.")
        if profile.get("description"):
            lines.append(f"Описание: {profile['description']}")
        if profile.get("intro_identity"):
            lines.append(f"Самоидентификация: {profile['intro_identity']}")
        if self._faq_store:
            lines.append(
                "Если вопрос касается мероприятия, FAQ, поступления, программы, локации или организационных деталей, "
                "используй FAQ retrieval tool и говори по найденным данным."
            )
            lines.append(
                "Если пользователь просит рэп, стих, шутку, историю, песню или другой перформанс про тему мероприятия, "
                "сначала подними факты из FAQ и только потом стилизуй ответ."
            )
            lines.append(
                "Если после FAQ нужен бит или музыка, сначала получи факты, потом при необходимости вызови handle_music, "
                "и только после этого озвучивай ответ."
            )
        lines.append(
            "Даже если исходный ответ в FAQ длинный, озвучивай короткую, понятную, разговорную версию: 1-3 предложения."
        )
        lines.append(
            "Если точного ответа в FAQ нет, честно скажи это и не выдумывай детали."
        )
        return "\n".join(lines) + "\n\n" + base_instructions

    def _build_event_faq_prefetch_context(
        self, user_input: str, limit: int = 3
    ) -> Optional[str]:
        """Prefetch relevant FAQ facts for the current event-mode user turn."""
        if not self._faq_store or not self._event_profile:
            return None

        query = user_input.strip()
        event_id = self._event_profile.get("event_id")
        if not query or not event_id:
            return None

        try:
            results = self._faq_store.search(
                query=query,
                event_id=event_id,
                limit=limit,
            )
        except Exception as exc:
            self.get_logger().warning(f"⚠️ Event FAQ prefetch failed: {exc}")
            return None

        self.get_logger().info(
            f"📚 Event FAQ prefetch: {len(results)} matches for '{query[:80]}'"
        )
        if not results:
            return None

        lines = [
            "[EVENT FAQ PREFETCH]",
            "FAQ для текущего запроса уже проверен. Для всех фактических утверждений о мероприятии опирайся сначала на данные ниже.",
            "Если пользователь просит рэп, шутку, стих, историю, песню или другой стиль по теме мероприятия, сначала используй FAQ факты, а уже потом стилизуй ответ.",
            "Если нужен бит или музыкальный фон, сначала используй факты ниже, затем при необходимости вызови handle_music, а потом озвучь ответ.",
            "Если фактов ниже недостаточно, честно скажи, что точных деталей в FAQ не найдено, и не выдумывай их.",
            "Найденные FAQ факты:",
        ]
        for index, item in enumerate(results, start=1):
            lines.append(f"{index}. Вопрос: {item.get('question', '')}")
            lines.append(f"   Ответ: {item.get('answer', '')}")
            category = item.get("category")
            if category:
                lines.append(f"   Категория: {category}")
            source = item.get("source")
            if source:
                lines.append(f"   Источник: {source}")
        return "\n".join(lines)

    def _render_faq_skill_prompt(self, base_prompt: str) -> str:
        """Prepend event details to the FAQ sub-agent prompt."""
        if not self._event_profile:
            return base_prompt

        profile = self._event_profile
        lines = [
            f"Активное мероприятие: {profile['name']}",
            f"Роль робота: {profile['robot_role']}",
        ]
        if profile.get("organization"):
            lines.append(f"Организация: {profile['organization']}")
        if profile.get("location"):
            lines.append(f"Локация: {profile['location']}")
        if profile.get("date"):
            lines.append(f"Дата: {profile['date']}")
        if profile.get("description"):
            lines.append(f"Описание: {profile['description']}")
        return "\n".join(lines) + "\n\n" + base_prompt

    def _build_agent(self, model_override: str = "") -> None:
        """(Re)build the Agent — also called after fallback provider switch.

        Args:
            model_override: If set, use this model instead of the configured one.
                            Used for fallback on timeout (e.g. deepseek-v4-pro → flash).
        """
        try:
            api_key = self._resolve_api_key()
            base_url = self._resolve_base_url()
            model_name = model_override or self._resolve_model()

            openai_client = AsyncOpenAI(
                api_key=api_key,
                base_url=base_url,
                timeout=HttpxTimeout(60.0, connect=15.0),
                max_retries=3,
            )
            model = OpenAIChatCompletionsModel(
                model=model_name, openai_client=openai_client
            )

            if USE_SKILLS and _SKILLS_AVAILABLE and self._mcp:
                # ── Compositor mode: Compositor + 4 skill sub-agents ────────
                instructions = (
                    self._load_prompt_file("compositor_prompt.txt")
                    or self._system_prompt
                )
                instructions = self._render_event_instructions(instructions)
                agent_name = "Compositor"
                tools = self._make_output_tools() + self._build_skills(model)
                self.get_logger().info(
                    f"🎭 Skills mode: USE_SKILLS=True, {len(tools)} tools "
                    f"(3 output + {len(tools) - 3} skills)"
                )
            else:
                # ── Flat mode: single agent with all 17 tools ────────────────
                if USE_SKILLS and not _SKILLS_AVAILABLE:
                    self.get_logger().warning(
                        "⚠️ USE_SKILLS=true but skills module unavailable — falling back to flat mode"
                    )
                instructions = self._render_event_instructions(self._system_prompt)
                agent_name = "RobBox"
                tools = self._make_tools() if self._mcp else []

            self._agent = Agent(
                name=agent_name,
                instructions=instructions,
                tools=tools,
                model=model,
                model_settings=ModelSettings(
                    temperature=self._temperature,
                    max_tokens=self._max_tokens,
                    parallel_tool_calls=False,
                    # MiMo: disable thinking mode so tool_choice works correctly
                    # frequency_penalty/presence_penalty reduce verbatim repetition
                    extra_body={
                        "thinking": {"type": "disabled"},
                        "frequency_penalty": 0.3,
                        "presence_penalty": 0.2,
                    },
                ),
            )
            prompt_preview = instructions[:200].replace("\n", "↵")
            self.get_logger().info(
                f"🤖 Agent built: {model_name} @ {base_url} ({len(tools)} tools) | "
                f"instructions={len(instructions)} bytes | "
                f'preview="{prompt_preview}..."'
            )
        except Exception as exc:
            self.get_logger().error(f"❌ Agent build failed: {exc}")

    def _make_tools(self) -> list:
        """Create @function_tool wrappers around MCP ROS2 calls."""
        mcp = self._mcp
        samples_root = Path(
            os.getenv("RENARDO_SAMPLES_PATH", "/root/.config/renardo/samples")
        )
        # Captured once per tool-set build; bound to self._loop on first await.
        lock = self._output_lock

        async def _call(tool_name: str, params: dict, timeout: float = 10.0) -> str:
            self._tools_called.append(tool_name)
            result = await asyncio.get_running_loop().run_in_executor(
                None,
                lambda: mcp.execute_tool_call_sync(tool_name, params, timeout=timeout),
            )
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        @function_tool
        async def speak_text(text: str, animation: str = "neutral") -> str:
            """Произнести текст голосом. ОБЯЗАТЕЛЬНЫЙ инструмент — весь ответ ТОЛЬКО через speak_text!
            НЕЛЬЗЯ просто напечатать текст — пользователь ничего не услышит без вызова этого инструмента.
            Возвращает TASK_COMPLETE. После ПОСЛЕДНЕГО speak_text верни строку 'done'.
            """
            if self._run_cancelled:
                return "CANCELLED"
            # Strip history marker prefix — LLM sometimes copies [выполнено через: ...] from history
            text = re.sub(
                r"^\[(?:выполнено через|executed via):[^\]]*\]\s*", "", text
            ).strip()
            # ── Dedup: skip if same text was spoken within last 5 seconds ──
            now = time.time()
            with self._recent_speak_lock:
                for ts, recent_text in self._recent_speak:
                    if recent_text == text and (now - ts) < 5.0:
                        self.get_logger().debug(
                            f"🔇 speak_text dedup: skipping duplicate ({text[:40]}...)"
                        )
                        return "TASK_COMPLETE"
                self._recent_speak.append((now, text))
            # Collect ALL spoken texts immediately (before lock) so that when
            # multiple speak_text calls queue on the lock, _spoken_texts already
            # has all texts for proper history saving.
            self._spoken_texts.append(text)
            async with lock:
                if self._run_cancelled:
                    return "CANCELLED"
                result_str = await _call(
                    "speak_text", {"text": text, "animation": animation}, timeout=60.0
                )
                # ── Wait for TTS to actually finish playing ───────────────────
                # MCP speak_text is async (returns immediately), but we hold
                # the output lock until TTS finishes, so the next tool call
                # (play_sound, play_animation, etc.) only starts afterwards.
                try:
                    speech_id = (
                        json.loads(result_str).get("data", {}).get("speech_id", "")
                    )
                except Exception:
                    speech_id = ""
                if speech_id:
                    event = asyncio.Event()
                    with self._tts_events_lock:
                        self._tts_events[speech_id] = event
                    try:
                        await asyncio.wait_for(event.wait(), timeout=30.0)
                    except asyncio.TimeoutError:
                        self.get_logger().warning(
                            f"⚠️ speak_text TTS timeout for {speech_id[:8]}"
                        )
                    finally:
                        with self._tts_events_lock:
                            self._tts_events.pop(speech_id, None)
            return "TASK_COMPLETE"

        @function_tool
        async def play_sound(sound: str) -> str:
            """Воспроизвести звуковой эффект."""
            async with lock:
                # Register done-event BEFORE sending trigger to avoid race.
                done_event = asyncio.Event()
                with self._sound_event_lock:
                    self._sound_done_event = done_event
                result_str = await _call("play_sound", {"sound": sound})
                # Wait for sound_node to publish "ready" (actual playback finished,
                # including cleanup_playback_noise 0.1s delay).
                # Fallback timeout = catalog duration + 2s safety margin.
                try:
                    duration = (
                        json.loads(result_str).get("data", {}).get("duration", 1.5)
                    )
                except Exception:
                    duration = 1.5
                try:
                    await asyncio.wait_for(
                        done_event.wait(), timeout=float(duration) + 2.0
                    )
                except asyncio.TimeoutError:
                    self.get_logger().warning(
                        f"⚠️ play_sound timeout waiting for 'ready': {sound}"
                    )
                finally:
                    with self._sound_event_lock:
                        self._sound_done_event = None
            return result_str

        @function_tool
        async def play_animation(animation: str, duration: float = 3.0) -> str:
            """Запустить LED анимацию на указанное время."""
            async with lock:
                return await _call(
                    "play_animation", {"animation": animation, "duration": duration}
                )

        @function_tool
        async def memory_context(limit: int = 10) -> str:
            """Получить контекст прошлых диалогов из долгосрочной памяти."""
            return await _call("memory_context", {"limit": limit})

        @function_tool
        async def memory_save(fact: str, category: str = "general") -> str:
            """Сохранить факт о пользователе в долгосрочную память (имя, предпочтения, привычки). НЕ для мест/локаций — для этого используй save_waypoint!"""
            return await _call("memory_save", {"fact": fact, "category": category})

        @function_tool
        async def memory_search(query: str, limit: int = 5) -> str:
            """Найти релевантную информацию в долгосрочной памяти."""
            return await _call("memory_search", {"query": query, "limit": limit})

        @function_tool
        async def faq_search(query: str, limit: int = 3) -> str:
            """Найти точный ответ в FAQ активного мероприятия."""
            if not self._faq_store or not self._event_profile:
                return json.dumps([], ensure_ascii=False)
            event_id = self._event_profile.get("event_id") or self._slugify_event_id(
                self._event_profile.get("name", "faq-event")
            )
            results = self._faq_store.search(
                query=query, event_id=event_id, limit=limit
            )
            return json.dumps(results, ensure_ascii=False)

        @function_tool
        def search_samples(
            query: str,
            pack: str = "0_foxdot_default",
            case: str = "lower",
        ) -> str:
            """Search Renardo samples by filename and return play() hints."""
            if not samples_root.exists():
                return json.dumps(
                    {
                        "error": f"Samples dir not found: {samples_root}",
                        "hint": "Mount RENARDO_SAMPLES_PATH volume in Docker",
                    },
                    ensure_ascii=False,
                )

            pack_path = samples_root / pack
            if not pack_path.exists():
                available = [
                    item.name for item in samples_root.iterdir() if item.is_dir()
                ]
                return json.dumps(
                    {
                        "error": f"Pack '{pack}' not found",
                        "available_packs": available,
                    },
                    ensure_ascii=False,
                )

            exts = {".wav", ".aif", ".aiff", ".mp3"}
            all_packs = sorted(
                item.name for item in samples_root.iterdir() if item.is_dir()
            )
            spack_num = all_packs.index(pack) if pack in all_packs else 0
            spack_suffix = f", spack={spack_num}" if spack_num != 0 else ""

            if query.strip() == "*":
                overview = {}
                for folder in sorted(pack_path.iterdir()):
                    if not folder.is_dir() or folder.name.startswith("."):
                        continue
                    subfolder = folder / case
                    if not subfolder.exists():
                        subfolder = folder
                    count = sum(
                        1
                        for file_path in subfolder.iterdir()
                        if file_path.is_file() and file_path.suffix.lower() in exts
                    )
                    if count:
                        overview[folder.name] = count
                return json.dumps(
                    {
                        "pack": pack,
                        "case": case,
                        "letters": overview,
                        "hint": 'Search by word: search_samples("kick") or search_samples("synth", pack="1_pitchglitch_samples")',
                    },
                    ensure_ascii=False,
                    indent=2,
                )

            normalized_query = query.lower().strip()
            results = []
            for folder in sorted(pack_path.iterdir()):
                if not folder.is_dir() or folder.name.startswith("."):
                    continue
                subfolder = folder / case
                if not subfolder.exists():
                    subfolder = folder
                files = sorted(
                    file_path
                    for file_path in subfolder.iterdir()
                    if file_path.is_file() and file_path.suffix.lower() in exts
                )
                for sample_index, file_path in enumerate(files):
                    if normalized_query not in file_path.name.lower():
                        continue
                    play_letter = (
                        folder.name.upper() if case == "upper" else folder.name
                    )
                    results.append(
                        {
                            "letter": play_letter,
                            "sample_index": sample_index,
                            "spack": spack_num,
                            "filename": file_path.name,
                            "play_code": f'd1 >> play("{play_letter}", sample={sample_index}{spack_suffix})',
                        }
                    )
                if len(results) >= 30:
                    break

            if not results:
                return json.dumps(
                    {
                        "query": query,
                        "pack": pack,
                        "found": 0,
                        "hint": 'Try: "kick", "snare", "hat", "bass", "synth", "dist", "loop", "*"',
                    },
                    ensure_ascii=False,
                )

            return json.dumps(
                {
                    "query": query,
                    "pack": pack,
                    "case": case,
                    "found": len(results),
                    "results": results,
                },
                ensure_ascii=False,
            )

        # ── search_artist_style (DuckDuckGo, free) ──────────────────────────
        @function_tool
        def search_artist_style(artist_name: str, song_names: str = "") -> str:
            """Search for an artist's music style, genre, BPM, key, instruments and mood.

            MANDATORY: Call this BEFORE generating music when the user mentions
            a specific artist, band, or musician (e.g. "Егор Летов", "Radiohead",
            "Kraftwerk", "Daft Punk"). Use the results to adapt your Renardo code
            to match the artist's characteristic sound.

            Args:
                artist_name: Name of the artist or band to research.
                song_names: Optional comma-separated song/album names if user
                    mentioned specific tracks (e.g. "Русское поле экспериментов,
                    Гражданская оборона"). Searches for chords and structure.
            """
            if not _DDGS_AVAILABLE:
                return json.dumps(
                    {"error": "duckduckgo-search not installed", "artist": artist_name},
                    ensure_ascii=False,
                )

            queries = [
                f"{artist_name} жанр стиль музыки",
                f"{artist_name} звучание инструменты темп",
                f"{artist_name} music genre BPM key instruments",
            ]

            if song_names and song_names.strip():
                for song in song_names.split(","):
                    song = song.strip()
                    if song:
                        queries.append(f"{song} {artist_name} аккорды тональность")
                        queries.append(f"{song} {artist_name} song structure tempo")

            snippets = []
            try:
                with DDGS() as ddgs:
                    for q in queries:
                        for r in ddgs.text(q, max_results=3, region="wt-wt"):
                            title = r.get("title", "")
                            body = r.get("body", "")
                            if body:
                                snippets.append(f"**{title}**: {body}")
            except Exception as e:
                return json.dumps(
                    {"error": str(e), "artist": artist_name},
                    ensure_ascii=False,
                )

            if not snippets:
                return json.dumps(
                    {"artist": artist_name, "found": False,
                     "hint": "Try spelling the name differently or use the original language name."},
                    ensure_ascii=False,
                )

            combined = "\n\n".join(snippets[:10])
            if len(combined) > 2500:
                combined = combined[:2500] + "..."

            return json.dumps(
                {
                    "artist": artist_name,
                    "found": True,
                    "research": combined,
                    "instruction": (
                        "Based on the above, adapt your Renardo code: "
                        "match the genre (BPM, scale, rhythm pattern), "
                        "use appropriate instruments/sounds, "
                        "recreate the characteristic mood and energy."
                    ),
                },
                ensure_ascii=False,
                indent=2,
            )

        @function_tool
        async def get_current_time() -> str:
            """Получить текущее время и дату."""
            return await _call("get_current_time", {})

        @function_tool
        async def get_robot_status() -> str:
            """Получить статус робота: батарея, сенсоры, состояние навигации."""
            return await _call("get_robot_status", {})

        @function_tool
        async def get_battery_level() -> str:
            """Получить уровень заряда батареи."""
            return await _call("get_battery_level", {})

        @function_tool
        async def navigate_to_waypoint(waypoint: str) -> str:
            """Направить робота к сохранённой точке. БЛОКИРУЕТСЯ до прибытия. Сначала проверь list_waypoints()."""
            return await _call(
                "navigate_to_waypoint", {"waypoint": waypoint}, timeout=130.0
            )

        @function_tool
        async def navigate_to_coordinates(
            x: float, y: float, theta: float = 0.0
        ) -> str:
            """Навигация к произвольным координатам (x, y, theta) на карте. Используй после get_current_pose() для возврата."""
            return await _call(
                "navigate_to_coordinates",
                {"x": x, "y": y, "theta": theta},
                timeout=130.0,
            )

        @function_tool
        async def move_direction(direction: str, distance: float = 0.5) -> str:
            """Передвинуть робота. direction: 'вперёд', 'назад', 'налево', 'направо'.
            БЛОКИРУЕТСЯ до завершения движения — speak_text вызывать ТОЛЬКО после этого.
            """
            return await _call(
                "move_direction",
                {"direction": direction, "distance": distance},
                timeout=70.0,
            )

        @function_tool
        async def list_waypoints() -> str:
            """Получить список всех сохранённых точек на текущей карте."""
            return await _call("list_waypoints", {})

        @function_tool
        async def save_waypoint(name: str) -> str:
            """Сохранить текущую позицию робота как именованную точку (waypoint). ВСЕГДА используй когда пользователь называет место/локацию: 'это кухня', 'тут база', 'здесь зал', 'запомни это место', 'это его база'. НЕ memory_save!"""
            return await _call("save_waypoint", {"name": name})

        @function_tool
        async def delete_waypoint(name: str) -> str:
            """Удалить сохранённую точку по имени. Используй когда пользователь говорит 'удали зал', 'забудь кухню'."""
            return await _call("delete_waypoint", {"name": name})

        @function_tool
        async def clear_waypoints() -> str:
            """Удалить ВСЕ сохранённые точки на текущей карте."""
            return await _call("clear_waypoints", {})

        @function_tool
        async def get_current_pose() -> str:
            """Получить текущую позицию робота (x, y, theta) на карте. Используй перед миссиями для запоминания точки возврата."""
            return await _call("get_current_pose", {})

        @function_tool
        async def set_volume(action: str) -> str:
            """Изменить громкость голоса.
            action: 'louder' — громче, 'quieter' — тише, 'max' — максимум, 'normal' — норма.
            """
            return await _call("set_volume", {"action": action})

        @function_tool
        async def set_pitch(pitch: float) -> str:
            """Установить высоту голоса 0.5-2.0."""
            return await _call("set_pitch", {"pitch": pitch})

        @function_tool
        async def execute_music_code(code: str, pattern_name: str = "p1") -> str:
            """Запустить музыкальный код на синтезаторе Renardo/SuperCollider.
            Использовать ВСЕГДА когда пользователь просит сыграть мелодию, музыку, ноты.
            Пример: execute_music_code("p1 >> pluck([0,2,4,7], dur=0.5, amp=0.8)", pattern_name="p1")
            """
            return await _call(
                "execute_music_code",
                {"code": code, "pattern_name": pattern_name},
                timeout=15.0,
            )

        @function_tool
        async def stop_music(pattern_name: str = "") -> str:
            """Остановить музыку. pattern_name="" — остановить всё, иначе конкретный паттерн."""
            params = {"pattern_name": pattern_name} if pattern_name else {}
            return await _call("stop_music", params)

        @function_tool
        async def set_vibe_preset(preset: str) -> str:
            """Установить музыкальный вайб-пресет перед игрой мелодии.
            Доступные пресеты: chill, energetic, ambient, jazz, dark."""
            return await _call("set_vibe_preset", {"preset_name": preset})

        @function_tool
        async def get_music_state() -> str:
            """Получить текущее состояние музыкального синтезатора: что играет, темп, вайб."""
            return await _call("get_music_state", {})

        @function_tool
        async def set_dj_mode(
            enabled: bool, next_transition_sec: int = 0, theme: str = ""
        ) -> str:
            """Включить или выключить автономный DJ режим.

            enabled=True: после запуска музыки вызови set_dj_mode, чтобы робот сам делал переходы.
            enabled=False: выключить DJ режим. theme передавай только при первом запуске
            или при полной смене темы вечеринки.
            """
            params: dict = {"enabled": enabled}
            if next_transition_sec:
                params["next_transition_sec"] = next_transition_sec
            if theme:
                params["theme"] = theme
            return await _call("set_dj_mode", params)

        @function_tool
        async def list_tracks(tag: str = "", min_rating: int = 0) -> str:
            """Показать список треков в медиатеке робота.
            ВСЕГДА вызывай этот инструмент когда пользователь спрашивает про сохранённые треки,
            мелодии или музыкальную библиотеку — НЕ отвечай по памяти!
            tag="" — показать все; примеры тегов: 'full_track', 'robot_authored', 'minor'.
            min_rating=0 — все треки; min_rating=4 — только хорошие."""
            params: dict = {}
            if tag:
                params["tag"] = tag
            if min_rating:
                params["min_rating"] = min_rating
            return await _call("list_tracks", params)

        @function_tool
        async def save_track(
            name: str,
            title: str = "",
            description: str = "",
            tags: str = "",
            rating: int = 0,
            notes: str = "",
        ) -> str:
            """Сохранить текущий или последний сыгранный трек в медиатеку.
            Используй когда пользователь говорит 'сохрани этот трек', 'запомни мелодию', 'сохрани'.
            name — уникальный slug (например: 'chill_dnb_v1').
            tags — строка через запятую, будет разбита в список."""
            params: dict = {"name": name}
            if title:
                params["title"] = title
            if description:
                params["description"] = description
            if tags:
                params["tags"] = [t.strip() for t in tags.split(",") if t.strip()]
            if rating:
                params["rating"] = rating
            if notes:
                params["notes"] = notes
            return await _call("save_track", params)

        @function_tool
        async def load_track(name: str) -> str:
            """Загрузить трек из медиатеки и воспроизвести его.
            Используй когда пользователь просит 'сыграй <название>', 'включи сохранённый трек'.
            Сначала вызови list_tracks() чтобы узнать точное имя."""
            return await _call("load_track", {"name": name})

        @function_tool
        async def delete_track(name: str) -> str:
            """Удалить трек из медиатеки. Необратимо.
            Используй когда пользователь говорит 'удали трек', 'забудь мелодию'."""
            return await _call("delete_track", {"name": name})

        return [
            speak_text,
            play_sound,
            play_animation,
            memory_context,
            memory_save,
            memory_search,
            faq_search,
            get_current_time,
            get_robot_status,
            get_battery_level,
            navigate_to_waypoint,
            navigate_to_coordinates,
            move_direction,
            list_waypoints,
            save_waypoint,
            delete_waypoint,
            clear_waypoints,
            get_current_pose,
            set_volume,
            set_pitch,
            search_samples,
            search_artist_style,
            execute_music_code,
            stop_music,
            set_vibe_preset,
            get_music_state,
            set_dj_mode,
            list_tracks,
            save_track,
            load_track,
            delete_track,
        ]

    def _make_output_tools(self) -> list:
        """Create speak_text / play_sound / play_animation tools for compositor mode.

        These tools stay as flat tools on the Compositor because they hold
        asyncio locks (_output_lock, _tts_events, _sound_done_event) that
        are tightly coupled to DialogueNode's lifecycle.
        """
        mcp = self._mcp
        lock = self._output_lock

        async def _call(tool_name: str, params: dict, timeout: float = 10.0) -> str:
            self._tools_called.append(tool_name)
            result = await asyncio.get_running_loop().run_in_executor(
                None,
                lambda: mcp.execute_tool_call_sync(tool_name, params, timeout=timeout),
            )
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        @function_tool
        async def speak_text(text: str, animation: str = "neutral") -> str:
            """Произнести текст голосом. ОБЯЗАТЕЛЬНЫЙ инструмент — весь ответ ТОЛЬКО через speak_text!
            НЕЛЬЗЯ просто напечатать текст — пользователь ничего не услышит без вызова этого инструмента.
            Возвращает TASK_COMPLETE. После ПОСЛЕДНЕГО speak_text верни строку 'done'.
            """
            if self._run_cancelled:
                return "CANCELLED"
            # Strip history marker prefix — LLM sometimes copies [выполнено через: ...] from history
            text = re.sub(
                r"^\[(?:выполнено через|executed via):[^\]]*\]\s*", "", text
            ).strip()
            # ── Dedup: skip if same text was spoken within last 5 seconds ──
            now = time.time()
            with self._recent_speak_lock:
                for ts, recent_text in self._recent_speak:
                    if recent_text == text and (now - ts) < 5.0:
                        self.get_logger().debug(
                            f"🔇 speak_text dedup: skipping duplicate ({text[:40]}...)"
                        )
                        return "TASK_COMPLETE"
                self._recent_speak.append((now, text))
            self._spoken_texts.append(text)
            async with lock:
                if self._run_cancelled:
                    return "CANCELLED"
                result_str = await _call(
                    "speak_text", {"text": text, "animation": animation}, timeout=60.0
                )
                try:
                    speech_id = (
                        json.loads(result_str).get("data", {}).get("speech_id", "")
                    )
                except Exception:
                    speech_id = ""
                if speech_id:
                    event = asyncio.Event()
                    with self._tts_events_lock:
                        self._tts_events[speech_id] = event
                    try:
                        await asyncio.wait_for(event.wait(), timeout=30.0)
                    except asyncio.TimeoutError:
                        self.get_logger().warning(
                            f"⚠️ speak_text TTS timeout for {speech_id[:8]}"
                        )
                    finally:
                        with self._tts_events_lock:
                            self._tts_events.pop(speech_id, None)
            return "TASK_COMPLETE"

        @function_tool
        async def play_sound(sound: str) -> str:
            """Воспроизвести звуковой эффект."""
            async with lock:
                done_event = asyncio.Event()
                with self._sound_event_lock:
                    self._sound_done_event = done_event
                result_str = await _call("play_sound", {"sound": sound})
                try:
                    duration = (
                        json.loads(result_str).get("data", {}).get("duration", 1.5)
                    )
                except Exception:
                    duration = 1.5
                try:
                    await asyncio.wait_for(
                        done_event.wait(), timeout=float(duration) + 2.0
                    )
                except asyncio.TimeoutError:
                    self.get_logger().warning(
                        f"⚠️ play_sound timeout waiting for 'ready': {sound}"
                    )
                finally:
                    with self._sound_event_lock:
                        self._sound_done_event = None
            return result_str

        @function_tool
        async def play_animation(animation: str, duration: float = 3.0) -> str:
            """Запустить LED анимацию на указанное время."""
            async with lock:
                return await _call(
                    "play_animation", {"animation": animation, "duration": duration}
                )

        @function_tool
        def save_dj_set_plan(plan: str) -> str:  # noqa: F811
            """Сохранить или обновить план DJ-сета.

            Вызвать при первом DJ-переходе чтобы зафиксировать план всего сета,
            и при любом запросе пользователя изменить программу вечеринки.

            Args:
                plan: Текстовое описание плана сета — треки, стили, порядок, особые пожелания.
                      Например: 'Трек 1: медленный C minor ambient 70bpm pads+strings.\nТрек 2: ...'
            """
            self._dj_set_plan = plan.strip()
            self.get_logger().info(
                f"🎧 DJ план сохранён ({len(self._dj_set_plan)} chars)"
            )
            return "✅ План сета сохранён."

        @function_tool
        def save_dj_persona(persona: str) -> str:  # noqa: F811
            """Сохранить имя/персонажа DJ-роба.

            Вызывать когда пользователь даёт роботу имя или образ диджея.
            Например: 'ты диджей Демогорган', 'твоё имя DJ Shadow', 'ты робо-шаман'.
            При смене ВСЕЙ темы мероприятия (корпоратив, хэллоуин, детский праздник и т.п.)
            дополнительно вызови save_dj_theme() с новой темой.

            Args:
                persona: Имя или краткое описание персонажа. Например: 'DJ Демогорган'.
            """
            self._dj_persona = persona.strip()
            self.get_logger().info(f"🎧 DJ персонаж: {self._dj_persona!r}")
            return f"✅ Принято — теперь я {self._dj_persona}!"

        @function_tool
        def save_dj_theme(theme: str) -> str:  # noqa: F811
            """Обновить тему/контекст вечеринки и немедленно начать новый сет.

            Вызывать когда пользователь меняет тип мероприятия или общую тему:
            'теперь мы на корпоративе', 'это детский праздник', 'хэллоуин-вечеринка'.

            ЧТО ДЕЛАЕТ: сбрасывает счётчик переходов, план сета, включает DJ режим
            если он был выключен, запускает первый переход через 5 секунд.
            ПОСЛЕ вызова — ОБЯЗАТЕЛЬНО немедленно запусти первый трек
            через handle_music (новый стиль + новое настроение под новую тему).

            Args:
                theme: Краткое описание новой темы/контекста. Например: 'корпоратив', 'хэллоуин'.
            """
            # Guard: не даём LLM сбросить активный сет!
            # Но РАЗРЕШАЕМ если DJ-режим выключен (пользователь сменил тему после остановки)
            if self._dj_mode_enabled and self._dj_transition_count > 0 and self._dj_set_plan:
                self.get_logger().warning(
                    f"🎧 save_dj_theme: ИГНОРИРУЕМ — сет уже идёт (переход #{self._dj_transition_count}). "
                    f"Сначала set_dj_mode(enabled=False), потом save_dj_theme()."
                )
                return (
                    "❌ Нельзя менять тему во время активного сета! "
                    "Сначала заверши текущий сет через set_dj_mode(enabled=False), "
                    "потом вызови save_dj_theme() с новой темой."
                )
            old = self._dj_theme
            self._dj_theme = theme.strip()
            self._dj_transition_count = 0
            self._dj_set_plan = ""
            # Включаем DJ режим если был выключён, запускаем первый тик через 5с
            self._dj_mode_enabled = True
            self._dj_next_transition_at = time.time() + 5.0
            self.get_logger().info(
                f"🎧 DJ тема: {old!r} → {self._dj_theme!r} (счётчик сброшен, таймер через 5с)"
            )
            return (
                f"✅ Тема обновлена: {self._dj_theme}. DJ режим включён. "
                "Немедленно запусти первый трек через handle_music!"
            )

        return [
            speak_text,
            play_sound,
            play_animation,
            save_dj_set_plan,
            save_dj_persona,
            save_dj_theme,
        ]

    def _build_skills(self, model) -> list:
        """Instantiate skill sub-agents and return them as FunctionTools for the Compositor.

        Each skill becomes a single tool that the Compositor can call with a
        natural-language task string.  The skill runs its own focused LLM loop
        and returns a plain string result.

        Args:
            model: Shared OpenAIChatCompletionsModel instance to pass to each skill.

        Returns:
            List of FunctionTool objects (one per skill that loaded successfully).
        """
        skill_tools = []

        # ── MusicSkill ─────────────────────────────────────────────────────
        try:
            music_prompt = self._load_prompt_file("skills/music_skill_prompt.txt")
            if not music_prompt:
                music_prompt = "Ты — музыкальный модуль РОББОКСА. Используй Renardo для создания музыки."
            skill = MusicSkill(
                adapter=self._mcp,
                model=model,
                prompt_template=music_prompt,
                agent_max_turns=10,
                max_tokens=2000,  # 500 was cutting off tool-call JSON mid-argument → 11 retries × 22s
                temperature=0.85,
                # MiMo only supports tool_choice="auto" (others silently downgraded to auto).
                # Thinking mode disabled via extra_body to ensure reliable tool calls.
                tool_choice="auto",
            )
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_music",
                    tool_description=(
                        "Музыкальный скилл: запрос на игру музыки, мелодии, вайба, "
                        "остановку музыки или изменение музыкального состояния. "
                        "ТАКЖЕ: все DJ-переходы и DJ-режим (execute_music_code, set_dj_mode)."
                    ),
                )
            )
            self.get_logger().info("✅ MusicSkill loaded")
        except Exception as exc:
            self.get_logger().error(f"❌ MusicSkill build failed: {exc}")

        # ── NavigationSkill ────────────────────────────────────────────────
        try:
            nav_prompt = self._load_prompt_file("skills/navigation_skill_prompt.txt")
            if not nav_prompt:
                nav_prompt = (
                    "Ты — модуль навигации РОББОКСА. Управляй движением робота."
                )
            skill = NavigationSkill(
                adapter=self._mcp,
                model=model,
                prompt=nav_prompt,
                name="NavigationSkill",
                max_tokens=1000,
                tool_choice="auto",
            )
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_navigation",
                    tool_description=(
                        "Навигационный скилл: переместить робота в именованную точку, "
                        "сохранить/удалить/список точек (вейпоинтов), "
                        "картографирование (маппинг), направление движения."
                    ),
                )
            )
            self.get_logger().info("✅ NavigationSkill loaded")
        except Exception as exc:
            self.get_logger().error(f"❌ NavigationSkill build failed: {exc}")

        # ── MemorySkill ────────────────────────────────────────────────────
        try:
            mem_prompt = self._load_prompt_file("skills/memory_skill_prompt.txt")
            if not mem_prompt:
                mem_prompt = (
                    "Ты — модуль памяти РОББОКСА. Управляй долгосрочной памятью."
                )
            skill = MemorySkill(
                adapter=self._mcp, model=model, prompt=mem_prompt, name="MemorySkill",
                tool_choice="auto",
            )
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_memory",
                    tool_description=(
                        "Модуль памяти: сохранить новую информацию или найти "
                        "ранее сохранённые факты в долгосрочной памяти."
                    ),
                )
            )
            self.get_logger().info("✅ MemorySkill loaded")
        except Exception as exc:
            self.get_logger().error(f"❌ MemorySkill build failed: {exc}")

        # ── StatusSkill ────────────────────────────────────────────────────
        try:
            status_prompt = self._load_prompt_file("skills/status_skill_prompt.txt")
            if not status_prompt:
                status_prompt = "Ты — модуль статуса РОББОКСА. Предоставляй информацию о состоянии робота."
            skill = StatusSkill(
                adapter=self._mcp, model=model, prompt=status_prompt, name="StatusSkill",
                tool_choice="auto",
            )
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_status",
                    tool_description=(
                        "Модуль статуса: батарея, текущее время, статус робота, "
                        "изменение громкости или высоты голоса."
                    ),
                )
            )
            self.get_logger().info("✅ StatusSkill loaded")
        except Exception as exc:
            self.get_logger().error(f"❌ StatusSkill build failed: {exc}")

        # ── FAQSkill ────────────────────────────────────────────────────────
        if self._faq_store and self._event_profile:
            try:
                event_id = self._event_profile.get(
                    "event_id"
                ) or self._slugify_event_id(
                    self._event_profile.get("name", "faq-event")
                )
                faq_prompt = self._load_prompt_file("skills/faq_skill_prompt.txt")
                if not faq_prompt:
                    faq_prompt = (
                        "Ты — FAQ модуль РОББОКСА. Ищи точные ответы по мероприятию."
                    )
                skill = FAQSkill(
                    store=self._faq_store,
                    event_id=event_id,
                    model=model,
                    prompt=self._render_faq_skill_prompt(faq_prompt),
                    name="FAQSkill",
                    temperature=0.2,
                    max_tokens=900,
                    tool_choice="auto",
                )
                skill_tools.append(
                    skill.as_tool(
                        tool_name="handle_faq",
                        tool_description=(
                            "FAQ мероприятия: поиск точных ответов по вопросам о событии, поступлении, "
                            "локациях, расписании и других организационных деталях."
                        ),
                    )
                )
                self.get_logger().info("✅ FAQSkill loaded")
            except Exception as exc:
                self.get_logger().error(f"❌ FAQSkill build failed: {exc}")

        return skill_tools

    def _log_config(self) -> None:
        self.get_logger().info(f"  Provider : {self._provider}")
        self.get_logger().info(f"  Model    : {self._resolve_model()}")
        self.get_logger().info(f"  Wake     : {self.dialogue_manager.wake_words}")
        self.get_logger().info(
            f"  History  : {self._max_turns} turns (SDK to_input_list format)"
        )
        self.get_logger().info(
            f"  Timeout  : {self.dialogue_manager.dialogue_timeout}s (dialogue) / {self._llm_timeout * 3:.0f}s (LLM wait_for)"
        )
        self.get_logger().info(f"  VerboseLLM: {self._verbose_llm}")

    # ────────────────────────────────────────────────────────────────
    # ROS2 callbacks
    # ────────────────────────────────────────────────────────────────

    def _on_vad(self, msg: Bool) -> None:
        """Track VAD speech state (flag only, no barge-in).

        Barge-in is handled exclusively by _on_stt() which requires
        a confirmed wake word before cancelling any active run.
        This prevents background noise (TV, radio) from interrupting
        the agent mid-response.
        """
        is_speech = msg.data
        if is_speech and not self._vad_speech_detected:
            self._vad_speech_detected = True
            self.get_logger().debug("🎤 VAD: speech start")
        elif not is_speech and self._vad_speech_detected:
            self._vad_speech_detected = False

    def _on_stt(self, msg: String) -> None:
        """Handle STT result — gate on wake word / silence, then start agent."""
        text = msg.data.strip()
        if not text:
            return

        state = self.dialogue_manager.state
        text_lower = text.lower()

        # ── SILENCED: only listen for unsilence command ──────────────
        if state == DialogueState.SILENCED:
            if self.dialogue_manager.is_unsilence_command(text_lower):
                self.dialogue_manager.transition_state(DialogueState.LISTENING)
                self._publish_state()
            return

        # ── IDLE: require wake word ──────────────────────────────────
        if state == DialogueState.IDLE:
            if not self.dialogue_manager.has_wake_word(text_lower):
                return
            self.dialogue_manager.transition_state(DialogueState.LISTENING)
            self._publish_state()

        # ── LISTENING / DIALOGUE: require wake word for every message ─
        if not self.dialogue_manager.has_wake_word(text_lower):
            self.get_logger().debug(f"🔇 Ignored (no wake word): {text[:60]}")
            return

        clean = self.dialogue_manager.remove_wake_word(text_lower) or text

        if self.dialogue_manager.is_silence_command(text_lower):
            self._handle_silence()
            return

        # Cancel any in-progress run before starting a new one
        self._cancel_run("new STT input")

        # ── Immediate thinking sound ─────────────────────────────────
        # Play confirmation immediately — before LLM even starts
        _snd = String()
        _snd.data = "thinking"
        self._sound_trigger_pub.publish(_snd)

        self.dialogue_manager.transition_state(DialogueState.DIALOGUE)
        self._publish_state()

        # Если DJ активен — инжектируем контекст чтобы агент мог скорректировать план
        if self._dj_mode_enabled:
            _dj_name = self._dj_persona if self._dj_persona else "ДиДжей РОббокс"
            persona_line = f'Твой DJ-образ: "{_dj_name}". '
            plan_line = (
                f"Текущий план сета:\n{self._dj_set_plan}\n"
                if self._dj_set_plan
                else ""
            )
            dj_ctx = (
                f"[🎧 DJ-РЕЖИМ АКТИВЕН, переход #{self._dj_transition_count}. "
                f'Тема: "{self._dj_theme}". '
                f"{persona_line}"
                f"{plan_line}"
                "Если пользователь даёт тебе имя или образ диджея — вызови save_dj_persona(). "
                "Если пользователь МЕНЯЕТ ТЕМУ или тип мероприятия (корпоратив, хэллоуин, другая "
                "вечеринка и т.п.) — ОБЯЗАТЕЛЬНО: "
                "1) вызови save_dj_theme(новая_тема) "
                "2) сразу же запусти новый трек через handle_music (музыка должна смениться НЕМЕДЛЕННО, не ждать таймера). "
                "Образы вроде 'циркач', 'бродячий музыкант', 'уличный артист' сами по себе НЕ означают смену темы вечеринки. "
                "Это стиль исполнения или локальная творческая правка текущего трека, а не save_dj_theme(). "
                "Если просит изменить музыку или план без смены темы — различай ЛОКАЛЬНОЕ и ГЛОБАЛЬНОЕ изменение. "
                "ЛОКАЛЬНОЕ (например: 'добавь вокал', 'сделай бас потише', 'добавь балалайку') = меняй только ТЕКУЩИЙ трек через handle_music, "
                "НЕ переписывай весь план сета. "
                "ГЛОБАЛЬНОЕ (например: 'теперь весь сет с вокалом', 'измени всю программу', 'сделай все треки хаусом') = save_dj_set_plan() + handle_music. "
                "Иначе просто ответь голосом.] "
            )
            clean = dj_ctx + clean

        asyncio.run_coroutine_threadsafe(self._agent_run(clean), self._loop)

    # ────────────────────────────────────────────────────────────────
    # Agent execution
    # ────────────────────────────────────────────────────────────────

    async def _run_agent_with_retry(
        self,
        input_list: list,
        max_retries: int = 2,
        base_delay: float = 2.0,
    ):
        """Run Agent with classification-aware retry + fallback model on timeout.

        Error taxonomy (v2.x):
        - APITimeoutError  → fallback to lighter model, no retry (timeout = model overload)
        - RateLimitError   → honor retry-after header, retry
        - APIStatusError   → retry on 5xx only
        - APIConnectionError → WiFi flakiness, exponential backoff
        """
        last_exc = None
        fallback_used = False

        for attempt in range(1 + max_retries):
            try:
                return await Runner.run(
                    self._agent, input_list, max_turns=self._agent_max_turns
                )
            except APITimeoutError as exc:
                fb_model = self._resolve_fallback_model()
                current_model = self._resolve_model()
                self.get_logger().error(
                    f"⏰ LLM timeout after {self._llm_timeout}s "
                    f"(model={current_model}, request_id={exc.request_id}, "
                    f"cause={exc.__cause__})"
                )
                if (
                    not fallback_used
                    and fb_model
                    and fb_model != current_model
                    and self.get_parameter("enable_fallback").value
                ):
                    self.get_logger().warning(
                        f"🔄 Switching to fallback model: {fb_model}"
                    )
                    self._build_agent(model_override=fb_model)
                    fallback_used = True
                    continue  # retry with fallback model
                raise

            except RateLimitError as exc:
                retry_after = exc.response.headers.get("retry-after", "5")
                self.get_logger().warn(
                    f"🔄 Rate limited (429), retry-after={retry_after}s "
                    f"(request_id={exc.request_id})"
                )
                # Honor retry-after, clamp to 2..30s
                try:
                    delay = max(2.0, min(float(retry_after), 30.0))
                except (ValueError, TypeError):
                    delay = 5.0
                await asyncio.sleep(delay)
                continue

            except APIStatusError as exc:
                self.get_logger().error(
                    f"🌩️ API error {exc.status_code}: {str(exc.response.text)[:200]} "
                    f"(request_id={exc.request_id})"
                )
                if exc.status_code >= 500 and attempt < max_retries:
                    delay = base_delay * (2**attempt)
                    await asyncio.sleep(delay)
                    continue
                raise

            except APIConnectionError as exc:
                last_exc = exc
                if attempt < max_retries:
                    delay = base_delay * (2**attempt)
                    self.get_logger().warning(
                        f"⚠️ APIConnectionError (attempt {attempt + 1}/{1 + max_retries}), "
                        f"retrying in {delay:.0f}s: {exc}"
                    )
                    await asyncio.sleep(delay)
                else:
                    self.get_logger().error(
                        f"❌ APIConnectionError after {1 + max_retries} attempts: {exc}"
                    )
                    raise
        raise last_exc  # unreachable, but keeps type checker happy

    async def _agent_run(self, user_input: str) -> None:
        """Run the OpenAI Agents SDK loop for a single user turn."""
        with self._task_lock:
            self._run_task = asyncio.current_task()
        self._run_cancelled = False
        self._spoken_texts = []
        with self._recent_speak_lock:
            self._recent_speak.clear()
        self._tools_called = []
        self.get_logger().info(f"🤔 User: {user_input[:120]}")
        if self._voice_memory is not None:
            try:
                self._voice_memory.save_turn("user", user_input)
            except Exception as exc:
                self.get_logger().warning(f"⚠️ memory save_turn(user) failed: {exc}")

        try:
            faq_prefetch_context = self._build_event_faq_prefetch_context(user_input)
            with self._conv_lock:
                # Build input: stored history (SDK to_input_list format) + current user message.
                # History contains real function_call / function_call_output items, so the
                # LLM sees actual tool invocations and cannot pattern-complete them as text.
                # This eliminates the DeepSeek multi-turn FC pattern-completion bug.
                input_list = list(self._conversation)
                if faq_prefetch_context:
                    input_list.append(
                        {"role": "system", "content": faq_prefetch_context}
                    )
                input_list.append({"role": "user", "content": user_input})

            if self._agent is None:
                self.get_logger().error("❌ Agent not initialised")
                return

            if self._verbose_llm:
                self.get_logger().info(
                    f"📥 LLM INPUT ({len(input_list)} messages):\n"
                    + json.dumps(input_list, ensure_ascii=False, indent=2)
                )

            result = await asyncio.wait_for(
                self._run_agent_with_retry(input_list),
                timeout=self._llm_timeout * 3,
            )

            spoken = (
                " ".join(self._spoken_texts)
                if self._spoken_texts
                else (result.final_output or "")
            )
            # Collect tool names used — for logging and auto-speak fallback detection.
            tool_names_used = set()
            try:
                for item in result.new_items:
                    if isinstance(item, ToolCallItem) and hasattr(
                        item.raw_item, "name"
                    ):
                        tool_names_used.add(item.raw_item.name)
            except Exception:
                pass
            tool_names_used.update(self._tools_called)

            # Auto-speak fallback: if LLM returned text without calling
            # speak_text, speak the response directly so the robot is never silent.
            # BUT: suppress for DJ planning tools — they produce internal state,
            # not user-facing speech. LLM should call speak_text explicitly.
            _DJ_PLANNING_TOOLS = {"save_dj_set_plan", "save_dj_persona", "save_dj_theme", "set_dj_mode"}
            _is_dj_planning = bool(tool_names_used & _DJ_PLANNING_TOOLS)
            if not self._spoken_texts and spoken and not _is_dj_planning:
                clean_spoken = re.sub(
                    r"^\[выполнено через:[^\]]*\]\s*", "", spoken
                ).strip()
                # Expand done filter to catch variants MiMo produces
                DONE_VARIANTS = {"done", "done!", "done.", "done,", "выполнено", "выполнено!", "готово"}
                if clean_spoken and clean_spoken.lower().rstrip("!.,").strip() not in DONE_VARIANTS:
                    self.get_logger().warning(
                        f"⚠️ Auto-speak fallback (LLM skipped speak_text): {clean_spoken[:80]}"
                    )
                    self._speak_direct(clean_spoken)

            # Empty response fallback: MiMo sometimes returns nothing (no text, no
            # tool_calls).  Speak a short prompt so the user isn't left confused.
            if not self._spoken_texts and not spoken:
                # Check if this was a DJ transition call (music tool was used in recent history)
                is_dj_context = any("handle_music" in str(t) or "execute_music_code" in str(t)
                                    for t in (self._last_tool_calls or []))
                if is_dj_context:
                    self.get_logger().warning(
                        "⚠️ Empty MiMo response during DJ context — retrying once"
                    )
                    try:
                        retry_result = await Runner.run(
                            self._agent, input_list, max_turns=self._agent_max_turns
                        )
                        # Extract spoken from retry result
                        retry_spoken = ""
                        for item in retry_result.new_items:
                            if hasattr(item, 'raw_item') and hasattr(item.raw_item, 'content'):
                                if item.raw_item.content:
                                    retry_spoken = item.raw_item.content
                        if retry_spoken:
                            spoken = retry_spoken
                        else:
                            self.get_logger().warning("DJ retry also empty — continuing current track")
                    except Exception as retry_exc:
                        self.get_logger().error(f"DJ retry failed: {retry_exc}")
                else:
                    self.get_logger().warning(
                        "⚠️ LLM returned empty response — speaking fallback"
                    )
                    self._speak_direct("Что-то я задумался, повтори пожалуйста")

            if self._verbose_llm:
                self.get_logger().info(f"📤 LLM OUTPUT:\n{spoken}")
            else:
                self.get_logger().info(
                    f"✅ Agent done. Tools: {sorted(tool_names_used)}. Response: {spoken[:80]}"
                )

            if self._voice_memory is not None and spoken:
                try:
                    self._voice_memory.save_turn("assistant", spoken)
                except Exception as exc:
                    self.get_logger().warning(
                        f"⚠️ memory save_turn(assistant) failed: {exc}"
                    )

            # In FAQ event mode history is intentionally NOT stored between turns.
            # Each question is answered from scratch using only the FAQ prefetch context.
            # This keeps every LLM call at a minimal, fixed context size regardless of
            # how long the session has been running, eliminating the latency growth we
            # observed: 10 msgs → ~9s, 34 msgs → ~44s, 68 msgs → ~57s.
            if self._event_profile:
                with self._conv_lock:
                    self._conversation = []
            else:
                # Store full SDK transcript for next turn.
                # result.to_input_list() contains real function_call / function_call_output items.
                # On the next turn the LLM sees actual tool invocations in history — not text
                # summaries it could pattern-complete. Robot naturally remembers what it did.
                with self._conv_lock:
                    self._conversation = self._trim_history(result.to_input_list())

        except asyncio.CancelledError:
            self.get_logger().info("🛑 Agent run cancelled (barge-in / new input)")
            # Do NOT update history — partial turn discarded

        except MaxTurnsExceeded:
            self.get_logger().error(
                f"❌ Agent exceeded max tool-call turns ({self._agent_max_turns}). "
                "Consider increasing agent_max_turns param."
            )

        except asyncio.TimeoutError:
            self.get_logger().error(
                f"❌ Agent timed out ({self._llm_timeout * 3:.0f}s)"
            )
            self._speak_direct("Извините, не получилось ответить вовремя.")

        except Exception as exc:
            self.get_logger().error(f"❌ Agent error: {exc}")
            if "invalid_request_error" in str(exc) or "tool" in str(exc).lower():
                self.get_logger().warning(
                    "🧹 Clearing conversation history due to malformed state"
                )
                with self._conv_lock:
                    self._conversation = []

        finally:
            with self._task_lock:
                self._run_task = None
            if self.dialogue_manager.state == DialogueState.DIALOGUE:
                self.dialogue_manager.transition_state(DialogueState.LISTENING)
                self._publish_state()

    def _on_tts_finished_dlg(self, msg: String) -> None:
        """Release speak_text awaiter when TTS finishes playing."""
        try:
            data = json.loads(msg.data)
            speech_id = data.get("speech_id", "")
        except Exception:
            speech_id = msg.data.strip()
        if not speech_id:
            return
        with self._tts_events_lock:
            event = self._tts_events.get(speech_id)
        if event:
            self._loop.call_soon_threadsafe(event.set)

    def _on_sound_state(self, msg: String) -> None:
        """Release play_sound awaiter when sound_node publishes 'ready'."""
        if msg.data != "ready":
            return
        with self._sound_event_lock:
            event = self._sound_done_event
        if event is not None:
            self._loop.call_soon_threadsafe(event.set)

    def _cancel_run(self, reason: str) -> None:
        self._spoken_texts = []
        with self._recent_speak_lock:
            self._recent_speak.clear()
        self._run_cancelled = True
        with self._task_lock:
            task = self._run_task
        if task and not task.done():
            self.get_logger().info(f"🛑 Cancel: {reason}")
            self._loop.call_soon_threadsafe(task.cancel)
        # Always flush TTS queue on cancel — even if no task was running,
        # the TTS node may still have queued phrases from a previous run.
        stop_msg = String()
        stop_msg.data = "STOP"
        self._tts_control_pub.publish(stop_msg)
        # Release any pending TTS events so speak_text doesn't hang
        with self._tts_events_lock:
            for event in self._tts_events.values():
                self._loop.call_soon_threadsafe(event.set)
            self._tts_events.clear()
        # Release pending sound-done event so play_sound doesn't hang
        with self._sound_event_lock:
            if self._sound_done_event is not None:
                self._loop.call_soon_threadsafe(self._sound_done_event.set)
                self._sound_done_event = None

    def _trim_history(self, items: list) -> list:
        """Keep the last `max_turns` complete turns in SDK input-list format.

        A turn starts at each user message. We keep the last max_turns turns
        intact, preserving all function_call / function_call_output pairs within.
        """
        user_positions = [
            i
            for i, item in enumerate(items)
            if isinstance(item, dict) and item.get("role") == "user"
        ]
        if len(user_positions) <= self._max_turns:
            trimmed = list(items)
        else:
            cutoff_idx = user_positions[-self._max_turns]
            trimmed = list(items[cutoff_idx:])
        return self._truncate_history_outputs(trimmed)

    def _truncate_history_outputs(self, items: list, max_len: int = 200) -> list:
        """Truncate long function_call_output values to prevent context bloat.

        Sub-agents like handle_music can return 500+ char descriptions.
        Storing them verbatim in history balloons every subsequent DeepSeek
        request, causing timeouts on turns like "develop the melody".
        We keep the first 200 chars — enough for the LLM to understand what
        happened without slowing down future calls.
        """
        result = []
        for item in items:
            if (
                isinstance(item, dict)
                and item.get("type") == "function_call_output"
                and isinstance(item.get("output"), str)
                and len(item["output"]) > max_len
            ):
                item = dict(item)  # shallow copy — don't mutate the original
                item["output"] = item["output"][:max_len] + "…[truncated]"
            result.append(item)
        return result

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────

    @staticmethod
    def _split_into_chunks(text: str, max_len: int = 200) -> list:
        """Split text into sentence chunks ≤ max_len to avoid Yandex TTS 'Too long text'."""
        raw = re.split(r"(?<=[.!?;])\s+", text.strip())
        chunks: list = []
        buf = ""
        for part in raw:
            part = part.strip()
            if not part:
                continue
            candidate = (buf + " " + part).strip() if buf else part
            if len(candidate) <= max_len:
                buf = candidate
            else:
                if buf:
                    chunks.append(buf)
                if len(part) > max_len:
                    sub_parts = re.split(r"(?<=,)\s+", part)
                    sub_buf = ""
                    for sp in sub_parts:
                        sub_c = (sub_buf + " " + sp).strip() if sub_buf else sp
                        if len(sub_c) <= max_len:
                            sub_buf = sub_c
                        else:
                            if sub_buf:
                                chunks.append(sub_buf)
                            sub_buf = sp
                    buf = sub_buf
                else:
                    buf = part
        if buf:
            chunks.append(buf)
        return [c for c in chunks if c.strip()] or [text]

    def _speak_direct(self, text: str) -> None:
        """Publish a response directly (no LLM) — errors, silence confirmations.

        Uses sentence splitting so each TTS request stays under Yandex's 250-char
        SSML limit.  Without this the fallback path caused 'Too long text' errors
        and forced slow Silero synthesis.
        """
        import uuid

        chunks = self._split_into_chunks(text, max_len=200)
        for chunk in chunks:
            msg = String()
            msg.data = json.dumps(
                {
                    "ssml": f"<speak>{chunk}</speak>",
                    "speech_id": str(uuid.uuid4()),
                    "emotion": "neutral",
                },
                ensure_ascii=False,
            )
            self._response_pub.publish(msg)

    def _handle_silence(self) -> None:
        self._cancel_run("silence command")
        self.dialogue_manager.enable_silence(duration=300.0)
        self._publish_state()
        self._speak_direct("Хорошо, молчу.")

    def _on_timeout_check(self) -> None:
        """Every-5s timer: transition LISTENING→IDLE after inactivity."""
        with self._task_lock:
            running = self._run_task is not None and not self._run_task.done()
        if running:
            return  # Never timeout while agent is working (fixes the bug we had!)
        if self.dialogue_manager.check_timeout():
            self.get_logger().info(
                "⏰ Dialogue timeout → IDLE (history preserved, sliding window)"
            )
            self._publish_state()

    # ── DJ mode ─────────────────────────────────────────────────────

    def _build_dj_prompt(self, n: int) -> str:
        """Build DJ auto-transition prompt.

        Transition #1: agent plans the full set via save_dj_set_plan(), then plays.
        Transitions #2+: agent follows its own saved plan.
        """
        theme_line = f'Тема вечеринки: "{self._dj_theme}". ' if self._dj_theme else ""
        dj_name = self._dj_persona if self._dj_persona else "ДиДжей РОббокс"
        persona_line = f'Твой DJ-образ: "{dj_name}". '

        if n == 1:
            # Первый переход — агент сам составляет план исходя из темы
            return (
                "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] "
                f"Ты {dj_name} — первый в мире робот-диджей. "
                f"{theme_line}"
                f"{persona_line}"
                "ПОРЯДОК ДЕЙСТВИЙ (строго по шагам): "
                "🚫 НЕ вызывай handle_music пока не выполнил шаги 0 и 0.1! "
                "БЕЗ research — плохая музыка. СНАЧАЛА собери информацию, ПОТОМ играй. "
                "ШАГ 0 — ОБЯЗАТЕЛЬНЫЙ: вызови search_artist_style(artist_name) для каждого упомянутого артиста/группы. "
                "   Это даст тебе: жанр, стиль, BPM, тональность, характерные инструменты. "
                "   Если артист не упомянут явно — всё равно вызови для ключевого слова из темы. "
                "ШАГ 0.1 — ОБЯЗАТЕЛЬНЫЙ: вызови list_tracks(tag=...) для 2-3 тегов из темы вечеринки. "
                f"   Например: list_tracks(tag='stranger_things'), list_tracks(tag='synthwave'), list_tracks(tag='dark'). "
                "   Попробуй 2-3 разных тега, чтобы найти релевантные треки. "
                "   Прочитай названия и теги, найди 3-5 треков — используй их как вдохновение! "
                "   НЕ вызывай load_track — код трека тебе не нужен, только названия и теги. "
                "   ❌ НЕ вызывай list_tracks() без параметров — это засоряет контекст! "
                "1) Сохрани план сета (5-8 треков) через save_dj_set_plan(). "
                "   В плане каждый трек: стиль, BPM, тональность, синты, атмосфера — всё в духе ТЕМЫ вечеринки. "
                "   Дуга: вход → нарастание → пик → спуск. "
                f"2) Сыграй трек #1 через handle_music. В task-строке ОБЯЗАТЕЛЬНО укажи тему вечеринки + описание трека. "
                f"   Пример task: '{self._dj_theme} — трек 1: [название], [стиль], [BPM] BPM, [тональность], [атмосфера]'. "
                f"   Внутри handle_music — только музыка и DJ control; речь НЕ поручай MusicSkill. "
                f"3) Одна короткая фраза через speak_text() (до 15 слов) — поздоровайся и объяви тему. НЕ рассказывай план сета! "
                "⚠️ ТЕХНИЧЕСКИЕ ПРАВИЛА для музыки: "
                "1) ПЕРВАЯ строка кода = Clock.clear(). "
                "2) Максимум 6 паттернов: d1-d3 + p1-p3. sus ≤ 8. "
                "🎛️ ПОСЛЕ КАЖДОГО ТРЕКА (критично!): вызови set_dj_mode(enabled=True, next_transition_sec=X) ОТДЕЛЬНЫМ tool call! "
                "   X = длина трека в секундах (30-90). БЕЗ этого вызова — тишина между треками! "
                "⚠️ В set_dj_mode НЕ передавай параметр theme! "
                "4) Для drum play() используй только безопасные буквы X/o/- или буквы, явно найденные через search_samples; НЕ выдумывай A/B/Q и другие sample folders. "
                "🚫 АНТИ-ЭСКАЛАЦИЯ: барабаны amp≤0.3, синты amp≤0.7, dur≥0.5, degree ≤ 5 нот. "
                "❌ Hi-hat: НЕ '--------' dur=0.5 — используй '--.-' dur=1 (иначе цоканье)! "
                "❌ НЕ используй pianovel и piano — оба цокают (MdaPiano физмодель). Вместо них rhpiano (FM Rhodes, чистый звук). "
                "✅ Доступные синты: "
                "Melody: rhpiano, blip, arpy, epiano, karp, sitar, marimba, bell, cs80lead, supersawlead, imperialbrass, strangerarp. "
                "Bass: dub, wobblebass, fuzz, dirt, subbass, moogbass, bass, retrobass (oct=2..3). "
                "Pads: strings, pads, ambi, space, sinepad, warmpad, marchstrings, strangerpulsepad. "
                "Brass: brass, flute, soprano, eoboe, organ, strangerbrass. "
                "Glitch: rave, donk, varsaw, pulse, tb303. "
                "⚠️ Для Stranger Things/stil: strangerpulsepad + retrobass + strangerarp + strangerbrass."
            )

        # Переходы #2+ — агент идёт по своему плану
        # Показываем ТОЛЬКО текущий трек, а не весь план — иначе LLM болтает обо всех
        plan_track_count = 0
        current_track_line = ""
        if self._dj_set_plan:
            plan_track_count = self._dj_set_plan.count("Трек ") or self._dj_set_plan.count("ТРЕК ")
            # Извлекаем строку текущего трека из плана
            m = re.search(
                rf"(?:Трек|ТРЕК)\s*{n}\s*[:\.].*",
                self._dj_set_plan,
                re.IGNORECASE,
            )
            if m:
                current_track_line = m.group(0).strip()
        plan_block = (
            f"Текущий трек: {current_track_line}\n\n"
            if current_track_line
            else "(План сета не сохранён — импровизируй в духе темы вечеринки.)\n\n"
        )

        return (
            f"[DJ_AUTO переход #{n}] "
            f"Ты {dj_name} — первый в мире робот-диджей. "
            f"{theme_line}"
            f"{persona_line}"
            f"{plan_block}"
            f"Сыграй трек #{n} через handle_music. "
            f"{'(Это последний трек по плану — после него коротко попрощайся с аудиторией и вызови set_dj_mode(enabled=False) чтобы завершить сет!) ' if plan_track_count > 0 and n == plan_track_count else ''}"
            "Изредка (раз в 3-4 перехода) короткая MC-фраза (до 12 слов) через speak_text(). Не говори на каждом переходе! "
            "⚠️ Говори ТОЛЬКО про текущий трек — НЕ упоминай другие треки плана, не читай весь список! "
            f"⚠️ При вызове handle_music в task-строке ОБЯЗАТЕЛЬНО укази тему вечеринки + описание трека. "
            f"   Пример task: '{self._dj_theme} — трек {n}: [название], [стиль], [BPM] BPM, [тональность], [атмосфера]'. "
            "   Внутри handle_music — только музыка и DJ control; речь делай отдельным speak_text(). "
            "⚠️ ТЕХНИЧЕСКИЕ ПРАВИЛА: "
            "1) ПЕРВАЯ строка кода = Clock.clear(). "
            "2) Максимум 6 паттернов: d1-d3 + p1-p3. sus ≤ 8. "
            "🎛️ ПОСЛЕ КАЖДОГО ТРЕКА (критично!): вызови set_dj_mode(enabled=True, next_transition_sec=X) ОТДЕЛЬНЫМ tool call! "
            "   X = длина текущего трека в секундах (30-90). БЕЗ этого вызова — тишина между треками! "
            "⚠️ В set_dj_mode НЕ передавай параметр theme! "
            "4) Для drum play() используй только безопасные буквы X/o/- или буквы, явно найденные через search_samples; НЕ выдумывай A/B/Q и другие sample folders. "
            "🚫 АНТИ-ЭСКАЛАЦИЯ: барабаны amp≤0.3, синты amp≤0.7, dur≥0.5, degree ≤ 5 нот. "
            "❌ После КАЖДОГО трека вызови set_dj_mode(enabled=True, next_transition_sec=X) для следующего перехода! "
            "❌ Если это ПОСЛЕДНИЙ трек по плану — попрощайся и вызови set_dj_mode(enabled=False) для завершения сета! "
            "❌ Hi-hat: НЕ '--------' dur=0.5 — используй '--.-' dur=1 (иначе цоканье)! "
            "❌ НЕ используй pianovel и piano — оба цокают (MdaPiano физмодель). Вместо них rhpiano (FM Rhodes, чистый звук). "
            "✅ Доступные синты: "
            "Melody: rhpiano, blip, arpy, epiano, karp, sitar, marimba, bell, cs80lead, supersawlead, imperialbrass, strangerarp. "
            "Bass: dub, wobblebass, fuzz, dirt, subbass, moogbass, bass, retrobass (oct=2..3). "
            "Pads: strings, pads, ambi, space, sinepad, warmpad, marchstrings, strangerpulsepad. "
            "Brass: brass, flute, soprano, eoboe, organ, strangerbrass. "
            "Glitch: rave, donk, varsaw, pulse, tb303. "
        )

    def _on_dj_mode_msg(self, msg: String) -> None:
        """Обработать команду включения/выключения DJ-режима от MCP-инструмента."""
        try:
            data = json.loads(msg.data)
            enabled = bool(data.get("enabled", False))
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warning(
                f"⚠️ DJ mode: некорректное сообщение: {msg.data!r}"
            )
            return

        self._dj_mode_enabled = enabled
        if enabled:
            # Сохраняем тему вечеринки только при первом включении.
            # Авто-переходы НЕ должны перезаписывать тему — она устанавливается один раз!
            theme = data.get("theme")
            if theme and isinstance(theme, str) and theme.strip():
                if self._dj_transition_count == 0 or not self._dj_theme:
                    self._dj_theme = theme.strip()
                    self.get_logger().info(f"🎧 DJ Mode тема: {self._dj_theme!r}")
                else:
                    self.get_logger().debug(
                        f"🎧 DJ Mode: тема уже установлена ({self._dj_theme!r}), игнорируем '{theme}'"
                    )
            # Интервал задаётся LLM через next_transition_sec; фолбэк 60с для первого включения
            next_sec = data.get("next_transition_sec")
            if next_sec:
                delay = float(max(15, min(300, int(next_sec))))
            else:
                delay = 60.0  # дефолт при первом включении без явного интервала
            self._dj_next_transition_at = time.time() + delay
            self.get_logger().info(f"🎧 DJ Mode: следующий переход через {delay:.0f}с")
            if self._dj_transition_count == 0:
                self.get_logger().info("🎧 DJ Mode ENABLED")
        else:
            # Выключаем DJ — сбрасываем ВСЕ состояние
            self._dj_next_transition_at = 0.0
            self._dj_transition_count = 0
            self._dj_theme = ""  # сбрасываем тему при выключении
            self._dj_set_plan = ""  # сбрасываем план при выключении
            self._dj_persona = ""  # сбрасываем персонажа при выключении
            # Auto-stop music after 45s (let last track fade out)
            self._dj_stop_at = time.time() + 45.0
            self.get_logger().info("🎧 DJ Mode DISABLED — автостоп музыки через 45с")

    def _on_dj_tick_check(self) -> None:
        """Every-5s timer: fire autonomous DJ transition when it's time."""
        if not self._dj_mode_enabled:
            return

        now = time.time()
        if now < self._dj_next_transition_at:
            return

        # Не прерывать активный диалог — откладываем на 15 секунд
        if self.dialogue_manager.state in (
            DialogueState.DIALOGUE,
            DialogueState.SILENCED,
        ):
            self._dj_next_transition_at = now + 15.0
            return

        with self._task_lock:
            running = self._run_task is not None and not self._run_task.done()
        if running:
            self._dj_next_transition_at = now + 15.0
            return

        # Жёсткий лимит безопасности: 50 переходов (защита от бесконечного цикла)
        next_n = self._dj_transition_count + 1
        if next_n > 50:
            self.get_logger().warning(
                f"🛑 DJ авто-стоп: переход #{next_n} > 50 (безопасный лимит). Отключаем DJ."
            )
            self._dj_mode_enabled = False
            self._dj_next_transition_at = 0.0
            self._dj_transition_count = 0
            self._dj_theme = ""
            self._dj_set_plan = ""
            self._dj_persona = ""
            return

        # Auto-stop music if DJ was disabled and cooldown expired
        if self._dj_stop_at and now >= self._dj_stop_at:
            self._dj_stop_at = 0.0
            try:
                self._mcp.execute_tool_call_sync("stop_music", {}, timeout=10.0)
                self.get_logger().info("🎧 DJ auto-stop: музыка остановлена")
            except Exception as e:
                self.get_logger().warning(f"🎧 DJ auto-stop: ошибка stop_music: {e}")
            return

        # Устанавливаем безопасный фолбэк-интервал (45с) — LLM должен перезаписать
        # его через set_dj_mode(enabled=True, next_transition_sec=X) в конце перехода.
        # 120с было слишком много: если LLM забывал set_dj_mode, трек заканчивался
        # и наступала долгая тишина (20-40+ сек). 45с — разумный компромисс.
        self._dj_transition_count += 1
        n = self._dj_transition_count
        self._dj_next_transition_at = (
            now + 45.0
        )  # фолбэк если LLM не вызвал set_dj_mode

        prompt = self._build_dj_prompt(n)
        self.get_logger().info(
            f"🎧 DJ Auto-transition #{n}: запускаю агента"
            + (f" [тема: {self._dj_theme!r}]" if self._dj_theme else "")
        )
        asyncio.run_coroutine_threadsafe(self._agent_run(prompt), self._loop)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.dialogue_manager.state.value
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DialogueNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
