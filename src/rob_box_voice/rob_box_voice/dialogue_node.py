#!/usr/bin/env python3
"""
dialogue_node.py — Voice dialogue agent (OpenAI Agents SDK + DeepSeek/Qwen)

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
import threading
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from agents import Agent, Runner, function_tool
from agents.exceptions import MaxTurnsExceeded
from agents.model_settings import ModelSettings
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from openai import AsyncOpenAI

from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

from .core.dialogue_manager import DialogueManager, DialogueState

try:
    from rob_box_voice.core.voice_memory import VoiceMemory as _VoiceMemory

    _VOICE_MEMORY_AVAILABLE = True
except ImportError:
    _VoiceMemory = None  # type: ignore[assignment,misc]
    _VOICE_MEMORY_AVAILABLE = False

try:
    from .skills import MusicSkill, NavigationSkill, MemorySkill, StatusSkill

    _SKILLS_AVAILABLE = True
except ImportError:
    _SKILLS_AVAILABLE = False  # skills module not yet installed

# ── Feature flag — set USE_SKILLS=false to fall back to flat tool mode ────────
USE_SKILLS: bool = os.getenv("USE_SKILLS", "true").lower() == "true"


class DialogueNode(Node):
    """Voice dialogue agent built on OpenAI Agents SDK."""

    # ── Provider configs ────────────────────────────────────────────
    PROVIDERS = {
        "deepseek": {
            "base_url": "https://api.deepseek.com/v1",
            "model": "deepseek-chat",
            "env_vars": ["DEEPSEEK_API_KEY", "LLM_API_KEY"],
        },
        "qwen": {
            "base_url": "https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            "model": "qwen-max",
            "env_vars": ["QWEN_API_KEY", "LLM_API_KEY"],
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
        self.declare_parameter("agent_max_turns", 10)
        self.declare_parameter("dialogue_timeout", 300.0)
        self.declare_parameter("wake_words", ["робок", "робот", "роббокс"])
        self.declare_parameter("enable_mcp_tools", True)
        self.declare_parameter("enable_fallback", False)
        self.declare_parameter("llm_timeout_sec", 35.0)
        self.declare_parameter("verbose_llm", True)

        self._provider: str = self.get_parameter("provider").value
        self._temperature: float = self.get_parameter("temperature").value
        self._max_tokens: int = self.get_parameter("max_tokens").value
        self._max_turns: int = self.get_parameter("history_max_turns").value
        self._agent_max_turns: int = self.get_parameter("agent_max_turns").value
        self._llm_timeout: float = self.get_parameter("llm_timeout_sec").value
        self._verbose_llm: bool = self.get_parameter("verbose_llm").value

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

        # ── Barge-in grace period ────────────────────────────────────
        # After STT recognition, suppress VAD barge-in for N seconds so that
        # room echo / noise immediately after user speech doesn't cancel the run.
        self._barge_in_grace_seconds: float = 5.0
        self._agent_run_start_time: float = 0.0

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

        self._response_pub = self.create_publisher(String, "/voice/dialogue/response", 10)
        self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
        self._sound_trigger_pub = self.create_publisher(String, "/voice/sound/trigger", 10)
        self._tts_control_pub = self.create_publisher(String, "/voice/tts/control", 10)

        self.create_subscription(
            String, "/voice/stt/result", self._on_stt, qos_r, callback_group=cbg
        )
        self.create_subscription(
            Bool, "/audio/vad", self._on_vad, 10, callback_group=cbg
        )
        self.create_subscription(
            String, "/voice/tts/finished", self._on_tts_finished_dlg, 10, callback_group=cbg
        )
        self.create_subscription(
            String, "/voice/sound/state", self._on_sound_state, 10, callback_group=cbg
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

        # ── Build agent ──────────────────────────────────────────────
        self._agent: Optional[Agent] = None
        self._build_agent()

        # ── Timeout timer ────────────────────────────────────────────
        self.create_timer(5.0, self._on_timeout_check)

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
            self.get_logger().info(f"✅ Prompt loaded: {prompt_file} ({len(prompt)} bytes)")
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
            self.get_logger().info(f"✅ Prompt loaded: {filename} ({len(content)} bytes)")
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
        return val or self.PROVIDERS.get(self._provider, {}).get("model", "deepseek-chat")

    def _init_voice_memory(self) -> None:
        """Init VoiceMemory for persistent turn logging. Fails silently."""
        if not _VOICE_MEMORY_AVAILABLE:
            self.get_logger().warning("⚠️ VoiceMemory unavailable — turn logging disabled")
            return
        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        ollama_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
        try:
            self._voice_memory = _VoiceMemory(db_path=db_path, ollama_base_url=ollama_url)
            stats = self._voice_memory.get_stats()
            self.get_logger().info(
                f"🧠 DialogueNode VoiceMemory: {db_path} "
                f"(turns={stats['turn_count']}, sessions={stats['session_count']})"
            )
        except Exception as exc:
            self.get_logger().error(f"❌ VoiceMemory init failed: {exc}")
            self._voice_memory = None

    def _build_agent(self) -> None:
        """(Re)build the Agent — also called after fallback provider switch."""
        try:
            api_key = self._resolve_api_key()
            base_url = self._resolve_base_url()
            model_name = self._resolve_model()

            openai_client = AsyncOpenAI(api_key=api_key, base_url=base_url)
            model = OpenAIChatCompletionsModel(
                model=model_name, openai_client=openai_client
            )

            if USE_SKILLS and _SKILLS_AVAILABLE and self._mcp:
                # ── Compositor mode: Compositor + 4 skill sub-agents ────────
                instructions = self._load_prompt_file("compositor_prompt.txt") or self._system_prompt
                agent_name = "Compositor"
                tools = self._make_output_tools() + self._build_skills(model)
                self.get_logger().info(
                    f"🎭 Skills mode: USE_SKILLS=True, {len(tools)} tools "
                    f"(3 output + {len(tools) - 3} skills)"
                )
            else:
                # ── Flat mode: single agent with all 17 tools ────────────────
                if USE_SKILLS and not _SKILLS_AVAILABLE:
                    self.get_logger().warning("⚠️ USE_SKILLS=true but skills module unavailable — falling back to flat mode")
                instructions = self._system_prompt
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
        # Captured once per tool-set build; bound to self._loop on first await.
        lock = self._output_lock

        async def _call(tool_name: str, params: dict, timeout: float = 10.0) -> str:
            result = await asyncio.get_running_loop().run_in_executor(
                None,
                lambda: mcp.execute_tool_call_sync(tool_name, params, timeout=timeout),
            )
            # Each tool result resets barge-in grace — LLM is actively working
            self._agent_run_start_time = time.monotonic()
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        @function_tool
        async def speak_text(text: str, animation: str = "neutral") -> str:
            """Произнести текст с анимацией. ВСЕГДА вызывать для ответа пользователю.
            Возвращает TASK_COMPLETE — после этого верни текстовый ответ без tool_calls чтобы завершить итерацию."""
            if self._run_cancelled:
                return "CANCELLED"
            # Collect ALL spoken texts immediately (before lock) so that when
            # multiple speak_text calls queue on the lock, _spoken_texts already
            # has all texts for proper history saving.
            self._spoken_texts.append(text)
            async with lock:
                if self._run_cancelled:
                    return "CANCELLED"
                result_str = await _call("speak_text", {"text": text, "animation": animation}, timeout=60.0)
                # ── Wait for TTS to actually finish playing ───────────────────
                # MCP speak_text is async (returns immediately), but we hold
                # the output lock until TTS finishes, so the next tool call
                # (play_sound, play_animation, etc.) only starts afterwards.
                try:
                    speech_id = json.loads(result_str).get("data", {}).get("speech_id", "")
                except Exception:
                    speech_id = ""
                if speech_id:
                    event = asyncio.Event()
                    with self._tts_events_lock:
                        self._tts_events[speech_id] = event
                    try:
                        await asyncio.wait_for(event.wait(), timeout=30.0)
                    except asyncio.TimeoutError:
                        self.get_logger().warning(f"⚠️ speak_text TTS timeout for {speech_id[:8]}")
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
                    duration = json.loads(result_str).get("data", {}).get("duration", 1.5)
                except Exception:
                    duration = 1.5
                try:
                    await asyncio.wait_for(done_event.wait(), timeout=float(duration) + 2.0)
                except asyncio.TimeoutError:
                    self.get_logger().warning(f"⚠️ play_sound timeout waiting for 'ready': {sound}")
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
        async def memory_save(content: str, tags: str = "") -> str:
            """Сохранить важную информацию в долгосрочную память."""
            return await _call("memory_save", {"content": content, "tags": tags})

        @function_tool
        async def memory_search(query: str, limit: int = 5) -> str:
            """Найти релевантную информацию в долгосрочной памяти."""
            return await _call("memory_search", {"query": query, "limit": limit})

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
            """Направить робота к именованной точке. БЛОКИРУЕТСЯ до прибытия — speak_text после вызывать ТОЛЬКО после завершения."""
            return await _call("navigate_to_waypoint", {"waypoint": waypoint}, timeout=130.0)

        @function_tool
        async def move_direction(direction: str, distance: float = 0.5) -> str:
            """Передвинуть робота. direction: 'вперёд', 'назад', 'налево', 'направо'.
            БЛОКИРУЕТСЯ до завершения движения — speak_text вызывать ТОЛЬКО после этого."""
            return await _call(
                "move_direction", {"direction": direction, "distance": distance}, timeout=70.0
            )

        @function_tool
        async def set_volume(action: str) -> str:
            """Изменить громкость голоса.
            action: 'louder' — громче, 'quieter' — тише, 'max' — максимум, 'normal' — норма."""
            return await _call("set_volume", {"action": action})

        @function_tool
        async def set_pitch(pitch: float) -> str:
            """Установить высоту голоса 0.5-2.0."""
            return await _call("set_pitch", {"pitch": pitch})

        @function_tool
        async def execute_music_code(code: str, pattern_name: str = "p1") -> str:
            """Запустить музыкальный код на синтезаторе Renardo/SuperCollider.
            Использовать ВСЕГДА когда пользователь просит сыграть мелодию, музыку, ноты.
            Пример: execute_music_code("p1 >> pluck([0,2,4,7], dur=0.5, amp=0.8)", pattern_name="p1")"""
            return await _call("execute_music_code", {"code": code, "pattern_name": pattern_name}, timeout=15.0)

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
        def search_samples(
            query: str,
            pack: str = "0_foxdot_default",
            case: str = "lower",
        ) -> str:
            """Найти семплы для play() по ключевому слову в имени файла.

            ВЫЗЫВАЙ перед созданием паттернов с play() если нужно найти букву.
            Для вокала/vocal: pack="1_pitchglitch_samples", для стандартных — pack="0_foxdot_default".
            query="*" → компактный обзор всех букв с количеством файлов.

            Args:
                query: Ключевое слово (kick, snare, hat, bass, synth, vocal, glitch).
                pack:  "0_foxdot_default" (стандартный) или "1_pitchglitch_samples" (вокал/FX).
                case:  "lower" → строчная буква в play(); "upper" → заглавная.

            Returns:
                JSON: letter, sample_index, filename, play_code для подстановки в play().
            """
            samples_root = Path(os.getenv("RENARDO_SAMPLES_PATH", "/root/.config/renardo/samples"))
            if not samples_root.exists():
                return json.dumps(
                    {"error": f"Samples dir not found: {samples_root}",
                     "hint": "Use: d1 >> play('c', sample=P[0,1,2,3]) for vocal"},
                    ensure_ascii=False,
                )
            pack_path = samples_root / pack
            if not pack_path.exists():
                available = [d.name for d in samples_root.iterdir() if d.is_dir()]
                return json.dumps(
                    {"error": f"Pack '{pack}' not found", "available_packs": available},
                    ensure_ascii=False,
                )
            exts = {".wav", ".aif", ".aiff", ".mp3"}
            all_packs = sorted([d.name for d in samples_root.iterdir() if d.is_dir()])
            spack_num = all_packs.index(pack) if pack in all_packs else 0
            spack_suffix = f", spack={spack_num}" if spack_num != 0 else ""
            if query.strip() == "*":
                overview = {}
                for folder in sorted(pack_path.iterdir()):
                    if not folder.is_dir() or folder.name.startswith("."):
                        continue
                    sub = folder / case
                    if not sub.exists():
                        sub = folder
                    count = sum(1 for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts)
                    if count:
                        overview[folder.name] = count
                return json.dumps(
                    {"pack": pack, "case": case, "letters": overview,
                     "hint": 'search_samples("kick") or search_samples("vocal", pack="1_pitchglitch_samples")'},
                    ensure_ascii=False, indent=2,
                )
            q = query.lower().strip()
            results = []
            for folder in sorted(pack_path.iterdir()):
                if not folder.is_dir() or folder.name.startswith("."):
                    continue
                sub = folder / case
                if not sub.exists():
                    sub = folder
                files = sorted([f for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts])
                for idx, f in enumerate(files):
                    if q in f.name.lower():
                        play_letter = folder.name.upper() if case == "upper" else folder.name
                        results.append(
                            {"letter": play_letter, "sample_index": idx, "spack": spack_num,
                             "filename": f.name,
                             "play_code": f'd1 >> play("{play_letter}", sample={idx}{spack_suffix})'}
                        )
                if len(results) >= 30:
                    break
            if not results:
                return json.dumps(
                    {"query": query, "pack": pack, "found": 0,
                     "hint": 'Try: "kick", "snare", "hat", "bass", "vocal", "*"'},
                    ensure_ascii=False,
                )
            return json.dumps(
                {"query": query, "pack": pack, "case": case, "found": len(results), "results": results},
                ensure_ascii=False, indent=2,
            )

        return [
            speak_text, play_sound, play_animation,
            memory_context, memory_save, memory_search,
            get_current_time, get_robot_status, get_battery_level,
            navigate_to_waypoint, move_direction,
            set_volume, set_pitch,
            search_samples, execute_music_code, stop_music, set_vibe_preset, get_music_state,
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
            result = await asyncio.get_running_loop().run_in_executor(
                None,
                lambda: mcp.execute_tool_call_sync(tool_name, params, timeout=timeout),
            )
            # Each tool result resets barge-in grace — LLM is actively working
            self._agent_run_start_time = time.monotonic()
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        @function_tool
        async def speak_text(text: str, animation: str = "neutral") -> str:
            """Произнести текст с анимацией. ВСЕГДА вызывать для ответа пользователю.
            Возвращает TASK_COMPLETE — после этого верни текстовый ответ без tool_calls чтобы завершить итерацию."""
            if self._run_cancelled:
                return "CANCELLED"
            self._spoken_texts.append(text)
            async with lock:
                if self._run_cancelled:
                    return "CANCELLED"
                result_str = await _call("speak_text", {"text": text, "animation": animation}, timeout=60.0)
                try:
                    speech_id = json.loads(result_str).get("data", {}).get("speech_id", "")
                except Exception:
                    speech_id = ""
                if speech_id:
                    event = asyncio.Event()
                    with self._tts_events_lock:
                        self._tts_events[speech_id] = event
                    try:
                        await asyncio.wait_for(event.wait(), timeout=30.0)
                    except asyncio.TimeoutError:
                        self.get_logger().warning(f"⚠️ speak_text TTS timeout for {speech_id[:8]}")
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
                    duration = json.loads(result_str).get("data", {}).get("duration", 1.5)
                except Exception:
                    duration = 1.5
                try:
                    await asyncio.wait_for(done_event.wait(), timeout=float(duration) + 2.0)
                except asyncio.TimeoutError:
                    self.get_logger().warning(f"⚠️ play_sound timeout waiting for 'ready': {sound}")
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

        return [speak_text, play_sound, play_animation]

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
            skill = MusicSkill(adapter=self._mcp, model=model, prompt_template=music_prompt, agent_max_turns=6)
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_music",
                    tool_description=(
                        "Музыкальный скилл: запрос на игру музыки, мелодии, вайба, "
                        "остановку музыки или изменение музыкального состояния."
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
                nav_prompt = "Ты — модуль навигации РОББОКСА. Управляй движением робота."
            skill = NavigationSkill(adapter=self._mcp, model=model, prompt=nav_prompt, name="NavigationSkill")
            skill_tools.append(
                skill.as_tool(
                    tool_name="handle_navigation",
                    tool_description=(
                        "Навигационный скилл: переместить робота в именованную точку "
                        "или задать направление движения."
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
                mem_prompt = "Ты — модуль памяти РОББОКСА. Управляй долгосрочной памятью."
            skill = MemorySkill(adapter=self._mcp, model=model, prompt=mem_prompt, name="MemorySkill")
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
            skill = StatusSkill(adapter=self._mcp, model=model, prompt=status_prompt, name="StatusSkill")
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

        return skill_tools

    def _log_config(self) -> None:
        self.get_logger().info(f"  Provider : {self._provider}")
        self.get_logger().info(f"  Model    : {self._resolve_model()}")
        self.get_logger().info(f"  Wake     : {self.dialogue_manager.wake_words}")
        self.get_logger().info(f"  History  : {self._max_turns} turns")
        self.get_logger().info(f"  Timeout  : {self.dialogue_manager.dialogue_timeout}s")
        self.get_logger().info(f"  VerboseLLM: {self._verbose_llm}")

    # ────────────────────────────────────────────────────────────────
    # ROS2 callbacks
    # ────────────────────────────────────────────────────────────────

    def _on_vad(self, msg: Bool) -> None:
        """Rising-edge VAD barge-in: cancel current run on speech start."""
        is_speech = msg.data
        if is_speech and not self._vad_speech_detected:
            self._vad_speech_detected = True
            self.get_logger().debug("🎤 VAD: speech start")
            # Grace period: ignore barge-in for N seconds after run started
            # to avoid room echo / noise right after STT cancelling the agent.
            elapsed = time.monotonic() - self._agent_run_start_time
            if elapsed < self._barge_in_grace_seconds:
                self.get_logger().debug(
                    f"🔕 Barge-in suppressed (grace period, {elapsed:.1f}s < {self._barge_in_grace_seconds}s)"
                )
                return
            self._cancel_run("barge-in VAD")
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

        # Reset grace period timer
        self._agent_run_start_time = time.monotonic()

        # ── Immediate thinking sound ─────────────────────────────────
        # Play confirmation immediately — before LLM even starts
        _snd = String()
        _snd.data = "thinking"
        self._sound_trigger_pub.publish(_snd)

        self.dialogue_manager.transition_state(DialogueState.DIALOGUE)
        self._publish_state()

        asyncio.run_coroutine_threadsafe(self._agent_run(clean), self._loop)

    # ────────────────────────────────────────────────────────────────
    # Agent execution
    # ────────────────────────────────────────────────────────────────

    async def _agent_run(self, user_input: str) -> None:
        """Run the OpenAI Agents SDK loop for a single user turn."""
        with self._task_lock:
            self._run_task = asyncio.current_task()
        self._run_cancelled = False
        self._spoken_texts = []
        self.get_logger().info(f"🤔 User: {user_input[:120]}")
        if self._voice_memory is not None:
            try:
                self._voice_memory.save_turn("user", user_input)
            except Exception as exc:
                self.get_logger().warning(f"⚠️ memory save_turn(user) failed: {exc}")

        try:
            with self._conv_lock:
                input_list = list(self._conversation) + [
                    {"role": "user", "content": user_input}
                ]

            if self._agent is None:
                self.get_logger().error("❌ Agent not initialised")
                return

            if self._verbose_llm:
                self.get_logger().info(
                    f"📥 LLM INPUT ({len(input_list)} messages):\n"
                    + json.dumps(input_list, ensure_ascii=False, indent=2)
                )

            result = await asyncio.wait_for(
                Runner.run(self._agent, input_list, max_turns=self._agent_max_turns),
                timeout=self._llm_timeout * 3,
            )

            # Build clean history: user text + what was spoken, without tool_call
            # patterns. Using to_input_list() would include assistant tool_call
            # messages which cause the LLM to repeat action sequences (e.g. counting)
            # when asked an unrelated follow-up request.
            spoken = " ".join(self._spoken_texts) if self._spoken_texts else (result.final_output or "")
            if self._verbose_llm:
                self.get_logger().info(f"📤 LLM OUTPUT:\n{spoken}")
            else:
                self.get_logger().info(f"✅ Agent done. Response: {spoken[:80]}")

            if self._voice_memory is not None and spoken:
                try:
                    self._voice_memory.save_turn("assistant", spoken)
                except Exception as exc:
                    self.get_logger().warning(f"⚠️ memory save_turn(assistant) failed: {exc}")

            with self._conv_lock:
                self._conversation = self._trim_history(
                    list(self._conversation)
                    + [{"role": "user", "content": user_input},
                       {"role": "assistant", "content": spoken}]
                )

        except asyncio.CancelledError:
            self.get_logger().info("🛑 Agent run cancelled (barge-in / new input)")
            # Do NOT update history — partial turn discarded

        except MaxTurnsExceeded:
            self.get_logger().error(
                f"❌ Agent exceeded max tool-call turns ({self._agent_max_turns}). "
                "Consider increasing agent_max_turns param."
            )

        except asyncio.TimeoutError:
            self.get_logger().error(f"❌ Agent timed out ({self._llm_timeout * 3:.0f}s)")
            self._speak_direct("Извините, не получилось ответить вовремя.")

        except Exception as exc:
            self.get_logger().error(f"❌ Agent error: {exc}")
            if "invalid_request_error" in str(exc) or "tool" in str(exc).lower():
                self.get_logger().warning("🧹 Clearing conversation history due to malformed state")
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
        """Keep the last `max_turns` complete user+assistant pairs."""
        max_items = self._max_turns * 2
        return items[-max_items:] if len(items) > max_items else items

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────

    def _speak_direct(self, text: str) -> None:
        """Publish a response directly (no LLM) — errors, silence confirmations."""
        msg = String()
        msg.data = json.dumps(
            {"chunk": "final", "ssml": f"<speak>{text}</speak>", "emotion": "neutral"},
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
            self.get_logger().info("⏰ Dialogue timeout → IDLE (history preserved, sliding window)")
            self._publish_state()

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
