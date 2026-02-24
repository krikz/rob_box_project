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
        self.declare_parameter("history_max_turns", 10)
        self.declare_parameter("agent_max_turns", 10)
        self.declare_parameter("dialogue_timeout", 30.0)
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
            tools = self._make_tools() if self._mcp else []

            self._agent = Agent(
                name="RobBox",
                instructions=self._system_prompt,
                tools=tools,
                model=model,
                model_settings=ModelSettings(
                    temperature=self._temperature,
                    max_tokens=self._max_tokens,
                    parallel_tool_calls=False,
                ),
            )
            prompt_preview = self._system_prompt[:200].replace("\n", "↵")
            self.get_logger().info(
                f"🤖 Agent built: {model_name} @ {base_url} ({len(tools)} tools) | "
                f"instructions={len(self._system_prompt)} bytes | "
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
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        @function_tool
        async def speak_text(text: str, animation: str = "neutral") -> str:
            """Произнести текст с анимацией. ВСЕГДА вызывать для ответа пользователю.
            Возвращает TASK_COMPLETE — после этого верни текстовый ответ без tool_calls чтобы завершить итерацию."""
            # Collect ALL spoken texts immediately (before lock) so that when
            # multiple speak_text calls queue on the lock, _spoken_texts already
            # has all texts for proper history saving.
            self._spoken_texts.append(text)
            async with lock:
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

        return [
            speak_text, play_sound, play_animation,
            memory_context, memory_save, memory_search,
            get_current_time, get_robot_status, get_battery_level,
            navigate_to_waypoint, move_direction,
            set_volume, set_pitch,
            execute_music_code, stop_music, set_vibe_preset, get_music_state,
        ]

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

        # ── LISTENING / DIALOGUE: process ───────────────────────────
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

        asyncio.run_coroutine_threadsafe(self._agent_run(clean), self._loop)

    # ────────────────────────────────────────────────────────────────
    # Agent execution
    # ────────────────────────────────────────────────────────────────

    async def _agent_run(self, user_input: str) -> None:
        """Run the OpenAI Agents SDK loop for a single user turn."""
        with self._task_lock:
            self._run_task = asyncio.current_task()
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
            self.get_logger().info("⏰ Dialogue timeout → IDLE")
            # Clear conversation history so next session starts fresh.
            # Without this, old context (e.g. joke about "таракашку") persists
            # into the next session and LLM uses it to answer unrelated requests.
            with self._conv_lock:
                if self._conversation:
                    self.get_logger().info(
                        f"🧹 Clearing {len(self._conversation)} history items on IDLE timeout"
                    )
                    self._conversation = []
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
