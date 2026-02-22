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

        self._provider: str = self.get_parameter("provider").value
        self._temperature: float = self.get_parameter("temperature").value
        self._max_tokens: int = self.get_parameter("max_tokens").value
        self._max_turns: int = self.get_parameter("history_max_turns").value
        self._agent_max_turns: int = self.get_parameter("agent_max_turns").value
        self._llm_timeout: float = self.get_parameter("llm_timeout_sec").value

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

        # ── speak_text stop-loop tracking ────────────────────────────
        # When speak_text raises CancelledError to stop the SDK loop,
        # this flag lets _agent_run distinguish it from external barge-in.
        # _spoken_texts accumulates ALL parallel speak_text calls so the full
        # assistant response (not just the first phrase) is saved to history.
        self._speak_done: bool = False
        self._spoken_texts: List[str] = []

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

        # ── MCP Adapter ──────────────────────────────────────────────
        self._mcp: Optional[LLMToolCallAdapter] = None
        if self.get_parameter("enable_mcp_tools").value:
            try:
                self._mcp = LLMToolCallAdapter(self)
                self.get_logger().info("✅ MCP adapter ready")
            except Exception as exc:
                self.get_logger().error(f"❌ MCP adapter failed: {exc}")

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
            # Collect ALL spoken texts immediately (before TTS wait) so that when
            # multiple speak_text calls fire in parallel in one LLM batch, the full
            # response is accumulated in _spoken_texts for proper history saving.
            self._spoken_texts.append(text)
            result_str = await _call("speak_text", {"text": text, "animation": animation}, timeout=60.0)
            # ── Wait for TTS to actually finish playing ───────────────────
            # MCP speak_text is async (returns immediately), but we must block
            # here so the agent calls speak_text sequentially, not in parallel
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
            # ── Stop the SDK agent loop ──────────────────────────────
            # Response is delivered. Raise CancelledError so Runner exits
            # immediately — prevents extra LLM turns (memory_save, animations,
            # extra speak_text calls) that exhaust max_turns and leak into
            # next request, causing OOM from accumulated async tool calls.
            self._speak_done = True
            raise asyncio.CancelledError("speak_text done — stopping agent loop")

        @function_tool
        async def play_sound(sound: str) -> str:
            """Воспроизвести звуковой эффект."""
            return await _call("play_sound", {"sound": sound})

        @function_tool
        async def play_animation(animation: str, duration: float = 3.0) -> str:
            """Запустить LED анимацию на указанное время."""
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

        return [
            speak_text, play_sound, play_animation,
            memory_context, memory_save, memory_search,
            get_current_time, get_robot_status, get_battery_level,
            navigate_to_waypoint, move_direction,
            set_volume, set_pitch,
        ]

    def _log_config(self) -> None:
        self.get_logger().info(f"  Provider : {self._provider}")
        self.get_logger().info(f"  Model    : {self._resolve_model()}")
        self.get_logger().info(f"  Wake     : {self.dialogue_manager.wake_words}")
        self.get_logger().info(f"  History  : {self._max_turns} turns")
        self.get_logger().info(f"  Timeout  : {self.dialogue_manager.dialogue_timeout}s")

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

        self._speak_done = False
        self._spoken_texts = []
        self.get_logger().info(f"🤔 User: {user_input[:120]}")

        # --- Role-reset / Role-assign detection ---
        # Clear conversation history when:
        # 1. User explicitly exits a role ("будь собой", etc.)
        # 2. User ENTERS a new role ("разговаривай как X", "ты теперь X")
        #
        # Why: to_input_list() stores all tool_call/result history but NOT the agent
        # instructions (system prompt). If the history accumulates 5+ turns of "Lenin"
        # responses, that pattern overrides the system prompt and the LLM stays in-role
        # regardless of what the user asks next. Clearing on role-assign prevents this.
        _ROLE_RESET_PHRASES = (
            "будь собой", "ты обычный", "вернись обратно", "прекрати роль",
            "сбрось роль", "забудь роль", "будь нормальным", "обычный режим",
            "ты снова ты", "стоп роль", "хватит роли", "выйди из роли",
            "прекрати быть", "хватит быть",
        )
        _ROLE_ASSIGN_PHRASES = (
            "разговаривай как", "говори как", "ты теперь", "побудь", "сыграй роль",
            "притворись", "изображай", "представь себя", "веди себя как",
            "отвечай как", "будь как", "ты пастырь", "ты ленин", "ты робот",
            "ты полицейский", "войди в роль",
        )
        _lower = user_input.lower()
        if any(ph in _lower for ph in _ROLE_RESET_PHRASES):
            with self._conv_lock:
                self._conversation = []
            self.get_logger().info("🧹 Role-reset detected — conversation history cleared")
        elif any(ph in _lower for ph in _ROLE_ASSIGN_PHRASES):
            with self._conv_lock:
                self._conversation = []
            self.get_logger().info("🎭 Role-assign detected — conversation history cleared for clean slate")

        try:
            with self._conv_lock:
                input_list = list(self._conversation) + [
                    {"role": "user", "content": user_input}
                ]

            if self._agent is None:
                self.get_logger().error("❌ Agent not initialised")
                return

            # Use run_streamed so we can monitor events and bail early
            # if agent loops on non-speak_text tool calls (play_sound/animation spam)
            streamed = Runner.run_streamed(
                self._agent, input_list, max_turns=self._agent_max_turns
            )

            loop_aborted = False  # set True when _consume() exits early

            async def _consume() -> None:
                nonlocal loop_aborted
                non_speech_calls = 0
                spoke_once = False  # True after first speak_text call
                async for event in streamed.stream_events():
                    if event.type != "run_item_stream_event":
                        continue
                    item = event.item
                    if item.type == "tool_call_item":
                        tool_name = getattr(item.raw_item, "name", "") or ""
                        self.get_logger().debug(f"🔧 tool_call: {tool_name}")
                        if tool_name == "speak_text":
                            non_speech_calls = 0  # reset counter on every speak
                            spoke_once = True
                        else:
                            non_speech_calls += 1
                            # After speak_text fired at least once, allow only 2
                            # decorative calls alongside it, then abort.
                            # Before first speak_text (e.g. thinking sounds) allow up to 3.
                            limit = 2 if spoke_once else 3
                            if non_speech_calls >= limit:
                                self.get_logger().warning(
                                    f"⚠️ Agent stuck: {non_speech_calls} non-speech "
                                    f"tool calls (spoke_once={spoke_once}) — aborting loop"
                                )
                                loop_aborted = True
                                return  # stop consuming — stream will be GC'd

            await asyncio.wait_for(_consume(), timeout=self._llm_timeout * 3)

            if loop_aborted:
                # Partial history from an aborted stream is corrupt: it contains
                # tool_result messages without their preceding tool_call messages,
                # which causes a 400 error on the next API call.  Keep existing
                # clean history instead.
                self.get_logger().info("🔁 Loop aborted early — keeping previous conversation history")
            else:
                # Persist trimmed history for the next turn
                with self._conv_lock:
                    self._conversation = self._trim_history(streamed.to_input_list())

            self.get_logger().info(
                f"✅ Agent done. Output: {(streamed.final_output or '')[:80]}"
            )

        except asyncio.CancelledError:
            if self._speak_done:
                # Graceful stop: speak_text completed and raised CancelledError
                # to prevent extra tool calls. Save history so next turn has context.
                # Join ALL spoken texts (LLM may fire multiple speak_text in parallel
                # in one batch — _spoken_texts collects them all, not just the first).
                full_response = " ".join(self._spoken_texts)
                self.get_logger().info(f"✅ Agent done (speak_text completed, loop stopped). Response: {full_response[:80]}")
                with self._conv_lock:
                    partial = list(self._conversation) + [
                        {"role": "user", "content": user_input},
                        {"role": "assistant", "content": full_response},
                    ]
                    self._conversation = self._trim_history(partial)
            else:
                self.get_logger().info("🛑 Agent run cancelled (barge-in / new input)")
                # Do NOT update history — partial turn discarded

        except MaxTurnsExceeded:
            self.get_logger().error(
                f"❌ Agent exceeded max tool-call turns ({self._agent_max_turns}). "
                "Consider increasing agent_max_turns param."
            )
            # Do NOT call _speak_direct — agent likely already spoke several phrases
            # and injecting another TTS would corrupt the playback queue

        except asyncio.TimeoutError:
            self.get_logger().error(f"❌ Agent timed out ({self._llm_timeout * 3:.0f}s)")
            self._speak_direct("Извините, не получилось ответить вовремя.")

        except Exception as exc:
            self.get_logger().error(f"❌ Agent error: {exc}")
            # If the API rejected our history as malformed (e.g. tool message
            # without a preceding tool_call), wipe it so the next turn starts fresh.
            if "invalid_request_error" in str(exc) or "tool" in str(exc).lower():
                self.get_logger().warning("🧹 Clearing conversation history due to malformed state")
                with self._conv_lock:
                    self._conversation = []
            # Do NOT call _speak_direct here — agent may have already
            # started speaking (e.g. mid-joke) and injecting an error
            # message would corrupt the TTS queue

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

    def _cancel_run(self, reason: str) -> None:
        # Reset _speak_done on external cancel so that a speak_text which
        # completed just before barge-in doesn't trigger history saving.
        # History for interrupted turns should always be discarded.
        self._speak_done = False
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
