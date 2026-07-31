#!/usr/bin/env python3
"""dialogue_node.py — Voice dialogue ROS2 shell (Phase 6 v2 / W5).

Thin ROS2 shell that composes DialogCore over the harness ports
(LLMProvider, ToolProvider, MemoryStore, DSM). Owns only ROS2 pub/sub,
the asyncio loop driver, DJ-mode hook, barge-in/cancel, TTS/sound
awaiter release, and lifecycle. Wake-word / silence classification
lives in core.dialogue_text; DJ-mode state machine in core.dj_mode;
TTS chunking + SSML framing + awaiter registry in core.speak_helpers.

Replaces the 2181-line predecessor by extracting the agent loop, every
function_tool body (OpenAI Agents SDK decorators), the provider registry,
history trimming, and all inline tool implementations into the harness
ports delivered in W1-W4.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import os
import threading
from typing import Any, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rob_box_harness.core.dialog_core import DialogCore, DialogResult
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider
from rob_box_harness.memory import InMemoryStore, MemoryStore, SQLiteVoiceMemory
from rob_box_harness.providers import (
    DEEPSEEK_DEFAULT_BASE_URL,
    DEEPSEEK_DEFAULT_MODEL,
    build_deepseek_provider,
)
from rob_box_harness.tools import FakeToolProvider, ToolProvider

from rob_box_voice.core.dialogue_text import (
    has_wake_word, is_silence_command, is_unsilence_command, strip_wake_word,
)
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.speak_helpers import (
    EffectAwaiterRegistry, build_ssml_payload, split_into_chunks,
    strip_history_marker,
)

ASYNCIO_LOOP_DRIVER_MAX_WORKERS: int = 1
ASYNCIO_LOOP_DRIVER_NAME_PREFIX: str = "dialogue-async-loop"
ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S: float = 2.0

class DialogueNode(Node):
    """ROS2 shell that composes DialogCore over the harness ports."""
    def __init__(self) -> None:  # noqa: D401 — ROS2 ctor signature
        super().__init__("dialogue_node")
        self._declare_params()
        self._system_prompt: str = self._load_system_prompt()
        self._verbose_llm: bool = bool(self.get_parameter("verbose_llm").value)
        self._wake_words: List[str] = list(self.get_parameter("wake_words").value)

        self._loop = asyncio.new_event_loop()
        self._asyncio_loop_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=ASYNCIO_LOOP_DRIVER_MAX_WORKERS,
            thread_name_prefix=ASYNCIO_LOOP_DRIVER_NAME_PREFIX,
        )
        self._asyncio_loop_future: Optional[concurrent.futures.Future] = (
            self._asyncio_loop_executor.submit(self._loop.run_forever)
        )
        self._run_task: Optional[asyncio.Task] = None
        self._task_lock = threading.Lock()
        self._run_cancelled: bool = False
        self._vad_speech_detected: bool = False
        self._effects = EffectAwaiterRegistry(
            release_tts=lambda ev: self._loop.call_soon_threadsafe(ev.set),
            release_sound=lambda ev: self._loop.call_soon_threadsafe(ev.set),
        )

        self._memory: MemoryStore = self._build_memory()
        self._dsm: DialogueStateMachine = DialogueStateMachine(
            silence_timeout=float(self.get_parameter("dialogue_timeout").value),
        )
        self._core: DialogCore = DialogCore(
            llm=self._build_llm(),
            tools=self._build_tool_provider(),
            memory=self._memory,
            dsm=self._dsm,
            history_trim_limit=int(self.get_parameter("history_max_turns").value),
            inactivity_timeout=float(self.get_parameter("dialogue_timeout").value),
        )

        cbg = ReentrantCallbackGroup()
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        self._response_pub = self.create_publisher(
            String, "/voice/dialogue/response", 10)
        self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
        self._sound_trigger_pub = self.create_publisher(
            String, "/voice/sound/trigger", 10)
        self._tts_control_pub = self.create_publisher(
            String, "/voice/tts/control", 10)
        # Music safety-net hook (issue #935): when the dialog ends and the
        # LLM forgot to call stop_music(), we still want playback to stop.
        # We publish a JSON payload on /mcp/music_cleanup so the MCP server
        # runs ``MusicManager.stop_music_on_session_end()`` for us.
        try:
            self._music_cleanup_pub = self.create_publisher(
                String, "/mcp/music_cleanup", 10)
            self.get_logger().info(
                "🎵 [dialogue_node] Publisher на /mcp/music_cleanup готов (issue #935)"
            )
        except Exception as exc:  # noqa: BLE001
            self._music_cleanup_pub = None
            self.get_logger().warning(
                f"⚠️ [dialogue_node] Не удалось создать /mcp/music_cleanup publisher: {exc}"
            )
        self.create_subscription(
            String, "/voice/stt/result", self._on_stt, qos_r, callback_group=cbg)
        self.create_subscription(
            Bool, "/audio/vad", self._on_vad, 10, callback_group=cbg)
        self.create_subscription(
            String, "/voice/tts/finished", self._on_tts_finished, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/voice/sound/state", self._on_sound_state, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/voice/dj_mode",
            lambda m: self._dj.handle_message(m.data), 10, callback_group=cbg)

        # Deferred music cleanup (issue #935 v2): music should keep playing
        # while TTS is still speaking (rap, poetry).  Cleanup is published
        # only after the *last* TTS chunk finishes.
        self._pending_music_cleanup: bool = False

        self._dj = DJModeController(
            hook=DJHook(
                dispatch=self._dispatch_turn,
                is_active=lambda: (self._run_task is not None
                                   and not self._run_task.done()),
                is_dialogue_active=lambda: self._dsm.current_state in (
                    DialogueStateKind.DIALOGUE, DialogueStateKind.SILENCED),
            ),
            logger=self.get_logger(),
        )
        self.create_timer(5.0, self._on_inactivity_check)
        self.create_timer(DJModeController.DJ_TICK_INTERVAL_S, self._dj.tick)
        self.get_logger().info("✅ DialogueNode shell ready (DialogCore wired)")
    def _declare_params(self) -> None:
        self.declare_parameter("llm_provider", "deepseek")
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
        self.declare_parameter("llm_timeout_sec", 90.0)
        self.declare_parameter("verbose_llm", True)
        self.declare_parameter("history_excluded_tools", ["handle_navigation"])
        self.declare_parameter("sqlite_db_path", "~/.rob_box/voice.db")
        # W5a: select the tool-provider backend. ``ros_mcp`` is the
        # production path (LLMToolCallAdapter → ROSMCPToolProvider);
        # ``fake`` swaps in FakeToolProvider for unit tests; ``none``
        # disables tools entirely. Tests override this via launch /
        # YAML so the W5a assertion (provider.list_tools() > 0) can
        # be exercised deterministically without spinning up rclpy
        # publishers.
        self.declare_parameter("tool_provider", "ros_mcp")
    def _load_system_prompt(self) -> str:
        prompt_file = self.get_parameter("system_prompt_file").value
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory("rob_box_voice")
            with open(os.path.join(pkg, "prompts", prompt_file),
                      "r", encoding="utf-8") as fh:
                prompt = fh.read()
            self.get_logger().info(
                f"✅ Prompt loaded: {prompt_file} ({len(prompt)} bytes)")
            return prompt
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ Prompt not found ({exc})")
            return "Ты ROBBOX — умный робот-ассистент. Отвечай кратко и по делу."
    def _build_memory(self) -> MemoryStore:
        try:
            store: MemoryStore = SQLiteVoiceMemory(
                db_path=self.get_parameter("sqlite_db_path").value)
            future = asyncio.run_coroutine_threadsafe(store.init(), self._loop)
            future.result(timeout=5.0)
            return store
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ SQLiteVoiceMemory init failed ({exc}); InMemoryStore")
            store = InMemoryStore()
            try:
                future = asyncio.run_coroutine_threadsafe(
                    store.init(), self._loop)
                future.result(timeout=3.0)
            except Exception:
                pass
            return store
    def _build_llm(self) -> Any:
        # Keep the provider selector explicit.  ``AsyncOpenAI`` defaults to
        # api.openai.com when ``base_url`` is omitted, so silently accepting an
        # unknown provider can leak a DeepSeek key to the wrong endpoint.
        provider_name = str(
            self.get_parameter("llm_provider").value or "deepseek"
        ).strip().lower()
        if provider_name != "deepseek":
            raise ValueError(
                f"Unsupported llm_provider={provider_name!r}; "
                "this dialogue shell currently supports only 'deepseek'"
            )

        # Resolve the endpoint here instead of relying on SDK defaults.  This
        # makes the production route visible and testable at the ROS shell
        # boundary, where YAML parameters enter the application.
        base_url = str(self.get_parameter("base_url").value or "").strip()
        model = str(self.get_parameter("model").value or "").strip()
        return build_deepseek_provider(
            api_key=self.get_parameter("api_key").value or None,
            base_url=base_url or DEEPSEEK_DEFAULT_BASE_URL,
            model=model or DEEPSEEK_DEFAULT_MODEL,
        )
    def _build_tool_provider(self) -> ToolProvider:
        # W5a: wire the real ROSMCPToolProvider when ``tool_provider``
        # is the default ``"ros_mcp"``. The previous version silently
        # fell through to FakeToolProvider, leaving voice commands
        # like "открой шторы" / "сохрани точку" as no-ops because the
        # LLM saw an empty tool registry. The new path:
        #
        # 1. Probe the ``rob_box_mcp_tools`` package (ament_index) so
        #    we know the bridge is buildable on this image.
        # 2. Lazy-import ``LLMToolCallAdapter`` from
        #    ``rob_box_mcp_tools`` (no static ``exec_depend`` on the
        #    voice package — the import only fires when the operator
        #    asks for MCP).
        # 3. Construct the bridge + ``ROSMCPToolProvider`` and feed
        #    it the 34 manifests from ``ToolRegistry``.
        # 4. Wrap with ``LegacyToolProviderAdapter`` so the legacy
        #    ``discover/execute`` contract that ``DialogCore`` accepts
        #    is satisfied.
        # 5. Assert ``list_tools()`` is non-empty — silent regression
        #    guard against the W5a mismatch re-appearing.
        backend = str(self.get_parameter("tool_provider").value or "ros_mcp")
        if backend == "fake" or backend == "none":
            self.get_logger().info(
                f"⚙️ tool_provider={backend}: using FakeToolProvider "
                "(0 MCP tools, LLM chat-only)."
            )
            return FakeToolProvider()
        if backend != "ros_mcp":
            self.get_logger().warning(
                f"⚠️ Unknown tool_provider backend {backend!r}; "
                "expected one of 'ros_mcp', 'fake', 'none'. "
                "Falling back to FakeToolProvider (LLM chat-only)."
            )
            return FakeToolProvider()
        # Probe rob_box_mcp_tools so we surface a clear error if the
        # image was built without the bridge — better than letting
        # the ImportError surface deep inside the LLM tool loop.
        try:
            from ament_index_python.packages import (
                get_package_share_directory as _ament_probe,
            )
            _ament_probe("rob_box_mcp_tools")
        except Exception as exc:  # noqa: BLE001 — startup probe
            raise RuntimeError(
                "tool_provider='ros_mcp' requires the rob_box_mcp_tools "
                f"ROS package to be installed; ament_index probe failed: "
                f"{exc!r}. Either install the package, rebuild the "
                "Docker image with --packages-up-to rob_box_voice, or "
                "set the tool_provider launch arg to 'fake'/'none' for "
                "a chat-only deployment."
            ) from exc
        # Lazy import — rob_box_voice does not statically depend on
        # rob_box_mcp_tools (no <exec_depend>); the import only
        # fires here when the operator asked for MCP tools.
        try:
            from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter
        except ImportError as exc:
            raise RuntimeError(
                "tool_provider='ros_mcp' requires the Python package "
                "'rob_box_mcp_tools' to be importable. The ament "
                "probe above succeeded, but `import "
                f"rob_box_mcp_tools.llm_adapter` failed: {exc!r}. "
                "Check that the install image includes rob_box_mcp_tools."
            ) from exc
        bridge = LLMToolCallAdapter(self)
        provider = ROSMCPToolProvider(bridge)
        # Feed the 34 manifests from the harness-side catalog. The
        # provider's update_tools() expects the OpenAI-style envelope
        # ({type: function, function: {name, description,
        # parameters}}); build it from ToolRegistry.list_tools()
        # so the LLM-facing surface is the single source of truth.
        registry = ToolRegistry()
        provider.update_tools(
            [
                {
                    "type": "function",
                    "function": {
                        "name": spec.name,
                        "description": spec.description,
                        "parameters": dict(spec.parameters),
                    },
                }
                for spec in registry.list_tools()
            ]
        )
        # Silent regression guard: if the bridge came up but the
        # manifest is empty, the LLM would silently no-op every
        # tool request. Surface a loud error instead so operators
        # notice the wiring mismatch at startup.
        catalogue = provider.list_tools()
        if not catalogue:
            raise RuntimeError(
                "ROSMCPToolProvider came up with an empty tool "
                "catalogue (0 tools). The 34-manifest ToolRegistry "
                "did not register correctly. Refusing to start — "
                "voice commands would silently no-op."
            )
        self.get_logger().info(
            f"✅ tool_provider='ros_mcp': {len(catalogue)} MCP tools "
            f"wired via LLMToolCallAdapter → ROSMCPToolProvider "
            f"(first: {catalogue[0].name!r})."
        )
        # DialogCore consumes the legacy ``discover/execute`` port
        # contract; adapt the core provider so the harness's
        # orchestration layer stays unchanged.
        return adapt_tool_provider(provider)
    def _on_vad(self, msg: Bool) -> None:
        if msg.data and not self._vad_speech_detected:
            self._vad_speech_detected = True
            self.get_logger().debug("🎤 VAD: speech start")
        elif not msg.data and self._vad_speech_detected:
            self._vad_speech_detected = False
    def _on_stt(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        text_lower = text.lower()
        state = self._dsm.current_state
        if state == DialogueStateKind.SILENCED:
            if is_unsilence_command(text_lower):
                self._dsm.on_event(DialogueEvent.UNSILENCE)
                self._publish_state()
            return
        # Universal wake-word gate — only direct address to robot can
        # start or interrupt a dialogue. This prevents false barge-in
        # from background noise, TV, or the robot's own TTS echo.
        # (Regression fix: was incorrectly gated on state==IDLE only.)
        if not has_wake_word(text_lower, self._wake_words):
            self.get_logger().debug(f"🔇 Ignored (no wake word): {text[:60]}")
            return
        clean = strip_wake_word(text, self._wake_words)
        if not clean:
            return
        if is_silence_command(text_lower):
            self._handle_silence()
            return
        self._cancel_run("new STT input")
        sfx = String()
        sfx.data = "thinking"
        self._sound_trigger_pub.publish(sfx)
        # Wake-word gate: when we cross from IDLE the wake-word itself
        # has to drive IDLE → LISTENING, then the speech below drives
        # LISTENING → DIALOGUE. Without the WAKE_WORD event the strip
        # above hides the trigger from DialogCore's on_user_input and
        # the DSM gets stuck in IDLE. (W6 integration tests caught
        # this regression in the W5 shell rewrite.)
        if state == DialogueStateKind.IDLE:
            self._dsm.on_event(DialogueEvent.WAKE_WORD)
            self._publish_state()
        self._dsm.on_event(DialogueEvent.STT_RESULT)
        self._publish_state()
        if self._dj.state.enabled:
            clean = self._dj.preamble() + clean
        if self._verbose_llm:
            self.get_logger().info(f"📥 LLM INPUT: {clean[:200]!r}")
        self._dispatch_turn(clean)
    def _on_tts_finished(self, msg: String) -> None:
        self._effects.handle_tts_finished(msg.data or "")
        if self._pending_music_cleanup:
            self.get_logger().info("🎵 TTS finished — scheduling deferred cleanup in 1s")
            if hasattr(self, '_cleanup_defer_handle') and self._cleanup_defer_handle:
                self._cleanup_defer_handle.cancel()
            self._cleanup_defer_handle = self._loop.call_later(
                1.0, self._publish_deferred_cleanup
            )
    def _on_sound_state(self, msg: String) -> None:
        self._effects.handle_sound_state(msg.data or "")
    def _publish_deferred_cleanup(self) -> None:
        """Timer callback: publish deferred music cleanup."""
        self.get_logger().info("🎵 deferred cleanup timer fired")
        if self._pending_music_cleanup:
            self._pending_music_cleanup = False
            self._publish_music_cleanup(reason="tts_finished")
    def _dispatch_turn(self, user_input: str) -> None:
        asyncio.run_coroutine_threadsafe(self._run_turn(user_input), self._loop)

    async def _run_turn(self, user_input: str) -> None:
        with self._task_lock:
            self._run_task = asyncio.current_task()
        self._run_cancelled = False
        try:
            result: DialogResult = await self._core.process_input(user_input)
            self._handle_result(result)
        except asyncio.CancelledError:
            self.get_logger().info("🛑 Turn cancelled (barge-in)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"❌ DialogCore error: {exc}")
            self._speak_direct("Что-то я задумался, повтори пожалуйста")
        finally:
            with self._task_lock:
                self._run_task = None
            # Issue #935 v2: always defer music cleanup — the
            # _on_tts_finished callback publishes it when TTS is
            # truly done. Fallback timer ensures cleanup fires even
            # when no TTS was queued (pure music-only turns).
            self._pending_music_cleanup = True
            self.get_logger().info(
                "🎵 music_cleanup deferred — waiting for TTS or 10s fallback"
            )
            if self._dsm.current_state == DialogueStateKind.DIALOGUE:
                self._dsm.on_event(DialogueEvent.DIALOGUE_END)
                self._publish_state()
    def _handle_result(self, result: DialogResult) -> None:
        if result.error is not None:
            self.get_logger().warning(f"⚠️ DialogCore error: {result.error}")
        spoken = strip_history_marker(result.spoken_text or "")
        if not spoken:
            self.get_logger().warning("⚠️ Empty assistant response — fallback")
            spoken = "Что-то я задумался, повтори пожалуйста"
        self._publish_response(spoken)
        self.get_logger().info(
            f"📤 LLM OUTPUT: {spoken[:200]!r}" if self._verbose_llm
            else f"✅ Turn done. Response: {spoken[:80]!r}")
    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._dsm.current_state.name
        self._state_pub.publish(msg)
    def _publish_response(self, text: str, animation: str = "neutral") -> None:
        msg = String()
        msg.data = build_ssml_payload(text, animation)
        self._response_pub.publish(msg)

    def _publish_music_cleanup(self, reason: str = "dialogue_end") -> None:
        """Issue #935 — signal mcp_server to auto-stop any active music.

        Best-effort: if the publisher was never created (e.g. mcp_server
        not running in this container), this is a silent no-op. mcp_server
        decides what to do — currently it logs and calls
        ``MusicManager.stop_music_on_session_end()``.
        """
        if getattr(self, "_music_cleanup_pub", None) is None:
            self.get_logger().debug("music_cleanup publisher not available")
            return
        try:
            payload = json.dumps({"reason": reason})
            msg = String()
            msg.data = payload
            self._music_cleanup_pub.publish(msg)
            self.get_logger().info(f"music_cleanup sent: reason={reason}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось опубликовать /mcp/music_cleanup: {exc}"
            )
    def _speak_direct(self, text: str) -> None:
        for chunk in split_into_chunks(text):
            self._publish_response(chunk)
    def _handle_silence(self) -> None:
        self._cancel_run("silence command")
        self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        self._publish_state()
        self._speak_direct("Хорошо, молчу.")
    def _cancel_run(self, reason: str) -> None:
        self._run_cancelled = True
        with self._task_lock:
            task = self._run_task
        if task is not None and not task.done():
            self.get_logger().info(f"🛑 Cancel: {reason}")
            self._loop.call_soon_threadsafe(task.cancel)
        stop_msg = String()
        stop_msg.data = "STOP"
        self._tts_control_pub.publish(stop_msg)
        self._effects.release_all_tts()
        self._effects.clear_sound_event()
    def _on_inactivity_check(self) -> None:
        if self._core.check_timeout():
            self.get_logger().info("⏰ Dialogue timeout → IDLE")
            self._publish_state()
    def shutdown_asyncio_loop(self, wait: bool = True) -> None:
        future = getattr(self, "_asyncio_loop_future", None)
        executor = getattr(self, "_asyncio_loop_executor", None)
        loop = getattr(self, "_loop", None)
        if future is None or executor is None or loop is None:
            return
        if future.done():
            executor.shutdown(wait=False)
            return
        try:
            loop.call_soon_threadsafe(loop.stop)
        except RuntimeError as exc:
            self.get_logger().warn(f"asyncio loop stop dispatch failed: {exc}")
        try:
            future.result(timeout=ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S)
        except concurrent.futures.TimeoutError:
            self.get_logger().warn(
                f"asyncio loop driver did not stop within "
                f"{ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S:.1f}s; cancelling")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"asyncio loop driver join raised: {exc}")
        finally:
            executor.shutdown(wait=False)
    def destroy_node(self) -> None:
        try:
            self.shutdown_asyncio_loop(wait=False)
        finally:
            super().destroy_node()

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DialogueNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    try:
        node.shutdown_asyncio_loop(wait=False)
    except Exception:  # noqa: BLE001
        logging.getLogger(__name__).exception("dialogue_node: shutdown failed")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
