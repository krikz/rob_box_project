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
import time
import uuid
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
    DEFAULT_BASE_URL as MINIMAX_DEFAULT_BASE_URL,
    DEFAULT_MODEL as MINIMAX_DEFAULT_MODEL,
    DEEPSEEK_DEFAULT_BASE_URL,
    DEEPSEEK_DEFAULT_MODEL,
    build_deepseek_provider,
    build_minimax_provider,
)
from rob_box_harness.tools import FakeToolProvider, ToolProvider

from rob_box_voice.core.dialogue_text import (
    has_wake_word, is_silence_command, is_unsilence_command, strip_wake_word,
)
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.speak_helpers import (
    EffectAwaiterRegistry, build_ssml_payload, split_into_chunks,
    strip_history_marker, strip_markdown, strip_thinking_blocks,
)

ASYNCIO_LOOP_DRIVER_MAX_WORKERS: int = 1
ASYNCIO_LOOP_DRIVER_NAME_PREFIX: str = "dialogue-async-loop"
ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S: float = 2.0

# Issue #992 Bug D — banned metalanguage openers. When the LLM returns
# plain text (no ``speak_text`` call) that begins with one of these
# phrases — e.g. "Зачитаю рэп про космос!", "Могу бит добавить,
# хочешь?", "Слушай, сейчас расскажу..." — the user hears a meta-
# promise instead of the actual performance. We catch that pattern at
# the dialogue_node boundary and force a single retry with a CRITICAL
# prompt-level reminder.
#
# The list is lowercase, comma/space separated, and checked via a
# substring match on the first ~80 chars of the LLM output (after
# strip_markdown). Add new phrases when the LLM invents a new opener;
# keep the list tight to avoid false positives on legitimate answers.
BABBLE_BANNED_OPENERS: tuple = (
    "зачит",   # зачитаю / зачитаем / зачитываю
    "погнали",  # погнали! / ну что, погнали?
    "могу ",    # могу бит добавить, могу спеть
    "хочешь",  # хочешь ещё? / хочешь послушать?
    "сейчас ",  # сейчас устроим / сейчас расскажу
    "устроим",  # устроим концерт / устроим вечеринку
    "давай-ка",  # давай-ка я спою
    "давай ",  # давай я / давай попробуем
    "слушай,",  # слушай, сейчас ...
    "слушай ",
    "окей, ",
    "окей ",
    "так, ",
    "так ",
    "ну что ж",
    "переключаюсь",
    "переключ",
)

# Issue #992 Bug D — keywords that mark the user request as a
# performance command. When the LLM babbles on a performance request
# we *must* retry, because the alternative is the user hearing nothing
# (the LLM promised but never spoke). When the user just asked a
# normal question and the LLM babbled, we still retry but the
# consequence is less severe — the user hears ONE meta-phrase instead
# of an answer. Keeping the heuristic narrow prevents false positives
# on ordinary chit-chat that happens to start with «слушай».
BABBLE_PERFORMANCE_KEYWORDS: tuple = (
    "рэп", "реп", "rap",
    "песн", "song", "песню", "песня",
    "стих", "стишок", "poem", "стихотворен",
    "зачитай", "прочитай", "прочти",
    "спой", "пой", "спела",
    "сыграй", "играй",
    "музык", "мелоди", "бит", "трек",
    "диджей", "dj ",
    "концерт",
    "джаз", "рок", "блюз", "частушки", "частушк",
)


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
            system_prompt=self._system_prompt,
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
        # Issue #980 — fire music_cleanup only after the *last* TTS chunk of a
        # batch (rap, poetry), not after the first. tts_node publishes this
        # event once ``batch_index == batch_total`` for a given ``batch_id``.
        self.create_subscription(
            String, "/voice/tts/batch_complete",
            self._on_tts_batch_complete, 10,
            callback_group=cbg)
        # Issue #992 — SpeakTextTool publishes this prelude BEFORE the first
        # TTS request of each ``speak_text`` call so we can pre-register the
        # ``batch_id``. Without it we only learn about a batch from the
        # first ``tts_finished`` for that batch — which works for a single
        # ``speak_text`` call but fails when the LLM fires two in a row:
        # batch #1's ``batch_complete`` would fire cleanup while batch #2 is
        # still pending. Payload: ``{"batch_id": str, "chunks_total": int}``.
        self.create_subscription(
            String, "/voice/tts/batch_registered",
            self._on_tts_batch_registered, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/voice/sound/state", self._on_sound_state, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/voice/dj_mode",
            lambda m: self._dj.handle_message(m.data), 10, callback_group=cbg)

        # Deferred music cleanup (issue #935 v2 → #980 → #992): music should
        # keep playing while TTS is still speaking (rap, poetry). Cleanup is
        # published strictly after the *last* TTS batch of a turn finishes
        # — see ``_on_tts_batch_complete``.
        #
        # Issue #992: a single ``_pending_music_cleanup`` boolean is not
        # enough when the LLM fires multiple ``speak_text`` tool calls
        # in one cycle (each call is a separate ``batch_id`` and produces
        # its own ``/voice/tts/batch_complete``). The first batch's
        # completion used to wipe music while the second batch was still
        # playing. ``_active_batches`` tracks every in-flight ``batch_id``
        # so we only fire cleanup when the registry is empty AND the
        # LLM actually asked to stop music.
        self._pending_music_cleanup: bool = False
        self._active_batches: Dict[str, int] = {}
        # Issue #992 Bug B — how many synchronous DJ retries have
        # already fired in the current transition. Caps the retry
        # loop so a stubborn LLM that keeps ignoring
        # ``execute_music_code`` doesn't lock the dialogue node in an
        # infinite coroutine chain. After ``MAX_DJ_AUTO_RETRIES`` the
        # cycle gives up and lets the normal 5 s tick take over.
        self._dj_auto_retry_count: int = 0
        self.MAX_DJ_AUTO_RETRIES: int = 2

        # Issue #992 Bug D — metalanguage / babble detector.
        # ``True`` after a single metalanguage retry has already been
        # dispatched for the current user turn. We only allow ONE
        # babble retry to avoid an infinite LLM ping-pong; if the LLM
        # babbles again after the retry, we fall through to publish the
        # meta-text verbatim and let the operator debug from logs.
        self._babble_retry_used: bool = False

        # Issue #992 Bug A — DJ auto-transitions MUST NOT take the
        # ``new_dialogue`` cleanup path. Wrapping ``_dispatch_turn`` here
        # sets ``is_dj_auto=True`` so the dispatcher skips
        # ``_publish_music_cleanup(reason="new_dialogue")`` and the
        # barge-in cancel that would otherwise wipe an in-flight
        # track and feed the LLM an empty input.
        self._dj = DJModeController(
            hook=DJHook(
                dispatch=self._dispatch_dj_turn,
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
        self.declare_parameter("llm_provider", "minimax")
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
                f"✅ Prompt loaded: {prompt_file} ({len(prompt)} bytes) "
                f"from {os.path.join(pkg, 'prompts', prompt_file)}\n"
                f"   first line: {prompt.split(chr(10))[0][:120]!r}")
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

        # Resolve the endpoint here instead of relying on SDK defaults.  This
        # makes the production route visible and testable at the ROS shell
        # boundary, where YAML parameters enter the application.
        base_url = str(self.get_parameter("base_url").value or "").strip()
        model = str(self.get_parameter("model").value or "").strip()

        if provider_name == "minimax":
            # 🔴 FIX (live 14:39): build_minimax_provider принимает
            # LLMConfig, а не kwargs (api_key=...) как deepseek. Коммит
            # 7d3e95c9 скопировал deepseek-стиль → TypeError при старте →
            # dialogue_node падал → робот молчал. Собираем LLMConfig.
            from rob_box_harness.config import LLMConfig

            return build_minimax_provider(
                LLMConfig(
                    provider="minimax",
                    model=model or MINIMAX_DEFAULT_MODEL,
                    api_key=self.get_parameter("api_key").value or None,
                    timeout_s=90.0,
                )
            )
        if provider_name == "deepseek":
            return build_deepseek_provider(
                api_key=self.get_parameter("api_key").value or None,
                base_url=base_url or DEEPSEEK_DEFAULT_BASE_URL,
                model=model or DEEPSEEK_DEFAULT_MODEL,
            )

        raise ValueError(
            f"Unsupported llm_provider={provider_name!r}; "
            "supported: 'minimax', 'deepseek'"
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
        # Issue 989 Fix A: dialogue_node НЕ должен реагировать на
        # rejected(empty) — это эхо собственной музыки/голоса, а не речь
        # пользователя. Защита на случай, если stt_node начнёт публиковать
        # маркеры отклонения в /voice/stt/result (сейчас он публикует только
        # accepted, но guard дешёвый и страхует от регрессий).
        if text_lower.startswith(("rejected", "«rejected", "empty", "«пусто", "тишина")):
            self.get_logger().info(f"🔇 [issue 989] Игнор rejected/empty маркера: {text[:60]}")
            return
        state = self._dsm.current_state
        was_idle = state == DialogueStateKind.IDLE  # FIX #992: для music_cleanup new_dialogue
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
        raw_user_command = clean
        if self._dj.state.enabled:
            clean = self._dj.preamble() + clean
        if self._verbose_llm:
            self.get_logger().info(f"📥 LLM INPUT: {clean[:200]!r}")
        # 🔴 FIX (live 12:45): Bug C guard должен смотреть ТОЛЬКО оригинальную
        # команду юзера, а не текст с DJ-preamble. Preamble содержит
        # «диджей: ...» — guard видел его и думал «юзер просит музыку»,
        # нудил Bug C и LLM начинала DJ-сессию вместо анекдота.
        self._dispatch_turn(clean, was_idle=was_idle, raw_user_command=raw_user_command)
    def _on_tts_finished(self, msg: String) -> None:
        """Awaiter-release only — cleanup moved to ``_on_tts_batch_complete``.

        Issue #980: firing ``music_cleanup`` from the *first* ``/voice/tts/finished``
        event of a multi-chunk turn cut the playback short (e.g. a 35 s rap
        only played 10 s). We now trigger cleanup exclusively from
        ``/voice/tts/batch_complete`` which fires once after the last chunk.

        Issue #992: register the in-flight ``batch_id`` here (idempotent) so
        ``_on_tts_batch_complete`` can know which batches are still active
        when the LLM fires multiple ``speak_text`` calls in one cycle. The
        first chunk of each batch is enough to register — subsequent chunks
        are no-ops because ``batch_id`` is already in ``_active_batches``.
        """
        self._effects.handle_tts_finished(msg.data or "")
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        batch_id = payload.get("batch_id")
        batch_total = payload.get("batch_total")
        if batch_id is None or batch_total is None:
            return
        try:
            self._register_active_batch(str(batch_id), int(batch_total))
        except (TypeError, ValueError):
            # Defensive: malformed batch metadata must not break the
            # awaiter-release path or the batch tracking.
            self.get_logger().debug(
                f"⚠️ [issue 992] Skipping batch registration: "
                f"batch_id={batch_id!r}, batch_total={batch_total!r}"
            )

    def _on_tts_batch_registered(self, msg: String) -> None:
        """Pre-register an in-flight TTS batch (issue #992).

        SpeakTextTool publishes this BEFORE the first ``/voice/tts/request``
        of each ``speak_text`` call so we can lock the batch_id into
        ``_active_batches`` up front. This is what makes multi-speak_text
        cycles work: each ``speak_text`` registers its own batch, the last
        ``batch_complete`` unregisters the last one, and only then do we
        fire ``/mcp/music_cleanup``.

        The registry is also updated by ``_on_tts_finished`` as a
        fallback — this prelude just makes registration *eager* so
        cleanup is correctly held across multiple batches.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        batch_id = payload.get("batch_id")
        chunks_total = payload.get("chunks_total")
        if batch_id is None or chunks_total is None:
            return
        try:
            self._register_active_batch(str(batch_id), int(chunks_total))
        except (TypeError, ValueError):
            self.get_logger().debug(
                f"⚠️ [issue 992] Skipping batch_registered: "
                f"batch_id={batch_id!r}, chunks_total={chunks_total!r}"
            )

    def _register_active_batch(self, batch_id: str, chunks_total: int) -> None:
        """Track an in-flight TTS batch for music-cleanup gating (issue #992).

        Idempotent: re-registering an already-known ``batch_id`` is a no-op
        so the second/third chunk's ``tts_finished`` doesn't double-count.
        """
        if batch_id in self._active_batches:
            return
        self._active_batches[batch_id] = chunks_total
        self.get_logger().debug(
            f"🎵 [issue 992] Registered active batch {batch_id[:8]}... "
            f"(chunks_total={chunks_total}, "
            f"now active={len(self._active_batches)})"
        )

    def _unregister_active_batch(self, batch_id: str) -> None:
        """Drop a batch from the in-flight registry (issue #992)."""
        removed = self._active_batches.pop(batch_id, None)
        if removed is None:
            return
        self.get_logger().debug(
            f"🎵 [issue 992] Unregistered batch {batch_id[:8]}... "
            f"(remaining={len(self._active_batches)})"
        )

    def _on_tts_batch_complete(self, msg: String) -> None:
        """Fire ``music_cleanup`` once *all* TTS batches have finished (issue #992).

        ``tts_node`` publishes this after the chunk whose ``batch_index ==
        batch_total`` lands on ``/voice/tts/finished`` (success *or* failure).
        Single-chunk turns still produce exactly one ``batch_complete`` so the
        previous behaviour is preserved.

        Issue #992: when the LLM calls ``speak_text`` more than once in a
        single cycle, each call creates a *separate* ``batch_id`` and
        therefore a *separate* ``/voice/tts/batch_complete`` event. The
        previous implementation fired ``/mcp/music_cleanup`` on the first
        event, which cut music off while the second batch was still
        playing. We now unregister the completed batch from
        ``_active_batches`` and only fire cleanup when the registry is
        empty AND ``_pending_music_cleanup`` is set.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            payload = {}
        batch_id_raw = payload.get("batch_id")
        chunks_total = payload.get("chunks_total")
        batch_id = str(batch_id_raw) if batch_id_raw is not None else None
        if batch_id is not None:
            self._unregister_active_batch(batch_id)
        if self._active_batches:
            # At least one batch is still in flight — hold the cleanup
            # until that one (and any others) finish too.
            self.get_logger().debug(
                f"🎵 [issue 992] batch_complete for "
                f"{batch_id[:8] if batch_id else 'None'}... but "
                f"{len(self._active_batches)} batch(es) still active — "
                f"deferring music_cleanup"
            )
            return
        # 🔴 FIX (issue 992 live 09:09): LLM часто делает speak_text(прелюдия)
        # ПЕРЕД execute_music_code в одном цикле. Прелюдия — короткий batch
        # (2-5s), его batch_complete приходит ПОКА цикл LLM ещё крутится
        # (дальше Мурка/зайчики). Если чистить здесь — музыка убивается
        # через секунды после старта. Откладываем cleanup, пока _run_turn
        # активен: настоящий cleanup придёт от последнего batch_complete
        # (или 10s fallback), когда цикл завершится.
        run_task = self._run_task
        if run_task is not None and not run_task.done():
            self.get_logger().debug(
                f"🎵 [issue 992] batch_complete for {batch_id[:8] if batch_id else 'None'}... "
                "but LLM turn still active — deferring music_cleanup "
                "(prelude/pre-talk batch inside a running cycle)"
            )
            return
        if not self._pending_music_cleanup:
            self.get_logger().debug(
                "tts_batch_complete received but no pending music_cleanup"
            )
            return
        self._pending_music_cleanup = False
        self._publish_music_cleanup(reason="tts_batch_complete")
        self.get_logger().info(
            "🎵 tts_batch_complete fired music_cleanup "
            f"(chunks_total={chunks_total}, last_batch={batch_id[:8] if batch_id else 'None'}...)"
        )
    def _on_sound_state(self, msg: String) -> None:
        self._effects.handle_sound_state(msg.data or "")
    def _dispatch_dj_turn(self, user_input: str, from_tick: bool = False) -> None:
        """Issue #992 Bug A — DJ auto-transition dispatcher.

        Behaves like :meth:`_dispatch_turn` but:

        * does NOT publish ``music_cleanup`` with ``reason="new_dialogue"`` —
          a DJ tick is an autonomous transition, not a fresh user dialogue.
          Wiping the active track on every transition would either cut the
          music mid-phrase (no execute_music_code in this cycle) or feed
          the LLM an empty input and trip the
          "Что-то я задумался, повтори пожалуйста" fallback.
        * does NOT cancel an in-flight ``_run_task`` — DJ transitions
          compose with the existing DJ cycle instead of barging in. If
          the current task is still running (e.g. the previous DJ turn's
          LLM call hasn't returned yet) we simply queue the next tick;
          :meth:`DJModeController.tick` already defers by 15 s when a
          dialogue is active, so collisions are rare and safe.
        * forwards ``is_dj_auto=True`` so :meth:`_run_turn` applies
          DJ-specific post-turn guards (Bug B + Bug C). The DJ flag is
          threaded through the parameter rather than stored on ``self``
          to avoid a race where :meth:`_apply_music_guard`'s synchronous
          retry is dispatched *before* the next turn's ``_run_turn`` reads
          the flag — without this, a retry launched from inside the
          ``finally`` block would observe the parent's already-cleared
          flag and silently lose the ``was_dj_auto=True`` semantics.

        ``from_tick`` distinguishes a fresh tick transition (resets the
        Bug-B retry budget) from a synchronous retry triggered by the
        music-guard in :meth:`_run_turn` (keeps the budget intact).
        """
        if from_tick:
            self._dj_auto_retry_count = 0
        self._dispatch_turn(user_input, is_dj_auto=True)

    def _dispatch_turn(
        self,
        user_input: str,
        is_dj_auto: bool = False,
        was_idle: bool = False,
        is_babble_retry: bool = False,
        raw_user_command: str | None = None,
    ) -> None:
        # Issue #992 Bug A — DJ auto-transitions must NOT publish
        # ``music_cleanup`` with ``reason="new_dialogue"``. Without this
        # guard the LLM cycle is reset mid-track, which in turn trips the
        # empty-assistant fallback ("Что-то я задумался, повтори
        # пожалуйста") and makes the robot sound broken during a DJ
        # session. Bug C guards against the complementary failure mode
        # (LLM skips execute_music_code).
        if is_dj_auto:
            self.get_logger().debug(
                "🎧 [issue 992] DJ auto-transition — skipping "
                "new_dialogue music_cleanup"
            )
        elif self._pending_music_cleanup and was_idle:
            # 🔴 FIX (issue 992 live 08:55): music_cleanup reason=new_dialogue
            # ТОЛЬКО при новом диалоге из IDLE. Продолжение диалога
            # (barge-in: «ещё спой про зайчиков» во время ответа) — это
            # тот же диалог: музыка живёт до tts_batch_complete (#968),
            # LLM сам сменит/оставит трек через execute_music_code.
            # Раньше cleanup слался на ЛЮБОЙ STT → музыка убивалась
            # в момент продолжения (v08:55:11 music_cleanup new_dialogue).
            self._pending_music_cleanup = False
            self._publish_music_cleanup(reason="new_dialogue")
        asyncio.run_coroutine_threadsafe(
            self._run_turn(
                user_input,
                is_dj_auto=is_dj_auto,
                is_babble_retry=is_babble_retry,
                raw_user_command=raw_user_command,
            ),
            self._loop,
        )

    async def _run_turn(
        self,
        user_input: str,
        *,
        is_dj_auto: bool = False,
        is_babble_retry: bool = False,
        raw_user_command: str | None = None,
    ) -> None:
        with self._task_lock:
            self._run_task = asyncio.current_task()
        self._run_cancelled = False
        # Issue #992 Bug B / Bug C — ``is_dj_auto`` is threaded through
        # ``_dispatch_turn`` rather than read from ``self`` so a
        # synchronous DJ retry dispatched from inside this turn's
        # ``finally`` block does not race with the parent's flag reset.
        was_dj_auto = is_dj_auto
        # Issue #992 Bug D — reset the babble-retry budget only at the
        # TOP of a *user-initiated* turn. When ``is_babble_retry=True``
        # we are inside the LLM-triggered follow-up dispatched by
        # :meth:`_check_babble_and_retry`, and the flag MUST stay True
        # so a still-babbling retry response is not escalated to a
        # second retry (which would loop forever).
        if not is_babble_retry:
            self._babble_retry_used = False
        # Issue #992 Bug D — when the babble detector schedules a retry
        # we MUST NOT end the dialogue at the bottom of this turn. The
        # retry's ``_run_turn`` will run on the same DSM session and
        # needs to find DIALOGUE (or at least LISTENING) state so the
        # wake-word classifier short-circuits to STT_RESULT and the
        # LLM gate fires. Without this guard the first turn's
        # unconditional ``DIALOGUE_END`` drops the DSM back to IDLE,
        # the retry's ``process_input`` classifies the synthetic
        # prompt as STT_RESULT but stays in IDLE (no-op), and the
        # LLM is never called — the user hears nothing.
        babble_retry_pending = False
        try:
            # Issue #992 — DJ auto-turns must bypass the wake-word
            # classifier so the LLM is actually called. The DJ prompt
            # intentionally mentions "роббокс" / "диджей" which would
            # otherwise short-circuit into a no-op transition.
            result: DialogResult = await self._core.process_input(
                user_input, is_dj_auto=was_dj_auto,
            )
            self._handle_result(result, user_input=user_input)
            babble_retry_pending = bool(self._babble_retry_used)
        except asyncio.CancelledError:
            self.get_logger().info("🛑 Turn cancelled (barge-in)")
            result = None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"❌ DialogCore error: {exc}")
            self._speak_direct("Что-то я задумался, повтори пожалуйста")
            result = None
        finally:
            # Issue #992 Bug B: ``_apply_music_guard`` may synchronously
            # dispatch a follow-up DJ turn, which sets ``self._run_task``
            # to a fresh coroutine. We must not wipe that reference here
            # or the test driver (and any future hook that watches
            # ``self._run_task``) would lose the retry. Only clear the
            # slot when no replacement was scheduled.
            with self._task_lock:
                if self._run_task is asyncio.current_task():
                    self._run_task = None
            # Issue #935 v3: if LLM called stop_music(), defer cleanup until
            # TTS finishes.  Otherwise keep music playing until next dialogue.
            # Issue #992: a second stop_music() call from a follow-up LLM
            # turn (while a previous cleanup is still pending) must be
            # ignored — the flag is already set and the next batch_complete
            # for any active batch will fire cleanup.
            if result and "stop_music" in (result.tools_called or ()):
                if self._pending_music_cleanup:
                    self.get_logger().debug(
                        "🎵 [issue 992] stop_music deferred — already pending, "
                        "ignoring duplicate"
                    )
                else:
                    self._pending_music_cleanup = True
                    self.get_logger().info(
                        "🎵 stop_music deferred — will cleanup after TTS finishes"
                    )
            else:
                # 🔴 FIX (live 09:35): если LLM в этом цикле САМ запустила
                # музыку (execute_music_code) — НЕ убивать её по
                # tts_batch_complete короткой прелюдии («Слушай Баха!»).
                # Музыка, запущенная как композиция, живёт до segments
                # или явного stop_music. Cleanup — только если музыка
                # НЕ запускалась в этом цикле (осталась от прошлого).
                tools_now = set(result.tools_called or ())
                if "execute_music_code" in tools_now:
                    if self._pending_music_cleanup:
                        self._pending_music_cleanup = False
                        self.get_logger().info(
                            "🎵 [issue 992] LLM restarted music via "
                            "execute_music_code — cancelled pending cleanup"
                        )
                    else:
                        self.get_logger().debug(
                            "🎵 [issue 992] LLM started music — no cleanup "
                            "scheduled for this turn"
                        )
                elif not self._pending_music_cleanup:
                    self._pending_music_cleanup = True
                    self.get_logger().info(
                        "🎵 music_cleanup deferred — waiting for TTS or 10s fallback"
                    )
                else:
                    self.get_logger().debug(
                        "🎵 [issue 992] music_cleanup already pending — "
                        "ignoring redundant re-arm"
                    )
            if self._pending_music_cleanup and not self._active_batches:
                self._pending_music_cleanup = False
                self._publish_music_cleanup(reason="tts_batch_complete")
                self.get_logger().info(
                    "🎵 turn finished, no active batches — fired music_cleanup "
                    "(issue 992 prelude-deferral catch-up)"
                )
            # Issue #992 Bug D — defer the DIALOGUE_END transition
            # when the babble detector scheduled a retry. The retry's
            # ``_run_turn`` needs the DSM to stay in DIALOGUE so the
            # LLM gate fires; otherwise the synthetic prompt is
            # classified as STT_RESULT but the state stays in IDLE
            # (no-op), and the user never hears the retry answer.
            if (
                self._dsm.current_state == DialogueStateKind.DIALOGUE
                and not babble_retry_pending
            ):
                self._dsm.on_event(DialogueEvent.DIALOGUE_END)
            # DialogCore completes the DIALOGUE → IDLE transition itself.
            # Publish the resulting state even when no transition is needed
            # here; otherwise the ROS state topic remains stuck at the
            # earlier DIALOGUE notification and scenario runners wait forever.
            self._publish_state()
            # Issue #992 Bug B / Bug C — DJ-mode post-turn guard.
            # ``is_dj_auto`` was threaded through the dispatch path so no
            # shared flag needs to be cleared here. The guard may
            # synchronously dispatch a follow-up DJ turn while we are
            # still inside this turn's ``finally``; that is intentional —
            # ``drive_one_turn`` (and any production loop) drains the
            # retry as part of this very turn cycle.
            self._apply_music_guard(
                was_dj_auto=was_dj_auto,
                user_input=raw_user_command or user_input,
                tools_called=result.tools_called if result else (),
            )

    # ── Issue #992 Bug B / Bug C — DJ-mode music guard ────────────────

    # Issue #992 Bug C — narrow keyword heuristic. ``трек`` and ``бит``
    # are deliberately excluded because they fire on chit-chat like
    # "роббокс какой трек посоветуешь?" (issue 992 test_user_normal_chat
    # regression). Keep the list focused on unambiguous "play something
    # NOW" commands so the spoken nudge only fires when the user clearly
    # asked for generated music.
    _MUSIC_GUARD_KEYWORDS = (
        "спой",
        "пой ",
        "рэп",
        "рап",
        "диджей",
        "dj ",
        "dj-",
        "песня",
        "песню",
        "зачитай",
        "зачита",
        "зачитывай",
    )

    # 🔴 FIX (live 10:00): для ГОЛОСОВЫХ запросов («спой/пой/песня»)
    # speak_text достаточно — бит не обязателен (юзер мог попросить
    # спеть ПОД уже играющую музыку, как «спой про мурку в этот
    # момент» — Григ играл, LLM правильно не перезапустила трек).
    # Bug C нудит только если LLM вообще НИЧЕГО не сделала (tools
    # пуст). Для БИТО-обязательных («рэп/зачитай/диджей») — как было:
    # нуднуть если нет execute_music_code.
    _MUSIC_GUARD_VOCAL_KEYWORDS = (
        "спой",
        "пой ",
        "песня",
        "песню",
    )

    def _user_wants_music(self, user_input: str) -> bool:
        """Heuristic: does the user request music / a track?

        Used by :meth:`_apply_music_guard` to decide whether Bug C's
        code-side fallback should fire. The check is intentionally
        narrow so we don't retry on ordinary chit-chat that happens
        to mention "track" in passing.
        """
        if not user_input:
            return False
        low = user_input.lower()
        return any(kw in low for kw in self._MUSIC_GUARD_KEYWORDS)

    # ── Issue #992 Bug D — metalanguage / babble detection ───────────

    def _is_metalanguage_babble(self, spoken_text: str) -> bool:
        """Issue #992 Bug D — does this LLM output read as meta-talk?

        Returns ``True`` when the LLM final response text starts with a
        known metalanguage opener («зачита», «могу», «хочешь»,
        «сейчас», «устроим», «погнали», «давай», «слушай», «окей»,
        «так», «переключ», «ну что ж»). The check operates on the
        first 80 chars after :func:`strip_markdown` so a lone "**"
        that survived cleaning cannot mask the opener.

        The detector is intentionally *conservative*: a normal answer
        that happens to contain «слушай» somewhere in the middle is
        safe — only the first 80 chars are inspected. When in doubt,
        return ``False``; :meth:`_check_babble_and_retry` will fall
        through to the standard TTS publish path.
        """
        if not spoken_text:
            return False
        head = spoken_text[:80].lower().lstrip(" \t*#>-")
        # Match the opener only at the START of the head or inside the
        # first 30 chars (after stripping). 30 chars is enough to cover
        # «Слушай, сейчас расскажу...» but short enough to skip
        # legitimate mid-sentence uses like «Если хочешь, могу
        # остановиться» or «А сейчас продолжу маршрут».
        opener_zone = head[:30]
        return any(
            opener_zone.startswith(opener) or f" {opener}" in opener_zone
            for opener in BABBLE_BANNED_OPENERS
        )

    def _user_wants_performance(self, user_input: str) -> bool:
        """Issue #992 Bug D — does the user request a *performance*?

        Used to decide whether a metalanguage reply is a hard bug
        (user asked for a rap, robot returned "Зачитаю рэп про X!") or
        just a stylistic miss (user asked "что нового?", robot replied
        "Слушай, у меня тут..." — still answer-shaped, just informal).
        """
        if not user_input:
            return False
        low = user_input.lower()
        return any(kw in low for kw in BABBLE_PERFORMANCE_KEYWORDS)

    def _check_babble_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
    ) -> bool:
        """Issue #992 Bug D — single-shot babble retry dispatcher.

        Inspects ``spoken`` (the LLM final text after strip_markdown)
        and decides whether to force ONE retry with a CRITICAL
        reminder. Returns ``True`` when a retry was scheduled (so the
        caller should skip the regular TTS publish), ``False`` when the
        detector passed and the caller should proceed normally.

        Retry rules — all must hold for a retry to fire:
        1. ``speak_text`` was NOT called this cycle (already handled
           by the anti-duplicate path otherwise).
        2. ``spoken`` is non-empty and starts with a banned opener
           (see :data:`BABBLE_BANNED_OPENERS`).
        3. The user request looks like a performance command OR the
           babble phrase is unambiguously a "promise to do" — i.e. the
           opener is in the *promise* subset («зачит», «погнали»,
           «устроим», «переключ», «давай-ка»). Other openers
           («слушай,», «окей,», «так,») only retry when the user
           explicitly asked for a performance.
        4. We have NOT already used our one-shot babble retry for
           this turn (avoids an infinite LLM ping-pong).
        """
        if "speak_text" in (tools_called or ()):
            return False
        if not spoken:
            return False
        if self._babble_retry_used:
            return False
        if not self._is_metalanguage_babble(spoken):
            return False
        user_wants_perf = self._user_wants_performance(user_input or "")
        if not user_wants_perf:
            # The promise-only subset always retries regardless of
            # user_input — these phrases are NEVER valid answers.
            promise_only = (
                "зачит", "погнали", "устроим",
                "переключ", "давай-ка",
            )
            head = spoken[:60].lower()
            if not any(p in head for p in promise_only):
                return False
        # Issue #992 Bug D — the retry turn needs the same DSM state
        # transitions as a real STT input (IDLE → LISTENING → DIALOGUE)
        # — otherwise DialogCore's process_input sees IDLE and returns
        # an empty result, which trips the
        # "Что-то я задумался, повтори пожалуйста" fallback. This is
        # exactly the wake-word gate logic from ``_on_stt``.
        try:
            from rob_box_harness.core.dialogue_state_machine import (
                DialogueEvent as _DE,
                DialogueStateKind as _DSK,
            )
            if self._dsm.current_state == _DSK.IDLE:
                self._dsm.on_event(_DE.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(_DE.STT_RESULT)
            self._publish_state()
        except ImportError:
            # dialog_state_machine is part of rob_box_harness; if it
            # ever disappears the safe default is to skip the
            # transition and let process_input return an empty result.
            pass
        # Mark the retry as used BEFORE dispatching so a re-entrant
        # call from the retry itself can never escalate to a second
        # retry.
        self._babble_retry_used = True
        retry_prompt = self._build_babble_retry_prompt(user_input or "")
        self.get_logger().warning(
            "🗣️ [issue 992 Bug D] LLM babble detected — retrying once with "
            f"CRITICAL reminder (head={spoken[:60]!r})"
        )
        self._dispatch_turn(retry_prompt, is_babble_retry=True)
        return True

    def _build_babble_retry_prompt(self, user_input: str) -> str:
        """Issue #992 Bug D — synthetic follow-up prompt for babble retry.

        Echoes the original ``user_input`` so the LLM has the request
        in context, then appends a CRITICAL instruction that names the
        babble pattern and demands a tool-call reply (no plain text
        promises).
        """
        return (
            f"{user_input}\n\n"
            "[CRITICAL] Твой предыдущий ответ был метатекст "
            "(начинался с «зачит», «могу», «хочешь», «сейчас», "
            "«устроим», «погнали», «слушай», «давай», «так» или "
            "«переключ») — пользователь слышит пустую болтовню "
            "вместо результата.\n"
            "❌ ЗАПРЕЩЕНО отвечать текстом-обещанием. "
            "✅ ОБЯЗАТЕЛЬНО: вызови нужный tool в ЭТОМ же turn:\n"
            "  • rap/песня → execute_music_code + speak_text(lyrics),\n"
            "  • поэзия → speak_text(...) × N строк,\n"
            "  • мелодия → execute_music_code(...),\n"
            "  • анекдот → speak_text(...) × N.\n"
            "После последнего speak_text верни 'done'. Никаких "
            "мета-фраз, никаких 'Слушай, сейчас...', 'Зачитаю...', "
            "'Могу бит добавить, хочешь?' — это BUG."
        )

    def _apply_music_guard(
        self,
        *,
        was_dj_auto: bool,
        user_input: str,
        tools_called: tuple,
    ) -> None:
        """Post-turn guard that catches LLM music-skip regressions.

        Issue #992 Bug B — DJ auto-transitions: the LLM was told
        ``Сыграй трек #N через handle_music``, but it frequently
        replies with just a spoken phrase and no ``execute_music_code``
        tool call. Without this guard the DJ cycle silently produces
        zero audio for that transition. We re-arm
        ``next_transition_at`` to ``now + POSTPONE_INTERVAL_S`` so the
        next tick fires shortly, AND publish a one-shot synthetic
        prompt that injects a CRITICAL reminder to call the tool.

        Issue #992 Bug C — user rap/song commands: even outside DJ
        mode, when the user asks for a rap/poem/song and the LLM
        forgets ``execute_music_code``, we publish a short spoken
        acknowledgment so the user hears *something* and can repeat
        the request. We deliberately do NOT auto-pick a beat without
        user consent: that would surprise the operator.
        """
        tools_set = set(tools_called or ())
        if "execute_music_code" in tools_set:
            # Reset the retry counter on success so a future failure
            # gets a fresh budget.
            self._dj_auto_retry_count = 0
            return
        if was_dj_auto and self._dj.state.enabled:
            # Bug B — DJ was supposed to play but didn't.
            if self._dj_auto_retry_count >= self.MAX_DJ_AUTO_RETRIES:
                self.get_logger().warning(
                    "🎵 [issue 992 Bug B] DJ retry budget exhausted "
                    f"({self._dj_auto_retry_count}/{self.MAX_DJ_AUTO_RETRIES}); "
                    "letting 5 s tick take over"
                )
                self._dj_auto_retry_count = 0
                return
            self._dj_auto_retry_count += 1
            self.get_logger().warning(
                "🎵 [issue 992 Bug B] DJ auto-transition completed "
                "without execute_music_code — forcing synchronous retry "
                f"({self._dj_auto_retry_count}/{self.MAX_DJ_AUTO_RETRIES})"
            )
            self._dj.state.next_transition_at = (
                time.time() + DJModeController.POSTPONE_INTERVAL_S
            )
            self._dispatch_dj_turn(self._build_dj_retry_prompt())
            return
        if self._user_wants_music(user_input):
            tools_now = set(tools_called or ())
            # 🔴 FIX (live 10:00): вокальные запросы («спой/пой/песня»)
            # — speak_text уже есть (песня озвучена), бит не обязателен:
            # не нудить. Только если LLM вообще ничего не сделала
            # (tools пуст) — это настоящий пропуск.
            if any(kw in user_input.lower() for kw in self._MUSIC_GUARD_VOCAL_KEYWORDS):
                if tools_now:
                    self.get_logger().debug(
                        "🎵 [issue 992 Bug C] vocal request, LLM replied "
                        f"(tools={sorted(tools_now)!r}) — no nudge needed"
                    )
                    return
            # Bug C — user asked for rap/song/DJ but LLM skipped music.
            self.get_logger().warning(
                "🎵 [issue 992 Bug C] user asked for music but LLM "
                f"skipped execute_music_code (tools={sorted(tools_now)!r}); "
                "publishing spoken nudge"
            )
            self._speak_direct(
                "Я тут растерялся — бит не запустился, попробуй ещё раз."
            )

    def _build_dj_retry_prompt(self) -> str:
        """Synthetic auto-prompt for the Bug-B synchronous retry.

        Re-uses the persona/theme context from
        :class:`DJModeController` but appends a CRITICAL reminder —
        the LLM has ignored the standard auto-prompt at least once,
        so this retry escalates the instruction with an explicit tool
        name and a no-tools rejection clause.
        """
        n = self._dj.state.transition_count
        base = self._dj.build_auto_prompt(n)
        return (
            base
            + "\n\n[CRITICAL] В прошлом цикле ты НЕ вызвал "
            "execute_music_code — DJ-режим остался без музыки. "
            "В этом цикле ОБЯЗАТЕЛЬНО вызови execute_music_code "
            "(Renardo code) ДО любого speak_text. Если ты снова "
            "не вызовешь execute_music_code, цикл будет считаться "
            "пустым и робот озвучит 'задумался'. Никаких tools_calls, "
            "кроме execute_music_code и speak_text, здесь не нужно."
        )
    def _handle_result(
        self,
        result: DialogResult,
        user_input: Optional[str] = None,
    ) -> None:
        """Publish (or swallow) the LLM response for one turn.

        ``user_input`` is threaded through from :meth:`_run_turn` so
        :meth:`_check_babble_and_retry` can correlate the LLM reply
        with the user request when deciding whether the response
        reads as metalanguage / babble (issue #992 Bug D). The
        parameter is optional for backwards compatibility with any
        external caller that bypasses :meth:`_run_turn`; when
        ``None`` the babble-retry path falls back to "promise-only"
        detection which is still safe (no false positives on
        ordinary answers).
        """
        if result.error is not None:
            self.get_logger().warning(f"⚠️ DialogCore error: {result.error}")
        spoken = strip_history_marker(result.spoken_text or "")
        # 🔴 FIX (live 16:02 «английская мысль на Бахе»): MiniMax M3
        # возвращает ``<think>...</think>`` в content перед реальным
        # ответом. Done-чекер ниже ловит «done», но если перед ним лежит
        # английский мысленный комментарий («Music started successfully.
        # Now return "done" — no speak_text...»), он весь уходит в TTS
        # как «озвучка ответа». Strip-блоков ДО done-чекера → в TTS идёт
        # либо пусто (маркер done → тишина), либо реальный финал.
        spoken = strip_thinking_blocks(spoken)
        # Issue #988 (code part): strip Markdown BEFORE chunking. Chunking
        # splits on punctuation, which can cut a paired "*...*" in half;
        # strip_markdown in tts_node only removes *paired* delimiters, so a
        # lone "*" survives and TTS reads it as «звёздочка». Sanitising here
        # (before split_into_chunks) fixes that. Applies to the auto-voice
        # path AND the speak_text path (which also passes through chunking).
        spoken = strip_markdown(spoken)
        # 🔴 FIX (live 09:35): LLM-маркер завершения «done» (и вариации)
        # НЕ должен озвучиваться буквально («дан»). Это происходит когда
        # LLM завершила цикл без speak_text (например повтор «сыграй баха»
        # после уже запущенной музыки → ответил просто done).
        _done_marker = spoken.strip().lower()
        if _done_marker in ("done", "task complete", "task_complete", "готово", "всё", "выполнено"):
            self.get_logger().info(
                f"🔇 LLM completion marker — skip auto-TTS: {spoken[:60]!r}"
            )
            spoken = ""
        tools_called = tuple(result.tools_called or ())
        # Issue #988 — anti-duplicate: when the LLM already called
        # ``speak_text`` during this cycle, the answer (song / poem /
        # phrase) was voiced directly by the MCP tool via
        # ``/voice/tts/request``. Auto-voicing ``result.spoken_text``
        # here would play the same content a second time (E2E v44:
        # песня читается дважды). Per the master-prompt contract the
        # final text after the LAST speak_text is expected to be "done"
        # (or empty), so we skip the TTS publish entirely and only log
        # the LLM output for debugging.
        if "speak_text" in tools_called:
            if self._verbose_llm:
                self.get_logger().info(
                    f"🔇 [issue 988] speak_text called in cycle — "
                    f"skipping auto-TTS of final text: {spoken[:200]!r}"
                )
            else:
                self.get_logger().info(
                    f"🔇 [issue 988] speak_text called — final text skipped "
                    f"(anti-duplicate): {spoken[:80]!r}"
                )
            return
        # Issue #992 Bug D — metalanguage / babble detector. Fires ONE
        # synchronous retry with a CRITICAL prompt reminder when the
        # LLM replied with meta-talk instead of performing the request.
        # Returns ``True`` when a retry was scheduled; in that case we
        # MUST NOT publish the meta-text to TTS — otherwise the user
        # would hear the babble AND then the retry answer.
        if spoken and self._check_babble_and_retry(
            spoken=spoken,
            user_input=user_input,
            tools_called=tools_called,
        ):
            return
        if not spoken:
            # 🔴 FIX (live 10:49): пустой spoken НЕ всегда ошибка — LLM
            # могла выполнить TRACK-запрос («сыграй баха») через
            # execute_music_code и вернуть 'done' без speak_text (новый
            # TRACK-контракт: музыка без болтовни). Мой done-strip
            # обнулил spoken → «задумался» был ЛОЖНЫМ. Fallback — только
            # если LLM вообще ничего не сделала (tools пуст).
            if not tools_called:
                self.get_logger().warning("⚠️ Empty assistant response — fallback")
                spoken = "Что-то я задумался, повтори пожалуйста"
            else:
                self.get_logger().info(
                    "🔇 TRACK-запрос выполнен тулами, spoken пуст — "
                    f"тихо завершаю (tools={list(tools_called)!r})"
                )
                return
        # Issue #980 — split into chunks and publish as a single TTS batch so
        # that /voice/tts/batch_complete fires only after the last chunk.
        chunks = split_into_chunks(spoken)
        if not chunks:
            self._publish_response(spoken)
        elif len(chunks) == 1:
            self._publish_response(chunks[0])
        else:
            total = self._publish_response_batch(chunks)
            self.get_logger().info(
                f"📦 [dialogue_node] TTS batch: {total} chunks (issue #980)"
            )
        self.get_logger().info(
            f"📤 LLM OUTPUT: {spoken[:200]!r}" if self._verbose_llm
            else f"✅ Turn done. Response: {spoken[:80]!r}")
    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._dsm.current_state.name
        self._state_pub.publish(msg)
    def _publish_response(self, text: str, animation: str = "neutral") -> None:
        """Single-chunk publish — kept for backwards compatibility.

        For multi-chunk turns prefer :meth:`_publish_response_batch` which
        attaches a shared ``batch_id`` so ``tts_node`` can fire
        ``/voice/tts/batch_complete`` once the last chunk lands (issue #980).
        """
        msg = String()
        msg.data = build_ssml_payload(text, animation)
        self._response_pub.publish(msg)

    def _publish_response_batch(self, chunks: List[str], animation: str = "neutral") -> int:
        """Publish ``chunks`` as a single TTS batch (issue #980).

        Each chunk carries the same ``batch_id`` plus 1-based ``batch_index``
        and ``batch_total`` counters. ``tts_node`` echoes those fields on
        ``/voice/tts/finished`` and publishes ``/voice/tts/batch_complete``
        after the final chunk. Returns the number of chunks published.

        Issue #992: the batch is registered eagerly on the dialogue_node
        side so the music-cleanup gate knows about it even if the first
        ``/voice/tts/finished`` event is lost (e.g. dropped subscription
        edge case). The registry is idempotent so the matching
        ``_on_tts_finished`` registration is a safe no-op.
        """
        if not chunks:
            return 0
        batch_id = str(uuid.uuid4())
        total = len(chunks)
        self._register_active_batch(batch_id, total)
        for idx, chunk in enumerate(chunks, start=1):
            msg = String()
            msg.data = build_ssml_payload(
                chunk,
                animation,
                batch_id=batch_id,
                batch_index=idx,
                batch_total=total,
            )
            self._response_pub.publish(msg)
        return total

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
