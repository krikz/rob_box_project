"""``DialogHarness`` — voice-dialogue harness wrapping the dialogue logic.

A :class:`Harness` subclass that adapts the full voice-dialogue
pipeline (wake-word → STT → LLM → TTS) into the harness framework
per ADR-0001 §2.7.1.

**Design — parallel implementation:**
    This harness lives side-by-side with the existing
    ``rob_box_voice.dialogue_node.DialogueNode``. Both coexist;
    switching is via the ``harness.kind`` config flag
    (``"dialog"`` → this harness vs. ``"legacy"`` → the ROS2 node).

**What moved in:**
    * LLM client + fallback → :class:`LLMProvider` (MiniMax / DeepSeek)
    * 30 tools / 5 skills → :class:`ToolProvider` + local skill registry
    * DialogueManager + IDLE/LISTENING/DIALOGUE/SILENCED → :class:`DialogueStateMachine`
    * voice_memory / faq_store → :class:`MemoryStore` (SQLiteVoiceMemory)

**What STAYS in dialogue_node.py:**
    * ROS2 subscribers/publishers (STT, VAD, dialogue/response, dialogue/state)
    * Lifecycle driver (the main async loop that feeds inputs into the harness)

**Usage:**
    >>> async with DialogHarness(config) as h:
    ...     result = await h.run("привет роббокс")
    ...     print(result.output)  # "Здравствуйте! Чем могу помочь?"
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

from rob_box_harness.config import HarnessConfig
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
    DialogState,
)
from rob_box_harness.effects import EffectContext
from rob_box_harness.harness import Harness
from rob_box_harness.harnesses._base import (
    PostProcessor,
    ensure_user_text,
    run_request_response_loop,
)
from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory
from rob_box_llm.provider import LLMMessage

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Simple skill abstraction (internal — mirrors the plan's SkillRegistry)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SkillSpec:
    """Minimal skill descriptor for the LLM tool-choice step."""

    name: str
    description: str
    parameters: dict[str, Any] = field(default_factory=dict)


class Skill:
    """A single named capability exposed to the LLM as a tool.

    Subclasses override :meth:`execute` and return a plain string.
    This is intentionally simpler than the full ``ToolProvider``
    contract — harness-level skills are few and stable.
    """

    name: str = ""
    description: str = ""

    def tool_spec(self) -> SkillSpec:
        """Return the tool spec the LLM sees."""
        return SkillSpec(name=self.name, description=self.description)

    def execute(self, text: str, state: DialogState) -> str:
        """Run the skill against the user text and harness state.

        Args:
            text: The user's original input (post-wake-word cleanup).
            state: The current dialog state for context.

        Returns:
            A string result that is appended to the LLM context.
        """
        raise NotImplementedError


class VoiceSettingsSkill(Skill):
    """Adjust TTS voice parameters (speed, pitch, emotion)."""

    name = "voice_settings"
    description = "Изменяет настройки голоса: скорость, высоту, эмоцию."

    def execute(self, text: str, state: DialogState) -> str:
        _ = text
        _ = state
        return "Настройки голоса не изменены (заглушка)."


class DJPlaylistSkill(Skill):
    """Control music playback via the DJ subsystem."""

    name = "dj_playlist"
    description = "Управляет музыкальным плейлистом: включить, пауза, следующий трек."

    def execute(self, text: str, state: DialogState) -> str:
        _ = text
        _ = state
        return "DJ-плейлист: команда принята (заглушка)."


class MappingSkill(Skill):
    """Mapping / navigation commands (waypoints, go-to)."""

    name = "mapping"
    description = "Управление картой и навигацией: сохранить точку, поехать к точке."

    def execute(self, text: str, state: DialogState) -> str:
        _ = text
        _ = state
        return "Карта: команда принята (заглушка)."


class SkillRegistry:
    """A simple registry mapping ``name → Skill`` for the harness.

    Not a full ``ToolProvider`` — the harness uses this registry
    to generate tool specs for the LLM and to execute skill calls
    that the LLM's ``complete()`` returns as tool invocations.
    """

    def __init__(self, skills: list[Skill] | None = None) -> None:
        self._skills: dict[str, Skill] = {}
        for s in (skills or []):
            self._skills[s.name] = s

    def register(self, skill: Skill) -> None:
        """Add a skill."""
        self._skills[skill.name] = skill

    def tool_specs(self) -> list[dict[str, Any]]:
        """Return the tool specs for the LLM's tool-choice step.

        Format matches OpenAI-compatible tool definitions:
        ``{"type": "function", "function": {"name": ..., "description": ..., "parameters": {...}}}``.
        """
        return [
            {
                "type": "function",
                "function": {
                    "name": s.name,
                    "description": s.description,
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
            }
            for s in self._skills.values()
        ]

    def execute(self, name: str, text: str, state: DialogState) -> str:
        """Execute a named skill."""
        skill = self._skills.get(name)
        if skill is None:
            return f"Навык '{name}' не найден."
        return skill.execute(text, state)

    def __len__(self) -> int:
        return len(self._skills)

    def __contains__(self, name: str) -> bool:
        return name in self._skills


# ---------------------------------------------------------------------------
# Dummy LLM provider (fallback when no real provider is configured)
# ---------------------------------------------------------------------------


class DummyLLMProvider:
    """Fallback LLM provider returning canned responses.

    Used when no real :class:`LLMProvider` is wired in — lets the
    harness run through its lifecycle without crashing.
    """

    name = "dummy"

    async def complete(self, messages: list[LLMMessage], **kwargs: Any) -> Any:
        """Return a canned response."""
        _ = messages, kwargs
        # Return a simple object matching the LLMResponse interface
        return _DummyResponse(content="Я заглушка LLM. Настоящий провайдер не подключён.")

    async def aclose(self) -> None:
        pass


@dataclass
class _DummyResponse:
    content: str
    tool_calls: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# DialogHarness
# ---------------------------------------------------------------------------


class DialogHarness(Harness[DialogState]):
    """Voice-dialogue harness wrapping the full dialog pipeline.

    Owns a :class:`DialogueStateMachine` for state transitions and
    delegates the core "user input → LLM → tool loop → final text"
    path to :func:`run_request_response_loop`.

    Parameters are passed through to :class:`Harness.__init__`
    (config, ports, clock, hooks). Missing ports are filled with
    sensible defaults in :meth:`init`.
    """

    name = "dialog"

    # ── init (port wiring) ──────────────────────────────────────

    async def init(self) -> None:
        """Wire real providers and initialise the state machine.

        Calls ``super().init()`` first (sets up default ports),
        then overrides with real implementations where available.
        """
        await super().init()

        # LLM provider — if super() gave us a DummyLLMProvider, we
        # keep it; concrete deployments inject a real one via
        # HarnessConfig → registry.
        if self.llm is None or getattr(self.llm, "name", "") == "dummy":
            self.llm = DummyLLMProvider()

        # Memory — prefer SQLite, fall back to in-memory
        if self.memory is None:
            try:
                self.memory = SQLiteVoiceMemory(db_path=":memory:")
                await self.memory.init()
            except Exception:
                logger.warning(
                    "SQLiteVoiceMemory init failed, using InMemoryStore",
                    exc_info=True,
                )
                # Keep whatever super().init() set up

        # Transport — ROS2 if available (requires a running ROS2 node)
        if self.transport is None:
            try:
                from rob_box_harness.transport.ros2_transport import ROS2Transport
                import rclpy
                node = rclpy.create_node("dialog_harness_node")
                self.transport = ROS2Transport(node=node)
            except Exception:
                logger.debug(
                    "ROS2Transport not available (no ROS2 runtime), "
                    "using FakeTransport",
                )
                # Keep the default FakeTransport from super().init()

        # State machine
        self._dsm = DialogueStateMachine(
            initial=DialogueStateKind.IDLE,
            silence_timeout=getattr(self.config, "silence_timeout", 300.0),
        )

        # Skill registry — the 3 skills from the plan
        self._skills = SkillRegistry([
            VoiceSettingsSkill(),
            DJPlaylistSkill(),
            MappingSkill(),
        ])

        # Initialize dialog state
        self.state = DialogState(user_id=self.config.name)

    # ── step (main turn logic) ─────────────────────────────────

    async def step(self, input_data: Any) -> str:
        """Process a single user turn through the dialogue pipeline.

        Flow:
        1. Coerce input to text
        2. Classify via DialogueStateMachine.on_user_input
        3. If silenced / wake-word-gated — return early
        4. Build messages from MemoryStore history
        5. Call LLM via run_request_response_loop
        6. Dispatch side-effects (TTS, LED, sound)
        7. Save turn to memory
        8. Update state and return response text
        """
        text = ensure_user_text(input_data)
        if not text.strip():
            return ""

        # ── State machine classification ──
        event = self._dsm.on_user_input(text, self.state)
        self._dsm.on_event(event, self.state)

        # Check silence timeout (periodic housekeeping)
        timeout_event = self._dsm.check_silence_timeout()
        if timeout_event is not None:
            self._dsm.on_event(timeout_event, self.state)

        dsm_state = self._dsm.state

        # While SILENCED, ignore all input except unsilence commands
        if dsm_state == DialogueStateKind.SILENCED:
            if event == DialogueEvent.UNSILENCE:
                self.state.silenced = False
                self.state.silenced_until = 0.0
            return ""

        # IDLE → only wake word triggers progression
        if dsm_state == DialogueStateKind.IDLE:
            if event != DialogueEvent.WAKE_WORD:
                return ""
            # Wake word detected — update state
            self.state.wake_active = False
            self.state.last_stt_text = text
            # Remove wake word from text for cleaner LLM input
            clean_text = self._strip_wake_word(text)
            if not clean_text.strip():
                # Just "роббокс" with nothing after — greet
                clean_text = "привет"
            return await self._process_turn(clean_text)

        # LISTENING / DIALOGUE — process normally
        self.state.last_stt_text = text
        return await self._process_turn(text)

    # ── teardown ────────────────────────────────────────────────

    async def teardown(self) -> None:
        """Release DSM and parent resources."""
        self._dsm.reset(DialogueStateKind.IDLE)
        await super().teardown()

    # ── internal helpers ────────────────────────────────────────

    async def _process_turn(self, text: str) -> str:
        """Execute the canonical LLM turn: history → LLM → side-effect → save.

        This is the core of the dialog pipeline. It delegates the
        heavy lifting to :func:`run_request_response_loop` (the same
        helper used by EchoHarness and UpperHarness), then adds
        dialog-specific side-effects and state updates.
        """
        # Build tool specs from the skill registry for this turn
        tool_specs = self._skills.tool_specs()

        # Use the shared run_request_response_loop helper.
        # It handles: coerce → LLM.complete → tool loop → memory.save → EchoEffect
        try:
            response_text = await run_request_response_loop(
                self,
                text,
                post_process=_dialog_post_process,
            )
        except Exception:
            logger.exception("LLM turn failed, returning fallback")
            response_text = "Извините, произошла ошибка. Попробуйте ещё раз."

        # Dialog-specific side-effects (TTS, LED animation)
        ctx = EffectContext(harness=self.name)
        try:
            # TTS — dispatch to the side-effect bus so the
            # real TTS node picks it up via ROS2
            await self.effects.dispatch(
                _DialogTTSEffect(text=response_text)
            )
        except Exception:
            logger.debug("TTS dispatch skipped (no consumer)", exc_info=True)

        # Update state
        self.state.last_response = response_text
        self.state.turn_count += 1

        # Signal dialogue end to the state machine
        self._dsm.on_event(DialogueEvent.DIALOGUE_END, self.state)

        return response_text

    @staticmethod
    def _strip_wake_word(text: str) -> str:
        """Remove known wake words from the beginning of ``text``.

        Delegates to :func:`rob_box_voice.core.dialogue_text.strip_wake_word`
        so the same logic is shared with the legacy ``DialogueNode``
        (see ADR-0001 §2.7 — single source of truth for the
        wake-word gate).
        """
        from rob_box_voice.core.dialogue_text import strip_wake_word
        return strip_wake_word(text)


def _dialog_post_process(text: str) -> str:
    """Identity post-processor for dialog responses (no transformation needed)."""
    return text


# ---------------------------------------------------------------------------
# Dialog-specific effect (internal)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _DialogTTSEffect:
    """Declarative TTS effect — "please speak this text".

    The side-effect bus routes this to the real TTS node or a
    fake for testing. The harness does NOT call TTS directly.
    """

    text: str


__all__ = [
    "DialogHarness",
    "DialogState",
    "DialogueStateKind",
    "DialogueEvent",
]

# Re-export DialogState from the core module for convenience
# (the canonical definition lives in core.dialogue_state_machine)
from rob_box_harness.core.dialogue_state_machine import DialogState  # noqa: E402, F811
