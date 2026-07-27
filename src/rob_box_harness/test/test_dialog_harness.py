"""Unit tests for :class:`DialogHarness` — voice-dialogue harness.

Tests the full dialog pipeline using ONLY fake ports — no ROS2,
no network, no real LLM API. Exercises wake-word handling, state
transitions, tool dispatch, memory persistence, side-effects,
lifecycle, and error handling.

Uses ``asyncio.run()`` for async execution (compatible with
pytest 6.2.5 + ROS2 ament plugins).

The DSM processes one event per ``step()`` call. The typical
two-step flow is:

1. ``step("robbox")`` — WAKE_WORD → IDLE→LISTENING, then
   the text is processed through the LLM. DSM stays at LISTENING.
2. ``step("hello")`` — STT_RESULT → LISTENING→DIALOGUE, LLM
   processes, DIALOGUE_END → DIALOGUE→IDLE.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.core.dialogue_state_machine import (
    DialogueStateKind,
)
from rob_box_harness.effects import RecordingBus
from rob_box_harness.harnesses.dialog import DialogHarness
from rob_box_harness.memory import InMemoryStore
from rob_box_harness.tools import FakeToolProvider, ToolSpec
from rob_box_harness.transport import FakeTransport
from rob_box_llm.provider import LLMMessage


# ---------------------------------------------------------------------------
# Configurable mock LLM provider
# ---------------------------------------------------------------------------

@dataclass
class MockResponse:
    """Canned LLM response with optional tool_calls."""
    content: str = "Hello! I am your assistant."
    tool_calls: list = field(default_factory=list)


class MockLLMProvider:
    """LLM provider returning configurable responses per test."""

    name = "mock"

    def __init__(self, response: MockResponse | None = None) -> None:
        self.response = response or MockResponse()
        self.calls: list[list[LLMMessage]] = []

    async def complete(self, messages: list[LLMMessage], **kwargs: Any) -> MockResponse:
        self.calls.append(messages)
        return self.response

    async def aclose(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_config(**overrides: Any) -> HarnessConfig:
    cfg: dict[str, Any] = {"harness": {"kind": "dialog", "name": "test_dialog"}}
    cfg["harness"].update(overrides)
    return HarnessConfig.from_dict(cfg)


def _make_harness(
    *,
    llm: MockLLMProvider | None = None,
    tools: FakeToolProvider | None = None,
    memory: InMemoryStore | None = None,
    effects: RecordingBus | None = None,
    transport: FakeTransport | None = None,
    config: HarnessConfig | None = None,
) -> DialogHarness:
    """Build a DialogHarness with fake ports pre-injected."""
    return DialogHarness(
        config=config or _make_config(),
        llm=llm or MockLLMProvider(),
        tools=tools or FakeToolProvider(),
        memory=memory or InMemoryStore(),
        effects=effects or RecordingBus(),
        transport=transport or FakeTransport(),
    )


def _run(coro):
    """Shorthand for asyncio.run(coro)."""
    return asyncio.run(coro)


# Russian wake words that the DSM recognizes
_WAKE = "роббокс"
_HELLO = "привет"


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestDialogHarnessBasicTurn:
    """Basic turn processing via the two-step wake→listen→dialogue flow."""

    def test_step_returns_string(self) -> None:
        """After wake word, a follow-up step returns an LLM response."""
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step(_WAKE))       # wake up
        result = _run(harness.step(_HELLO))  # speak
        assert isinstance(result, str)
        assert len(result) > 0

    def test_turn_count_increments(self) -> None:
        """Each step that processes through the LLM increments turn_count."""
        harness = _make_harness()
        _run(harness.init())
        assert harness.state.turn_count == 0
        _run(harness.step(_WAKE))
        assert harness.state.turn_count == 1
        _run(harness.step(_HELLO))
        assert harness.state.turn_count == 2

    def test_last_stt_text_updated(self) -> None:
        """state.last_stt_text holds the last STT transcript."""
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step(_WAKE))
        _run(harness.step("как дела"))
        assert harness.state.last_stt_text == "как дела"

    def test_last_response_updated(self) -> None:
        """state.last_response reflects the LLM output."""
        llm = MockLLMProvider(MockResponse(content="Здравствуйте!"))
        harness = _make_harness(llm=llm)
        _run(harness.init())
        _run(harness.step(_WAKE))
        result = _run(harness.step(_HELLO))
        assert result == "Здравствуйте!"
        assert harness.state.last_response == "Здравствуйте!"


class TestWakeWordTransition:
    """Wake word detection triggers state transitions."""

    def test_wake_word_transitions_to_listening(self) -> None:
        """step('robbox') transitions IDLE→LISTENING and DSM stays at LISTENING."""
        harness = _make_harness()
        _run(harness.init())
        assert harness._dsm.state == DialogueStateKind.IDLE
        _run(harness.step(_WAKE))
        # After wake word: WAKE_WORD → IDLE→LISTENING.
        # Then text processed through LLM, DIALOGUE_END fires in LISTENING
        # (no transition — stays LISTENING).
        assert harness._dsm.state == DialogueStateKind.LISTENING

    def test_two_step_cycle_returns_to_idle(self) -> None:
        """Wake → listen → STT → dialogue → end → IDLE (two-step flow)."""
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step(_WAKE))       # IDLE→LISTENING
        assert harness._dsm.state == DialogueStateKind.LISTENING
        _run(harness.step(_HELLO))      # STT_RESULT → LISTENING→DIALOGUE, then DIALOGUE_END→IDLE
        assert harness._dsm.state == DialogueStateKind.IDLE

    def test_idle_ignores_non_wake(self) -> None:
        """In IDLE state, non-wake-word input returns empty string."""
        harness = _make_harness()
        _run(harness.init())
        result = _run(harness.step(_HELLO))
        assert result == ""


class TestSilenceCommand:
    """Silence command handling with Russian patterns."""

    def test_tikho_sets_silenced(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step("тихо"))
        assert harness._dsm.state == DialogueStateKind.SILENCED

    def test_silenced_ignores_input(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step("тихо"))
        result = _run(harness.step(_HELLO))
        assert result == ""

    def test_molchi_sets_silenced(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step("молчи"))
        assert harness._dsm.state == DialogueStateKind.SILENCED

    def test_zamolchi_sets_silenced(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step("замолчи"))
        assert harness._dsm.state == DialogueStateKind.SILENCED


class TestToolDispatch:
    """Tool execution when LLM returns tool_calls."""

    def test_tool_call_executed(self) -> None:
        tools = FakeToolProvider()
        tool_called: list[str] = []

        async def handler(args: dict[str, Any]) -> str:
            tool_called.append(args.get("text", ""))
            return "done"

        tools.register(
            ToolSpec(name="test_tool", description="Test tool", parameters={}),
            handler,
        )
        llm = MockLLMProvider(MockResponse(
            content="Let me check...",
            tool_calls=[
                type("TC", (), {
                    "id": "c1",
                    "name": "test_tool",
                    "arguments": {"text": "hello"},
                })(),
            ],
        ))
        harness = _make_harness(llm=llm, tools=tools)
        _run(harness.init())
        _run(harness.step(_WAKE))
        result = _run(harness.step(_HELLO))
        assert len(tool_called) >= 1
        assert "hello" in tool_called
        assert isinstance(result, str)


class TestMemoryPersistence:
    """Memory store tracking of conversation turns."""

    def test_turns_saved(self) -> None:
        memory = InMemoryStore()
        harness = _make_harness(memory=memory)
        _run(harness.init())
        _run(harness.step(_WAKE))
        _run(harness.step(_HELLO))
        turns = _run(memory.load_recent("test_dialog"))
        assert len(turns) >= 2  # user + assistant from at least one full turn

    def test_multiple_turns(self) -> None:
        memory = InMemoryStore()
        harness = _make_harness(memory=memory)
        _run(harness.init())
        _run(harness.step(_WAKE))
        _run(harness.step(_HELLO))
        _run(harness.step("как дела"))
        turns = _run(memory.load_recent("test_dialog"))
        assert len(turns) >= 3


class TestSideEffects:
    """Side-effect bus captures dispatched effects."""

    def test_effect_dispatched(self) -> None:
        effects = RecordingBus()
        harness = _make_harness(effects=effects)
        _run(harness.init())
        _run(harness.step(_WAKE))
        _run(harness.step(_HELLO))
        # run_request_response_loop + _DialogTTSEffect
        assert len(effects.effects) >= 2


class TestLifecycle:
    """Init/teardown idempotency and lifecycle contracts."""

    def test_init_idempotent(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        llm_before = harness.llm
        _run(harness.init())
        assert harness.llm is llm_before

    def test_teardown_idempotent(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.teardown())
        _run(harness.teardown())

    def test_async_with_context(self) -> None:
        harness = _make_harness()

        async def _use():
            async with harness as h:
                result = await h.step(_WAKE)
                assert isinstance(result, str)

        _run(_use())
        assert harness.is_initialized is False

    def test_run_requires_init(self) -> None:
        harness = _make_harness()
        with pytest.raises(Exception):
            _run(harness.run("hello"))


class TestErrorHandling:
    """Graceful degradation on provider failures."""

    def test_llm_error_fallback(self) -> None:

        class FailingLLM:
            name = "failing"

            async def complete(self, messages, **kwargs):
                raise RuntimeError("LLM failed")

            async def aclose(self):
                pass

        harness = _make_harness(llm=FailingLLM())
        _run(harness.init())
        _run(harness.step(_WAKE))
        result = _run(harness.step(_HELLO))
        assert isinstance(result, str)
        assert len(result) > 0


class TestStateMachineIntegration:
    """Verify DialogHarness + DSM integration."""

    def test_dsm_reset_on_teardown(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        _run(harness.step(_WAKE))
        _run(harness.teardown())
        assert harness._dsm.state == DialogueStateKind.IDLE


class TestEmptyInput:
    """Edge cases for empty / whitespace input."""

    def test_empty_input(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        assert _run(harness.step("")) == ""

    def test_whitespace_only(self) -> None:
        harness = _make_harness()
        _run(harness.init())
        assert _run(harness.step("   ")) == ""


class TestStripWakeWord:
    """Wake word stripping from input text."""

    def test_strip_robbox(self) -> None:
        assert DialogHarness._strip_wake_word("роббокс привет как дела") == "привет как дела"

    def test_strip_rob_box(self) -> None:
        assert DialogHarness._strip_wake_word("роб бокс расскажи анекдот") == "расскажи анекдот"

    def test_strip_with_comma(self) -> None:
        assert DialogHarness._strip_wake_word("роббокс, привет") == "привет"

    def test_strip_only_wake(self) -> None:
        assert DialogHarness._strip_wake_word("роббокс") == ""

    def test_strip_case_insensitive(self) -> None:
        assert DialogHarness._strip_wake_word("РОББОКС привет") == "привет"
