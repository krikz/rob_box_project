"""Tests for the harness-side ``DialogCore``.

``DialogCore`` is the high-level facade that wraps the four core
ports (``LLMProvider``, ``ToolProvider``, ``MemoryStore``,
``DialogueStateMachine``) into a single object the dialogue shell can
call. It owns no ROS2 state — all transport lives in the shell.

The class exists so that:

1. The dialogue shell can stay thin (~300 LOC) and only deal with
   ROS2 pub/sub.
2. The whole conversation loop (state transitions + LLM call +
   tool dispatch + memory persistence) is testable in pure Python.
3. The shell can swap any of the four ports (for tests / fallbacks)
   without re-implementing the loop.

Coverage:
* Construction accepts the four ports
* ``process_input(text, history)`` returns a DialogResult
* Result carries the new state, spoken text, and tools called
* Silence / wake-word / timeout paths delegate to the DSM
* MemoryStore.append_turn is invoked for each turn
* Errors in the LLM are wrapped into DialogResult.error (not raised)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

import pytest

from rob_box_harness.core.dialog_core import DialogCore, DialogResult
from rob_box_harness.core.dialogue_state_machine import (
    DialogState,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMMessage, LLMResponse


# ---------------------------------------------------------------------------
# Fakes for the four ports
# ---------------------------------------------------------------------------


@dataclass
class _FakeLLMMessage:
    role: str = "user"
    content: str = ""


class _FakeLLMProvider:
    """Records every complete() call and returns a canned response."""

    name = "fake_llm"

    def __init__(self, response_text: str = "ok", error: Exception | None = None) -> None:
        self.response_text = response_text
        self.error = error
        self.calls: list[list[Any]] = []

    async def complete(
        self,
        messages: Any = None,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> Any:
        # Materialise the iterable so we can inspect it.
        if messages is None:
            materialised: list[Any] = []
        elif isinstance(messages, list):
            materialised = messages
        else:
            materialised = list(messages)
        self.calls.append(materialised)
        if self.error is not None:
            raise self.error
        return LLMResponse(
            content=self.response_text,
            tool_calls=(),
        )

    async def aclose(self) -> None:
        return None


class _FakeToolProvider:
    """Reports an empty tool manifest."""

    name = "fake_tools"

    async def discover(self) -> tuple[Any, ...]:
        return ()

    async def execute(self, call: Any) -> Any:
        return None

    async def aclose(self) -> None:
        return None


class _FakeMemoryStore:
    """Records append_turn / load_recent / save_fact / search_facts calls."""

    def __init__(self) -> None:
        from rob_box_harness.memory import Turn
        self.turns: list[Turn] = []
        self.facts: list[tuple[str, str]] = []
        self.load_recent_calls: list[tuple[str, int]] = []

    async def append_turn(self, scope: str, turn: Any) -> None:
        self.turns.append(turn)

    async def load_recent(self, scope: str, limit: int = 10) -> list[Any]:
        self.load_recent_calls.append((scope, limit))
        # Return the last ``limit`` turns in chronological order —
        # mirrors the real MemoryStore contract.
        return list(self.turns[-limit:])

    async def save_fact(self, scope: str, fact: Any) -> None:
        self.facts.append((fact.key, fact.value))

    async def search_facts(self, scope: str, query: str, limit: int = 5) -> list[Any]:
        return []

    async def aclose(self) -> None:
        return None


@pytest.fixture
def dsm() -> DialogueStateMachine:
    return DialogueStateMachine()


@pytest.fixture
def llm() -> _FakeLLMProvider:
    return _FakeLLMProvider(response_text="hello back")


@pytest.fixture
def tools_provider() -> _FakeToolProvider:
    return _FakeToolProvider()


@pytest.fixture
def memory() -> _FakeMemoryStore:
    return _FakeMemoryStore()


@pytest.fixture
def core(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> DialogCore:
    obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    # Drive DSM into LISTENING — most tests below assume the wake
    # word has already fired.
    asyncio.run(obj.handle_wake_word(""))
    return obj


def _prime_listening(core: DialogCore) -> None:
    """Drive the DSM into LISTENING so a subsequent STT_RESULT.
    transitions into DIALOGUE. Mirrors what the shell does when the
    wake-word detector fires."""
    asyncio.run(core.handle_wake_word(""))


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


def test_construction_accepts_all_four_ports(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """DialogCore accepts the four ports without errors."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    assert core_obj is not None


def test_construction_rejects_missing_llm(
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """DialogCore must refuse to construct without an LLM."""
    with pytest.raises(TypeError):
        DialogCore(llm=None, tools=tools_provider, memory=memory, dsm=dsm)


# ---------------------------------------------------------------------------
# process_input
# ---------------------------------------------------------------------------


def test_process_input_returns_dialog_result(core: DialogCore) -> None:
    """process_input returns a DialogResult with state + spoken text."""
    result = asyncio.run(core.process_input("hello", history=[]))
    assert isinstance(result, DialogResult)
    assert result.spoken_text == "hello back"
    assert result.error is None


def test_process_input_persists_turns(core: DialogCore, memory: _FakeMemoryStore) -> None:
    """User turn + assistant turn are appended to memory."""
    asyncio.run(core.process_input("hello", history=[]))
    # 2 turns: user + assistant
    roles = [t.role for t in memory.turns]
    assert roles == ["user", "assistant"]


def test_process_input_invokes_llm(core: DialogCore, llm: _FakeLLMProvider) -> None:
    """process_input calls llm.complete() with at least the user turn."""
    asyncio.run(core.process_input("hello", history=[]))
    assert len(llm.calls) == 1


def test_process_input_includes_history_in_llm_messages(
    core: DialogCore, llm: _FakeLLMProvider
) -> None:
    """History turns are prepended to the LLM call's message list."""
    history = [
        LLMMessage(role="user", content="earlier"),
        LLMMessage(role="assistant", content="earlier reply"),
    ]
    asyncio.run(core.process_input("now", history=history))
    sent = llm.calls[0]
    # user msg, assistant msg, user msg
    assert len(sent) == 3
    assert sent[0].content == "earlier"
    assert sent[-1].content == "now"


def test_process_input_wraps_llm_errors(core: DialogCore, llm: _FakeLLMProvider) -> None:
    """LLM exceptions are surfaced via DialogResult.error, not raised."""
    llm.error = ProviderError("boom")
    result = asyncio.run(core.process_input("hello", history=[]))
    assert result.error is not None
    assert isinstance(result.error, ProviderError)


def test_process_input_error_does_not_persist_assistant_turn(
    core: DialogCore, memory: _FakeMemoryStore, llm: _FakeLLMProvider
) -> None:
    """When the LLM fails, only the user turn is persisted."""
    llm.error = ProviderError("boom")
    asyncio.run(core.process_input("hello", history=[]))
    assert len(memory.turns) == 1
    turn = memory.turns[0]
    assert turn.role == "user"
    assert turn.content == "hello"


# ---------------------------------------------------------------------------
# State transitions via DSM
# ---------------------------------------------------------------------------


def test_process_input_updates_state_to_dialogue_then_idle(
    core: DialogCore, dsm: DialogueStateMachine
) -> None:
    """Successful input drives IDLE → DIALOGUE → IDLE."""
    asyncio.run(core.process_input("hello", history=[]))
    # After processing, the DSM should be back to IDLE.
    assert dsm.state == DialogueStateKind.IDLE


def test_process_input_result_reports_final_state(core: DialogCore) -> None:
    """DialogResult.new_state reflects the DSM after the turn."""
    result = asyncio.run(core.process_input("hello", history=[]))
    assert result.new_state == DialogueStateKind.IDLE


# ---------------------------------------------------------------------------
# Wake-word / silence shortcuts (W3 plan §3 short hooks)
# ---------------------------------------------------------------------------


def test_is_wake_word_returns_bool_for_wake_text(dsm: DialogueStateMachine) -> None:
    """is_wake_word matches the W3 plan signature ``→ bool``."""
    core_obj = DialogCore(
        llm=_FakeLLMProvider(),
        tools=_FakeToolProvider(),
        memory=_FakeMemoryStore(),
        dsm=dsm,
    )
    assert core_obj.is_wake_word("роббокс") is True
    assert core_obj.is_wake_word("привет") is False


def test_handle_wake_word_transitions_idle_to_listening(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """handle_wake_word drives IDLE → LISTENING."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    assert dsm.current_state == DialogueStateKind.IDLE
    result = asyncio.run(core_obj.handle_wake_word("роббокс"))
    assert dsm.current_state == DialogueStateKind.LISTENING
    assert result.new_state == DialogueStateKind.LISTENING


def test_handle_silence_transitions_to_silenced(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """handle_silence drives any state → SILENCED."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    # Wake up first so we're in LISTENING, not IDLE.
    asyncio.run(core_obj.handle_wake_word("роббокс"))
    assert dsm.current_state == DialogueStateKind.LISTENING

    result = asyncio.run(core_obj.handle_silence())
    assert dsm.current_state == DialogueStateKind.SILENCED
    assert result.new_state == DialogueStateKind.SILENCED


def test_handle_silence_from_dialogue(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """handle_silence works from DIALOGUE too (mid-flow interrupt)."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    dsm.transition(DialogueStateKind.LISTENING)
    dsm.transition(DialogueStateKind.DIALOGUE)
    asyncio.run(core_obj.handle_silence())
    assert dsm.current_state == DialogueStateKind.SILENCED


# ---------------------------------------------------------------------------
# check_timeout / inactivity
# ---------------------------------------------------------------------------


def test_check_timeout_with_inactivity_drops_listening_to_idle(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """When ``inactivity_timeout`` is set, check_timeout drops LISTENING→IDLE."""
    core_obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        inactivity_timeout=0.001,  # 1 ms — will fire immediately
    )
    dsm.transition(DialogueStateKind.LISTENING)
    assert dsm.current_state == DialogueStateKind.LISTENING
    # Force the activity clock to look old.
    dsm._last_activity_at -= 10.0  # type: ignore[attr-defined]
    fired = core_obj.check_timeout()
    assert fired is True
    assert dsm.current_state == DialogueStateKind.IDLE


def test_check_timeout_legacy_event_path(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Without ``inactivity_timeout``, check_timeout drives a TIMEOUT event."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    dsm.transition(DialogueStateKind.LISTENING)
    fired = core_obj.check_timeout()
    # LISTENING + TIMEOUT → IDLE (per on_event semantics).
    assert fired is True
    assert dsm.current_state == DialogueStateKind.IDLE


# ---------------------------------------------------------------------------
# History trimming delegation to MemoryStore (W3 plan §3 'history trimming
# delegates to MemoryStore')
# ---------------------------------------------------------------------------


def test_history_trim_delegates_to_memory_store(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """When history=None and trim_limit is set, DialogCore asks memory."""
    # Seed memory with prior turns.
    from rob_box_harness.memory import Turn
    prior = [
        Turn(role="user", content="earlier question"),
        Turn(role="assistant", content="earlier answer"),
    ]
    memory.turns.extend(prior)

    core_obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        history_trim_limit=10,
    )
    # Drive to LISTENING so the next STT_RESULT transitions into DIALOGUE.
    asyncio.run(core_obj.handle_wake_word(""))
    asyncio.run(core_obj.process_input("now question", history=None))

    # memory.load_recent must have been called for the trim delegation.
    assert memory.load_recent_calls, "DialogCore did not delegate to memory"
    sent = llm.calls[0]
    # Two prior turns from memory + the new user message.
    assert len(sent) == 3
    assert sent[0].content == "earlier question"
    assert sent[1].content == "earlier answer"
    assert sent[2].content == "now question"


def test_explicit_history_overrides_memory_trim(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """When the caller passes history, memory is NOT consulted."""
    core_obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        history_trim_limit=10,
    )
    asyncio.run(core_obj.handle_wake_word(""))
    history = [LLMMessage(role="user", content="explicit")]
    asyncio.run(core_obj.process_input("now", history=history))
    sent = llm.calls[0]
    assert len(sent) == 2
    assert sent[0].content == "explicit"
    assert sent[1].content == "now"
    # Explicit history → memory must NOT have been consulted.
    assert memory.load_recent_calls == []
