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
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, ToolCall


# ---------------------------------------------------------------------------
# Fakes for the four ports
# ---------------------------------------------------------------------------


@dataclass
class _FakeLLMMessage:
    role: str = "user"
    content: str = ""


class _FakeLLMProvider:
    """Records every complete() call and returns a canned response.

    The ``responses`` queue lets a test script the LLM's tool-call
    behaviour: each ``complete()`` pops the next entry. Entries are
    either an :class:`LLMResponse` (returned verbatim) or an
    :class:`Exception` (raised). When the queue is empty the
    provider falls back to ``response_text``.
    """

    name = "fake_llm"

    def __init__(
        self,
        response_text: str = "ok",
        error: Exception | None = None,
        responses: list[object] | None = None,
    ) -> None:
        self.response_text = response_text
        self.error = error
        self.responses: list[object] = list(responses or [])
        # Records every complete() call as (messages, tools).
        self.calls: list[tuple[list[Any], Any]] = []

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
        self.calls.append((materialised, tools))

        if self.responses:
            item = self.responses.pop(0)
            if isinstance(item, BaseException):
                raise item
            return item

        if self.error is not None:
            raise self.error
        return LLMResponse(
            content=self.response_text,
            tool_calls=(),
        )

    async def aclose(self) -> None:
        return None


class _FakeToolProvider:
    """Reports a manifest and dispatches calls to ``handler_map``.

    The default manifest advertises ``memory_context`` and
    ``echo`` so tests can exercise the OpenAI wire format. Tests
    can pass their own manifest via the constructor to override.
    """

    name = "fake_tools"

    def __init__(
        self,
        manifest: tuple[Any, ...] | None = None,
        handler_map: dict[str, Any] | None = None,
    ) -> None:
        from rob_box_harness.tools import ToolSpec
        self._manifest: tuple[Any, ...] = (
            manifest
            if manifest is not None
            else (
                ToolSpec(
                    name="memory_context",
                    description="Load recent conversation context.",
                    parameters={
                        "type": "object",
                        "properties": {"limit": {"type": "integer"}},
                    },
                ),
                ToolSpec(
                    name="echo",
                    description="Echo back the provided arguments.",
                    parameters={
                        "type": "object",
                        "properties": {"text": {"type": "string"}},
                    },
                ),
            )
        )
        self._handler_map: dict[str, Any] = handler_map or {}
        self.executed: list[Any] = []

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> Any:
        from rob_box_llm.provider import ToolResult
        self.executed.append(call)
        handler = self._handler_map.get(call.name)
        if handler is None:
            return ToolResult(
                tool_call_id=call.id,
                content=f"unknown tool: {call.name}",
                is_error=True,
            )
        result = handler(dict(call.arguments))
        if hasattr(result, "__await__"):
            result = await result
        # A handler may return a full ToolResult (e.g. is_error=True) —
        # respect it verbatim instead of stringifying it into a
        # non-error result (issue #1253 babble filter depends on
        # is_error reaching DialogCore).
        if isinstance(result, ToolResult):
            return result
        return ToolResult(
            tool_call_id=call.id,
            content=str(result),
            is_error=False,
        )

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


def test_construction_rejects_missing_tools(
    llm: _FakeLLMProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """DialogCore must refuse to construct without a ToolProvider.

    Without tools the model can never see ``memory_context`` and
    issue #916 regresses.
    """
    with pytest.raises(TypeError):
        DialogCore(llm=llm, tools=None, memory=memory, dsm=dsm)


def test_construction_rejects_missing_memory(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    dsm: DialogueStateMachine,
) -> None:
    """DialogCore must refuse to construct without a MemoryStore."""
    with pytest.raises(TypeError):
        DialogCore(llm=llm, tools=tools_provider, memory=None, dsm=dsm)


def test_construction_rejects_missing_dsm(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
) -> None:
    """DialogCore must refuse to construct without a DSM."""
    with pytest.raises(TypeError):
        DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=None)


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
    sent = llm.calls[0][0]  # (messages, tools) tuple — only need messages here
    # user msg, assistant msg, user msg
    assert len(sent) == 3
    assert sent[0].content == "earlier"
    assert sent[-1].content == "now"


def test_system_prompt_is_prepended_when_configured(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """System prompt must reach the LLM as the first message.

    Regression: dialogue_node loaded _system_prompt but never passed it
    to DialogCore — deepseek was running WITHOUT a system prompt, so
    every rule (ALWAYS execute_music_code, NO METALANGUAGE, ...) was
    silently dropped. Found via verbose LLM trace: messages had no
    [0] system entry. (#992)
    """
    obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        system_prompt="ТЫ ДИДЖЕЙ. ВСЕГДА вызывай execute_music_code.",
    )
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input("сыграй баха"))
    sent = llm.calls[0][0]
    assert sent[0].role == "system"
    assert "execute_music_code" in sent[0].content
    # user message follows the system prompt
    assert sent[1].role == "user"
    assert sent[1].content == "сыграй баха"


def test_process_input_wraps_llm_errors(core: DialogCore, llm: _FakeLLMProvider) -> None:
    """LLM exceptions are surfaced via DialogResult.error, not raised."""
    llm.error = ProviderError("boom")
    result = asyncio.run(core.process_input("hello", history=[]))
    assert result.error is not None
    assert isinstance(result.error, ProviderError)


# ---------------------------------------------------------------------------
# Issue #1077 / #1101 — speaker_context, dynamic_system, preclassified_event
# ---------------------------------------------------------------------------


def test_speaker_context_inserted_after_system_prompt(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """speaker_context (профиль спикера из scope=speaker:<tag>) вставляется
    system-сообщением сразу после основного системного промпта."""
    obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        system_prompt="БАЗОВЫЙ ПРОМПТ",
    )
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input(
        "привет",
        speaker_context="Контекст о собеседнике:\nСобеседника зовут Саша.",
    ))
    sent = llm.calls[0][0]
    assert sent[0].role == "system"
    assert sent[0].content == "БАЗОВЫЙ ПРОМПТ"
    assert sent[1].role == "system"
    assert "Саша" in sent[1].content
    assert sent[2].role == "user"


def test_speaker_context_without_system_prompt_is_first(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Если system_prompt не задан, speaker_context становится messages[0]."""
    obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input(
        "привет",
        speaker_context="Контекст о собеседнике:\nСобеседника зовут Пётр.",
    ))
    sent = llm.calls[0][0]
    assert sent[0].role == "system"
    assert "Пётр" in sent[0].content
    assert sent[1].role == "user"


def test_dynamic_system_inserted_with_speaker_context(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """dynamic_system (two-system-prompt pattern) вставляется вторым
    system-сообщением, speaker_context — третьим (до user)."""
    obj = DialogCore(
        llm=llm,
        tools=tools_provider,
        memory=memory,
        dsm=dsm,
        system_prompt="БАЗОВЫЙ ПРОМПТ",
    )
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input(
        "привет",
        speaker_context="Контекст о собеседнике:\nСобеседника зовут Саша.",
        dynamic_system="<system_context><user_profile><name>Саша</name></user_profile></system_context>",
    ))
    sent = llm.calls[0][0]
    assert sent[0].content == "БАЗОВЫЙ ПРОМПТ"
    # dynamic_system идёт сразу после базового промпта
    assert sent[1].role == "system"
    assert "<system_context>" in sent[1].content
    # speaker_context после dynamic_system, до user
    assert sent[2].role == "system"
    assert "Контекст о собеседнике" in sent[2].content
    assert sent[3].role == "user"
    assert sent[3].content == "привет"


def test_preclassified_event_skips_double_classification(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """preclassified_event=STT_RESULT — DialogCore НЕ переклассифицирует
    текст (в котором может быть 'робот' внутри) и НЕ делает повторный
    DSM-переход; LLM вызывается, результат DIALOGUE."""
    from rob_box_harness.core.dialogue_state_machine import DialogueEvent

    obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    # _on_stt в dialogue_node делает IDLE→LISTENING (WAKE_WORD) →
    # LISTENING→DIALOGUE (STT_RESULT) ДО вызова process_input, и передаёт
    # preclassified_event=STT_RESULT чтобы DialogCore не повторял
    # классификацию (текст «робот меня зовут...» содержит wake-word внутри).
    asyncio.run(obj.handle_wake_word(""))
    dsm.on_event(DialogueEvent.STT_RESULT)
    assert dsm.current_state == DialogueStateKind.DIALOGUE
    result = asyncio.run(obj.process_input(
        "робот меня зовут Саша",
        preclassified_event=DialogueEvent.STT_RESULT,
    ))
    assert len(llm.calls) == 1
    assert result.spoken_text == "hello back"
    # После хода DSM возвращается в IDLE (DIALOGUE_END) — это норма.
    assert result.new_state == DialogueStateKind.IDLE


def test_speaker_tag_persisted_in_turn_metadata(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Turn.metadata['speaker_tag'] проставляется для user и assistant ходов."""
    obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input("привет", speaker_tag="0"))
    user_turn = memory.turns[-2]
    assistant_turn = memory.turns[-1]
    assert user_turn.role == "user"
    assert user_turn.metadata == {"speaker_tag": "0"}
    assert assistant_turn.role == "assistant"
    assert assistant_turn.metadata == {"speaker_tag": "0"}


def test_speaker_tag_none_means_empty_metadata(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Без speaker_tag (Vosk fallback) metadata остаётся пустым — инвариант
    «каждый ход имеет metadata.speaker_tag (или None)»."""
    obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(obj.handle_wake_word(""))
    asyncio.run(obj.process_input("привет"))
    user_turn = memory.turns[-2]
    assert user_turn.metadata == {}


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
    sent = llm.calls[0][0]
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
    sent = llm.calls[0][0]
    assert len(sent) == 2
    assert sent[0].content == "explicit"
    assert sent[1].content == "now"
    # Explicit history → memory must NOT have been consulted.
    assert memory.load_recent_calls == []


def test_history_none_without_trim_limit_yields_empty(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """history=None with no trim limit → empty list, no memory call."""
    # No history_trim_limit → DialogCore returns [] without
    # consulting memory.
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))
    asyncio.run(core_obj.process_input("now", history=None))
    assert memory.load_recent_calls == []
    sent = llm.calls[0][0]
    # Only the user turn we just appended.
    assert len(sent) == 1
    assert sent[0].content == "now"


# ---------------------------------------------------------------------------
# Tool-loop behaviour — issue #916
# ---------------------------------------------------------------------------
#
# These tests cover the fix for krikz/rob_box_project#916: DialogCore
# never forwarded ``tools=`` to ``LLMProvider.complete()``, so the
# model never saw ``memory_context`` and memory-dependent scenarios
# (barge_in_joke_then_memory Step 6) failed. The fix passes the
# OpenAI-style tool schema on every complete() call and runs the
# tool loop in-core until the model returns plain text.
# ---------------------------------------------------------------------------


def test_process_input_passes_tools_to_llm(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """DialogCore forwards tools.discover() to llm.complete().

    Regression for issue #916: the model needs to *see* the tool
    schema in order to ever invoke memory_context.
    """
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    asyncio.run(core_obj.process_input("hi", history=[]))

    assert len(llm.calls) == 1
    _messages, tools_passed = llm.calls[0]
    tool_names = {t["function"]["name"] for t in tools_passed}
    assert tool_names == {"memory_context", "echo"}


def test_process_input_passes_openai_wire_shape(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Each tool dict is in the OpenAI Chat-Completions shape."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    asyncio.run(core_obj.process_input("hi", history=[]))

    _messages, tools_passed = llm.calls[0]
    assert tools_passed, "no tools passed at all"
    for tool in tools_passed:
        assert tool["type"] == "function"
        func = tool["function"]
        assert isinstance(func["name"], str) and func["name"]
        assert "parameters" in func
        assert func["parameters"]["type"] == "object"


def test_process_input_records_tools_called_on_plain_reply(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """When the LLM returns plain text, tools_called is empty."""
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("hi", history=[]))
    assert result.tools_called == []
    assert result.spoken_text == "hello back"


def test_process_input_runs_tool_loop_and_records_names(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Tool-loop happy path — reproduces barge_in_joke_then_memory Step 6.

    The scripted LLM returns a tool_call on the first turn, then a
    plain-text reply after seeing the tool result. DialogCore must
    execute the tool, feed the result back, and report the tool
    name on DialogResult.
    """
    # First call → ask for memory_context; second call → final answer.
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(
                    id="call_1",
                    name="memory_context",
                    arguments={"limit": 5},
                ),
            ),
        ),
        LLMResponse(content="Мы обсуждали тему X", tool_calls=()),
    ]
    llm.responses = scripted

    handler_calls: list[dict[str, object]] = []

    async def memory_handler(args: dict[str, object]) -> str:
        handler_calls.append(args)
        return "recent turns: ..."
    tools_provider._handler_map = {"memory_context": memory_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("о чём мы говорили?", history=[]))

    # Tool actually executed.
    assert handler_calls == [{"limit": 5}]
    assert len(tools_provider.executed) == 1

    # LLM called twice — first with tools, then after the tool result.
    assert len(llm.calls) == 2

    # Final result surfaces the tool name and the model's reply.
    assert result.spoken_text == "Мы обсуждали тему X"
    assert result.tools_called == ["memory_context"]
    assert result.error is None


def test_process_input_tool_loop_dedupes_repeated_tool_names(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """tools_called is unique even when the model asks twice."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="memory_context", arguments={"limit": 3}),
                ToolCall(id="c2", name="echo", arguments={"text": "a"}),
            ),
        ),
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c3", name="memory_context", arguments={"limit": 1}),
            ),
        ),
        LLMResponse(content="final", tool_calls=()),
    ]
    llm.responses = scripted

    async def echo(args: dict[str, object]) -> str:
        return f"echo:{args.get('text')}"
    async def memory_handler(args: dict[str, object]) -> str:
        return "ctx"
    tools_provider._handler_map = {
        "memory_context": memory_handler,
        "echo": echo,
    }

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("q", history=[]))
    # Ordered by first appearance; deduped across iterations.
    assert result.tools_called == ["memory_context", "echo"]


def test_process_input_tool_loop_caps_at_max_iterations(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """A runaway tool loop is bounded by _MAX_TOOL_ITERATIONS.

    Each scripted response asks for one more tool. After the cap is
    hit DialogCore returns the last spoken text and stops issuing
    further complete() calls.
    """
    from rob_box_harness.core import dialog_core as dc

    # 1 initial + cap more turns.
    iterations = dc._MAX_TOOL_ITERATIONS + 1
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(ToolCall(id=f"c{i}", name="echo",
                                 arguments={"text": str(i)}),),
        )
        for i in range(iterations)
    ]
    llm.responses = scripted

    async def echo(args: dict[str, object]) -> str:
        return f"e:{args.get('text')}"
    tools_provider._handler_map = {"echo": echo}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("q", history=[]))

    # Exactly 1 initial + cap retries = cap+1 complete() calls.
    assert len(llm.calls) == dc._MAX_TOOL_ITERATIONS + 1
    # No plain-text response was ever given → spoken_text is empty.
    assert result.spoken_text == ""
    # echo was the only tool invoked.
    assert result.tools_called == ["echo"]


def test_process_input_tool_loop_survives_tool_level_error(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """is_error=True from a handler does NOT abort the loop.

    The LLM gets the error string as a tool message and can
    correct itself on the next turn.
    """
    from rob_box_llm.provider import ToolResult

    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="memory_context", arguments={"limit": 5}),
            ),
        ),
        # Second complete: LLM sees the error string and corrects.
        LLMResponse(content="no memory found", tool_calls=()),
    ]
    llm.responses = scripted

    async def broken_handler(args: dict[str, object]) -> ToolResult:
        return ToolResult(
            tool_call_id="c1",
            content="backend unavailable",
            is_error=True,
        )
    tools_provider._handler_map = {"memory_context": broken_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("о чём?", history=[]))
    assert result.error is None
    assert result.spoken_text == "no memory found"
    assert result.tools_called == ["memory_context"]


def test_silent_done_first_response_triggers_corrective_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 — bare 'done' with no tools must be retried once.

    deepseek-v4-flash intermittently answers with the completion marker
    'done' as its FIRST response and no tool calls — the user hears
    nothing and nothing happens. DialogCore must inject a corrective
    user message and ask the LLM once more; the retry's real answer
    then reaches the user instead of the silent 'done'.
    """
    llm.responses = [
        LLMResponse(content="done", tool_calls=()),
        LLMResponse(content="Привет, Саша!", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("привет", history=[]))

    assert result.error is None
    assert result.spoken_text == "Привет, Саша!"
    assert result.tools_called == []
    # Two LLM calls: the silent 'done' + the corrective retry.
    assert len(llm.calls) == 2
    # The retry's message list carries the correction after the 'done' echo.
    retry_messages = llm.calls[1][0]
    assert retry_messages[-1].role == "user"
    assert "[SYSTEM CORRECTION]" in retry_messages[-1].content
    assert retry_messages[-2].role == "assistant"
    assert retry_messages[-2].content == "done"


def test_empty_first_response_triggers_corrective_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 — an EMPTY first payload is retried once too.

    The empty-content variant (stream ended with no deltas) must get
    the same corrective retry; no assistant echo is appended because
    there is no content to echo (OpenAI-compatible shape stays valid).
    """
    llm.responses = [
        LLMResponse(content="", tool_calls=()),
        LLMResponse(content="ok", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("привет", history=[]))

    assert result.error is None
    assert result.spoken_text == "ok"
    assert len(llm.calls) == 2
    retry_messages = llm.calls[1][0]
    # No assistant echo for the empty content — the correction is the
    # last message and follows the user turn directly.
    assert retry_messages[-1].role == "user"
    assert "[SYSTEM CORRECTION]" in retry_messages[-1].content


def test_empty_finish_reason_none_triggers_corrective_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 follow-up — finish_reason=None with empty content retries.

    e2e run on develop 79a284f showed ``Empty assistant response
    (LLM вернул пустоту): finish_reason=None tools=[] raw=''`` — the
    deepseek stream finished without a terminal finish_reason chunk, so
    the aggregator returned ``finish_reason=None`` instead of 'stop'.
    The corrective retry must treat that shape exactly like the bare
    'done' / empty-'stop' variants (it keys on empty content + no tool
    calls, NOT on finish_reason).
    """
    llm.responses = [
        LLMResponse(content="", tool_calls=(), finish_reason=None),
        LLMResponse(content="Сейчас пять часов", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("который час", history=[]))

    assert result.error is None
    assert result.spoken_text == "Сейчас пять часов"
    assert len(llm.calls) == 2
    retry_messages = llm.calls[1][0]
    assert retry_messages[-1].role == "user"
    assert "[SYSTEM CORRECTION]" in retry_messages[-1].content


def test_finish_reason_insufficient_system_resource_triggers_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1253 — DeepSeek insufficient_system_resource retries.

    DeepSeek documents ``finish_reason="insufficient_system_resource"``
    (HTTP 200, generation interrupted by provider resource pressure).
    Even when the aggregator carried a partial content fragment, the
    answer is truncated — the corrective retry must fire so the user
    gets a complete reply instead of a cut-off fragment.
    """
    llm.responses = [
        LLMResponse(
            content="Сейчас пять",
            tool_calls=(),
            finish_reason="insufficient_system_resource",
        ),
        LLMResponse(content="Сейчас пять часов", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("который час", history=[]))

    assert result.error is None
    assert result.spoken_text == "Сейчас пять часов"
    assert len(llm.calls) == 2
    retry_messages = llm.calls[1][0]
    assert retry_messages[-1].role == "user"
    assert "[SYSTEM CORRECTION]" in retry_messages[-1].content


def test_finish_reason_content_filter_triggers_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1253 — content_filter with partial content retries.

    ``finish_reason="content_filter"`` means output was omitted by a
    content filter — a partial fragment is not a usable answer.
    """
    llm.responses = [
        LLMResponse(
            content="извини, я не",
            tool_calls=(),
            finish_reason="content_filter",
        ),
        LLMResponse(content="Извини, я не могу это сделать", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("расскажи анекдот", history=[]))

    assert result.error is None
    assert result.spoken_text == "Извини, я не могу это сделать"
    assert len(llm.calls) == 2


def test_tool_error_babble_is_suppressed_system_transition(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1253 — tool error + babble-only answer is NOT voiced.

    A tool returned ``is_error=True`` and the LLM answered with ONLY a
    bare completion marker ('done') — no retry tool-call, no speak_text.
    Voicing that would make the robot say «дан» while nothing happened.
    DialogCore returns empty spoken (system transition) so dialogue_node
    moves to the next round instead of parroting the babble.
    """
    from rob_box_llm.provider import ToolResult

    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="memory_context", arguments={"limit": 5}),
            ),
        ),
        # LLM saw the error string but answered with a bare marker.
        LLMResponse(content="done", tool_calls=()),
    ]
    llm.responses = scripted

    async def broken_handler(args: dict[str, object]) -> ToolResult:
        return ToolResult(
            tool_call_id="c1",
            content="backend unavailable",
            is_error=True,
        )
    tools_provider._handler_map = {"memory_context": broken_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("о чём?", history=[]))

    assert result.error is None
    # Babble suppressed — empty spoken (system transition), no «дан».
    assert result.spoken_text == ""
    # The tool WAS attempted this turn.
    assert result.tools_called == ["memory_context"]
    assert len(llm.calls) == 2


def test_tool_error_substantive_answer_not_suppressed(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1253 — substantive text after tool error is still voiced.

    A tool returned ``is_error=True`` and the LLM answered with a real
    plain-text reply ("no memory found"). That is NOT babble — the user
    gets an explanation, not silence.
    """
    from rob_box_llm.provider import ToolResult

    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="memory_context", arguments={"limit": 5}),
            ),
        ),
        LLMResponse(content="no memory found", tool_calls=()),
    ]
    llm.responses = scripted

    async def broken_handler(args: dict[str, object]) -> ToolResult:
        return ToolResult(
            tool_call_id="c1",
            content="backend unavailable",
            is_error=True,
        )
    tools_provider._handler_map = {"memory_context": broken_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("о чём?", history=[]))

    assert result.error is None
    assert result.spoken_text == "no memory found"
    assert result.tools_called == ["memory_context"]


def test_dj_auto_with_preclassified_event_reaches_llm_from_idle(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 — DJ-auto turn with preclassified_event must reach the LLM.

    Regression from #1101: ``_run_turn`` passes
    ``preclassified_event=STT_RESULT`` for every turn, so ``process_input``
    skipped its own DSM transition (the ``pass`` in the preclassified
    branch). DJ auto-turns do NOT come through ``_on_stt`` (which drives
    IDLE→LISTENING→DIALOGUE) — they are dispatched straight from the
    DJ tick. After a previous turn left the DSM in IDLE (DIALOGUE_END),
    the LLM gate ``current_state == DIALOGUE`` silently dropped the turn:
    the LLM was never called and the robot answered nothing in ~2 ms
    (e2e run4: 4/4 DJ-auto turns ``spoken='' tools=[] finish_reason=None``
    with zero ``[health] stream`` log lines).
    """
    llm.responses = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="echo", arguments={"text": "dj"}),
            ),
        ),
        LLMResponse(content="done", tool_calls=()),
    ]
    async def echo(args: dict[str, object]) -> str:
        return f"e:{args.get('text')}"
    tools_provider._handler_map = {"echo": echo}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    # DSM starts in IDLE (default) — no wake word fired for a DJ tick.
    assert dsm.current_state == DialogueStateKind.IDLE

    result = asyncio.run(core_obj.process_input(
        "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] ...",
        history=[],
        is_dj_auto=True,
        preclassified_event=DialogueEvent.STT_RESULT,
    ))

    # The turn must have reached the LLM and driven DSM into DIALOGUE.
    assert result.error is None
    assert len(llm.calls) == 2  # tool turn + final 'done'
    assert result.tools_called == ["echo"]
    assert result.spoken_text == "done"
    assert dsm.current_state == DialogueStateKind.IDLE  # DIALOGUE_END ran


def test_silent_done_retry_does_not_loop(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 — a second silent 'done' is accepted, not retried again.

    The corrective retry is one-shot: if the model STILL returns a bare
    marker, DialogCore returns it as-is so the shell's existing
    empty-response fallback (e.g. «Принял.») can handle it. No infinite
    LLM ping-pong.
    """
    llm.responses = [
        LLMResponse(content="done", tool_calls=()),
        LLMResponse(content="done", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("привет", history=[]))

    assert len(llm.calls) == 2
    assert result.spoken_text == "done"
    assert result.error is None


def test_tool_then_done_does_not_retry(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """A legitimate 'done' AFTER a tool call must NOT be retried.

    The master-prompt contract: the model calls speak_text (etc.), then
    answers 'done'. Because a tool ran in this turn, the final marker is
    valid — the corrective retry must not fire (it would confuse the model
    with a false «твой ответ был пустым» and burn an extra LLM call).
    """
    llm.responses = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="echo", arguments={"text": "hi"}),
            ),
        ),
        LLMResponse(content="done", tool_calls=()),
    ]
    async def echo(args: dict[str, object]) -> str:
        return f"e:{args.get('text')}"
    tools_provider._handler_map = {"echo": echo}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("привет", history=[]))

    # Tool + final marker = exactly 2 LLM calls, no corrective retry.
    assert len(llm.calls) == 2
    assert result.spoken_text == "done"
    assert result.tools_called == ["echo"]
    # The final call's messages must NOT contain the correction.
    final_messages = llm.calls[1][0]
    assert not any("[SYSTEM CORRECTION]" in str(m.content) for m in final_messages)


def test_normal_plain_reply_does_not_retry(
    core: DialogCore,
    llm: _FakeLLMProvider,
) -> None:
    """A substantive plain-text reply must NOT trigger the silent-retry."""
    # Default fake returns "hello back" — a real answer.
    result = asyncio.run(core.process_input("hello", history=[]))
    assert result.spoken_text == "hello back"
    assert len(llm.calls) == 1


def test_silent_failure_turn_not_persisted_to_memory(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Issue #1217 — a silent-failure assistant turn stays OUT of memory.

    When the corrective retry also fails (second bare 'done'), the turn
    produced no real assistant reply. Persisting it would teach the model
    that 'done' is a normal reply and the next history would mimic it.
    The user turn IS stored (real input), the failed assistant reply is not.
    """
    llm.responses = [
        LLMResponse(content="done", tool_calls=()),
        LLMResponse(content="done", tool_calls=()),
    ]
    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("привет", history=[]))
    assert result.spoken_text == "done"

    roles = [t.role for t in memory.turns]
    # Only the user turn was persisted — no synthetic 'done' assistant turn.
    assert roles == ["user"]


def test_normal_turn_still_persisted_to_memory(
    core: DialogCore,
    memory: _FakeMemoryStore,
) -> None:
    """A healthy turn (real reply) is still persisted as user+assistant."""
    asyncio.run(core.process_input("hello", history=[]))
    roles = [t.role for t in memory.turns]
    assert roles == ["user", "assistant"]
    assert memory.turns[-1].content == "hello back"


def test_process_input_tool_loop_aborts_on_transport_error(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """A transport-level ToolExecutionError is wrapped into result.error."""
    from rob_box_harness.tools import ToolExecutionError

    llm.responses = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="memory_context", arguments={}),
            ),
        ),
    ]

    async def raise_exec(args: dict[str, object]) -> str:
        raise ToolExecutionError("bridge down", provider="ros_mcp")
    tools_provider._handler_map = {"memory_context": raise_exec}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("о чём?", history=[]))
    assert isinstance(result.error, ToolExecutionError)
    assert result.spoken_text == ""
    # Only the user turn made it to memory.
    assert [t.role for t in memory.turns] == ["user"]


def test_process_input_tool_loop_keeps_user_turn_once_on_error(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """User turn is persisted before the LLM call and never re-added on error."""
    llm.responses = [ProviderError("network down")]

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("hello", history=[]))
    assert isinstance(result.error, ProviderError)
    # Exactly one user turn in memory — no duplicate row.
    assert len(memory.turns) == 1
    assert memory.turns[0].role == "user"
    assert memory.turns[0].content == "hello"


# ---------------------------------------------------------------------------
# Issue #1343 — speak_text_real_count (real vs phantom speak_text calls)
# ---------------------------------------------------------------------------
#
# deepseek sometimes emits ``speak_text({})`` / ``speak_text({"text": ""})``
# with EMPTY text. The tool name lands in ``tools_called``, but validation
# rejects the call and NOTHING is voiced. dialogue_node's issue-988
# anti-duplicate guard must skip auto-TTS only when speech REALLY happened
# (``speak_text_real_count > 0``); a phantom call must NOT silence the final
# text. These tests verify DialogCore computes the real count correctly.


def test_speak_text_real_count_counts_nonempty_text_calls(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """Non-empty speak_text → real_count == 1; phantom variants → not counted."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="speak_text", arguments={"text": "Жил да был енот"}),
                ToolCall(id="c2", name="speak_text", arguments={"text": ""}),
                ToolCall(id="c3", name="speak_text", arguments={}),
            ),
        ),
        LLMResponse(content="done", tool_calls=()),
    ]
    llm.responses = scripted

    async def speak_handler(args: dict[str, object]) -> str:
        return "TTS запрос отправлен: ok"

    tools_provider._handler_map = {"speak_text": speak_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    # NOTE: no wake word in the input — «робот» would classify as
    # WAKE_WORD and skip the LLM gate (see process_input docstring).
    result = asyncio.run(core_obj.process_input("спой песенку", history=[]))

    # All three names were requested → tools_called has the name (deduped).
    assert result.tools_called == ["speak_text"]
    # Only the FIRST call carried a non-empty text → real count is 1.
    assert result.speak_text_count == 3
    assert result.speak_text_real_count == 1


def test_speak_text_real_count_zero_for_all_phantom_calls(
    llm: _FakeLLMProvider,
    tools_provider: _FakeToolProvider,
    memory: _FakeMemoryStore,
    dsm: DialogueStateMachine,
) -> None:
    """All speak_text calls empty → real_count == 0 (silence-after-accept bug)."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(
                ToolCall(id="c1", name="speak_text", arguments={"text": ""}),
                ToolCall(id="c2", name="speak_text", arguments={}),
            ),
        ),
        LLMResponse(content="Это ошибка — пустой вызов. Выполню правильно.", tool_calls=()),
    ]
    llm.responses = scripted

    async def speak_handler(args: dict[str, object]) -> str:
        return "TTS запрос отправлен: ok"

    tools_provider._handler_map = {"speak_text": speak_handler}

    core_obj = DialogCore(llm=llm, tools=tools_provider, memory=memory, dsm=dsm)
    asyncio.run(core_obj.handle_wake_word(""))

    result = asyncio.run(core_obj.process_input("это иван а теперь пожалуйста мне на денчика", history=[]))

    assert result.tools_called == ["speak_text"]
    assert result.speak_text_count == 2
    assert result.speak_text_real_count == 0
    # The final spoken text is still the real answer for dialogue_node to voice.
    assert result.spoken_text == "Это ошибка — пустой вызов. Выполню правильно."


# ---------------------------------------------------------------------------
# W7a — batch re-ordering (_order_tool_calls)
# ---------------------------------------------------------------------------


def _call(cid: str, name: str) -> ToolCall:
    return ToolCall(id=cid, name=name, arguments={})


def test_order_tool_calls_puts_stop_music_after_speak_text() -> None:
    """[speak_text, stop_music] keeps order but flags stop as deferred."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    ordered, deferred = _order_tool_calls(
        [_call("c1", "speak_text"), _call("c2", "stop_music")]
    )
    assert [c.name for c in ordered] == ["speak_text", "stop_music"]
    # stop_music must wait for the voice channel to drain.
    assert deferred == {"c2"}


def test_order_tool_calls_music_prelude_before_speak_text() -> None:
    """Music tools run before voice; destructive tools run last."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    ordered, deferred = _order_tool_calls(
        [
            _call("c1", "stop_music"),
            _call("c2", "speak_text"),
            _call("c3", "execute_music_code"),
        ]
    )
    assert [c.name for c in ordered] == [
        "execute_music_code",
        "speak_text",
        "stop_music",
    ]
    assert deferred == {"c1"}


def test_order_tool_calls_stop_music_alone_not_deferred() -> None:
    """A lone stop_music has no voice to wait for — not deferred."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    ordered, deferred = _order_tool_calls([_call("c1", "stop_music")])
    assert [c.name for c in ordered] == ["stop_music"]
    assert deferred == set()


def test_order_tool_calls_independent_tools_keep_relative_order() -> None:
    """Bypass tools (memory_save etc.) keep their original relative order."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    ordered, deferred = _order_tool_calls(
        [
            _call("c1", "memory_save"),
            _call("c2", "speak_text"),
            _call("c3", "search_web"),
        ]
    )
    assert [c.name for c in ordered] == [
        "memory_save",
        "speak_text",
        "search_web",
    ]
    assert deferred == set()


def test_order_tool_calls_stop_navigation_is_destructive_too() -> None:
    """stop_navigation is treated like stop_music (defer to end)."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    ordered, deferred = _order_tool_calls(
        [_call("c1", "stop_navigation"), _call("c2", "speak_text")]
    )
    assert [c.name for c in ordered] == ["speak_text", "stop_navigation"]
    assert deferred == {"c1"}


def test_run_with_tools_executes_in_safe_order_but_returns_original_order() -> None:
    """Execution re-orders, but tool-result messages keep the LLM's order."""
    from rob_box_harness.core.dialog_core import _order_tool_calls

    calls = [
        _call("c1", "stop_music"),
        _call("c2", "speak_text"),
    ]
    ordered, deferred = _order_tool_calls(calls)
    assert [c.name for c in ordered] == ["speak_text", "stop_music"]
    # And the original tuple is untouched (frozen dataclass semantics).
    assert [c.name for c in calls] == ["stop_music", "speak_text"]


# ---------------------------------------------------------------------------
# Issue #1280 — LLM stream must be aborted on barge-in (new STT input)
# ---------------------------------------------------------------------------


class _TrackedStreamIterator:
    """AsyncIterator (NOT an async generator) with an observable ``aclose``.

    The real production stream (OpenAI SDK ``AsyncStream``) is not an
    async generator — cancelling the consuming task does NOT close it
    automatically. ``DialogCore._stream_response`` must call
    ``aclose()`` explicitly in its ``finally`` (issue #1280), otherwise
    the HTTP request to the LLM provider keeps running to the end of
    generation (wasted quota, "robot finishes the old topic").
    """

    def __init__(self) -> None:
        self.closed = False
        self.aclose_calls = 0
        self._first = True
        self._blocker = asyncio.Event()  # never set → __anext__ blocks forever

    def __aiter__(self) -> "_TrackedStreamIterator":
        return self

    async def __anext__(self) -> Any:
        # Yield one chunk, then block forever — the consuming task
        # suspends inside the stream, which is exactly the barge-in
        # window we want to test.
        if self._first:
            self._first = False
            return LLMChunk(content_delta="partial-old-topic")
        await self._blocker.wait()  # pragma: no cover — never returns
        raise StopAsyncIteration  # pragma: no cover — unreachable

    async def aclose(self) -> None:
        self.closed = True
        self.aclose_calls += 1


class _FakeStreamingLLM:
    """Minimal LLMProvider double that ONLY supports ``stream()``.

    ``stream()`` returns a :class:`_TrackedStreamIterator` so the test
    can observe whether the core closed the stream on cancellation.
    ``complete()`` must never be called in streaming mode.
    """

    name = "fake_streaming_llm"

    def __init__(self) -> None:
        self.stream_obj: _TrackedStreamIterator | None = None

    def stream(self, messages: Any, tools: Any = ()) -> _TrackedStreamIterator:
        self.stream_obj = _TrackedStreamIterator()
        return self.stream_obj

    async def complete(self, *args: Any, **kwargs: Any) -> Any:
        raise AssertionError("complete() must not be called with use_streaming=True")

    async def aclose(self) -> None:
        return None


def test_stream_response_aborts_and_closes_stream_on_barge_in() -> None:
    """Issue #1280 — cancelling a run mid-stream must abort the LLM.

    Regression: before the fix, ``_stream_response`` iterated the stream
    without a ``finally`` — cancellation left the HTTP stream open, the
    provider kept generating the old topic (wasted quota) and the old
    answer could still reach TTS after the user asked a new question.
    """
    llm = _FakeStreamingLLM()

    async def scenario() -> None:
        core_obj = DialogCore(
            llm=llm,
            tools=_FakeToolProvider(),
            memory=_FakeMemoryStore(),
            dsm=DialogueStateMachine(),
            use_streaming=True,
        )
        await core_obj.handle_wake_word("")
        task = asyncio.create_task(core_obj.process_input("hello", history=[]))
        # Let the task reach the stream and suspend inside __anext__.
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        assert llm.stream_obj is not None
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        # The stream must be actively closed — otherwise the HTTP
        # request to the provider keeps burning quota on the old topic.
        assert llm.stream_obj.closed
        assert llm.stream_obj.aclose_calls >= 1

    asyncio.run(scenario())


def test_stream_response_cancelled_task_does_not_return_partial_answer() -> None:
    """Issue #1280 — barge-in must NOT surface a partial old-topic answer.

    When the run is cancelled while the stream is still producing, the
    turn must end with ``CancelledError`` (the shell's barge-in path),
    never with a partial ``DialogResult`` that would be voiced.
    """
    llm = _FakeStreamingLLM()

    async def scenario() -> None:
        core_obj = DialogCore(
            llm=llm,
            tools=_FakeToolProvider(),
            memory=_FakeMemoryStore(),
            dsm=DialogueStateMachine(),
            use_streaming=True,
        )
        await core_obj.handle_wake_word("")
        task = asyncio.create_task(core_obj.process_input("hello", history=[]))
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        # The stream was closed AND no partial content was aggregated.
        assert llm.stream_obj is not None
        assert llm.stream_obj.closed

    asyncio.run(scenario())
