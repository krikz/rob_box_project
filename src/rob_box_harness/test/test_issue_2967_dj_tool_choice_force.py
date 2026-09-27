"""test_issue_2967_dj_tool_choice_force.py — issue #2967 part 2.

Живая сессия 24.09.2026 (DJ «Сапог», minimax): 2-3 DJ auto-transition
подряд отвечали ТОЛЬКО текстом, без единого вызова тула — до сих пор это
чинилось только ПОСТ-ФАКТУМ, синхронным ретраем ``MusicGuard`` (Bug B,
issue #992). Systemic fix (товарищ Шифу, 24.09.2026 — опираться на факт
вызова тула, а не на текст): ``AgentCore._run_with_tools`` теперь
поддерживает ``force_tool_choice`` — форсирует ``tool_choice`` на ПЕРВОМ
LLM-запросе хода, и ``process_input(is_dj_auto=True)`` форсирует
``"required"`` для КАЖДОГО DJ auto-transition (включая Bug-B синхронные
ретраи — они тоже несут ``is_dj_auto=True``, см.
``dialogue_node._dispatch_dj_turn``). Провайдер отвечает текстом БЕЗ
тула уже реже, потому что ему не оставлено такой возможности на первом
запросе цикла.

Not verified against a live MiniMax API call in this PR — that would
require raw evidence from an actual API round-trip, which this
unit-level test cannot provide. What IS verified here (and provable
without a live call): the harness computes ``tool_choice="required"``
and puts it on the wire-level ``LLMSettings`` for the FIRST completion
of a DJ turn only, preserves any other configured settings (e.g.
``temperature``), and does NOT force it on the continuation call after
a tool has already run (that would break the "finish with plain text /
speak_text" cycle-end contract).
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_llm.provider import LLMResponse, LLMSettings, ToolCall, ToolResult


class _RecordingLLM:
    """Records every ``complete()`` call's ``settings=`` argument."""

    name = "fake_llm"

    def __init__(self, responses: list[Any]) -> None:
        self.responses = list(responses)
        self.settings_calls: list[Any] = []

    async def complete(
        self,
        messages: Any = None,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> LLMResponse:
        self.settings_calls.append(settings)
        item = self.responses.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    async def aclose(self) -> None:
        return None


class _FakeToolProvider:
    name = "fake_tools"

    def __init__(self) -> None:
        from rob_box_harness.tools import ToolSpec

        self._manifest = (
            ToolSpec(
                name="compose_music",
                description="Start a track.",
                parameters={
                    "type": "object",
                    "properties": {"name": {"type": "string"}},
                },
            ),
        )
        self.executed: list[Any] = []

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> ToolResult:
        self.executed.append(call)
        return ToolResult(tool_call_id=call.id, content="ok", is_error=False)

    async def aclose(self) -> None:
        return None


class _FakeMemoryStore:
    async def save_fact(self, scope: str, fact: Any) -> None:
        return None

    async def search_facts(self, scope: str, query: str, limit: int = 5) -> list[Any]:
        return []


def _wake(core: AgentCore) -> None:
    core._dsm.on_event(DialogueEvent.WAKE_WORD)


def test_dj_auto_first_call_forces_tool_choice_required() -> None:
    """``is_dj_auto=True`` → first ``complete()`` carries
    ``tool_choice="required"``."""
    llm = _RecordingLLM(
        [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(id="c1", name="compose_music", arguments={"name": "x"}),
                ),
            ),
            LLMResponse(content="done", tool_calls=()),
        ]
    )
    core_obj = AgentCore(
        llm=llm, tools=_FakeToolProvider(), memory=_FakeMemoryStore(),
        dsm=DialogueStateMachine(),
    )

    result = asyncio.run(
        core_obj.process_input("[DJ_AUTO] переход #2", is_dj_auto=True)
    )

    assert result.error is None
    assert len(llm.settings_calls) == 2
    first, second = llm.settings_calls
    assert first is not None
    assert first.tool_choice == "required"
    # Issue #2967 — the CONTINUATION call (after compose_music already
    # ran) must NOT be forced, or the model could never finish the
    # cycle with plain text / a single speak_text per the master-prompt
    # contract.
    assert second is None or getattr(second, "tool_choice", None) is None


def test_normal_user_turn_does_not_force_tool_choice() -> None:
    """``is_dj_auto=False`` (ordinary user turn) — ``tool_choice`` untouched."""
    llm = _RecordingLLM([LLMResponse(content="hello", tool_calls=())])
    core_obj = AgentCore(
        llm=llm, tools=_FakeToolProvider(), memory=_FakeMemoryStore(),
        dsm=DialogueStateMachine(),
    )
    _wake(core_obj)

    result = asyncio.run(core_obj.process_input("привет", history=[]))

    assert result.error is None
    assert llm.settings_calls == [None]


def test_dj_auto_preserves_configured_llm_settings() -> None:
    """Forcing ``tool_choice`` must not drop ``temperature`` / ``max_tokens``
    configured via ``AgentCore(llm_settings=...)`` (issue #1883 contract)."""
    base_settings = LLMSettings(temperature=0.3, max_tokens=250)
    llm = _RecordingLLM([LLMResponse(content="done", tool_calls=())])
    core_obj = AgentCore(
        llm=llm, tools=_FakeToolProvider(), memory=_FakeMemoryStore(),
        dsm=DialogueStateMachine(), llm_settings=base_settings,
    )

    asyncio.run(core_obj.process_input("[DJ_AUTO] переход #1", is_dj_auto=True))

    used = llm.settings_calls[0]
    assert used.tool_choice == "required"
    assert used.temperature == 0.3
    assert used.max_tokens == 250
    # The instance-level settings object itself must stay untouched —
    # ``replace()`` returns a NEW instance, not a mutation.
    assert base_settings.tool_choice is None
