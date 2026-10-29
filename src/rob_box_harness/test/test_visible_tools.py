"""Срез тулов хода в AgentCore (ADR-0150 PR-6 #3696): код задаёт схемы до хода, подгрузка скиллов моделью удалена."""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, Mapping

import pytest

from rob_box_core.tool_catalog import llm_visible_tools
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider, ToolProvider
from rob_box_llm.provider import LLMChunk, LLMResponse, ToolCall, ToolResult

class _CatalogProvider(ToolProvider):
    """ToolProvider, отдающий реальный каталог робота."""

    name = "catalog"

    def __init__(self) -> None:
        self._registry = ToolRegistry()

    async def discover(self):
        return self._registry.list_tools()

    async def execute(self, call: ToolCall) -> ToolResult:
        return ToolResult(tool_call_id=call.id, content="ok", is_error=False)


class _Scripted:
    name = "scripted"

    def __init__(self, *responses: LLMResponse) -> None:
        self._responses = list(responses)
        self.offered: list[list[str]] = []

    def _next(self, tools) -> LLMResponse:
        self.offered.append(
            [t["function"]["name"] for t in tools]
        )
        if self._responses:
            return self._responses.pop(0)
        return LLMResponse(content="готово", finish_reason="stop")

    async def complete(self, messages, *, tools=(), **_: Any) -> LLMResponse:
        return self._next(list(tools))

    async def stream(self, messages, *, tools=(), **_: Any) -> AsyncIterator[LLMChunk]:
        response = self._next(list(tools))
        for call in response.tool_calls:
            yield LLMChunk(content_delta="", tool_call_delta=call)
        yield LLMChunk(
            content_delta=response.content or "",
            finish_reason=response.finish_reason or "stop",
        )


def _dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


def _core(provider) -> AgentCore:
    return AgentCore(
        llm=provider, tools=_CatalogProvider(), memory=InMemoryStore(), dsm=_dsm(),
        system_prompt="ты РОББОКС", use_streaming=False,
    )


def _run(core: AgentCore, text: str = "сыграй бит") -> Any:
    return asyncio.run(core.process_input(text, preclassified_event=DialogueEvent.STT_RESULT))


def test_without_a_view_the_whole_catalog_is_offered() -> None:
    """Среза нет (ТАРС, тесты) — весь каталог провайдера, как раньше; тула подгрузки скилла нет."""
    provider = _Scripted()
    _run(_core(provider))
    assert set(provider.offered[0]) == {e.name for e in llm_visible_tools()}


def test_view_narrows_every_iteration_of_the_turn() -> None:
    first = LLMResponse(content="", tool_calls=(ToolCall(id="1", name="get_current_time", arguments={}),),
                        finish_reason="tool_calls")
    provider = _Scripted(first)
    core = _core(provider)
    core.set_visible_tools(["speak_text", "get_current_time"], "status")
    _run(core, "который час")
    assert len(provider.offered) == 2
    assert all(set(offered) == {"speak_text", "get_current_time"} for offered in provider.offered)


def _stats_core(provider, seen) -> AgentCore:
    return AgentCore(llm=provider, tools=_CatalogProvider(), memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты", use_streaming=False, on_prompt=seen.append)


def test_view_labels_prompt_stats() -> None:
    seen = []
    provider = _Scripted()
    core = _stats_core(provider, seen)
    core.set_visible_tools(["speak_text"], "music+voice")
    _run(core)
    assert provider.offered and all(offered == ["speak_text"] for offered in provider.offered)
    assert seen and {s.skill for s in seen} == {"music+voice"}


def test_view_can_be_reset_to_the_whole_catalog() -> None:
    seen = []
    provider = _Scripted()
    core = _stats_core(provider, seen)
    core.set_visible_tools(["speak_text"], "status")
    core.set_visible_tools(None)
    _run(core)
    assert provider.offered and all(len(offered) == len(llm_visible_tools()) for offered in provider.offered)
    assert {s.skill for s in seen} == {"all"}


def test_skill_loading_tool_is_gone() -> None:
    import rob_box_harness.core.agent_core as ac

    assert not hasattr(ac, "LOAD_SKILL_TOOL") and not hasattr(AgentCore, "set_active_skill")
