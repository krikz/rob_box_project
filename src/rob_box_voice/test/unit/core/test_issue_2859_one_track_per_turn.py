"""Issue #2859 — не больше одного успешного запуска трека за ход.

Живой DJ-переход #23 (23.09.2026): LLM в одном ходе 8 раз вызвала
``compose_music`` + ``set_dj_mode``; каждый вызов — ``Clock.clear()`` и
новый трек, музыка переключалась каждые ~3 с до ``_MAX_TOOL_ITERATIONS``.

Покрытие:

* чистый :class:`TrackStartGuard` (успех → лимит, провал → повтор разрешён,
  ``reset`` → новый ход);
* :class:`SchedulerToolExecutor` — отказ не доходит до провайдера;
* сквозной сценарий через настоящий ``AgentCore`` со скриптованной LLM:
  8 запусков в одном ходе → выполнен ровно 1, 7 отказов с текстом.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.tools import ToolSpec
from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult
from rob_box_voice.core.track_start_guard import (
    REFUSAL_ERROR_CODE,
    REFUSAL_MESSAGE,
    TRACK_STARTING_TOOLS,
    TrackStartGuard,
)
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class _FakeUnderlying:
    """Провайдер тулов: пишет исполненные вызовы, может «уронить» первые N."""

    def __init__(self, fail_first: int = 0) -> None:
        self.executed: list[ToolCall] = []
        self._fail_left = fail_first

    async def discover(self) -> tuple[ToolSpec, ...]:
        obj = {"type": "object", "properties": {}}
        return tuple(
            ToolSpec(name=n, description=n, parameters=obj)
            for n in ("compose_music", "set_dj_mode", "execute_music_code")
        )

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call)
        if call.name == "compose_music" and self._fail_left > 0:
            self._fail_left -= 1
            return ToolResult(
                tool_call_id=call.id, content="scsynth down", is_error=True
            )
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"success": True, "tool": call.name}),
            is_error=False,
        )

    async def aclose(self) -> None:
        return None

    def names(self) -> list[str]:
        return [c.name for c in self.executed]


class _ScriptedLLM:
    """Отдаёт заготовленные ответы по одному; дальше — финальный текст."""

    name = "scripted"

    def __init__(self, responses: list[LLMResponse]) -> None:
        self.responses = list(responses)
        self.calls: list[list[Any]] = []

    async def complete(self, messages: Any = None, **_kw: Any) -> LLMResponse:
        self.calls.append(list(messages or []))
        if self.responses:
            return self.responses.pop(0)
        return LLMResponse(content="Йоу, качаем дальше!", tool_calls=())

    async def aclose(self) -> None:
        return None


class _NullMemory:
    async def save_fact(self, scope: str, fact: Any) -> None:
        return None

    async def search_facts(self, scope: str, query: str, limit: int = 5):
        return []

    async def aclose(self) -> None:
        return None


def _dj_batch(i: int) -> LLMResponse:
    """Один шаг живого хода #23: compose_music + set_dj_mode."""
    return LLMResponse(
        content="",
        tool_calls=(
            ToolCall(id=f"cm{i}", name="compose_music",
                     arguments={"name": f"still dre {i}"}),
            ToolCall(id=f"dj{i}", name="set_dj_mode",
                     arguments={"enabled": True, "interval": 75}),
        ),
    )


def _core(llm: _ScriptedLLM, executor: SchedulerToolExecutor) -> AgentCore:
    dsm = DialogueStateMachine()
    core = AgentCore(llm=llm, tools=executor, memory=_NullMemory(), dsm=dsm)
    dsm.on_event(DialogueEvent.WAKE_WORD)
    return core


def _tool_messages(llm: _ScriptedLLM) -> list[str]:
    """Тексты tool-сообщений из последнего запроса к LLM."""
    last = llm.calls[-1]
    return [m.content for m in last if getattr(m, "role", "") == "tool"]


def _refusals(contents: list[str]) -> list[dict]:
    out = []
    for text in contents:
        try:
            payload = json.loads(text)
        except (TypeError, ValueError):
            continue
        if payload.get("error") == REFUSAL_ERROR_CODE:
            out.append(payload)
    return out


# ---------------------------------------------------------------------------
# Pure guard
# ---------------------------------------------------------------------------


def test_track_starting_tools_cover_issue_list() -> None:
    expected = {
        "compose_music", "execute_music_code", "load_track",
        "gen_play_from_library",
    }
    assert expected <= TRACK_STARTING_TOOLS
    assert "set_dj_mode" not in TRACK_STARTING_TOOLS
    assert "lookup_melody" not in TRACK_STARTING_TOOLS


def test_guard_allows_first_success_then_refuses() -> None:
    guard = TrackStartGuard()
    assert not guard.should_refuse("compose_music")
    guard.record("compose_music", is_error=False)
    assert guard.should_refuse("compose_music")
    assert guard.should_refuse("load_track")
    assert not guard.should_refuse("set_dj_mode")
    assert guard.started_tool == "compose_music"


def test_guard_failed_start_does_not_consume_limit() -> None:
    guard = TrackStartGuard()
    guard.record("compose_music", is_error=True)
    assert not guard.should_refuse("compose_music")


def test_guard_ignores_non_music_success() -> None:
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=False)
    assert not guard.should_refuse("compose_music")


def test_guard_reset_opens_next_turn() -> None:
    guard = TrackStartGuard()
    guard.record("execute_music_code", is_error=False)
    guard.reset()
    assert not guard.should_refuse("execute_music_code")
    assert guard.started_tool is None


# ---------------------------------------------------------------------------
# SchedulerToolExecutor wiring
# ---------------------------------------------------------------------------


def test_executor_refuses_second_start_without_calling_provider() -> None:
    underlying = _FakeUnderlying()
    executor = SchedulerToolExecutor(underlying)

    async def _go() -> tuple[ToolResult, ToolResult]:
        executor.begin_turn()
        first = await executor.execute(
            ToolCall(id="a", name="compose_music", arguments={}))
        second = await executor.execute(
            ToolCall(id="b", name="load_track", arguments={}))
        return first, second

    first, second = asyncio.run(_go())
    assert underlying.names() == ["compose_music"]
    assert json.loads(first.content)["success"] is True
    payload = json.loads(second.content)
    assert second.tool_call_id == "b"
    assert second.is_error is False
    assert payload["success"] is False
    assert payload["error"] == REFUSAL_ERROR_CODE
    assert payload["message"] == REFUSAL_MESSAGE
    assert payload["already_started_by"] == "compose_music"


# ---------------------------------------------------------------------------
# End-to-end through AgentCore (scripted LLM)
# ---------------------------------------------------------------------------


def test_eight_compose_calls_in_one_turn_execute_exactly_one() -> None:
    """Живой ход #23: 8 × (compose_music + set_dj_mode) → 1 трек."""
    underlying = _FakeUnderlying()
    executor = SchedulerToolExecutor(underlying)
    llm = _ScriptedLLM([_dj_batch(i) for i in range(8)])
    core = _core(llm, executor)

    asyncio.run(core.process_input("[DJ_AUTO переход #23]", history=[]))

    assert underlying.names().count("compose_music") == 1
    assert underlying.names().count("set_dj_mode") == 8
    refusals = _refusals(_tool_messages(llm))
    assert len(refusals) == 7
    assert all(r["message"] == REFUSAL_MESSAGE for r in refusals)
    # Первый (исполненный) трек — именно первый вызов модели.
    first = next(c for c in underlying.executed if c.name == "compose_music")
    assert first.id == "cm0"


def test_failed_start_then_retry_executes_both() -> None:
    underlying = _FakeUnderlying(fail_first=1)
    executor = SchedulerToolExecutor(underlying)
    llm = _ScriptedLLM([_dj_batch(i) for i in range(3)])
    core = _core(llm, executor)

    asyncio.run(core.process_input("[DJ_AUTO переход #24]", history=[]))

    # cm0 упал, cm1 — исправление (выполнен), cm2 — отказ.
    assert underlying.names().count("compose_music") == 2
    assert len(_refusals(_tool_messages(llm))) == 1


def test_next_turn_allows_new_track() -> None:
    underlying = _FakeUnderlying()
    executor = SchedulerToolExecutor(underlying)
    llm = _ScriptedLLM([_dj_batch(0), _dj_batch(1)])
    core = _core(llm, executor)

    asyncio.run(core.process_input("[DJ_AUTO переход #25]", history=[]))
    assert underlying.names().count("compose_music") == 1

    llm.responses = [_dj_batch(2)]
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    asyncio.run(core.process_input("[DJ_AUTO переход #26]", history=[]))

    started = [c.id for c in underlying.executed if c.name == "compose_music"]
    assert started == ["cm0", "cm2"]


@pytest.mark.parametrize("tool", sorted(TRACK_STARTING_TOOLS))
def test_every_track_starter_is_limited(tool: str) -> None:
    underlying = _FakeUnderlying()
    executor = SchedulerToolExecutor(underlying)

    async def _go() -> None:
        executor.begin_turn()
        await executor.execute(ToolCall(id="x", name=tool, arguments={}))
        await executor.execute(ToolCall(id="y", name=tool, arguments={}))

    asyncio.run(_go())
    assert [c.id for c in underlying.executed] == ["x"]
