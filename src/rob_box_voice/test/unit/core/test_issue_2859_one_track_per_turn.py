"""Issue #2859 — не больше одного успешного запуска трека за ход.

Живой DJ-переход #23 (23.09.2026): LLM в одном ходе 8 раз вызвала
``execute_music_code`` + ``set_dj_mode`` (старый путь удалён в #3614; стартёр здесь — ``generate_music``); каждый вызов — ``Clock.clear()`` и
новый трек, музыка переключалась каждые ~3 с до ``_MAX_TOOL_ITERATIONS``.

Покрытие:

* чистый :class:`TrackStartGuard` (успех → лимит, провал → повтор разрешён,
  ``reset`` → новый ход);
* :class:`PlanToolExecutor` — отказ не доходит до провайдера;
* каждый стартёр трека ограничен (сквозные сценарии через цикл v1 ``AgentCore`` удалены вместе с ним, #4354 A3b).
"""

from __future__ import annotations

import asyncio
import json

import pytest

from rob_box_harness.tools import ToolSpec
from rob_box_llm.provider import ToolCall, ToolResult
from rob_box_voice.core.track_start_guard import (
    REFUSAL_ERROR_CODE,
    REFUSAL_MESSAGE,
    TRACK_STARTING_TOOLS,
    TrackStartGuard,
)
from rob_box_voice.core.plan_tools import PlanToolExecutor


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
            for n in ("generate_music", "set_dj_mode", "generate_music")
        )

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call)
        if call.name == "generate_music" and self._fail_left > 0:
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


# ---------------------------------------------------------------------------
# Pure guard
# ---------------------------------------------------------------------------


def test_track_starting_tools_cover_issue_list() -> None:
    expected = {
        "generate_music",
    }
    assert expected <= TRACK_STARTING_TOOLS
    assert "set_dj_mode" not in TRACK_STARTING_TOOLS
    assert "lookup_melody" not in TRACK_STARTING_TOOLS
    # ADR-0158 PR-9: music_library — один тул на 7 действий, флаг starts_music на уровне тула; с ним guard отказал бы
    # play после list в одном ходе. Запускает музыку только action=play — guard его намеренно не считает.
    assert "music_library" not in TRACK_STARTING_TOOLS


def test_guard_allows_first_success_then_refuses() -> None:
    guard = TrackStartGuard()
    assert not guard.should_refuse("generate_music")
    guard.record("generate_music", is_error=False)
    assert guard.should_refuse("generate_music")
    assert guard.should_refuse("generate_music")
    assert not guard.should_refuse("set_dj_mode")
    assert guard.started_tool == "generate_music"


def test_guard_failed_start_does_not_consume_limit() -> None:
    guard = TrackStartGuard()
    guard.record("generate_music", is_error=True)
    assert not guard.should_refuse("generate_music")


def test_guard_ignores_non_music_success() -> None:
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=False)
    assert not guard.should_refuse("generate_music")


def test_guard_reset_opens_next_turn() -> None:
    guard = TrackStartGuard()
    guard.record("generate_music", is_error=False)
    guard.reset()
    assert not guard.should_refuse("generate_music")
    assert guard.started_tool is None


# ---------------------------------------------------------------------------
# PlanToolExecutor wiring
# ---------------------------------------------------------------------------


def test_executor_refuses_second_start_without_calling_provider() -> None:
    underlying = _FakeUnderlying()
    executor = PlanToolExecutor(underlying)

    async def _go() -> tuple[ToolResult, ToolResult]:
        executor.begin_turn()
        first = await executor.execute(
            ToolCall(id="a", name="generate_music", arguments={}))
        second = await executor.execute(
            ToolCall(id="b", name="generate_music", arguments={}))
        return first, second

    first, second = asyncio.run(_go())
    assert underlying.names() == ["generate_music"]
    assert json.loads(first.content)["success"] is True
    payload = json.loads(second.content)
    assert second.tool_call_id == "b"
    assert second.is_error is False
    assert payload["success"] is False
    assert payload["error"] == REFUSAL_ERROR_CODE
    assert payload["message"] == REFUSAL_MESSAGE
    assert payload["already_started_by"] == "generate_music"


@pytest.mark.parametrize("tool", sorted(TRACK_STARTING_TOOLS))
def test_every_track_starter_is_limited(tool: str) -> None:
    underlying = _FakeUnderlying()
    executor = PlanToolExecutor(underlying)

    async def _go() -> None:
        executor.begin_turn()
        await executor.execute(ToolCall(id="x", name=tool, arguments={}))
        await executor.execute(ToolCall(id="y", name=tool, arguments={}))

    asyncio.run(_go())
    assert [c.id for c in underlying.executed] == ["x"]
