"""Smoke test: agent cycle — 1 запрос → 3 tool_calls → финальный ответ.

TASK-041 acceptance: «Smoke test agent cycle (1 запрос с 3 tool_calls)
проходит в headless режиме». Полный цикл:

  1. ``process_input`` собирает сообщения и вызывает LLM (с тулами).
  2. LLM возвращает **3 tool_calls** на первом ходу.
  3. Каждый tool выполняется через ToolProvider; результаты подмешиваются
     в историю как ``role="tool"``.
  4. LLM вызывается повторно и возвращает plain-text финальный ответ.
  5. ``DialogResult`` содержит озвучиваемый текст и список вызванных тулов.

Тест не требует ROS2/Ollama/железа: все четыре порта DialogCore
(LLM, tools, memory, DSM) — фейки. Запускается в CI обычным pytest.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any

from rob_box_harness.core.dialog_core import DialogCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueStateMachine,
)
from rob_box_llm.provider import LLMResponse, ToolCall


# ---------------------------------------------------------------------------
# Минимальные фейки (по образцу test_dialog_core / test_dialog_core_with_acceptance)
# ---------------------------------------------------------------------------


@dataclass
class _FakeMemory:
    """In-memory store: append-only, возвращает последние N ходов."""

    turns: list[dict[str, Any]] | None = None

    def __post_init__(self) -> None:
        if self.turns is None:
            self.turns = []

    async def append_turn(self, role: str, content: str, **meta: Any) -> None:
        self.turns.append({"role": role, "content": content, **meta})

    async def recent(self, limit: int = 10, **kwargs: Any) -> list[dict[str, Any]]:
        return self.turns[-limit:]


class _ScriptedLLM:
    """Возвращает скриптованные ответы по очереди."""

    name = "scripted_llm"

    def __init__(self, responses: list[LLMResponse]) -> None:
        self._responses = list(responses)
        self.calls: list[Any] = []

    async def complete(
        self,
        messages: Any = None,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> Any:
        self.calls.append(messages)
        if self._responses:
            return self._responses.pop(0)
        return LLMResponse(content="ok", tool_calls=())

    async def aclose(self) -> None:
        return None


class _FakeTools:
    """ToolProvider: реестр обработчиков + журнал выполненных вызовов."""

    name = "fake_tools"

    def __init__(self, handlers: dict[str, Any]) -> None:
        self._handlers = handlers
        self.executed: list[ToolCall] = []

    async def discover(self) -> tuple[Any, ...]:
        from rob_box_harness.tools import ToolSpec

        return tuple(
            ToolSpec(name=name, description=f"tool {name}", parameters={"type": "object"})
            for name in self._handlers
        )

    async def execute(self, call: ToolCall) -> Any:
        from rob_box_llm.provider import ToolResult

        self.executed.append(call)
        handler = self._handlers.get(call.name)
        if handler is None:
            return ToolResult(
                tool_call_id=call.id,
                content=f"unknown tool: {call.name}",
                is_error=True,
            )
        result = handler(dict(call.arguments or {}))
        if hasattr(result, "__await__"):
            result = await result
        return ToolResult(tool_call_id=call.id, content=str(result), is_error=False)

    async def aclose(self) -> None:
        return None


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------


def test_agent_cycle_one_request_three_tool_calls() -> None:
    """1 запрос → LLM делает 3 tool_calls → тулы выполняются → финальный ответ."""
    executed: list[str] = []

    def make_handler(tool_name: str):
        async def handler(args: dict[str, Any]) -> str:
            executed.append(tool_name)
            return f"{tool_name}:ok"

        return handler

    tools = _FakeTools(
        {
            "get_current_time": make_handler("get_current_time"),
            "play_animation": make_handler("play_animation"),
            "memory_search": make_handler("memory_search"),
        }
    )

    llm = _ScriptedLLM(
        [
            # Ход 1 — LLM решает вызвать 3 тула.
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(id="c1", name="get_current_time", arguments={}),
                    ToolCall(id="c2", name="play_animation", arguments={"name": "happy"}),
                    ToolCall(id="c3", name="memory_search", arguments={"query": "кухня"}),
                ),
            ),
            # Ход 2 — после результатов тулов LLM отвечает обычным текстом.
            LLMResponse(content="Сейчас 12:00, анимация запущена, я вспомнил.", tool_calls=()),
        ]
    )

    core = DialogCore(
        llm=llm,
        tools=tools,
        memory=_FakeMemory(),
        dsm=DialogueStateMachine(),
    )

    # Wake-word → LISTENING, чтобы process_input перешёл в DIALOGUE.
    asyncio.run(core.handle_wake_word(""))

    result = asyncio.run(core.process_input("сколько времени?", history=[]))

    # Все три тула реально выполнились, каждый ровно один раз.
    assert executed == ["get_current_time", "play_animation", "memory_search"]
    assert len(tools.executed) == 3

    # LLM вызывался дважды: с тулами и после результатов.
    assert len(llm.calls) == 2

    # Финальный ответ озвучен, список вызванных тулов в результате.
    assert result.spoken_text == "Сейчас 12:00, анимация запущена, я вспомнил."
    assert result.tools_called == ["get_current_time", "play_animation", "memory_search"]
    assert result.error is None
