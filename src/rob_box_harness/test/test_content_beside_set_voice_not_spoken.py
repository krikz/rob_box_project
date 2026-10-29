"""ADR-0150 §4.4, PR-7 ч.7d-3 (#3698): ``content`` рядом с ``set_voice`` — не речь.

Issue #3269 (e2e mv03, MiniMax-M3: ``content='<реплика>' tool_calls=(set_voice(...),)``) закрывался
временной заплаткой в цикле тулов (модуль удалён): цикл сам превращал ``content`` в ``speak_text``. Её сняли:
content вне канала речи не озвучивается никогда, сказку разными голосами в режиме команд несёт
``perform.voice_per_segment``. В v1 (MiniMax-фолбек) такой ход исполняет только ``set_voice``; до слушателя
доходит один финальный текст хода (авто-TTS ноды).
"""

from __future__ import annotations

import asyncio
from typing import Any

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.tools import ToolProvider, ToolSpec
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult


class _ScriptedLLM:
    name = "scripted_llm"

    def __init__(self, responses: list[LLMResponse]) -> None:
        self._responses = list(responses)
        self.requests: list[list[LLMMessage]] = []

    async def complete(self, messages: Any = None, *, tools: Any = (), settings: Any = None, **_: Any) -> Any:
        self.requests.append(list(messages or []))
        if self._responses:
            return self._responses.pop(0)
        return LLMResponse(content="done", tool_calls=())

    async def aclose(self) -> None:
        return None


class _Tools(ToolProvider):
    name = "recording_tools"

    def __init__(self, names: tuple[str, ...]) -> None:
        self._manifest = tuple(
            ToolSpec(
                name=n,
                description=n,
                parameters={
                    "type": "object",
                    "properties": {"text": {"type": "string"}, "voice": {"type": "string"}},
                },
            )
            for n in names
        )
        self.executed: list[ToolCall] = []

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> Any:
        self.executed.append(call)
        return ToolResult(tool_call_id=call.id, content='{"status": "ok"}', is_error=False)

    async def aclose(self) -> None:
        return None


class _Memory:
    def __init__(self) -> None:
        self.turns: list[Any] = []

    async def append_turn(self, scope: str, turn: Any) -> None:
        self.turns.append(turn)

    async def load_recent(self, scope: str, limit: int = 10) -> list[Any]:
        return list(self.turns[-limit:])

    async def save_fact(self, scope: str, fact: Any) -> None:
        return None

    async def search_facts(self, scope: str, query: str, limit: int = 5) -> list[Any]:
        return []

    async def aclose(self) -> None:
        return None


VOICE_TOOLS = ("set_voice", "speak_text")


def _run(
    responses: list[LLMResponse], names: tuple[str, ...] = VOICE_TOOLS, text: str = ""
) -> tuple[Any, _Tools, _ScriptedLLM]:
    llm = _ScriptedLLM(responses)
    tools = _Tools(names)
    core = AgentCore(llm=llm, tools=tools, memory=_Memory(), dsm=DialogueStateMachine())
    core._dsm.on_event(DialogueEvent.WAKE_WORD)
    result = asyncio.run(
        core.process_input(text or "расскажи сказку про красную шапочку разными голосами", history=[])
    )
    return result, tools, llm


def _set_voice(n: int, voice: str) -> ToolCall:
    return ToolCall(id=f"call_{n}", name="set_voice", arguments={"voice": voice})


def _said(tools: _Tools) -> list[tuple[str, str, Any]]:
    return [(c.name, c.arguments.get("text") or c.arguments.get("voice"), c.arguments.get("voice"))
            for c in tools.executed]


# Дословные реплики шага mv03 из лога робота (30.09, run 36775544782).
LIVE = [
    ("Жила-была девочка, и звали её Красная Шапочка.", "zahar"),
    ("Здравствуй, Красная Шапочка, куда ты идёшь?", "alena"),
    ("Я иду к бабушке, несу ей пирожки и горшочек масла!", "zahar"),
]


def test_content_beside_set_voice_is_not_turned_into_speech() -> None:
    responses = [
        LLMResponse(content=line, tool_calls=(_set_voice(i, voice),))
        for i, (line, voice) in enumerate(LIVE)
    ]
    responses.append(LLMResponse(content="А тут мимо шли охотники и спасли Красную Шапочку."))
    result, tools, _ = _run(responses)

    assert [c.name for c in tools.executed] == ["set_voice"] * len(LIVE)
    assert result.speak_text_real_count == 0 and "speak_text" not in result.tools_called
    assert result.spoken_text == "А тут мимо шли охотники и спасли Красную Шапочку."


def test_history_keeps_the_model_calls_as_they_were() -> None:
    responses = [LLMResponse(content=LIVE[0][0], tool_calls=(_set_voice(0, "zahar"),)),
                 LLMResponse(content="Конец.")]
    _, _, llm = _run(responses)
    assistant = [m for m in llm.requests[-1] if m.role == "assistant" and m.tool_calls]
    assert [c.name for c in assistant[-1].tool_calls] == ["set_voice"]
