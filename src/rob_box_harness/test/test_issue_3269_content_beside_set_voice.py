"""Issue #3269 — реплики сказки в ``content`` рядом с ``set_voice`` не звучали.

e2e mv03, прогоны 36775544782 / 36777967495 (``docker logs voice-assistant``
Vision Pi, 30.09): MiniMax-M3 восемь итераций отвечала
``content='<реплика>' tool_calls=(set_voice(...),)``, ``speak_text`` не
звал никто, цикл упёрся в ``_MAX_TOOL_ITERATIONS=8`` — прозвучала одна
последняя фраза. Фикс — :mod:`rob_box_harness.core.tool_loop.content_speech`.
"""

from __future__ import annotations

import asyncio
from typing import Any

from rob_box_harness.core import agent_core
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_loop.content_speech import (
    ContentSpeech,
    speak_content_beside_set_voice,
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


class TestLiveMv03Shape:
    def test_every_line_is_spoken_in_order_with_voice_of_previous_set_voice(self) -> None:
        responses = [
            LLMResponse(content=line, tool_calls=(_set_voice(i, voice),))
            for i, (line, voice) in enumerate(LIVE)
        ]
        responses.append(LLMResponse(content="А тут мимо шли охотники и спасли Красную Шапочку.\n\ndone"))
        result, tools, _ = _run(responses)

        assert _said(tools) == [
            ("speak_text", LIVE[0][0], None),  # голос до хода харнессу неизвестен
            ("set_voice", "zahar", "zahar"),
            ("speak_text", LIVE[1][0], "zahar"),
            ("set_voice", "alena", "alena"),
            ("speak_text", LIVE[2][0], "alena"),
            ("set_voice", "zahar", "zahar"),
            # финал хода — тем голосом, что стоит сейчас; «done» не звучит
            ("speak_text", "А тут мимо шли охотники и спасли Красную Шапочку.", "zahar"),
        ]
        # dialogue_node пропустит финальный текст как дубль (#988), а в
        # историю ляжет то, что реально прозвучало.
        assert result.speak_text_real_count >= 1
        assert "speak_text" in result.tools_called
        assert "охотники" in result.spoken_text

    def test_history_shows_speak_text_call_not_bare_content(self) -> None:
        responses = [LLMResponse(content=LIVE[0][0], tool_calls=(_set_voice(0, "zahar"),)),
                     LLMResponse(content="done")]
        _, _, llm = _run(responses)
        assistant = [m for m in llm.requests[-1] if m.role == "assistant" and m.tool_calls]
        assert [c.name for c in assistant[-1].tool_calls] == ["speak_text", "set_voice"]
        assert assistant[-1].content == ""

    def test_iteration_cap_still_voices_every_line(self) -> None:
        cap = agent_core._MAX_TOOL_ITERATIONS
        responses = [
            LLMResponse(content=f"Реплика {i}.", tool_calls=(_set_voice(i, "zahar" if i % 2 else "alena"),))
            for i in range(cap + 1)
        ]
        _, tools, _ = _run(responses)
        spoken = [c.arguments["text"] for c in tools.executed if c.name == "speak_text"]
        # cap итераций исполнены + финальный ответ, упёршийся в лимит.
        assert spoken == [f"Реплика {i}." for i in range(cap + 1)]


class TestNarrowness:
    def test_dj_content_beside_music_tool_is_not_spoken(self) -> None:
        responses = [
            LLMResponse(
                content="Переход номер два отыгран — нарастание с дропом!",
                tool_calls=(ToolCall(id="c1", name="compose_music", arguments={"text": "x"}),),
            ),
            LLMResponse(content="done"),
        ]
        _, tools, _ = _run(responses, names=("compose_music", "set_voice", "speak_text"), text="диджей, дальше")
        assert [c.name for c in tools.executed] == ["compose_music"]

    def test_set_voice_with_own_speak_text_is_not_duplicated(self) -> None:
        responses = [
            LLMResponse(
                content="Говорю голосом Алёны.",
                tool_calls=(_set_voice(1, "alena"),
                            ToolCall(id="c2", name="speak_text", arguments={"text": "Говорю голосом Алёны."})),
            ),
            LLMResponse(content="done"),
        ]
        _, tools, _ = _run(responses, text="говори голосом алёны")
        assert [c.name for c in tools.executed] == ["set_voice", "speak_text"]

    def test_plain_reply_without_set_voice_is_left_to_dialogue_node(self) -> None:
        result, tools, _ = _run([LLMResponse(content="Привет! Всё хорошо.")], text="как дела")
        assert tools.executed == []
        assert result.spoken_text == "Привет! Всё хорошо."
        assert result.speak_text_real_count == 0

    def test_service_content_beside_set_voice_is_not_spoken(self) -> None:
        for service in ("done", "<set_voice>", '<invoke name="set_voice"></invoke>', "  "):
            state = ContentSpeech()
            resp = LLMResponse(content=service, tool_calls=(_set_voice(1, "zahar"),))
            out = speak_content_beside_set_voice(resp, state, [{"function": {"name": "speak_text"}}])
            assert out is resp, service
            assert state.spoke is False
            assert state.voice == "zahar"

    def test_nothing_spoken_when_speak_text_not_offered(self) -> None:
        state = ContentSpeech()
        resp = LLMResponse(content=LIVE[0][0], tool_calls=(_set_voice(1, "zahar"),))
        out = speak_content_beside_set_voice(resp, state, [{"function": {"name": "set_voice"}}])
        assert out is resp
        assert state.spoke is False

    def test_story_ending_word_vsyo_is_speech(self) -> None:
        state = ContentSpeech()
        resp = LLMResponse(content="Вот и сказке конец, вот и всё.", tool_calls=(_set_voice(1, "zahar"),))
        out = speak_content_beside_set_voice(resp, state, [{"function": {"name": "speak_text"}}])
        assert out.tool_calls[0].arguments["text"] == "Вот и сказке конец, вот и всё."
