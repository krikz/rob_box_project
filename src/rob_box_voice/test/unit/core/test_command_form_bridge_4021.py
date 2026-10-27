"""#4021: форма команды хода на мосту режима команд — схема без ``passed``/``voice``, нарушение и повтор моста, голос
без просьбы отброшен (``fields_dropped``); и весь ход v2 (``TurnEngine`` → ``V2Ports`` → мост) на n813 «Робот, тишина?».
"""

from __future__ import annotations

import asyncio
import types
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import command as cm  # noqa: E402
from rob_box_dialog import knowledge as kn, phrases  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Form  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import (LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities,  # noqa: E402
                                  ToolCall, ToolResult)
from rob_box_llm.providers.deepseek import DeepSeekProvider  # noqa: E402
from rob_box_voice.core import command_bridge as cb  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports, counted_wrapper  # noqa: E402

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
ADDRESSED = Form(passed=False, voice=False)
TIME = type(M.TurnEvent().stamp)


def _call(**fields: Any) -> ToolCall:
    return ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=fields)


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди, запросы — в ``requests``."""

    def __init__(self, replies: List[LLMResponse], name: str = "deepseek") -> None:
        self.name, self.model, self._replies, self.requests = name, f"{name}-model", list(replies), []

    @property
    def capabilities(self) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    def capabilities_for(self, model: Optional[str]) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        self.requests.append({"messages": list(messages), "tools": list(tools)})
        return self._replies.pop(0)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: Optional[LLMSettings] = None) -> AsyncIterator[LLMChunk]:  # pragma: no cover
        raise NotImplementedError

    async def aclose(self) -> None:
        return None


def _command_schema(request: Mapping[str, Any]) -> Mapping[str, Any]:
    return next(t["function"]["parameters"] for t in request["tools"] if t["function"]["name"] == kn.COMMAND_TOOL)


# -- мост ---------------------------------------------------------------------------------------------------


def test_addressed_turn_request_has_no_passed_and_no_voice_in_schema_or_prompt():
    bridge = cb.CommandModeLLM(_Inner([]))
    msgs = [LLMMessage(role="system", content="ты робот"), LLMMessage(role="user", content="тишина")]
    with cm.turn_form(ADDRESSED):
        shaped, tools, _ = bridge.shape(msgs, [_tool("get_current_time")], None)
    schema = tools[0]["function"]["parameters"]
    assert "passed" not in schema["properties"] and "voice" not in schema["properties"]
    assert "- passed ∈" not in shaped[0].content and "- voice:" not in shaped[0].content
    _, open_tools, _ = bridge.shape(msgs, [_tool("get_current_time")], None)  # вне хода — как было
    assert "passed" in open_tools[0]["function"]["parameters"]["properties"]


def test_n813_passed_on_addressed_turn_is_retried_with_the_violation_and_the_answer_is_spoken():
    inner = _Inner([LLMResponse(tool_calls=(_call(passed="not_addressed"),)),
                    LLMResponse(tool_calls=(_call(speech="Тишина так тишина — молчу."),))])
    lines: List[str] = []
    bridge = cb.CommandModeLLM(inner, log=lines.append)

    async def run() -> LLMResponse:
        with cm.turn_form(ADDRESSED):
            return await bridge.complete([LLMMessage(role="user", content="тишина")], tools=[])

    out = asyncio.run(run())
    assert out.content == "Тишина так тишина — молчу." and not out.tool_calls
    retry = inner.requests[1]["messages"]
    assert any(kn.VIOLATION_PASSED_ADDRESSED in (m.content or "") for m in retry if m.role == "tool")
    assert any(f"violations=['{kn.VIOLATION_PASSED_ADDRESSED}']" in line for line in lines)


def test_backlog_pickup_passed_goes_through_without_retry():
    inner = _Inner([LLMResponse(tool_calls=(_call(passed="not_addressed"),))])
    bridge = cb.CommandModeLLM(inner)

    async def run() -> LLMResponse:
        with cm.turn_form(Form(passed=True, voice=False)):
            return await bridge.complete([LLMMessage(role="user", content="фон")], tools=[])

    out = asyncio.run(run())
    assert len(inner.requests) == 1 and out.tool_calls[0].arguments == {"passed": "not_addressed"}
    assert "passed" in _command_schema(inner.requests[0])["properties"]


def test_n109_unasked_voice_is_dropped_and_logged_the_line_is_plain_speech():
    inner = _Inner([LLMResponse(tool_calls=(_call(speech="Умею сирену и звонок.", voice="female-shaonv"),))])
    lines: List[str] = []
    bridge = cb.CommandModeLLM(inner, log=lines.append)

    async def run() -> LLMResponse:
        with cm.turn_form(ADDRESSED):
            return await bridge.complete([LLMMessage(role="user", content="какие звуки умеешь")], tools=[])

    out = asyncio.run(run())
    assert out.content == "Умею сирену и звонок." and not out.tool_calls, "реплика без голоса — текстом, без command"
    assert any("fields_dropped=['voice']" in line for line in lines)
    assert len(inner.requests) == 1, "лишнее поле — не нарушение, повтора нет"


# -- весь ход v2: TurnEngine → V2Ports → мост ---------------------------------------------------------------------


class _Pub:
    def __init__(self) -> None:
        self.sent: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg)


class _Node:
    def __init__(self) -> None:
        self.pubs: dict = {}

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> _Pub:
        return self.pubs.setdefault(topic, _Pub())

    def get_clock(self) -> Any:
        stamp = TIME(sec=1, nanosec=0)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


class _Tools:
    async def execute(self, call: Any) -> ToolResult:
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


def _v2(replies: List[LLMResponse]) -> tuple:
    counter, node = CallCounter(), _Node()
    inner = _Inner(replies)
    llm = counted_wrapper(lambda p: cb.CommandModeLLM(p), counter)(inner)
    return V2Ports(node, llm, counter, _Tools(), intent_pub=_Pub()), node, inner


def test_n813_whole_v2_turn_answers_in_words_not_with_the_mumble():
    """«Робот, тишина?»: модель первым ответом «промолчать» — мост вернул ей нарушение, второй ответ — слова; робот
    говорит их, а не «Хм, я задумался…»."""
    ports, node, inner = _v2([LLMResponse(tool_calls=(_call(passed="not_addressed"),)),
                              LLMResponse(tool_calls=(_call(speech="Тихо так тихо."),))])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    asyncio.run(engine.run(Addressed("тишина", VOICE)))
    said = [s.text for s in node.pubs["/voice/speech/request"].sent]
    assert said == ["<speak>Тихо так тихо.</speak>"]
    assert not any(v in said[0] for v in phrases.catalog()[kn.PASSED_ADDRESSED_PHRASE])
    assert "passed" not in _command_schema(inner.requests[0])["properties"]
    assert cm.current_form() == Form()
