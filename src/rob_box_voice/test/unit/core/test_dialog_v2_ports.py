"""PR-8 ч.8e (#3702): транспорт хода v2 поверх дороги v1 — LLM через мост команд, действия через исполнитель тулов."""

from __future__ import annotations

import asyncio
import json
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Command, Invalid, ToolCall as CommandCall  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult  # noqa: E402
from rob_box_voice.core import command_bridge  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import (  # noqa: E402
    EMPTY, CallCounter, Counted, V2Ports, command_providers, counted_wrapper, follow_for, to_result, tool_schema,
)

TIME = type(M.TurnEvent().stamp)


class _Pub:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, msg_type, topic, depth):
        return self.pubs.setdefault(topic, _Pub(topic))

    def get_clock(self):
        stamp = TIME(sec=1, nanosec=0)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


class _LLM:
    """Цепочка с одним ответом на вызов; ``answered_by`` — как у HealthAwareFallbackLLM."""

    def __init__(self, responses, counter):
        self.responses, self.counter, self.calls = list(responses), counter, []
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.counter.calls += 1
        self.calls.append((list(messages), list(tools)))
        return self.responses.pop(0)


class _Tools:
    def __init__(self, data=None):
        self.data, self.calls, self.turns = data or {}, [], 0

    def begin_turn(self):
        self.turns += 1

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        status, data = self.data.get(call.name, ("done", {}))
        return ToolResult(tool_call_id=call.id, content="ok", is_error=status != "done",
                          data={"status": status, **data})


def _command_call(args):
    return ToolCall(id="cmd-1", name=kn.COMMAND_TOOL, arguments=args)


def test_to_result_speech_text_command_and_invalid():
    assert to_result(LLMResponse(content="Привет!")) == Command(speech="Привет!")
    assert to_result(LLMResponse(content="")) == Invalid(EMPTY, "ни речи, ни команды")
    marked = LLMResponse(tool_calls=(_command_call({kn.COMMAND_INVALID_FIELD: "bad_json"}),))
    assert to_result(marked) == Invalid("bad_json")
    cmd = to_result(LLMResponse(tool_calls=(_command_call({"actions": [{"tool": "stop_music", "args": {}}]}),)))
    assert isinstance(cmd, Command) and [a.tool for a in cmd.actions] == ["stop_music"]


def test_bridge_legacy_form_round_trips_to_the_same_command():
    """Мост отдаёт команду аргументами ``to_args`` — ``to_result`` разбирает её тем же парсером без потерь."""
    original = Command(actions=(CommandCall("play_sound", {"sound": "boop"}),))
    text, calls, verdict = command_bridge.to_legacy(original)
    assert verdict == "command" and text == ""
    back = to_result(LLMResponse(tool_calls=calls))
    assert back.actions == original.actions


def test_follow_answers_the_command_call_with_tool_result():
    response = LLMResponse(tool_calls=(_command_call({"actions": []}),))
    assistant, tool = follow_for(response)('{"query_results": []}')
    assert assistant.role == "assistant" and assistant.tool_calls[0].id == "cmd-1"
    assert tool.role == "tool" and tool.tool_call_id == "cmd-1" and tool.content == '{"query_results": []}'
    assert follow_for(LLMResponse(content="x"))("d") == ({"role": "user", "content": "d"},)


def test_counted_counts_every_provider_call_and_proxies_the_rest():
    counter = CallCounter()

    class Inner:
        name, capabilities = "minimax", "caps"

        async def complete(self, messages, **kw):
            return LLMResponse(content="ok")

    wrapped = counted_wrapper(lambda p: p, counter)(Inner())
    assert isinstance(wrapped, Counted) and wrapped.name == "minimax" and wrapped.capabilities == "caps"
    asyncio.run(wrapped.complete([]))
    asyncio.run(wrapped.complete([]))
    assert counter.calls == 2


def test_command_providers_takes_whole_chain():
    assert command_providers(["deepseek", "", "minimax"]) == "deepseek,minimax"


def test_tool_schema_is_openai_function():
    entry = types.SimpleNamespace(name="stop_music", description="стоп", parameters={"type": "object"})
    assert tool_schema(entry) == {"type": "function", "function": {"name": "stop_music", "description": "стоп",
                                                                   "parameters": {"type": "object"}}}


def _ports(responses, tools=None):
    counter = CallCounter()
    node = _Node()
    intent_pub = _Pub("/dialog/intent")
    ports = V2Ports(node, _LLM(responses, counter), counter, tools or _Tools(), intent_pub=intent_pub)
    return ports, node, intent_pub


def test_full_turn_through_ports_speaks_outcome_and_publishes_turn_events():
    cmd = {"actions": [{"tool": "stop_music", "args": {}}]}
    tools = _Tools()
    ports, node, _ = _ports([LLMResponse(tool_calls=(_command_call(cmd),), usage={"prompt_tokens": 1234})], tools)
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed("выключи музыку пожалуйста сейчас же", VOICE)))
    assert tools.calls == [("stop_music", {})] and tools.turns == 1
    speech = node.pubs["/voice/speech/request"].sent
    assert [s.text for s in speech] == ["<speak>Выключил музыку.</speak>"]
    assert speech[0].agent == "personality" and speech[0].turn_id == agg.turn_id
    events = [(e.stage, e.status) for e in node.pubs["/dialog/turn_event"].sent]
    assert events == [("addressed", "started"), ("thinking", "started"), ("executing", "started"),
                      ("done", "finished")]
    assert node.pubs["/voice/expression/request"].sent, "рефлексы стадий без LLM"
    assert (agg.llm_calls, agg.llm_provider, agg.tokens) == (1, "deepseek", 1234)


def test_owner_rejection_reaches_phrase():
    cmd = {"actions": [{"tool": "stop_music", "args": {}}]}
    ports, node, _ = _ports([LLMResponse(tool_calls=(_command_call(cmd),))],
                            _Tools({"stop_music": ("rejected", {})}))
    engine = TurnEngine(Session("personality"), ports)
    asyncio.run(engine.run(Addressed("выключи музыку пожалуйста сейчас же", VOICE)))
    said = [s.text for s in node.pubs["/voice/speech/request"].sent]
    assert said == ["<speak>Не получилось выключить музыку.</speak>"]


def test_tier1_stop_publishes_stop_motion_intent_and_cancel():
    ports, node, intent_pub = _ports([])
    asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("стоп", VOICE)))
    (intent,) = intent_pub.sent
    assert intent.kind == "stop_motion" and json.loads(intent.args_json)["entities"] == {}
    assert node.pubs["/voice/speech/cancel"].sent
