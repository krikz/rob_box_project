"""Мост режима команд в v1 (ADR-0150 §4.1, §4.5; #3696 ч.5): форма запроса — по capabilities адаптера,
ответ — одна команда через ``rob_box_dialog.command.parse``, исполнение — прежним v1-циклом ``AgentCore``."""

from __future__ import annotations

import asyncio
import json
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import DialogueEvent, DialogueStateMachine
from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.minimax import MiniMaxProvider
from rob_box_voice.core import command_bridge as cb

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
MINIMAX_CAPS = MiniMaxProvider(api_key="test").capabilities
INVALID_TEXT = phrases.text("understand.invalid")


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


def _command_call(**fields: Any) -> ToolCall:
    return ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=fields)


class _Inner:
    """Провайдер-двойник: отдаёт заготовленные ответы по очереди и запоминает запросы."""

    def __init__(self, caps: ProviderCapabilities, replies: List[LLMResponse], name: str = "deepseek") -> None:
        self.name = name
        self.model = f"{name}-model"
        self._caps = caps
        self._replies = list(replies)
        self.requests: List[dict] = []
        self.closed_streams = 0

    @property
    def capabilities(self) -> ProviderCapabilities:
        return self._caps

    def capabilities_for(self, model: Optional[str]) -> ProviderCapabilities:
        return self._caps

    def _take(self, messages: Iterable[LLMMessage], tools: Iterable[Mapping[str, Any]],
              settings: Optional[LLMSettings]) -> LLMResponse:
        self.requests.append({"messages": list(messages), "tools": list(tools), "settings": settings})
        return self._replies.pop(0)

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        return self._take(messages, tools, settings)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: Optional[LLMSettings] = None) -> AsyncIterator[LLMChunk]:
        reply = self._take(messages, tools, settings)
        try:
            text = reply.content
            for i in range(0, len(text), 7):  # JSON приходит кусками — мост разбирает только целиком
                yield LLMChunk(content_delta=text[i:i + 7])
            for call in reply.tool_calls:
                yield LLMChunk(tool_call_delta=call)
            yield LLMChunk(finish_reason=reply.finish_reason or "stop", usage=reply.usage or None,
                           truncated_tool_args=reply.truncated_tool_args)
        finally:
            self.closed_streams += 1

    async def aclose(self) -> None:
        return None


def _run(coro: Any) -> Any:
    return asyncio.run(coro)


def _messages() -> List[LLMMessage]:
    return [LLMMessage(role="system", content="ты РОББОКС"), LLMMessage(role="system", content="факты"),
            LLMMessage(role="user", content="привет")]


# ── режим по capabilities ──────────────────────────────────────────────────────────────────────────────────


def test_mode_follows_adapter_capabilities_not_prompt() -> None:
    assert cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, [])).mode == "tool_choice"
    assert cb.CommandModeLLM(_Inner(MINIMAX_CAPS, [], name="minimax")).mode == "json_in_text"
    caps = ProviderCapabilities(tools=True, json_mode=True)
    assert cb.CommandModeLLM(_Inner(caps, [])).mode == "json_mode"


def test_tool_choice_request_forces_command_tool_and_hides_speak_text() -> None:
    bridge = cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, []))
    original = _messages()
    msgs, tools, settings = bridge.shape(original, [_tool("speak_text"), _tool("memory_save")],
                                         LLMSettings(temperature=0.3, extra={"thinking": {"type": "disabled"}}))
    assert [t["function"]["name"] for t in tools] == [kn.COMMAND_TOOL, "memory_save"]
    assert tools[0]["function"]["parameters"] == kn.command_schema("json")
    assert settings.tool_choice == "required" and settings.temperature == 0.3
    assert settings.extra == {"thinking": {"type": "disabled"}}
    assert msgs[0].content.startswith("ты РОББОКС") and cb.V1_TRANSPORT_NOTE in msgs[0].content
    assert kn.AFTER_ACTIONS["reply"] in msgs[0].content and "JSON-схема" not in msgs[0].content
    assert msgs[1:] == original[1:] and original[0].content == "ты РОББОКС"  # исходный список не тронут


def test_json_in_text_request_keeps_native_tools_and_puts_schema_in_prompt() -> None:
    bridge = cb.CommandModeLLM(_Inner(MINIMAX_CAPS, [], name="minimax"))
    msgs, tools, settings = bridge.shape(_messages(), [_tool("speak_text"), _tool("get_current_time")], None)
    assert [t["function"]["name"] for t in tools] == ["get_current_time"]
    assert settings.tool_choice is None and "response_format" not in settings.extra
    note = msgs[0].content
    assert "JSON-схема команды: " + json.dumps(kn.command_schema("json"), ensure_ascii=False) in note
    assert "одним JSON-объектом" in note


def test_json_mode_sets_response_format() -> None:
    bridge = cb.CommandModeLLM(_Inner(ProviderCapabilities(tools=True, json_mode=True), []))
    _, _, settings = bridge.shape([LLMMessage(role="user", content="привет")], [], None)
    assert settings.extra["response_format"] == {"type": "json_object"}


def test_note_becomes_first_system_message_when_there_is_none() -> None:
    msgs, _, _ = cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, [])).shape([LLMMessage(role="user", content="x")], [], None)
    assert msgs[0].role == "system" and cb.V1_TRANSPORT_NOTE in msgs[0].content and msgs[1].content == "x"


# ── разбор ответа и перевод в форму v1 ─────────────────────────────────────────────────────────────────────


def _convert(caps: ProviderCapabilities, reply: LLMResponse, log: Optional[list] = None) -> LLMResponse:
    lines: list = [] if log is None else log
    bridge = cb.CommandModeLLM(_Inner(caps, [reply]), lines.append)
    return _run(bridge.complete(_messages(), tools=[_tool("memory_save")]))


def test_speech_only_command_becomes_spoken_text_and_ends_turn() -> None:
    log: list = []
    out = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(speech="Привет!"),),
                                              usage={"prompt_tokens": 900}), log)
    assert (out.content, out.tool_calls, out.finish_reason) == ("Привет!", (), "stop")
    assert out.usage == {"prompt_tokens": 900}
    assert log == [f"{cb.COMMAND_LOG_MARK} provider=deepseek mode=tool_choice verdict=speech actions=[] "
                   "speech_dropped=0"]


def test_actions_become_tool_calls_and_speech_waits_for_results() -> None:
    log: list = []
    reply = LLMResponse(tool_calls=(
        _command_call(speech="Запомнил!", actions=[{"tool": "memory_save", "args": {"content": "чай"}}]),
        ToolCall(id="n1", name="get_current_time", arguments={}),
    ))
    out = _convert(DEEPSEEK_CAPS, reply, log)
    assert out.content == "" and out.finish_reason == "tool_calls"
    expected = [("memory_save", {"content": "чай"}), ("get_current_time", {})]
    assert [(c.name, dict(c.arguments)) for c in out.tool_calls] == expected
    assert all(c.id.startswith(cb.CALL_ID_PREFIX) for c in out.tool_calls)
    assert "verdict=actions" in log[0] and "speech_dropped=1" in log[0]


def test_json_in_text_after_reasoning_is_parsed() -> None:
    reply = LLMResponse(content='<think>{"черновик": 1}</think>\n{"speech": "Сейчас полдень."}')
    out = _convert(MINIMAX_CAPS, reply)
    assert out.content == "Сейчас полдень." and out.tool_calls == ()


def test_json_in_text_with_native_tool_call_merges_actions() -> None:
    search = ToolCall(id="n", name="search_web", arguments={"query": "погода"})
    reply = LLMResponse(content='{"speech": "Смотрю"}', tool_calls=(search,))
    out = _convert(MINIMAX_CAPS, reply)
    assert [c.name for c in out.tool_calls] == ["search_web"] and out.content == ""


def test_native_tool_calls_without_command_are_actions() -> None:
    out = _convert(MINIMAX_CAPS, LLMResponse(tool_calls=(ToolCall(id="n", name="get_battery_level", arguments={}),)))
    assert [c.name for c in out.tool_calls] == ["get_battery_level"]


def test_minimax_false_passed_and_empty_args_are_not_invalid() -> None:
    """Бейк-офф 21.10: MiniMax json_in_text — ``"passed": false`` (49 из 52 invalid), tool_choice — ``args: ""``."""
    log: list = []
    said = _convert(MINIMAX_CAPS, LLMResponse(content='{"speech": "Привет!", "actions": [], "passed": false}'), log)
    assert (said.content, said.tool_calls) == ("Привет!", ())
    assert "verdict=speech" in log[0]
    empty_args = [{"tool": "get_current_time", "args": ""}]
    acted = _convert(MINIMAX_CAPS, LLMResponse(tool_calls=(_command_call(actions=empty_args),)))
    assert [(c.name, dict(c.arguments)) for c in acted.tool_calls] == [("get_current_time", {})]


@pytest.mark.parametrize("reply, reason", [
    (LLMResponse(content="Просто текст без команды."), "no_command"),
    (LLMResponse(content='{"speech": "", "passed": true}'), "bad_type"),  # true — не исход из перечисления
    (LLMResponse(content='{"speech": "Привет", "mood": "весело"}'), "unknown_field"),
    (LLMResponse(tool_calls=(_command_call(speech="обре"),), truncated_tool_args=True), "truncated"),
])
def test_invalid_command_asks_to_repeat_without_second_call(reply: LLMResponse, reason: str) -> None:
    log: list = []
    out = _convert(MINIMAX_CAPS, reply, log)
    assert (out.content, out.tool_calls) == (INVALID_TEXT, ())
    assert f"verdict=invalid:{reason}" in log[0]


def test_passed_is_honest_silence() -> None:
    out = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(passed="nothing_to_say"),)))
    assert (out.content, out.tool_calls) == ("", ())


def test_perform_and_ask_are_spoken_when_there_is_no_speech() -> None:
    perform = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(
        perform={"kind": "tale", "segments": ["Жил-был ёж.", "И жил он долго."]}),)))
    assert perform.content == "Жил-был ёж.\nИ жил он долго."
    ask = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(ask={"expects": "name", "text": "Как тебя зовут?"}),)))
    assert ask.content == "Как тебя зовут?"


def test_speak_text_in_actions_is_speech_not_action() -> None:
    out = _convert(MINIMAX_CAPS, LLMResponse(content='{"actions": [{"tool": "speak_text", "args": {"text": "Привет"}}]}'))
    assert (out.content, out.tool_calls) == ("Привет", ())


@pytest.mark.parametrize("reply, content, calls", [
    (LLMResponse(tool_calls=(_command_call(speech="Привет!"),), usage={"prompt_tokens": 5}), "Привет!", []),
    (LLMResponse(content='{"actions": [{"tool": "memory_save", "args": {}}]}'), "", ["memory_save"]),
])
def test_stream_is_collected_then_replayed_in_v1_shape(reply: LLMResponse, content: str, calls: list) -> None:
    inner = _Inner(DEEPSEEK_CAPS, [reply])
    bridge = cb.CommandModeLLM(inner)

    async def _drain() -> list:
        return [chunk async for chunk in bridge.stream(_messages(), tools=[])]

    chunks = _run(_drain())
    assert "".join(c.content_delta for c in chunks) == content
    assert [c.tool_call_delta.name for c in chunks if c.tool_call_delta] == calls
    assert chunks[-1].finish_reason == ("tool_calls" if calls else "stop")
    assert inner.closed_streams == 1


# ── подключение: какие провайдеры, имя и атрибуты обёрнутого ────────────────────────────────────────────────


def test_wrapper_for_wraps_only_listed_providers_and_logs_the_mode() -> None:
    log: list = []
    wrap = cb.wrapper_for(" DeepSeek , ", log.append)
    deepseek, minimax = _Inner(DEEPSEEK_CAPS, []), _Inner(MINIMAX_CAPS, [], name="minimax")
    wrapped = wrap(deepseek)
    assert isinstance(wrapped, cb.CommandModeLLM) and wrap(minimax) is minimax
    assert wrapped.name == "deepseek" and wrapped.model == "deepseek-model"
    assert wrapped.capabilities is DEEPSEEK_CAPS
    assert log == [f"{cb.COMMAND_LOG_MARK} provider=deepseek mode=tool_choice (по capabilities адаптера)"]
    assert cb.wrapper_for("")(deepseek) is deepseek
    assert cb.command_providers(["minimax", " "]) == frozenset({"minimax"})


# ── сквозной ход через v1-цикл AgentCore и цепочку здоровья ────────────────────────────────────────────────


def _dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_turn_runs_action_then_speaks_result(streaming: bool) -> None:
    """Команда с действием → v1 исполняет тул → вторая команда говорит итог по результату; ``speak_text`` не звался."""
    inner = _Inner(DEEPSEEK_CAPS, [
        LLMResponse(tool_calls=(_command_call(speech="Сейчас", actions=[{"tool": "echo", "args": {"text": "x"}}]),)),
        LLMResponse(tool_calls=(_command_call(speech="Эхо ответило: x."),)),
    ])
    chain = HealthAwareFallbackLLM([cb.wrapper_for("deepseek")(inner), _Inner(MINIMAX_CAPS, [], name="minimax")],
                                   cache=HealthCache())
    core = AgentCore(llm=chain, tools=FakeToolProvider(), memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты РОББОКС", use_streaming=streaming)

    result = asyncio.run(core.process_input("повтори x", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None
    assert result.spoken_text == "Эхо ответило: x."
    assert list(result.tools_called) == ["echo"]
    assert len(inner.requests) == 2 and chain.answered_by == "deepseek"
    second = inner.requests[1]
    assert [t["function"]["name"] for t in second["tools"]][0] == kn.COMMAND_TOOL
    assert "speak_text" not in [t["function"]["name"] for t in second["tools"]]
    assert second["settings"].tool_choice == "required"
    assert any(m.role == "tool" for m in second["messages"])  # результат тула дошёл до модели
