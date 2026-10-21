"""Мост режима команд в v1 (ADR-0150 §4.1, §4.5; #3696 ч.5, PR-7 ч.7c): форма запроса — по capabilities адаптера,
ответ — одна команда через ``rob_box_dialog.command.parse``; голая реплика — текстом, остальное — вызовом ``command``,
который v1-цикл ``AgentCore`` исполняет кодом (``command_turn``: событие владельца, фраза кода, ход терминален)."""

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
from rob_box_harness.tools import FakeToolProvider, ToolSpec
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
    assert kn.prompt_fragment() in msgs[0].content and "JSON-схема" not in msgs[0].content
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


def _actions(out: LLMResponse) -> list:
    """Действия единственного вызова ``command``, который мост отдал v1-циклу."""
    assert [c.name for c in out.tool_calls] == [kn.COMMAND_TOOL] and out.content == ""
    return [(a["tool"], a["args"]) for a in out.tool_calls[0].arguments.get("actions", [])]


def test_actions_go_as_one_command_call_and_speech_is_only_logged() -> None:
    log: list = []
    reply = LLMResponse(tool_calls=(
        _command_call(speech="Запомнил!", actions=[{"tool": "memory_save", "args": {"content": "чай"}}]),
        ToolCall(id="n1", name="get_current_time", arguments={}),
    ))
    out = _convert(DEEPSEEK_CAPS, reply, log)
    assert out.finish_reason == "tool_calls" and out.tool_calls[0].id.startswith(cb.CALL_ID_PREFIX)
    assert _actions(out) == [("memory_save", {"content": "чай"}), ("get_current_time", {})]
    assert out.tool_calls[0].arguments["speech"] == "Запомнил!"  # не звучит: исполнитель команды его не озвучит
    assert log == [f"{cb.COMMAND_LOG_MARK} provider=deepseek mode=tool_choice verdict=command "
                   "actions=['memory_save', 'get_current_time'] speech_dropped=1 speech='Запомнил!'"]


def test_face_slots_beside_expression_are_moved_and_logged_not_invalid() -> None:
    """Живой прогон 7c (#3698): DeepSeek кладёт слоты лица в ``expression`` — команда исполняется, поправка в логе."""
    log: list = []
    reply = LLMResponse(tool_calls=(_command_call(
        actions=[{"tool": "memory_save", "args": {"content": "чай"}}],
        expression={"emotion": "happy", "eyes": "wide", "rear": "party", "mood": "весело"}),))
    out = _convert(DEEPSEEK_CAPS, reply, log)
    assert out.tool_calls[0].arguments["expression"] == {
        "emotion": "happy", "face": {"eyes": "wide", "rear": "party", "duration_s": 5.0}}
    assert log == [f"{cb.COMMAND_LOG_MARK} provider=deepseek mode=tool_choice verdict=command "
                   "actions=['memory_save'] speech_dropped=0 fields_moved=['expression.eyes', 'expression.rear'] "
                   "fields_dropped=['expression.mood']"]


def test_json_in_text_after_reasoning_is_parsed() -> None:
    reply = LLMResponse(content='<think>{"черновик": 1}</think>\n{"speech": "Сейчас полдень."}')
    out = _convert(MINIMAX_CAPS, reply)
    assert out.content == "Сейчас полдень." and out.tool_calls == ()


def test_json_in_text_with_native_tool_call_merges_actions() -> None:
    search = ToolCall(id="n", name="search_web", arguments={"query": "погода"})
    reply = LLMResponse(content='{"speech": "Смотрю"}', tool_calls=(search,))
    out = _convert(MINIMAX_CAPS, reply)
    assert _actions(out) == [("search_web", {"query": "погода"})]


def test_native_tool_calls_without_command_are_actions() -> None:
    out = _convert(MINIMAX_CAPS, LLMResponse(tool_calls=(ToolCall(id="n", name="get_battery_level", arguments={}),)))
    assert _actions(out) == [("get_battery_level", {})]


def test_minimax_false_passed_and_empty_args_are_not_invalid() -> None:
    """Бейк-офф 21.10: MiniMax json_in_text — ``"passed": false`` (49 из 52 invalid), tool_choice — ``args: ""``."""
    log: list = []
    said = _convert(MINIMAX_CAPS, LLMResponse(content='{"speech": "Привет!", "actions": [], "passed": false}'), log)
    assert (said.content, said.tool_calls) == ("Привет!", ())
    assert "verdict=speech" in log[0]
    empty_args = [{"tool": "get_current_time", "args": ""}]
    acted = _convert(MINIMAX_CAPS, LLMResponse(tool_calls=(_command_call(actions=empty_args),)))
    assert _actions(acted) == [("get_current_time", {})]


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


def test_perform_and_voice_go_to_the_command_executor_ask_is_spoken() -> None:
    tale = {"kind": "tale", "segments": ["Жил-был ёж.", "И жил он долго."], "voice_per_segment": ["", "zahar"]}
    perform = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(perform=tale),)))
    assert _actions(perform) == [] and perform.tool_calls[0].arguments["perform"]["voice_per_segment"] == ["", "zahar"]
    voiced = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(speech="Привет", voice="alena"),)))
    assert voiced.tool_calls[0].arguments["voice"] == "alena"
    ask = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(ask={"expects": "name", "text": "Как тебя зовут?"}),)))
    assert ask.content == "Как тебя зовут?"


def test_speak_text_in_actions_is_speech_not_action() -> None:
    out = _convert(MINIMAX_CAPS, LLMResponse(content='{"actions": [{"tool": "speak_text", "args": {"text": "Привет"}}]}'))
    assert (out.content, out.tool_calls) == ("Привет", ())
    voiced = _convert(MINIMAX_CAPS, LLMResponse(
        content='{"actions": [{"tool": "speak_text", "args": {"text": "Привет", "voice": "zahar"}}]}'))
    args = voiced.tool_calls[0].arguments
    assert (args["speech"], args["voice"], args.get("actions")) == ("Привет", "zahar", None)


@pytest.mark.parametrize("reply, content, calls", [
    (LLMResponse(tool_calls=(_command_call(speech="Привет!"),), usage={"prompt_tokens": 5}), "Привет!", []),
    (LLMResponse(content='{"actions": [{"tool": "memory_save", "args": {}}]}'), "", [kn.COMMAND_TOOL]),
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


class _Spoken:
    """Тулы хода v1: ``memory_save``, ``get_current_time`` и ``speak_text`` с журналом реплик."""

    def __init__(self) -> None:
        self.said: List[str] = []
        self.tools = FakeToolProvider()
        for name, handler in (("memory_save", lambda a: {"fact_id": 1, "fact": a["fact"]}),
                              ("get_current_time", lambda a: "12:00"), ("speak_text", self._say)):
            self.tools.register(ToolSpec(name=name, description=name, parameters={"type": "object"}), handler)

    def _say(self, args: Mapping[str, Any]) -> str:
        self.said.append(args["text"])
        return "ok"


def _core(inner: _Inner, streaming: bool, spoken: _Spoken) -> AgentCore:
    chain = HealthAwareFallbackLLM([cb.wrapper_for("deepseek")(inner), _Inner(MINIMAX_CAPS, [], name="minimax")],
                                   cache=HealthCache())
    return AgentCore(llm=chain, tools=spoken.tools, memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты РОББОКС", use_streaming=streaming)


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_turn_with_action_is_terminal_and_code_tells_the_outcome(streaming: bool) -> None:
    """Команда с действием → исполнитель команды → фраза кода по исходу; LLM второго слова не получает,
    её «Запомнила!» не звучит."""
    action = [{"tool": "memory_save", "args": {"fact": "Саша любит чай"}}]
    inner = _Inner(DEEPSEEK_CAPS, [LLMResponse(tool_calls=(_command_call(speech="Запомнила!", actions=action),))])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "запомни что Саша любит чай", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and len(inner.requests) == 1
    assert spoken.said == ["Запомнил: Саша любит чай."]
    assert list(result.tools_called) == ["memory_save", "speak_text"] and result.speak_text_real_count == 1
    assert result.spoken_text == ""
    tools = [t["function"]["name"] for t in inner.requests[0]["tools"]]
    assert tools[0] == kn.COMMAND_TOOL and "speak_text" not in tools


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_query_returns_data_to_llm_then_speech_is_final(streaming: bool) -> None:
    inner = _Inner(DEEPSEEK_CAPS, [
        LLMResponse(tool_calls=(_command_call(actions=[{"tool": "get_current_time", "args": {}}]),)),
        LLMResponse(tool_calls=(_command_call(speech="Сейчас полдень."),)),
    ])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "который час", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and result.spoken_text == "Сейчас полдень." and spoken.said == []
    second = inner.requests[1]
    assert second["settings"].tool_choice == "required"
    assert any(m.role == "tool" and "12:00" in str(m.content) for m in second["messages"])
