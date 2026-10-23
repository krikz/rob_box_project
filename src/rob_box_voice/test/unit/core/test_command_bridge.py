"""Мост режима команд в v1 (ADR-0150 §4.1, §4.5; #3696 ч.5, PR-7 ч.7c): форма запроса — по capabilities адаптера,
ответ — одна команда через ``rob_box_dialog.command.parse``; голая реплика — текстом, остальное — вызовом ``command``,
который v1-цикл ``AgentCore`` исполняет кодом (``command_turn``: событие владельца, фраза кода, ход терминален)."""

from __future__ import annotations

import asyncio
import dataclasses
import json
import re
from pathlib import Path
from types import SimpleNamespace
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

import pytest
import yaml

from rob_box_dialog import knowledge as kn
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import DialogueEvent, DialogueStateMachine
from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.tools import FakeToolProvider, ToolSpec
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.minimax import MiniMaxProvider
from rob_box_voice.core import command_bridge as cb
from rob_box_voice.core.dialogue_stage_mixin import DialogueStageMixin
from rob_box_voice.core.turn_stages import TurnStages

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
MINIMAX_CAPS = MiniMaxProvider(api_key="test").capabilities


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
    assert cb.CommandModeLLM(_Inner(MINIMAX_CAPS, [], name="minimax")).mode == "tool_offered"
    caps = ProviderCapabilities(tools=True, json_mode=True)
    assert cb.CommandModeLLM(_Inner(caps, [])).mode == "json_mode"


def test_tool_choice_request_forces_command_tool_and_hides_speak_text() -> None:
    bridge = cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, []))
    original = _messages()
    msgs, tools, settings = bridge.shape(original, [_tool("speak_text"), _tool("memory_save")],
                                         LLMSettings(temperature=0.3, extra={"thinking": {"type": "disabled"}}))
    assert [t["function"]["name"] for t in tools] == [kn.COMMAND_TOOL, "memory_save"]
    assert tools[0]["function"]["parameters"] == kn.with_tool_enum(kn.command_schema("json"), ["speak_text", "memory_save"])
    assert settings.tool_choice == "required" and settings.temperature == 0.3
    assert settings.extra == {"thinking": {"type": "disabled"}}
    assert msgs[0].content.startswith("ты РОББОКС") and cb.V1_TRANSPORT_NOTE in msgs[0].content
    assert kn.prompt_fragment() in msgs[0].content and "JSON-схема" not in msgs[0].content
    assert msgs[1:] == original[1:] and original[0].content == "ты РОББОКС"  # исходный список не тронут


def test_json_in_text_request_keeps_native_tools_and_puts_schema_in_prompt() -> None:
    bridge = cb.CommandModeLLM(_Inner(ProviderCapabilities(text=True), [], name="plain"))
    assert bridge.mode == "json_in_text"
    msgs, tools, settings = bridge.shape(_messages(), [_tool("speak_text"), _tool("get_current_time")], None)
    assert [t["function"]["name"] for t in tools] == ["get_current_time"]
    assert settings.tool_choice is None and "response_format" not in settings.extra
    note = msgs[0].content
    assert "JSON-схема команды: " + json.dumps(kn.with_tool_enum(kn.command_schema("json"), ["speak_text", "get_current_time"]), ensure_ascii=False) in note
    assert "одним JSON-объектом" in note


def test_tool_offered_request_offers_command_tool_without_tool_choice() -> None:
    """MiniMax (#3698 ч.7d): тул ``command`` со схемой — в ``tools``, ``tool_choice`` не шлётся (ADR-0143), схемы в
    промпте нет; тул речи скрыт, свои тулы остаются."""
    bridge = cb.CommandModeLLM(_Inner(MINIMAX_CAPS, [], name="minimax"))
    msgs, tools, settings = bridge.shape(_messages(), [_tool("speak_text"), _tool("get_current_time")],
                                         LLMSettings(temperature=0.7))
    assert [t["function"]["name"] for t in tools] == [kn.COMMAND_TOOL, "get_current_time"]
    assert tools[0]["function"]["parameters"] == kn.with_tool_enum(kn.command_schema("json"), ["speak_text", "get_current_time"])
    assert settings.tool_choice is None and settings.temperature == 0.7
    assert kn.prompt_fragment() in msgs[0].content and "JSON-схема" not in msgs[0].content


def test_tool_offered_accepts_command_call_or_json_in_text() -> None:
    """Без принуждения модель отвечает и вызовом ``command``, и JSON-ом в тексте — оба дают одну команду."""
    called = _convert(MINIMAX_CAPS, LLMResponse(tool_calls=(_command_call(speech="Привет!"),)))
    texted = _convert(MINIMAX_CAPS, LLMResponse(content='{"speech": "Привет!"}'))
    assert called.content == texted.content == "Привет!" and not called.tool_calls and not texted.tool_calls


def test_two_command_calls_in_one_answer_are_one_violation_and_go_to_the_retry() -> None:
    """Два вызова ``command`` в одном ответе — ``command.parse`` → ``several_commands``: повтор с нарушением, а не
    первая команда молча (что ответила бы модель на втором — неизвестно)."""
    two = LLMResponse(tool_calls=(_command_call(speech="а"), ToolCall(id="c2", name=kn.COMMAND_TOOL,
                                                                      arguments={"speech": "б"})))
    result = cb.judge(two, ["get_current_time"])
    assert getattr(result, "reason", None) == "several_commands"


def test_json_mode_sets_response_format() -> None:
    bridge = cb.CommandModeLLM(_Inner(ProviderCapabilities(tools=True, json_mode=True), []))
    _, _, settings = bridge.shape([LLMMessage(role="user", content="привет")], [], None)
    assert settings.extra["response_format"] == {"type": "json_object"}


def test_note_becomes_first_system_message_when_there_is_none() -> None:
    msgs, _, _ = cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, [])).shape([LLMMessage(role="user", content="x")], [], None)
    assert msgs[0].role == "system" and cb.V1_TRANSPORT_NOTE in msgs[0].content and msgs[1].content == "x"


# ── разбор ответа и перевод в форму v1 ─────────────────────────────────────────────────────────────────────


#: Срез тулов запроса: команда проверяется по нему (``command.validate``), аргументы — по схеме каталога.
TOOLS = [_tool(n) for n in ("memory_save", "get_current_time", "search_web", "get_battery_level", "play_sound")]


def _convert(caps: ProviderCapabilities, reply: LLMResponse, log: Optional[list] = None) -> LLMResponse:
    lines: list = [] if log is None else log
    bridge = cb.CommandModeLLM(_Inner(caps, [reply]), lines.append)
    return _run(bridge.complete(_messages(), tools=TOOLS))


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


def test_actions_go_as_one_command_call() -> None:
    log: list = []
    reply = LLMResponse(tool_calls=(
        _command_call(actions=[{"tool": "memory_save", "args": {"fact": "чай"}}]),
        ToolCall(id="n1", name="get_current_time", arguments={}),
    ))
    out = _convert(DEEPSEEK_CAPS, reply, log)
    assert out.finish_reason == "tool_calls" and out.tool_calls[0].id.startswith(cb.CALL_ID_PREFIX)
    assert _actions(out) == [("memory_save", {"fact": "чай"}), ("get_current_time", {})]
    assert log == [f"{cb.COMMAND_LOG_MARK} provider=deepseek mode=tool_choice verdict=command "
                   "actions=['memory_save', 'get_current_time'] speech_dropped=0"]


def test_face_slots_beside_expression_are_moved_and_logged_not_invalid() -> None:
    """Живой прогон 7c (#3698): DeepSeek кладёт слоты лица в ``expression`` — команда исполняется, поправка в логе."""
    log: list = []
    reply = LLMResponse(tool_calls=(_command_call(
        actions=[{"tool": "memory_save", "args": {"fact": "чай"}}],
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
    reply = LLMResponse(content='{"expression": {"emotion": "happy"}}', tool_calls=(search,))
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
    (LLMResponse(content='{"expression": {"emotion": "happy"}}'), kn.VIOLATION_EMPTY_COMMAND),
    (LLMResponse(tool_calls=(_command_call(speech=""),)), kn.VIOLATION_EMPTY_COMMAND),
])
def test_invalid_twice_marks_the_command_invalid_after_one_retry(reply: LLMResponse, reason: str) -> None:
    """Повтор ровно один (``COMMAND_RETRIES_MAX``): второй невалидный ответ — вызов ``command`` с пометкой, без речи."""
    log: list = []
    inner = _Inner(MINIMAX_CAPS, [reply, reply], name="minimax")
    out = _run(cb.CommandModeLLM(inner, log.append).complete(_messages(), tools=TOOLS))
    (call,) = out.tool_calls
    assert out.content == "" and call.name == kn.COMMAND_TOOL and len(inner.requests) == 2
    assert call.arguments == {kn.COMMAND_INVALID_FIELD: reason}
    assert f"verdict=invalid:{reason}" in log[0] and f"verdict=invalid:{reason}" in log[1]
    assert log[2].startswith(f"{cb.COMMAND_LOG_MARK} retry=1 violations=['{reason}'] → verdict=invalid:{reason} (")


def test_passed_is_honest_silence_carried_as_command() -> None:
    """#3859: молчание — вызов ``command`` с одним полем ``passed`` (пустой ответ потребитель принял бы за ошибку)."""
    out = _convert(DEEPSEEK_CAPS, LLMResponse(tool_calls=(_command_call(passed="nothing_to_say"),)))
    (call,) = out.tool_calls
    assert out.content == "" and call.name == kn.COMMAND_TOOL and call.arguments == {"passed": "nothing_to_say"}


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
    (LLMResponse(content='{"actions": [{"tool": "memory_save", "args": {"fact": "чай"}}]}'), "", [kn.COMMAND_TOOL]),
])
def test_stream_is_collected_then_replayed_in_v1_shape(reply: LLMResponse, content: str, calls: list) -> None:
    inner = _Inner(DEEPSEEK_CAPS, [reply])
    bridge = cb.CommandModeLLM(inner)

    async def _drain() -> list:
        return [chunk async for chunk in bridge.stream(_messages(), tools=TOOLS)]

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


_REPO = Path(__file__).resolve().parents[5]
_NODE_YAMLS = (_REPO / "src/rob_box_voice/config/dialogue_node.yaml",
               _REPO / "docker/sound/config/voice_assistant/dialogue_node.yaml")


def test_command_mode_default_has_one_source_in_knowledge() -> None:
    """Значение по умолчанию - одно (``knowledge``): параметр берёт его оттуда, yaml его не дублируют."""
    src = (_REPO / "src/rob_box_voice/rob_box_voice/dialogue_node.py").read_text(encoding="utf-8")
    match = re.search(r"declare_parameter\(\s*[\"']llm_command_providers[\"']\s*,\s*([^)]+?)\s*\)", src)
    assert match and match.group(1) == "COMMAND_PROVIDERS_DEFAULT"
    for path in _NODE_YAMLS:
        params = yaml.safe_load(path.read_text(encoding="utf-8"))["dialogue_node"]["ros__parameters"]
        assert not params.get("llm_command_providers"), f"{path}: второй источник значения (пустая строка - выключатель)"


def test_command_mode_is_on_by_default_for_deepseek_and_minimax_fallback() -> None:
    """Решения Шифу 22.10 (#3698): без конфига командой отвечают DeepSeek (``tool_choice``) и MiniMax-фолбек
    (``tool_offered``, замер ч.7d); прочие провайдеры — старый путь v1."""
    assert kn.COMMAND_PROVIDERS_DEFAULT == "deepseek,minimax"
    wrap = cb.wrapper_for(kn.COMMAND_PROVIDERS_DEFAULT)
    deepseek, minimax = _Inner(DEEPSEEK_CAPS, []), _Inner(MINIMAX_CAPS, [], name="minimax")
    mimo = _Inner(ProviderCapabilities(tools=True), [], name="mimo")
    assert wrap(deepseek).mode == "tool_choice" and wrap(minimax).mode == "tool_offered" and wrap(mimo) is mimo


# ── сквозной ход через v1-цикл AgentCore и цепочку здоровья ────────────────────────────────────────────────


def _dsm() -> DialogueStateMachine:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return dsm


class _OwnerTools(FakeToolProvider):
    """Как реальный MCP: успех ``memory_save`` несёт в данных результата ``saved{fact_id}`` владельца памяти."""

    async def execute(self, call):  # type: ignore[no-untyped-def]
        result = await super().execute(call)
        if call.name == "memory_save" and not result.is_error:
            return dataclasses.replace(result, data={"event": "saved", "fact_id": 1})
        return result


class _Spoken:
    """Тулы хода v1: ``memory_save``, ``get_current_time`` и ``speak_text`` с журналом реплик."""

    def __init__(self) -> None:
        self.said: List[str] = []
        self.tools = _OwnerTools()
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


class _Node(DialogueStageMixin):
    """Узел v1 в части стадий хода: ``TurnStages`` с записью рефлексов, открытый ход личности."""

    def __init__(self) -> None:
        self.rig = SimpleNamespace(requests=[], events=[])
        self._stages = TurnStages(lambda **kw: self.rig.requests.append(kw), lambda **kw: self.rig.events.append(kw))
        self._stages.addressed("u1")
        self._turn_id = self._stages.turn_id_for("u1")
        self.stages = SimpleNamespace(
            requests=self.rig.requests, stage_names=lambda: [(e["stage"], e["status"]) for e in self.rig.events])


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_turn_with_action_is_terminal_and_code_tells_the_outcome(streaming: bool) -> None:
    """Команда с действием → исполнитель команды → фраза кода по исходу; LLM второго слова не получает."""
    action = [{"tool": "memory_save", "args": {"fact": "Саша любит чай"}}]
    inner = _Inner(DEEPSEEK_CAPS, [LLMResponse(tool_calls=(_command_call(actions=action),))])
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


class _Down(_Inner):
    """DeepSeek лёг (нет денег / сеть): любой запрос падает, цепочка здоровья идёт к следующему."""

    def _take(self, messages: Iterable[LLMMessage], tools: Iterable[Mapping[str, Any]],
              settings: Optional[LLMSettings]) -> LLMResponse:
        self.requests.append({"messages": list(messages), "tools": list(tools), "settings": settings})
        raise RuntimeError("deepseek is down")


@pytest.mark.parametrize("streaming", [False, True])
def test_minimax_fallback_answers_with_the_same_command_by_default(streaming: bool) -> None:
    """#3698 ч.7d: DeepSeek лёг — MiniMax-фолбек по умолчанию отвечает той же командой (тул ``command`` без
    ``tool_choice``)."""
    down = _Down(DEEPSEEK_CAPS, [])
    minimax = _Inner(MINIMAX_CAPS, [LLMResponse(tool_calls=(_command_call(speech="Привет, я РОББОКС."),))],
                     name="minimax")
    wrap = cb.wrapper_for(kn.COMMAND_PROVIDERS_DEFAULT)
    chain = HealthAwareFallbackLLM([wrap(down), wrap(minimax)], cache=HealthCache())
    core = AgentCore(llm=chain, tools=_Spoken().tools, memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты РОББОКС", use_streaming=streaming)
    result = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and down.requests and result.spoken_text == "Привет, я РОББОКС."
    offered = [t["function"]["name"] for t in minimax.requests[0]["tools"]]
    assert offered[0] == kn.COMMAND_TOOL and "speak_text" not in offered
    assert minimax.requests[0]["settings"].tool_choice is None


@pytest.mark.parametrize("streaming", [False, True])
def test_fallback_to_a_provider_outside_the_list_keeps_the_old_v1_path(streaming: bool) -> None:
    """Провайдер вне списка ``llm_command_providers`` - свои тулы и ``speak_text``, без команды."""
    down = _Down(DEEPSEEK_CAPS, [])
    say = ToolCall(id="s1", name="speak_text", arguments={"text": "Привет, я РОББОКС."})
    minimax = _Inner(MINIMAX_CAPS, [LLMResponse(tool_calls=(say,)), LLMResponse(content="")], name="minimax")
    spoken = _Spoken()
    wrap = cb.wrapper_for("deepseek")
    chain = HealthAwareFallbackLLM([wrap(down), wrap(minimax)], cache=HealthCache())
    core = AgentCore(llm=chain, tools=spoken.tools, memory=InMemoryStore(), dsm=_dsm(),
                     system_prompt="ты РОББОКС", use_streaming=streaming)
    result = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and down.requests
    assert spoken.said == ["Привет, я РОББОКС."]
    offered = [t["function"]["name"] for t in minimax.requests[0]["tools"]]
    assert kn.COMMAND_TOOL not in offered and "speak_text" in offered
    assert minimax.requests[0]["settings"] is None or minimax.requests[0]["settings"].tool_choice != "required"


# ── повтор невалидной команды с перечнем нарушений (решение Шифу 21.10, #3698) ─────────────────────────────────


def _answer(caps: ProviderCapabilities, replies: List[LLMResponse], name: str = "deepseek") -> tuple:
    log: list = []
    inner = _Inner(caps, replies, name=name)
    out = _run(cb.CommandModeLLM(inner, log.append).complete(_messages(), tools=TOOLS))
    return out, inner, log


SOUND = [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]
#: Действие, которое есть в стенде ``_core`` (play_sound там нет); фразу об исходе говорит код.
SAVE = [{"tool": "memory_save", "args": {"fact": "Саша любит чай"}}]
SAVED = "Запомнил: Саша любит чай."


def test_invalid_command_goes_back_as_tool_result_and_the_fixed_one_is_executed() -> None:
    """Живой прогон 7c: ``speech`` рядом с ``perform`` — модель получает нарушение на свой вызов и исправляется."""
    tale = {"kind": "tale", "segments": ["Жил-был ёж.", "И жил он долго."]}
    bad = LLMResponse(tool_calls=(_command_call(speech="Слушай!", perform=tale, actions=SOUND),))
    fixed = {"perform": tale, "actions": SOUND}
    good = LLMResponse(tool_calls=(ToolCall(id="c2", name=kn.COMMAND_TOOL, arguments=fixed),))
    out, inner, log = _answer(DEEPSEEK_CAPS, [bad, good])

    assert _actions(out) == [("play_sound", {"sound": "robot_happy"})]
    assert len(inner.requests) == 2
    first, second = inner.requests[0]["messages"], inner.requests[1]["messages"]
    assert second[:len(first)] == first  # тот же запрос, не новый ход и не новый промпт
    said, result = second[len(first):]
    assert said.role == "assistant" and said.tool_calls == bad.tool_calls
    assert result.role == "tool" and result.tool_call_id == "c1" and result.tool_result.is_error
    assert result.content.startswith("invalid arguments") and "при perform поле speech не пиши" in result.content
    assert inner.requests[1]["settings"].tool_choice == "required"
    assert "verdict=invalid:perform_with_speech" in log[0] and "verdict=command" in log[1]
    assert log[2].startswith(f"{cb.COMMAND_LOG_MARK} retry=1 violations=['perform_with_speech'] → verdict=command (")


def test_enum_violation_lists_the_allowed_values_from_knowledge() -> None:
    bad = LLMResponse(tool_calls=(_command_call(speech="Привет", expression={"emotion": "ecstatic"}),))
    out, inner, _ = _answer(DEEPSEEK_CAPS, [bad, LLMResponse(tool_calls=(_command_call(speech="Привет"),))])
    assert out.content == "Привет"
    note = inner.requests[1]["messages"][-1].content
    assert "emotion=ecstatic" in note and f"Допустимо: {', '.join(kn.EMOTIONS)}" in note


def test_bad_json_arguments_get_one_retry() -> None:
    """Адаптер отдаёт ``{}``, когда JSON аргументов не разобрался: это ``bad_json``, а не пустая команда."""
    broken = LLMResponse(tool_calls=(_command_call(),))
    out, inner, log = _answer(DEEPSEEK_CAPS, [broken, LLMResponse(tool_calls=(_command_call(speech="Сейчас."),))])
    assert out.content == "Сейчас." and len(inner.requests) == 2
    assert "verdict=invalid:bad_json" in log[0]
    assert "аргументы command не разобраны" in inner.requests[1]["messages"][-1].content


def test_json_in_text_retry_goes_as_user_message() -> None:
    """MiniMax (запасной, JSON в тексте): вызова нет — нарушения идут репликой пользователя после ответа модели."""
    bad = LLMResponse(content='{"speech": "Привет", "mood": "весело"}')
    out, inner, _ = _answer(MINIMAX_CAPS, [bad, LLMResponse(content='{"speech": "Привет"}')], name="minimax")
    assert out.content == "Привет"
    said, note = inner.requests[1]["messages"][-2:]
    assert (said.role, said.content) == ("assistant", bad.content)
    assert note.role == "user" and "unknown_field" in note.content and "Допустимо: speech" in note.content


def test_other_calls_of_an_invalid_answer_get_not_run_results() -> None:
    bad = LLMResponse(tool_calls=(_command_call(speech="x", when="later"),
                                  ToolCall(id="n1", name="get_current_time", arguments={})))
    _, inner, _ = _answer(DEEPSEEK_CAPS, [bad, LLMResponse(tool_calls=(_command_call(speech="Полдень."),))])
    results = [m for m in inner.requests[1]["messages"] if m.role == "tool"]
    assert [m.tool_call_id for m in results] == ["c1", "n1"]
    assert "when=later" in results[0].content and results[1].content == cb.NOT_RUN


def test_layer_one_repair_needs_no_retry() -> None:
    reply = LLMResponse(tool_calls=(_command_call(actions=SOUND, expression={"emotion": "happy", "eyes": "wide"}),))
    out, inner, log = _answer(DEEPSEEK_CAPS, [reply])
    assert _actions(out) == [("play_sound", {"sound": "robot_happy"})] and len(inner.requests) == 1
    assert not any("retry=" in line for line in log)


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_invalid_then_fixed_command_executes_the_action(streaming: bool) -> None:
    """Сквозной ход: невалидная команда → повтор внутри моста → действие исполнено, фраза кода; «Не понял» не звучит,
    ход один."""
    bad = [{"tool": "memory_save", "args": {"fact": "Саша любит чай", "category": "food"}}]
    good = [{"tool": "memory_save", "args": {"fact": "Саша любит чай", "category": "preference"}}]
    inner = _Inner(DEEPSEEK_CAPS, [LLMResponse(tool_calls=(_command_call(actions=bad),)),
                                   LLMResponse(tool_calls=(_command_call(actions=good),))])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "запомни что Саша любит чай", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and len(inner.requests) == 2
    assert "bad_args" in inner.requests[1]["messages"][-1].content
    assert spoken.said == ["Запомнил: Саша любит чай."] and result.turn_failure == ""


@pytest.mark.parametrize("streaming", [False, True])
def test_joke_in_speech_beside_a_sound_is_retried_into_perform_and_the_sound_stays_silent(streaming: bool) -> None:
    """Живой прогон 7c (#3698, вариант 2): шутка в ``speech`` рядом с ``play_sound`` раньше пропадала молча; теперь
    модель получает нарушение и кладёт шутку в ``perform`` - она звучит, о звуке код молчит."""
    joke = {"kind": "tale", "segments": ["Заходит робот в бар.", "А бар ему: опять ты со своими битами!"]}
    bad = LLMResponse(tool_calls=(_command_call(speech="Вот звук! Шутка: заходит робот в бар.", actions=SAVE),))
    good = LLMResponse(tool_calls=(_command_call(perform=joke, actions=SAVE),))
    inner = _Inner(DEEPSEEK_CAPS, [bad, good])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "включи звук радостный и расскажи шутку", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and len(inner.requests) == 2 and result.turn_failure == ""
    assert "speech_with_actions" in inner.requests[1]["messages"][-1].content
    assert spoken.said == [SAVED, *joke["segments"]]


@pytest.mark.parametrize("streaming", [False, True])
def test_words_about_the_action_in_speech_are_retried_away_and_never_spoken(streaming: bool) -> None:
    bad = LLMResponse(tool_calls=(_command_call(speech="Вот запомнил!", actions=SAVE),))
    good = LLMResponse(tool_calls=(_command_call(actions=SAVE),))
    inner = _Inner(DEEPSEEK_CAPS, [bad, good])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "включи звук радостный", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and len(inner.requests) == 2 and spoken.said == [SAVED]
    assert "убери speech" in inner.requests[1]["messages"][-1].content


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_invalid_twice_fails_the_turn_without_speech_and_next_turn_is_normal(streaming: bool) -> None:
    bad = LLMResponse(tool_calls=(_command_call(actions=[{"tool": "memory_save", "args": {}}]),))
    good = LLMResponse(tool_calls=(_command_call(speech="Привет, Шифу."),))
    inner = _Inner(DEEPSEEK_CAPS, [bad, bad, good])
    spoken = _Spoken()
    core = _core(inner, streaming, spoken)
    result = asyncio.run(core.process_input("запомни", preclassified_event=DialogueEvent.STT_RESULT))
    assert len(inner.requests) == 2 and result.spoken_text == "" and spoken.said == []
    assert result.turn_failure.startswith("invalid_command") and result.error is None

    # Узел закрывает ход рефлексом failed (лицо error, earcon fail; кольцо flash_red — в таблице рефлексов) — тем же путём, что провал тула.
    node = _Node()
    node._stage_finish(result, False, "")
    assert node.stages.stage_names()[-1] == ("failed", "failed")
    assert (node.stages.requests[-1]["emotion"], node.stages.requests[-1]["earcon"]) == ("error", "fail")

    # Следующий ход идёт обычно: никаких хвостов состояния.
    core._dsm.on_event(DialogueEvent.WAKE_WORD)  # новая реплика с вейк-словом, как у узла
    core._dsm.on_event(DialogueEvent.STT_RESULT)
    nxt = asyncio.run(core.process_input("привет", preclassified_event=DialogueEvent.STT_RESULT))
    assert nxt.turn_failure == "" and nxt.spoken_text == "Привет, Шифу."


# ── пустая команда — нарушение команды, а не «тихий ответ» v1 (ночной замер #3698 22.10, ход 4) ──────────────────


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_empty_command_is_retried_by_the_bridge_not_by_v1(streaming: bool) -> None:
    """Пустой ответ → один повтор моста с объяснением → исправленная команда исполнена. Ретрая #1217 (SYSTEM
    CORRECTION, третий запрос к LLM) нет: ровно 2 запроса."""
    empty = LLMResponse(tool_calls=(_command_call(expression={"emotion": "happy"}),))
    good = LLMResponse(tool_calls=(ToolCall(id="c2", name=kn.COMMAND_TOOL, arguments={"actions": SAVE}),))
    inner = _Inner(DEEPSEEK_CAPS, [empty, good])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "запомни что Саша любит чай", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.error is None and len(inner.requests) == 2 and result.turn_failure == ""
    assert spoken.said == [SAVED]
    assert kn.VIOLATION_EMPTY_COMMAND in inner.requests[1]["messages"][-1].content


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_empty_twice_fails_the_turn_without_a_third_llm_call(streaming: bool) -> None:
    empty = LLMResponse(tool_calls=(_command_call(expression={"emotion": "happy"}),))
    inner = _Inner(DEEPSEEK_CAPS, [empty, empty])
    spoken = _Spoken()
    result = asyncio.run(_core(inner, streaming, spoken).process_input(
        "включи звук радостный", preclassified_event=DialogueEvent.STT_RESULT))

    assert len(inner.requests) == 2 and result.spoken_text == "" and spoken.said == []
    assert result.turn_failure == f"invalid_command:{kn.VIOLATION_EMPTY_COMMAND}" and result.error is None


def test_telegram_agent_may_write_text_beside_actions_personality_may_not() -> None:
    """PR-12a (#3706): мост проверяет команду для своего агента — у телеграма ``speech`` при действиях — текст в чат
    (законно), у Личности — нарушение ``speech_with_actions`` (§4.1). Цепочка телеграма — ``wrapper_for(agent=)``."""
    reply = LLMResponse(tool_calls=(_command_call(speech="Записываю.", actions=SAVE),))
    assert cb.judge(reply, ["memory_save"]).reason == kn.VIOLATION_SPEECH_WITH_ACTIONS
    chat = cb.judge(reply, ["memory_save"], agent=kn.TEXT_CHANNEL_AGENT)
    assert chat.speech == "Записываю." and [a.tool for a in chat.actions] == ["memory_save"]
    wrapped = cb.wrapper_for("deepseek", agent=kn.TEXT_CHANNEL_AGENT)(_Inner(DEEPSEEK_CAPS, [reply]))
    assert wrapped._judged(reply, ["memory_save"]).speech == "Записываю."


# ── тул вне среза хода (#3897, ADR-0148) ───────────────────────────────────────────────────────────────────


MUSIC_SLICE = [_tool(n) for n in ("request_music", "stop_music", "dj_set", "memory_save", "set_volume")]


def test_command_schema_enum_is_the_slice_of_this_turn_not_the_whole_catalog() -> None:
    bridge = cb.CommandModeLLM(_Inner(DEEPSEEK_CAPS, []))
    _, tools, _ = bridge.shape(_messages(), [_tool("speak_text"), *MUSIC_SLICE, _tool("not_in_catalog")], None)
    enum = tools[0]["function"]["parameters"]["properties"]["actions"]["items"]["properties"]["tool"]["enum"]
    assert enum == sorted(["speak_text", "request_music", "stop_music", "dj_set", "memory_save", "set_volume"])
    assert "play_sound" not in enum
    assert "enum" not in kn.command_schema("json")["properties"]["actions"]["items"]["properties"]["tool"]


def test_tool_outside_the_slice_is_retried_with_same_class_tools_and_the_fixed_action_runs() -> None:
    wrong = LLMResponse(tool_calls=(_command_call(actions=[{"tool": "play_sound", "args": {"name": "beat"}}]),))
    right = LLMResponse(tool_calls=(_command_call(actions=[{"tool": "request_music", "args": {"intent": "track", "text": "ровный ритм"}}]),))
    log: list = []
    inner = _Inner(DEEPSEEK_CAPS, [wrong, right])
    out = _run(cb.CommandModeLLM(inner, log.append).complete(_messages(), tools=MUSIC_SLICE))

    note = inner.requests[1]["messages"][-1].content
    assert "tool_not_available" in note and "(play_sound)" in note
    assert "Допустимо: dj_set, request_music, stop_music" in note  # только класс action.media, отсортирован
    assert "memory_save" not in note and "set_volume" not in note
    assert _actions(out) == [("request_music", {"intent": "track", "text": "ровный ритм"})]
    assert "retry=1 violations=['tool_not_available'] → verdict=command" in log[-1]


def _query_followup() -> list:
    """Запрос второй итерации хода: данные ``query`` вернулись модели (как их возвращает ``turn.py``)."""
    data = '{"query_results": [{"tool": "list_tts_voices", "status": "done", "reason": null, "data": {}}]}'
    return [*_messages(), LLMMessage(role="user", content=data)]


def test_retry_after_query_says_data_received_not_nothing_done() -> None:
    """#3883: речь длиннее потолка после выполненного запроса - подсказка повтора «данные получены, сократи до N»."""
    long = LLMResponse(tool_calls=(_command_call(speech="а" * (kn.SPEECH_MAX_CHARS + 63)),))
    good = LLMResponse(tool_calls=(_command_call(speech="Коротко."),))
    inner = _Inner(DEEPSEEK_CAPS, [long, good])
    out = _run(cb.CommandModeLLM(inner, [].append).complete(_query_followup(), tools=TOOLS))
    assert out.content == "Коротко." and len(inner.requests) == 2
    hint = inner.requests[1]["messages"][-1].content
    assert "too_long" in hint and str(kn.SPEECH_MAX_CHARS) in hint
    assert "получены" in hint and "ничего не сделано" not in hint


def test_retry_without_query_keeps_the_plain_hint() -> None:
    long = LLMResponse(tool_calls=(_command_call(speech="а" * (kn.SPEECH_MAX_CHARS + 1)),))
    good = LLMResponse(tool_calls=(_command_call(speech="Коротко."),))
    inner = _Inner(DEEPSEEK_CAPS, [long, good])
    _run(cb.CommandModeLLM(inner, [].append).complete(_messages(), tools=TOOLS))
    assert "ничего не сделано" in inner.requests[1]["messages"][-1].content
