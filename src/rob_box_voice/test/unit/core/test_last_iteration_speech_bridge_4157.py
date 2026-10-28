"""#4157: форма последней итерации на мосту режима команд - схема требует ``speech``, ответ одним лицом получает повтор
с подсказкой; и весь ход v2 (``TurnEngine`` → ``V2Ports`` → мост) на живом 28.10 13:40 «скажи обо мне пять фактов»:
два чтения памяти, на последней итерации - одно лицо без слов; говорит код по данным, а не «не смог сложить в
короткую фразу».
"""

from __future__ import annotations

import asyncio
import json
import types
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import command as cm  # noqa: E402
from rob_box_dialog import knowledge as kn, phrases  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Form  # noqa: E402
from rob_box_dialog.session import Session, SpeakerRef  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import (LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities,  # noqa: E402
                                  ToolCall, ToolResult)
from rob_box_llm.providers.deepseek import DeepSeekProvider  # noqa: E402
from rob_box_voice.core import command_bridge as cb  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports, counted_wrapper  # noqa: E402

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
ASKED = Form(passed=False, voice=False, speech_required=True)
TIME = type(M.TurnEvent().stamp)
FACE = {"emotion": "happy", "duration_s": 12}  # ответ последнего вызова в живом логе: лицо без слов

HITS = [
    {"kind": "turn", "role": "user", "content": "я люблю кататься на велосипеде", "score": 0.99},
    {"kind": "fact", "content": "Денчик любит зелёный чай.", "score": 0.40, "category": "preference"},
    {"kind": "fact", "content": "Денчик попросил называть его Шифу.", "score": 0.91, "category": "preference"},
    {"kind": "fact", "content": "Денчик болеет за Спартак", "score": 0.75, "category": "general"},
]
FACTS_PHRASE = ("Вот что нашёл в памяти: Денчик попросил называть его Шифу; Денчик болеет за Спартак; "
                "Денчик любит зелёный чай.")


def _call(**fields: Any) -> ToolCall:
    return ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=fields)


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди, запросы - в ``requests``."""

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


def _after_query() -> List[LLMMessage]:
    """Запрос с данными ``query`` хода (их возвращает ``turn._query_data``): мост узнаёт по ним, что запрос выполнен."""
    data = json.dumps({kn.QUERY_RESULTS_FIELD: [{"tool": "memory_search", "status": "done", "data": {"results": HITS}}]},
                      ensure_ascii=False)
    return [LLMMessage(role="user", content=N_FACTS), LLMMessage(role="tool", content=data, tool_call_id="q1")]


N_FACTS = "робот а теперь скажи обо мне пять фактов которые ты знаешь"


# -- мост ---------------------------------------------------------------------------------------------------------


def test_asked_form_makes_speech_required_in_the_request_schema():
    bridge = cb.CommandModeLLM(_Inner([]))
    msgs = [LLMMessage(role="system", content="ты робот"), LLMMessage(role="user", content=N_FACTS)]
    with cm.turn_form(ASKED):
        _, tools, _ = bridge.shape(msgs, [_tool("memory_search")], None)
    assert tools[0]["function"]["parameters"]["required"] == [kn.SPEECH_FIELD]
    _, open_tools, _ = bridge.shape(msgs, [_tool("memory_search")], None)  # вне такой итерации - как было
    assert open_tools[0]["function"]["parameters"]["required"] == []


def test_bare_face_on_the_asked_form_is_retried_with_the_hint_and_the_words_are_spoken():
    inner = _Inner([LLMResponse(tool_calls=(_call(expression=FACE),)),
                    LLMResponse(tool_calls=(_call(speech="Помню: ты просил звать тебя Шифу."),))])
    lines: List[str] = []
    bridge = cb.CommandModeLLM(inner, log=lines.append)

    async def run() -> LLMResponse:
        with cm.turn_form(ASKED):
            return await bridge.complete(_after_query(), tools=[_tool("memory_search")])

    out = asyncio.run(run())
    assert out.content == "Помню: ты просил звать тебя Шифу." and not out.tool_calls
    note = next(m.content for m in inner.requests[1]["messages"] if m.role == "tool" and m.tool_call_id == "c1")
    assert kn.VIOLATION_SPEECH_REQUIRED in note and "данные запроса уже получены" in note
    assert any(f"violations=['{kn.VIOLATION_SPEECH_REQUIRED}']" in line for line in lines)
    assert kn.SPEECH_FIELD in _command_schema(inner.requests[0])["required"]


def test_bare_face_twice_goes_to_the_turn_as_the_speech_required_violation():
    inner = _Inner([LLMResponse(tool_calls=(_call(expression=FACE),)), LLMResponse(tool_calls=(_call(expression=FACE),))])
    bridge = cb.CommandModeLLM(inner)

    async def run() -> LLMResponse:
        with cm.turn_form(ASKED):
            return await bridge.complete(_after_query(), tools=[_tool("memory_search")])

    out = asyncio.run(run())
    assert len(inner.requests) == 2 and out.tool_calls[0].arguments[kn.COMMAND_INVALID_FIELD] == kn.VIOLATION_SPEECH_REQUIRED


def test_the_same_face_on_an_open_form_is_a_valid_command_without_retry():
    inner = _Inner([LLMResponse(tool_calls=(_call(expression=FACE),))])
    bridge = cb.CommandModeLLM(inner)
    out = asyncio.run(bridge.complete(_after_query(), tools=[_tool("memory_search")]))
    assert len(inner.requests) == 1 and kn.COMMAND_INVALID_FIELD not in out.tool_calls[0].arguments


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


class _Memory:
    """Тул памяти: на «любить» ничего, на остальные запросы - результаты как у ``memory_search``."""

    def __init__(self) -> None:
        self.calls: List[Any] = []

    async def execute(self, call: Any) -> ToolResult:
        self.calls.append((call.name, dict(call.arguments)))
        hits = [] if call.arguments.get("query") == "любить" else HITS
        return ToolResult(tool_call_id=call.id, content="ok", data={"results": hits, "total": len(hits)})


def _v2(replies: List[LLMResponse]) -> tuple:
    counter, node, tools = CallCounter(), _Node(), _Memory()
    inner = _Inner(replies)
    llm = counted_wrapper(lambda p: cb.CommandModeLLM(p), counter)(inner)
    return V2Ports(node, llm, counter, tools, intent_pub=_Pub()), node, inner, tools


def _said(node: _Node) -> List[str]:
    return [s.text for s in node.pubs["/voice/speech/request"].sent]


def _live_trace(last: List[LLMResponse]) -> tuple:
    """Живой 28.10 13:40: итерация 1 - ``memory_search``, итерация 2 - два поиска в одной команде, дальше - ``last``."""
    first = LLMResponse(tool_calls=(ToolCall(id="m1", name="memory_search",
                                             arguments={"query": "факт", "limit": 10}),))
    second = LLMResponse(tool_calls=(_call(actions=[{"tool": "memory_search", "args": {"query": "Денчик"}},
                                                    {"tool": "memory_search", "args": {"query": "любить"}}]),))
    ports, node, inner, tools = _v2([first, second, *last])
    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref="fcdbb976", name="Шифу", name_state="confirmed"))  # память — его
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed(N_FACTS, VOICE)))
    return agg, node, inner, tools


def test_five_facts_trace_face_without_words_twice_code_names_the_facts():
    """Лицо без слов на последней итерации: схема требовала ``speech``, мост дал повтор с подсказкой, и после него без слов -
    говорит код по данным результатов, а не «не смог сложить в короткую фразу»."""
    agg, node, inner, tools = _live_trace([LLMResponse(tool_calls=(_call(expression=FACE),)),
                                           LLMResponse(tool_calls=(_call(expression=FACE),))])
    assert [c[1].get("query") for c in tools.calls] == ["факт", "Денчик", "любить"]
    assert len(inner.requests) == kn.LLM_QUERY_ITERATIONS_MAX + 2, "последняя итерация + один повтор моста"
    assert [kn.SPEECH_FIELD in _command_schema(r)["required"] for r in inner.requests] == [False, False, True, True]
    said = _said(node)
    assert len(said) == 1 and FACTS_PHRASE in said[0]
    assert not any(v in said[0] for v in phrases.catalog()[kn.QUERY_FALLBACK_GENERIC])
    assert (agg.stage, agg.status) == ("done", "done")


def test_five_facts_trace_the_retry_gets_words_and_the_model_speaks_for_itself():
    agg, node, inner, _ = _live_trace([LLMResponse(tool_calls=(_call(expression=FACE),)),
                                       LLMResponse(tool_calls=(_call(speech="Ты просил называть тебя Шифу."),))])
    assert _said(node) == ["<speak>Ты просил называть тебя Шифу.</speak>"]
    assert (agg.stage, agg.status) == ("done", "finished") and cm.current_form() == Form()
