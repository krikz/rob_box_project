"""Issue #3220 — thinking MiniMax по ходам и его побочки (инкремент #3136).

Что проверяем (провайдер — настоящий ``HarnessMiniMaxProvider`` поверх
настоящего upstream ``MiniMaxProvider``; подменён только клиент SDK, как
в остальных тестах провайдера):

* обычный ход → в запросе ``thinking={"type": "disabled"}``, как раньше;
* ход с рассуждением (``TURN_REASONING``) → поля ``thinking`` в запросе нет
  (дефолт модели — единственный формат, виденный вживую, b5879b79→6901a14e),
  ``max_tokens`` с запасом на ``<think>``; вызов после результата тула —
  снова без thinking;
* строка ``LLM REQUEST START`` называет режим thinking (след для живого
  прогона);
* first-chunk-гуард #2718: ``complete()`` хода с рассуждением не рубится на
  10-й секунде; в стриме рассуждение отдельным полем считается «первым
  чанком», а мёртвый провайдер по-прежнему уходит в фолбек;
* ``<think>`` не ломает сборку tool-call из стрима и не попадает ни в
  ответ AgentCore, ни в восстановление вызова из текста (#2760).
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any, AsyncIterator, Awaitable, Callable

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_loop.markup_recovery import parse_tool_call_markup
from rob_box_harness.memory.base import InMemoryStore
from rob_box_harness.providers import minimax as harness_minimax
from rob_box_harness.providers.minimax import (
    FirstChunkTimeoutError,
    HarnessMiniMaxProvider,
    RetryPolicy,
)
from rob_box_harness.providers.reasoning import (
    REASONING_MAX_TOKENS_HEADROOM,
    TURN_REASONING,
    strip_reasoning,
    without_reasoning,
)
from rob_box_harness.tools import FakeToolProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse

DISABLED = {"type": "disabled"}

#: Рассуждение, в котором модель «пишет» вызов тула разметкой протокола —
#: ровно то, что #2760 восстанавливает из ``content`` и исполняет.
THINK_WITH_MARKUP = (
    "<think>Юзер просит осеннюю тему. План: "
    '<invoke name="compose_music"><parameter name="style">club</parameter>'
    "</invoke> и потом скажу пару слов.</think>"
)
COMPOSE_TOOL = {
    "type": "function",
    "function": {
        "name": "compose_music",
        "parameters": {"type": "object", "properties": {"style": {"type": "string"}}},
    },
}


# ── двойник клиента SDK ─────────────────────────────────────────────────


def _reply(content: str) -> Any:
    message = SimpleNamespace(content=content, tool_calls=None)
    return SimpleNamespace(
        choices=[SimpleNamespace(message=message, finish_reason="stop")],
        usage=None,
        base_resp={},
    )


def _event(
    content: str | None = None,
    *,
    reasoning: str | None = None,
    tool: tuple[str, str] | None = None,
    finish: str | None = None,
) -> Any:
    tool_calls = None
    if tool is not None:
        tool_calls = [
            SimpleNamespace(
                index=0,
                id="call_1",
                function=SimpleNamespace(name=tool[0], arguments=tool[1]),
            )
        ]
    delta = SimpleNamespace(content=content, tool_calls=tool_calls, reasoning_content=reasoning)
    return SimpleNamespace(
        choices=[SimpleNamespace(delta=delta, finish_reason=finish)],
        usage=None,
        base_resp={},
    )


class _Completions:
    def __init__(
        self,
        *,
        reply: Any = None,
        events: tuple[tuple[float, Any], ...] = (),
        complete_delay_s: float = 0.0,
    ) -> None:
        self.reply = reply if reply is not None else _reply("ок")
        self.events = events
        self.complete_delay_s = complete_delay_s
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        if kwargs.get("stream"):
            return self._stream()
        await asyncio.sleep(self.complete_delay_s)
        return self.reply

    async def _stream(self) -> AsyncIterator[Any]:
        for delay_s, event in self.events:
            await asyncio.sleep(delay_s)
            yield event


class _Client:
    def __init__(self, completions: _Completions) -> None:
        self.chat = SimpleNamespace(completions=completions)
        self.is_closed = False

    async def close(self) -> None:
        self.is_closed = True


def _provider(
    completions: _Completions, *, first_chunk_s: float | None = 10.0
) -> HarnessMiniMaxProvider:
    return HarnessMiniMaxProvider(
        api_key="sk-test",
        client=_Client(completions),  # type: ignore[arg-type]
        first_chunk_timeout_s=first_chunk_s,
        retry=RetryPolicy(max_attempts=1),
    )


def _user_turn() -> list[LLMMessage]:
    return [
        LLMMessage(role="system", content="ты робот"),
        LLMMessage(role="user", content="[DJ_AUTO — ПЕРЕХОД #2] смени трек"),
    ]


def _after_tool_result() -> list[LLMMessage]:
    return _user_turn() + [
        LLMMessage(role="assistant", content=""),
        LLMMessage(role="tool", content='{"ok": true}'),
    ]


async def _drain(provider: HarnessMiniMaxProvider, messages: list[LLMMessage], **kw: Any) -> list[LLMChunk]:
    return [chunk async for chunk in provider.stream(messages, **kw)]


async def _call(provider: HarnessMiniMaxProvider, messages: list[LLMMessage], *, stream: bool) -> Any:
    if stream:
        return await _drain(provider, messages)
    return await provider.complete(messages)


def _run(coro_fn: Callable[[], Awaitable[Any]], *, reasoning: bool) -> Any:
    """Прогнать корутину «внутри хода» с данным решением о thinking."""

    async def _turn() -> Any:
        token = TURN_REASONING.set(reasoning)
        try:
            return await coro_fn()
        finally:
            TURN_REASONING.reset(token)

    return asyncio.run(_turn())


def _stream_events_ok() -> tuple[tuple[float, Any], ...]:
    return ((0.0, _event("ок")), (0.0, _event(finish="stop")))


# ── какой запрос уходит ─────────────────────────────────────────────────


@pytest.mark.parametrize("stream", [False, True])
def test_ordinary_voice_turn_keeps_thinking_disabled(stream: bool) -> None:
    completions = _Completions(events=_stream_events_ok())
    p = _provider(completions)

    _run(lambda: _call(p, _user_turn(), stream=stream), reasoning=False)

    sent = completions.calls[0]
    assert sent["extra_body"]["thinking"] == DISABLED
    assert sent["max_tokens"] == 4096


@pytest.mark.parametrize("stream", [False, True])
def test_reasoning_turn_sends_no_thinking_field_and_token_headroom(stream: bool) -> None:
    completions = _Completions(events=_stream_events_ok())
    p = _provider(completions)

    _run(lambda: _call(p, _user_turn(), stream=stream), reasoning=True)

    sent = completions.calls[0]
    assert "thinking" not in sent.get("extra_body", {}), (
        f"ход с рассуждением прислал thinking={sent['extra_body'].get('thinking')!r}: "
        "включённый режим — отсутствие поля (дефолт модели)"
    )
    assert sent["max_tokens"] == 4096 + REASONING_MAX_TOKENS_HEADROOM


def test_reasoning_turn_follow_up_after_tool_result_is_fast() -> None:
    """Думает только вызов, читающий реплику; «готово, играю» после тула — нет."""
    completions = _Completions(events=_stream_events_ok())
    p = _provider(completions)

    _run(lambda: _drain(p, _after_tool_result()), reasoning=True)

    assert completions.calls[0]["extra_body"]["thinking"] == DISABLED


@pytest.mark.parametrize(
    ("reasoning", "label"),
    [(False, 'thinking={"type": "disabled"}'), (True, "thinking=model-default")],
)
def test_request_trace_line_names_thinking_mode(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, reasoning: bool, label: str
) -> None:
    monkeypatch.setenv("ROBOT_LLM_VERBOSE", "1")
    p = _provider(_Completions(events=_stream_events_ok()))

    _run(lambda: _drain(p, _user_turn()), reasoning=reasoning)

    err = capsys.readouterr().err
    assert f"LLM REQUEST START provider=minimax mode=stream {label}" in err, err[:300]


# ── first-chunk-гуард #2718 ─────────────────────────────────────────────


def test_complete_with_reasoning_is_not_cut_by_first_chunk_guard(monkeypatch: pytest.MonkeyPatch) -> None:
    """У ``complete()`` гуард ждёт весь ответ; ход с рассуждением получает свой дедлайн."""
    monkeypatch.setattr(harness_minimax, "REASONING_COMPLETE_TIMEOUT_S", 2.0)
    p = _provider(_Completions(reply=_reply("играю"), complete_delay_s=0.3), first_chunk_s=0.1)

    response = _run(lambda: p.complete(_user_turn()), reasoning=True)

    assert response.content == "играю"


def test_complete_without_reasoning_keeps_the_short_guard(monkeypatch: pytest.MonkeyPatch) -> None:
    """Контроль: обычный ход с тем же медленным ответом рубится как раньше."""
    monkeypatch.setattr(harness_minimax, "REASONING_COMPLETE_TIMEOUT_S", 2.0)
    p = _provider(_Completions(reply=_reply("играю"), complete_delay_s=0.3), first_chunk_s=0.1)

    with pytest.raises(FirstChunkTimeoutError):
        _run(lambda: p.complete(_user_turn()), reasoning=False)


def test_stream_reasoning_field_counts_as_first_chunk() -> None:
    """Рассуждение отдельным полем приходит сразу, ответ — через 0,4 с (гуард 0,15 с)."""
    events = tuple((0.05, _event(reasoning="думаю…")) for _ in range(8)) + (
        (0.0, _event("Играю осень.")),
        (0.0, _event(finish="stop")),
    )
    p = _provider(_Completions(events=events), first_chunk_s=0.15)

    chunks = _run(lambda: _drain(p, _user_turn()), reasoning=True)

    text = "".join(c.content_delta for c in chunks)
    assert text == "Играю осень.", f"рассуждение утекло в текст: {text!r}"


def test_stream_silent_provider_still_falls_back() -> None:
    """Контроль: без единой дельты за дедлайн — прежний FirstChunkTimeoutError."""
    events = ((0.4, _event("поздно")), (0.0, _event(finish="stop")))
    p = _provider(_Completions(events=events), first_chunk_s=0.15)

    with pytest.raises(FirstChunkTimeoutError):
        _run(lambda: _drain(p, _user_turn()), reasoning=True)


def test_stream_inline_think_arrives_as_first_chunk() -> None:
    """MiniMax-M3 пишет ``<think>`` в ``content`` — первый чанк приходит сразу."""
    events = (
        (0.0, _event("<think>")),
        *((0.05, _event("думаю ")) for _ in range(8)),
        (0.0, _event("</think>Играю.")),
        (0.0, _event(finish="stop")),
    )
    p = _provider(_Completions(events=events), first_chunk_s=0.15)

    chunks = _run(lambda: _drain(p, _user_turn()), reasoning=True)

    assert "".join(c.content_delta for c in chunks).endswith("Играю.")


# ── парсинг: tool-calls, разметка, AgentCore ───────────────────────────


def test_think_in_content_does_not_break_streamed_tool_call() -> None:
    events = (
        (0.0, _event("<think>Юзер просит ")),
        (0.0, _event('клуб. Вызову compose_music(style="club").</think>')),
        (0.0, _event(tool=("compose_music", '{"sty'))),
        (0.0, _event(tool=("", 'le": "club"}'))),
        (0.0, _event(finish="tool_calls")),
    )
    p = _provider(_Completions(events=events))

    chunks = _run(lambda: _drain(p, _user_turn(), tools=[COMPOSE_TOOL]), reasoning=True)

    calls = [c.tool_call_delta for c in chunks if c.tool_call_delta is not None]
    assert [(c.name, dict(c.arguments)) for c in calls] == [("compose_music", {"style": "club"})]
    content = "".join(c.content_delta for c in chunks)
    assert without_reasoning(LLMResponse(content=content)).content == ""


def test_markup_inside_reasoning_is_not_recovered_as_a_tool_call() -> None:
    """#2760 исполняет вызов, написанный текстом; рассуждение — не вызов."""
    raw = THINK_WITH_MARKUP + "Готово."
    assert parse_tool_call_markup(raw, tools=[COMPOSE_TOOL]), (
        "контроль: без вырезания рассуждения #2760 восстановил бы вызов из <think>"
    )
    cleaned = without_reasoning(LLMResponse(content=raw)).content
    assert cleaned == "Готово."
    assert parse_tool_call_markup(cleaned, tools=[COMPOSE_TOOL]) == ()


class _ThinkingProvider:
    """LLM-двойник: отвечает рассуждением + репликой в обоих путях."""

    name = "fake-thinking"

    def __init__(self, content: str) -> None:
        self._content = content

    async def complete(self, messages: Any, *, tools: Any = (), **_: Any) -> LLMResponse:
        return LLMResponse(content=self._content, finish_reason="stop")

    async def stream(self, messages: Any, *, tools: Any = (), **_: Any) -> AsyncIterator[LLMChunk]:
        for piece in (self._content[:20], self._content[20:]):
            yield LLMChunk(content_delta=piece)
        yield LLMChunk(content_delta="", finish_reason="stop")


def _core(provider: Any, *, streaming: bool) -> AgentCore:
    dsm = DialogueStateMachine()
    dsm.on_event(DialogueEvent.WAKE_WORD)
    dsm.on_event(DialogueEvent.STT_RESULT)
    return AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=dsm,
        system_prompt="ты робот",
        use_streaming=streaming,
    )


@pytest.mark.parametrize("streaming", [False, True])
def test_agent_core_reply_has_no_reasoning(streaming: bool) -> None:
    core = _core(_ThinkingProvider("<think>The user wants music. Plan…</think>Держи осень!"), streaming=streaming)

    result = asyncio.run(core.process_input("сыграй осень", preclassified_event=DialogueEvent.STT_RESULT))

    assert result.spoken_text == "Держи осень!"


# ── вырезание рассуждения ───────────────────────────────────────────────


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("<think>план</think>Ответ.", "Ответ."),
        ("<think>a</think>Раз. <think>b</think>Два.", "Раз. Два."),
        # срезано max_tokens посреди рассуждения
        ("Ответ. <think>The user wants", "Ответ."),
        ("<think>The user wants", ""),
        # открывающий тег потерян
        ("план без тега</think>Ответ.", "Ответ."),
    ],
)
def test_strip_reasoning_shapes(raw: str, expected: str) -> None:
    assert strip_reasoning(raw) == expected


def test_text_without_tags_is_untouched_byte_for_byte() -> None:
    raw = "  Привет!\n"
    assert strip_reasoning(raw) is raw
    response = LLMResponse(content=raw)
    assert without_reasoning(response) is response
