"""MiniMax-M3.1-Flash в харнессе — то, что видно только на уровне провайдера харнесса.

Решение Шифу 20.10: по умолчанию остаётся MiniMax-M3, адаптер под Flash сохранён
на будущее. Поэтому здесь модель называется явно (``model=FLASH``), а то, что
по умолчанию всё осталось как было, держит ``test_minimax_m3_wire_unchanged.py``.

Форма ответов взята с живой пробы 20.10 (Sound Pi, ``scripts/research/
minimax_model_compat_probe.py``): рассуждение приходит ОТДЕЛЬНЫМ полем
``reasoning_content`` (в стриме — в каждой дельте до ответа), а не
``<think>`` в ``content``; голосовой ход идёт с ``reasoning_effort=low``,
потому что ``thinking=disabled`` Flash отвергает (HTTP 400).

Подменён только клиент SDK; провайдер — настоящий ``HarnessMiniMaxProvider``.
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any, AsyncIterator

import pytest
from openai import APIStatusError

from rob_box_harness.providers.catalog import LLM_PROVIDER_REGISTRY, build_provider
from rob_box_harness.providers.minimax import (
    DEFAULT_MODEL,
    FirstChunkTimeoutError,
    HarnessMiniMaxProvider,
    RetryPolicy,
)
from rob_box_harness.providers.reasoning import (
    REASONING_MAX_TOKENS_HEADROOM,
    TURN_REASONING,
    without_reasoning,
)
from rob_box_llm.provider import LLMMessage, LLMResponse

FLASH = "MiniMax-M3.1-Flash"
SET_VOICE_TOOL = {
    "type": "function",
    "function": {
        "name": "set_voice",
        "parameters": {"type": "object", "properties": {"voice": {"type": "string"}}},
    },
}


class _Rejected(APIStatusError):
    def __init__(self) -> None:
        self.status_code = 400
        self.body = {"type": "bad_request_error", "message": "requires adaptive thinking (2013)"}
        self.request = None
        Exception.__init__(self, "400")


def _reject_disabled(kwargs: dict[str, Any]) -> None:
    thinking = (kwargs.get("extra_body") or {}).get("thinking") or {}
    if thinking.get("type") == "disabled":
        raise _Rejected()


def _message(content: str, *, reasoning: str | None = None, tool: tuple[str, str] | None = None) -> Any:
    calls = None
    if tool is not None:
        function = SimpleNamespace(name=tool[0], arguments=tool[1])
        calls = [SimpleNamespace(id="call_1", function=function)]
    return SimpleNamespace(content=content, tool_calls=calls, reasoning_content=reasoning)


def _reply(message: Any, finish: str = "stop") -> Any:
    return SimpleNamespace(
        choices=[SimpleNamespace(message=message, finish_reason=finish)], usage=None, base_resp={}
    )


def _event(content: str | None = None, *, reasoning: str | None = None, tool: tuple[str, str] | None = None,
           finish: str | None = None) -> Any:
    calls = None
    if tool is not None:
        function = SimpleNamespace(name=tool[0], arguments=tool[1])
        calls = [SimpleNamespace(index=0, id="call_1", function=function)]
    delta = SimpleNamespace(content=content, tool_calls=calls, reasoning_content=reasoning)
    return SimpleNamespace(choices=[SimpleNamespace(delta=delta, finish_reason=finish)], usage=None, base_resp={})


class _Completions:
    def __init__(self, *, reply: Any = None, events: tuple[tuple[float, Any], ...] = ()) -> None:
        self.reply = reply if reply is not None else _reply(_message("ок"))
        self.events = events
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        _reject_disabled(kwargs)
        if kwargs.get("stream"):
            return self._stream()
        return self.reply

    async def _stream(self) -> AsyncIterator[Any]:
        for delay_s, event in self.events:
            await asyncio.sleep(delay_s)
            yield event


def _provider(completions: _Completions, *, first_chunk_s: float | None = 10.0) -> HarnessMiniMaxProvider:
    client = SimpleNamespace(chat=SimpleNamespace(completions=completions), is_closed=False)
    return HarnessMiniMaxProvider(
        api_key="sk-test",
        model=FLASH,
        client=client,  # type: ignore[arg-type]
        first_chunk_timeout_s=first_chunk_s,
        retry=RetryPolicy(max_attempts=1),
    )


def _turn() -> list[LLMMessage]:
    return [LLMMessage(role="system", content="ты робот"), LLMMessage(role="user", content="говори голосом Алёна")]


async def _stream(provider: HarnessMiniMaxProvider, **kwargs: Any) -> list[Any]:
    return [chunk async for chunk in provider.stream(_turn(), **kwargs)]


def _in_turn(coro_fn: Any, *, reasoning: bool) -> Any:
    async def run() -> Any:
        token = TURN_REASONING.set(reasoning)
        try:
            return await coro_fn()
        finally:
            TURN_REASONING.reset(token)

    return asyncio.run(run())


# ── модель называется в одном месте ─────────────────────────────────────


def test_catalog_and_provider_share_the_one_default_model() -> None:
    assert DEFAULT_MODEL == "MiniMax-M3"
    assert LLM_PROVIDER_REGISTRY["minimax"]["default_model"] == DEFAULT_MODEL


def test_catalog_builds_the_personality_provider_on_the_default_model() -> None:
    provider = build_provider("minimax", api_key="sk-test")

    assert provider._inner._default_model == DEFAULT_MODEL
    assert provider._first_chunk_timeout_s == 10.0


def test_catalog_switches_to_flash_by_the_model_argument_alone() -> None:
    """Переключение — одна строка (``DEFAULT_MODEL``) или ``model=`` в каталоге; адаптер включается сам."""
    provider = build_provider("minimax", api_key="sk-test", model=FLASH)

    assert provider._inner._default_model == FLASH
    assert provider.capabilities_for(FLASH).image_input is True


# ── запрос: голосовой ход и ход с рассуждением ──────────────────────────


@pytest.mark.parametrize("stream", [False, True])
def test_ordinary_turn_is_accepted_by_an_api_that_rejects_disabled(stream: bool) -> None:
    completions = _Completions(events=((0.0, _event("ок")), (0.0, _event(finish="stop"))))
    provider = _provider(completions)

    async def call() -> Any:
        return await _stream(provider) if stream else await provider.complete(_turn())

    _in_turn(call, reasoning=False)

    sent = completions.calls[0]
    assert sent["extra_body"] == {"reasoning_effort": "low"}
    assert sent["max_tokens"] == 4096


@pytest.mark.parametrize("stream", [False, True])
def test_reasoning_turn_keeps_the_model_default_depth_and_token_headroom(stream: bool) -> None:
    completions = _Completions(events=((0.0, _event("ок")), (0.0, _event(finish="stop"))))
    provider = _provider(completions)

    async def call() -> Any:
        return await _stream(provider) if stream else await provider.complete(_turn())

    _in_turn(call, reasoning=True)

    sent = completions.calls[0]
    assert "extra_body" not in sent, sent.get("extra_body")
    assert sent["max_tokens"] == 4096 + REASONING_MAX_TOKENS_HEADROOM


# ── ответ: reasoning_content не речь ────────────────────────────────────


def test_complete_ignores_reasoning_content_and_keeps_the_tool_call() -> None:
    message = _message("", reasoning="Нужно сменить голос на Алёну.", tool=("set_voice", '{"voice": "alena"}'))
    provider = _provider(_Completions(reply=_reply(message, finish="tool_calls")))

    response = _in_turn(lambda: provider.complete(_turn(), tools=[SET_VOICE_TOOL]), reasoning=False)

    assert response.content == ""
    assert [(c.name, dict(c.arguments)) for c in response.tool_calls] == [("set_voice", {"voice": "alena"})]
    assert response.finish_reason == "tool_calls"


def test_complete_answer_text_does_not_carry_the_reasoning() -> None:
    message = _message("Привет, Шифу!", reasoning="Поздороваюсь коротко.")
    provider = _provider(_Completions(reply=_reply(message)))

    response = _in_turn(lambda: provider.complete(_turn()), reasoning=False)

    assert without_reasoning(response).content == "Привет, Шифу!"
    assert "Поздороваюсь" not in response.content


def test_stream_with_reasoning_deltas_first_survives_the_first_chunk_guard() -> None:
    """Flash думает 0,4 с отдельным полем (гуард 0,15 с), потом отдаёт вызов тула."""
    events = (
        *((0.05, _event(reasoning="думаю…")) for _ in range(8)),
        (0.0, _event(tool=("set_voice", '{"voi'))),
        (0.0, _event(tool=("", 'ce": "alena"}'))),
        (0.0, _event(finish="tool_calls")),
    )
    provider = _provider(_Completions(events=events), first_chunk_s=0.15)

    chunks = _in_turn(lambda: _stream(provider, tools=[SET_VOICE_TOOL]), reasoning=False)

    assert "".join(c.content_delta for c in chunks) == "", "рассуждение утекло в текст"
    calls = [c.tool_call_delta for c in chunks if c.tool_call_delta is not None]
    assert [(c.name, dict(c.arguments)) for c in calls] == [("set_voice", {"voice": "alena"})]
    assert chunks[-1].finish_reason == "tool_calls"


def test_stream_that_is_silent_even_in_reasoning_still_falls_back() -> None:
    """Контроль гуарда: ни рассуждения, ни ответа за дедлайн — фолбек, как раньше."""
    events = ((0.4, _event("поздно")), (0.0, _event(finish="stop")))
    provider = _provider(_Completions(events=events), first_chunk_s=0.15)

    with pytest.raises(FirstChunkTimeoutError):
        _in_turn(lambda: _stream(provider), reasoning=False)


def test_reasoning_only_answer_cut_by_max_tokens_is_an_empty_not_a_leak() -> None:
    """``finish_reason=length`` посреди рассуждения: content пуст — не подсовываем рассуждение как речь."""
    message = _message("", reasoning="Длинное рассуждение без ответа")
    provider = _provider(_Completions(reply=_reply(message, finish="length")))

    response: LLMResponse = _in_turn(lambda: provider.complete(_turn()), reasoning=True)

    assert response.content == ""
    assert response.finish_reason == "length"
