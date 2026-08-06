"""Tests for `DeepSeekProvider` / `MiMoProvider` — fully offline via mock client.

We never hit the network: we inject a fake `AsyncOpenAI` whose `.chat.completions.create`
returns canned objects. This is enough to exercise:
  - request shape (model, messages, stream flag)
  - response parsing (text vs. tool calls)
  - exception mapping (auth/timeout/rate-limit/content-filter)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, AsyncIterator
from unittest.mock import MagicMock

import pytest
from openai import (
    APIConnectionError,
    APITimeoutError,
    AuthenticationError,
)

from rob_box_llm.errors import (
    AuthError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.providers.deepseek import DeepSeekProvider, _OpenAICompatibleProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse

# ---------------------------------------------------------------------------
# Fake SDK objects
# ---------------------------------------------------------------------------


@dataclass
class _ToolCallObj:
    id: str
    function: Any  # .name, .arguments


@dataclass
class _FunctionObj:
    name: str
    arguments: str  # raw JSON


@dataclass
class _ToolCallDelta:
    index: int
    id: str | None = None
    name: str | None = None
    arguments: str | None = None


@dataclass
class _ToolCallDeltaWrapper:
    """OpenAI stream delta shape: tool_calls[i].function..."""

    index: int
    id: str | None = None
    function: _FunctionObj | None = None


@dataclass
class _MessageObj:
    content: str | None = None
    tool_calls: list[_ToolCallObj] | None = None


@dataclass
class _ChoiceObj:
    message: _MessageObj | None = None
    delta: _MessageObj | None = None
    finish_reason: str | None = None


@dataclass
class _UsageObj:
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0


@dataclass
class _ResponseObj:
    choices: list[_ChoiceObj]
    usage: _UsageObj | None = None


def _ok_response(content: str = "hi", tool_calls=None, finish_reason: str = "stop") -> _ResponseObj:
    return _ResponseObj(
        choices=[
            _ChoiceObj(
                message=_MessageObj(content=content, tool_calls=tool_calls),
                finish_reason=finish_reason,
            )
        ],
        usage=_UsageObj(prompt_tokens=1, completion_tokens=2, total_tokens=3),
    )


def _stream_chunk(
    content: str = "",
    finish_reason: str | None = None,
    tool_call: _ToolCallDelta | None = None,
) -> _ResponseObj:
    delta_kwargs: dict = {"content": content}
    if tool_call is not None:
        delta_kwargs["tool_calls"] = [
            _ToolCallDeltaWrapper(
                index=tool_call.index,
                id=tool_call.id,
                function=_FunctionObj(
                    name=tool_call.name or "",
                    arguments=tool_call.arguments or "",
                )
                if (tool_call.name or tool_call.arguments)
                else None,
            )
        ]
    return _ResponseObj(
        choices=[_ChoiceObj(delta=_MessageObj(**delta_kwargs), finish_reason=finish_reason)]
    )


class _FakeCompletions:
    """Stand-in for `AsyncOpenAI().chat.completions`.

    `next_response` returns a single canned response; `next_stream` yields the
    pre-loaded chunks in order.
    """

    def __init__(self) -> None:
        self.next_response: _ResponseObj | None = None
        self.next_stream: list[_ResponseObj] = []
        self.next_exception: BaseException | None = None
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs) -> Any:
        self.calls.append(kwargs)
        if self.next_exception is not None:
            exc, self.next_exception = self.next_exception, None
            raise exc
        if kwargs.get("stream"):
            return self._stream_iter()
        resp, self.next_response = self.next_response, None
        return resp

    async def _stream_iter(self) -> AsyncIterator[_ResponseObj]:
        for c in self.next_stream:
            yield c


class _FakeOpenAIClient:
    def __init__(self) -> None:
        self.chat = MagicMock()
        self.chat.completions = _FakeCompletions()
        self._is_closed = False
        self.close_calls = 0

    @property
    def is_closed(self) -> bool:
        # Mirrors ``openai.AsyncOpenAI.is_closed`` — the real provider relies
        # on this attribute to make ``aclose`` idempotent.
        return self._is_closed

    async def close(self) -> None:
        self.close_calls += 1
        self._is_closed = True


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_deepseek() -> tuple[DeepSeekProvider, _FakeOpenAIClient]:
    client = _FakeOpenAIClient()
    return (
        DeepSeekProvider(base_url="https://example.invalid", api_key="x", client=client),
        client,
    )


def _make_mimo() -> tuple[MiMoProvider, _FakeOpenAIClient]:
    client = _FakeOpenAIClient()
    return (
        MiMoProvider(base_url="https://example.invalid", api_key="x", client=client),
        client,
    )


# ---------------------------------------------------------------------------
# complete()
# ---------------------------------------------------------------------------


def test_complete_returns_text_content():
    p, c = _make_deepseek()
    c.chat.completions.next_response = _ok_response("hello world")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="hi")]))
    assert resp.content == "hello world"
    assert resp.finish_reason == "stop"
    assert resp.usage == {
        "prompt_tokens": 1,
        "completion_tokens": 2,
        "total_tokens": 3,
    }


def test_complete_parses_tool_calls():
    p, c = _make_deepseek()
    tc = _ToolCallObj(
        id="call_1",
        function=_FunctionObj(name="play_sound", arguments='{"name":"beep"}'),
    )
    c.chat.completions.next_response = _ok_response(tool_calls=[tc], finish_reason="tool_calls")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="play sound")]))
    assert len(resp.tool_calls) == 1
    assert resp.tool_calls[0].name == "play_sound"
    assert resp.tool_calls[0].arguments == {"name": "beep"}
    assert resp.finish_reason == "tool_calls"


def test_complete_passes_model_and_messages_to_sdk():
    p, c = _make_deepseek()
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(
        p.complete(
            [
                LLMMessage(role="system", content="be terse"),
                LLMMessage(role="user", content="hi"),
            ]
        )
    )
    kwargs = c.chat.completions.calls[0]
    assert kwargs["model"] == "deepseek-chat"
    assert kwargs["stream"] is False
    assert kwargs["messages"][0]["role"] == "system"
    assert kwargs["messages"][1]["role"] == "user"


def test_complete_settings_override_model_and_temperature():
    from rob_box_llm.provider import LLMChunk, LLMSettings

    p, c = _make_deepseek()
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(
        p.complete(
            [LLMMessage(role="user", content="hi")],
            settings=LLMSettings(model="deepseek-reasoner", temperature=0.0),
        )
    )
    kwargs = c.chat.completions.calls[0]
    assert kwargs["model"] == "deepseek-reasoner"
    assert kwargs["temperature"] == 0.0


def test_complete_accepts_one_shot_generator_messages() -> None:
    """BLK-4 regression: ``messages`` is iterated twice.
    (``_require_capability_for_messages`` then ``_build_kwargs``). A
    one-shot generator would be empty on the second pass, producing
    ``messages=[]`` in the SDK call → 400 / empty reply. The fix freezes
    iterables to a tuple at the top of ``complete()``.
    """
    p, c = _make_deepseek()
    c.chat.completions.next_response = _ok_response("ok")

    def gen():
        yield LLMMessage(role="system", content="sys")
        yield LLMMessage(role="user", content="hi")

    resp = asyncio.run(p.complete(gen()))
    assert resp.content == "ok"
    kwargs = c.chat.completions.calls[0]
    assert [m["role"] for m in kwargs["messages"]] == ["system", "user"]


# ---------------------------------------------------------------------------
# stream()
# ---------------------------------------------------------------------------


def test_stream_yields_chunks_and_stops_on_finish_reason():
    p, c = _make_deepseek()
    c.chat.completions.next_stream = [
        _stream_chunk("He"),
        _stream_chunk("llo"),
        _stream_chunk("", finish_reason="stop"),
    ]
    chunks = []

    async def drain():
        async for ch in p.stream([LLMMessage(role="user", content="hi")]):
            chunks.append(ch)

    asyncio.run(drain())
    assert [ch.content_delta for ch in chunks] == ["He", "llo", ""]
    assert chunks[-1].finish_reason == "stop"


def test_stream_propagates_tools_to_sdk():
    """Streaming + tools now WORKS (streaming_tools=True, live 06.08).

    The OpenAI-compatible adapter aggregates streaming tool-call deltas
    (id/name/arguments fragments) into a ToolCall. The SDK must be called
    with stream=True and tools passed through.
    """
    p, c = _make_deepseek()

    # Simulate streaming tool-call deltas (fragmented arguments!).
    c.chat.completions.next_stream = [
        _stream_chunk(tool_call=_ToolCallDelta(
            index=0,
            id="call_abc",
            name="play_sound",
            arguments='{"sound',
        )),
        _stream_chunk(tool_call=_ToolCallDelta(
            index=0,
            arguments='_name": "cute"}',
        )),
        _stream_chunk(tool_call=_ToolCallDelta(
            index=1,
            name="speak_text",
            arguments="{}",
        )),
        _stream_chunk(finish_reason="tool_calls"),
    ]

    collected: list[LLMChunk] = []

    async def drain():
        async for chunk in p.stream(
            [LLMMessage(role="user", content="hi")],
            tools=({"type": "function", "function": {"name": "play_sound"}},),
        ):
            collected.append(chunk)

    asyncio.run(drain())
    kwargs = c.chat.completions.calls[0]
    assert kwargs.get("stream") is True
    assert "tools" in kwargs
    # Two fully-assembled tool calls.
    calls = [ch.tool_call_delta for ch in collected if ch.tool_call_delta is not None]
    assert len(calls) == 2
    assert calls[0].name == "play_sound"
    assert calls[0].arguments == {"sound_name": "cute"}
    assert calls[1].name == "speak_text"
    assert calls[0].id == "call_abc"
    assert collected[-1].finish_reason == "tool_calls"


def test_stream_without_tools_works():
    p, c = _make_deepseek()
    c.chat.completions.next_stream = [_stream_chunk("ok", finish_reason="stop")]

    async def drain():
        async for _ in p.stream([LLMMessage(role="user", content="hi")]):
            pass

    asyncio.run(drain())
    kwargs = c.chat.completions.calls[0]
    assert kwargs.get("stream") is True
    assert "tools" not in kwargs


def test_complete_swallows_bad_tool_json():
    p, c = _make_deepseek()
    tc = _ToolCallObj(
        id="call_1",
        function=_FunctionObj(name="play_sound", arguments="not-json"),
    )
    c.chat.completions.next_response = _ok_response(tool_calls=[tc], finish_reason="tool_calls")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="x")]))
    assert resp.tool_calls[0].arguments == {}


def test_complete_round_trips_assistant_tool_calls_with_frozen_arguments():
    """Issue #917 regression: ``ToolCall.arguments`` is a ``MappingProxyType``
    after ``__post_init__`` (immutability invariant), but
    ``_json_dumps`` must still serialise it when the caller feeds the
    previous assistant turn back into the model — exactly what
    ``dialog_core._run_with_tools`` does on every tool-loop iteration.
    Before the fix this raised
    ``TypeError: Object of type MappingProxyType is not JSON serializable``
    and broke the entire tool loop.
    """
    from rob_box_llm.provider import LLMChunk, ToolCall
    from rob_box_llm.providers.deepseek import _json_dumps

    # Sanity: ``ToolCall.__post_init__`` does freeze the dict view.
    tc = ToolCall(id="call_1", name="play_sound", arguments={"name": "beep"})
    from types import MappingProxyType
    assert isinstance(tc.arguments, MappingProxyType)

    # 1. Unit-level: the helper itself unwraps proxy → dict.
    assert _json_dumps(tc.arguments) == '{"name": "beep"}'

    # 2. Integration-level: feeding an assistant tool-turn back into
    # ``complete()`` must not raise and must hand the SDK a JSON string
    # for ``function.arguments`` (OpenAI wire format).
    p, c = _make_deepseek()
    c.chat.completions.next_response = _ok_response("done", finish_reason="stop")

    messages = [
        LLMMessage(role="user", content="play sound"),
        LLMMessage(
            role="assistant",
            content="",
            tool_calls=(tc,),
        ),
    ]
    resp = asyncio.run(p.complete(messages))
    assert resp.content == "done"

    sent = c.chat.completions.calls[0]["messages"]
    sent_tc = sent[1]["tool_calls"][0]
    assert sent_tc["function"]["arguments"] == '{"name": "beep"}'


def test_json_dumps_unwraps_top_level_mapping_proxy():
    """Issue #917 regression: even when a caller hands a ``MappingProxyType``
    directly (not wrapped in ``ToolCall``), the helper must serialise it.
    """
    from rob_box_llm.providers.deepseek import _json_dumps
    from types import MappingProxyType

    proxy = MappingProxyType({"foo": "bar", "n": 1})
    assert _json_dumps(proxy) == '{"foo": "bar", "n": 1}'


# ---------------------------------------------------------------------------
# Error mapping
# ---------------------------------------------------------------------------


def test_authentication_error_maps_to_AuthError():
    p, c = _make_deepseek()
    c.chat.completions.next_exception = AuthenticationError(
        message="bad key",
        response=_fake_401_response(),
        body={"error": {"message": "bad key"}},
    )
    with pytest.raises(AuthError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_timeout_error_maps_to_TimeoutError():
    p, c = _make_deepseek()
    c.chat.completions.next_exception = APITimeoutError(request=MagicMock())
    with pytest.raises(TimeoutError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_connection_error_maps_to_TimeoutError():
    """ConnectionError is also 'try again' — we treat it as transient."""
    p, c = _make_deepseek()
    c.chat.completions.next_exception = APIConnectionError(request=MagicMock())
    with pytest.raises(TimeoutError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_status_error_429_maps_to_RateLimitError():
    p, c = _make_deepseek()
    err = _FakeStatusError(status=429, body={"error": {"message": "rate-limited"}})
    c.chat.completions.next_exception = err
    with pytest.raises(RateLimitError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_status_error_content_filter_maps_to_ContentFilterError():
    p, c = _make_deepseek()
    err = _FakeStatusError(
        status=400,
        body={"error": {"message": "content filtered by policy"}},
    )
    c.chat.completions.next_exception = err
    with pytest.raises(ContentFilterError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_unknown_error_maps_to_ProviderError():
    p, c = _make_deepseek()
    c.chat.completions.next_exception = RuntimeError("nope")
    with pytest.raises(ProviderError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_stream_raises_before_yielding_when_initial_request_fails():
    p, c = _make_deepseek()
    c.chat.completions.next_exception = APITimeoutError(request=MagicMock())

    async def drain():
        out = []
        async for ch in p.stream([LLMMessage(role="user", content="x")]):
            out.append(ch)
        return out

    with pytest.raises(TimeoutError):
        asyncio.run(drain())


# ---------------------------------------------------------------------------
# Provider identity
# ---------------------------------------------------------------------------


def test_deepseek_default_base_url_and_model():
    assert DeepSeekProvider.DEFAULT_BASE_URL == "https://api.deepseek.com"
    assert DeepSeekProvider.DEFAULT_MODEL == "deepseek-chat"


def test_mimo_default_base_url_and_model():
    assert MiMoProvider.DEFAULT_BASE_URL == "https://api.xiaomimimo.com/v1"
    assert MiMoProvider.DEFAULT_MODEL == "mimo-v2.5-pro"


def test_provider_name_is_set():
    p, _ = _make_deepseek()
    assert p.name == "deepseek"
    m, _ = _make_mimo()
    assert m.name == "mimo"


# ---------------------------------------------------------------------------
# aclose()
# ---------------------------------------------------------------------------


def test_aclose_closes_client():
    p, c = _make_deepseek()
    asyncio.run(p.aclose())
    assert c.is_closed is True
    assert c.close_calls == 1


def test_aclose_is_idempotent():
    """Calling aclose() twice (e.g. from nested finally blocks) must not.
    raise and must not invoke the underlying client close() again."""
    p, c = _make_deepseek()
    asyncio.run(p.aclose())
    # Second call must be a no-op — no RuntimeError, no extra close().
    asyncio.run(p.aclose())
    assert c.is_closed is True
    assert c.close_calls == 1


def test_aclose_is_noop_when_client_already_closed():
    """If the underlying client was closed out from under us (e.g. caller.
    owns the client), aclose() must stay silent rather than masking the
    original teardown with a RuntimeError."""
    p, c = _make_deepseek()

    async def _simulate():
        # Simulate someone else having already closed the client.
        await c.close()
        await p.aclose()

    asyncio.run(_simulate())
    assert c.is_closed is True
    assert c.close_calls == 1


# ---------------------------------------------------------------------------
# Helper: fake APIStatusError without hitting the SDK
# ---------------------------------------------------------------------------


from openai import APIStatusError  # noqa: E402


def _fake_401_response():
    """A real httpx.Response so openai 2.x's AuthenticationError accepts it."""
    import httpx

    return httpx.Response(401, request=httpx.Request("POST", "http://x.invalid"))


class _FakeStatusError(APIStatusError):
    def __init__(self, *, status: int, body: Any) -> None:
        # Skip APIStatusError.__init__ (it tries to parse the response);
        # we set the attributes our error mapper reads directly.
        self.status_code = status
        self.body = body
        self.request = MagicMock()
        Exception.__init__(self, f"{status}: {body}")
