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
from rob_box_llm.provider import LLMMessage, LLMResponse

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


def _stream_chunk(content: str = "", finish_reason: str | None = None) -> _ResponseObj:
    return _ResponseObj(choices=[_ChoiceObj(delta=_MessageObj(content=content), finish_reason=finish_reason)])


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
        self.closed = False

    async def close(self) -> None:
        self.closed = True


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
    from rob_box_llm.provider import LLMSettings

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
    """BLK-4 regression: ``messages`` is iterated twice
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
    """Streaming + tools is gated behind ``streaming_tools`` capability.

    The current OpenAI-compatible adapter does not aggregate streaming tool-call
    deltas, so this combination MUST raise ``CapabilityUnavailableError``
    before touching the network. Callers that need tools + streaming should
    fall back to ``complete()``.
    """
    from rob_box_llm.errors import CapabilityUnavailableError

    p, c = _make_deepseek()

    # No SDK call should happen — we expect an early capability refusal.
    async def drain():
        async for _ in p.stream(
            [LLMMessage(role="user", content="hi")],
            tools=({"type": "function", "function": {"name": "play_sound"}},),
        ):
            pass

    with pytest.raises(CapabilityUnavailableError):
        asyncio.run(drain())
    assert c.chat.completions.calls == []


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
    assert c.closed is True


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
