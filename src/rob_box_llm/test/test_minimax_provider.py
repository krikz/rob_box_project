"""Tests for `MiniMaxProvider` — fully offline via fake SDK client.

We never hit the network: we inject a fake ``AsyncOpenAI`` whose
``.chat.completions.create`` returns canned objects. This is enough to exercise:

- request shape (model, messages, stream flag, image parts, thinking policy)
- response parsing (text vs. tool calls, base_resp envelope)
- exception mapping (auth/timeout/rate-limit/content-filter)
- capability gating (image_input per-model, streaming + tools refusal)
- image payload size limit
- API-key redaction in log records

The fake SDK here mirrors the one in ``test_deepseek_provider.py`` so the
existing ``_FakeCompletions`` / ``_FakeOpenAIClient`` pattern is preserved.
"""

from __future__ import annotations

import asyncio
import base64
import logging
from dataclasses import dataclass, field
from typing import Any, AsyncIterator
from unittest.mock import MagicMock

import httpx
import pytest
from openai import (
    APIConnectionError,
    APIStatusError,
    APITimeoutError,
    AuthenticationError,
)

from rob_box_llm.errors import (
    AuthError,
    CapabilityUnavailableError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMMessage,
    LLMSettings,
    TextPart,
)
from rob_box_llm.providers.minimax import (
    DEFAULT_THINKING_POLICY,
    MINIMAX_MAX_IMAGE_BYTES,
    MiniMaxProvider,
    MiniMaxRedactedLogFilter,
)

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
    """Stand-in for the OpenAI SDK chat completion response."""

    choices: list[_ChoiceObj]
    usage: _UsageObj | None = None
    base_resp: dict[str, Any] = field(default_factory=dict)


def _ok_response(
    content: str = "hi",
    tool_calls=None,
    finish_reason: str = "stop",
    *,
    base_resp: dict[str, Any] | None = None,
) -> _ResponseObj:
    return _ResponseObj(
        choices=[
            _ChoiceObj(
                message=_MessageObj(content=content, tool_calls=tool_calls),
                finish_reason=finish_reason,
            )
        ],
        usage=_UsageObj(prompt_tokens=1, completion_tokens=2, total_tokens=3),
        base_resp=base_resp or {},
    )


@dataclass
class _ToolCallDelta:
    index: int
    id: str | None = None
    name: str | None = None
    arguments: str | None = None


@dataclass
class _ToolCallDeltaWrapper:
    index: int
    id: str | None = None
    function: Any | None = None


def _stream_chunk(
    content: str = "",
    finish_reason: str | None = None,
    tool_call: _ToolCallDelta | None = None,
    *,
    base_resp: dict[str, Any] | None = None,
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
        choices=[_ChoiceObj(delta=_MessageObj(**delta_kwargs), finish_reason=finish_reason)],
        base_resp=base_resp or {},
    )


class _FakeCompletions:
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
        # Mirrors ``openai.AsyncOpenAI.is_closed`` — the real provider
        # (inherited from ``_OpenAICompatibleProvider``) relies on this
        # attribute to make ``aclose`` idempotent.
        return self._is_closed

    async def close(self) -> None:
        self.close_calls += 1
        self._is_closed = True


def _fake_401_response() -> httpx.Response:
    return httpx.Response(401, request=httpx.Request("POST", "http://x.invalid"))


class _FakeStatusError(APIStatusError):
    def __init__(self, *, status: int, body: Any) -> None:
        self.status_code = status
        self.body = body
        self.request = MagicMock()
        Exception.__init__(self, f"{status}: {body}")


# Sentinel for "use provider default" in tests. Defined up here so the
# ``_make_minimax`` signature below can reference it as a default value.
_USE_DEFAULT = object()


def _make_minimax(
    *,
    thinking: dict[str, str] | None | object = _USE_DEFAULT,
    model: str = MiniMaxProvider.DEFAULT_MODEL,
) -> tuple[MiniMaxProvider, _FakeOpenAIClient]:
    """Build a MiniMaxProvider backed by a fake SDK.

    ``thinking`` semantics mirror the provider constructor: omit (sentinel)
    to use ``MiniMaxProvider``'s default policy; pass a mapping to override;
    pass ``None`` explicitly to disable.
    """
    client = _FakeOpenAIClient()
    kwargs: dict[str, Any] = dict(
        base_url="https://example.invalid",
        api_key="sk-test",
        model=model,
        client=client,
    )
    if thinking is not _USE_DEFAULT:
        kwargs["thinking"] = thinking
    return MiniMaxProvider(**kwargs), client


# ---------------------------------------------------------------------------
# Provider identity & defaults
# ---------------------------------------------------------------------------


def test_default_base_url_and_model():
    assert MiniMaxProvider.DEFAULT_BASE_URL == "https://api.minimax.io/v1"
    assert MiniMaxProvider.DEFAULT_MODEL == "MiniMax-M3"


def test_provider_name_is_minimax():
    p, _ = _make_minimax()
    assert p.name == "minimax"


def test_capabilities_advertise_text_streaming_tools_and_image_input():
    caps = MiniMaxProvider._CAPABILITIES
    assert caps.text is True
    assert caps.streaming_text is True
    assert caps.tools is True
    assert caps.image_input is True
    # 🔴 FIX (live 06.08): streaming_tools=True — OpenAI-совместимый API
    # стримит tool-call deltas, stream() агрегирует их в ToolCall.
    assert caps.streaming_tools is True


def test_capabilities_for_vision_model_keeps_image_input():
    p, _ = _make_minimax()
    caps = p.capabilities_for("MiniMax-M3")
    assert caps.image_input is True


def test_capabilities_for_non_vision_model_drops_image_input():
    p, _ = _make_minimax()
    caps = p.capabilities_for("MiniMax-M2.7")
    assert caps.image_input is False


def test_capabilities_for_unknown_model_drops_image_input():
    p, _ = _make_minimax()
    caps = p.capabilities_for("some-other-model")
    assert caps.image_input is False


# ---------------------------------------------------------------------------
# complete() — text + tools
# ---------------------------------------------------------------------------


def test_complete_returns_text_content():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("hello world")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="hi")]))
    assert resp.content == "hello world"
    assert resp.finish_reason == "stop"
    assert resp.usage["total_tokens"] == 3


def test_complete_parses_tool_calls():
    p, c = _make_minimax()
    tc = _ToolCallObj(
        id="call_1",
        function=_FunctionObj(name="play_sound", arguments='{"name":"beep"}'),
    )
    c.chat.completions.next_response = _ok_response(tool_calls=[tc], finish_reason="tool_calls")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="beep")]))
    assert len(resp.tool_calls) == 1
    assert resp.tool_calls[0].name == "play_sound"
    assert resp.tool_calls[0].arguments == {"name": "beep"}


def test_complete_passes_model_and_messages():
    p, c = _make_minimax()
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
    assert kwargs["model"] == "MiniMax-M3"
    assert kwargs["stream"] is False
    assert kwargs["messages"][0]["role"] == "system"
    assert kwargs["messages"][1]["role"] == "user"
    # Default thinking policy is injected via extra_body (live 06.08:
    # кастомные поля через extra_body — create(thinking=...) = TypeError).
    assert kwargs["extra_body"]["thinking"] == DEFAULT_THINKING_POLICY


def test_complete_settings_override_model_and_temperature():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(
        p.complete(
            [LLMMessage(role="user", content="hi")],
            settings=LLMSettings(model="MiniMax-M2.7", temperature=0.0),
        )
    )
    kwargs = c.chat.completions.calls[0]
    assert kwargs["model"] == "MiniMax-M2.7"
    assert kwargs["temperature"] == 0.0


def test_complete_swallows_bad_tool_json():
    p, c = _make_minimax()
    tc = _ToolCallObj(
        id="call_1",
        function=_FunctionObj(name="play_sound", arguments="not-json"),
    )
    c.chat.completions.next_response = _ok_response(tool_calls=[tc], finish_reason="tool_calls")
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="x")]))
    assert resp.tool_calls[0].arguments == {}


# ---------------------------------------------------------------------------
# Thinking policy
# ---------------------------------------------------------------------------


def test_thinking_policy_applied_by_default():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(p.complete([LLMMessage(role="user", content="hi")]))
    assert c.chat.completions.calls[0]["extra_body"]["thinking"] == {"type": "disabled"}


def test_thinking_policy_can_be_disabled_per_instance():
    p, c = _make_minimax(thinking=None)
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(p.complete([LLMMessage(role="user", content="hi")]))
    assert "thinking" not in c.chat.completions.calls[0]


def test_thinking_policy_callers_can_override():
    p, c = _make_minimax(thinking={"type": "enabled"})
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(
        p.complete(
            [LLMMessage(role="user", content="hi")],
            settings=LLMSettings(extra={"thinking": {"type": "enabled", "budget": 200}}),
        )
    )
    # Caller-supplied extra wins over instance default.
    assert c.chat.completions.calls[0]["extra_body"]["thinking"] == {
        "type": "enabled",
        "budget": 200,
    }


# ---------------------------------------------------------------------------
# stream() — text only (streaming + tools is capability-gated)
# ---------------------------------------------------------------------------


def test_stream_yields_chunks_and_stops_on_finish_reason():
    p, c = _make_minimax()
    c.chat.completions.next_stream = [
        _stream_chunk("He"),
        _stream_chunk("llo"),
        _stream_chunk("", finish_reason="stop"),
    ]

    async def drain() -> list[Any]:
        out = []
        async for ch in p.stream([LLMMessage(role="user", content="hi")]):
            out.append(ch)
        return out

    chunks = asyncio.run(drain())
    assert [ch.content_delta for ch in chunks] == ["He", "llo", ""]
    assert chunks[-1].finish_reason == "stop"


def test_stream_with_tools_raises_capability_error():
    """Streaming + tools now WORKS (streaming_tools=True, live 06.08).

    The OpenAI-compatible adapter aggregates streaming tool-call deltas.
    No CapabilityUnavailableError should be raised; stream must hit the SDK.
    """
    p, c = _make_minimax()
    c.chat.completions.next_stream = [
        _stream_chunk(tool_call=_ToolCallDelta(index=0, id="call_1", name="play_sound", arguments="{}")),
        _stream_chunk(finish_reason="tool_calls"),
    ]

    async def drain():
        async for _ in p.stream(
            [LLMMessage(role="user", content="hi")],
            tools=({"type": "function", "function": {"name": "play_sound"}},),
        ):
            pass

    asyncio.run(drain())
    assert c.chat.completions.calls
    kwargs = c.chat.completions.calls[0]
    assert kwargs.get("stream") is True
    assert "tools" in kwargs


# ---------------------------------------------------------------------------
# Vision / image input (M4)
# ---------------------------------------------------------------------------


def test_complete_with_image_url_serialises_as_image_url_part():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("a cat")
    asyncio.run(
        p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(
                        TextPart(text="What is in this image?"),
                        ImagePart(source="https://x.invalid/cat.jpg"),
                    ),
                )
            ]
        )
    )
    kwargs = c.chat.completions.calls[0]
    parts = kwargs["messages"][0]["content"]
    assert isinstance(parts, list) and len(parts) == 2
    assert parts[0] == {"type": "text", "text": "What is in this image?"}
    assert parts[1]["type"] == "image_url"
    assert parts[1]["image_url"]["url"] == "https://x.invalid/cat.jpg"
    # detail == "default" is omitted from the wire format.
    assert "detail" not in parts[1]["image_url"]


def test_complete_with_image_bytes_serialises_as_data_url():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok")
    payload = b"\x89PNG\r\n\x1a\n" + b"binary-image-bytes"
    asyncio.run(
        p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(ImagePart(source=payload, media_type="image/png"),),
                )
            ]
        )
    )
    parts = c.chat.completions.calls[0]["messages"][0]["content"]
    url = parts[0]["image_url"]["url"]
    assert url.startswith("data:image/png;base64,")
    decoded = base64.b64decode(url.split(",", 1)[1])
    assert decoded == payload


def test_image_part_with_explicit_detail_is_forwarded():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok")
    asyncio.run(
        p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(ImagePart(source="https://x/y.jpg", detail="high"),),
                )
            ]
        )
    )
    parts = c.chat.completions.calls[0]["messages"][0]["content"]
    assert parts[0]["image_url"]["detail"] == "high"


def test_image_input_to_non_vision_model_raises_capability_error():
    p, c = _make_minimax(model="MiniMax-M2.7")
    with pytest.raises(CapabilityUnavailableError, match="image input not supported"):
        asyncio.run(
            p.complete(
                [
                    LLMMessage(
                        role="user",
                        content=(ImagePart(source="https://x/y.jpg"),),
                    )
                ]
            )
        )
    # Crucially: no SDK call was made.
    assert c.chat.completions.calls == []


def test_image_input_to_text_only_provider_raises_capability_error():
    """DeepSeek doesn't support image input — gating is provider-agnostic."""
    from rob_box_llm.providers.deepseek import DeepSeekProvider

    p = DeepSeekProvider(base_url="https://x", api_key="k")
    client = _FakeOpenAIClient()
    p._client = client  # type: ignore[attr-defined]
    with pytest.raises(CapabilityUnavailableError, match="image input not supported"):
        asyncio.run(
            p.complete(
                [
                    LLMMessage(
                        role="user",
                        content=(ImagePart(source="https://x/y.jpg"),),
                    )
                ]
            )
        )
    assert client.chat.completions.calls == []


def test_image_payload_above_limit_raises_capability_error():
    p, c = _make_minimax()
    huge = b"x" * (MINIMAX_MAX_IMAGE_BYTES + 1)
    with pytest.raises(CapabilityUnavailableError, match="exceeds limit"):
        asyncio.run(
            p.complete(
                [
                    LLMMessage(
                        role="user",
                        content=(ImagePart(source=huge, media_type="image/jpeg"),),
                    )
                ]
            )
        )
    assert c.chat.completions.calls == []


def test_image_payload_url_above_limit_does_not_block_size_check():
    """URL-based payloads don't carry bytes — only byte sources are bounded."""
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok")
    long_url = "https://x.invalid/" + ("a" * (MINIMAX_MAX_IMAGE_BYTES * 2))
    asyncio.run(
        p.complete(
            [
                LLMMessage(
                    role="user",
                    content=(ImagePart(source=long_url),),
                )
            ]
        )
    )
    assert len(c.chat.completions.calls) == 1


# ---------------------------------------------------------------------------
# Error mapping (HTTP layer)
# ---------------------------------------------------------------------------


def test_authentication_error_maps_to_AuthError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = AuthenticationError(
        message="bad key",
        response=_fake_401_response(),
        body={"error": {"message": "bad key"}},
    )
    with pytest.raises(AuthError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_timeout_error_maps_to_TimeoutError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = APITimeoutError(request=MagicMock())
    with pytest.raises(TimeoutError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_connection_error_maps_to_TimeoutError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = APIConnectionError(request=MagicMock())
    with pytest.raises(TimeoutError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_status_error_429_maps_to_RateLimitError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = _FakeStatusError(status=429, body={"error": {"message": "rate-limited"}})
    with pytest.raises(RateLimitError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_status_error_content_filter_maps_to_ContentFilterError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = _FakeStatusError(
        status=400, body={"error": {"message": "content filtered by policy"}}
    )
    with pytest.raises(ContentFilterError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_unknown_error_maps_to_ProviderError():
    p, c = _make_minimax()
    c.chat.completions.next_exception = RuntimeError("nope")
    with pytest.raises(ProviderError):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_stream_raises_before_yielding_when_initial_request_fails():
    p, c = _make_minimax()
    c.chat.completions.next_exception = APITimeoutError(request=MagicMock())

    async def drain() -> list[Any]:
        return [ch async for ch in p.stream([LLMMessage(role="user", content="x")])]

    with pytest.raises(TimeoutError):
        asyncio.run(drain())


# ---------------------------------------------------------------------------
# Error mapping (MiniMax-specific base_resp envelope on HTTP 200)
# ---------------------------------------------------------------------------


def test_base_resp_quota_error_maps_to_RateLimitError():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response(base_resp={"status_code": 1008, "status_msg": "insufficient quota"})
    with pytest.raises(RateLimitError, match="quota"):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_base_resp_auth_error_maps_to_AuthError():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response(base_resp={"status_code": 1002, "status_msg": "invalid api key"})
    with pytest.raises(AuthError, match="key"):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_base_resp_safety_error_maps_to_ContentFilterError():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response(
        base_resp={"status_code": 1001, "status_msg": "content safety violation"}
    )
    with pytest.raises(ContentFilterError, match="safety"):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_base_resp_unknown_status_maps_to_ProviderError():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response(
        base_resp={"status_code": 9999, "status_msg": "weird internal thing"}
    )
    with pytest.raises(ProviderError, match="9999"):
        asyncio.run(p.complete([LLMMessage(role="user", content="x")]))


def test_base_resp_zero_is_treated_as_success():
    p, c = _make_minimax()
    c.chat.completions.next_response = _ok_response("ok", base_resp={"status_code": 0, "status_msg": "ok"})
    resp = asyncio.run(p.complete([LLMMessage(role="user", content="hi")]))
    assert resp.content == "ok"


def test_stream_base_resp_error_raises_before_yielding():
    p, c = _make_minimax()
    # First event carries the bad envelope; stream() must surface it as
    # an exception, not as a silent chunk with finish_reason="error".
    c.chat.completions.next_stream = [
        _stream_chunk(base_resp={"status_code": 1008, "status_msg": "quota exceeded"}),
    ]

    async def drain() -> list[Any]:
        return [ch async for ch in p.stream([LLMMessage(role="user", content="x")])]

    with pytest.raises(RateLimitError):
        asyncio.run(drain())


# ---------------------------------------------------------------------------
# aclose
# ---------------------------------------------------------------------------


def test_aclose_closes_client():
    p, c = _make_minimax()
    asyncio.run(p.aclose())
    assert c.is_closed is True
    assert c.close_calls == 1


def test_aclose_is_idempotent():
    """Inherited from ``_OpenAICompatibleProvider`` — re-closing must be a.
    no-op rather than raising RuntimeError."""
    p, c = _make_minimax()
    asyncio.run(p.aclose())
    asyncio.run(p.aclose())
    assert c.is_closed is True
    assert c.close_calls == 1


# ---------------------------------------------------------------------------
# API-key redaction
# ---------------------------------------------------------------------------


def test_provider_installs_api_key_redaction_on_sdk_and_httpx_loggers(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    api_key = "minimax-auto-filter-secret-do-not-log"
    monkeypatch.setenv("MINIMAX_API_KEY", api_key)
    provider_logger = logging.getLogger("rob_box_llm.providers.minimax")
    httpx_logger = logging.getLogger("httpx")

    previous_filters = {
        provider_logger: list(provider_logger.filters),
        httpx_logger: list(httpx_logger.filters),
    }
    for logger in previous_filters:
        logger.filters[:] = [
            item
            for item in logger.filters
            if not isinstance(item, MiniMaxRedactedLogFilter)
        ]

    try:
        p, _ = _make_minimax()
        del p

        with caplog.at_level(logging.INFO):
            provider_logger.info("Authorization: Bearer %s", api_key)
            httpx_logger.info("Authorization: Bearer %s", api_key)

        assert [record.getMessage() for record in caplog.records] == [
            "Authorization: Bearer ***",
            "Authorization: Bearer ***",
        ]
    finally:
        for logger, filters in previous_filters.items():
            logger.filters[:] = filters


def test_redacted_log_filter_replaces_key_in_message():
    f = MiniMaxRedactedLogFilter(api_key="secret-key-123")
    rec = logging.LogRecord(
        name="t",
        level=logging.INFO,
        pathname="x.py",
        lineno=1,
        msg="Authorization: Bearer secret-key-123 failed",
        args=(),
        exc_info=None,
    )
    assert f.filter(rec) is True
    assert rec.msg == "Authorization: Bearer *** failed"


def test_redacted_log_filter_no_op_when_no_key():
    f = MiniMaxRedactedLogFilter(api_key=None, api_key_env="UNSET_VAR")
    rec = logging.LogRecord(
        name="t",
        level=logging.INFO,
        pathname="x.py",
        lineno=1,
        msg="no key here",
        args=(),
        exc_info=None,
    )
    assert f.filter(rec) is True
    assert rec.msg == "no key here"


def test_redacted_log_filter_resolves_env_var():
    import os

    os.environ["MINIMAX_TEST_KEY"] = "env-secret"
    try:
        f = MiniMaxRedactedLogFilter(api_key_env="MINIMAX_TEST_KEY")
        rec = logging.LogRecord(
            name="t",
            level=logging.INFO,
            pathname="x.py",
            lineno=1,
            msg="key was env-secret in payload",
            args=(),
            exc_info=None,
        )
        f.filter(rec)
        assert "env-secret" not in rec.msg
        assert "***" in rec.msg
    finally:
        del os.environ["MINIMAX_TEST_KEY"]
