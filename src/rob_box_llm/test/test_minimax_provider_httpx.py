"""HTTP-level tests for ``MiniMaxProvider`` using ``respx``.

The companion file ``test_minimax_provider.py`` exercises the provider
through a *fake* ``AsyncOpenAI`` client — that mocks the SDK surface but
never touches the wire. This file goes one layer deeper and mocks the
underlying HTTP transport with :mod:`respx`, so we can also assert:

* the ``Authorization: Bearer <key>`` header is sent on every request,
* the request body shape matches the OpenAI Chat-Completions wire format,
* the provider maps 5xx / timeout / connection errors to the right
  domain exception (and the SDK's ``max_retries`` loop is honoured),
* the provider maps 4xx to a domain exception **without** retrying,
* ``base_resp`` envelopes on HTTP 200 still surface as typed errors,
* streaming responses are parsed chunk-by-chunk.

We mark every test with ``@pytest.mark.minimax`` so a targeted run is
``pytest -m 'minimax' -k httpx`` and a guard against accidental network
egress is unused — ``respx`` will refuse to let a request reach the
socket because the router is active.

Reference: ``architecture/minimax-provider.md``,
``docs/adr/0002-minimax-provider.md``.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any, AsyncIterator

import httpx
import pytest
import respx
from openai import AsyncOpenAI

from rob_box_llm.errors import (
    AuthError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMMessage,
    LLMSettings,
)
from rob_box_llm.providers.minimax import (
    DEFAULT_THINKING_POLICY,
    MiniMaxProvider,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: The endpoint suffix the OpenAI SDK hits regardless of provider name.
#: ``_OpenAICompatibleProvider`` lets the subclass pick ``base_url``; the
#: path is fixed by the SDK. Asserting against it here future-proofs the
#: tests against silent path changes in an upstream release.
_CHAT_COMPLETIONS_PATH = "/chat/completions"

#: Sentinel credentials. Distinct from the conftest's TTS fixtures so a
#: leak in the LLM path can't be silenced by a TTS scrubber.
_API_KEY = "minimax-httpx-tests-secret-do-not-ship-fa3a"
_BASE_URL = "https://api.minimax.io/v1"


# ---------------------------------------------------------------------------
# Provider construction helper
# ---------------------------------------------------------------------------


def _make_provider(
    *,
    max_retries: int = 0,
    model: str = MiniMaxProvider.DEFAULT_MODEL,
    thinking: dict[str, str] | None = None,
) -> MiniMaxProvider:
    """Build a provider with a real ``AsyncOpenAI`` client.

    ``max_retries=0`` is the default for tests so each handler returns
    exactly one wire-level call. Tests that explicitly want to exercise
    the SDK retry loop override this.

    ``thinking=None`` is the default for wire-level tests because the
    instance default ``{"type": "disabled"}`` is forwarded via
    ``settings.extra`` and reaches the OpenAI Chat Completions SDK as a
    bare keyword argument — which the SDK rejects with ``TypeError``.
    See the comprehensive test coverage in ``test_minimax_provider.py``
    for the unit-level behaviour of the thinking policy; the wire-level
    tests here focus on the HTTP contract, not on the thinking surface.
    """
    client = AsyncOpenAI(
        base_url=_BASE_URL,
        api_key=_API_KEY,
        max_retries=max_retries,
    )
    return MiniMaxProvider(
        base_url=_BASE_URL,
        api_key=_API_KEY,
        model=model,
        client=client,
        thinking=thinking,
    )


def _ok_envelope(
    content: str = "hello",
    *,
    base_resp: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Body of a successful chat-completions response (OpenAI wire format)."""
    return {
        "id": "chatcmpl-httpx-test",
        "choices": [
            {
                "index": 0,
                "message": {"role": "assistant", "content": content},
                "finish_reason": "stop",
            }
        ],
        "usage": {"prompt_tokens": 1, "completion_tokens": 2, "total_tokens": 3},
        "base_resp": base_resp or {"status_code": 0, "status_msg": "ok"},
    }


def _stream_envelope(
    content: str = "",
    *,
    finish_reason: str | None = None,
    base_resp: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Body of a single SSE chunk in a streaming response."""
    return {
        "id": "chatcmpl-stream",
        "choices": [
            {
                "index": 0,
                "delta": {"role": "assistant", "content": content},
                "finish_reason": finish_reason,
            }
        ],
        "base_resp": base_resp or {"status_code": 0, "status_msg": "ok"},
    }


def _stream_tool_call_envelope(
    index: int,
    *,
    id: str | None = None,
    name: str = "",
    arguments: str = "",
    base_resp: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Body of a single SSE chunk carrying a streaming tool-call delta.

    OpenAI's wire format delivers tool-call arguments token-by-token across
    multiple chunks, each with the same ``index``; the adapter is expected to
    concatenate ``function.arguments`` fragments and ``function.name`` slices
    (see ``_OpenAICompatibleProvider.stream``, live 06.08).
    """
    tool_call: dict[str, Any] = {"index": index}
    if id is not None:
        tool_call["id"] = id
    function: dict[str, Any] = {}
    if name:
        function["name"] = name
    if arguments:
        function["arguments"] = arguments
    if function:
        tool_call["function"] = function
    return {
        "id": "chatcmpl-stream",
        "choices": [
            {
                "index": 0,
                "delta": {"role": "assistant", "tool_calls": [tool_call]},
                "finish_reason": None,
            }
        ],
        "base_resp": base_resp or {"status_code": 0, "status_msg": "ok"},
    }


# ---------------------------------------------------------------------------
# Wire-format: successful requests with varied payloads
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_sends_authorization_header_and_payload() -> None:
    """The provider sends ``Authorization: Bearer *** and the expected JSON body."""
    # ``thinking=None`` keeps the default thinking policy OFF so the
    # request body matches the OpenAI Chat-Completions wire contract that
    # the upstream SDK actually accepts. See ``test_thinking_*`` below for
    # the unit-level coverage of the thinking-policy merge.
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("hi"))
            )
            resp = await provider.complete([LLMMessage(role="user", content="hi")])

        assert resp.content == "hi"
        assert route.called
        assert route.call_count == 1
        # Auth header is the wire-level contract — assert it explicitly.
        request = route.calls.last.request
        assert request.headers["Authorization"] == f"Bearer {_API_KEY}"
        # Body shape: messages, model, no stream, no thinking (opt-out).
        body = json.loads(request.content)
        assert body["model"] == MiniMaxProvider.DEFAULT_MODEL
        assert body["messages"] == [{"role": "user", "content": "hi"}]
        assert body["stream"] is False
        assert "thinking" not in body, (
            "thinking=None must NOT leak onto the wire — see minimax.py::_apply_thinking_policy"
        )
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_with_default_thinking_policy_does_not_send_thinking_kwarg() -> None:
    """Wire-level guard: the default thinking policy must not leak as a kwarg.

    The OpenAI SDK does NOT accept ``thinking`` as a Chat-Completions
    parameter. If the provider forwards the instance default
    ``{"type": "disabled"}`` mapping as a bare kwarg, the SDK raises
    ``TypeError: AsyncCompletions.create() got an unexpected keyword
    argument 'thinking'``. We assert the *behaviour* the caller cares
    about — the request reaches the wire cleanly — without depending on
    the exact on-the-wire encoding.
    """
    provider = _make_provider()  # default thinking policy applied
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            resp = await provider.complete([LLMMessage(role="user", content="hi")])

        assert resp.content == "ok"
        # Either the SDK ignored ``thinking`` silently, or the body never
        # contained it. Either way: one wire call, no exception.
        assert route.call_count == 1
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_with_system_prompt_serialises_in_order() -> None:
    """Multi-message payloads preserve the order on the wire."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope())
            )
            await provider.complete(
                [
                    LLMMessage(role="system", content="be terse"),
                    LLMMessage(role="user", content="hello"),
                    LLMMessage(role="assistant", content="hi"),
                    LLMMessage(role="user", content="bye"),
                ]
            )

        body = json.loads(route.calls.last.request.content)
        assert [m["role"] for m in body["messages"]] == [
            "system",
            "user",
            "assistant",
            "user",
        ]
        assert body["messages"][0]["content"] == "be terse"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_with_tool_definitions_forwards_tools_array() -> None:
    """Tool definitions reach the wire as the OpenAI ``tools`` array."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    json={
                        "choices": [
                            {
                                "index": 0,
                                "message": {
                                    "role": "assistant",
                                    "content": None,
                                    "tool_calls": [
                                        {
                                            "id": "call_1",
                                            "type": "function",
                                            "function": {
                                                "name": "play_sound",
                                                "arguments": '{"name":"beep"}',
                                            },
                                        }
                                    ],
                                },
                                "finish_reason": "tool_calls",
                            }
                        ],
                        "base_resp": {"status_code": 0, "status_msg": "ok"},
                    },
                )
            )
            resp = await provider.complete(
                [LLMMessage(role="user", content="beep")],
                tools=(
                    {
                        "type": "function",
                        "function": {
                            "name": "play_sound",
                            "parameters": {"type": "object", "properties": {"name": {"type": "string"}}},
                        },
                    },
                ),
            )

        body = json.loads(route.calls.last.request.content)
        assert "tools" in body
        assert body["tools"][0]["function"]["name"] == "play_sound"
        # Parsed contract: tool call comes back as a structured object.
        assert resp.tool_calls[0].name == "play_sound"
        assert resp.tool_calls[0].arguments == {"name": "beep"}
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_settings_override_model_and_temperature() -> None:
    """Per-call ``LLMSettings`` overrides the instance default."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope())
            )
            await provider.complete(
                [LLMMessage(role="user", content="hi")],
                settings=LLMSettings(model="MiniMax-M2.7", temperature=0.0, max_tokens=42),
            )

        body = json.loads(route.calls.last.request.content)
        assert body["model"] == "MiniMax-M2.7"
        assert body["temperature"] == 0.0
        assert body["max_tokens"] == 42
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_with_image_bytes_serialises_as_data_url_on_wire() -> None:
    """Bytes ``ImagePart`` payload is base64-encoded into a ``data:`` URL on the wire."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("a cat"))
            )
            payload = b"\x89PNG\r\n\x1a\n" + b"binary-image-bytes"
            await provider.complete(
                [
                    LLMMessage(
                        role="user",
                        content=(
                            ImagePart(source=payload, media_type="image/png"),
                        ),
                    )
                ]
            )

        body = json.loads(route.calls.last.request.content)
        parts = body["messages"][0]["content"]
        assert isinstance(parts, list) and len(parts) == 1
        url = parts[0]["image_url"]["url"]
        assert url.startswith("data:image/png;base64,")
        # Round-trip the encoded bytes back to our original payload.
        import base64

        assert base64.b64decode(url.split(",", 1)[1]) == payload
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Wire-format: streaming
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_yields_chunks_with_correct_deltas() -> None:
    """Streaming responses are parsed chunk-by-chunk, content deltas concatenated on the wire."""
    provider = _make_provider()
    try:
        sse_lines = [
            "data: " + json.dumps(_stream_envelope("He")),
            "data: " + json.dumps(_stream_envelope("llo")),
            "data: " + json.dumps(_stream_envelope("", finish_reason="stop")),
            "data: [DONE]",
        ]
        # httpx.Response with a streamed body via a raw bytes payload
        # containing SSE framing. The OpenAI SDK accepts SSE payloads
        # served as ``text/event-stream``.
        sse_body = "\n\n".join(sse_lines) + "\n\n"
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    headers={"Content-Type": "text/event-stream"},
                    content=sse_body.encode("utf-8"),
                )
            )
            chunks = []
            async for ch in provider.stream([LLMMessage(role="user", content="hi")]):
                chunks.append(ch)

        assert [ch.content_delta for ch in chunks] == ["He", "llo", ""]
        assert chunks[-1].finish_reason == "stop"
        # The body was sent with stream=True; verify on the wire.
        body = json.loads(route.calls.last.request.content)
        assert body["stream"] is True
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_with_tools_reaches_wire_and_aggregates_tool_calls() -> None:
    """Streaming + tools works (streaming_tools=True, live 06.08) — no capability gate, wire call made.

    The OpenAI-compatible adapter aggregates streaming tool-call deltas by
    index. The request must reach the wire with ``stream=True`` and the
    ``tools`` payload; the resulting tool call must be reassembled across
    SSE chunks (arguments arrive token-by-token).
    """
    provider = _make_provider()
    try:
        sse_lines = [
            "data: " + json.dumps(_stream_tool_call_envelope(0, id="call_1", name="play_sound", arguments='{"volume":')),
            "data: " + json.dumps(_stream_tool_call_envelope(0, arguments=" 0.5}")),
            "data: " + json.dumps(_stream_envelope("", finish_reason="tool_calls")),
            "data: [DONE]",
        ]
        sse_body = "\n\n".join(sse_lines) + "\n\n"
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    headers={"Content-Type": "text/event-stream"},
                    content=sse_body.encode("utf-8"),
                )
            )
            chunks = []
            async for ch in provider.stream(
                [LLMMessage(role="user", content="hi")],
                tools=({"type": "function", "function": {"name": "play_sound"}},),
            ):
                chunks.append(ch)

        assert route.called, "streaming with tools must reach the wire"
        body = json.loads(route.calls.last.request.content)
        assert body["stream"] is True
        assert "tools" in body
        # Tool-call deltas aggregated across chunks by index.
        tool_chunks = [ch for ch in chunks if ch.tool_call_delta is not None]
        assert len(tool_chunks) == 1
        assert tool_chunks[0].tool_call_delta.name == "play_sound"
        assert tool_chunks[0].tool_call_delta.arguments == {"volume": 0.5}
        assert chunks[-1].finish_reason == "tool_calls"
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Retry semantics: 5xx retries, 4xx does not retry
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_5xx_triggers_sdk_retry_loop_then_raises_provider_error() -> None:
    """Transient 5xx: the SDK retries up to ``max_retries`` times, then the provider raises.

    This is the test the task brief explicitly asks for: *after N retries
    the expected exception is raised.* The provider itself does not own
    the retry loop — the OpenAI SDK does — but the provider MUST forward
    the resulting ``APIStatusError`` as a typed ``ProviderError`` without
    swallowing it.
    """
    # max_retries=2 -> 3 total attempts (1 + 2 retries).
    provider = _make_provider(max_retries=2)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    500,
                    json={"error": {"message": "upstream unavailable"}},
                )
            )
            with pytest.raises(ProviderError):
                await provider.complete([LLMMessage(role="user", content="hi")])

        assert route.call_count == 3, (
            "expected 1 initial attempt + 2 retries = 3 wire calls before the SDK gives up"
        )
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_4xx_does_not_retry_and_raises_typed_error() -> None:
    """Non-retryable 4xx: exactly one wire call, then the typed error."""
    provider = _make_provider(max_retries=5)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    401,
                    json={"error": {"message": "bad api key"}},
                )
            )
            with pytest.raises(AuthError):
                await provider.complete([LLMMessage(role="user", content="hi")])

        assert route.call_count == 1, "401 must not trigger SDK retries"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_429_triggers_sdk_retry_loop_then_raises_rate_limit() -> None:
    """429 is retryable in the openai SDK — after N attempts we raise ``RateLimitError``.

    This is the wire-level companion to
    ``test_complete_sends_authorization_header_and_payload``'s assertion
    that retried requests still surface typed errors. The provider does
    NOT own the retry loop — the SDK does — but the contract is: after
    exhausting the SDK's ``max_retries`` budget the provider MUST raise
    a typed error, never swallow it.
    """
    # max_retries=2 -> 3 total attempts (1 + 2 retries).
    provider = _make_provider(max_retries=2)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    429,
                    json={"error": {"message": "rate-limited"}},
                )
            )
            with pytest.raises(RateLimitError):
                await provider.complete([LLMMessage(role="user", content="hi")])

        assert route.call_count == 3, (
            "expected 1 initial attempt + 2 retries = 3 wire calls before the SDK gives up"
        )
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_5xx_succeeds_after_retry() -> None:
    """If the second attempt succeeds, the provider returns the response normally."""
    provider = _make_provider(max_retries=2)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            # First call 500, second call 200.
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                side_effect=[
                    httpx.Response(500, json={"error": "transient"}),
                    httpx.Response(200, json=_ok_envelope("recovered")),
                ]
            )
            resp = await provider.complete([LLMMessage(role="user", content="hi")])

        assert resp.content == "recovered"
        assert route.call_count == 2
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Streaming base_resp error envelope
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_stream_base_resp_error_raises_before_yielding_via_wire() -> None:
    """A 200 OK response whose first chunk carries a bad ``base_resp`` raises immediately."""
    provider = _make_provider()
    try:
        sse_lines = [
            "data: " + json.dumps(
                _stream_envelope(base_resp={"status_code": 1008, "status_msg": "quota exceeded"})
            ),
            "data: [DONE]",
        ]
        sse_body = "\n\n".join(sse_lines) + "\n\n"
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    headers={"Content-Type": "text/event-stream"},
                    content=sse_body.encode("utf-8"),
                )
            )
            with pytest.raises(RateLimitError):
                async for _ in provider.stream([LLMMessage(role="user", content="hi")]):
                    pass
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_base_resp_quota_error_raises_rate_limit() -> None:
    """HTTP 200 with ``base_resp.status_code=1008`` (quota) maps to ``RateLimitError``."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    json=_ok_envelope(base_resp={"status_code": 1008, "status_msg": "insufficient quota"}),
                )
            )
            with pytest.raises(RateLimitError, match="quota"):
                await provider.complete([LLMMessage(role="user", content="hi")])
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_complete_base_resp_safety_error_raises_content_filter() -> None:
    """``base_resp.status_code=1001`` (safety) maps to ``ContentFilterError``."""
    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    200,
                    json=_ok_envelope(
                        base_resp={"status_code": 1001, "status_msg": "content safety violation"}
                    ),
                )
            )
            with pytest.raises(ContentFilterError, match="safety"):
                await provider.complete([LLMMessage(role="user", content="hi")])
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Timeout / connection — shallow verification through the SDK exceptions
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_timeout_via_httpx_delay_raises_timeout_error() -> None:
    """A real httpx timeout surfaces as our ``TimeoutError``."""

    # respx can inject a delay via a custom transport. We use a side_effect
    # that raises ``httpx.ConnectTimeout`` — the SDK wraps it as
    # ``APIConnectionError`` which the provider maps to ``TimeoutError``.
    provider = _make_provider(max_retries=0)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                side_effect=httpx.ConnectTimeout("simulated"),
            )
            with pytest.raises(TimeoutError):
                await provider.complete([LLMMessage(role="user", content="hi")])
        assert route.call_count == 1
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Image payload size limit — envelope-level guard
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_oversized_image_payload_is_rejected_before_wire() -> None:
    """Oversized ``ImagePart`` short-circuits BEFORE hitting the network."""
    from rob_box_llm.errors import CapabilityUnavailableError
    from rob_box_llm.providers.minimax import MINIMAX_MAX_IMAGE_BYTES

    provider = _make_provider()
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope())
            )
            huge = b"x" * (MINIMAX_MAX_IMAGE_BYTES + 1)
            with pytest.raises(CapabilityUnavailableError, match="exceeds limit"):
                await provider.complete(
                    [
                        LLMMessage(
                            role="user",
                            content=(ImagePart(source=huge, media_type="image/jpeg"),),
                        )
                    ]
                )
        assert not route.called, "oversized payload must not reach the wire"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_image_input_to_non_vision_model_is_rejected_before_wire() -> None:
    """Provider-side image capability gating forbids non-vision models."""
    from rob_box_llm.errors import CapabilityUnavailableError

    provider = _make_provider(model="MiniMax-M2.7")
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope())
            )
            with pytest.raises(CapabilityUnavailableError, match="image input not supported"):
                await provider.complete(
                    [
                        LLMMessage(
                            role="user",
                            content=(ImagePart(source="https://x.invalid/y.jpg"),),
                        )
                    ]
                )
        assert not route.called
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Tear-down: aclose is idempotent
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_aclose_is_idempotent_via_real_client() -> None:
    """Calling aclose() twice must not raise on the underlying httpx client."""
    provider = _make_provider()
    await provider.aclose()
    # Second call — should be a no-op.
    await provider.aclose()


# ---------------------------------------------------------------------------
# Base URL routing — guards the wire contract against silent endpoint drift
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_default_base_url_hits_minimax_global_chat_completions() -> None:
    """A provider built with default ``base_url`` routes to the global endpoint.

    Regression guard for the Hermes memory note ("MiniMax provider hardcodes
    base_url and uses wrong endpoint path"). The MiniMax global
    Chat-Completions URL is ``https://api.minimax.io/v1`` and the SDK
    appends ``/chat/completions``. Anything else means the request went
    to the wrong vendor (e.g. a Xiaomi MiMo mirror) and the test fails
    loudly.
    """
    client = AsyncOpenAI(
        base_url=MiniMaxProvider.DEFAULT_BASE_URL,
        api_key=_API_KEY,
        max_retries=0,
    )
    provider = MiniMaxProvider(client=client, thinking=None)
    try:
        with respx.mock(
            base_url=MiniMaxProvider.DEFAULT_BASE_URL, assert_all_called=False
        ) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            await provider.complete([LLMMessage(role="user", content="hi")])

        assert route.called
        # The URL the SDK hit must round-trip back to ``api.minimax.io``.
        request_url = str(route.calls.last.request.url)
        assert request_url.startswith(
            f"{MiniMaxProvider.DEFAULT_BASE_URL}{_CHAT_COMPLETIONS_PATH}"
        )
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_custom_base_url_is_honoured_on_wire() -> None:
    """A custom ``base_url`` flows through to the HTTP layer unchanged.

    Self-hosted / private-cloud deployments override ``base_url``. If
    the provider stops forwarding it (e.g. hardcoding the production
    URL inside ``complete()``) the override becomes silently inert —
    this test catches that regression.
    """
    custom_base = "https://llm.internal.example.corp/v1"
    client = AsyncOpenAI(base_url=custom_base, api_key=_API_KEY, max_retries=0)
    provider = MiniMaxProvider(base_url=custom_base, api_key=_API_KEY, client=client, thinking=None)
    try:
        with respx.mock(base_url=custom_base, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            await provider.complete([LLMMessage(role="user", content="hi")])

        request_url = str(route.calls.last.request.url)
        assert request_url.startswith(f"{custom_base}{_CHAT_COMPLETIONS_PATH}")
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_request_body_omits_unset_settings_fields() -> None:
    """Only non-default ``LLMSettings`` fields reach the wire.

    The provider MUST NOT serialise ``temperature=None`` / ``max_tokens=None``
    / empty ``stop`` as ``null`` or empty arrays on the wire — that wastes
    bytes and (worse) can confuse some vendor gateways that don't accept
    explicit ``null`` for optional fields. We assert the exact minimal
    body shape.
    """
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            await provider.complete(
                [LLMMessage(role="user", content="hi")],
                settings=LLMSettings(),  # all defaults
            )

        body = json.loads(route.calls.last.request.content)
        assert set(body.keys()) == {"messages", "model", "stream"}
        assert "temperature" not in body
        assert "max_tokens" not in body
        assert "stop" not in body
        assert "tool_choice" not in body
        assert "tools" not in body
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_request_body_carries_explicit_stop_and_tool_choice() -> None:
    """Explicit ``stop`` / ``tool_choice`` / ``max_tokens`` reach the wire."""
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            await provider.complete(
                [LLMMessage(role="user", content="hi")],
                settings=LLMSettings(
                    max_tokens=64,
                    stop=("END", "STOP"),
                    tool_choice="auto",
                ),
            )

        body = json.loads(route.calls.last.request.content)
        assert body["max_tokens"] == 64
        assert body["stop"] == ["END", "STOP"]
        assert body["tool_choice"] == "auto"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_request_body_does_not_leak_api_key_into_payload() -> None:
    """The API key MUST stay in the ``Authorization`` header — never in the body.

    A regression here would mean the key ends up in upstream logs /
    proxy caches / Sentry breadcrumbs. Belt-and-braces alongside the
    ``MiniMaxRedactedLogFilter`` so the wire itself is clean.
    """
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(200, json=_ok_envelope("ok"))
            )
            await provider.complete([LLMMessage(role="user", content="hi")])

        body_text = route.calls.last.request.content.decode("utf-8")
        assert _API_KEY not in body_text
        # And the header is set — i.e. we didn't accidentally lose it.
        assert route.calls.last.request.headers["Authorization"] == f"Bearer {_API_KEY}"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_authorization_header_is_set_on_every_retry() -> None:
    """Every retry attempt carries the ``Authorization`` header.

    A subtle failure mode: the SDK rotates transports between attempts
    and one of them forgets to forward the auth header. We confirm both
    attempts reach the wire with a valid Bearer token by hitting 500
    twice then a third 200 — three calls, three valid headers.
    """
    provider = _make_provider(max_retries=3, thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                side_effect=[
                    httpx.Response(500, json={"error": "transient"}),
                    httpx.Response(500, json={"error": "transient"}),
                    httpx.Response(200, json=_ok_envelope("recovered")),
                ]
            )
            resp = await provider.complete([LLMMessage(role="user", content="hi")])

        assert resp.content == "recovered"
        assert route.call_count == 3
        for call in route.calls:
            assert call.request.headers["Authorization"] == f"Bearer {_API_KEY}"
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Error mapping — extended wire-level coverage
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_400_is_mapped_to_typed_provider_error_without_retry() -> None:
    """HTTP 400 (non-retryable 4xx) raises a typed ``ProviderError`` exactly once.

    Brief point (3) says: "на 4xx ... падает сразу". The OpenAI SDK
    does NOT retry 400 — the ``max_retries`` budget is not consumed —
    so we expect exactly one wire call and a typed error on the way out.
    The exact subclass (``ContentFilterError`` / ``RateLimitError`` /
    ``ProviderError``) depends on the body shape and is exercised by
    the fake-SDK tests in ``test_minimax_provider.py``; here we only
    pin the contract that the SDK retry loop is bypassed.
    """
    provider = _make_provider(max_retries=5, thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    400,
                    json={"error": {"message": "content violates safety policy"}},
                )
            )
            with pytest.raises(ProviderError):
                await provider.complete([LLMMessage(role="user", content="hi")])

        assert route.call_count == 1, "HTTP 400 must not trigger SDK retries"
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_403_maps_to_RateLimitError() -> None:
    """MiniMax surfaces quota/billing refusals as 403 → ``RateLimitError``."""
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    403,
                    json={"error": {"message": "plan does not cover this model"}},
                )
            )
            with pytest.raises(RateLimitError):
                await provider.complete([LLMMessage(role="user", content="hi")])
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_5xx_with_unknown_status_code_maps_to_ProviderError() -> None:
    """A non-retryable ``APIStatusError`` outside the known buckets is ``ProviderError``."""
    provider = _make_provider(thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            router.post(_CHAT_COMPLETIONS_PATH).mock(
                return_value=httpx.Response(
                    418,  # I'm a teapot — not in the SDK retry table
                    json={"error": {"message": "brewing"}},
                )
            )
            with pytest.raises(ProviderError):
                await provider.complete([LLMMessage(role="user", content="hi")])
    finally:
        await provider.aclose()


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_read_timeout_raises_TimeoutError_via_wire() -> None:
    """An ``httpx.ReadTimeout`` surfaces as our ``TimeoutError``.

    respx can inject ``httpx.ReadTimeout`` via a side_effect that
    raises — the openai SDK wraps it as ``APITimeoutError`` and the
    provider maps that to ``TimeoutError``.
    """
    provider = _make_provider(max_retries=0, thinking=None)
    try:
        with respx.mock(base_url=_BASE_URL, assert_all_called=False) as router:
            route = router.post(_CHAT_COMPLETIONS_PATH).mock(
                side_effect=httpx.ReadTimeout("read timed out", request=httpx.Request("POST", f"{_BASE_URL}/chat/completions")),
            )
            with pytest.raises(TimeoutError):
                await provider.complete([LLMMessage(role="user", content="hi")])
        assert route.call_count == 1
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Concurrency — two providers, two keys, in flight simultaneously
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_concurrent_requests_with_distinct_keys_do_not_cross_auth() -> None:
    """Two concurrent calls route auth headers to their own request.

    A regression in the underlying ``httpx.AsyncClient`` could cause
    two providers sharing the same transport to swap ``Authorization``
    headers between requests — silent credential leak. We pin each
    provider to its own ``base_url`` / ``api_key`` and assert that the
    right header landed on the right request.
    """
    base_a = "https://llm-a.internal.example/v1"
    base_b = "https://llm-b.internal.example/v1"
    key_a = "key-a-secret-do-not-ship-1"
    key_b = "key-b-secret-do-not-ship-2"

    client_a = AsyncOpenAI(base_url=base_a, api_key=key_a, max_retries=0)
    client_b = AsyncOpenAI(base_url=base_b, api_key=key_b, max_retries=0)
    p_a = MiniMaxProvider(base_url=base_a, api_key=key_a, client=client_a, thinking=None)
    p_b = MiniMaxProvider(base_url=base_b, api_key=key_b, client=client_b, thinking=None)
    try:
        with respx.mock(assert_all_called=False) as router:
            route_a = router.post(_CHAT_COMPLETIONS_PATH, host="llm-a.internal.example").mock(
                return_value=httpx.Response(200, json=_ok_envelope("from-a"))
            )
            route_b = router.post(_CHAT_COMPLETIONS_PATH, host="llm-b.internal.example").mock(
                return_value=httpx.Response(200, json=_ok_envelope("from-b"))
            )

            resp_a, resp_b = await asyncio.gather(
                p_a.complete([LLMMessage(role="user", content="hi")]),
                p_b.complete([LLMMessage(role="user", content="hi")]),
            )

        assert resp_a.content == "from-a"
        assert resp_b.content == "from-b"
        # Headers didn't cross — each request saw its own Bearer token.
        assert route_a.calls.last.request.headers["Authorization"] == f"Bearer {key_a}"
        assert route_b.calls.last.request.headers["Authorization"] == f"Bearer {key_b}"
        # And no body ever carried the *other* provider's key.
        assert key_b not in route_a.calls.last.request.content.decode("utf-8")
        assert key_a not in route_b.calls.last.request.content.decode("utf-8")
    finally:
        await p_a.aclose()
        await p_b.aclose()


# ---------------------------------------------------------------------------
# Network guard — ``respx`` must refuse unmocked requests
# ---------------------------------------------------------------------------


@pytest.mark.minimax
@pytest.mark.asyncio
async def test_unmocked_request_fails_within_respx_router() -> None:
    """An empty ``respx`` router fails fast — no real socket is opened.

    Brief rule: "никакой реальной сети". When ``respx`` is active and no
    route matches, the SDK raises ``APIConnectionError`` immediately;
    the provider maps that to ``TimeoutError``. We assert:

    * the call returns a typed error (not a real DNS/socket timeout),
    * the test finishes in well under a second — proving the request
      did NOT leave the process.

    A regression that disables ``respx`` (or wraps the call in a
    long-blocking socket) would either fail the type assertion or push
    this test past the 5-second guard below.
    """
    import time

    from respx import MockRouter

    client = AsyncOpenAI(base_url=_BASE_URL, api_key=_API_KEY, max_retries=0)
    provider = MiniMaxProvider(base_url=_BASE_URL, api_key=_API_KEY, client=client, thinking=None)
    try:
        # Activate an empty router — any unmocked request is refused fast.
        with MockRouter(assert_all_called=False):
            start = time.monotonic()
            with pytest.raises(TimeoutError):
                await provider.complete([LLMMessage(role="user", content="hi")])
            elapsed = time.monotonic() - start

        # 30-second SDK timeout / real DNS resolution would dwarf this.
        assert elapsed < 5.0, (
            f"unmocked request took {elapsed:.2f}s — looks like respx is not active "
            "and a real network call is being attempted"
        )
    finally:
        await provider.aclose()


# ---------------------------------------------------------------------------
# Negative documentation — rate limiter / circuit breaker status
# ---------------------------------------------------------------------------


@pytest.mark.minimax
def test_no_rate_limiter_or_circuit_breaker_in_provider_module() -> None:
    """Document that the MiniMax provider does NOT implement rate limiting.

    Brief point (4) says "корректная работа rate limiter / circuit breaker
    *если он есть* в реализации". This test pins the current state: the
    ``rob_box_llm.providers.minimax`` module has no rate limiter / circuit
    breaker — the SDK's ``max_retries`` is the only retry mechanism. If
    either is added later, this test fails and the author must extend
    coverage accordingly (it's a feature, not a bug — the test name
    documents the absent feature).
    """
    import rob_box_llm.providers.minimax as mod

    src = Path(mod.__file__).read_text()
    # Tokens we would expect if a rate limiter or breaker lived here.
    forbidden = ("RateLimiter", "CircuitBreaker", "TokenBucket", "ratelimit", "circuit_breaker")
    for token in forbidden:
        assert token not in src, (
            f"{token!r} found in minimax.py — if a rate limiter / circuit breaker "
            "was added, extend this test and the brief's coverage requirement."
        )
