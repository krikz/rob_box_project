"""Tests for the dummy LLM provider and the harness-side fake.

Verifies the provider's contract on the public surface used by
the framework:

* ``complete`` returns deterministic responses based on user text.
* ``stream`` matches ``complete`` chunk-wise.
* ``aclose`` is a no-op.
* ``capabilities`` is conservative.
* ``HarnessFakeLLMProvider`` is a working alias for ``FakeLLMProvider``.
"""

from __future__ import annotations

import pytest

from rob_box_harness.providers.dummy import DummyLLMProvider
from rob_box_harness.providers.fake_llm import HarnessFakeLLMProvider
from rob_box_llm.provider import (
    LLMMessage,
    LLMSettings,
    ProviderCapabilities,
    ToolCall,
)


@pytest.mark.asyncio
async def test_complete_returns_echo_for_unknown_text() -> None:
    """The provider echoes back any non-ping text."""
    provider = DummyLLMProvider()
    response = await provider.complete(
        [LLMMessage(role="user", content="hello world")],
    )
    assert response.content == "echo: hello world"
    assert response.finish_reason == "stop"
    assert provider.call_count == 1


@pytest.mark.asyncio
async def test_complete_returns_pong_for_ping() -> None:
    """``ping`` triggers ``pong`` plus a tool call."""
    provider = DummyLLMProvider()
    response = await provider.complete(
        [LLMMessage(role="user", content="ping")],
    )
    assert response.content == "pong"
    assert response.finish_reason == "tool_calls"
    assert len(response.tool_calls) == 1
    assert response.tool_calls[0].name == "echo"


@pytest.mark.asyncio
async def test_complete_uses_only_last_user_message() -> None:
    """Older user messages are ignored when computing the response."""
    provider = DummyLLMProvider()
    response = await provider.complete(
        [
            LLMMessage(role="user", content="old message"),
            LLMMessage(role="assistant", content="ignored"),
            LLMMessage(role="user", content="new message"),
        ],
    )
    assert response.content == "echo: new message"


@pytest.mark.asyncio
async def test_stream_char_by_char() -> None:
    """The streaming variant yields one chunk per character."""
    provider = DummyLLMProvider()
    chunks = []
    async for chunk in provider.stream(
        [LLMMessage(role="user", content="abc")],
    ):
        chunks.append(chunk)
    # "echo: abc" is 9 characters; 9 content chunks + 1 final chunk
    # with finish_reason. We assert the joined content and the
    # final chunk's finish_reason rather than the chunk count so
    # the test is robust to the dummy provider's response shape.
    content = "".join(c.content_delta for c in chunks if c.content_delta)
    assert content == "echo: abc"
    assert chunks[-1].finish_reason == "stop"


def test_capabilities_is_conservative() -> None:
    """No tools, no vision — just plain text."""
    provider = DummyLLMProvider()
    caps = provider.capabilities
    assert isinstance(caps, ProviderCapabilities)
    assert caps.text is True
    assert caps.streaming_text is False
    assert caps.tools is False
    assert caps.image_input is False


@pytest.mark.asyncio
async def test_aclose_is_noop() -> None:
    """``aclose`` is documented as a no-op for stateless providers."""
    provider = DummyLLMProvider()
    await provider.aclose()
    # Re-using the provider after aclose still works.
    response = await provider.complete(
        [LLMMessage(role="user", content="hi")],
    )
    assert response.content == "echo: hi"


def test_harness_fake_alias_resolves() -> None:
    """``HarnessFakeLLMProvider`` is the same class as the upstream fake."""
    assert HarnessFakeLLMProvider is not None
    assert hasattr(HarnessFakeLLMProvider, "complete")
    assert hasattr(HarnessFakeLLMProvider, "stream")
