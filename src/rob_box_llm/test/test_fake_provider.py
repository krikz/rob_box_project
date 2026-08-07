"""Tests for `FakeLLMProvider` (no network, no SDK)."""

from __future__ import annotations

import pytest

from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ToolCall
from rob_box_llm.providers.fake import FakeLLMProvider


def _user(text: str) -> LLMMessage:
    return LLMMessage(role="user", content=text)


def test_complete_pops_scripted_responses_in_order():
    p = FakeLLMProvider(
        responses=[
            LLMResponse(content="first"),
            LLMResponse(content="second"),
        ]
    )
    import asyncio

    assert asyncio.run(p.complete([_user("a")])).content == "first"
    assert asyncio.run(p.complete([_user("b")])).content == "second"


def test_complete_raises_when_exhausted():
    p = FakeLLMProvider()
    with pytest.raises(IndexError):
        import asyncio

        asyncio.run(p.complete([_user("a")]))


def test_complete_records_calls():
    p = FakeLLMProvider(responses=[LLMResponse(content="ok")])
    import asyncio

    asyncio.run(p.complete([_user("hi")], tools=({"type": "function"},)))
    assert len(p.calls) == 1
    call = p.calls[0]
    assert call.kind == "complete"
    assert call.messages[0].content == "hi"
    assert call.tools and call.tools[0]["type"] == "function"


def test_stream_emits_chunks_in_order_and_ends_with_finish_reason():
    p = FakeLLMProvider()
    p.queue_stream_from_text("Hello", " world", finish_reason="stop")
    import asyncio

    async def drain():
        out = []
        async for c in p.stream([_user("a")]):
            out.append(c)
        return out

    chunks = asyncio.run(drain())
    assert [c.content_delta for c in chunks if c.content_delta] == ["Hello", " world"]
    assert chunks[-1].finish_reason == "stop"


def test_stream_tool_calls_via_helper():
    tc = ToolCall(id="1", name="play_sound", arguments={"name": "beep"})
    p = FakeLLMProvider()
    p.queue_tool_call_stream(tc)
    import asyncio

    async def drain():
        out = []
        async for c in p.stream([_user("a")]):
            out.append(c)
        return out

    chunks = asyncio.run(drain())
    assert any(c.tool_call_delta and c.tool_call_delta.id == "1" for c in chunks)
    assert chunks[-1].finish_reason == "tool_calls"


def test_on_complete_callback_overrides_scripted():
    async def cb(messages, tools, settings):
        return LLMResponse(content=f"echo:{messages[0].content}")

    p = FakeLLMProvider(on_complete=cb)
    import asyncio

    assert asyncio.run(p.complete([_user("ping")])).content == "echo:ping"


def test_queue_methods_extend_scripts():
    p = FakeLLMProvider()
    p.queue_stream_from_text("a")
    p.queue_stream_from_text("b")
    assert len(p._stream_scripts) == 2


def test_stream_on_stream_callback():
    async def cb(messages, tools, settings):
        for ch in (LLMChunk(content_delta="x"), LLMChunk(content_delta="y")):
            yield ch
        yield LLMChunk(finish_reason="stop")

    p = FakeLLMProvider(on_stream=cb)
    import asyncio

    async def drain():
        return [ch async for ch in p.stream([_user("a")])]

    chunks = asyncio.run(drain())
    assert [c.content_delta for c in chunks if c.content_delta] == ["x", "y"]
    assert chunks[-1].finish_reason == "stop"
