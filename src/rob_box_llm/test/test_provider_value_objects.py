"""Tests for value-object and ABC basics."""

from __future__ import annotations

import pytest

from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ToolCall,
    ToolResult,
)


def test_llmmessage_is_frozen():
    m = LLMMessage(role="user", content="x")
    with pytest.raises(Exception):
        m.role = "assistant"  # type: ignore[misc]


def test_llmresponse_default_factories():
    r = LLMResponse()
    assert r.content == ""
    assert r.tool_calls == ()
    assert r.usage == {}
    assert r.finish_reason is None


def test_llmchunk_optional_fields_default_to_empty():
    c = LLMChunk()
    assert c.content_delta == ""
    assert c.tool_call_delta is None
    assert c.finish_reason is None
    assert c.usage is None


def test_tool_call_is_hashable_so_it_can_sit_in_a_tuple():
    tc = ToolCall(id="1", name="x", arguments={"a": 1})
    s = {tc, ToolCall(id="1", name="x", arguments={"a": 1})}
    assert len(s) == 1


def test_abc_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        LLMProvider()  # type: ignore[abstract]


def test_llmprovider_default_aclose_is_noop():
    class _Stub(LLMProvider):
        name = "stub"

        async def complete(self, messages, *, tools=(), settings=None):
            return LLMResponse(content="x")

        async def stream(self, messages, *, tools=(), settings=None):
            yield LLMChunk(content_delta="x")
            yield LLMChunk(finish_reason="stop")

    import asyncio

    s = _Stub()
    asyncio.run(s.aclose())  # must not raise


def test_settings_extra_is_per_instance():
    a = LLMSettings(extra={"k": 1})
    b = LLMSettings(extra={"k": 1})
    assert a.extra is not b.extra


def test_tool_result_is_error_flag_default_false():
    assert ToolResult(tool_call_id="x", content="y").is_error is False
