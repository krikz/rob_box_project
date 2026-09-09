"""Tests for value-object and ABC basics."""

from __future__ import annotations

import pytest

from rob_box_llm.provider import (
    ImagePart,
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
    TextPart,
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


# ---------------------------------------------------------------------------
# Multimodal content parts (M0 / P1)
# ---------------------------------------------------------------------------


def test_text_part_is_frozen_and_simple():
    p = TextPart(text="hello")
    assert p.text == "hello"
    with pytest.raises(Exception):
        p.text = "bye"  # type: ignore[misc]


def test_image_part_default_detail_is_default():
    p = ImagePart(source=b"\x89PNG", media_type="image/png")
    assert p.detail == "default"
    assert p.media_type == "image/png"


def test_image_part_validates_mime_prefix():
    with pytest.raises(ValueError, match="must start with 'image/'"):
        ImagePart(source="x", media_type="text/plain")


def test_image_part_validates_detail():
    with pytest.raises(ValueError, match="must be 'low', 'default' or 'high'"):
        ImagePart(source="x", media_type="image/jpeg", detail="ultra")


def test_image_part_accepts_url_and_bytes():
    a = ImagePart(source="https://x/y.jpg")
    b = ImagePart(source=b"\xff\xd8\xff", media_type="image/jpeg")
    assert isinstance(a.source, str)
    assert isinstance(b.source, bytes)


def test_llmmessage_accepts_text_parts_tuple():
    msg = LLMMessage(
        role="user",
        content=(TextPart(text="hi"),),
    )
    assert msg.content == (TextPart(text="hi"),)


def test_llmmessage_string_content_still_works():
    """Backward compatibility: existing string callers keep working."""
    msg = LLMMessage(role="user", content="hi")
    assert msg.content == "hi"


# ---------------------------------------------------------------------------
# ProviderCapabilities
# ---------------------------------------------------------------------------


def test_capabilities_defaults_are_conservative():
    caps = ProviderCapabilities()
    assert caps.text is True
    assert caps.streaming_text is False
    assert caps.tools is False
    assert caps.streaming_tools is False
    assert caps.image_input is False


def test_provider_default_capabilities_property_is_conservative():
    """``LLMProvider.capabilities`` defaults to text-only unless overridden."""
    from rob_box_llm.providers.deepseek import DeepSeekProvider

    # DeepSeek declares streaming_text + tools but no image_input.
    caps = DeepSeekProvider(base_url="https://x", api_key="k").capabilities
    assert caps.text is True
    assert caps.streaming_text is True
    assert caps.tools is True
    assert caps.image_input is False


def test_capabilities_for_defaults_to_capabilities():
    """Providers without model-specific narrowing return their base caps."""
    from rob_box_llm.providers.deepseek import DeepSeekProvider

    p = DeepSeekProvider(base_url="https://x", api_key="k")
    assert p.capabilities_for("any-model") == p.capabilities
