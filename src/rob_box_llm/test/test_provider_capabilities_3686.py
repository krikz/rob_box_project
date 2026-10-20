"""Честные возможности адаптеров для режима команд (ADR-0150 §4.5, ADR-0143; #3686)."""

from __future__ import annotations

from rob_box_llm.provider import ProviderCapabilities
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.fake import FakeLLMProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.providers.minimax import MiniMaxProvider


def test_new_flags_are_conservative_by_default():
    caps = ProviderCapabilities()
    assert (caps.tool_choice, caps.json_mode, caps.structured_output, caps.thinking) == (False,) * 4
    assert caps.max_context_tokens == 0


def test_deepseek_declares_tool_choice_and_json_mode():
    caps = DeepSeekProvider(api_key="k").capabilities
    assert caps.tool_choice and caps.json_mode
    assert not caps.structured_output and not caps.thinking
    assert caps.max_context_tokens == 128_000


def test_minimax_does_not_claim_tool_choice_adr_0143():
    caps = MiniMaxProvider(api_key="k").capabilities
    assert caps.tool_choice is False
    assert caps.thinking is True
    assert caps.tools and caps.image_input


def test_minimax_non_vision_model_keeps_new_flags():
    caps = MiniMaxProvider(api_key="k").capabilities_for("MiniMax-M2.1")
    assert caps.image_input is False
    assert caps.thinking is True and caps.tool_choice is False


def test_mimo_is_unmeasured_hence_all_false():
    caps = MiMoProvider(api_key="k").capabilities
    assert (caps.tool_choice, caps.json_mode, caps.structured_output) == (False, False, False)


def test_fake_modes_are_switchable_by_parameter():
    assert FakeLLMProvider().capabilities == ProviderCapabilities()
    caps = ProviderCapabilities(tools=True, tool_choice=True, structured_output=True)
    assert FakeLLMProvider(capabilities=caps).capabilities is caps
