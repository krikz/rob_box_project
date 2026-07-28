"""Tests for :mod:`rob_box_voice.core.voice_settings`.

ADR-0001 §2.7 D12 — VoiceSettings unification (volume + pitch
collapsed into a single ``voice_settings`` tool). The tests cover
the pure parser (always importable) and the dispatch path of
``make_voice_settings_tool`` via a stubbed ``function_tool``
decorator when the ``agents`` SDK is unavailable.
"""

from __future__ import annotations

import importlib
import json
from typing import Any

import pytest


# ── Import the module under test (agents SDK optional) ─────────────────


rob_box_voice_settings = importlib.import_module(
    "rob_box_voice.core.voice_settings"
)
SUPPORTED_ACTIONS = rob_box_voice_settings.SUPPORTED_ACTIONS
ACTION_REQUIRES_VALUE = rob_box_voice_settings.ACTION_REQUIRES_VALUE
VoiceSettingsCall = rob_box_voice_settings.VoiceSettingsCall
parse_voice_settings = rob_box_voice_settings.parse_voice_settings
make_voice_settings_tool = rob_box_voice_settings.make_voice_settings_tool


# Decide at module-load time whether we can exercise the real factory.
try:
    import agents  # noqa: F401
    HAS_AGENTS_SDK = True
except ImportError:
    HAS_AGENTS_SDK = False


# ── Vocabulary sanity ───────────────────────────────────────────────────


def test_supported_actions_contains_all_volume_presets():
    assert {"louder", "quieter", "max", "normal", "pitch"} <= SUPPORTED_ACTIONS


def test_pitch_is_the_only_value_requiring_action():
    assert ACTION_REQUIRES_VALUE == frozenset({"pitch"})


def test_supported_actions_is_immutable():
    assert isinstance(SUPPORTED_ACTIONS, frozenset)


# ── parse_voice_settings ───────────────────────────────────────────────


class TestParseVoiceSettings:
    def test_pitch_with_value(self):
        parsed = parse_voice_settings({"action": "pitch", "value": 1.5})
        assert isinstance(parsed, VoiceSettingsCall)
        assert parsed.action == "pitch"
        assert parsed.value == 1.5
        assert parsed.mcp_tool == "set_pitch"
        assert parsed.mcp_params == {"pitch": 1.5}

    def test_pitch_with_int_value_coerced_to_float(self):
        parsed = parse_voice_settings({"action": "pitch", "value": 1})
        assert parsed.value == 1.0
        assert parsed.mcp_params == {"pitch": 1.0}

    def test_pitch_without_value_raises(self):
        with pytest.raises(ValueError) as exc_info:
            parse_voice_settings({"action": "pitch"})
        assert "pitch" in str(exc_info.value)

    def test_pitch_with_non_numeric_value_raises(self):
        with pytest.raises(ValueError) as exc_info:
            parse_voice_settings({"action": "pitch", "value": "high"})
        assert "numeric" in str(exc_info.value)

    def test_louder_action_dispatches_to_set_volume(self):
        parsed = parse_voice_settings({"action": "louder"})
        assert parsed.mcp_tool == "set_volume"
        assert parsed.mcp_params == {"action": "louder"}

    def test_quieter_action_dispatches_to_set_volume(self):
        parsed = parse_voice_settings({"action": "quieter"})
        assert parsed.mcp_tool == "set_volume"
        assert parsed.mcp_params == {"action": "quieter"}

    def test_max_action_dispatches_to_set_volume(self):
        parsed = parse_voice_settings({"action": "max"})
        assert parsed.mcp_tool == "set_volume"
        assert parsed.mcp_params == {"action": "max"}

    def test_normal_action_dispatches_to_set_volume(self):
        parsed = parse_voice_settings({"action": "normal"})
        assert parsed.mcp_tool == "set_volume"
        assert parsed.mcp_params == {"action": "normal"}

    def test_symbolic_action_ignores_extra_value(self):
        parsed = parse_voice_settings({"action": "louder", "value": 9.9})
        assert parsed.mcp_params == {"action": "louder"}

    def test_symbolic_action_with_garbage_value_ignores_it(self):
        parsed = parse_voice_settings({"action": "louder", "value": "junk"})
        assert parsed.mcp_params == {"action": "louder"}

    def test_unknown_action_raises(self):
        with pytest.raises(ValueError) as exc_info:
            parse_voice_settings({"action": "teleport"})
        assert "teleport" in str(exc_info.value)
        assert "Supported" in str(exc_info.value)

    def test_missing_action_raises(self):
        with pytest.raises(ValueError) as exc_info:
            parse_voice_settings({})
        assert "action" in str(exc_info.value)

    def test_empty_action_raises(self):
        with pytest.raises(ValueError):
            parse_voice_settings({"action": ""})

    def test_non_string_action_raises(self):
        with pytest.raises(ValueError):
            parse_voice_settings({"action": 7})


# ── make_voice_settings_tool: pre-flight ───────────────────────────────


def test_factory_rejects_none_caller():
    with pytest.raises(ValueError):
        make_voice_settings_tool(caller=None)


@pytest.mark.skipif(
    HAS_AGENTS_SDK,
    reason="agents SDK present — the factory succeeds, exercised below",
)
def test_factory_raises_when_agents_sdk_missing():
    async def caller(name, params, timeout=10.0):
        return ""

    with pytest.raises(RuntimeError) as exc_info:
        make_voice_settings_tool(caller=caller)
    assert "agents" in str(exc_info.value).lower()


# ── make_voice_settings_tool: dispatch path (requires agents SDK) ───────


@pytest.mark.skipif(
    not HAS_AGENTS_SDK,
    reason="openai-agents SDK not installed in this environment",
)
class TestFactoryDispatchWithSDK:
    """End-to-end factory tests — only run when ``agents`` is available."""

    @pytest.fixture
    def captured_calls(self):
        return []

    @pytest.fixture
    def caller(self, captured_calls):
        async def _caller(name, params, timeout=10.0):
            captured_calls.append((name, dict(params), timeout))
            return f"{name}({params})"

        return _caller

    @pytest.mark.asyncio
    async def test_pitch_action_dispatches_with_value(self, captured_calls, caller):
        tool = make_voice_settings_tool(caller=caller)
        await tool.on_invoke_tool(
            _FakeToolContext(),
            json.dumps({"action": "pitch", "value": 1.4}),
        )
        assert captured_calls == [("set_pitch", {"pitch": 1.4}, 10.0)]

    @pytest.mark.asyncio
    async def test_louder_action_dispatches_to_volume(self, captured_calls, caller):
        tool = make_voice_settings_tool(caller=caller)
        await tool.on_invoke_tool(
            _FakeToolContext(), json.dumps({"action": "louder"})
        )
        assert captured_calls == [("set_volume", {"action": "louder"}, 10.0)]

    @pytest.mark.asyncio
    async def test_max_action_dispatches_to_volume(self, captured_calls, caller):
        tool = make_voice_settings_tool(caller=caller)
        await tool.on_invoke_tool(
            _FakeToolContext(), json.dumps({"action": "max"})
        )
        assert captured_calls == [("set_volume", {"action": "max"}, 10.0)]

    @pytest.mark.asyncio
    async def test_unknown_action_returns_error_no_caller(self, captured_calls, caller):
        tool = make_voice_settings_tool(caller=caller)
        result = await tool.on_invoke_tool(
            _FakeToolContext(), json.dumps({"action": "explode"})
        )
        assert "ERROR" in result
        assert captured_calls == []

    @pytest.mark.asyncio
    async def test_pitch_without_value_returns_error(self, captured_calls, caller):
        tool = make_voice_settings_tool(caller=caller)
        result = await tool.on_invoke_tool(
            _FakeToolContext(), json.dumps({"action": "pitch"})
        )
        assert "ERROR" in result
        assert captured_calls == []

    @pytest.mark.asyncio
    async def test_custom_timeout_forwarded(self, captured_calls):
        async def _caller(name, params, timeout=10.0):
            captured_calls.append((name, dict(params), timeout))
            return ""

        tool = make_voice_settings_tool(caller=_caller, timeout=42.5)
        await tool.on_invoke_tool(
            _FakeToolContext(), json.dumps({"action": "louder"})
        )
        assert captured_calls[0][2] == 42.5


# ── Helper: ToolContext stub ───────────────────────────────────────────


class _FakeToolContext:
    """Minimal stand-in for the SDK's ``ToolContext``."""

    def __init__(self) -> None:
        self.context = ""