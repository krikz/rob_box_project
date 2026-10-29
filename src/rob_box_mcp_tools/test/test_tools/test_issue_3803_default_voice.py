"""#3803: «верни свой обычный голос» - голос робота по умолчанию (minimax/male-qn-qingse), а не дефолт текущего
провайдера. Решает код (``voice="default"``), модель провайдера и голос не выбирает (ADR-0148)."""

import sys
from unittest.mock import Mock

import pytest

sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
for _name in ("navigation", "system", "perception", "mapping", "memory", "music"):
    sys.modules.setdefault(f"rob_box_mcp_tools.tools.{_name}", Mock())

from rob_box_mcp_tools.tools.dialogue import SetVoiceTool  # noqa: E402
from rob_box_mcp_tools.voice_request import VoiceOutcome  # noqa: E402


class _Param:
    def __init__(self, value):
        self.value = value


class _Owner:
    def __init__(self):
        self.calls = []

    def request(self, voice, provider="", wait_s=0.0):
        self.calls.append((voice, provider))
        return VoiceOutcome("applied", voice, voice, provider)


def _robot(mock_node, monkeypatch, provider, voice):
    owner = _Owner()
    monkeypatch.setattr("rob_box_mcp_tools.tools.dialogue.voice_requests", lambda node: owner)
    mock_node.get_parameter = lambda name: _Param("minimax")
    mock_node.actual_tts_provider = provider
    mock_node.actual_tts_voice = voice
    return owner


@pytest.mark.unit
def test_default_from_yandex_zahar_goes_to_minimax_qingse(mock_node, monkeypatch):
    owner = _robot(mock_node, monkeypatch, "yandex", "zahar")
    result = SetVoiceTool(mock_node).execute(voice="default")
    assert owner.calls == [("male-qn-qingse", "minimax")]
    assert result.success is True
    assert result.data["provider"] == "minimax" and result.data["voice_applied"] == "male-qn-qingse"


@pytest.mark.unit
def test_default_when_already_default_is_honest_noop(mock_node, monkeypatch):
    owner = _robot(mock_node, monkeypatch, "minimax", None)
    result = SetVoiceTool(mock_node).execute(voice="default")
    assert owner.calls == []
    assert result.success is True and result.data["status"] == "already_default"
    assert result.data["voice_default"] is True  # диалог скажет «Я и так говорю своим обычным голосом.» (VOICE_DONE)
    assert "и так" in result.message


@pytest.mark.unit
def test_default_on_minimax_with_other_voice_switches_voice(mock_node, monkeypatch):
    owner = _robot(mock_node, monkeypatch, "minimax", "Russian_ReliableMan")
    SetVoiceTool(mock_node).execute(voice="Default")
    assert owner.calls == [("male-qn-qingse", "minimax")]


@pytest.mark.unit
def test_ordinary_voice_name_is_untouched(mock_node, monkeypatch):
    owner = _robot(mock_node, monkeypatch, "yandex", "anton")
    SetVoiceTool(mock_node).execute(voice="zahar")
    assert owner.calls == [("zahar", "")]