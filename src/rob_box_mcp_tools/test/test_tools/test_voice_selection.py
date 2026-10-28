"""
test_voice_selection.py — issue #1219: LLM voice selection in MCP tools.

Covers:
* SpeakTextTool accepts ``voice`` param, validates against the active
  provider's voice list, includes ``voice_used`` + ``provider`` in the
  result, and publishes ``voice`` in the SpeechRequest (/voice/speech/request);
* unknown voice → provider default + voice_fell_back=True (Q6);
* PR-4 ч.3: голос хранит tts_node (VoiceOwner); speak_text без voice= не несёт голос (говорит текущим);
* SetVoiceTool: просит владельца голоса и озвучивает исход по событию voice_changed.

Mocked ROS2: see conftest.py MockNode.
"""

import sys
from unittest.mock import Mock

import pytest

# Mock std_msgs перед импортом — локальный паттерн test_tools/.
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем тяжёлые модули, которые тянет tools/__init__.py.
sys.modules.setdefault("rob_box_mcp_tools.tools.navigation", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.system", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.perception", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.mapping", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.memory", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.music", Mock())

from rob_box_mcp_tools.tools.dialogue import (  # noqa: E402
    SetVoiceTool,
    SpeakTextTool,
)
from rob_box_mcp_tools.voice_request import VoiceOutcome, resolve_speak_voice  # noqa: E402


def _published_tts_payloads(tool) -> list:
    """Поля ``SpeechRequest`` (ADR-0150 PR-3), опубликованных speak_text."""
    pub = tool.node.get_publisher("/voice/speech/request")
    return [{"voice": m.voice, "ssml": m.text, "agent": m.agent} for m in pub.published_messages]


@pytest.mark.unit
class TestSpeakTextVoiceParam:
    """Phase 1 — voice в speak_text."""

    def test_schema_includes_voice_optional(self, mock_node):
        tool = SpeakTextTool(mock_node)
        params = {p.name: p for p in tool.parameters}
        assert "voice" in params
        assert params["voice"].required is False

    def test_known_voice_passed_in_payload(self, mock_node):
        tool = SpeakTextTool(mock_node)
        # female-shaonv — голос MiniMax (активный провайдер по умолчанию).
        result = tool.execute(text="Привет", animation="idle", voice="female-shaonv")
        assert result.success is True
        assert result.data["voice_used"] == "female-shaonv"
        assert result.data["provider"] == "minimax"
        assert result.data["voice_fell_back"] is False
        payloads = _published_tts_payloads(tool)
        assert payloads, "TTS request must be published"
        assert all(p.get("voice") == "female-shaonv" for p in payloads)

    def test_cross_provider_voice_falls_back_to_default(self, mock_node):
        """Голос Yandex (alena) недоступен у MiniMax → дефолт MiniMax (Q6)."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", voice="alena")
        assert result.success is True
        # minimax default (tts_provider param default = minimax)
        assert result.data["voice_used"] == "male-qn-qingse"
        assert result.data["voice_fell_back"] is True
        payloads = _published_tts_payloads(tool)
        assert all(p.get("voice") == "male-qn-qingse" for p in payloads)

    def test_cast_voice_by_russian_name_is_resolved_by_code(self, mock_node):
        """#4086: «Весельчак» - имя из каталога MiniMax; в запрос уходит id, `voice_used` называет применённый."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", voice="Весельчак")
        assert result.data["voice_used"] == "English_Jovialman"
        assert result.data["voice_fell_back"] is False
        assert all(p.get("voice") == "English_Jovialman" for p in _published_tts_payloads(tool))

    def test_unknown_voice_falls_back_to_default(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", voice="terminator_3000")
        assert result.success is True
        # minimax default (tts_provider param default = minimax)
        assert result.data["voice_used"] == "male-qn-qingse"
        assert result.data["voice_fell_back"] is True
        payloads = _published_tts_payloads(tool)
        assert all(p.get("voice") == "male-qn-qingse" for p in payloads)

    def test_no_voice_means_current_voice_of_tts_node(self, mock_node):
        """Без voice= реплика не несёт голос: звучит текущим голосом tts_node (его хранит VoiceOwner)."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет")
        assert result.data["voice_used"] == ""
        assert result.data["provider"] == "minimax"
        assert all(not p.get("voice") for p in _published_tts_payloads(tool))
        assert resolve_speak_voice("minimax", None) == ("", False)

    def test_provider_from_node_param(self, mock_node):
        """Если mcp_server задал tts_provider=yandex — валидация по Yandex."""
        mock_node._declared_params = {"tts_provider": "yandex"}

        def _get_parameter(name):
            return type("P", (), {"value": mock_node._declared_params.get(name, "minimax")})()

        mock_node.get_parameter = _get_parameter
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", voice="alena")
        assert result.data["voice_used"] == "alena"
        assert result.data["provider"] == "yandex"

    def test_actual_provider_wins_over_param(self, mock_node):
        """Issue #1229 — фактический провайдер от tts_node имеет приоритет.

        После фолбека minimax→yandex mcp_server получает
        actual_tts_provider=yandex из /voice/tts/provider_state. SpeakTextTool
        должен валидировать голоса по yandex, а не по номинальному
        tts_provider=minimax — иначе LLM выбирает minimax-голоса, которых
        нет у фактического провайдера, и робот говорит тем же голосом.
        """
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.actual_tts_provider = "yandex"

        def _get_parameter(name):
            return type("P", (), {"value": mock_node._declared_params.get(name, "minimax")})()

        mock_node.get_parameter = _get_parameter
        tool = SpeakTextTool(mock_node)
        # minimax-голос недоступен у фактического yandex → дефолт anton.
        result = tool.execute(text="Привет", voice="male-chengshu")
        assert result.data["voice_used"] == "anton"
        assert result.data["voice_fell_back"] is True
        assert result.data["provider"] == "yandex"
        payloads = _published_tts_payloads(tool)
        assert all(p.get("voice") == "anton" for p in payloads)
        # yandex-голос проходит валидацию у фактического провайдера.
        result2 = tool.execute(text="Привет", voice="zahar")
        assert result2.data["voice_used"] == "zahar"
        assert result2.data["voice_fell_back"] is False

@pytest.mark.unit
class TestSetVoiceTool:
    """set_voice: просит владельца голоса (tts_node) и говорит исход по событию (PR-4 ч.3)."""

    @pytest.fixture(autouse=True)
    def _owner(self, monkeypatch):
        self.calls = []
        self.outcome = None

        class _Fake:
            def request(inner, voice, provider="", wait_s=0.0):
                self.calls.append((voice, provider))
                return self.outcome or VoiceOutcome("applied", voice, voice, provider or "minimax")

        monkeypatch.setattr("rob_box_mcp_tools.tools.dialogue.voice_requests", lambda node: _Fake())

    def test_set_voice_ok_outcome_comes_from_event(self, mock_node):
        result = SetVoiceTool(mock_node).execute(voice="female-shaonv")
        assert self.calls == [("female-shaonv", "")]
        assert result.success is True
        assert result.data["voice_applied"] == "female-shaonv"
        assert result.data["status"] == "applied" and result.data["voice_default"] is False

    def test_set_voice_passes_the_name_as_said_and_code_of_the_owner_finds_the_id(self, mock_node):
        """#4086: тул не угадывает голос по имени сам - просит владельца («Весельчак»), id находит реестр владельца."""
        self.outcome = VoiceOutcome("applied", "Весельчак", "English_Jovialman", "minimax")
        result = SetVoiceTool(mock_node).execute(voice="Весельчак")
        assert self.calls == [("Весельчак", "")]
        assert result.success is True
        assert result.data["voice_requested"] == "Весельчак"
        assert result.data["voice_applied"] == "English_Jovialman"
        assert "English_Jovialman" in result.data["available"]
        assert result.data["status"] == "applied"

    def test_set_voice_unknown_voice_is_owner_decision(self, mock_node):
        self.outcome = VoiceOutcome("fallback", "terminator_3000", "male-qn-qingse", "minimax", "voice_unavailable")
        result = SetVoiceTool(mock_node).execute(voice="terminator_3000")
        assert result.success is False
        assert result.data["voice_applied"] == "male-qn-qingse"
        assert result.data["status"] == "fallback" and result.data["voice_default"] is True

    def test_set_voice_empty_rejected(self, mock_node):
        result = SetVoiceTool(mock_node).execute(voice="  ")
        assert result.success is False
        assert result.error == "voice_empty"
        assert self.calls == []
