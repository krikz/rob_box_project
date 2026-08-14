"""
test_voice_selection.py — issue #1219: LLM voice selection in MCP tools.

Covers:
* SpeakTextTool accepts ``voice`` param, validates against the active
  provider's voice list, includes ``voice_used`` + ``provider`` in the
  result, and publishes ``voice`` in the /voice/tts/request payload;
* unknown voice → provider default + voice_fell_back=True (Q6);
* VoiceStateStore: set_voice persists current_voice; next speak_text
  without voice= uses it (Q7);
* SetVoiceTool: validates against the provider list, stores voice,
  publishes /voice/tts/current_voice, returns {status, voice_set,
  default_voice}.

Mocked ROS2: see conftest.py MockNode.
"""

import json
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
from rob_box_mcp_tools.voice_state import VoiceStateStore  # noqa: E402


def _published_tts_payloads(tool) -> list:
    pub = tool.node.get_publisher("/voice/tts/request")
    out = []
    for m in pub.published_messages:
        try:
            out.append(json.loads(m.data))
        except (TypeError, ValueError):
            out.append({"_raw": m.data})
    return out


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

    def test_unknown_voice_falls_back_to_default(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", voice="terminator_3000")
        assert result.success is True
        # minimax default (tts_provider param default = minimax)
        assert result.data["voice_used"] == "male-qn-qingse"
        assert result.data["voice_fell_back"] is True
        payloads = _published_tts_payloads(tool)
        assert all(p.get("voice") == "male-qn-qingse" for p in payloads)

    def test_no_voice_uses_store_current_voice(self, mock_node):
        """set_voice(zahar) → следующий speak_text без voice= использует store.

        VoiceStateStore.resolve() возвращает fell_back=False для голоса из
        store (line 93) — fallback на дефолт провайдера при несовпадении
        провайдера НЕ считается ошибкой явного запроса.
        """
        store = VoiceStateStore()
        tool = SpeakTextTool(mock_node, voice_store=store)
        store.set_voice("female-shaonv")
        result = tool.execute(text="Привет")
        assert result.data["voice_used"] == "female-shaonv"
        assert result.data["voice_fell_back"] is False

    def test_no_voice_and_no_store_uses_provider_default(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет")
        assert result.data["voice_used"] == "male-qn-qingse"
        assert result.data["provider"] == "minimax"

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


@pytest.mark.unit
class TestSetVoiceTool:
    """Phase 3 — set_voice инструмент."""

    def test_set_voice_ok(self, mock_node):
        tool = SetVoiceTool(mock_node)
        # female-shaonv — голос MiniMax (активный провайдер по умолчанию).
        result = tool.execute(voice="female-shaonv")
        assert result.success is True
        assert result.data["status"] == "ok"
        assert result.data["voice_set"] == "female-shaonv"
        assert result.data["default_voice"] == "male-qn-qingse"
        # опубликовано в /voice/tts/current_voice для dialogue_node
        pub = mock_node.get_publisher("/voice/tts/current_voice")
        assert pub is not None and len(pub.published_messages) == 1
        payload = json.loads(pub.published_messages[0].data)
        assert payload["voice"] == "female-shaonv"

    def test_set_voice_invalid_rejected(self, mock_node):
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="terminator_3000")
        assert result.success is False
        assert result.data["error"] == "voice_unavailable"
        # ничего не сохранено и не опубликовано
        pub = mock_node.get_publisher("/voice/tts/current_voice")
        assert pub.published_messages == []

    def test_set_voice_empty_rejected(self, mock_node):
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="  ")
        assert result.success is False
        assert result.data is None or result.data.get("error") == "voice_empty"

    def test_set_voice_then_speak_text_uses_it(self, mock_node):
        """set_voice(голос провайдера) → speak_text без voice= говорит им."""
        store = VoiceStateStore()
        set_tool = SetVoiceTool(mock_node, voice_store=store)
        speak_tool = SpeakTextTool(mock_node, voice_store=store)
        set_tool.execute(voice="female-shaonv")  # minimax voice
        result = speak_tool.execute(text="Привет")
        assert result.data["voice_used"] == "female-shaonv"
        assert result.data["voice_fell_back"] is False


@pytest.mark.unit
class TestVoiceStateStore:
    """Phase 3 — in-memory хранилище current_voice."""

    def test_set_and_get(self):
        store = VoiceStateStore()
        assert store.get_voice() is None
        store.set_voice("zahar")
        assert store.get_voice() == "zahar"

    def test_speaker_binding(self):
        store = VoiceStateStore()
        store.set_voice("zahar", speaker_id="speaker-1")
        assert store.get_voice("speaker-1") == "zahar"
        assert store.get_voice() is None  # default не тронут

    def test_resolve_requested_wins(self):
        store = VoiceStateStore()
        store.set_voice("zahar")
        voice, fell = store.resolve("minimax", requested="female-shaonv")
        assert voice == "female-shaonv"
        assert fell is False

    def test_resolve_current_voice(self):
        store = VoiceStateStore()
        store.set_voice("female-shaonv")
        voice, fell = store.resolve("minimax")
        assert voice == "female-shaonv"
        assert fell is False

    def test_resolve_default_when_nothing_set(self):
        store = VoiceStateStore()
        voice, fell = store.resolve("minimax")
        assert voice == "male-qn-qingse"
        assert fell is True

    def test_resolve_cross_provider_falls_back(self):
        """Голос Yandex (zahar) установлен, но провайдер minimax → дефолт.

        По контракту voice_state.py (line 93) fell_back=False: голос пришёл
        из store (не явный запрос), fallback на дефолт провайдера — штатное
        поведение, а не ошибка явного запроса.
        """
        store = VoiceStateStore()
        store.set_voice("zahar")
        voice, fell = store.resolve("minimax")
        assert voice == "male-qn-qingse"
        assert fell is False

    def test_clear(self):
        store = VoiceStateStore()
        store.set_voice("zahar")
        store.clear()
        assert store.get_voice() is None
