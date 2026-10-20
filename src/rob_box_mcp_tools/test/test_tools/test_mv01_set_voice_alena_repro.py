"""
test_mv01_set_voice_alena_repro.py — repro-тест для e2e scenario
`.github/e2e/scenarios/voice_core_suite_v1.json::mv01_set_voice_alena`.

Сценарий: пользователь говорит «Робот, говори голосом Алены».
* «alena» — yandex-голос.
* Активный TTS-провайдер в rob_box_project = ``minimax`` (default).
* Acceptance: ``expected_tool_calls: ["set_voice"]`` + ``voice_changed: true``
  → нужен лог ``[set_voice] voice='X' provider=minimax default=male-qn-qingse``
  где X != male-qn-qingse.

Этот тест фиксирует **поведение инструмента** на текущем develop:
1. ``SetVoiceTool.execute(voice="alena")`` на minimax-провайдере возвращает
   ``voice_unavailable`` (НЕ успех) — гипотеза #2/#3.
2. ``SpeakTextTool.execute(text=..., voice="alena")`` на minimax молча
   фоллбечит на default (male-qn-qingse) — это объясняет, почему LLM может
   не вызвать ``set_voice`` вообще, а вызвать ``speak_text(voice=...)``.

Цель — дать backend-воркеру **готовый repro**, чтобы он мог проверять
гипотезы, прежде чем менять ``SetVoiceTool.description`` или
``master_prompt_compact.txt``.

Refs:
* t_f29f15bc — research-карточка (этот тест — её артефакт)
* issue #1219 — multi-voice canon
* PR #1572 / run #32876911211 — fail-streak 25.08
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


class _Param:
    """Минимальный мок rclpy Parameter: ``.value``."""
    def __init__(self, value):
        self.value = value


def _wire_minimax_node(mock_node):
    """Настроить mock_node как робот на minimax-провайдере."""
    mock_node._declared_params = {"tts_provider": "minimax"}

    def _get_parameter(name):
        return _Param(mock_node._declared_params.get(name, "minimax"))

    mock_node.get_parameter = _get_parameter
    # Убедимся, что info/warning через MockLogger-инстанс (conftest.MockNode
    # уже это делает, но на всякий случай).
    if not hasattr(mock_node, "_logger_info"):
        mock_node._logger_info = []
    return mock_node


@pytest.mark.unit
class TestMv01SetVoiceAlenaRepro:
    """Repro для e2e scenario mv01_set_voice_alena (issue #1219)."""

    def test_set_voice_alena_on_minimax_is_reported_by_code(self, mock_node, monkeypatch):
        """PR-4 ч.3: «alena» (голос Yandex) у minimax - решает владелец голоса; робот честно говорит
        «Голос alena недоступен, остаюсь на male-qn-qingse», а не молча «готово»."""
        from rob_box_mcp_tools.voice_request import VoiceOutcome

        class _Owner:
            def request(self, voice, provider="", wait_s=0.0):
                return VoiceOutcome("fallback", voice, "male-qn-qingse", "minimax", "voice_unavailable")

        monkeypatch.setattr("rob_box_mcp_tools.tools.dialogue.voice_requests", lambda node: _Owner())
        _wire_minimax_node(mock_node)
        result = SetVoiceTool(mock_node).execute(voice="alena")
        assert result.success is False
        assert result.data["voice_requested"] == "alena"
        assert result.data["voice_applied"] == "male-qn-qingse"
        assert "alena" not in result.data["available"]
        assert "Голос alena недоступен, остаюсь на male-qn-qingse" in result.message
        assert "установлен" not in result.message

    def test_speak_text_alena_silently_falls_back_on_minimax(self, mock_node):
        """Гипотеза #3: speak_text(voice="alena") МОЛЧА фоллбечит на дефолт
        minimax-провайдера.

        LLM может «выбрать» speak_text вместо set_voice — и acceptance
        ``expected_tool_calls: [\"set_voice\"]`` провалится, даже если
        фактически голос прозвучал (default).
        """
        _wire_minimax_node(mock_node)
        tool = SpeakTextTool(mock_node)

        result = tool.execute(text="Говорю голосом Алены!", voice="alena")

        # SpeakTextTool молча подменяет на default.
        assert result.success is True
        assert result.data["voice_used"] == "male-qn-qingse"  # default
        assert result.data["voice_fell_back"] is True
        # НО expected_tool_calls требует set_voice → acceptance FAIL.

    def test_set_voice_minimax_voice_succeeds_and_logs(self, mock_node, monkeypatch):
        """Контр-тест: minimax-голос владелец применил - успех, лог с исходом."""
        from rob_box_mcp_tools.voice_request import VoiceOutcome

        class _Owner:
            def request(self, voice, provider="", wait_s=0.0):
                return VoiceOutcome("applied", voice, voice, "minimax")

        monkeypatch.setattr("rob_box_mcp_tools.tools.dialogue.voice_requests", lambda node: _Owner())
        _wire_minimax_node(mock_node)
        result = SetVoiceTool(mock_node).execute(voice="female-shaonv")
        assert result.success is True
        assert result.data["voice_applied"] == "female-shaonv"
        info_messages = mock_node.get_logger().info_messages
        assert any("[set_voice] voice='female-shaonv' provider=minimax" in m for m in info_messages)
