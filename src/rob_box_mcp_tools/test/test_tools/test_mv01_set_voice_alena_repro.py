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

    def test_set_voice_alena_rejected_on_minimax_provider(self, mock_node):
        """Гипотеза #2: «alena» (yandex-голос) НЕ доступен у minimax-провайдера.

        Acceptance в voice_core_suite_v1.json: ``expected_tool_calls=["set_voice"]``
        + ``voice_changed=true``. После ``set_voice("alena")`` LLM получает
        ``voice_unavailable``, и в логе нет ``[set_voice] voice='alena'...``,
        что FAILит ``voice_changed=true``.

        Этот тест фиксирует текущее поведение. Если кто-то изменит
        ``SetVoiceTool`` так, чтобы yandex-голоса молча фоллбечились на
        minimax-голоса, этот тест сломается — что и нужно.
        """
        _wire_minimax_node(mock_node)
        tool = SetVoiceTool(mock_node)

        result = tool.execute(voice="alena")

        # Текущее поведение: voice_unavailable (НЕ успех).
        assert result.success is False
        assert result.data["error"] == "voice_unavailable"
        assert result.data["requested"] == "alena"
        assert result.data["provider"] == "minimax"
        # В available — список minimax-голосов (НЕ alena).
        assert "alena" not in result.data["available"]
        # Дефолтный голос minimax — male-qn-qingse.
        assert result.data["default_voice"] == "male-qn-qingse"
        # ВНИМАНИЕ: НЕ опубликовано в /voice/tts/current_voice.
        pub = mock_node.get_publisher("/voice/tts/current_voice")
        assert pub.published_messages == []

    def test_set_voice_alena_lacks_log_for_voice_changed_check(self, mock_node):
        """Проверяем, что e2e-скрипт ``e2e_voice_test.sh`` не найдёт
        ``[set_voice] voice='...' provider=... default=...`` в логах.

        Скрипт (строки 1115-1129) делает::

            re.findall(r"\\[set_voice\\] voice='([^']*)'\\s+provider=(\\S+)\\s+default=(\\S+)", logs)

        и если совпадений нет → ``voice_change_ok = False`` → acceptance FAIL.
        """
        _wire_minimax_node(mock_node)
        tool = SetVoiceTool(mock_node)

        tool.execute(voice="alena")  # → voice_unavailable, no log emitted

        # Лог SetVoiceTool — только ``[set_voice] voice=...`` пишется при
        # УСПЕШНОМ выполнении (см. tools/dialogue.py:1007-1010). На
        # voice_unavailable tool только возвращает MCPToolResult.
        info_messages = mock_node.get_logger().info_messages
        set_voice_success_logs = [
            m for m in info_messages if "[set_voice] voice='" in m
        ]
        assert set_voice_success_logs == [], (
            "ОЖИДАЕМО: после set_voice('alena') на minimax-провайдере НЕТ "
            "успешного лога. Это и есть причина fail-streak'а mv01 в "
            "voice_core_suite_v1.json (acceptance voice_changed=true)."
        )

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

    def test_set_voice_minimax_voice_succeeds_and_logs(self, mock_node):
        """Контр-тест: если LLM выбирает minimax-голос, set_voice УСПЕШЕН.

        Это значит для прохождения mv01 LLM должна выбрать minimax-голос
        (``female-shaonv``, ``Russian_BrightHeroine`` и т.п.) — но
        мастер-промпт сейчас не даёт явного правила «alena недоступна на
        minimax, используй female-shaonv».
        """
        _wire_minimax_node(mock_node)
        tool = SetVoiceTool(mock_node)

        result = tool.execute(voice="female-shaonv")

        assert result.success is True
        assert result.data["voice_set"] == "female-shaonv"
        assert result.data["default_voice"] == "male-qn-qingse"
        # Этот лог проходит voice_changed=true.
        info_messages = mock_node.get_logger().info_messages
        assert any(
            "[set_voice] voice='female-shaonv'" in m for m in info_messages
        )
