"""
test_dialogue_speak_text_inline_params.py — санитизация вшитых параметров.

LLM (MiniMax-M3 / DeepSeek в DJ-режиме) иногда вставляет ``animation=`` /
``voice=`` внутрь ``text`` вызова ``speak_text``, например::

    speak_text(text='...рубильник!", animation="happy"')

TTS читает хвост буквально → робот произносит «анимейшин хеппи».
SpeakTextTool.execute() обязан вырезать такой вшитый синтаксис ДО отправки
в TTS. Покрываем варианты:
  * двойные кавычки + «лишняя» закрывающая кавычка перед запятой;
  * двойные кавычки без лишней кавычки;
  * одинарные кавычки;
  * пробелы вокруг ``=`` и хвостовой ``)``;
  * вшитый ``voice=`` вместо ``animation=``.

Mocked ROS2: см. conftest.py MockNode.
"""

import json
import sys
from unittest.mock import Mock

import pytest

# Mock std_msgs перед импортом SpeakTextTool (локальный паттерн test_tools/).
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем модули, которые тянут rclpy через tools/__init__.py.
sys.modules.setdefault("rob_box_mcp_tools.tools.navigation", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.system", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.perception", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.mapping", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.memory", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.music", Mock())

from rob_box_mcp_tools.tools.dialogue import SpeakTextTool  # noqa: E402


def _spoken_ssml(mock_node) -> str:
    """Достать SSML первого TTS-запроса, отправленного инструментом."""
    tts_pub = mock_node.get_publisher("/voice/tts/request")
    assert tts_pub is not None
    assert len(tts_pub.published_messages) == 1
    return json.loads(tts_pub.published_messages[0].data)["ssml"]


@pytest.mark.unit
class TestSpeakTextInlineParamsSanitization:
    """Вшитые ``animation=``/``voice=`` в text не должны попадать в TTS."""

    def test_strips_double_quoted_animation_with_stray_quote(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text='Когда скажете стоп — выключу рубильник!", animation="happy"',
            animation="happy",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "рубильник!" in ssml
        assert "animation" not in ssml
        assert "happy" not in ssml

    def test_strips_double_quoted_animation_without_stray_quote(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text='Когда скажете стоп — выключу рубильник!, animation="happy"',
            animation="happy",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "рубильник!" in ssml
        assert "animation" not in ssml

    def test_strips_single_quoted_animation(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text="Когда скажете стоп — выключу рубильник!, animation='happy'",
            animation="happy",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "рубильник!" in ssml
        assert "animation" not in ssml

    def test_strips_spaces_around_equals_and_trailing_paren(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text='Когда скажете стоп — выключу рубильник!, animation = "happy")',
            animation="happy",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "рубильник!" in ssml
        assert "animation" not in ssml

    def test_strips_inlined_voice_param(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text='Когда скажете стоп — выключу рубильник!", voice="alena"',
            animation="neutral",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "рубильник!" in ssml
        assert "voice" not in ssml

    def test_does_not_touch_plain_text(self, mock_node):
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text="Обычная фраза без параметров, просто так.",
            animation="neutral",
        )
        assert result.success is True
        ssml = _spoken_ssml(mock_node)
        assert "Обычная фраза без параметров, просто так." in ssml
