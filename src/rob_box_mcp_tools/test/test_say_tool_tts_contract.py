"""Инструмент ``say`` публикует запрос, который ``tts_node`` принимает.

Баг (живой робот, 29.09.2026): ``say`` слал в ``/voice/tts/request``
``{"text": ..., "source": "operator"}``, ``tts_node.dialogue_callback``
отбрасывал его с «⚠ Chunk без SSML», а инструмент отвечал success=True.

Контракт приёмника — ``rob_box_core.utterance.missing_tts_request_fields``:
ей же пользуется ``dialogue_callback`` (см.
``rob_box_voice/test/unit/tts/test_tts_request_contract.py``), поэтому
разъехаться продюсер и приёмник больше не могут молча.

Запуск:
    PYTHONPATH=src/rob_box_mcp_tools:src/rob_box_core \\
        pytest src/rob_box_mcp_tools/test/test_say_tool_tts_contract.py -v
"""

from __future__ import annotations

import json
import sys
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.utterance import missing_tts_request_fields
from rob_box_mcp_tools.tools.say import SayTool, build_say_request


class _String:
    def __init__(self):
        self.data = ""


@pytest.fixture
def say_tool(monkeypatch):
    """SayTool на фейковой ноде; std_msgs подменён, если ROS нет."""
    try:
        import std_msgs.msg  # noqa: F401
    except ImportError:
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.String = _String
        std_msgs.msg = std_msgs_msg
        monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
        monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    node = MagicMock()
    publisher = MagicMock()
    node.create_publisher.return_value = publisher
    return SayTool(node), node, publisher


def test_build_say_request_is_accepted_by_tts_contract():
    payload = build_say_request("Всем привет")

    assert missing_tts_request_fields(payload) == []
    assert payload["ssml"] == "<speak>Всем привет</speak>"
    assert payload["source"] == "operator"


def test_build_say_request_escapes_xml_special_chars():
    payload = build_say_request("x<y & z>0")

    assert payload["ssml"] == "<speak>x&lt;y &amp; z&gt;0</speak>"


def test_legacy_text_only_payload_is_rejected_by_contract():
    """Регресс-якорь: старый формат ``say`` приёмник отбрасывает."""
    assert missing_tts_request_fields({"text": "привет", "source": "operator"}) == [
        "ssml"
    ]


def test_execute_publishes_payload_accepted_by_tts_node(say_tool):
    tool, node, publisher = say_tool

    result = tool.execute(text="Робот, скажи & улыбнись")

    assert result.success is True
    node.create_publisher.assert_called_once()
    assert node.create_publisher.call_args.args[1] == "/voice/tts/request"
    publisher.publish.assert_called_once()
    published = json.loads(publisher.publish.call_args.args[0].data)
    assert missing_tts_request_fields(published) == []
    assert published["ssml"] == "<speak>Робот, скажи &amp; улыбнись</speak>"
    assert published["source"] == "operator"


def test_execute_empty_text_publishes_nothing(say_tool):
    tool, _node, publisher = say_tool

    result = tool.execute(text="   ")

    assert result.success is False
    publisher.publish.assert_not_called()


def test_say_executes_via_mcp_registry_not_unknown_tool(say_tool):
    """MCPToolRegistry.execute — не «unknown tool» (issue #3305)."""
    from rob_box_mcp_tools.registry import MCPToolRegistry

    tool, _node, publisher = say_tool
    registry = MCPToolRegistry()
    registry.register(tool)

    result = registry.execute("say", text="Привет, люди")

    assert result.success is True, result
    publisher.publish.assert_called_once()
