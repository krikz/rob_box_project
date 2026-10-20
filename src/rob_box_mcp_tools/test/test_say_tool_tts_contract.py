"""Инструмент ``say`` говорит единственным клиентом речи (ADR-0150 PR-3, #3688).

Баг (живой робот, 29.09.2026): ``say`` слал в ``/voice/tts/request``
``{"text": ..., "source": "operator"}`` без ``ssml``, ``tts_node`` отбрасывал
запрос, а инструмент отвечал success=True. Теперь JSON никто не собирает:
``say`` публикует ``SpeechRequest`` через ``rob_box_core.speech_client.say``
(SSML строит ``Utterance``), агент — ``operator``.

Запуск:
    PYTHONPATH=src/rob_box_mcp_tools:src/rob_box_core \\
        pytest src/rob_box_mcp_tools/test/test_say_tool_tts_contract.py -v
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock, Mock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

# Без ROS (Windows/lint): модули, тянущие rclpy через tools/__init__.py.
try:
    import rclpy  # noqa: F401
except ImportError:
    for _mod in ("rob_box_mcp_tools.tools.navigation", "rob_box_mcp_tools.tools.system",
                 "rob_box_mcp_tools.tools.perception", "rob_box_mcp_tools.tools.mapping",
                 "rob_box_mcp_tools.tools.memory", "rob_box_mcp_tools.tools.music"):
        sys.modules.setdefault(_mod, Mock())

from rob_box_mcp_tools.tools.say import SayTool  # noqa: E402

M = install_dialog_msgs_stub()


@pytest.fixture
def say_tool():
    node = MagicMock()
    publisher = MagicMock()
    node.create_publisher.return_value = publisher
    stamp = type(M.SpeechRequest().stamp)()
    node.get_clock.return_value = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    return SayTool(node), node, publisher


def test_execute_publishes_speech_request_as_operator(say_tool):
    tool, node, publisher = say_tool

    result = tool.execute(text="Робот, скажи & улыбнись")

    assert result.success is True
    node.create_publisher.assert_called_once()
    assert node.create_publisher.call_args.args[1] == "/voice/speech/request"
    publisher.publish.assert_called_once()
    req = publisher.publish.call_args.args[0]
    assert isinstance(req, M.SpeechRequest)
    assert req.text == "<speak>Робот, скажи &amp; улыбнись</speak>"
    assert req.agent == "operator"
    assert result.data["speech_id"] == req.speech_id


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
