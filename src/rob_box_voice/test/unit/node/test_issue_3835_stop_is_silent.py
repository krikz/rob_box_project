"""#3835: «Робот, стой» - тишина без фразы; command_node не озвучивает «Останавливаюсь».

Живой прогон 22.10.2026: Tier-1 stop_all гасил всё верно, а command_node на тот же стоп публиковал feedback
«Останавливаюсь» -> dialogue_node синтезировал его (``expect_speech``), тот же стоп отменял речь до звука, и ход через 10 с
закрывался ``timeout speech_not_started`` (earcon fail + красное кольцо). Фразу про стоп не строит никто: подтверждение
стопа - сам факт остановки (ADR-0150 §5.4, «стой» - Tier-1 без речи).
"""

from __future__ import annotations

import importlib
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.command_parser import Command, IntentType

_STUBBED = ("rclpy", "rclpy.action", "std_msgs", "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg",
            "nav2_msgs", "nav2_msgs.action")


@pytest.fixture
def command_node_module(monkeypatch):
    """``rob_box_voice.command_node`` поверх заглушек ROS (без rclpy): модуль грузится и выгружается в пределах теста."""
    for name in _STUBBED:
        monkeypatch.setitem(sys.modules, name, MagicMock(name=name))
    node_mod = ModuleType("rclpy.node")
    node_mod.Node = type("Node", (), {})
    monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)
    monkeypatch.delitem(sys.modules, "rob_box_voice.command_node", raising=False)
    module = importlib.import_module("rob_box_voice.command_node")
    yield module
    sys.modules.pop("rob_box_voice.command_node", None)


def _node(module, *, navigation: bool):
    node = object.__new__(module.CommandNode)
    node.get_logger = lambda: MagicMock()
    node.feedback_pub = MagicMock()
    node.enable_navigation = navigation
    node.current_goal_handle = object()
    return node


@pytest.mark.parametrize("navigation", [False, True])
def test_stop_is_executed_but_says_nothing(command_node_module, navigation):
    node = _node(command_node_module, navigation=navigation)
    node.create_client = MagicMock()
    node.create_client.return_value.wait_for_service.return_value = False  # Nav2 cancel недоступен - стоп всё равно тихий
    node.handle_stop(Command(intent=IntentType.STOP, text="стой", entities={}, confidence=1.0))
    node.feedback_pub.publish.assert_not_called()
    assert node.current_goal_handle is None  # локальный handle цели сброшен


def test_stop_intent_from_dialog_does_not_reach_the_feedback_topic(command_node_module):
    """Весь путь ``/dialog/intent stop_motion`` -> ``execute_command``: на /voice/command/feedback не уходит ничего."""
    node = _node(command_node_module, navigation=False)
    node.publish_intent = MagicMock()
    msg = SimpleNamespace(kind="stop_motion", epoch=0, turn_id="", args_json='{"entities": {}, "text": "стой"}')
    node.intent_callback(msg)
    node.publish_intent.assert_called_once()
    node.feedback_pub.publish.assert_not_called()
