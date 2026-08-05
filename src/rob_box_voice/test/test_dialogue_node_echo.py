#!/usr/bin/env python3
"""
test_dialogue_node_echo.py — Unit-тесты Fix A (issue 989) для DialogueNode.

Проверяет, что dialogue_node НЕ реагирует на rejected(empty)/маркеры
отклонения в /voice/stt/result: молчит, не запускает диалог, не дёргает
TTS/LLM. Тест НЕ требует rclpy: DialogueNode._on_stt тестируется через
__new__ + ручные моки (как в test_stt_node_fallback).
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

import pytest

_OPTIONAL_DEPS = {
    "rclpy": True,
    "rclpy.node": True,
    "rclpy.qos": True,
    "rclpy.callback_groups": True,
    "std_msgs": True,
    "std_msgs.msg": True,
    "audio_common_msgs": True,
    "audio_common_msgs.msg": True,
    "rob_box_harness": False,
    "rob_box_harness.core": False,
    "rob_box_harness.core.dialog_core": False,
    "rob_box_harness.core.dialogue_state_machine": False,
    "rob_box_harness.core.tool_registry": False,
    "rob_box_harness.executors": False,
    "rob_box_harness.memory": False,
    "rob_box_harness.providers": False,
    "rob_box_harness.tools": False,
    "rob_box_voice.core": False,
    "rob_box_voice.core.dialogue_text": False,
    "rob_box_voice.core.dj_mode": False,
    "rob_box_voice.core.speak_helpers": False,
}


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальный rclpy mock (как в test_stt_node_fallback)."""

    class _NodeBase:
        def __init__(self, *a, **kw):
            pass

        def declare_parameter(self, *a, **kw):
            pass

        def get_parameter(self, name):
            return MagicMock(value="")

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def create_timer(self, *a, **kw):
            return MagicMock()

        def get_logger(self):
            return MagicMock(
                info=_node_no_op,
                warning=_node_no_op,
                warn=_node_no_op,
                error=_node_no_op,
                debug=_node_no_op,
            )

    class _NodeMod:
        Node = _NodeBase

    class _Rclpy:
        node = _NodeMod()

        @staticmethod
        def init(*a, **kw):
            pass

        @staticmethod
        def shutdown(*a, **kw):
            pass

        @staticmethod
        def spin(*a, **kw):
            pass

    monkeypatch.setitem(sys.modules, "rclpy", _Rclpy())
    monkeypatch.setitem(sys.modules, "rclpy.node", _Rclpy.node)

    class _QoSMod:
        QoSProfile = MagicMock()
        ReliabilityPolicy = MagicMock()
        HistoryPolicy = MagicMock()

    monkeypatch.setitem(sys.modules, "rclpy.qos", _QoSMod())

    class _CBGMod:
        ReentrantCallbackGroup = MagicMock()

    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", _CBGMod())

    class _Msg:
        Bool = MagicMock()
        Int32 = MagicMock()
        String = MagicMock()

    monkeypatch.setitem(sys.modules, "std_msgs", _Msg())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", _Msg)

    class _AudioMsg:
        AudioData = MagicMock()

    monkeypatch.setitem(sys.modules, "audio_common_msgs", _AudioMsg())
    monkeypatch.setitem(sys.modules, "audio_common_msgs.msg", _AudioMsg)

    # Заглушки для тяжёлых зависимостей DialogueNode — _on_stt их не трогает,
    # но import модуля их требует.
    for mod in [
        "rob_box_harness",
        "rob_box_harness.core",
        "rob_box_harness.core.dialog_core",
        "rob_box_harness.core.dialogue_state_machine",
        "rob_box_harness.core.tool_registry",
        "rob_box_harness.executors",
        "rob_box_harness.memory",
        "rob_box_harness.providers",
        "rob_box_harness.tools",
        "rob_box_voice.core",
        "rob_box_voice.core.dj_mode",
        "rob_box_voice.core.speak_helpers",
    ]:
        monkeypatch.setitem(sys.modules, mod, MagicMock())

    # dialogue_text нужен с реалистичным поведением: has_wake_word /
    # strip_wake_word / is_silence_command / is_unsilence_command.
    dt = MagicMock()
    dt.has_wake_word = MagicMock(
        side_effect=lambda text, words: any(w in text for w in words)
    )
    dt.strip_wake_word = MagicMock(
        side_effect=lambda text, words: _strip_wake_word(text, words)
    )
    dt.is_silence_command = MagicMock(return_value=False)
    dt.is_unsilence_command = MagicMock(return_value=False)
    monkeypatch.setitem(sys.modules, "rob_box_voice.core.dialogue_text", dt)


def _strip_wake_word(text: str, words) -> str:
    """Удалить wake word из начала строки (реалистичная заглушка)."""
    lowered = text.lower()
    for w in words:
        if lowered.startswith(w):
            return text[len(w):].strip(" ,.!")
    return text


@pytest.fixture(autouse=True)
def _ensure_optional_deps(monkeypatch):
    for cached in ["rob_box_voice.dialogue_node"]:
        sys.modules.pop(cached, None)
    _ensure_rclpy_mock(monkeypatch)
    yield


def _make_dialogue_node_stub():
    """DialogueNode через __new__ + ручные атрибуты, только для _on_stt."""
    from rob_box_voice import dialogue_node as dialogue_node_module

    node = dialogue_node_module.DialogueNode.__new__(
        dialogue_node_module.DialogueNode
    )
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    node._wake_words = ["робок", "робот", "роббокс"]
    node._dsm = MagicMock()
    node._dsm.current_state = MagicMock()
    node._dj = MagicMock()
    node._dj.state.enabled = False
    node._cancel_run = MagicMock()
    node._sound_trigger_pub = MagicMock()
    node._publish_state = MagicMock()
    node._dispatch_turn = MagicMock()
    node._verbose_llm = False
    return node


class TestRejectedEmptyGuard:
    """Fix A: dialogue_node молчит на rejected(empty)-маркеры."""

    @pytest.mark.parametrize(
        "marker",
        [
            "rejected(empty)",
            "«rejected(empty)»",
            "rejected",
            "empty",
            "«пусто»",
            "тишина",
        ],
    )
    def test_rejected_marker_ignored(self, marker):
        node = _make_dialogue_node_stub()

        msg = MagicMock()
        msg.data = marker
        node._on_stt(msg)

        # Диалог не запущен, ничего не опубликовано
        node._dispatch_turn.assert_not_called()
        node._cancel_run.assert_not_called()
        node._sound_trigger_pub.publish.assert_not_called()
        node._dsm.on_event.assert_not_called()

    def test_empty_string_ignored(self):
        node = _make_dialogue_node_stub()

        msg = MagicMock()
        msg.data = ""
        node._on_stt(msg)

        node._dispatch_turn.assert_not_called()

    def test_normal_text_still_dispatched(self):
        """Обычный текст с wake word продолжает работать (регрессия-гейт)."""
        node = _make_dialogue_node_stub()
        node._dsm.current_state = MagicMock()

        msg = MagicMock()
        msg.data = "робок, включи музыку"
        node._on_stt(msg)

        node._dispatch_turn.assert_called_once()
