"""test_quest_stt_source.py — Unit-тесты маршрутизации Quest STT (ADR-0027 §3.4).

Проверяет в dialogue_node:
  1. ``_on_quest_stt`` при ``voice_input_mode=quest_ttts`` (или quest_stt)
     диспатчит фразу в LLM БЕЗ wake-word (источник задаёт флаг
     ``from_quest``, а не текстовый маркер).
  2. ``_on_quest_stt`` при ``voice_input_mode=respeaker`` (default) —
     игнорирует фразу (Quest-режим не активен).
  3. ``_on_stt(from_quest=True)`` пропускает wake-gate так же, как
     Telegram-источник.

Не требует ROS2 — rclpy/rcl_interfaces замоканы в ``conftest.py``.
DialogueNode создаётся через ``object.__new__`` + ручные атрибуты
(как в test_issue_1195_tg_source.py).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


class _Param:
    def __init__(self, value):
        self.value = value


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None

    # get_parameter("voice_input_mode") → режим, которым управляет супервизор.
    n._voice_mode = "respeaker"
    n.get_parameter = lambda name: _Param(n._voice_mode)
    return n


def _make_stt_msg(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


class TestQuestSttRouting:
    def test_quest_ttts_dispatches_without_wake_word(self, node):
        """voice_input_mode=quest_ttts → фраза без wake-word доходит до LLM."""
        node._voice_mode = "quest_ttts"
        node._on_quest_stt(_make_stt_msg("привет как дела"))

        node._dispatch_turn.assert_called_once()
        # Исходный текст без wake word ушёл в диспатч (маркер не добавлялся).
        assert node._dispatch_turn.call_args.args[0] == "привет как дела"
        assert node._dispatch_turn.call_args.kwargs.get("from_tg") is False

    def test_quest_stt_mode_also_dispatches(self, node):
        """voice_input_mode=quest_stt — тоже LLM → TTS без wake-word."""
        node._voice_mode = "quest_stt"
        node._on_quest_stt(_make_stt_msg("расскажи что видишь"))
        node._dispatch_turn.assert_called_once()

    def test_respeaker_mode_ignores_quest_stt(self, node):
        """voice_input_mode=respeaker (default) → /voice/stt/quest игнорируется."""
        node._voice_mode = "respeaker"
        node._on_quest_stt(_make_stt_msg("привет как дела"))
        node._dispatch_turn.assert_not_called()

    def test_quest_passthrough_ignored_here(self, node):
        """quest_passthrough не идёт через STT (звук играет sound_node напрямую)."""
        node._voice_mode = "quest_passthrough"
        node._on_quest_stt(_make_stt_msg("привет как дела"))
        node._dispatch_turn.assert_not_called()

    def test_from_quest_flag_bypasses_wake_gate(self, node):
        """_on_stt(from_quest=True) без wake word — как Telegram-источник."""
        node._on_stt(_make_stt_msg("привет как дела"), from_quest=True)
        node._dispatch_turn.assert_called_once()
        assert node._dispatch_turn.call_args.args[0] == "привет как дела"
