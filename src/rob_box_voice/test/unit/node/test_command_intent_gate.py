"""
test_command_intent_gate.py — Unit-тесты command-intent gate (issue #1279).

Проверяет, что dialogue_node НЕ отправляет в LLM фразы, которые уже
распознаны command_node как команды движения/статуса (NAVIGATE/STOP/
STATUS/MAP). До фикса LLM интерпретировал «вперёд» как музыку →
execute_music_code вместо движения («робот поёт вместо того чтобы ехать»).

Не требует ROS2 — rclpy замокан в conftest.py. DialogueNode создаётся
через object.__new__ + ручные атрибуты (как в test_pure_methods.py).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.command_parser import CommandParser, IntentType
from rob_box_voice.dialogue_node import DialogueNode

_GATE_CONFIDENCE = 0.7


def _make_node(gate_enabled: bool = True) -> DialogueNode:
    """Минимальная DialogueNode без __init__ — только атрибуты для _on_stt."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс"]
    n._command_intent_gate_enabled = gate_enabled
    n._command_intent_gate_confidence = _GATE_CONFIDENCE
    n._command_parser = CommandParser(
        wake_words=["робот", "робокс", "робобокс"],
        confidence_base=0.8,
    )
    n._speaker_by_text = {}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
        "command_intent": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    return n


def _stt(node: DialogueNode, text: str) -> None:
    msg = MagicMock()
    msg.data = text
    node._on_stt(msg)


class TestCommandIntentGate:
    """Issue #1279 — команды движения/статуса НЕ идут в LLM."""

    @pytest.mark.parametrize(
        "phrase",
        [
            "робот вперёд",
            "робот вперед",
            "робот стоп",
            "робот налево",
            "робот направо",
            "робот, иди вперёд",
            "робот, поверни налево",
            "робот где ты",
            "робокс стоп",
        ],
    )
    def test_command_phrase_skips_llm(self, phrase):
        node = _make_node(gate_enabled=True)
        _stt(node, phrase)
        node._dispatch_turn.assert_not_called()
        # Счётчик command_intent увеличился, LLM-диспатч пропущен.
        assert node._llm_skipped_counter["command_intent"] == 1
        assert node._cancel_run.called

    @pytest.mark.parametrize(
        "phrase",
        [
            "робот как дела",
            "робот расскажи анекдот",
            "робот спой песню",
            "робот включи музыку",
            "робот что ты видишь",
        ],
    )
    def test_non_command_phrase_goes_to_llm(self, phrase):
        node = _make_node(gate_enabled=True)
        _stt(node, phrase)
        node._dispatch_turn.assert_called_once()
        assert node._llm_skipped_counter["command_intent"] == 0

    @pytest.mark.parametrize(
        "phrase",
        [
            "робот стоп музыку",
            "робот хватит диджеить",
            "робот выключи музыку",
        ],
    )
    def test_music_stop_phrase_still_goes_to_llm(self, phrase):
        """Music-stop фразы НЕ гейтятся — LLM должен вызвать stop_music."""
        node = _make_node(gate_enabled=True)
        _stt(node, phrase)
        node._dispatch_turn.assert_called_once()
        assert node._llm_skipped_counter["command_intent"] == 0

    def test_gate_disabled_allows_llm(self):
        node = _make_node(gate_enabled=False)
        _stt(node, "робот вперёд")
        node._dispatch_turn.assert_called_once()
        assert node._llm_skipped_counter["command_intent"] == 0

    def test_tg_message_not_gated(self):
        """Telegram-текст ([TG:...]) не гейтится — это не микрофон."""
        node = _make_node(gate_enabled=True)
        node._active_tg_chat_id = None
        _stt(node, "[TG:123] робот вперёд")
        node._dispatch_turn.assert_called_once()
        assert node._llm_skipped_counter["command_intent"] == 0

    def test_gate_classification_matches_command_node(self):
        """Один и тот же CommandParser, что и command_node, — совпадение 1:1."""
        node = _make_node(gate_enabled=True)
        parser = node._command_parser
        # Фразы, которые command_node распознает как команды.
        assert parser.parse("робот вперёд").intent == IntentType.NAVIGATE
        assert parser.parse("робот стоп").intent == IntentType.STOP
        assert parser.parse("робот где ты").intent == IntentType.STATUS
        # Фразы, которые остаются LLM.
        assert parser.parse("робот как дела").intent == IntentType.UNKNOWN
        assert parser.parse("робот спой песню").intent == IntentType.UNKNOWN


class TestCommandFeedbackSpeech:
    """Issue #1279 — feedback command_node озвучивается через TTS."""

    def test_feedback_spoken(self):
        node = object.__new__(DialogueNode)
        logger = MagicMock()
        node.get_logger = lambda: logger
        node._speak_direct = MagicMock()

        msg = MagicMock()
        msg.data = "Двигаюсь вперёд"
        node._on_command_feedback(msg)

        node._speak_direct.assert_called_once_with("Двигаюсь вперёд")

    def test_empty_feedback_ignored(self):
        node = object.__new__(DialogueNode)
        logger = MagicMock()
        node.get_logger = lambda: logger
        node._speak_direct = MagicMock()

        msg = MagicMock()
        msg.data = "  "
        node._on_command_feedback(msg)

        node._speak_direct.assert_not_called()
