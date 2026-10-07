"""Issue #3518 — «Клод …» из Telegram не идёт в диалог робота."""

from unittest.mock import MagicMock

import pytest
from std_msgs.msg import String

from rob_box_voice.core.coordinator_address import is_coordinator_address
from rob_box_voice.dialogue_node import DialogueNode


@pytest.mark.parametrize("text", [
    "Клод на Моцарте не попадает в ритм", "клод тест", "Клод, привет",
    "  Клод", "Claude, look", "claude look", "Клод: смотри", "КЛОД",
])
def test_address_matches(text):
    assert is_coordinator_address(text)


@pytest.mark.parametrize("text", [
    "Клодия, привет", "робот, клод тест", "это клод", "", "Клодт", "Claudette",
    "включи сет", None,
])
def test_address_does_not_match(text):
    assert not is_coordinator_address(text)


def _node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._claim_utterance_id = MagicMock()
    n._stt_admission = MagicMock()
    n._dispatch_cleaned = MagicMock()
    return n


def test_tg_klod_never_reaches_dialogue():
    n = _node()
    n._on_stt(String(data="[TG:495039871] Клод на Моцарте не попадает в ритм"))
    n._claim_utterance_id.assert_not_called()
    n._stt_admission.evaluate.assert_not_called()
    n._dispatch_cleaned.assert_not_called()


def test_voice_klod_is_not_intercepted():
    """Только Telegram: голосом «Клод» идёт обычным путём (issue #3518)."""
    n = _node()
    n._claim_utterance_id.side_effect = RuntimeError("прошло дальше")
    with pytest.raises(RuntimeError):
        n._on_stt(String(data="Клод тест"))
