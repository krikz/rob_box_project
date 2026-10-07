"""Issue #3518 — «Клод …» из Telegram не идёт в диалог робота."""

from unittest.mock import MagicMock

import pytest
from std_msgs.msg import String

from rob_box_voice.core.coordinator_address import is_coordinator_address, skip_coordinator_tg
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


def _wrapped():
    handler = MagicMock()
    logger = MagicMock()
    return skip_coordinator_tg(handler, lambda: logger), handler


def test_tg_klod_never_reaches_dialogue():
    w, handler = _wrapped()
    w(String(data="[TG:495039871] Клод на Моцарте не попадает в ритм"))
    handler.assert_not_called()


@pytest.mark.parametrize("data", [
    "Клод тест",  # голосом — как раньше
    "[TG:495039871] Робот, стоп",  # обычный TG
    "[TG:495039871] Клодия, привет",
])
def test_other_messages_pass(data):
    w, handler = _wrapped()
    msg = String(data=data)
    w(msg)
    handler.assert_called_once_with(msg)


def test_dialogue_node_subscribes_through_filter():
    import inspect
    src = inspect.getsource(DialogueNode.__init__)
    assert "skip_coordinator_tg(self._on_stt" in src
