"""Намерение движения/статуса: диалог → /dialog/intent → command_node (ADR-0150 PR-5 #3694)."""

import json

from rob_box_core.msg_stub import install_dialog_msgs_stub

from rob_box_voice.core.command_intent import KIND_BY_INTENT, command_from_intent, fill_intent
from rob_box_voice.core.command_parser import Command, IntentType

Intent = install_dialog_msgs_stub().Intent


def test_round_trip_keeps_intent_and_entities():
    cmd = Command(intent=IntentType.NAVIGATE, text="иди вперёд", entities={"direction": "forward"}, confidence=0.9)
    msg = fill_intent(Intent(), cmd, epoch=3, turn_id="u1")
    assert (msg.kind, msg.epoch, msg.turn_id) == ("move", 3, "u1")
    assert json.loads(msg.args_json) == {"entities": {"direction": "forward"}, "text": "иди вперёд"}
    back = command_from_intent(msg)
    assert (back.intent, back.entities, back.text) == (IntentType.NAVIGATE, {"direction": "forward"}, "иди вперёд")


def test_stop_is_stop_motion():
    msg = fill_intent(Intent(), Command(IntentType.STOP, "стой", {}, 1.0), epoch=0)
    assert msg.kind == "stop_motion" and command_from_intent(msg).intent is IntentType.STOP


def test_every_executable_intent_has_a_kind_and_unknown_has_none():
    assert set(KIND_BY_INTENT) == set(IntentType) - {IntentType.UNKNOWN}


def test_bad_messages_are_rejected():
    assert command_from_intent(Intent(kind="dance", args_json="{}")) is None
    assert command_from_intent(Intent(kind="move", args_json="{not json")) is None
    assert command_from_intent(Intent(kind="move", args_json="[]")) is None
    assert command_from_intent(Intent(kind="move", args_json="")).entities == {}
