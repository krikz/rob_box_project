"""PR-8 (#3702): адаптер речи v1 слушает ответы ``dialogue_node`` только при ``dialog_engine=v1``."""

from __future__ import annotations

from unittest.mock import MagicMock

from rob_box_core import dialog_engine
from rob_box_voice.core import speech_compat


def _topics(engine):
    node = MagicMock()
    speech_compat.attach_legacy(node, avatar_control_topic="/avatar/tts/control", engine=engine)
    return [c.args[1] for c in node.create_subscription.call_args_list]


def test_v1_keeps_every_legacy_input():
    assert _topics(dialog_engine.V1) == [
        "/voice/dialogue/response", "/voice/tts/request", "/voice/tts/control", "/avatar/tts/control"]


def test_v2_drops_only_dialogue_response():
    """Внешние писатели (`/voice/tts/request` — e2e-раннер, джойстик; STOP ТАРС) остаются при обоих движках."""
    assert _topics(dialog_engine.V2) == ["/voice/tts/request", "/voice/tts/control", "/avatar/tts/control"]


def test_default_is_v1():
    node = MagicMock()
    speech_compat.attach_legacy(node, avatar_control_topic="/a")
    assert "/voice/dialogue/response" in [c.args[1] for c in node.create_subscription.call_args_list]
