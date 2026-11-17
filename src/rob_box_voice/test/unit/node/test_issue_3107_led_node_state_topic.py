"""Issue #3107: led_node listens to the dialogue state topic that exists.

led_node subscribed to ``/voice/state`` since it was written (bf494be3), but
nothing on the robot publishes that name (live graph, run 36422274238).
dialogue_node publishes ``DialogueStateKind.name`` as ``std_msgs/String`` on
``/voice/dialogue/state`` (dialogue_node.py ``_state_pub``), the canonical
name per ADR-0027 #2 and ``arbiter_node.VOICE_DIALOGUE_STATE_TOPIC``.

ADR-0150 PR-3b (#3690): the ring lives by the turn stages (``/dialog/turn_event``,
``core/ring_reflex.py``); silence comes from the ``Silence`` scene in ``/dialog/session`` (#3911, #3984).
#4354 A2: ``/voice/dialogue/state`` lost its only publisher with ``dialogue_node``; led_node no longer
subscribes to it.

ROS2 stubs come from this directory's conftest; ``usb`` (pyusb) is stubbed
here because the ReSpeaker ring is hardware.
"""

import sys
import types
from unittest.mock import MagicMock

_usb = types.ModuleType("usb")
_usb.core = MagicMock()
_usb.util = MagicMock()
_usb.core.find.return_value = None  # no ReSpeaker attached
sys.modules.setdefault("usb", _usb)
sys.modules.setdefault("usb.core", _usb.core)
sys.modules.setdefault("usb.util", _usb.util)

from rob_box_voice import led_node  # noqa: E402
from rob_box_voice.core.ring_reflex import SILENCE_FRAMES  # noqa: E402

DIALOGUE_STATE_TOPIC = "/voice/dialogue/state"
TURN_EVENT_TOPIC = "/dialog/turn_event"


def _make_node():
    node = led_node.LEDNode()
    node.pixel_ring = MagicMock()
    node.reflex._ring = node.pixel_ring
    return node


def test_subscribes_to_neither_v1_dialogue_state_nor_legacy_name():
    node = _make_node()
    topics = [topic for topic, _ in node._subscribers]
    assert DIALOGUE_STATE_TOPIC not in topics
    assert "/voice/state" not in topics
    assert "/dialog/session" in topics


def test_ring_follows_turn_events_not_the_removed_animation_topic():
    node = _make_node()
    topics = [topic for topic, _ in node._subscribers]
    assert TURN_EVENT_TOPIC in topics and "/voice/animation/request" not in topics
    sub = [fake for topic, fake in node._subscribers if topic == TURN_EVENT_TOPIC][0]
    assert sub.callback == node.turn_event_callback


def test_silence_scene_puts_the_ring_into_the_silence_animation_and_back():
    node = _make_node()
    node.auto_mode = True

    node.session_callback(types.SimpleNamespace(agent="personality", scenes=[types.SimpleNamespace(kind="Silence")]))
    node.pixel_ring.mono.assert_called_once_with(*SILENCE_FRAMES[0])   # #3984: своя анимация, не погасшее кольцо

    node.session_callback(types.SimpleNamespace(agent="personality", scenes=[]))
    node.pixel_ring.trace.assert_called()


def test_addressed_turn_event_lights_the_ring_and_answers_with_an_event():
    node = _make_node()
    node.pixel_ring.listen.return_value = True
    node.turn_event_callback(types.SimpleNamespace(turn_id="t1", stage="addressed", status="started"))
    node.pixel_ring.listen.assert_called_once()
    ((event,), _kw) = node._event_pub.publish.call_args
    assert (event.executor, event.turn_id, event.kind, event.reason) == ("ring", "t1", 0, "")


def test_auto_mode_off_ignores_stages():
    node = _make_node()
    node.auto_mode = False
    node.turn_event_callback(types.SimpleNamespace(turn_id="t1", stage="addressed", status="started"))
    node.pixel_ring.listen.assert_not_called()
    node._event_pub.publish.assert_not_called()
