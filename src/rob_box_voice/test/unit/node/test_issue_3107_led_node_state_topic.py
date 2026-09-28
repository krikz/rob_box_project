"""Issue #3107: led_node listens to the dialogue state topic that exists.

led_node subscribed to ``/voice/state`` since it was written (bf494be3), but
nothing on the robot publishes that name (live graph, run 36422274238).
dialogue_node publishes ``DialogueStateKind.name`` as ``std_msgs/String`` on
``/voice/dialogue/state`` (dialogue_node.py ``_state_pub``), the canonical
name per ADR-0027 #2 and ``arbiter_node.VOICE_DIALOGUE_STATE_TOPIC``.

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

DIALOGUE_STATE_TOPIC = "/voice/dialogue/state"


def _make_node():
    node = led_node.LEDNode()
    node.pixel_ring = MagicMock()
    return node


def test_topic_constant_is_dialogue_state():
    assert led_node.VOICE_STATE_TOPIC == DIALOGUE_STATE_TOPIC


def test_subscribes_to_dialogue_state_not_legacy_name():
    node = _make_node()
    topics = [topic for topic, _ in node._subscribers]
    assert DIALOGUE_STATE_TOPIC in topics
    assert "/voice/state" not in topics


def test_subscription_is_wired_to_state_callback():
    node = _make_node()
    sub = [fake for topic, fake in node._subscribers if topic == DIALOGUE_STATE_TOPIC][0]
    assert sub.callback == node.state_callback


def test_dialogue_node_payload_drives_ring():
    """dialogue_node publishes upper-case enum names; led_node lower-cases them."""
    node = _make_node()
    node.auto_mode = True

    node.state_callback(types.SimpleNamespace(data="LISTENING"))
    r, g, b = node.colors["listening"]
    node.pixel_ring.mono.assert_called_once_with(r, g, b)

    node.state_callback(types.SimpleNamespace(data="IDLE"))
    node.pixel_ring.trace.assert_called()
    assert node.current_mode == "idle"
