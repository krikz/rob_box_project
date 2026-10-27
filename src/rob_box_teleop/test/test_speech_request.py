"""PR-15 part 15-0d (#3711, D8): the joystick speaks through ``speech_client``.

Before this part the node wrote a JSON payload (``ssml``/``sink``/``priority``/
``emotion``) to ``/voice/tts/request`` (issue #3107) and the teleop image had to
reproduce the ``Utterance`` contract by hand. Now the node publishes a typed
``SpeechRequest`` on ``/voice/speech/request`` like every live v2 writer, and
``/voice/tts/request`` has no writer here.
"""

import types

import pytest

from rob_box_core import speech_client
from rob_box_teleop.joystick_control_node import JoystickControlNode

ARMED = "Моторы активированы, готов к езде"
DISARMED = "Моторы отключены"
ARM_THEN_DISARM = [f"<speak>{ARMED}</speak>", f"<speak>{DISARMED}</speak>"]
LEGACY_TOPICS = ("/voice/tts/request", "/tts/speak")


def _publisher(node, topic):
    found = [pub for name, pub in node.publishers if name == topic]
    assert len(found) == 1, (topic, [name for name, _ in node.publishers])
    return found[0]


def _speech(node):
    return _publisher(node, speech_client.SPEECH_REQUEST_TOPIC).messages


def _joy(*, armed):
    """Joy with 16 centred axes; the ARM / enable switch is button 4 (SBUS) or 0 (HID)."""
    buttons = [0] * 16
    buttons[4] = buttons[0] = 1 if armed else 0
    return types.SimpleNamespace(axes=[0.0] * 16, buttons=buttons)


def test_node_has_no_writer_on_the_legacy_tts_topics(node_params):
    node = JoystickControlNode()
    topics = [topic for topic, _ in node.publishers]
    for legacy in LEGACY_TOPICS:
        assert legacy not in topics


def test_speech_publisher_exists_before_the_first_phrase(node_params):
    """prepare() in __init__: the "joystick connected" phrase must not be lost to writer matching."""
    node = JoystickControlNode()
    assert _speech(node) == []


def test_speak_publishes_one_speech_request_from_the_system_author(node_params):
    node = JoystickControlNode()

    node.speak(ARMED)

    (req,) = _speech(node)
    assert req.text == f"<speak>{ARMED}</speak>"
    assert req.agent == speech_client.SYSTEM_AGENT == "system"
    # same values the old JSON carried: speakers, normal priority, neutral emotion
    assert req.sink == "speakers"
    assert req.priority == speech_client.PRIORITY_NORMAL
    assert req.emotion == "neutral"
    # the joystick sets no voice or language of its own: the robot's current voice speaks
    assert (req.voice, req.language) == ("", "")
    # outside any dialog session: no turn, epoch 0, plays at once
    assert (req.turn_id, req.epoch, req.at) == ("", 0, 0.0)
    assert req.commit is True and req.interruptible is True
    assert len(req.speech_id) == 36


def test_speak_escapes_xml_in_the_text(node_params):
    node = JoystickControlNode()
    node.speak("a & b < c > d")
    assert _speech(node)[-1].text == "<speak>a &amp; b &lt; c &gt; d</speak>"


def test_speak_gives_every_phrase_its_own_speech_id(node_params):
    node = JoystickControlNode()
    node.speak(ARMED)
    node.speak(ARMED)
    first, second = _speech(node)
    assert first.speech_id != second.speech_id


def test_sbus_arm_and_disarm_are_spoken(node_params):
    node = JoystickControlNode()

    node.joy_callback_sbus(_joy(armed=True))
    node.joy_callback_sbus(_joy(armed=True))  # still armed: no repeat
    node.joy_callback_sbus(_joy(armed=False))

    assert [r.text for r in _speech(node)] == ARM_THEN_DISARM


def test_hid_enable_button_press_and_release_are_spoken(node_params):
    node = JoystickControlNode()

    node.joy_callback(_joy(armed=True))
    node.joy_callback(_joy(armed=False))

    assert [r.text for r in _speech(node)] == ARM_THEN_DISARM


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_voice_feedback_off_keeps_the_robot_silent(node_params, callback):
    node_params["enable_voice_feedback"] = False
    node = JoystickControlNode()

    getattr(node, callback)(_joy(armed=True))
    getattr(node, callback)(_joy(armed=False))

    assert _speech(node) == []
