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


class _Clock:
    """Injectable monotonic clock: the ARM announcement is debounced by time (issue #4003)."""

    def __init__(self, node):
        self.t = 100.0
        node._monotonic = lambda: self.t

    def feed(self, node, callback, *, armed, for_s):
        """Deliver ``armed`` samples every 50 ms (the SBUS timer is 20 Hz) for ``for_s`` seconds."""
        for _ in range(max(1, round(for_s / 0.05))):
            getattr(node, callback)(_joy(armed=armed))
            self.t += 0.05


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


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_arm_and_disarm_are_spoken_once_each_after_the_hold(node_params, callback):
    node = JoystickControlNode()
    clock = _Clock(node)

    clock.feed(node, callback, armed=True, for_s=1.0)  # still armed: no repeat
    clock.feed(node, callback, armed=False, for_s=6.0)

    assert [r.text for r in _speech(node)] == ARM_THEN_DISARM


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_arm_chatter_is_not_spoken(node_params, callback):
    """Robot log of 2026-10-24: LOW for ~50 ms, then HIGH again, every few seconds."""
    node = JoystickControlNode()
    clock = _Clock(node)
    clock.feed(node, callback, armed=True, for_s=1.0)
    spoken_after_arming = len(_speech(node))

    for _ in range(10):
        clock.feed(node, callback, armed=False, for_s=0.05)
        clock.feed(node, callback, armed=True, for_s=2.0)

    assert len(_speech(node)) == spoken_after_arming == 1


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_driving_still_reacts_to_the_raw_switch_without_delay(node_params, callback):
    """Debounce is for speech only: the brake goes out on the first LOW sample, the phrase much later."""
    node = JoystickControlNode()
    clock = _Clock(node)
    cmd_vel = _publisher(node, "cmd_vel_joy").messages
    clock.feed(node, callback, armed=True, for_s=1.0)
    sent_before = len(cmd_vel)
    spoken_before = len(_speech(node))

    getattr(node, callback)(_joy(armed=False))  # one LOW sample

    assert node.was_enabled is False
    assert len(cmd_vel) == sent_before + 1
    assert (cmd_vel[-1].linear.x, cmd_vel[-1].angular.z) == (0.0, 0.0)
    assert len(_speech(node)) == spoken_before


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_voice_feedback_off_keeps_the_robot_silent(node_params, callback):
    node_params["enable_voice_feedback"] = False
    node = JoystickControlNode()
    clock = _Clock(node)

    clock.feed(node, callback, armed=True, for_s=1.0)
    clock.feed(node, callback, armed=False, for_s=6.0)

    assert _speech(node) == []


def test_announce_timing_comes_from_node_parameters(node_params):
    node_params["arm_announce_hold_s"] = 2.0
    node_params["arm_announce_min_interval_s"] = 9.0
    node = JoystickControlNode()
    assert (node.arm_announcer.hold_s, node.arm_announcer.min_interval_s) == (2.0, 9.0)


# -- a failed phrase must not break driving -------------------------------------------------------


def _say_fails(monkeypatch):
    """Make the speech client raise, as it would with a shut-down context or a bad Utterance."""

    def boom(*args, **kwargs):
        raise RuntimeError("speech client down")

    monkeypatch.setattr(speech_client, "say", boom)


def test_speak_failure_is_logged_and_does_not_propagate(node_params, monkeypatch):
    node = JoystickControlNode()
    _say_fails(monkeypatch)

    node.speak(ARMED)  # must not raise

    (call,) = node.get_logger().error.call_args_list
    assert ARMED in call.args[0] and "speech client down" in call.args[0]
    assert _speech(node) == []


@pytest.mark.parametrize("callback", ["joy_callback_sbus", "joy_callback"])
def test_disarm_still_sends_the_brake_when_the_phrase_fails(node_params, monkeypatch, callback):
    """The brake goes out on the LOW sample, the phrase later: a speech failure must not touch the stop."""
    node = JoystickControlNode()
    clock = _Clock(node)
    cmd_vel = _publisher(node, "cmd_vel_joy").messages
    clock.feed(node, callback, armed=True, for_s=1.0)
    _say_fails(monkeypatch)
    sent_before_disarm = len(cmd_vel)

    getattr(node, callback)(_joy(armed=False))
    clock.t += 0.05
    clock.feed(node, callback, armed=False, for_s=6.0)  # the phrase fails when it is finally due

    assert node.was_enabled is False
    assert len(cmd_vel) == sent_before_disarm + 1
    assert (cmd_vel[-1].linear.x, cmd_vel[-1].angular.z) == (0.0, 0.0)
    (call,) = node.get_logger().error.call_args_list  # the phrase was due, failed and was only logged
    assert DISARMED in call.args[0]


class _Stop(BaseException):
    """Ends the reader's endless ``while True`` (BaseException: its ``except Exception`` lets it through)."""


class _FakeSerialModule:
    """pyserial stand-in: the first port opens already closed (no frames), the second open ends the test."""

    EIGHTBITS, PARITY_EVEN, STOPBITS_TWO = 8, "E", 2

    class SerialException(Exception):
        pass

    def __init__(self):
        self.opened = 0

    def Serial(self, **kwargs):
        self.opened += 1
        if self.opened > 1:
            raise _Stop
        return types.SimpleNamespace(is_open=False, close=lambda: None)


def test_sbus_reader_survives_a_failed_connect_phrase(node_params, monkeypatch):
    """The "joystick connected" phrase is spoken from the serial thread: its failure must not trigger the 5 s reconnect."""
    from rob_box_teleop import joystick_control_node as module

    node = JoystickControlNode()
    fake_serial, sleeps = _FakeSerialModule(), []
    monkeypatch.setattr(module, "serial", fake_serial, raising=False)
    monkeypatch.setattr(module, "time", types.SimpleNamespace(sleep=sleeps.append, monotonic=lambda: 0.0))
    _say_fails(monkeypatch)

    with pytest.raises(_Stop):
        node._run_sbus_reader()

    assert node.device_connected is True
    assert sleeps == []  # a speech failure did not send the reader into "SBUS reader error -> sleep(5)"
    assert fake_serial.opened == 2  # the loop went on to the next iteration by itself
