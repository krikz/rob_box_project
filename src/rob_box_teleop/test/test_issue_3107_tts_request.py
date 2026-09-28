"""Issue #3107: joystick voice feedback goes to tts_node's real input.

The node used to publish plain text on ``/tts/speak``, which has no
subscriber anywhere (live graph, run 36422274238). tts_node listens on
``/voice/tts/request`` and drops chunks that are not JSON with an ``ssml``
field, so both the topic and the payload have to match.
"""

import json

from rob_box_teleop.joystick_control_node import JoystickControlNode
from rob_box_teleop.joystick_logic import TTS_REQUEST_TOPIC, build_tts_request


def test_tts_request_topic_is_tts_node_input():
    assert TTS_REQUEST_TOPIC == "/voice/tts/request"


def test_build_tts_request_is_tts_node_contract():
    payload = json.loads(build_tts_request("Моторы отключены"))
    assert payload["ssml"] == "<speak>Моторы отключены</speak>"
    # "speakers" is Sink.SPEAKERS; tts_node canonicalises it to "speaker".
    assert payload["sink"] == "speakers"
    assert payload["priority"] == "normal"
    assert payload["emotion"] == "neutral"


def test_build_tts_request_escapes_xml():
    payload = json.loads(build_tts_request("a & b < c > d"))
    assert payload["ssml"] == "<speak>a &amp; b &lt; c &gt; d</speak>"


def test_build_tts_request_empty_text():
    assert json.loads(build_tts_request(""))["ssml"] == "<speak></speak>"


def test_node_speaks_on_voice_tts_request(node_params):
    node = JoystickControlNode()
    topics = [topic for topic, _ in node.publishers]
    assert "/tts/speak" not in topics
    _, tts_pub = [p for p in node.publishers if p[0] == "/voice/tts/request"][0]

    node.speak("Моторы активированы, готов к езде")

    assert len(tts_pub.messages) == 1
    payload = json.loads(tts_pub.messages[-1].data)
    assert payload["ssml"] == "<speak>Моторы активированы, готов к езде</speak>"
