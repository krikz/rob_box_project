"""voice_state (0x1202) в шлем из контрактов диалога v2 (issue #3911).

``streams.voice_state.send_voice_state`` - единственный путь кадра: из ``watch`` (v2: TurnEvent/SpeechEvent/
SessionState) и из ``QuestNode._on_dialogue_state`` (v1, адаптер до PR-15). Без rclpy: ``quest_node_mod`` ставит
ROS-заглушки, узел - MagicMock.
Таблицы состояний и переходы - ``rob_box_core/test/test_voice_state.py``.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import msgpack

from conftest import quest_node_mod  # noqa: F401  — фикстура ставит ROS-stub
from rob_box_core import voice_state
from rob_box_quest.streams.voice_state import send_voice_state


def _host():
    """Узел без ``__init__``: настоящий трекер, перехват кадров моста."""
    host = MagicMock()
    host._voice = voice_state.VoiceStateTracker()
    host._last_voice_state = {"state": "idle", "detail": None}
    host.frames = []
    host.bridge.publish_frame.side_effect = lambda name, payload: host.frames.append((name, msgpack.unpackb(payload)))
    return host


def _string(data: str):
    msg = MagicMock()
    msg.data = data
    return msg


def _frames(host):
    return [(body["state"], body.get("detail")) for _name, body in host.frames]


def test_v2_state_change_becomes_voice_state_frame(quest_node_mod):
    host = _host()
    thinking = host._voice.on_turn_event(MagicMock(agent="personality", stage="thinking", status="started"))
    send_voice_state(host, thinking)
    (name, body), = host.frames
    assert name == "voice_state"
    assert body["state"] == "thinking" and "detail" not in body and isinstance(body["ts_ms"], int)
    assert host._last_voice_state == {"state": "thinking", "detail": None}


def test_v2_speech_and_silence_reach_the_headset(quest_node_mod):
    host = _host()
    send_voice_state(host, host._voice.on_speech_event(MagicMock(agent="personality", kind=2, speech_id="s")))
    scenes = [MagicMock(kind="Silence")]
    send_voice_state(host, host._voice.on_session(MagicMock(agent="personality", scenes=scenes)))
    # речь ещё идёт, но тишина важнее: шлем видит «mute», как и у v1
    assert _frames(host) == [("speaking", None), ("idle", "silenced")]


def test_v1_string_still_reaches_the_headset(quest_node_mod):
    """TEMP до PR-15: dialogue_node v1 пишет строку ``DIALOGUE`` - шлем видит ``speaking``."""
    host = _host()
    quest_node_mod.QuestNode._on_dialogue_state(host, _string("DIALOGUE"))
    quest_node_mod.QuestNode._on_dialogue_state(host, _string("SILENCED"))
    assert _frames(host) == [("speaking", None), ("idle", "silenced")]


def test_publish_failure_does_not_crash_the_subscription(quest_node_mod):
    host = _host()
    host.bridge.publish_frame.side_effect = RuntimeError("ws closed")
    quest_node_mod.QuestNode._on_dialogue_state(host, _string("LISTENING"))
    assert host._last_voice_state["state"] == "listening"
