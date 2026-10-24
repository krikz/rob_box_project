"""face_node: panels on /panel_image (one publisher, frame_id = group, only changed panels)."""

from __future__ import annotations

import types

import pytest

from rob_box_animations.face.face import GROUP_SIZE
from rob_box_animations.face.scene import Scene
from rob_box_animations.face.states import SCENES
from rob_box_animations.face_node import PanelPublisher


class _FakeNode:
    def __init__(self):
        self.pubs = []

    def create_publisher(self, msg_type, topic, qos):
        pub = types.SimpleNamespace(topic=topic, sent=[])
        pub.publish = pub.sent.append
        self.pubs.append(pub)
        return pub

    def get_clock(self):
        try:    # real sensor_msgs/Image (CI) insists on builtin_interfaces/Time
            from builtin_interfaces.msg import Time
        except ImportError:
            Time = type(None)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=Time))


def test_one_publisher_all_groups_first_then_only_changes():
    node = _FakeNode()
    panels = PanelPublisher(node, full_refresh_s=1.0)
    scene = Scene(SCENES['sleep'])
    panels.publish(scene.frame(0.0), 100.0)
    assert [p.topic for p in node.pubs] == ['/panel_image']
    first = node.pubs[0].sent
    assert sorted(m.header.frame_id for m in first) == sorted(GROUP_SIZE)
    for m in first:
        w, h = GROUP_SIZE[m.header.frame_id]
        assert (m.width, m.height, m.encoding, m.step, len(m.data)) == (w, h, 'rgb8', w * 3, w * h * 3)
    n = len(first)
    panels.publish(scene.frame(0.0), 100.04)            # same picture -> nothing sent
    assert len(first) == n
    panels.publish(scene.frame(0.0), 101.1)             # once a second everything again
    assert len(first) == 2 * n


def test_node_wires_inputs_and_timers():
    from rclpy.node import Node
    if Node.__name__ != '_Node':
        pytest.skip('real rclpy installed: node wiring needs a ROS context')
    from rob_box_animations.face_node import FaceNode
    node = FaceNode()
    topics = {t for t, _ in node.subscriptions}
    assert {'/voice/speech/envelope', '/voice/speech/event', '/voice/tts/state', '/audio/direction',
            '/audio/vad', '/voice/music/state', '/voice/expression/request',
            '~/scene'} <= topics
    assert '/voice/animation/request' not in topics     # ADR-0150 PR-3b: v1-топик снят, лицо слушает ExpressionRequest
    assert sorted(p for p, _ in node.timers) == [0.04, 1.0]


def _expression(**fields):
    base = dict(turn_id='t1', epoch=1, agent='personality', emotion='', gesture='', earcon='', ttl_ms=0, face_json='')
    return types.SimpleNamespace(**{**base, **fields})


def _face_node():
    from rclpy.node import Node
    if Node.__name__ != '_Node':
        pytest.skip('real rclpy installed: node wiring needs a ROS context')
    from rob_box_animations.face_node import FaceNode
    return FaceNode()


def test_expression_request_is_answered_with_started_for_a_known_scene():
    node = _face_node()
    node._on_expression(_expression(emotion='listening', ttl_ms=8000))
    (ev,) = node._event_pub.sent
    assert (ev.executor, ev.turn_id, ev.kind, ev.reason) == ('face', 't1', 0, '')
    assert node.director._request[0] == 'listening'


def test_unknown_scene_is_answered_with_skipped_and_reason_not_with_silence():
    node = _face_node()
    node._on_expression(_expression(emotion='moonwalk'))
    (ev,) = node._event_pub.sent
    assert (ev.kind, ev.reason) == (1, 'unknown_scene')


def test_gesture_wins_over_emotion_and_earcon_only_request_gets_no_face_answer():
    node = _face_node()
    node._on_expression(_expression(emotion='happy', gesture='wink'))
    assert node.director._request[0] == 'wink'
    node._on_expression(_expression(earcon='heard'))
    assert len(node._event_pub.sent) == 1


def test_session_speaker_azimuth_holds_the_gaze_over_raw_doa():
    """#3695: собеседник из /dialog/session (азимут его фразы) держит взгляд; сырой DOA в это время его не уводит."""
    import math

    node = _face_node()
    assert '/dialog/session' in {t for t, _ in node.subscriptions}
    node._on_session(types.SimpleNamespace(speaker_azimuth_deg=90.0))
    looked = node.director.gaze._speaker
    node._on_direction(types.SimpleNamespace(data=270))
    assert node.director.gaze._speaker == looked
    node._on_session(types.SimpleNamespace(speaker_azimuth_deg=math.nan))  # собеседник без азимута — взгляд не трогаем
    assert node.director.gaze._speaker == looked


def test_silence_scene_of_the_session_holds_the_taped_mouth_and_reflexes_are_answered_skipped():
    """#3984: сцена Silence в /dialog/session -> рот заклеен; рефлекс стадии (listening) лицо не меняет; снятие - покой."""
    node = _face_node()
    node._on_expression(_expression(emotion='listening', ttl_ms=8000))
    assert node.director._request[0] == 'listening'
    ns = types.SimpleNamespace
    nan = float('nan')
    node._on_session(ns(agent='personality', scenes=[ns(kind='Silence')], speaker_azimuth_deg=nan))
    assert node.director.silenced and node.director._request is None
    node._event_pub.sent.clear()
    node._on_expression(_expression(emotion='listening', ttl_ms=8000, turn_id='t2'))
    (ev,) = node._event_pub.sent
    assert (ev.turn_id, ev.kind, ev.reason) == ('t2', 1, 'silenced')
    assert node.director._request is None
    node._on_session(ns(agent='telegram', scenes=[], speaker_azimuth_deg=nan))   # чужой агент тишину не снимает
    assert node.director.silenced
    node._on_session(ns(agent='personality', scenes=[], speaker_azimuth_deg=nan))
    assert not node.director.silenced
    node._on_expression(_expression(emotion='listening', ttl_ms=8000))
    assert node.director._request[0] == 'listening'              # обычные рефлексы вернулись
