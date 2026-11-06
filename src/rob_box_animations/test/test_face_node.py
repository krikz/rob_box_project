"""face_node: panels on /panel_image (one publisher, one Image per frame, frame_id = changed groups)."""

from __future__ import annotations

import types

import pytest

from rob_box_animations.face.layout import GROUP_SIZE
from rob_box_animations.face.scene import FaceContext, Scene
from rob_box_animations.face.states import SCENES
from rob_box_animations.face_node import FULL_REFRESH_S, PanelPublisher


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


def _groups_of(msg):
    return msg.header.frame_id.split(',')


def test_one_publisher_one_message_per_frame_then_only_changes():
    node = _FakeNode()
    panels = PanelPublisher(node, full_refresh_s=1.0)
    scene = Scene(SCENES['sleep'])
    panels.publish(scene.frame(0.0), 100.0)
    assert [p.topic for p in node.pubs] == ['/panel_image']
    sent = node.pubs[0].sent
    assert len(sent) == 1                                # the whole first frame is ONE message
    (m,) = sent
    assert sorted(_groups_of(m)) == sorted(GROUP_SIZE)
    pixels = sum(w * h for w, h in GROUP_SIZE.values())
    assert (m.width, m.height, m.encoding, m.step, len(m.data)) == (pixels, 1, 'rgb8', pixels * 3, pixels * 3)
    panels.publish(scene.frame(0.0), 100.04)            # same picture -> nothing sent
    assert len(sent) == 1
    panels.publish(scene.frame(0.0), 101.1)             # once a second everything again
    assert len(sent) == 2 and sorted(_groups_of(sent[1])) == sorted(GROUP_SIZE)


def test_message_data_is_the_groups_wire_images_back_to_back():
    from rob_box_animations.face.face import to_wire
    from rob_box_animations.face.layout import GROUP_OF
    node = _FakeNode()
    panels = PanelPublisher(node)
    frame = Scene(SCENES['music_shades']).frame(0.3, FaceContext(bpm=118))
    panels.publish(frame, 100.0)
    (m,) = node.pubs[0].sent
    want = {GROUP_OF[p]: to_wire(a).tobytes() for p, a in frame.panels().items()}
    off = 0
    for g in _groups_of(m):
        n = GROUP_SIZE[g][0] * GROUP_SIZE[g][1] * 3
        assert bytes(m.data[off:off + n]) == want[g], g
        off += n
    assert off == len(m.data)


def test_static_scene_publishes_nothing_after_the_first_frame():
    """#3733: a still face costs only the first frame and the slow pulse; 10 s at 25 fps."""
    node = _FakeNode()
    panels = PanelPublisher(node)
    frame = Scene(SCENES['sleep']).frame(0.0)
    for i in range(250):
        panels.publish(frame, 100.0 + i / 25.0)
    seconds = 250 / 25.0
    assert len(node.pubs[0].sent) <= 1 + seconds / FULL_REFRESH_S     # first frame + one pulse per period


def test_animated_scene_is_one_message_per_frame():
    node = _FakeNode()
    panels = PanelPublisher(node, full_refresh_s=1e9)
    scene = Scene(SCENES['music_shades'])
    ctx = FaceContext(bpm=118)
    first = scene.frame(0.0, ctx)
    panels.publish(first, 100.0)
    before = len(node.pubs[0].sent)
    changed, prev = 0, first
    for i in range(1, 51):
        f = scene.frame(i / 25.0, ctx)
        panels.publish(f, 100.0 + i / 25.0)
        changed += any(a.tobytes() != b.tobytes() for a, b in zip(f.panels().values(), prev.panels().values()))
        prev = f
    assert changed > 0
    assert len(node.pubs[0].sent) - before == changed    # one message per changed frame, none for a repeat
    assert panels.messages == len(node.pubs[0].sent)


def test_node_wires_inputs_and_timers():
    from rclpy.node import Node
    if Node.__name__ != '_Node':
        pytest.skip('real rclpy installed: node wiring needs a ROS context')
    from rob_box_animations.face_node import FaceNode
    node = FaceNode()
    topics = {t for t, _ in node.subscriptions}
    assert {'/voice/speech/envelope', '/voice/speech/event', '/audio/direction',
            '/audio/vad', '/voice/music/state', '/voice/expression/request',
            '~/scene'} <= topics
    assert '/voice/animation/request' not in topics     # ADR-0150 PR-3b: v1-топик снят, лицо слушает ExpressionRequest
    assert '/voice/tts/state' not in topics             # PR-15 15-0b: «говорит» — по SpeechEvent
    assert sorted(p for p, _ in node.timers) == [0.04, 1.0]


def test_speech_event_through_the_subscription_makes_the_face_talk_and_cancel_shuts_it():
    """PR-15 15-0b (#3711): лицо говорит, пока звучит реплика (SpeechEvent started ... finished); cancelled — рот
    закрыт сразу. Событие идёт через ту же подписку /voice/speech/event, что на роботе."""
    import time

    from rob_box_core.msg_stub import build_dialog_msgs_module

    speech_event = build_dialog_msgs_module().SpeechEvent
    node = _face_node()
    on_event = dict(node.subscriptions)['/voice/speech/event']
    now = time.time()
    node.director.voice_seen(now)
    node.director._wake_until = now                      # wakeup start-up scene is over
    on_event(speech_event(kind=speech_event.KIND_STARTED, speech_id='s1', agent='personality'))
    assert node.director.choose(time.time())[0] == 'talk_jaw'
    on_event(speech_event(kind=speech_event.KIND_CANCELLED, speech_id='s1', agent='personality'))
    assert not node.director.speaking(time.time())


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
