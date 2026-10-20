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
            '/audio/vad', '/voice/music/state', '/voice/animation/request', '/voice/expression/request',
            '~/scene'} <= topics
    assert sorted(p for p, _ in node.timers) == [0.04, 1.0]
