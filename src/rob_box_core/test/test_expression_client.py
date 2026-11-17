"""ADR-0150 PR-3b (#3690): клиент экспрессии и событий хода ``expression_client``."""

from __future__ import annotations

import json
import types

import pytest

from rob_box_core import expression_client as ec
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
TIME = type(M.TurnEvent().stamp)


class _Pub:
    def __init__(self, msg_type, topic):
        self.msg_type, self.topic, self.sent = msg_type, topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.pubs, self.subs = [], []

    def create_publisher(self, msg_type, topic, depth):
        pub = _Pub(msg_type, topic)
        self.pubs.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, depth):
        self.subs.append((msg_type, topic, callback))

    def get_clock(self):
        stamp = TIME(sec=9, nanosec=1)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


def test_express_fills_request_and_none_means_do_not_change():
    node = _Node()
    req = ec.express(node, turn_id="t1", epoch=4, agent="personality", emotion="happy", earcon="done", ttl_ms=3000)
    (pub,) = node.pubs
    assert pub.topic == "/voice/expression/request" and pub.msg_type is M.ExpressionRequest
    assert pub.sent == [req]
    assert (req.turn_id, req.epoch, req.agent) == ("t1", 4, "personality")
    assert (req.emotion, req.gesture, req.earcon, req.ttl_ms, req.face_json) == ("happy", "", "done", 3000, "")


def test_face_scene_goes_as_sorted_json_and_ttl_is_clamped():
    node = _Node()
    req = ec.express(node, turn_id="", epoch=0, agent="operator", face={"mouth": "grin", "eyes": "wide"},
                     ttl_ms=10 ** 9)
    assert json.loads(req.face_json) == {"eyes": "wide", "mouth": "grin"}
    assert req.face_json.index("eyes") < req.face_json.index("mouth")
    assert req.ttl_ms == ec.TTL_MAX_MS


def test_one_publisher_per_kind_and_prepare_creates_them_up_front():
    node = _Node()
    ec.prepare(node)
    assert sorted(p.topic for p in node.pubs) == ["/dialog/turn_event", "/voice/expression/request"]
    ec.express(node, turn_id="a", epoch=1, agent="personality", earcon="heard")
    ec.express(node, turn_id="b", epoch=1, agent="personality", earcon="fail")
    ec.turn_event(node, turn_id="a", epoch=1, agent="personality", stage="addressed", status="started")
    assert len(node.pubs) == 2
    by_topic = {p.topic: len(p.sent) for p in node.pubs}
    assert by_topic == {"/voice/expression/request": 2, "/dialog/turn_event": 1}


def test_turn_event_carries_stage_status_and_stamp():
    node = _Node()
    ev = ec.turn_event(node, turn_id="t", epoch=2, agent="personality", stage="failed", status="failed",
                       reason="tool_error", llm_calls=300, latency_ms=12, source="respeaker")
    assert (ev.stage, ev.status, ev.reason, ev.source) == ("failed", "failed", "tool_error", "respeaker")
    assert (ev.llm_calls, ev.latency_ms) == (255, 12)
    assert (ev.stamp.sec, ev.stamp.nanosec) == (9, 1)


def test_unknown_agent_and_status_are_rejected_not_published():
    node = _Node()
    with pytest.raises(ValueError):
        ec.express(node, turn_id="t", epoch=0, agent="robot")
    with pytest.raises(ValueError):
        ec.turn_event(node, turn_id="t", epoch=0, agent="personality", stage="done", status="ok")
    assert all(not p.sent for p in node.pubs)


def test_watch_events_subscribes_only_to_requested_kinds():
    node = _Node()
    ec.watch_events(node, on_expression=print)
    assert [(t.__name__, topic) for t, topic, _ in node.subs] == [("ExpressionEvent", "/voice/expression/event")]
    ec.watch_events(node, on_sound=print)
    assert node.subs[-1][1] == "/voice/sound/event"
