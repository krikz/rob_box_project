"""Tests for rob_box_telegram.camera_on_demand (подписка на камеры по запросу)."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_telegram.camera_cache import CameraCache
from rob_box_telegram.camera_on_demand import OnDemandCameraSubscriptions


class _FakeNode:
    def __init__(self):
        self.subs = []

    def create_subscription(self, msg_type, topic, callback, qos, callback_group=None):
        sub = SimpleNamespace(topic=topic, callback=callback)
        self.subs.append(sub)
        return sub

    def destroy_subscription(self, sub):
        self.subs.remove(sub)


def _make(linger_s=10.0):
    now = [100.0]
    node = _FakeNode()
    cache = CameraCache(ttl=5.0)
    subs = OnDemandCameraSubscriptions(node, cache, MagicMock(), MagicMock(), linger_s=linger_s, clock=lambda: now[0])
    return node, cache, subs, now


def test_no_subscription_before_request():
    node, _, subs, _ = _make()
    assert node.subs == []
    assert subs.active_topics == []


def test_request_subscribes_once_and_returns_frame():
    node, _, subs, _ = _make()
    timer = threading.Timer(0.05, lambda: node.subs[0].callback(SimpleNamespace(data=b"jpeg")))
    timer.start()
    assert subs.request("/cam", timeout_s=2.0) == b"jpeg"
    assert subs.request("/cam", timeout_s=0) == b"jpeg"
    assert [s.topic for s in node.subs] == ["/cam"]


def test_request_times_out_without_frames():
    node, _, subs, _ = _make()
    started = time.monotonic()
    assert subs.request("/cam", timeout_s=0.1) is None
    assert time.monotonic() - started >= 0.09
    assert len(node.subs) == 1


def test_reap_drops_idle_subscriptions_only():
    node, _, subs, now = _make(linger_s=10.0)
    subs.request("/old", timeout_s=0)
    now[0] += 8
    subs.request("/new", timeout_s=0)
    now[0] += 3
    subs.reap()
    assert [s.topic for s in node.subs] == ["/new"]
    assert subs.active_topics == ["/new"]


def test_request_extends_linger():
    node, _, subs, now = _make(linger_s=10.0)
    subs.request("/cam", timeout_s=0)
    now[0] += 8
    subs.request("/cam", timeout_s=0)
    now[0] += 8
    subs.reap()
    assert len(node.subs) == 1


def test_cache_wait_for_returns_existing_fresh_frame():
    cache = CameraCache(ttl=5.0)
    cache.update("/cam", b"x")
    assert cache.wait_for("/cam", timeout_s=0) == b"x"
