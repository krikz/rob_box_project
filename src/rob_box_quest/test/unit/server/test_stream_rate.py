"""Юнит-тесты ограничителя частоты кадров (issue #3150)."""

import math
import threading

import pytest

from rob_box_quest.server.stream_rate import (
    MAX_HZ_CEILING,
    StreamRateLimiter,
    parse_max_hz,
)


@pytest.mark.parametrize(
    "raw,expected",
    [
        (5, 5.0),
        (2.5, 2.5),
        (MAX_HZ_CEILING, MAX_HZ_CEILING),
        (None, None),
        (0, None),
        (-1, None),
        (math.nan, None),
        (math.inf, None),
        (121, None),
        (True, None),
        (False, None),
        ("5", None),
        ([5], None),
    ],
)
def test_parse_max_hz(raw, expected):
    assert parse_max_hz(raw) == expected


def test_no_limit_always_sends():
    lim = StreamRateLimiter()
    for i in range(10):
        assert lim.offer("camera_rear", i, now=0.0).send_now
    assert lim.limit_hz("camera_rear") is None


def test_first_frame_sent_early_frame_deferred():
    lim = StreamRateLimiter()
    lim.set_limit("lidar_2d", 5.0)  # интервал 0.2 с
    assert lim.offer("lidar_2d", "a", now=10.0).send_now
    res = lim.offer("lidar_2d", "b", now=10.05)
    assert not res.send_now
    assert res.flush_in_s == pytest.approx(0.15)


def test_newer_pending_replaces_older_and_counts_drop():
    lim = StreamRateLimiter()
    lim.set_limit("lidar_2d", 5.0)
    lim.offer("lidar_2d", "a", now=0.0)
    first = lim.offer("lidar_2d", "b", now=0.05)
    second = lim.offer("lidar_2d", "c", now=0.10)
    assert first.flush_in_s is not None
    # flush уже запланирован — второй раз не просим
    assert second.flush_in_s is None and not second.send_now
    item, again = lim.flush("lidar_2d", now=0.2)
    assert item == "c" and again is None
    assert lim.stats("lidar_2d") == (2, 1)


def test_last_event_is_never_lost():
    """Событийный поток: одно раннее событие → доставляется по flush."""
    lim = StreamRateLimiter()
    lim.set_limit("voice_state", 1.0)
    lim.offer("voice_state", "speaking", now=0.0)
    res = lim.offer("voice_state", "idle", now=0.1)
    assert res.flush_in_s == pytest.approx(0.9)
    assert lim.flush("voice_state", now=1.0) == ("idle", None)
    # слот пуст — повторный flush ничего не шлёт
    assert lim.flush("voice_state", now=2.0) == (None, None)


def test_flush_too_early_asks_reschedule():
    lim = StreamRateLimiter()
    lim.set_limit("map_2d", 10.0)
    lim.offer("map_2d", "a", now=0.0)
    lim.offer("map_2d", "b", now=0.01)
    lim.set_limit("map_2d", 1.0)  # лимит ужесточили до flush
    item, again = lim.flush("map_2d", now=0.1)
    assert item is None and again == pytest.approx(0.9)
    assert lim.flush("map_2d", now=1.0) == ("b", None)


def test_frame_after_interval_sent_directly():
    lim = StreamRateLimiter()
    lim.set_limit("camera_rear", 5.0)
    assert lim.offer("camera_rear", 1, now=0.0).send_now
    assert lim.offer("camera_rear", 2, now=0.25).send_now


def test_remove_drops_pending():
    lim = StreamRateLimiter()
    lim.set_limit("camera_rear", 5.0)
    lim.offer("camera_rear", 1, now=0.0)
    lim.offer("camera_rear", 2, now=0.01)
    lim.remove("camera_rear")
    assert lim.flush("camera_rear", now=1.0) == (None, None)
    assert lim.offer("camera_rear", 3, now=1.0).send_now


def test_throttles_steady_stream_to_max_hz():
    """30 fps на входе, лимит 5 Гц → за 2 с уходит ~10 кадров."""
    lim = StreamRateLimiter()
    lim.set_limit("camera_rear", 5.0)
    sent = 0
    flush_at = None
    for i in range(60):
        now = i / 30.0
        if flush_at is not None and now >= flush_at:
            item, _ = lim.flush("camera_rear", now=flush_at)
            sent += item is not None
            flush_at = None
        res = lim.offer("camera_rear", i, now=now)
        sent += res.send_now
        if res.flush_in_s is not None:
            flush_at = now + res.flush_in_s
    assert 9 <= sent <= 11


def test_thread_safe_offers():
    lim = StreamRateLimiter()
    lim.set_limit("lidar_2d", 5.0)
    results = []

    def worker():
        for _ in range(500):
            results.append(lim.offer("lidar_2d", 0, now=0.0).send_now)

    threads = [threading.Thread(target=worker) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    # в один и тот же момент — ровно один кадр уходит сразу
    assert sum(results) == 1
