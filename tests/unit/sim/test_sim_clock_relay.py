"""Прореживание /clock для HIL (rob_box_sim.clock_relay): решение — чистая функция, rclpy не нужен.

Эпик #4371, план docs/research/2026-11-04-hil-robot-on-gazebo-data.md §8.5 п.6.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_sim import clock_relay as cr

MS = 1_000_000
LAUNCH = Path(__file__).resolve().parents[3] / "src" / "rob_box_sim" / "launch" / "sim.launch.py"


def _run(stamps_ns, hz):
    period, last, out = cr.period_ns(hz), None, []
    for s in stamps_ns:
        if cr.should_publish(s, last, period):
            last = s
            out.append(s)
    return out


def test_period_100hz_is_10ms():
    assert cr.period_ns(100) == 10 * MS


@pytest.mark.parametrize("bad", [0, -1, float("nan")])
def test_period_rejects_nonpositive(bad):
    with pytest.raises(ValueError):
        cr.period_ns(bad)


def test_1khz_in_gives_100hz_out():
    out = _run([i * MS for i in range(1000)], 100)  # 1 с sim-времени по 1 мс
    assert len(out) == 100
    assert all(b - a == 10 * MS for a, b in zip(out, out[1:]))


def test_first_sample_always_published():
    assert cr.should_publish(5 * MS, None, 10 * MS)


def test_time_going_back_is_published_after_sim_reset():
    assert cr.should_publish(1 * MS, 900 * MS, 10 * MS)


def test_throttle_is_by_sim_time_not_by_message_count():
    out = _run([i * 4 * MS for i in range(30)], 100)  # шаг 4 мс → каждый третий
    assert all(b - a >= 10 * MS for a, b in zip(out, out[1:]))
    assert len(out) == 10


def test_stamps_never_invented():
    stamps = [i * MS for i in range(500)]
    assert set(_run(stamps, 100)) <= set(stamps)


def test_launch_hil_routes_raw_clock_away_from_robot():
    # В HIL мост не имеет права писать в /clock напрямую: только через реле.
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'remappings=[("/clock", CLOCK_RAW_TOPIC)]' in text
    assert "rob_box_sim.clock_relay" in text
    assert cr.RAW_TOPIC != cr.OUT_TOPIC
    # обычный мост (без HIL) — только при hil != true
    assert re.search(r"condition=UnlessCondition\(hil\)", text)
