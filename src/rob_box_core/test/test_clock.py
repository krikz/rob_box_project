"""Tests for Clock / SystemClock / MockClock."""

from __future__ import annotations

import asyncio
import time

import pytest

from rob_box_core.clock import MockClock, SystemClock


def test_mock_clock_starts_at_zero():
    c = MockClock()
    assert c.now() == 0.0
    assert c.monotonic() == 0.0


def test_mock_clock_advance_moves_both_clocks():
    c = MockClock(start=100.0)
    c.advance(5.0)
    assert c.now() == 105.0
    assert c.monotonic() == 105.0


def test_mock_clock_advance_rejects_negative():
    c = MockClock()
    with pytest.raises(ValueError):
        c.advance(-1.0)


def test_mock_clock_set_now_jumps_both_clocks():
    c = MockClock()
    c.set_now(42.5)
    assert c.now() == 42.5
    assert c.monotonic() == 42.5


def test_mock_clock_sleep_advances_clock_records_call():
    c = MockClock(start=0.0)

    async def go():
        await c.sleep(2.0)
        await c.sleep(0.5)

    asyncio.run(go())
    assert c.sleeps == [2.0, 0.5]
    assert c.now() == 2.5
    assert c.monotonic() == 2.5


def test_mock_clock_sleep_zero_or_negative_is_noop():
    c = MockClock(start=10.0)

    async def go():
        await c.sleep(0)
        await c.sleep(-3.0)

    asyncio.run(go())
    assert c.now() == 10.0
    assert c.sleeps == [0, -3.0]


def test_system_clock_now_is_close_to_time_time():
    c = SystemClock()
    before = time.time()
    now = c.now()
    after = time.time()
    assert before <= now <= after


def test_system_clock_sleep_zero_returns_immediately():
    c = SystemClock()

    async def go():
        await c.sleep(0)

    asyncio.run(go())  # must not raise, must not block
