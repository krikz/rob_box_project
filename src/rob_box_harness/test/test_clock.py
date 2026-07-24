"""Tests for the Clock port and its two implementations.

Covers:

* ``SystemClock`` is wall-clock-backed and non-blocking.
* ``MockClock`` advances on ``advance`` / ``set`` and reports the
  new value to ``now`` / ``monotonic``.
* ``MockClock.sleep`` yields control but advances virtual time
  without waiting.
* Negative ``sleep`` / ``advance`` are rejected.
* ``MockClock.set`` requires a timezone-aware ``datetime``.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone

import pytest

from rob_box_harness.clock import MockClock, SystemClock


def test_system_clock_now_is_utc() -> None:
    """``SystemClock.now()`` returns an aware UTC datetime."""
    clock = SystemClock()
    now = clock.now()
    assert now.tzinfo is not None
    assert now.tzinfo == timezone.utc


def test_system_clock_monotonic_is_float() -> None:
    """``SystemClock.monotonic()`` returns a non-negative float."""
    clock = SystemClock()
    m = clock.monotonic()
    assert isinstance(m, float)
    assert m >= 0


@pytest.mark.asyncio
async def test_system_clock_sleep() -> None:
    """``SystemClock.sleep`` actually waits the requested time."""
    clock = SystemClock()
    start = clock.monotonic()
    await clock.sleep(0.01)
    elapsed = clock.monotonic() - start
    assert elapsed >= 0.005  # half a sleep is fine; CI is noisy


@pytest.mark.asyncio
async def test_system_clock_negative_sleep_raises() -> None:
    """Negative sleep duration is rejected."""
    clock = SystemClock()
    with pytest.raises(ValueError, match="non-negative"):
        await clock.sleep(-1)


def test_mock_clock_default_epoch() -> None:
    """``MockClock`` starts at the 2026-01-01 epoch by default."""
    clock = MockClock()
    assert clock.now() == datetime(2026, 1, 1, tzinfo=timezone.utc)
    assert clock.monotonic() == 0.0


def test_mock_clock_advance() -> None:
    """``advance`` moves both wall and monotonic clocks forward."""
    clock = MockClock()
    clock.advance(60)
    assert clock.now() == datetime(2026, 1, 1, 0, 1, 0, tzinfo=timezone.utc)
    assert clock.monotonic() == 60.0


def test_mock_clock_set() -> None:
    """``set`` rewrites the wall clock to the given moment."""
    clock = MockClock()
    target = datetime(2027, 5, 4, 12, 0, 0, tzinfo=timezone.utc)
    clock.set(target)
    assert clock.now() == target


def test_mock_clock_set_rejects_naive_datetime() -> None:
    """``set`` requires a timezone-aware ``datetime``."""
    clock = MockClock()
    with pytest.raises(ValueError, match="timezone-aware"):
        clock.set(datetime(2027, 1, 1))  # no tzinfo


def test_mock_clock_advance_rejects_negative() -> None:
    """``advance`` rejects negative values."""
    clock = MockClock()
    with pytest.raises(ValueError, match="non-negative"):
        clock.advance(-1)


@pytest.mark.asyncio
async def test_mock_clock_sleep_advances_time() -> None:
    """``sleep`` advances virtual time without actually waiting."""
    clock = MockClock()
    await clock.sleep(30)
    assert clock.monotonic() == 30.0
    assert clock.now() == datetime(2026, 1, 1, 0, 0, 30, tzinfo=timezone.utc)


@pytest.mark.asyncio
async def test_mock_clock_sleep_rejects_negative() -> None:
    """``MockClock.sleep`` also rejects negative values."""
    clock = MockClock()
    with pytest.raises(ValueError, match="non-negative"):
        await clock.sleep(-0.1)
