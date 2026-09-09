"""``Clock`` port — DI for wall time so tests can advance time deterministically.

Why a port at all:

* The harness lifecycle sometimes needs to log "elapsed since init"
  or schedule a deferred ``teardown`` retry. Forcing the harness to
  call :func:`time.time` directly ties it to the real wall clock and
  makes tests either flaky or dependent on ``freezegone``-style
  monkey-patching.
* The :class:`MockClock` implementation lets a unit test pin time
  to a known value, then advance it explicitly by calling
  ``clock.advance(seconds)`` or :meth:`MockClock.set`.

Rules (ADR-0001 §2.4.6 and §2.4.8):

* :meth:`Clock.now` returns a timezone-aware ``datetime`` in UTC.
* :meth:`Clock.sleep` is async-only — never use blocking ``time.sleep``
  inside a harness.
* :meth:`Clock.monotonic` returns a float in seconds, suitable for
  measuring intervals.
"""

from __future__ import annotations

import abc
import asyncio
import time
from datetime import datetime, timezone


class Clock(abc.ABC):
    """Abstract time source for harnesses."""

    @abc.abstractmethod
    def now(self) -> datetime:
        """Return the current wall-clock time as an aware UTC datetime."""

    @abc.abstractmethod
    async def sleep(self, seconds: float) -> None:
        """Sleep for ``seconds`` without blocking the event loop."""

    @abc.abstractmethod
    def monotonic(self) -> float:
        """Return a monotonic counter (seconds) for interval measurement."""


class SystemClock(Clock):
    """Production clock backed by :mod:`time` and :mod:`asyncio`."""

    def now(self) -> datetime:
        """Return the current wall-clock time as an aware UTC datetime."""
        return datetime.now(tz=timezone.utc)

    async def sleep(self, seconds: float) -> None:
        """Sleep for ``seconds`` without blocking the event loop."""
        if seconds < 0:
            raise ValueError(f"sleep seconds must be non-negative, got {seconds}")
        await asyncio.sleep(seconds)

    def monotonic(self) -> float:
        """Return a monotonic counter (seconds) for interval measurement."""
        return time.monotonic()


class MockClock(Clock):
    """Deterministic clock for tests.

    Defaults to a fixed epoch (``2026-01-01T00:00:00Z``). Tests advance
    time with :meth:`advance` or :meth:`set`. ``sleep`` is implemented
    without actually waiting — tests can iterate through long virtual
    sequences instantly.
    """

    DEFAULT_EPOCH = datetime(2026, 1, 1, tzinfo=timezone.utc)

    def __init__(self, *, start: datetime | None = None) -> None:
        self._current: datetime = (start or self.DEFAULT_EPOCH).astimezone(timezone.utc)
        self._monotonic: float = 0.0

    def now(self) -> datetime:
        """Return the (deterministic) current wall-clock time."""
        return self._current

    async def sleep(self, seconds: float) -> None:
        """Advance the virtual clock by ``seconds`` and yield once."""
        if seconds < 0:
            raise ValueError(f"sleep seconds must be non-negative, got {seconds}")
        # Advance the virtual clock immediately: async sleep yields
        # control but the *test* doesn't need to wait. This keeps
        # test runs fast regardless of how long the harness "slept".
        self._monotonic += seconds
        self._current = self._current.fromtimestamp(
            self._current.timestamp() + seconds, tz=timezone.utc
        )
        # Yield once so the event loop can interleave other tasks.
        await asyncio.sleep(0)

    def monotonic(self) -> float:
        """Return the (deterministic) monotonic counter in seconds."""
        return self._monotonic

    def set(self, moment: datetime) -> None:
        """Set the wall clock to ``moment`` (must be UTC-aware)."""
        if moment.tzinfo is None:
            raise ValueError("MockClock.set requires a timezone-aware datetime")
        self._current = moment.astimezone(timezone.utc)

    def advance(self, seconds: float) -> None:
        """Advance both wall and monotonic clocks by ``seconds``."""
        if seconds < 0:
            raise ValueError("advance seconds must be non-negative")
        self._monotonic += seconds
        self._current = self._current.fromtimestamp(
            self._current.timestamp() + seconds, tz=timezone.utc
        )


__all__ = ["Clock", "SystemClock", "MockClock"]
