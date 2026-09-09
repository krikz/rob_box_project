"""`Clock` — time injection port.

Production code uses ``SystemClock``. Tests use ``MockClock`` to make
time-deterministic assertions (timeout, idle checks, retry backoff).

The clock's only contract is "give me the current time". ``sleep`` is provided
for ergonomics but production code is free to ignore it and use ``rclpy`` /
``asyncio`` primitives directly — the port exists mainly to support tests
that want to advance time without sleeping.
"""

from __future__ import annotations

import abc
import time as _time


class Clock(abc.ABC):
    """Minimal time abstraction."""

    @abc.abstractmethod
    def now(self) -> float:
        """Wall-clock seconds since the epoch (matches ``time.time()``)."""

    @abc.abstractmethod
    def monotonic(self) -> float:
        """Monotonic seconds (matches ``time.monotonic()``)."""

    @abc.abstractmethod
    async def sleep(self, seconds: float) -> None:
        """Sleep for ``seconds`` (wall clock)."""


class SystemClock(Clock):
    """Production clock. Just delegates to ``time`` / ``asyncio``."""

    def now(self) -> float:
        return _time.time()

    def monotonic(self) -> float:
        return _time.monotonic()

    async def sleep(self, seconds: float) -> None:
        if seconds <= 0:
            return
        import asyncio

        await asyncio.sleep(seconds)


class MockClock(Clock):
    """Deterministic clock for tests.

    - ``now()`` / ``monotonic()`` return the current fake time.
    - ``advance(seconds)`` moves the fake clock forward.
    - ``set_now(t)`` jumps to an absolute timestamp.
    - ``sleep(seconds)`` advances the clock but does NOT yield to the event loop
      (so a test running in a single event loop tick stays deterministic).
      Use ``asyncio.sleep(0)`` explicitly between steps if you need a tick.
    """

    def __init__(self, *, start: float = 0.0) -> None:
        self._now = float(start)
        self._mono = float(start)
        self.sleeps: list[float] = []

    # -- read ----------------------------------------------------------------

    def now(self) -> float:
        return self._now

    def monotonic(self) -> float:
        return self._mono

    async def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        if seconds > 0:
            self._now += seconds
            self._mono += seconds

    # -- mutate --------------------------------------------------------------

    def advance(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("advance() requires a non-negative duration")
        self._now += seconds
        self._mono += seconds

    def set_now(self, t: float) -> None:
        self._now = float(t)
        self._mono = float(t)


__all__ = ["Clock", "SystemClock", "MockClock"]
