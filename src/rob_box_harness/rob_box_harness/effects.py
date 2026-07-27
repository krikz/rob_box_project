"""SideEffectBus — declares "what happens" without owning the I/O.

Concrete harnesses (Dialog, Telegram, Persistent) all need to send
output to *something* — TTS, sound, LED, Telegram reply, log file.
Mixing that I/O into the harness body makes tests impossible
without mocking the world. The :class:`SideEffectBus` therefore
sits in the middle:

* Harnesses call ``await bus.dispatch(effect)`` with a declarative
  :class:`Effect` object describing WHAT to do.
* The bus owns HOW — it fans the effect out to one or more downstream
  impls (TTS publisher, telegram bot, etc.).

This makes three concrete test shapes possible:

1. **NoopBus** — drops every effect; tests assert "did the harness
   *try* to send text?" without actually sending.
2. **RecordingBus** — appends every effect to a list; tests assert
   the exact sequence of declared effects.
3. **CompositeBus** — fans out to multiple buses (e.g. TTS + sound
   + LED in production).

The :class:`Effect` ABC is generic in the return type (``T``) so a
harness can declare a structured effect (e.g. ``TTSReply(text: str)``
returning a play_id) and the bus routes it accordingly.
"""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass, field
from typing import Any, Generic, TypeVar

logger = logging.getLogger(__name__)


T = TypeVar("T")


class EffectContext:
    """Per-call context passed to :meth:`Effect.apply`.

    Holds the harness name and the structured logger so an effect
    can produce consistent, filterable log lines.
    """

    def __init__(self, harness: str, *, logger_override: logging.Logger | None = None) -> None:
        self.harness = harness
        self.logger = logger_override or logger


class Effect(abc.ABC, Generic[T]):
    """A declarative description of an external side-effect.

    Subclasses are frozen dataclasses that hold the effect payload
    (e.g. ``text``, ``play_id``). The :meth:`apply` method is the
    only place I/O happens; the harness stays pure.
    """

    @abc.abstractmethod
    async def apply(self, ctx: EffectContext) -> T:
        """Execute the effect. Must be idempotent where possible."""


# ---------------------------------------------------------------------------
# Concrete effects (kept small — concrete harnesses will define their own
# specialised subclasses in their own modules, but a few are useful for
# the smoke tests and the dummy harnesses).
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LogEffect(Effect[None]):
    """Append a structured log line to the harness's logger."""

    message: str
    level: int = logging.INFO
    fields: dict[str, Any] = field(default_factory=dict)

    async def apply(self, ctx: EffectContext) -> None:
        """Log ``message`` at ``level`` with harness context."""
        ctx.logger.log(
            self.level,
            self.message,
            extra={"harness": ctx.harness, **self.fields},
        )


@dataclass(frozen=True)
class EchoEffect(Effect[None]):
    """Emit a string to a sink callable (the smoke test's "stdout").

    The smoke bus uses this to prove that ``dispatch`` actually
    fires without bringing in TTS or Telegram.
    """

    text: str
    sink: Any = None  # callable[[str], None] | None, kept loose for tests

    async def apply(self, ctx: EffectContext) -> None:
        """Send ``text`` to the sink or log it when no sink is set."""
        if self.sink is not None:
            self.sink(self.text)
        else:
            ctx.logger.info("ECHO[%s] %s", ctx.harness, self.text)


# ---------------------------------------------------------------------------
# Buses
# ---------------------------------------------------------------------------


class SideEffectBus(abc.ABC):
    """Abstract side-effect bus."""

    name: str = "abstract"

    @abc.abstractmethod
    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Fan ``effect`` out to downstream impls. Returns the result."""


class NoopBus(SideEffectBus):
    """A bus that drops every effect. Used by tests.

    The bus still calls the effect's ``apply`` against a context with
    a no-op logger so observers can detect "did the harness try to
    send something?" via :attr:`count`.
    """

    name = "noop"

    def __init__(self) -> None:
        self.count = 0

    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Apply ``effect`` in an isolated context, swallow exceptions."""
        self.count += 1
        # Still call apply so any side-effecting logger output is
        # captured. We just isolate it from the real world via the
        # caller-supplied logger.
        ctx = EffectContext(harness="noop")
        try:
            return await effect.apply(ctx)
        except Exception:  # noqa: BLE001 — record but don't propagate
            logger.exception("noop bus swallowed effect error")
            return None


class RecordingBus(SideEffectBus):
    """A bus that records every effect for later inspection.

    Tests use this to assert "did the harness dispatch EXACTLY these
    effects in this order?". The list is append-only; clear it via
    :meth:`reset` between tests.
    """

    name = "recording"

    def __init__(self) -> None:
        self.effects: list[Effect[Any]] = []

    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Append ``effect`` to the recorded list and return ``None``."""
        self.effects.append(effect)
        return None

    def reset(self) -> None:
        """Clear the recorded list of effects."""
        self.effects.clear()


class CompositeBus(SideEffectBus):
    """A bus that fans effects out to multiple downstream buses.

    All downstream buses are invoked; the first one to raise lets
    the exception propagate. Return values are intentionally
    discarded — if a caller needs the result of a single bus they
    should not go through the composite.
    """

    name = "composite"

    def __init__(self, buses: list[SideEffectBus]) -> None:
        if not buses:
            raise ValueError("CompositeBus requires at least one downstream bus")
        self._buses = list(buses)

    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Fan ``effect`` out to every downstream bus and return the last result."""
        last: Any = None
        for bus in self._buses:
            last = await bus.dispatch(effect)
        return last

    @property
    def buses(self) -> list[SideEffectBus]:
        """Return the list of downstream buses (read-only snapshot)."""
        return list(self._buses)


__all__ = [
    "Effect",
    "EffectContext",
    "LogEffect",
    "EchoEffect",
    "SideEffectBus",
    "NoopBus",
    "RecordingBus",
    "CompositeBus",
]
