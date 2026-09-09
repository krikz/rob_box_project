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
from dataclasses import asdict, dataclass, field
from typing import Any, Awaitable, Callable, Generic, Type, TypeVar, Union, cast

logger = logging.getLogger(__name__)


T = TypeVar("T")

# Effect "kind" identifiers — ADR-0001 §2.4.4 names the full fan-out.
# Each new effect type registers here (see :func:`_register_effect`)
# so :func:`effect_kind`, :func:`to_dict`, and :func:`from_dict` can
# route / serialise it. The core ``LogEffect`` / ``EchoEffect`` carry
# the kinds ``"log"`` / ``"echo"``; the TG-flavoured effects
# (``SendReplyEffect`` / ``SpeakEffect`` / ``PlaySoundEffect`` /
# ``SetLEDEffect`` / ``MoveEffect``) register ``"send_reply"`` /
# ``"speak"`` / ``"play_sound"`` / ``"set_led"`` / ``"move"`` and
# are defined below alongside the buses.
_EFFECT_KIND_ATTR = "__effect_kind__"


def effect_kind(effect: "Effect[Any]") -> str:
    """Return the canonical kind string for ``effect``.

    Effect subclasses expose their kind via the
    :data:`_EFFECT_KIND_ATTR` class attribute (set by the
    :func:`_register_effect` decorator). Used for snapshot
    serialisation (``to_dict``/``from_dict``) and for the routing
    decisions in concrete buses.
    """
    kind = getattr(effect, _EFFECT_KIND_ATTR, None)
    if not isinstance(kind, str):
        raise ValueError(
            f"{type(effect).__name__} has no effect kind attribute; "
            "did you forget to register it via rob_box_harness.effects._register_effect?"
        )
    return kind


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
    """Append a structured log line to the harness's logger.

    ``kind`` is registered as ``"log"`` so :func:`effect_kind` /
    :func:`to_dict` route and serialise it.
    """

    message: str
    level: int = logging.INFO
    fields: dict[str, Any] = field(default_factory=dict)
    __effect_kind__: str = "log"

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
    __effect_kind__: str = "echo"

    async def apply(self, ctx: EffectContext) -> None:
        """Send ``text`` to the sink or log it when no sink is set."""
        if self.sink is not None:
            self.sink(self.text)
        else:
            ctx.logger.info("ECHO[%s] %s", ctx.harness, self.text)


# ---------------------------------------------------------------------------
# Telegram-flavoured effects (P1.4, ADR-0001 §2.4.4).
#
# These effects are part of the *core* effects union (not a separate
# module) so the harness framework can dispatch them through
# ``CompositeBus`` without a separate import. Their concrete routing
# destinations (TelegramBus is the only one shipping today per H.3;
# TTSBus / SoundBus / LEDBus / MoveBus belong to P2.4) live alongside
# in this same module so the ``__effect_kind__`` registration is
# co-located with the effect definitions.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SendReplyEffect(Effect[None]):
    """Send a plain-text reply to a Telegram chat.

    Args:
        channel: Telegram chat_id (stringified numeric id).
        text: Message body. UTF-8. The sink decides parse mode.
        reply_markup: Optional pre-built ``telegram.InlineKeyboardMarkup``
            / ``ReplyKeyboardMarkup``. Kept as ``Any`` so this module
            does not depend on ``python-telegram-bot``.
    """

    channel: str
    text: str
    reply_markup: Any = None
    __effect_kind__: str = "send_reply"

    async def apply(self, ctx: EffectContext) -> None:
        """Default no-op; production wires :class:`TelegramBus`."""
        ctx.logger.debug(
            "SendReplyEffect[%s → %s]: %s",
            ctx.harness,
            self.channel,
            self.text,
        )


@dataclass(frozen=True)
class SpeakEffect(Effect[None]):
    """Forward-compat: speak ``text`` via the TTS pipeline (P2.4 routes it)."""

    text: str
    voice: str | None = None
    ssml: str | None = None
    __effect_kind__: str = "speak"

    async def apply(self, ctx: EffectContext) -> None:
        ctx.logger.debug("SpeakEffect[%s]: %s", ctx.harness, self.text)


@dataclass(frozen=True)
class PlaySoundEffect(Effect[None]):
    """Forward-compat: play a named sound from the sound package."""

    name: str
    volume: float = 1.0
    __effect_kind__: str = "play_sound"

    async def apply(self, ctx: EffectContext) -> None:
        ctx.logger.debug("PlaySoundEffect[%s]: %s", ctx.harness, self.name)


@dataclass(frozen=True)
class SetLEDEffect(Effect[None]):
    """Forward-compat: set an LED pattern on the LED matrix."""

    pattern: str
    color: str = "white"
    duration_ms: int = 0
    __effect_kind__: str = "set_led"

    async def apply(self, ctx: EffectContext) -> None:
        ctx.logger.debug(
            "SetLEDEffect[%s]: %s/%s/%dms",
            ctx.harness,
            self.pattern,
            self.color,
            self.duration_ms,
        )


@dataclass(frozen=True)
class MoveEffect(Effect[None]):
    """Forward-compat: publish a movement command to /cmd_vel."""

    linear: float = 0.0
    angular: float = 0.0
    __effect_kind__: str = "move"

    async def apply(self, ctx: EffectContext) -> None:
        ctx.logger.debug(
            "MoveEffect[%s]: lin=%.2f ang=%.2f",
            ctx.harness,
            self.linear,
            self.angular,
        )


# ---------------------------------------------------------------------------
# Public type alias covering every TG-flavoured effect (used by
# :class:`TelegramFilteredBus` and downstream annotations).
# ---------------------------------------------------------------------------


# Type alias kept here (not TYPE_CHECKING) so it's importable at runtime
# by downstream packages that want a single "telegram-side" union.
TelegramEffect = Union[  # noqa: F821 — defined after this point in module
    "SendReplyEffect",
    "SpeakEffect",
    "PlaySoundEffect",
    "SetLEDEffect",
    "MoveEffect",
]


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


# ---------------------------------------------------------------------------
# TelegramBus — the only concrete fan-out destination shipping in P1.4
# (host decision H.3). Routes ``SendReplyEffect`` to an injected channel
# adapter; ignores every other effect kind so a ``CompositeBus`` can
# route TTS/Sound/LED independently once P2 lands.
# ---------------------------------------------------------------------------


class TelegramChannel(abc.ABC):
    """Abstract sink that :class:`TelegramBus` sends replies to.

    The concrete ``python-telegram-bot`` adapter lives outside the
    harness package; tests inject a tiny in-memory implementation.
    """

    @abc.abstractmethod
    async def send_message(
        self,
        chat_id: str,
        text: str,
        reply_markup: Any = None,
    ) -> None:
        """Send ``text`` to ``chat_id``."""


class TelegramBus(SideEffectBus):
    """A bus that routes :class:`SendReplyEffect` to a :class:`TelegramChannel`.

    Every other effect kind is a no-op: the bus is intentionally
    narrow so it can be safely combined with other concrete buses
    inside a :class:`CompositeBus` without double-routing.

    The channel is set via the constructor or via :meth:`bind`. Both
    are intentional — construction is the production path, ``bind``
    is the test path (and lets the bus outlive a channel swap).
    """

    name = "telegram"

    def __init__(self, channel: "TelegramChannel | None" = None) -> None:
        self._channel: "TelegramChannel | None" = channel

    def bind(self, channel: "TelegramChannel") -> None:
        """Attach ``channel`` (idempotent — last call wins)."""
        self._channel = channel

    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Route ``SendReplyEffect`` to the channel; drop everything else."""
        if not isinstance(effect, SendReplyEffect):
            return None
        if self._channel is None:
            logger.warning(
                "TelegramBus: dropped SendReplyEffect — no channel bound "
                "(chat_id=%s, len=%d)",
                effect.channel,
                len(effect.text),
            )
            return None
        await self._channel.send_message(
            effect.channel,
            effect.text,
            effect.reply_markup,
        )
        return None


# Signature of a Telegram send sink — tests inject a lambda; production
# wires ``bot.send_message``. ``chat_id``, ``text``, ``reply_markup``.
TelegramSendSink = Callable[[str, str, Any], Awaitable[None]]


class TelegramFilteredBus(SideEffectBus):
    """A bus that forwards ONLY ``kind`` instances to a child bus.

    Composes with :class:`CompositeBus` to build production wiring
    where each channel has its own bus::

        TelegramBus(channel=TelegramChannelAdapter(bot))        ← SendReply
        RecordingBus()                                          ← everything else

        bus = CompositeBus([
            TelegramFilteredBus(TelegramBus(...), kind=SendReplyEffect),
            RecordingBus(),
        ])
    """

    name = "filtered"

    def __init__(self, inner: SideEffectBus, *, kind: "Type[TelegramEffect]") -> None:
        if inner is None:
            raise ValueError("TelegramFilteredBus requires a non-None inner bus")
        self._inner = inner
        self._kind = kind

    async def dispatch(self, effect: Effect[Any]) -> Any:
        """Forward ``effect`` to ``inner`` iff it is a ``kind`` instance."""
        if not isinstance(effect, self._kind):
            return None
        return await self._inner.dispatch(effect)

    @property
    def inner(self) -> SideEffectBus:
        """Expose the wrapped bus for assertions in tests."""
        return self._inner


# ---------------------------------------------------------------------------
# Snapshot serialisation — :func:`to_dict` / :func:`from_dict` round-trip
# every effect in the union (registered via :func:`_register_effect`).
# Used by ``SessionSnapshot`` to persist the pending-effects list across
# a process restart (P1.4 e2e acceptance).
# ---------------------------------------------------------------------------


_KIND_TO_CLASS: dict[str, type[Effect[Any]]] = {}


def _register_effect(cls: type[Effect[Any]]) -> type[Effect[Any]]:
    """Register an :class:`Effect` subclass by its ``__effect_kind__``.

    Each Effect subclass carries a class-level ``__effect_kind__``
    string (set by the module that defines the effect). Calling
    :func:`_register_effect` populates the global kind → class map
    used by :func:`effect_kind` and :func:`from_dict`. The function
    is idempotent: re-registering the same kind raises to catch
    accidental shadowing between modules.
    """
    kind = getattr(cls, _EFFECT_KIND_ATTR, None)
    if not isinstance(kind, str):
        raise TypeError(
            f"{cls.__name__} is missing the {_EFFECT_KIND_ATTR!r} attribute"
        )
    existing = _KIND_TO_CLASS.get(kind)
    if existing is not None and existing is not cls:
        raise ValueError(
            f"effect kind {kind!r} already registered to {existing.__name__}; "
            f"refusing to overwrite with {cls.__name__}"
        )
    _KIND_TO_CLASS[kind] = cls
    return cls


# Register the core + TG-flavoured effects so :func:`effect_kind` /
# :func:`from_dict` work without requiring callers to import every
# module. The TG effects are co-located with the core effects so
# registration happens once at module import.
for _cls in (
    LogEffect,
    EchoEffect,
    SendReplyEffect,
    SpeakEffect,
    PlaySoundEffect,
    SetLEDEffect,
    MoveEffect,
):
    _register_effect(_cls)


def to_dict(effect: Effect[Any]) -> dict[str, Any]:
    """Serialise ``effect`` to a JSON-safe dict keyed by ``kind``.

    Concrete Effect subclasses are dataclasses (see LogEffect,
    SendReplyEffect, etc.); the abstract :class:`Effect` itself is
    not, so mypy can't see the dataclass-ness through the parent
    type. The ``cast`` documents that every concrete effect
    registered through :func:`_register_effect` IS a dataclass.
    """
    return {
        "kind": effect_kind(effect),
        "payload": asdict(cast(Any, effect)),
    }


def from_dict(data: dict[str, Any]) -> Effect[Any]:
    """Restore an :class:`Effect` previously serialised with :func:`to_dict`."""
    kind = data.get("kind")
    if not isinstance(kind, str):
        raise ValueError(f"effect payload missing 'kind' string: {data!r}")
    cls = _KIND_TO_CLASS.get(kind)
    if cls is None:
        raise ValueError(
            f"unknown effect kind {kind!r}; known kinds: {sorted(_KIND_TO_CLASS)}"
        )
    payload = data.get("payload", {})
    if not isinstance(payload, dict):
        raise ValueError(
            f"effect payload must be a mapping, got {type(payload).__name__}"
        )
    # ``__effect_kind__`` is a class attribute, not a field, but
    # ``asdict`` may pick it up if a future effect renames a field
    # to match — drop it defensively to keep ``from_dict`` decoupled
    # from the registration mechanism.
    kwargs = {key: value for key, value in payload.items() if key != _EFFECT_KIND_ATTR}
    return cls(**kwargs)


__all__ = [
    "Effect",
    "EffectContext",
    "LogEffect",
    "EchoEffect",
    "SendReplyEffect",
    "SpeakEffect",
    "PlaySoundEffect",
    "SetLEDEffect",
    "MoveEffect",
    "SideEffectBus",
    "NoopBus",
    "RecordingBus",
    "CompositeBus",
    "TelegramChannel",
    "TelegramBus",
    "TelegramFilteredBus",
    "TelegramSendSink",
    "TelegramEffect",
    "effect_kind",
    "to_dict",
    "from_dict",
]
