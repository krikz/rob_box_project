"""Transport port — normalises "how an input event reaches the harness".

A transport is a thin adapter between an external event source
(STT, VAD, Telegram update, keyboard, vision event) and the
harness's ``on_user_input`` / ``on_audio_chunk`` / ``on_event``
contract. We separate transport from the harness itself so:

* The harness stays HTTP/ROS-agnostic — test code can drive the
  same code path with a :class:`FakeTransport`.
* A single harness can multiplex several transports
  (e.g. Dialog accepts both voice and keyboard).
* The transport layer is the right place to debounce / normalise
  schema differences between input sources.

The contract is intentionally narrow (ADR-0001 §2.4.5). Concrete
harnesses can subclass :class:`BaseTransport` to add helpers
specific to their input type (e.g. ``stt_confidence_threshold``).
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Mapping


# ---------------------------------------------------------------------------
# Event value objects
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VadEvent:
    """Voice activity detection event."""

    is_speech: bool
    confidence: float = 1.0


@dataclass(frozen=True)
class KeyEvent:
    """Keyboard / hardware-key event."""

    code: str
    pressed: bool = True


@dataclass(frozen=True)
class TelegramUpdate:
    """Raw payload from the Telegram ``Update`` object.

    The harness doesn't need the full ``python-telegram-bot`` type
    — it just needs to know what kind of update it is and any
    helper payload. Dataclass, not dict, so ``isinstance`` checks
    work in tests.
    """

    update_id: int
    kind: str  # "message" | "callback" | "edited" | "inline_query" | ...
    payload: Mapping[str, Any]


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------


class Transport(abc.ABC):
    """Abstract transport."""

    name: str = "abstract"

    @abc.abstractmethod
    async def on_stt_result(self, text: str, *, confidence: float) -> None:
        """Forward ``text`` (and its recognition confidence) to the harness."""

    @abc.abstractmethod
    async def on_vad(self, event: VadEvent) -> None:
        """Forward a VAD event."""

    @abc.abstractmethod
    async def on_telegram_update(self, update: TelegramUpdate) -> None:
        """Forward a Telegram update."""

    @abc.abstractmethod
    async def on_key_event(self, event: KeyEvent) -> None:
        """Forward a hardware / keyboard event."""

    async def aclose(self) -> None:
        """Release resources. Default no-op for stateless transports."""
        return None


# Callback type for transport forwarding. A harness sets a handler once
# during ``init``; the transport calls it whenever an event arrives.
EventHandler = Callable[[str, Any], Awaitable[None]]
"""A handler attached to a transport. Signature: ``(kind, payload)``."""


class BaseTransport(Transport):
    """Convenience base class that stores a handler and forwards events.

    Concrete transports (ROS2, in-memory, etc.) override the
    individual ``on_*`` methods and just call into the handler.
    """

    def __init__(self, name: str = "base") -> None:
        self.name = name
        self._handler: EventHandler | None = None

    def bind(self, handler: EventHandler) -> None:
        """Attach a handler that receives ``(kind, payload)`` tuples."""
        self._handler = handler

    def unbind(self) -> None:
        """Detach the currently bound handler (idempotent)."""
        self._handler = None

    async def _dispatch(self, kind: str, payload: Any) -> None:
        if self._handler is None:
            return
        await self._handler(kind, payload)


class FakeTransport(BaseTransport):
    """In-memory transport directly driven by tests.

    No I/O, no subscriptions. Tests call :meth:`feed_stt`,
    :meth:`feed_vad`, etc. to simulate external events. The history
    is recorded so callers can assert "did the transport receive
    N events?" after the test.
    """

    def __init__(self, name: str = "fake") -> None:
        super().__init__(name=name)
        self.history: list[tuple[str, Any]] = []

    async def on_stt_result(self, text: str, *, confidence: float) -> None:
        """Record an STT result and forward it to the bound handler."""
        self.history.append(("stt_result", {"text": text, "confidence": confidence}))
        await self._dispatch("stt_result", {"text": text, "confidence": confidence})

    async def on_vad(self, event: VadEvent) -> None:
        """Record a VAD event and forward it to the bound handler."""
        self.history.append(("vad", event))
        await self._dispatch("vad", event)

    async def on_telegram_update(self, update: TelegramUpdate) -> None:
        """Record a Telegram update and forward it to the bound handler."""
        self.history.append(("telegram_update", update))
        await self._dispatch("telegram_update", update)

    async def on_key_event(self, event: KeyEvent) -> None:
        """Record a keyboard event and forward it to the bound handler."""
        self.history.append(("key_event", event))
        await self._dispatch("key_event", event)

    # ---------- test helpers ------------------------------------------------

    async def feed_stt(self, text: str, *, confidence: float = 1.0) -> None:
        """Test helper: simulate an STT result arriving."""
        await self.on_stt_result(text, confidence=confidence)

    async def feed_vad(self, is_speech: bool, *, confidence: float = 1.0) -> None:
        """Test helper: simulate a VAD event arriving."""
        await self.on_vad(VadEvent(is_speech=is_speech, confidence=confidence))

    async def feed_key(self, code: str, *, pressed: bool = True) -> None:
        """Test helper: simulate a keyboard event arriving."""
        await self.on_key_event(KeyEvent(code=code, pressed=pressed))

    async def feed_telegram(self, update: TelegramUpdate) -> None:
        """Test helper: simulate a Telegram update arriving."""
        await self.on_telegram_update(update)


__all__ = [
    "VadEvent",
    "KeyEvent",
    "TelegramUpdate",
    "Transport",
    "BaseTransport",
    "FakeTransport",
    "EventHandler",
]
