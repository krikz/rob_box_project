"""SnapshotStore port — ephemeral camera/frame cache (ADR §2.4.7, §2.8).

A :class:`SnapshotStore` is a thin port that lets harnesses stash
the latest JPEG / PNG / depth-frame / occupancy-grid per scope
without depending on the ROS 2 subscription layer. The Telegram
harness uses scopes like ``"tg:{chat_id}"`` so different chats get
isolated caches.

Why a port (not a concrete class):
    The existing ``SnapshotStore`` inside ``harnesses/telegram.py``
    is the *TG-side* impl — it lives next to the harness code. The
    *port* lives here so other harnesses (Dialog, Persistent) can
    reach for the same contract without importing the Telegram
    harness.

Implementations:
    * :class:`InMemorySnapshotStore` — default; single-process,
      thread-unsafe by design (event loop scope).

Usage::

    store: SnapshotStore = InMemorySnapshotStore()
    key = await store.put(scope="tg:42", payload=b"jpeg-bytes")
    latest = await store.get_latest(scope="tg:42")
"""

from __future__ import annotations

import abc
import time as _time
import uuid
from dataclasses import dataclass
from typing import Any, Mapping, Optional

from rob_box_harness.transport import TelegramUpdate


@dataclass(frozen=True)
class SnapshotEntry:
    """One entry in the snapshot store: bytes + capture timestamp.

    ``payload`` is opaque to the port — bytes for JPEGs, dicts for
    occupancy grids, etc. The harness decides the encoding.
    """

    payload: Any
    captured_at: float


class SnapshotStore(abc.ABC):
    """Abstract snapshot cache.

    Implementations MUST be safe for single-event-loop access only
    (no locks). They MUST be idempotent under :meth:`aclose`.
    """

    name: str = "abstract"

    @abc.abstractmethod
    async def put(self, *, scope: str, payload: Any) -> str:
        """Store ``payload`` under ``scope`` and return the cache key."""

    @abc.abstractmethod
    async def get_latest(self, scope: str) -> Optional[SnapshotEntry]:
        """Return the most recent entry for ``scope`` (or None)."""

    @abc.abstractmethod
    async def expire(self, max_age_seconds: float) -> int:
        """Drop entries older than ``max_age_seconds``. Return count removed."""

    async def aclose(self) -> None:
        """Release resources. Default no-op."""
        return None


class InMemorySnapshotStore(SnapshotStore):
    """Thread-unsafe in-memory snapshot store keyed by scope.

    The most-recent entry per scope is kept indefinitely; older
    entries are dropped by :meth:`expire`. Default TTL is 5 minutes
    (matching the legacy TG-side CameraCache contract).
    """

    name = "in_memory"

    def __init__(self, *, default_ttl_seconds: float = 300.0) -> None:
        self._store: dict[str, SnapshotEntry] = {}
        self._closed: bool = False
        self._default_ttl = default_ttl_seconds

    async def put(self, *, scope: str, payload: Any) -> str:
        """Store ``payload`` under ``scope``; return a unique key."""
        if self._closed:
            raise RuntimeError("InMemorySnapshotStore is closed")
        key = f"snap:{scope}:{uuid.uuid4().hex[:8]}"
        self._store[key] = SnapshotEntry(payload=payload, captured_at=_time.monotonic())
        return key

    async def get_latest(self, scope: str) -> Optional[SnapshotEntry]:
        """Return the entry with the largest ``captured_at`` for ``scope``."""
        if self._closed:
            return None
        candidates = [
            entry for key, entry in self._store.items()
            if key.startswith(f"snap:{scope}:")
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda e: e.captured_at)

    async def expire(self, max_age_seconds: float) -> int:
        """Drop entries older than ``max_age_seconds``. Return count removed."""
        now = _time.monotonic()
        expired_keys = [
            k for k, e in self._store.items()
            if now - e.captured_at > max_age_seconds
        ]
        for k in expired_keys:
            del self._store[k]
        return len(expired_keys)

    async def aclose(self) -> None:
        """Mark the store closed. Idempotent."""
        self._closed = True
        self._store.clear()


# ---------------------------------------------------------------------------
# Telegram update normalisation (ADR §2.4.5)
# ---------------------------------------------------------------------------


def parse_telegram_update(value: Any) -> TelegramUpdate:
    """Coerce ``value`` into a :class:`TelegramUpdate`.

    Accepts:
        * :class:`TelegramUpdate` — returned as-is (identity).
        * ``str`` — wrapped as a message update with ``payload = {"text": value}``.
        * ``Mapping`` — copied into a new ``payload`` dict, ``kind`` defaults
          to ``"message"`` (the only kind the TelegramHarness cares about
          today; callback/edited/inline kinds will be added when the harness
          grows).
        * anything else — :class:`TypeError`.

    The parser is intentionally tiny: the harness-side
    ``telegram_node`` already does python-telegram-bot
    deserialisation; this function is the boundary between the
    transport port and the harness's ``step(input)``.
    """
    if isinstance(value, TelegramUpdate):
        return value
    if isinstance(value, str):
        return TelegramUpdate(
            update_id=0,
            kind="message",
            payload={"text": value},
        )
    if isinstance(value, Mapping):
        payload = dict(value)
        kind = str(payload.get("kind", "message"))
        return TelegramUpdate(update_id=int(payload.get("update_id", 0)), kind=kind, payload=payload)
    raise TypeError(
        f"parse_telegram_update expected TelegramUpdate | str | Mapping, "
        f"got {type(value).__name__}"
    )


__all__ = [
    "SnapshotEntry",
    "SnapshotStore",
    "InMemorySnapshotStore",
    "parse_telegram_update",
]