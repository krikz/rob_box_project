"""MemoryStore port — per-scope conversation history and facts.

The contract covers the core operations every concrete memory
(``SQLiteVoiceMemory``, ``RedisStore``, ``fake.InMemoryStore``) MUST
expose (ADR-0001 §2.4.3 + Phase 6 v2 W4):

Conversation + facts (scoped):
* ``load_recent`` — fetch the last N turns for a given scope.
* ``append_turn``  — append a single turn (idempotent).
* ``save_fact``    — persist a structured fact (e.g. "user likes jazz").
* ``search_facts`` — best-effort semantic search over stored facts.

Waypoints (global navigation memory):
* ``save_waypoint(name, x, y, theta)`` — store a named pose.
* ``list_waypoints()``                 — all saved waypoints.
* ``delete_waypoint(name)``            — remove by name.
* ``clear_waypoints()``                — wipe all.

FAQ (per event):
* ``load_faq(event_id, items)``        — replace FAQ rows for an event.
* ``search_faq(event_id, query, limit)`` — keyword search over FAQ.

Event profile (singleton, active event context):
* ``set_event_profile(profile)``       — set/overwrite active profile.
* ``get_event_profile()``              — read the active profile.

The ``scope`` argument is deliberately a string (not an enum) so
harnesses can pass arbitrary bucket names: ``"user:42"``,
``"chat:tg:9001"``, ``"session:abc"``. The format is harness-specific
and the MemoryStore doesn't need to interpret it.

Waypoints, FAQ, and the event profile are *not* scoped per-user —
they belong to the robot itself or to a single active event — so
their API does not take a ``scope`` argument.

Concrete implementations in this package:

* :class:`InMemoryStore` — list-of-dicts backed; perfect for tests
  and the smoke harness. No persistence.

The SQL implementation (``SQLiteVoiceMemory``) lives in this package
under ``memory/sqlite_voice.py`` and is wired in via the registry.
"""

from __future__ import annotations

import abc
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


@dataclass(frozen=True)
class Turn:
    """A single conversation turn.

    ``role`` is ``"user" | "assistant" | "tool" | "system"``. For
    ``role == "tool"`` the ``content`` may be the raw tool-result
    payload, and ``tool_call_id`` MUST be set.
    """

    role: str
    content: str
    name: str | None = None
    tool_call_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Fact:
    """A persisted fact with semantic-search metadata.

    ``key`` is the canonical identifier (e.g. ``"music_genre"``);
    ``value`` is the actual data (string, JSON, anything JSONable).
    ``tags`` are searchable keywords; ``confidence`` is a float in
    [0.0, 1.0] tracking how sure the system is about the fact.
    """

    key: str
    value: Any
    tags: tuple[str, ...] = ()
    confidence: float = 1.0


@dataclass(frozen=True)
class Waypoint:
    """A named navigation pose.

    Stores the (x, y) position in the map frame plus the heading
    ``theta`` (radians). ``name`` is the human-readable label
    ("kitchen", "dock", "lobby").
    """

    name: str
    x: float
    y: float
    theta: float = 0.0


@dataclass(frozen=True)
class FAQItem:
    """A single FAQ entry for an event.

    Mirrors the shape used by ``rob_box_voice.core.faq_store``: an
    ``event_id`` tag, a ``question``, an ``answer``, an optional
    ``category`` (defaults to ``"general"``) and an optional
    ``source`` reference (file path / URL).
    """

    event_id: str
    question: str
    answer: str
    category: str = "general"
    source: str = ""


class MemoryStore(abc.ABC):
    """Abstract memory store."""

    name: str = "abstract"

    @abc.abstractmethod
    async def load_recent(
        self,
        scope: str,
        *,
        limit: int = 20,
    ) -> list[Turn]:
        """Return the most recent ``limit`` turns for ``scope``."""

    @abc.abstractmethod
    async def append_turn(self, scope: str, turn: Turn) -> None:
        """Append ``turn`` to ``scope``. Idempotent on (role, content)."""

    @abc.abstractmethod
    async def save_fact(self, scope: str, fact: Fact) -> None:
        """Persist ``fact`` under ``scope``."""

    @abc.abstractmethod
    async def search_facts(
        self,
        scope: str,
        query: str,
        *,
        top_k: int = 5,
    ) -> list[Fact]:
        """Return up to ``top_k`` facts matching ``query``."""

    # ── Waypoints (global navigation memory) ─────────────────────────

    @abc.abstractmethod
    async def save_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
    ) -> None:
        """Persist (or overwrite) a named pose in navigation memory.

        ``name`` is the user-facing label; coordinates are in the
        map frame. Storing the same ``name`` twice replaces the
        previous pose (upsert by name).
        """

    @abc.abstractmethod
    async def list_waypoints(self) -> list[Waypoint]:
        """Return every saved waypoint, sorted by name ascending."""

    @abc.abstractmethod
    async def delete_waypoint(self, name: str) -> bool:
        """Remove a single waypoint by name.

        Returns ``True`` if a row was removed, ``False`` if no
        waypoint with that name existed.
        """

    @abc.abstractmethod
    async def clear_waypoints(self) -> int:
        """Remove every saved waypoint.

        Returns the number of rows removed (0 if the table was
        already empty).
        """

    # ── FAQ (per-event knowledge base) ───────────────────────────────

    @abc.abstractmethod
    async def load_faq(
        self,
        event_id: str,
        items: Iterable[Mapping[str, Any]],
    ) -> int:
        """Replace FAQ rows for ``event_id`` with ``items``.

        ``items`` is an iterable of mappings with at least
        ``question`` and ``answer`` keys; ``category`` and
        ``source`` are optional. Any rows previously stored for
        the same ``event_id`` are deleted first. Returns the
        number of rows actually inserted.
        """

    @abc.abstractmethod
    async def search_faq(
        self,
        event_id: str,
        query: str,
        *,
        limit: int = 5,
    ) -> list[FAQItem]:
        """Return up to ``limit`` FAQ items for ``event_id`` matching ``query``.

        Implementations should do a best-effort keyword search
        over both ``question`` and ``answer``. Empty / blank
        ``query`` returns an empty list.
        """

    # ── Event profile (singleton active-event context) ──────────────

    @abc.abstractmethod
    async def set_event_profile(self, profile: Mapping[str, Any]) -> None:
        """Store ``profile`` as the active event context.

        There is at most one event profile at a time — calling
        ``set_event_profile`` overwrites any previous value.
        """

    @abc.abstractmethod
    async def get_event_profile(self) -> dict[str, Any] | None:
        """Return the active event profile, or ``None`` if unset."""

    async def aclose(self) -> None:
        """Release resources. Default no-op."""
        return None


class InMemoryStore(MemoryStore):
    """In-memory implementation of :class:`MemoryStore`.

    Thread-unsafe by design — single event loop, single scope access.
    Used by tests and the dummy harnesses to keep the smoke path
    dependency-free.

    The recent-turn buffer is a ``deque`` with a hard cap so a
    runaway test doesn't blow out memory.
    """

    name = "in_memory"

    def __init__(self, *, max_recent: int = 1000) -> None:
        self._turns: dict[str, deque[Turn]] = {}
        self._facts: dict[str, list[Fact]] = {}
        self._waypoints: dict[str, Waypoint] = {}
        self._faq: dict[str, list[FAQItem]] = {}
        self._event_profile: dict[str, Any] | None = None
        self._max_recent = max_recent

    async def load_recent(
        self,
        scope: str,
        *,
        limit: int = 20,
    ) -> list[Turn]:
        """Return the most recent ``limit`` turns for ``scope`` (newest first)."""
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")
        bucket = self._turns.get(scope, ())
        # Return most-recent first; tests assert ordering.
        return list(reversed(list(bucket)[-limit:]))

    async def append_turn(self, scope: str, turn: Turn) -> None:
        """Append ``turn`` to ``scope``; oldest are dropped at the deque cap."""
        bucket = self._turns.setdefault(scope, deque(maxlen=self._max_recent))
        bucket.append(turn)

    async def save_fact(self, scope: str, fact: Fact) -> None:
        """Persist ``fact`` under ``scope``, replacing any same-key fact."""
        bucket = self._facts.setdefault(scope, [])
        # Replace existing fact with the same key (idempotent).
        for index, existing in enumerate(bucket):
            if existing.key == fact.key:
                bucket[index] = fact
                return
        bucket.append(fact)

    async def search_facts(
        self,
        scope: str,
        query: str,
        *,
        top_k: int = 5,
    ) -> list[Fact]:
        """Return up to ``top_k`` facts whose key/tags overlap ``query``."""
        if top_k <= 0:
            raise ValueError(f"top_k must be positive, got {top_k}")
        if not query:
            raise ValueError("query must be a non-empty string")
        query_tokens = {token.lower() for token in query.split()}
        bucket = self._facts.get(scope, [])
        scored: list[tuple[int, Fact]] = []
        for fact in bucket:
            haystack = {fact.key.lower()}
            haystack.update(tag.lower() for tag in fact.tags)
            score = len(query_tokens & haystack)
            if score > 0:
                scored.append((score, fact))
        scored.sort(key=lambda pair: (-pair[0], pair[1].key))
        return [fact for _, fact in scored[:top_k]]

    # ── Waypoints ────────────────────────────────────────────────

    async def save_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
    ) -> None:
        """Upsert a waypoint by name (last write wins)."""
        if not name:
            raise ValueError("waypoint name must be a non-empty string")
        self._waypoints[name] = Waypoint(name=name, x=x, y=y, theta=theta)

    async def list_waypoints(self) -> list[Waypoint]:
        """Return every waypoint sorted by name."""
        return [self._waypoints[name] for name in sorted(self._waypoints)]

    async def delete_waypoint(self, name: str) -> bool:
        """Remove a waypoint by name; True if a row was deleted."""
        return self._waypoints.pop(name, None) is not None

    async def clear_waypoints(self) -> int:
        """Wipe all waypoints; returns the count removed."""
        count = len(self._waypoints)
        self._waypoints.clear()
        return count

    # ── FAQ ──────────────────────────────────────────────────────

    async def load_faq(
        self,
        event_id: str,
        items: Iterable[Mapping[str, Any]],
    ) -> int:
        """Replace all FAQ rows for ``event_id`` with ``items``.

        Existing rows for the same ``event_id`` are dropped first.
        """
        if not event_id:
            raise ValueError("event_id must be a non-empty string")
        # Drop existing rows for this event
        self._faq.pop(event_id, None)
        inserted = 0
        for item in items:
            question = item.get("question", "") if isinstance(item, Mapping) else ""
            answer = item.get("answer", "") if isinstance(item, Mapping) else ""
            if not question and not answer:
                continue  # skip malformed rows
            self._faq.setdefault(event_id, []).append(
                FAQItem(
                    event_id=event_id,
                    question=question,
                    answer=answer,
                    category=(
                        item.get("category", "general")
                        if isinstance(item, Mapping)
                        else "general"
                    ),
                    source=(
                        item.get("source", "")
                        if isinstance(item, Mapping)
                        else ""
                    ),
                )
            )
            inserted += 1
        return inserted

    async def search_faq(
        self,
        event_id: str,
        query: str,
        *,
        limit: int = 5,
    ) -> list[FAQItem]:
        """Keyword search across question + answer for ``event_id``."""
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")
        if not query or not query.strip():
            return []
        query_tokens = [token.lower() for token in query.split() if token]
        if not query_tokens:
            return []
        bucket = self._faq.get(event_id, [])
        scored: list[tuple[int, FAQItem]] = []
        for item in bucket:
            haystack = (item.question + " " + item.answer).lower()
            score = sum(1 for token in query_tokens if token in haystack)
            if score > 0:
                scored.append((score, item))
        # Newest match first on tie; tests assert ordering is stable.
        scored.sort(key=lambda pair: (-pair[0], pair[1].question))
        return [item for _, item in scored[:limit]]

    # ── Event profile ────────────────────────────────────────────

    async def set_event_profile(self, profile: Mapping[str, Any]) -> None:
        """Store the active event profile (overwrites any previous)."""
        self._event_profile = dict(profile)

    async def get_event_profile(self) -> dict[str, Any] | None:
        """Return the active event profile or ``None`` if unset."""
        return None if self._event_profile is None else dict(self._event_profile)

    # ---------- test helpers ------------------------------------------------

    def all_turns(self, scope: str) -> Iterable[Turn]:
        """Return every turn in ``scope`` (oldest first). Test-only."""
        return list(self._turns.get(scope, ()))


__all__ = [
    "Turn",
    "Fact",
    "Waypoint",
    "FAQItem",
    "MemoryStore",
    "InMemoryStore",
]
