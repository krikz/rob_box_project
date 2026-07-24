"""MemoryStore port — per-scope conversation history and facts.

The contract covers the four operations every concrete memory
(``SQLiteVoiceMemory``, ``RedisStore``, ``fake.InMemoryStore``) MUST
expose (ADR-0001 §2.4.3):

* ``load_recent`` — fetch the last N turns for a given scope.
* ``append_turn``  — append a single turn (idempotent).
* ``save_fact``    — persist a structured fact (e.g. "user likes jazz").
* ``search_facts`` — best-effort semantic search over stored facts.

The ``scope`` argument is deliberately a string (not an enum) so
harnesses can pass arbitrary bucket names: ``"user:42"``,
``"chat:tg:9001"``, ``"session:abc"``. The format is harness-specific
and the MemoryStore doesn't need to interpret it.

Concrete implementations in this package:

* :class:`InMemoryStore` — list-of-dicts backed; perfect for tests
  and the smoke harness. No persistence.

The SQL implementation (``SQLiteVoiceMemory``) lives in the
``rob_box_voice`` package and is wired in via the registry.
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

    # ---------- test helpers ------------------------------------------------

    def all_turns(self, scope: str) -> Iterable[Turn]:
        """Return every turn in ``scope`` (oldest first). Test-only."""
        return list(self._turns.get(scope, ()))


__all__ = [
    "Turn",
    "Fact",
    "MemoryStore",
    "InMemoryStore",
]
