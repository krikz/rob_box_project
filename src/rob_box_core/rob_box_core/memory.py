"""MemoryStore — persistence port for dialog memory + facts.

The existing `VoiceMemory` (SQLite + FTS5 + optional sqlite-vec) is one
implementation; this module defines the port + a thread-safe in-memory
implementation for tests and short-lived sessions.

P0.4 contract (from `docs/refactoring-plan.md`):

    async def append_turn(role, content, **meta) -> None
    async def load_recent(limit, scope="default") -> list[Turn]
    async def save_fact(fact, category="general") -> int
    async def search_facts(query, limit=5) -> list[Fact]
    async def search(query, limit=5) -> list[MemoryHit]   # hybrid turns + facts

Methods are intentionally async even for the in-memory impl — that way
``AgentSession`` (P1) can treat every store identically without checking the
concrete class.
"""

from __future__ import annotations

import abc
import itertools
import re
import threading
from dataclasses import dataclass, field
from typing import Any, Iterable


# ---------------------------------------------------------------------------
# Value objects
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Turn:
    """One message in the conversation log."""

    id: int
    role: str            # "user" | "assistant" | "tool" | "system"
    content: str
    scope: str = "default"  # user id, chat id, session id, …
    timestamp: float = 0.0
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Fact:
    """A persistent user-fact ("user is allergic to nuts")."""

    id: int
    text: str
    category: str = "general"
    timestamp: float = 0.0


@dataclass(frozen=True)
class MemoryHit:
    """Hybrid search result. `source` is "turn" or "fact"."""

    source: str
    score: float
    text: str
    ref: Turn | Fact


# ---------------------------------------------------------------------------
# Port
# ---------------------------------------------------------------------------


class MemoryStore(abc.ABC):
    """Abstract persistence layer.

    Adapters:
        - InMemoryStore (this module — tests + short-lived sessions)
        - SQLiteVoiceMemory (existing — see ``rob_box_voice.core.voice_memory``)
    """

    @abc.abstractmethod
    async def append_turn(
        self, role: str, content: str, *, scope: str = "default", timestamp: float | None = None, **meta: Any
    ) -> int: ...

    @abc.abstractmethod
    async def load_recent(self, limit: int, *, scope: str = "default") -> list[Turn]: ...

    @abc.abstractmethod
    async def save_fact(self, fact: str, *, category: str = "general", timestamp: float | None = None) -> int: ...

    @abc.abstractmethod
    async def search_facts(self, query: str, *, limit: int = 5) -> list[Fact]: ...

    @abc.abstractmethod
    async def search(self, query: str, *, limit: int = 5) -> list[MemoryHit]: ...

    async def aclose(self) -> None:
        """Release resources. Default impl is no-op."""
        return None


# ---------------------------------------------------------------------------
# In-memory implementation
# ---------------------------------------------------------------------------


_TOKEN_RE = re.compile(r"\w+", flags=re.UNICODE)


def _tokenize(text: str) -> list[str]:
    return [t.lower() for t in _TOKEN_RE.findall(text) if t]


class InMemoryStore(MemoryStore):
    """Thread-safe, ephemeral store. Useful for tests and one-shot sessions.

    Search uses simple bag-of-words overlap; results are scored by the fraction
    of query tokens that appear in the candidate text. This is deliberately
    naive — for real semantic search, plug in `SQLiteVoiceMemory` (which has
    FTS5 + optional sqlite-vec).
    """

    def __init__(self, *, clock_factory=lambda: 0.0) -> None:
        self._turn_id = itertools.count(1)
        self._fact_id = itertools.count(1)
        self._turns: list[Turn] = []
        self._facts: list[Fact] = []
        self._lock = threading.Lock()
        self._clock_factory = clock_factory

    def _now(self) -> float:
        try:
            return float(self._clock_factory())
        except Exception:  # noqa: BLE001
            return 0.0

    # -- turns --------------------------------------------------------------

    async def append_turn(
        self, role: str, content: str, *, scope: str = "default", timestamp: float | None = None, **meta: Any
    ) -> int:
        ts = self._now() if timestamp is None else float(timestamp)
        turn_id = next(self._turn_id)
        with self._lock:
            self._turns.append(
                Turn(
                    id=turn_id,
                    role=role,
                    content=content,
                    scope=scope,
                    timestamp=ts,
                    meta=dict(meta),
                )
            )
        return turn_id

    async def load_recent(self, limit: int, *, scope: str = "default") -> list[Turn]:
        if limit <= 0:
            return []
        with self._lock:
            scoped = [t for t in self._turns if t.scope == scope]
        # Newest first, as the spec says "recent".
        scoped.sort(key=lambda t: t.id, reverse=True)
        return list(reversed(scoped[:limit]))

    # -- facts --------------------------------------------------------------

    async def save_fact(self, fact: str, *, category: str = "general", timestamp: float | None = None) -> int:
        ts = self._now() if timestamp is None else float(timestamp)
        fact_id = next(self._fact_id)
        with self._lock:
            self._facts.append(Fact(id=fact_id, text=fact, category=category, timestamp=ts))
        return fact_id

    async def search_facts(self, query: str, *, limit: int = 5) -> list[Fact]:
        scored = self._score_facts(query)
        return [f for _, f in scored[: max(0, limit)]]

    # -- hybrid search ------------------------------------------------------

    async def search(self, query: str, *, limit: int = 5) -> list[MemoryHit]:
        scored_turns = [(s, t) for s, t in self._score_turns(query)]
        scored_facts = [(s, f) for s, f in self._score_facts(query)]
        merged: list[MemoryHit] = []
        for s, ref in scored_turns + scored_facts:
            src = "turn" if isinstance(ref, Turn) else "fact"
            merged.append(MemoryHit(source=src, score=s, text=ref.text if hasattr(ref, "text") else ref.content, ref=ref))
        merged.sort(key=lambda h: h.score, reverse=True)
        return merged[: max(0, limit)]

    # -- internal scoring ---------------------------------------------------

    def _score(self, query_tokens: list[str], text: str) -> float:
        if not query_tokens:
            return 0.0
        text_tokens = set(_tokenize(text))
        if not text_tokens:
            return 0.0
        hits = sum(1 for q in query_tokens if q in text_tokens)
        return hits / len(query_tokens)

    def _score_turns(self, query: str) -> Iterable[tuple[float, Turn]]:
        qtoks = _tokenize(query)
        if not qtoks:
            return []
        with self._lock:
            snapshot = list(self._turns)
        out: list[tuple[float, Turn]] = []
        for t in snapshot:
            s = self._score(qtoks, t.content)
            if s > 0:
                out.append((s, t))
        out.sort(key=lambda x: (x[0], x[1].id), reverse=True)
        return out

    def _score_facts(self, query: str) -> Iterable[tuple[float, Fact]]:
        qtoks = _tokenize(query)
        if not qtoks:
            return []
        with self._lock:
            snapshot = list(self._facts)
        out: list[tuple[float, Fact]] = []
        for f in snapshot:
            s = self._score(qtoks, f.text)
            if s > 0:
                out.append((s, f))
        out.sort(key=lambda x: (x[0], x[1].id), reverse=True)
        return out


__all__ = ["MemoryStore", "InMemoryStore", "Turn", "Fact", "MemoryHit"]
