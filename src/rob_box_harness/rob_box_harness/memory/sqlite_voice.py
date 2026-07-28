"""SQLiteVoiceMemory — persistent SQLite-backed MemoryStore.

Implements the MemoryStore ABC from ``rob_box_harness.memory`` with
a local SQLite database. Uses ``asyncio.to_thread`` to avoid blocking
the event loop on synchronous sqlite3 I/O.

Per ADR-0001 §2.4.3: scoped conversation history and facts, with
idempotent turn appends and best-effort semantic search.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sqlite3
import time
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from rob_box_harness.memory import FAQItem, Fact, MemoryStore, Turn, Waypoint

_logger = logging.getLogger(__name__)

# ── SQL table DDL ──────────────────────────────────────────────

_TURNS_DDL = """
CREATE TABLE IF NOT EXISTS turns (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    scope       TEXT    NOT NULL,
    role        TEXT    NOT NULL,
    content     TEXT    NOT NULL,
    name        TEXT,
    tool_call_id TEXT,
    metadata_json TEXT,
    created_at  REAL    NOT NULL DEFAULT (strftime('%s', 'now'))
);
"""

_FACTS_DDL = """
CREATE TABLE IF NOT EXISTS facts (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    key         TEXT    NOT NULL,
    value       TEXT    NOT NULL,
    scope       TEXT    NOT NULL,
    metadata_json TEXT,
    created_at  REAL    NOT NULL DEFAULT (strftime('%s', 'now'))
);
"""

_WAYPOINTS_DDL = """
CREATE TABLE IF NOT EXISTS waypoints (
    name        TEXT    PRIMARY KEY,
    x           REAL    NOT NULL,
    y           REAL    NOT NULL,
    theta       REAL    NOT NULL DEFAULT 0.0,
    created_at  REAL    NOT NULL DEFAULT (strftime('%s', 'now')),
    updated_at  REAL    NOT NULL DEFAULT (strftime('%s', 'now'))
);
"""

_FAQ_ITEMS_DDL = """
CREATE TABLE IF NOT EXISTS faq_items (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id    TEXT    NOT NULL,
    question    TEXT    NOT NULL,
    answer      TEXT    NOT NULL,
    category    TEXT    NOT NULL DEFAULT 'general',
    source      TEXT    NOT NULL DEFAULT '',
    created_at  REAL    NOT NULL DEFAULT (strftime('%s', 'now'))
);
"""

_EVENT_PROFILE_DDL = """
CREATE TABLE IF NOT EXISTS event_profile (
    id           INTEGER PRIMARY KEY CHECK (id = 1),
    profile_json TEXT    NOT NULL,
    updated_at   REAL    NOT NULL DEFAULT (strftime('%s', 'now'))
);
"""

_INDEXES_DDL = [
    "CREATE INDEX IF NOT EXISTS idx_turns_scope ON turns(scope, created_at);",
    "CREATE INDEX IF NOT EXISTS idx_facts_scope_key ON facts(scope, key);",
    "CREATE INDEX IF NOT EXISTS idx_faq_event ON faq_items(event_id);",
]


class SQLiteVoiceMemory(MemoryStore):
    """SQLite-backed persistent conversation history and facts.

    Parameters
    ----------
    db_path:
        Path to the SQLite database file. ``:memory:`` for in-memory
        (useful for tests). ``~`` is expanded to the user's home dir.
        Defaults to ``~/.rob_box/voice.db``.
    """

    name = "sqlite_voice"

    def __init__(self, db_path: str = "~/.rob_box/voice.db") -> None:
        self._db_path: str = (
            ":memory:" if db_path == ":memory:"
            else str(Path(db_path).expanduser().resolve())
        )
        self._conn: sqlite3.Connection | None = None
        self._initialized: bool = False

    # ── lifecycle ───────────────────────────────────────────────

    async def init(self) -> None:
        """Create / open the database and ensure tables exist. Idempotent."""
        if self._initialized:
            return

        # Ensure parent directory exists (skip for :memory:)
        if self._db_path != ":memory:":
            os.makedirs(os.path.dirname(self._db_path), exist_ok=True)

        self._conn = await asyncio.to_thread(
            sqlite3.connect, self._db_path, check_same_thread=False
        )
        self._conn.row_factory = sqlite3.Row

        # Enable WAL for better concurrent read/write (safe for single-writer)
        await self._execute("PRAGMA journal_mode=WAL;")

        # Create schema
        await self._execute(_TURNS_DDL)
        await self._execute(_FACTS_DDL)
        await self._execute(_WAYPOINTS_DDL)
        await self._execute(_FAQ_ITEMS_DDL)
        await self._execute(_EVENT_PROFILE_DDL)
        for idx_ddl in _INDEXES_DDL:
            await self._execute(idx_ddl)

        self._initialized = True
        _logger.info("SQLiteVoiceMemory initialized at %s", self._db_path)

    async def teardown(self) -> None:
        """Close the database connection. Idempotent."""
        if not self._initialized or self._conn is None:
            return
        await asyncio.to_thread(self._conn.close)
        self._conn = None
        self._initialized = False
        _logger.info("SQLiteVoiceMemory torndown")

    # ── helpers ─────────────────────────────────────────────────

    async def _execute(self, sql: str, params: tuple[Any, ...] = ()) -> sqlite3.Cursor:
        """Run SQL in a thread; return cursor."""
        if self._conn is None:
            raise RuntimeError("SQLiteVoiceMemory not initialised — call init() first")
        return await asyncio.to_thread(self._conn.execute, sql, params)

    async def _commit(self) -> None:
        if self._conn is not None:
            await asyncio.to_thread(self._conn.commit)

    # ── MemoryStore ABC ─────────────────────────────────────────

    async def load_recent(
        self,
        scope: str,
        *,
        limit: int = 20,
    ) -> list[Turn]:
        """Return the most recent ``limit`` turns for ``scope`` in chronological order."""
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")
        cursor = await self._execute(
            "SELECT role, content, name, tool_call_id, metadata_json "
            "FROM turns WHERE scope = ? ORDER BY created_at DESC LIMIT ?",
            (scope, limit),
        )
        rows = cursor.fetchall()
        # Reverse DESC result to chronological order
        turns = []
        for row in reversed(rows):
            metadata = json.loads(row["metadata_json"]) if row["metadata_json"] else {}
            turns.append(
                Turn(
                    role=row["role"],
                    content=row["content"],
                    name=row["name"],
                    tool_call_id=row["tool_call_id"],
                    metadata=metadata,
                )
            )
        return turns

    async def append_turn(self, scope: str, turn: Turn) -> bool:
        """Append ``turn`` to ``scope``. Idempotent on (scope, role, content) within ~5 sec.

        Returns ``True`` if inserted, ``False`` if duplicate was detected.
        """
        # Duplicate detection: check same scope+role+content within last 5 seconds
        cutoff = time.time() - 5.0
        cursor = await self._execute(
            "SELECT id FROM turns "
            "WHERE scope = ? AND role = ? AND content = ? AND created_at > ? "
            "LIMIT 1",
            (scope, turn.role, turn.content, cutoff),
        )
        if cursor.fetchone() is not None:
            _logger.debug("Duplicate turn detected for scope=%s role=%s", scope, turn.role)
            return False

        metadata_json = (
            json.dumps(dict(turn.metadata), ensure_ascii=False)
            if turn.metadata
            else None
        )
        await self._execute(
            "INSERT INTO turns (scope, role, content, name, tool_call_id, metadata_json) "
            "VALUES (?, ?, ?, ?, ?, ?)",
            (scope, turn.role, turn.content, turn.name, turn.tool_call_id, metadata_json),
        )
        await self._commit()
        return True

    async def save_fact(self, scope: str, fact: Fact) -> None:
        """Persist ``fact`` under ``scope``, replacing existing fact with the same key."""
        value_str = (
            json.dumps(fact.value, ensure_ascii=False)
            if not isinstance(fact.value, str)
            else fact.value
        )
        metadata_json = json.dumps(
            {"tags": list(fact.tags), "confidence": fact.confidence},
            ensure_ascii=False,
        )
        await self._execute(
            "INSERT OR REPLACE INTO facts (key, value, scope, metadata_json) "
            "VALUES (?, ?, ?, ?)",
            (fact.key, value_str, scope, metadata_json),
        )
        await self._commit()

    async def search_facts(
        self,
        scope: str,
        query: str,
        *,
        top_k: int = 5,
    ) -> list[Fact]:
        """Return up to ``top_k`` facts matching ``query`` via LIKE search.

        Best-effort: uses simple SQL LIKE with wildcards. No semantic
        embedding for P0 (P1 enhancement).
        """
        if top_k <= 0:
            raise ValueError(f"top_k must be positive, got {top_k}")
        if not query:
            raise ValueError("query must be a non-empty string")

        pattern = f"%{query}%"
        cursor = await self._execute(
            "SELECT key, value, metadata_json FROM facts "
            "WHERE scope = ? AND (key LIKE ? OR value LIKE ?) "
            "ORDER BY created_at DESC LIMIT ?",
            (scope, pattern, pattern, top_k),
        )
        rows = cursor.fetchall()
        facts = []
        for row in rows:
            meta = json.loads(row["metadata_json"]) if row["metadata_json"] else {}
            value = row["value"]
            # Attempt JSON decode for structured values
            try:
                value = json.loads(value)
            except (json.JSONDecodeError, TypeError):
                pass
            facts.append(
                Fact(
                    key=row["key"],
                    value=value,
                    tags=tuple(meta.get("tags", ())),
                    confidence=float(meta.get("confidence", 1.0)),
                )
            )
        return facts

    async def aclose(self) -> None:
        """Alias for teardown."""
        await self.teardown()

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
        await self._execute(
            """
            INSERT INTO waypoints (name, x, y, theta, created_at, updated_at)
            VALUES (?, ?, ?, ?, strftime('%s', 'now'), strftime('%s', 'now'))
            ON CONFLICT(name) DO UPDATE SET
                x = excluded.x,
                y = excluded.y,
                theta = excluded.theta,
                updated_at = strftime('%s', 'now')
            """,
            (name, float(x), float(y), float(theta)),
        )
        await self._commit()

    async def list_waypoints(self) -> list[Waypoint]:
        """Return every waypoint sorted by name."""
        cursor = await self._execute(
            "SELECT name, x, y, theta FROM waypoints ORDER BY name ASC"
        )
        return [
            Waypoint(name=row["name"], x=row["x"], y=row["y"], theta=row["theta"])
            for row in cursor.fetchall()
        ]

    async def delete_waypoint(self, name: str) -> bool:
        """Remove a single waypoint by name; True if a row was deleted."""
        cursor = await self._execute(
            "DELETE FROM waypoints WHERE name = ?", (name,)
        )
        await self._commit()
        return cursor.rowcount > 0

    async def clear_waypoints(self) -> int:
        """Wipe every waypoint; returns the count removed."""
        cursor = await self._execute("DELETE FROM waypoints")
        await self._commit()
        return cursor.rowcount

    # ── FAQ ──────────────────────────────────────────────────────

    async def load_faq(
        self,
        event_id: str,
        items: Iterable[Mapping[str, Any]],
    ) -> int:
        """Replace FAQ rows for ``event_id`` with ``items``.

        Existing rows for the same ``event_id`` are deleted first,
        then each item is inserted (skipping entries that lack both
        question and answer). Returns the count actually inserted.
        """
        if not event_id:
            raise ValueError("event_id must be a non-empty string")

        # Drop existing rows for this event
        await self._execute(
            "DELETE FROM faq_items WHERE event_id = ?", (event_id,)
        )
        inserted = 0
        for item in items:
            if not isinstance(item, Mapping):
                continue
            question = item.get("question", "") or ""
            answer = item.get("answer", "") or ""
            if not question and not answer:
                continue
            category = item.get("category", "general") or "general"
            source = item.get("source", "") or ""
            await self._execute(
                "INSERT INTO faq_items (event_id, question, answer, category, source) "
                "VALUES (?, ?, ?, ?, ?)",
                (event_id, question, answer, category, source),
            )
            inserted += 1
        await self._commit()
        return inserted

    async def search_faq(
        self,
        event_id: str,
        query: str,
        *,
        limit: int = 5,
    ) -> list[FAQItem]:
        """Keyword search over ``question`` + ``answer`` for ``event_id``.

        Uses simple SQL LIKE with leading/trailing ``%`` wildcards.
        Returns rows in ``created_at`` DESC order. Empty/blank
        ``query`` returns an empty list.
        """
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")
        if not event_id:
            raise ValueError("event_id must be a non-empty string")
        if not query or not query.strip():
            return []
        pattern = f"%{query}%"
        cursor = await self._execute(
            "SELECT event_id, question, answer, category, source "
            "FROM faq_items "
            "WHERE event_id = ? AND (question LIKE ? OR answer LIKE ?) "
            "ORDER BY created_at DESC LIMIT ?",
            (event_id, pattern, pattern, limit),
        )
        return [
            FAQItem(
                event_id=row["event_id"],
                question=row["question"],
                answer=row["answer"],
                category=row["category"],
                source=row["source"],
            )
            for row in cursor.fetchall()
        ]

    # ── Event profile ────────────────────────────────────────────

    async def set_event_profile(self, profile: Mapping[str, Any]) -> None:
        """Store the active event profile (overwrites any previous)."""
        profile_json = json.dumps(dict(profile), ensure_ascii=False)
        # CHECK (id = 1) guarantees the table holds at most one row;
        # UPSERT pattern keeps that invariant.
        await self._execute(
            "INSERT INTO event_profile (id, profile_json, updated_at) "
            "VALUES (1, ?, strftime('%s', 'now')) "
            "ON CONFLICT(id) DO UPDATE SET "
            "  profile_json = excluded.profile_json, "
            "  updated_at   = strftime('%s', 'now')",
            (profile_json,),
        )
        await self._commit()

    async def get_event_profile(self) -> dict[str, Any] | None:
        """Return the active event profile, or ``None`` if unset."""
        cursor = await self._execute(
            "SELECT profile_json FROM event_profile WHERE id = 1"
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return json.loads(row["profile_json"])
