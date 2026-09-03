"""SQLiteVoiceMemory — persistent SQLite-backed MemoryStore.

Implements the MemoryStore ABC from ``rob_box_harness.memory`` with
a local SQLite database. Uses ``asyncio.to_thread`` to avoid blocking
the event loop on synchronous sqlite3 I/O.

Per ADR-0001 §2.4.3: scoped facts and robot-global state (waypoints,
FAQ, event profile). Conversation turns are NOT persisted (Shifu
Directive 2026-09-02) — they live in an in-memory sliding window.

Thread safety
-------------
The store owns a single shared connection (``check_same_thread=False``)
and all SQLite work is pushed to worker threads via ``asyncio.to_thread``.
To keep that safe, every public operation runs through ``_run_sync``:
the whole statement sequence (SELECTs, INSERTs, COMMIT) executes in one
worker thread while holding ``self._lock``. This prevents the classic
``sqlite3.OperationalError: cannot start a transaction within a
transaction`` caused by two coroutines interleaving statements on the
same connection (one thread's implicit BEGIN is still open when the
other thread runs another write statement). A small retry loop also
handles a leftover open transaction from a crashed neighbour call.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sqlite3
import threading
import time
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, Callable

from rob_box_harness.memory import Fact, FAQItem, MemoryStore, Waypoint

_logger = logging.getLogger(__name__)

# Max retries for a transaction-conflict on the shared connection. The
# conflict means a previous call left a transaction open; we roll back
# and re-run the whole operation. Kept small (2 retries) — under the
# connection lock the conflict should essentially never happen.
_TXN_CONFLICT_MSG = "cannot start a transaction within a transaction"
_MAX_TXN_RETRIES = 2
_TXN_RETRY_DELAY_S = 0.05

# ── SQL table DDL ──────────────────────────────────────────────

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
            ":memory:"
            if db_path == ":memory:"
            else str(Path(db_path).expanduser().resolve())
        )
        self._conn: sqlite3.Connection | None = None
        self._initialized: bool = False
        # Guards the single shared connection. All SQLite work is pushed
        # to worker threads via ``asyncio.to_thread``; without a lock,
        # concurrent coroutines can run statements on the same connection
        # from different threads at the same time, which surfaces as
        # ``sqlite3.OperationalError: cannot start a transaction within a
        # transaction`` (one thread's implicit BEGIN is still open when the
        # other thread tries to run another write statement).
        self._lock = threading.RLock()

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
        await self._run_sync(lambda conn: conn.execute("PRAGMA journal_mode=WAL;"))

        # Create schema
        await self._run_sync(lambda conn: conn.execute(_FACTS_DDL))
        await self._run_sync(lambda conn: conn.execute(_WAYPOINTS_DDL))
        await self._run_sync(lambda conn: conn.execute(_FAQ_ITEMS_DDL))
        await self._run_sync(lambda conn: conn.execute(_EVENT_PROFILE_DDL))
        for idx_ddl in _INDEXES_DDL:
            await self._run_sync(lambda conn, d=idx_ddl: conn.execute(d))

        self._initialized = True
        _logger.info("SQLiteVoiceMemory initialized at %s", self._db_path)

    async def teardown(self) -> None:
        """Close the database connection. Idempotent."""
        if not self._initialized or self._conn is None:
            return

        async def _close() -> None:
            def _sync_close() -> None:
                with self._lock:
                    conn = self._conn
                    if conn is not None:
                        conn.close()

            await asyncio.to_thread(_sync_close)
            self._conn = None

        await _close()
        self._initialized = False
        _logger.info("SQLiteVoiceMemory torndown")

    # ── helpers ─────────────────────────────────────────────────

    async def _run_sync(self, fn: Callable[[sqlite3.Connection], Any]) -> Any:
        """Run ``fn(conn)`` in a worker thread while holding the connection lock.

        The whole operation (SELECTs, INSERTs, COMMIT) executes in a single
        worker thread under the lock, so concurrent coroutines cannot
        interleave statements on the shared connection. If a leftover
        transaction blocks the start of the operation, it is rolled back
        and retried (see ``_MAX_TXN_RETRIES`` / ``_TXN_RETRY_DELAY_S``).
        """
        if self._conn is None:
            raise RuntimeError("SQLiteVoiceMemory not initialised — call init() first")
        conn: sqlite3.Connection = self._conn

        def _locked() -> Any:
            with self._lock:
                for attempt in range(_MAX_TXN_RETRIES + 1):
                    try:
                        return fn(conn)
                    except sqlite3.OperationalError as exc:
                        if (
                            _TXN_CONFLICT_MSG not in str(exc)
                            or attempt >= _MAX_TXN_RETRIES
                        ):
                            raise
                        _logger.warning(
                            "SQLite transaction conflict on %s (attempt %d/%d): %s",
                            getattr(fn, "__name__", "operation"),
                            attempt + 1,
                            _MAX_TXN_RETRIES,
                            exc,
                        )
                        self._safe_rollback(conn)
                        time.sleep(_TXN_RETRY_DELAY_S * (attempt + 1))
                raise RuntimeError("unreachable")  # pragma: no cover

        return await asyncio.to_thread(_locked)

    def _safe_rollback(self, conn: sqlite3.Connection) -> None:
        """Best-effort rollback of a leftover transaction (lock held)."""
        try:
            conn.rollback()
        except sqlite3.Error:
            pass

    # ── MemoryStore ABC ─────────────────────────────────────────

    async def save_fact(self, scope: str, fact: Fact) -> None:
        """Persist ``fact`` under ``scope``, replacing existing fact with the same key.

        The legacy ``INSERT OR REPLACE`` was a no-op upsert because the
        ``facts`` table has no UNIQUE constraint on ``(scope, key)`` —
        repeated saves created duplicate rows and readers returned stale
        values (issue #1077 speaker profiles). Now we DELETE the old row
        first, then INSERT: idempotent regardless of schema. Uses the
        thread-safe ``_run_sync`` wrapper (commit+lock) introduced in
        issue #1086 — see also ``_append``.
        """

        def _save(conn: sqlite3.Connection) -> None:
            value_str = (
                json.dumps(fact.value, ensure_ascii=False)
                if not isinstance(fact.value, str)
                else fact.value
            )
            metadata_json = json.dumps(
                {"tags": list(fact.tags), "confidence": fact.confidence},
                ensure_ascii=False,
            )
            conn.execute(
                "DELETE FROM facts WHERE scope = ? AND key = ?",
                (scope, fact.key),
            )
            conn.execute(
                "INSERT INTO facts (key, value, scope, metadata_json) "
                "VALUES (?, ?, ?, ?)",
                (fact.key, value_str, scope, metadata_json),
            )
            conn.commit()

        await self._run_sync(_save)

    async def clear_facts(self, scope: str) -> int:
        """Remove every fact for ``scope``; returns the number of rows removed.

        Issue W5-4 — используется ``merge_speaker_facts()`` для очистки
        исходного scope после переноса фактов в основной профиль.
        """

        def _clear(conn: sqlite3.Connection) -> int:
            cursor = conn.execute("DELETE FROM facts WHERE scope = ?", (scope,))
            conn.commit()
            return cursor.rowcount

        return await self._run_sync(_clear)

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

        def _search(conn: sqlite3.Connection) -> list[Fact]:
            pattern = f"%{query}%"
            cursor = conn.execute(
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

        return await self._run_sync(_search)

    async def list_facts(
        self,
        scope: str,
        *,
        limit: int = 50,
    ) -> list[Fact]:
        """Return up to ``limit`` facts stored under ``scope`` (newest first).

        Direct scan without a query — used to load ALL speaker facts into
        the LLM context (issue #1077). Thread-safe via ``_run_sync``
        (issue #1086).
        """
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")

        def _list(conn: sqlite3.Connection) -> list[Fact]:
            cursor = conn.execute(
                "SELECT key, value, metadata_json FROM facts "
                "WHERE scope = ? ORDER BY created_at DESC, id DESC LIMIT ?",
                (scope, limit),
            )
            facts = []
            for row in cursor.fetchall():
                meta = json.loads(row["metadata_json"]) if row["metadata_json"] else {}
                value = row["value"]
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

        return await self._run_sync(_list)

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

        def _save(conn: sqlite3.Connection) -> None:
            conn.execute(
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
            conn.commit()

        await self._run_sync(_save)

    async def list_waypoints(self) -> list[Waypoint]:
        """Return every waypoint sorted by name."""

        def _list(conn: sqlite3.Connection) -> list[Waypoint]:
            cursor = conn.execute(
                "SELECT name, x, y, theta FROM waypoints ORDER BY name ASC"
            )
            return [
                Waypoint(name=row["name"], x=row["x"], y=row["y"], theta=row["theta"])
                for row in cursor.fetchall()
            ]

        return await self._run_sync(_list)

    async def delete_waypoint(self, name: str) -> bool:
        """Remove a single waypoint by name; True if a row was deleted."""

        def _delete(conn: sqlite3.Connection) -> bool:
            cursor = conn.execute("DELETE FROM waypoints WHERE name = ?", (name,))
            conn.commit()
            return cursor.rowcount > 0

        return await self._run_sync(_delete)

    async def clear_waypoints(self) -> int:
        """Wipe every waypoint; returns the count removed."""

        def _clear(conn: sqlite3.Connection) -> int:
            cursor = conn.execute("DELETE FROM waypoints")
            conn.commit()
            return cursor.rowcount

        return await self._run_sync(_clear)

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

        def _load(conn: sqlite3.Connection) -> int:
            # Drop existing rows for this event
            conn.execute("DELETE FROM faq_items WHERE event_id = ?", (event_id,))
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
                conn.execute(
                    "INSERT INTO faq_items (event_id, question, answer, category, source) "
                    "VALUES (?, ?, ?, ?, ?)",
                    (event_id, question, answer, category, source),
                )
                inserted += 1
            conn.commit()
            return inserted

        return await self._run_sync(_load)

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

        def _search(conn: sqlite3.Connection) -> list[FAQItem]:
            pattern = f"%{query}%"
            cursor = conn.execute(
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

        return await self._run_sync(_search)

    # ── Event profile ────────────────────────────────────────────

    async def set_event_profile(self, profile: Mapping[str, Any]) -> None:
        """Store the active event profile (overwrites any previous)."""

        def _set(conn: sqlite3.Connection) -> None:
            profile_json = json.dumps(dict(profile), ensure_ascii=False)
            # CHECK (id = 1) guarantees the table holds at most one row;
            # UPSERT pattern keeps that invariant.
            conn.execute(
                "INSERT INTO event_profile (id, profile_json, updated_at) "
                "VALUES (1, ?, strftime('%s', 'now')) "
                "ON CONFLICT(id) DO UPDATE SET "
                "  profile_json = excluded.profile_json, "
                "  updated_at   = strftime('%s', 'now')",
                (profile_json,),
            )
            conn.commit()

        await self._run_sync(_set)

    async def get_event_profile(self) -> dict[str, Any] | None:
        """Return the active event profile, or ``None`` if unset."""

        def _get(conn: sqlite3.Connection) -> dict[str, Any] | None:
            cursor = conn.execute("SELECT profile_json FROM event_profile WHERE id = 1")
            row = cursor.fetchone()
            if row is None:
                return None
            return json.loads(row["profile_json"])

        return await self._run_sync(_get)
