"""VoiceMemoryAdapter — sync API shim from ``VoiceMemory`` to ``SQLiteVoiceMemory``.

ADR-0055 Phase 1 (path consolidation): MCP server (mcp_server.py) and
WaypointStore used to instantiate ``VoiceMemory`` (in
``rob_box_voice.core.voice_memory``) and write to ``/data/voice_memory.db``.
After this adapter is in place, those callers write to the same SQLite file
the dialogue node uses — ``/data/harness_voice.db`` — via the existing
``SQLiteVoiceMemory`` (harness) implementation. This kills the silent
two-writer situation flagged in ``dialogue_node.yaml:97-103``.

API surface — sync, matches ``VoiceMemory`` 1:1:
    * ``save_turn(role, content, speaker_id=None, session_id=None, timestamp=None) -> int``
    * ``save_fact(fact, category='general', speaker_id=None, timestamp=None) -> int``
    * ``search(query, limit=5) -> list[dict]``
    * ``get_stats() -> dict``

Persistence behaviour
---------------------
* ``save_fact`` — persisted to ``facts`` (SQLiteVoiceMemory) under the
  scope ``"mcp:legacy"`` (will become ``"personality"`` after Phase 2).
* ``save_turn`` — DEPRECATED stub. Shifu directive 2026-09-02 forbids
  persisting dialogue turns in production; we honour it here too.
  ``save_turn`` returns ``-1`` and logs a warning. Callers should
  use ``search`` against the ``facts`` table instead. When AgentCore
  step 03 merges, Phase 2 will add an opt-in ``turns`` table behind
  the same adapter — until then, callers writing turns get a clear log
  line, not a silent drop and not an exception.
* ``search`` — over ``facts`` only (turns are not in the store). Returns
  list of dicts shaped like ``{"source": "fact", "score": float,
  "text": str, "ref": {"key": ..., "value": ..., ...}}`` to keep call
  sites working without changes.

Why a per-call ``asyncio.run``
------------------------------
``SQLiteVoiceMemory`` is async (it uses ``asyncio.to_thread`` for
sqlite3). MCP server callers (mcp_server.py, waypoint_store.py,
tools/memory.py) all call ``voice_memory`` from synchronous ROS2 Node
callbacks — wrapping every callsite in ``asyncio.run`` is exactly what
the original ``VoiceMemory`` class did (``VoiceMemory.__init__`` opens
the connection lazily on first use). ``asyncio.run`` per call is
acceptable here because memory writes are infrequent (one write per
user message / per fact save — sub-Hz rate).
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Dict, List, Optional

from rob_box_harness.memory import Fact, SQLiteVoiceMemory

_logger = logging.getLogger(__name__)


# Scope written to ``facts`` for rows originating from MCP tools.
# This stays stable through Phase 1; Phase 2 will re-home them under
# ``"personality"`` once AgentCore (step 03) defines ``agent``.
LEGACY_FACTS_SCOPE = "mcp:legacy"


class VoiceMemoryAdapter:
    """Sync facade exposing ``VoiceMemory`` API on top of ``SQLiteVoiceMemory``.

    Parameters
    ----------
    db_path:
        Path to the SQLite database file (typically
        ``/data/harness_voice.db``). Matches ``VoiceMemory(db_path=...)``
        constructor signature.
    """

    def __init__(self, db_path: str) -> None:
        self._db_path = db_path
        self._store: Optional[SQLiteVoiceMemory] = None

    # ── internal helpers ────────────────────────────────────────

    def _get_store(self) -> SQLiteVoiceMemory:
        if self._store is None:
            self._store = SQLiteVoiceMemory(db_path=self._db_path)
        return self._store

    def _run(self, coro: Any) -> Any:
        """Run an awaitable from a sync context.

        ``asyncio.run`` per call: SQLiteVoiceMemory schedules its work
        through ``asyncio.to_thread`` anyway, so the cost of the extra
        loop is negligible at MCP-write rates.
        """
        return asyncio.run(coro)

    # ── save_turn (DEPRECATED stub) ─────────────────────────────

    def save_turn(
        self,
        role: str,
        content: str,
        *,
        speaker_id: Optional[str] = None,
        session_id: Optional[str] = None,
        timestamp: Optional[float] = None,  # accepted for API parity, unused
    ) -> int:
        """DEPRECATED — turns are not persisted by the unified store.

        Shifu directive 2026-09-02 (``memory/base.py:3-7``) forbids
        persisting dialogue turns in production. Phase 2 of ADR-0055
        (after AgentCore step 03) will add an opt-in ``turns`` table;
        until then this stub keeps call sites working without writing
        anything to disk.

        Returns ``-1`` and logs a warning on every call so silent data
        loss is impossible (ADR-0018).
        """
        _logger.warning(
            "VoiceMemoryAdapter.save_turn is a deprecated no-op "
            "(role=%s, speaker_id=%s). "
            "Turns are not persisted — use save_fact or search instead. "
            "See ADR-0055 §2.3.",
            role,
            speaker_id,
        )
        return -1

    # ── save_fact ───────────────────────────────────────────────

    def save_fact(
        self,
        fact: str,
        *,
        category: str = "general",
        speaker_id: Optional[str] = None,
        timestamp: Optional[float] = None,  # accepted for API parity, unused
    ) -> int:
        """Persist a fact into ``facts`` under ``mcp:legacy`` scope.

        Matches ``VoiceMemory.save_fact`` signature: returns the new
        row id. ``timestamp`` is honoured by the caller contract but not
        persisted here (SQLiteVoiceMemory owns created_at via
        ``strftime('%s', 'now')``). If we ever need a stable timestamp
        later, Phase 2 (ADR-0055) will add an ``on_update`` hook.
        """
        store = self._get_store()

        async def _do() -> int:
            if not store._initialized:  # noqa: SLF001 — internal contract
                await store.init()
            tags_list: List[str] = [category]
            if speaker_id is not None:
                tags_list.append(f"speaker:{speaker_id}")
            fact_obj = Fact(
                key=category,
                value=fact,
                tags=tuple(tags_list),
            )
            await store.save_fact(LEGACY_FACTS_SCOPE, fact_obj)
            # return last inserted id (SELECT MAX(id) — fine, single writer)
            return await store._run_sync(  # noqa: SLF001
                lambda conn: int(
                    conn.execute(
                        "SELECT COALESCE(MAX(id), 0) FROM facts WHERE scope = ?",
                        (LEGACY_FACTS_SCOPE,),
                    ).fetchone()[0]
                )
            )

        return self._run(_do())

    # ── search ──────────────────────────────────────────────────

    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Hybrid search over ``facts`` (turns not in store).

        Returns a list of dicts shaped like ``VoiceMemory.search``:
        ``[{"source": "fact", "score": float, "text": str, "ref": dict}, …]``.
        """
        if not query:
            return []

        async def _do() -> List[Dict[str, Any]]:
            store = self._get_store()
            if not store._initialized:  # noqa: SLF001
                await store.init()
            hits = await store.search(query=query, limit=limit)
            return [
                {
                    "source": hit.source,
                    "score": hit.score,
                    "text": hit.text,
                    "ref": (
                        {
                            "id": hit.ref.id,
                            "key": hit.ref.key,
                            "value": hit.ref.value,
                        }
                        if hasattr(hit.ref, "key")
                        else {"content": getattr(hit.ref, "content", "")}
                    ),
                }
                for hit in hits
            ]

        return self._run(_do())

    # ── get_stats ───────────────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        """Return minimal stats — fact count under legacy scope.

        Matches ``VoiceMemory.get_stats`` shape (dict with at least
        ``"facts"`` key) so dashboards don't crash.
        """
        async def _do() -> Dict[str, Any]:
            store = self._get_store()
            if not store._initialized:  # noqa: SLF001
                await store.init()

            def _stats(conn: Any) -> Dict[str, Any]:
                fact_count = conn.execute(
                    "SELECT COUNT(*) FROM facts WHERE scope = ?",
                    (LEGACY_FACTS_SCOPE,),
                ).fetchone()[0]
                # waypoints belong to the dialogue node, not us — report
                # zero here so the shape matches VoiceMemory.get_stats.
                return {
                    "turns": 0,
                    "facts": fact_count,
                    "fts": False,
                    "vec": False,
                    "db_path": self._db_path,
                }

            return await store._run_sync(_stats)  # noqa: SLF001

        return self._run(_do())

    # ── lifecycle ───────────────────────────────────────────────

    def teardown(self) -> None:
        """Release the SQLiteVoiceMemory connection. Idempotent."""
        if self._store is None:
            return

        async def _do() -> None:
            await self._store.teardown()

        try:
            self._run(_do())
        except Exception as exc:  # noqa: BLE001 — best-effort cleanup
            _logger.warning("VoiceMemoryAdapter teardown failed: %s", exc)
        finally:
            self._store = None