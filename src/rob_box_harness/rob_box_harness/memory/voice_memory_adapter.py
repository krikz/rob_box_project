"""VoiceMemoryAdapter — sync API shim from ``VoiceMemory`` to ``SQLiteVoiceMemory``.

ADR-0055 Phase 1 (path consolidation): ``mcp_server.py`` used to
instantiate ``VoiceMemory`` (in ``rob_box_voice.core.voice_memory``) for
long-term facts and write to ``/data/voice_memory.db``. When
``MCP_USE_HARNESS_VOICE_MEMORY=1``, ``mcp_server.py`` instead builds this
adapter and writes facts to the same SQLite file the dialogue node uses —
``/data/harness_voice.db`` — via the existing ``SQLiteVoiceMemory``
(harness) implementation.

Scope note (issue #2000 Phase 2, investigated and NOT done here):
``WaypointStore`` and the voice tools (``tools/memory.py`` FAQ/TrackLibrary
paths) still write to ``/data/voice_memory.db`` and are **not** wired to
this adapter. ``SQLiteVoiceMemory`` already defines its own ``waypoints``
(``name`` PRIMARY KEY) and ``faq_items`` (``created_at``, no FTS5) tables
in ``harness_voice.db``, with schemas incompatible with WaypointStore's
``waypoints`` (``id`` PK, ``map_id`` NOT NULL FK -> ``maps``) and the
legacy FAQStore's ``faq_items`` (``indexed_at``, FTS5 triggers) — pointing
either store at ``harness_voice.db`` today would silently keep whichever
schema was created first and break every write from the other. See the
``sqlite_db_path`` comment in
``docker/vision/config/voice_assistant/dialogue_node.yaml`` and
``docs/adr/0055-voice-memory-db-unify-with-harness.md`` (§1.3, §3) for the
full analysis. Unifying those needs a schema-reconciling adapter of their
own, not a change of this one's scope.

This adapter still kills the silent two-writer situation for *facts*
flagged in ``dialogue_node.yaml:97-103``.

API surface — sync, matches ``VoiceMemory`` 1:1 (so mcp_server.py does
not need anything but an import swap)::

    save_turn(role, content, speaker_id=None, session_id=None,
              timestamp=None) -> int
    save_fact(fact, category='general', speaker_id=None,
              timestamp=None) -> int
    search(query, limit=5, speaker_id=None) -> list[dict]
    get_context(limit=10, query=None, speaker_id=None) -> dict
    format_facts_for_prompt(speaker_id=None) -> str
    get_stats() -> dict
    teardown() -> None

Persistence behaviour
---------------------
* ``save_fact`` — persisted to ``facts`` (SQLiteVoiceMemory) under the
  scope ``"mcp:legacy"`` (will become ``"personality"`` after Phase 2).
* ``save_turn`` — DEPRECATED stub. Shifu directive 2026-09-02 forbids
  persisting dialogue turns in production; we honour it here too.
  ``save_turn`` returns ``-1`` and logs a warning. Callers should
  use ``search`` / ``format_facts_for_prompt`` against the ``facts``
  table instead. When AgentCore step 03 merges, Phase 2 will add an
  opt-in ``turns`` table behind the same adapter — until then, callers
  writing turns get a clear log line, not a silent drop and not an
  exception.
* ``search`` — over ``facts`` only (turns are not in the store). The
  result shape matches what ``tools/memory.py`` expects (``role``,
  ``content``, ``session_id``, ``score``, ``source``). ``speaker_id``
  filters via the ``speaker:<id>`` tag convention that
  ``SQLiteVoiceMemory`` already uses (``base.py:445``).
* ``get_context`` — returns the same dict shape ``MemoryContextTool``
  needs: ``recent_turns`` is empty (turns not persisted), ``facts`` is
  the list of facts for ``speaker_id``. ``total_turns``/``sessions``
  are ``0`` — the dialogue node owns turn counting on its side.
* ``format_facts_for_prompt`` — same output format as ``VoiceMemory``
  so the system prompt block does not change.

Why a per-call ``asyncio.run``
------------------------------
``SQLiteVoiceMemory`` is async (it uses ``asyncio.to_thread`` for
sqlite3). MCP server callers (``mcp_server.py``, ``waypoint_store.py``,
``tools/memory.py``) all call ``voice_memory`` from synchronous ROS 2
Node callbacks — wrapping every callsite in ``asyncio.run`` is exactly
what the original ``VoiceMemory`` class did (``VoiceMemory.__init__``
opens the connection lazily on first use). ``asyncio.run`` per call is
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


def _speaker_tag(speaker_id: Optional[str]) -> Optional[str]:
    """Return the canonical ``speaker:<id>`` tag, or ``None``.

    Mirrors the convention used by ``base.py:445`` (and by
    ``dialogue_node`` save_fact calls).
    """
    if not speaker_id:
        return None
    return f"speaker:{speaker_id}"


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

        Key strategy
        ------------
        ``SQLiteVoiceMemory.save_fact`` upserts by ``(scope, key)``: any
        new save with the same key deletes the previous row. To keep
        multiple speakers' facts in the same scope (issue #1770), we
        fold ``speaker_id`` into the key::

            category                     → global fact (replaces itself)
            category@speaker:<speaker>   → speaker-scoped fact

        This preserves the «last write wins» semantic for the same
        (category, speaker) while keeping one speaker's facts from
        clobbering another's.
        """
        if speaker_id:
            key = f"{category}@{_speaker_tag(speaker_id)}"  # type: ignore[arg-type]
        else:
            key = category

        store = self._get_store()

        async def _do() -> int:
            if not store._initialized:  # noqa: SLF001 — internal contract
                await store.init()
            tags_list: List[str] = [category]
            if speaker_id is not None:
                tags_list.append(_speaker_tag(speaker_id))  # type: ignore[arg-type]
            fact_obj = Fact(
                key=key,
                value=fact,
                tags=tuple(t for t in tags_list if t is not None),
            )
            await store.save_fact(LEGACY_FACTS_SCOPE, fact_obj)
            # Return the row id of the fact we just inserted. SQL semantics:
            # ``save_fact`` does ``DELETE WHERE scope,key`` then ``INSERT``,
            # so MAX(id) is stable across same-key re-saves (single writer).
            return await store._run_sync(  # noqa: SLF001
                lambda conn: int(
                    conn.execute(
                        "SELECT COALESCE(MAX(id), 0) FROM facts "
                        "WHERE scope = ? AND key = ?",
                        (LEGACY_FACTS_SCOPE, key),
                    ).fetchone()[0]
                )
            )

        return self._run(_do())

    # ── search ──────────────────────────────────────────────────

    def search(
        self,
        query: str,
        limit: int = 5,
        speaker_id: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Search facts.

        Returns a list of dicts shaped like the legacy ``VoiceMemory.search``
        result so ``MemorySearchTool`` does not need to know about the
        backing store::

            [{"role": "fact", "content": <value>,
              "session_id": None, "speaker_id": <id|None>,
              "score": <float>, "source": "facts"}, ...]

        ``speaker_id`` filter is enforced by re-scanning the candidate
        set against ``speaker:<id>`` tags — ``SQLiteVoiceMemory.search_facts``
        does not expose a tag predicate so we apply it here. Cost is OK
        because ``search_facts`` already pre-filters via LIKE.
        """
        if not query or not query.strip():
            return []

        async def _do() -> List[Dict[str, Any]]:
            store = self._get_store()
            if not store._initialized:  # noqa: SLF001
                await store.init()
            # Pull a slightly larger candidate pool so the speaker_id
            # post-filter does not underflow the requested ``limit``.
            pool = max(limit * 2, 10)
            hits = await store.search_facts(
                scope=LEGACY_FACTS_SCOPE,
                query=query,
                top_k=pool,
            )
            tag_filter = _speaker_tag(speaker_id)
            out: List[Dict[str, Any]] = []
            for fact in hits:
                if tag_filter is not None and tag_filter not in fact.tags:
                    continue
                # tags may include the speaker filter as well as the
                # category — surface the original ``speaker_id`` if so.
                fact_speaker_id: Optional[str] = None
                for tag in fact.tags:
                    if tag.startswith("speaker:"):
                        fact_speaker_id = tag[len("speaker:"):]
                        break
                out.append(
                    {
                        "id": None,  # not surfaced by MemoryStore ABC
                        "session_id": None,
                        "role": "fact",
                        "content": fact.value
                        if isinstance(fact.value, str)
                        else str(fact.value),
                        "timestamp": None,
                        "speaker_id": fact_speaker_id,
                        "score": float(fact.confidence),
                        "source": "facts",
                        "key": fact.key,
                    }
                )
                if len(out) >= limit:
                    break
            return out

        return self._run(_do())

    # ── get_context (for MemoryContextTool) ─────────────────────

    def get_context(
        self,
        limit: int = 10,
        query: Optional[str] = None,
        speaker_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Return the context bundle ``MemoryContextTool`` expects.

        The dialogue node owns turn persistence (and turns are NOT
        persisted by directive 02.09.2026 anyway), so ``recent_turns``
        is an empty list and ``total_turns`` / ``sessions`` are zero.
        ``facts`` is the list of facts scoped to ``speaker_id``.
        ``vec_enabled`` is always ``False`` (SQLiteVoiceMemory has no
        vector search today).
        """
        async def _do() -> Dict[str, Any]:
            store = self._get_store()
            if not store._initialized:  # noqa: SLF001
                await store.init()
            tag_filter = _speaker_tag(speaker_id)
            pool = max(limit * 4, 20)

            def _read(conn: Any) -> List[Dict[str, Any]]:
                if tag_filter is None:
                    rows = conn.execute(
                        "SELECT key, value, metadata_json FROM facts "
                        "WHERE scope = ? ORDER BY created_at DESC, id DESC LIMIT ?",
                        (LEGACY_FACTS_SCOPE, pool),
                    ).fetchall()
                else:
                    # ``tags`` is stored as JSON in metadata_json; LIKE on
                    # the JSON string is good enough at MCP-write rate
                    # (small pool, indexed by (scope, key)).
                    rows = conn.execute(
                        "SELECT key, value, metadata_json FROM facts "
                        "WHERE scope = ? AND metadata_json LIKE ? "
                        "ORDER BY created_at DESC, id DESC LIMIT ?",
                        (LEGACY_FACTS_SCOPE, f'%"{tag_filter}"%', pool),
                    ).fetchall()
                out: List[Dict[str, Any]] = []
                for row in rows:
                    try:
                        meta = (
                            __import__("json").loads(row["metadata_json"])
                            if row["metadata_json"]
                            else {}
                        )
                    except Exception:  # noqa: BLE001 — defensive
                        meta = {}
                    try:
                        value = (
                            __import__("json").loads(row["value"])
                            if row["value"]
                            else ""
                        )
                    except Exception:  # noqa: BLE001
                        value = row["value"]
                    fact_speaker_id: Optional[str] = None
                    for tag in meta.get("tags", ()):
                        if isinstance(tag, str) and tag.startswith("speaker:"):
                            fact_speaker_id = tag[len("speaker:"):]
                            break
                    out.append(
                        {
                            "key": row["key"],
                            "fact": value,
                            "value": value,
                            "speaker_id": fact_speaker_id,
                            "tags": meta.get("tags", ()),
                            "confidence": float(meta.get("confidence", 1.0)),
                        }
                    )
                return out

            facts_list = await store._run_sync(_read)  # noqa: SLF001
            if query:
                recent_turns = self.search(query=query, limit=limit, speaker_id=speaker_id)
            else:
                recent_turns = []
            return {
                "recent_turns": recent_turns,
                "facts": facts_list[:limit],
                "total_turns": 0,
                "sessions": 0,
                "vec_enabled": False,
                "current_session": None,
            }

        return self._run(_do())

    # ── format_facts_for_prompt ─────────────────────────────────

    def format_facts_for_prompt(self, speaker_id: Optional[str] = None) -> str:
        """Format facts for the system-prompt block.

        Mirrors the ``VoiceMemory`` output shape:

            Known user facts:
            - <fact 1>
            - <fact 2>
        """
        ctx = self.get_context(limit=50, query=None, speaker_id=speaker_id)
        facts = ctx.get("facts", [])
        if not facts:
            return ""
        lines: List[str] = []
        for entry in facts:
            value = entry.get("fact") or entry.get("value")
            if value is None:
                continue
            lines.append(f"- {value}")
        if not lines:
            return ""
        return "Known user facts:\n" + "\n".join(lines)

    # ── get_stats ───────────────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        """Return minimal stats — fact count under legacy scope.

        Matches ``VoiceMemory.get_stats`` shape so dashboards do not
        crash. ``turn_count``/``session_count`` are always 0 (turns are
        not persisted). ``db_size_kb`` and ``vec_enabled`` are filled
        when the underlying SQLite file is reachable.
        """
        import os as _os

        async def _do() -> Dict[str, Any]:
            store = self._get_store()
            if not store._initialized:  # noqa: SLF001
                await store.init()

            def _stats(conn: Any) -> Dict[str, Any]:
                fact_count = conn.execute(
                    "SELECT COUNT(*) FROM facts WHERE scope = ?",
                    (LEGACY_FACTS_SCOPE,),
                ).fetchone()[0]
                return {
                    "turn_count": 0,
                    "session_count": 0,
                    "fact_count": fact_count,
                    "vec_count": 0,
                    "vec_enabled": False,
                    "ollama_available": False,
                    "db_size_kb": (
                        _os.path.getsize(self._db_path) // 1024
                        if _os.path.exists(self._db_path)
                        else 0
                    ),
                    "current_session": None,
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