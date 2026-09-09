"""Unit tests for :class:`VoiceMemoryAdapter` (ADR-0055 Phase 1).

The adapter is a sync facade over :class:`SQLiteVoiceMemory` that exposes
the legacy ``VoiceMemory`` API (``save_turn`` / ``save_fact`` / ``search``
/ ``get_context`` / ``format_facts_for_prompt`` / ``get_stats`` /
``teardown``) so the MCP server can swap to a single harness DB without
touching the call-sites of the tools in ``rob_box_mcp_tools``.

These tests use ``:memory:`` so no file I/O — and exercise the public
contract end-to-end (each method's happy path + the most-likely regression
classes: per-speaker key isolation, search ranking, turn-noop warn).

References
----------
* ADR-0055 §2.3 — adapter contract.
* ADR-0055 §6.1 — acceptance test list.
* ``memory/voice_memory_adapter.py`` docstring (sync facade rationale).
"""
from __future__ import annotations

import asyncio
import logging
import os
import tempfile
import uuid

import pytest

from rob_box_harness.memory.voice_memory_adapter import (
    LEGACY_FACTS_SCOPE,
    VoiceMemoryAdapter,
)


def _run(coro):
    """asyncio.run per call — same idiom as ``test_sqlite_voice_memory.py``."""
    return asyncio.run(coro)


def _make_adapter(db_path: str | None = None) -> VoiceMemoryAdapter:
    """Fresh adapter; the SQLite file is owned by the caller (test or tmp)."""
    if db_path is None:
        # Use a unique per-test tmpfile to avoid cross-test WAL/SHM pollution.
        tmpdir = tempfile.mkdtemp(prefix="vma-test-")
        db_path = os.path.join(tmpdir, f"voice-{uuid.uuid4().hex}.db")
    return VoiceMemoryAdapter(db_path=db_path)


# ── Lifecycle / shape ──────────────────────────────────────────────


class TestLifecycle:
    """Adapter can be constructed and torn down without touching the DB."""

    def test_construct_with_path(self) -> None:
        adapter = _make_adapter()
        assert adapter._db_path.endswith(".db")  # noqa: SLF001 — sanity
        assert adapter._store is None            # noqa: SLF001 — lazy

    def test_teardown_is_idempotent_without_init(self) -> None:
        adapter = _make_adapter()
        # Must not raise even if init was never called.
        adapter.teardown()
        adapter.teardown()


# ── save_turn (DEPRECATED no-op, ADR-0018 honest-warn) ─────────────


class TestSaveTurnDeprecated:
    """``save_turn`` is a no-op per Shifu directive 2026-09-02.

    The contract: returns ``-1``, logs a WARNING (not silent), does NOT
    write to disk. Phase 2 of ADR-0055 will re-introduce a real turns
    table; until then the stub is the only legal behaviour.
    """

    def test_returns_minus_one(self) -> None:
        adapter = _make_adapter()
        out = adapter.save_turn("user", "привет", speaker_id="42")
        assert out == -1

    def test_logs_warning(self, caplog: pytest.LogCaptureFixture) -> None:
        adapter = _make_adapter()
        with caplog.at_level(logging.WARNING, logger="rob_box_harness.memory.voice_memory_adapter"):
            adapter.save_turn("assistant", "world")
        assert any("deprecated" in r.message for r in caplog.records)

    def test_does_not_create_turns_table(self) -> None:
        """save_turn must NOT touch the DB. The harness store has no
        ``turns`` table at all (Shifu directive 02.09.2026) — a stray
        INSERT here would crash the schema."""
        adapter = _make_adapter()
        adapter.save_turn("user", "x")
        adapter.save_turn("assistant", "y", speaker_id="0")
        # The lazy store was never even opened.
        assert adapter._store is None  # noqa: SLF001


# ── save_fact (per-speaker key isolation, ADR-0013 incremental) ─────


class TestSaveFact:
    """``save_fact`` writes into ``facts`` under ``mcp:legacy`` scope,
    isolating per speaker via the ``category@speaker:<id>`` key pattern.
    """

    def test_persists_to_facts_table(self) -> None:
        adapter = _make_adapter()
        rid = adapter.save_fact("любит лыжи", category="hobby", speaker_id="0")
        assert isinstance(rid, int) and rid >= 1

    def test_per_speaker_key_isolation(self) -> None:
        """Two speakers with the same category must NOT clobber each
        other — issue #1770 regression coverage."""
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        adapter.save_fact("Саша любит музыку", category="hobby", speaker_id="1")

        # Direct DB inspection (bypass the adapter so we know what's
        # actually persisted, not what search returns).
        store = adapter._get_store()  # noqa: SLF001 — test introspection
        _run(store.init())

        def _count(conn):
            return conn.execute(
                "SELECT key, value FROM facts WHERE scope = ? ORDER BY key",
                (LEGACY_FACTS_SCOPE,),
            ).fetchall()

        rows = _run(store._run_sync(_count))  # noqa: SLF001
        # Two distinct rows — one per speaker.
        keys = [r["key"] for r in rows]
        assert "hobby@speaker:0" in keys
        assert "hobby@speaker:1" in keys
        # And both values are present.
        values = {r["key"]: r["value"] for r in rows}
        assert values["hobby@speaker:0"] == "Денчик любит лыжи"
        assert values["hobby@speaker:1"] == "Саша любит музыку"

    def test_global_fact_when_no_speaker(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("все любят пятницу", category="mood")
        # Same key, no @-suffix — global fact, not per-speaker.
        store = adapter._get_store()  # noqa: SLF001
        _run(store.init())

        def _row(conn):
            return conn.execute(
                "SELECT key, value FROM facts WHERE scope = ?",
                (LEGACY_FACTS_SCOPE,),
            ).fetchone()

        row = _run(store._run_sync(_row))  # noqa: SLF001
        assert row["key"] == "mood"
        assert row["value"] == "все любят пятницу"

    def test_upsert_last_write_wins(self) -> None:
        """Re-saving with the same (category, speaker) replaces the
        previous fact — same semantic as ``SQLiteVoiceMemory.save_fact``."""
        adapter = _make_adapter()
        adapter.save_fact("first", category="hobby", speaker_id="0")
        adapter.save_fact("second", category="hobby", speaker_id="0")

        store = adapter._get_store()  # noqa: SLF001
        _run(store.init())

        def _count(conn):
            return conn.execute(
                "SELECT COUNT(*) AS c FROM facts "
                "WHERE scope = ? AND key = ?",
                (LEGACY_FACTS_SCOPE, "hobby@speaker:0"),
            ).fetchone()["c"]

        n = _run(store._run_sync(_count))  # noqa: SLF001
        assert n == 1, "expected upsert to leave exactly one row"


# ── search ─────────────────────────────────────────────────────────


class TestSearch:
    def test_empty_query_returns_empty(self) -> None:
        adapter = _make_adapter()
        assert adapter.search("") == []
        assert adapter.search("   ") == []

    def test_finds_fact_by_value(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        hits = adapter.search("лыжи", limit=5, speaker_id="0")
        assert len(hits) == 1
        assert hits[0]["role"] == "fact"
        assert "лыжи" in hits[0]["content"]
        assert hits[0]["speaker_id"] == "0"
        assert hits[0]["source"] == "facts"

    def test_speaker_id_filter_excludes_others(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        adapter.save_fact("Саша любит музыку", category="hobby", speaker_id="1")
        hits = adapter.search("любит", limit=10, speaker_id="0")
        speakers = {h["speaker_id"] for h in hits}
        assert speakers == {"0"}, f"expected only speaker 0, got {speakers}"

    def test_no_speaker_returns_all(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        adapter.save_fact("общая любовь к пятницам", category="mood")
        hits = adapter.search("люб", limit=10)
        assert len(hits) >= 2
        contents = {h["content"] for h in hits}
        assert any("Денчик" in c for c in contents)
        assert any("пятниц" in c for c in contents)

    def test_respects_limit(self) -> None:
        adapter = _make_adapter()
        # Five DISTINCT rows — each save uses a different category so the
        # ``(scope, key)`` upsert in SQLiteVoiceMemory does not collapse
        # them. With five real rows, ``search(..., limit=2)`` must return
        # exactly 2 hits.
        for i in range(5):
            adapter.save_fact(
                f"факт номер {i}",
                category=f"misc-{i}",
                speaker_id=str(i),  # unique per-iteration key
            )
        hits = adapter.search("факт", limit=2)
        assert len(hits) == 2


# ── get_context / format_facts_for_prompt / get_stats ──────────────


class TestGetContext:
    def test_shape(self) -> None:
        adapter = _make_adapter()
        ctx = adapter.get_context(limit=10)
        assert set(ctx.keys()) == {
            "recent_turns",
            "facts",
            "total_turns",
            "sessions",
            "vec_enabled",
            "current_session",
        }
        assert ctx["recent_turns"] == []
        assert ctx["total_turns"] == 0
        assert ctx["sessions"] == 0
        assert ctx["vec_enabled"] is False
        assert ctx["current_session"] is None

    def test_with_speaker_id_facts(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        ctx = adapter.get_context(limit=10, speaker_id="0")
        assert len(ctx["facts"]) == 1
        assert "лыжи" in ctx["facts"][0]["value"]


class TestFormatFactsForPrompt:
    def test_empty(self) -> None:
        adapter = _make_adapter()
        assert adapter.format_facts_for_prompt(speaker_id="0") == ""

    def test_header_and_dash_lines(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
        adapter.save_fact("Денчик — инженер", category="job", speaker_id="0")
        out = adapter.format_facts_for_prompt(speaker_id="0")
        assert out.startswith("Known user facts:")
        # One dash-prefixed line per fact.
        body = out.splitlines()[1:]
        assert len(body) == 2
        assert all(line.startswith("- ") for line in body)


class TestGetStats:
    def test_shape_and_counts(self) -> None:
        adapter = _make_adapter()
        adapter.save_fact("foo", category="hobby", speaker_id="0")
        adapter.save_fact("bar", category="hobby", speaker_id="1")
        stats = adapter.get_stats()
        assert stats["turn_count"] == 0
        assert stats["session_count"] == 0
        assert stats["fact_count"] == 2
        assert stats["vec_enabled"] is False
        assert stats["vec_count"] == 0
        assert stats["ollama_available"] is False
        assert stats["db_size_kb"] >= 0
        assert stats["current_session"] is None


# ── Persistence: facts survive a fresh adapter on the same DB ───────


class TestPersistence:
    """ADR-0055 §2.1 — single DB on /data/, no per-process in-memory
    cache. A second adapter on the same file must read the facts the
    first adapter wrote.
    """

    def test_second_adapter_sees_writes(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix="vma-persist-")
        db_path = os.path.join(tmpdir, "voice.db")
        try:
            a1 = VoiceMemoryAdapter(db_path=db_path)
            a1.save_fact("Денчик любит лыжи", category="hobby", speaker_id="0")
            a1.teardown()

            a2 = VoiceMemoryAdapter(db_path=db_path)
            hits = a2.search("лыжи", speaker_id="0")
            assert len(hits) == 1
            assert "лыжи" in hits[0]["content"]
            a2.teardown()
        finally:
            # Best-effort cleanup; tmpdir is on /tmp so OS GC will sweep.
            import shutil

            shutil.rmtree(tmpdir, ignore_errors=True)
