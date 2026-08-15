"""Unit tests for :class:`rob_box_voice.core.voice_memory.VoiceMemory`.

Covers the TASK-036 acceptance surface without requiring a live Ollama
or the sqlite-vec C extension:

* DB creation through ``migrations/002_voice_memory.sql`` (user_version=2)
* ``save_turn`` / ``load_recent_turns`` (chronological, limit,
  ``exclude_current_session``)
* facts CRUD + ``format_facts_for_prompt``
* FTS5 hybrid search on Russian queries
* Ollama graceful degradation — ``search`` still returns FTS5 results
  when the embedder is unavailable; ``save_turn`` never raises.
* hybrid merge logic (dedup by id, sort by score)
"""

from __future__ import annotations

import os
import sqlite3
import time
from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.core.voice_memory import VoiceMemory


def _make_memory(tmp_path, session_id: str = "20260101_000000") -> VoiceMemory:
    db = tmp_path / "voice_memory.db"
    return VoiceMemory(db_path=str(db), session_id=session_id)


# ---------------------------------------------------------------------------
# Migrations / schema
# ---------------------------------------------------------------------------


class TestMigrations:
    def test_db_created_via_migration_sql(self, tmp_path) -> None:
        """Acceptance: voice_memory.db is created through migrations/002_voice_memory.sql."""
        mem = _make_memory(tmp_path)
        try:
            tables = {
                r["name"]
                for r in mem.conn.execute(
                    "SELECT name FROM sqlite_master WHERE type IN ('table','virtual table')"
                ).fetchall()
            }
            assert "voice_turns" in tables
            assert "voice_turns_fts" in tables
            assert "voice_facts" in tables
            assert "voice_memory_meta" in tables
            # Migration version must be tracked
            ver = mem.conn.execute("PRAGMA user_version").fetchone()[0]
            assert ver >= 2, f"expected user_version>=2, got {ver}"
        finally:
            mem.close()

    def test_migrations_dir_resolves_to_repo_migrations(self, tmp_path) -> None:
        """The default migrations dir must point at the real migrations/ folder."""
        mem = _make_memory(tmp_path)
        try:
            assert os.path.isdir(mem.migrations_dir)
            assert os.path.isfile(os.path.join(mem.migrations_dir, "002_voice_memory.sql"))
        finally:
            mem.close()

    def test_fts_triggers_keep_index_in_sync(self, tmp_path) -> None:
        """FTS5 triggers must mirror voice_turns inserts (turns searchable)."""
        mem = _make_memory(tmp_path)
        try:
            mem.save_turn("user", "пользователь спрашивает про кухню")
            # Exact word is searchable (prefix token `"кухню"*` matches "кухню")
            hits = mem._fts_search("кухню", limit=5)
            assert len(hits) == 1
            assert hits[0]["content"] == "пользователь спрашивает про кухню"
            # Stem prefix also matches (morphology via prefix search)
            hits_stem = mem._fts_search("кухн", limit=5)
            assert len(hits_stem) == 1
        finally:
            mem.close()


# ---------------------------------------------------------------------------
# Turns
# ---------------------------------------------------------------------------


class TestTurns:
    def test_save_and_load_recent_turns_chronological(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            mem.save_turn("user", "привет")
            mem.save_turn("assistant", "здравствуй")
            turns = mem.load_recent_turns(limit=10)
            assert [t["role"] for t in turns] == ["user", "assistant"]
            assert [t["content"] for t in turns] == ["привет", "здравствуй"]
        finally:
            mem.close()

    def test_load_recent_respects_limit(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            for i in range(10):
                mem.save_turn("user", f"msg-{i}")
            turns = mem.load_recent_turns(limit=3)
            assert len(turns) == 3
            # Newest 3 in chronological order → msg-7, msg-8, msg-9
            assert [t["content"] for t in turns] == ["msg-7", "msg-8", "msg-9"]
        finally:
            mem.close()

    def test_exclude_current_session(self, tmp_path) -> None:
        mem = _make_memory(tmp_path, session_id="sess_A")
        try:
            mem.save_turn("user", "old turn", session_id="sess_OLD")
            mem.save_turn("user", "current turn")
            turns = mem.load_recent_turns(limit=10, exclude_current_session=True)
            assert len(turns) == 1
            assert turns[0]["content"] == "old turn"
            assert turns[0]["session_id"] == "sess_OLD"
        finally:
            mem.close()

    def test_empty_content_not_saved(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            assert mem.save_turn("user", "   ") == -1
            assert mem.get_stats()["turn_count"] == 0
        finally:
            mem.close()

    def test_recent_turns_cap_for_startup_context(self, tmp_path) -> None:
        """Acceptance: startup context loads up to 15 turns from previous sessions."""
        mem = _make_memory(tmp_path, session_id="current")
        try:
            for i in range(25):
                mem.save_turn("user" if i % 2 == 0 else "assistant", f"turn-{i}",
                              session_id=f"past_{i % 3}")
            turns = mem.load_recent_turns(limit=15, exclude_current_session=True)
            assert len(turns) == 15
        finally:
            mem.close()


# ---------------------------------------------------------------------------
# Facts
# ---------------------------------------------------------------------------


class TestFacts:
    def test_save_and_get_facts(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            mem.save_fact("Пользователя зовут Алексей", category="name")
            mem.save_fact("Предпочитает краткие ответы", category="preference")
            facts = mem.get_facts()
            assert len(facts) == 2
            by_cat = {f["category"]: f["fact"] for f in facts}
            assert by_cat["name"] == "Пользователя зовут Алексей"
            assert by_cat["preference"] == "Предпочитает краткие ответы"
        finally:
            mem.close()

    def test_get_facts_by_category(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            mem.save_fact("a", category="habit")
            mem.save_fact("b", category="general")
            habits = mem.get_facts(category="habit")
            assert len(habits) == 1
            assert habits[0]["fact"] == "a"
        finally:
            mem.close()

    def test_update_and_delete_fact(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            fid = mem.save_fact("old text")
            assert mem.update_fact(fid, "new text") is True
            facts = mem.get_facts()
            assert facts[0]["fact"] == "new text"
            assert mem.delete_fact(fid) is True
            assert mem.get_facts() == []
        finally:
            mem.close()

    def test_format_facts_for_prompt(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            assert mem.format_facts_for_prompt() == ""
            mem.save_fact("Пользователя зовут Алексей")
            block = mem.format_facts_for_prompt()
            assert "Known user facts:" in block
            assert "Пользователя зовут Алексей" in block
        finally:
            mem.close()


# ---------------------------------------------------------------------------
# Hybrid search (FTS5) + graceful degradation
# ---------------------------------------------------------------------------


class TestSearch:
    def _seed(self, mem: VoiceMemory) -> None:
        mem.save_turn("user", "моя любимая кухня находится рядом с гостиной")
        mem.save_turn("assistant", "кухня слева от гостиной")
        mem.save_turn("user", "какая сегодня погода на улице")
        mem.save_turn("assistant", "сегодня солнечно и тепло")

    def test_fts_search_russian_keyword(self, tmp_path) -> None:
        """Acceptance: hybrid FTS search returns relevant Russian results."""
        mem = _make_memory(tmp_path)
        try:
            self._seed(mem)
            hits = mem.search("кухня", limit=5)
            assert len(hits) >= 1
            assert any("кухня" in h["content"] for h in hits)
            assert all(h["source"] == "fts" for h in hits)
        finally:
            mem.close()

    def test_search_empty_query_returns_empty(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            assert mem.search("") == []
            assert mem.search("   ") == []
        finally:
            mem.close()

    def test_graceful_degradation_when_ollama_down(self, tmp_path) -> None:
        """Acceptance: works with FTS5 only when Ollama is down."""
        mem = _make_memory(tmp_path)
        try:
            self._seed(mem)
            # Embedder reports unavailable (Ollama down)
            mem.embedder._available = False
            mem.embedder.embed = MagicMock(return_value=None)  # type: ignore[method-assign]
            hits = mem.search("кухня", limit=5)
            assert len(hits) >= 1
            assert all(h["source"] == "fts" for h in hits)
        finally:
            mem.close()

    def test_save_turn_never_raises_when_ollama_down(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            mem.embedder.embed = MagicMock(return_value=None)  # type: ignore[method-assign]
            rid = mem.save_turn("user", "сохраняем без эмбеддингов")
            assert rid > 0
            assert mem.get_stats()["turn_count"] == 1
        finally:
            mem.close()

    def test_hybrid_supplements_sparse_fts(self, tmp_path) -> None:
        """When FTS returns fewer than FTS_MIN_RESULTS and vec is enabled,
        vector results are merged in (source becomes 'hybrid')."""
        mem = _make_memory(tmp_path)
        try:
            self._seed(mem)
            # Force vector path: embedder available + vec table present.
            mem.embedder._available = True
            mem.embedder.embed = MagicMock(return_value=[0.1, 0.2, 0.3, 0.4])  # type: ignore[method-assign]
            mem._has_vec_table = MagicMock(return_value=True)  # type: ignore[method-assign]
            fake_vec = [
                {"id": 999, "session_id": "s", "role": "user",
                 "content": "векторный результат про кухню", "timestamp": time.time(),
                 "score": 0.9, "source": "vec"},
            ]
            mem._vector_search = MagicMock(return_value=fake_vec)  # type: ignore[method-assign]
            hits = mem.search("кухня", limit=5)
            assert hits, "expected at least one result"
            assert hits[0]["source"] == "hybrid" or any(
                h["source"] == "hybrid" for h in hits
            ), hits
        finally:
            mem.close()

    def test_merge_results_dedup_and_sort(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            fts = [
                {"id": 1, "content": "a", "score": 0.5, "source": "fts"},
                {"id": 2, "content": "b", "score": 0.1, "source": "fts"},
            ]
            vec = [
                {"id": 2, "content": "b", "score": 0.9, "source": "vec"},  # dup id
                {"id": 3, "content": "c", "score": 0.8, "source": "vec"},
            ]
            merged = mem._merge_results(fts, vec, limit=10)
            ids = [m["id"] for m in merged]
            assert ids == [2, 3, 1]  # sorted by score desc
            assert all(m["source"] == "hybrid" for m in merged)
        finally:
            mem.close()


# ---------------------------------------------------------------------------
# Context / stats
# ---------------------------------------------------------------------------


class TestContextAndStats:
    def test_get_context_returns_turns_facts_stats(self, tmp_path) -> None:
        mem = _make_memory(tmp_path, session_id="sess_C")
        try:
            mem.save_turn("user", "в прошлой сессии говорили про кухню",
                          session_id="sess_P")
            mem.save_fact("Пользователя зовут Алексей")
            ctx = mem.get_context(limit=5)
            assert ctx["total_turns"] == 1
            assert ctx["sessions"] >= 1
            assert len(ctx["facts"]) == 1
            assert ctx["current_session"] == "sess_C"
        finally:
            mem.close()

    def test_get_stats_shape(self, tmp_path) -> None:
        mem = _make_memory(tmp_path)
        try:
            stats = mem.get_stats()
            for key in ("turn_count", "session_count", "fact_count",
                        "vec_count", "db_size_kb", "current_session"):
                assert key in stats, f"missing {key}"
        finally:
            mem.close()
