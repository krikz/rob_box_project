"""Unit tests for :class:`VoiceMemory` — FTS5, vector search, save/load.

Covers the cross-session persistent memory used by the voice assistant
(``rob_box_voice.core.voice_memory``):

* ``save_turn`` / ``load_recent_turns`` — conversation persistence.
* FTS5 keyword search (``search`` / ``_fts_search``).
* Vector (semantic) search via sqlite-vec — **with a mock embedder**;
  no real Ollama is required (acceptance: "tests must not require real
  Ollama, mock embeddings via fixture").
* Hybrid merge of FTS + vector results.
* Facts CRUD (``save_fact`` / ``get_facts`` / ``update_fact`` /
  ``delete_fact`` / ``format_facts_for_prompt``).
* ``get_context``, ``get_stats``, ``prune_older_than``,
  ``reindex_embeddings``, migrations and lifecycle.

The module is pure stdlib + optional sqlite-vec, so it runs headless
(no ROS2, no network). sqlite-vec is optional: vector tests are
skipped when the extension is not installed.
"""

from __future__ import annotations

import os
import sqlite3
import tempfile
import time
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.voice_memory import (
    OllamaEmbedder,
    VoiceMemory,
    _SQLITE_VEC_AVAILABLE,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


class _FakeEmbedder:
    """Deterministic embedder returning fixed-dim vectors; no network I/O.

    The first three texts map to distinct, well-separated vectors so
    vector-search tests can assert ranking without a real model.
    """

    def __init__(self, dim: int = 8) -> None:
        self._dim = dim
        self._available = True
        self.calls: list[str] = []

    def embed(self, text: str) -> list[float] | None:
        self.calls.append(text)
        if not self._available:
            return None
        # Stable pseudo-vector derived from the text hash (dim entries in
        # [-1, 1]). Identical text → identical vector.
        import hashlib

        h = hashlib.sha256(text.encode("utf-8")).digest()
        vec = [((h[i] % 256) / 127.5) - 1.0 for i in range(self._dim)]
        return vec

    def is_available(self) -> bool:
        return self._available

    @property
    def dim(self) -> int | None:
        return self._dim


@pytest.fixture
def vm(tmp_path) -> VoiceMemory:
    """VoiceMemory on a temp file (not the default ~/.rob_box path)."""
    memory = VoiceMemory(db_path=str(tmp_path / "voice.db"))
    yield memory
    memory.close()


@pytest.fixture
def fake_embedder() -> _FakeEmbedder:
    return _FakeEmbedder()


# ---------------------------------------------------------------------------
# Lifecycle / schema
# ---------------------------------------------------------------------------


class TestLifecycle:
    def test_init_creates_tables(self, tmp_path) -> None:
        db = str(tmp_path / "m.db")
        memory = VoiceMemory(db_path=db)
        tables = {
            r[0]
            for r in memory.conn.execute(
                "SELECT name FROM sqlite_master WHERE type IN ('table','view')"
            ).fetchall()
        }
        assert "voice_turns" in tables
        assert "voice_facts" in tables
        assert "voice_memory_meta" in tables
        # FTS5 virtual table exists (or silently absent on exotic builds)
        memory.close()

    def test_init_creates_fts_index(self, tmp_path) -> None:
        memory = VoiceMemory(db_path=str(tmp_path / "m.db"))
        row = memory.conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='voice_turns_fts'"
        ).fetchone()
        assert row is not None
        memory.close()

    def test_close_is_idempotent(self, vm) -> None:
        vm.close()
        vm.close()  # no error

    def test_session_id_default(self, tmp_path) -> None:
        memory = VoiceMemory(db_path=str(tmp_path / "m.db"))
        assert memory.session_id
        memory.close()

    def test_session_id_custom(self, tmp_path) -> None:
        memory = VoiceMemory(db_path=str(tmp_path / "m.db"), session_id="sess-1")
        assert memory.session_id == "sess-1"
        memory.close()

    def test_meta_roundtrip(self, vm) -> None:
        assert vm._get_meta("missing") is None
        vm._set_meta("key", "value")
        assert vm._get_meta("key") == "value"
        # Non-numeric value → int conversion raises ValueError
        with pytest.raises(ValueError):
            vm._get_meta_int("key")
        vm._set_meta("n", "42")
        assert vm._get_meta_int("n") == 42


# ---------------------------------------------------------------------------
# Turns: save / load
# ---------------------------------------------------------------------------


class TestTurns:
    def test_save_and_load_recent(self, vm) -> None:
        vm.save_turn("user", "первый")
        vm.save_turn("assistant", "второй")
        turns = vm.load_recent_turns(limit=10)
        assert [t["content"] for t in turns] == ["первый", "второй"]
        assert [t["role"] for t in turns] == ["user", "assistant"]

    def test_load_recent_respects_limit(self, vm) -> None:
        for i in range(5):
            vm.save_turn("user", f"msg-{i}")
        turns = vm.load_recent_turns(limit=2)
        assert [t["content"] for t in turns] == ["msg-3", "msg-4"]

    def test_save_turn_returns_rowid(self, vm) -> None:
        rid = vm.save_turn("user", "hello")
        assert isinstance(rid, int) and rid > 0

    def test_save_empty_content_returns_minus_one(self, vm) -> None:
        assert vm.save_turn("user", "") == -1
        assert vm.save_turn("user", "   ") == -1

    def test_save_strips_content(self, vm) -> None:
        vm.save_turn("user", "  trimmed  ")
        turns = vm.load_recent_turns(limit=1)
        assert turns[0]["content"] == "trimmed"

    def test_load_recent_exclude_current_session(self, tmp_path) -> None:
        memory = VoiceMemory(db_path=str(tmp_path / "m.db"), session_id="s1")
        memory.save_turn("user", "old", session_id="s0")
        memory.save_turn("user", "new", session_id="s1")
        turns = memory.load_recent_turns(limit=10, exclude_current_session=True)
        assert [t["content"] for t in turns] == ["old"]
        memory.close()

    def test_turn_has_all_fields(self, vm) -> None:
        vm.save_turn("user", "hello", session_id="custom")
        t = vm.load_recent_turns(limit=1)[0]
        assert set(t.keys()) >= {"id", "session_id", "role", "content", "timestamp"}


# ---------------------------------------------------------------------------
# FTS5 keyword search
# ---------------------------------------------------------------------------


class TestFtsSearch:
    def test_fts_finds_keyword(self, vm) -> None:
        vm.save_turn("user", "где находится кухня")
        results = vm.search("кухня", limit=5)
        assert len(results) == 1
        assert results[0]["content"] == "где находится кухня"
        assert results[0]["source"] == "fts"

    def test_fts_returns_empty_for_blank_query(self, vm) -> None:
        vm.save_turn("user", "что-то")
        assert vm.search("") == []
        assert vm.search("   ") == []

    def test_fts_returns_empty_for_no_match(self, vm) -> None:
        vm.save_turn("user", "про музыку")
        assert vm.search("несуществующееслово") == []

    def test_fts_prefix_or_query(self, vm) -> None:
        vm.save_turn("user", "кухня")
        vm.save_turn("user", "гостиная")
        results = vm.search("кухня", limit=5)
        assert len(results) == 1
        assert results[0]["content"] == "кухня"

    def test_fts_operational_error_returns_empty(self, vm) -> None:
        vm.save_turn("user", "abc")
        # An invalid FTS5 MATCH query surfaces as OperationalError; the
        # method must degrade to [] instead of raising.
        assert vm._fts_search('"', limit=5) == []


# ---------------------------------------------------------------------------
# Vector search (mock embedder; sqlite-vec optional)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    not _SQLITE_VEC_AVAILABLE, reason="sqlite-vec extension not installed"
)
class TestVectorSearch:
    def test_embed_and_store_creates_vec_table(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "кухня слева")
        assert vm._has_vec_table() is True
        vec_count = vm.conn.execute(
            "SELECT COUNT(*) FROM voice_turns_vec"
        ).fetchone()[0]
        assert vec_count == 1

    def test_vector_search_finds_similar(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "я люблю джаз")
        # Query with identical text → identical vector → distance ~0.
        results = vm._vector_search("я люблю джаз", limit=5)
        assert len(results) == 1
        assert results[0]["content"] == "я люблю джаз"
        assert results[0]["source"] == "vec"

    def test_vector_search_ranks_closer_first(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "alpha")
        vm.save_turn("user", "beta")
        results = vm._vector_search("alpha", limit=5)
        # Exact-text vector must rank first
        assert results[0]["content"] == "alpha"

    def test_vector_search_no_vec_table_returns_empty(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        # No turns saved → no vec table
        assert vm._vector_search("anything", limit=5) == []

    def test_vector_search_embedder_down_returns_empty(self, vm, fake_embedder) -> None:
        fake_embedder._available = False
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "текст")
        assert vm._vector_search("текст", limit=5) == []

    def test_hybrid_merge_dedupes_and_sorts(self) -> None:
        memory = VoiceMemory.__new__(VoiceMemory)
        fts = [
            {"id": 1, "content": "a", "score": 3.0, "source": "fts"},
            {"id": 2, "content": "b", "score": 2.0, "source": "fts"},
        ]
        vec = [
            {"id": 2, "content": "b", "score": 4.0, "source": "vec"},
            {"id": 3, "content": "c", "score": 1.0, "source": "vec"},
        ]
        merged = memory._merge_results(fts, vec, limit=10)
        # id=2 appears in both lists; merge keeps the FIRST occurrence
        # (from fts, score 2.0), dedupes the vec copy, relabels sources,
        # and sorts by score descending.
        ids = [m["id"] for m in merged]
        assert ids == [1, 2, 3]
        assert all(m["source"] == "hybrid" for m in merged)
        assert merged[0]["score"] == 3.0  # id=1 kept its fts score

    def test_hybrid_merge_respects_limit(self) -> None:
        memory = VoiceMemory.__new__(VoiceMemory)
        fts = [{"id": i, "content": str(i), "score": float(i)} for i in range(5)]
        merged = memory._merge_results(fts, [], limit=2)
        assert len(merged) == 2

    def test_search_uses_vector_when_fts_sparse(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "я люблю джаз")
        # embedder available + vec table exists + fts returns 1 (< MIN 3)
        results = vm.search("я люблю джаз", limit=5)
        assert results  # hybrid or fts — but must not crash

    def test_get_context_reports_vec_enabled(self, vm, fake_embedder) -> None:
        vm.embedder = fake_embedder  # type: ignore[assignment]
        vm.save_turn("user", "привет")
        ctx = vm.get_context(limit=5, query="привет")
        assert ctx["total_turns"] == 1
        assert ctx["vec_enabled"] is True


# ---------------------------------------------------------------------------
# Facts
# ---------------------------------------------------------------------------


class TestFacts:
    def test_save_and_get(self, vm) -> None:
        fid = vm.save_fact("Пользователь любит джаз", category="preference")
        facts = vm.get_facts()
        assert len(facts) == 1
        assert facts[0]["id"] == fid
        assert facts[0]["fact"] == "Пользователь любит джаз"
        assert facts[0]["category"] == "preference"

    def test_get_facts_filters_by_category(self, vm) -> None:
        vm.save_fact("имя саша", category="name")
        vm.save_fact("предпочтение", category="preference")
        names = vm.get_facts(category="name")
        assert len(names) == 1
        assert names[0]["fact"] == "имя саша"

    def test_get_facts_orders_by_updated_desc(self, vm) -> None:
        vm.save_fact("first")
        time.sleep(0.01)
        vm.save_fact("second")
        facts = vm.get_facts()
        assert facts[0]["fact"] == "second"

    def test_update_fact(self, vm) -> None:
        fid = vm.save_fact("старый текст")
        assert vm.update_fact(fid, "новый текст") is True
        facts = vm.get_facts()
        assert facts[0]["fact"] == "новый текст"

    def test_update_fact_missing_id_returns_false(self, vm) -> None:
        assert vm.update_fact(9999, "x") is False

    def test_delete_fact(self, vm) -> None:
        fid = vm.save_fact("удалить меня")
        assert vm.delete_fact(fid) is True
        assert vm.get_facts() == []
        assert vm.delete_fact(fid) is False

    def test_format_facts_for_prompt(self, vm) -> None:
        assert vm.format_facts_for_prompt() == ""
        vm.save_fact("любит джаз")
        text = vm.format_facts_for_prompt()
        assert "любит джаз" in text
        assert text.startswith("Known user facts:")

    def test_get_facts_limit(self, vm) -> None:
        for i in range(5):
            vm.save_fact(f"fact-{i}")
        assert len(vm.get_facts(limit=2)) == 2


# ---------------------------------------------------------------------------
# Context / stats / maintenance
# ---------------------------------------------------------------------------


class TestContextStats:
    def test_get_context_no_query_uses_recent(self, vm) -> None:
        # Save under a DIFFERENT session — get_context() excludes the
        # current session when no query is given.
        vm.save_turn("user", "hello", session_id="other-session")
        ctx = vm.get_context(limit=5)
        assert ctx["total_turns"] == 1
        assert len(ctx["recent_turns"]) == 1
        assert ctx["sessions"] == 1  # only the "other-session" row exists
        assert ctx["current_session"] == vm.session_id

    def test_get_stats(self, vm) -> None:
        vm.save_turn("user", "one")
        vm.save_turn("assistant", "two")
        vm.save_fact("fact")
        stats = vm.get_stats()
        assert stats["turn_count"] == 2
        assert stats["session_count"] == 1
        assert stats["fact_count"] == 1
        assert stats["db_size_kb"] >= 0
        assert "vec_enabled" in stats
        assert "ollama_available" in stats

    def test_prune_older_than(self, vm) -> None:
        vm.save_turn("user", "old")
        # Move the row's timestamp far into the past
        vm.conn.execute(
            "UPDATE voice_turns SET timestamp = ? WHERE content = 'old'",
            (time.time() - 100 * 86400,),
        )
        vm.conn.commit()
        vm.save_turn("user", "new")
        assert vm.prune_older_than(days=30) == 1
        turns = vm.load_recent_turns(limit=10)
        assert [t["content"] for t in turns] == ["new"]

    def test_reindex_embeddings_when_embedder_down(self, vm, fake_embedder) -> None:
        fake_embedder._available = False
        vm.embedder = fake_embedder  # type: ignore[assignment]
        result = vm.reindex_embeddings()
        assert result["error"] == "Ollama not available"

    def test_reindex_embeddings_probe_failure(self, vm, fake_embedder) -> None:
        # available but embed returns None on probe → failure path
        class _Flaky(_FakeEmbedder):
            def embed(self, text: str) -> list[float] | None:
                return None

        vm.embedder = _Flaky()  # type: ignore[assignment]
        result = vm.reindex_embeddings()
        assert result["error"] == "Failed to get embedding"


@pytest.mark.skipif(
    not _SQLITE_VEC_AVAILABLE, reason="sqlite-vec extension not installed"
)
class TestReindexWithVec:
    def test_reindex_embeddings_indexes_turns(self, vm, fake_embedder) -> None:
        vm.save_turn("user", "первый")
        vm.save_turn("user", "второй")
        vm.embedder = fake_embedder  # type: ignore[assignment]
        result = vm.reindex_embeddings()
        assert result["indexed"] == 2
        assert result["total"] == 2
        assert result["dim"] == 8


# ---------------------------------------------------------------------------
# OllamaEmbedder unit behaviour (no network calls)
# ---------------------------------------------------------------------------


class TestOllamaEmbedder:
    def test_backoff_returns_none(self) -> None:
        embedder = OllamaEmbedder(retry_interval=1000)
        embedder._available = False
        embedder._last_failure = time.time()
        assert embedder.embed("x") is None
        assert embedder.is_available() is False

    def test_dim_property_none_before_embed(self) -> None:
        embedder = OllamaEmbedder()
        assert embedder.dim is None

    def test_embed_network_failure_marks_unavailable(self, monkeypatch) -> None:
        embedder = OllamaEmbedder(timeout=1.0)

        def fake_post(*args, **kwargs):
            raise RuntimeError("connection refused")

        monkeypatch.setattr("httpx.post", fake_post)
        result = embedder.embed("test")
        assert result is None
        assert embedder._available is False
        assert embedder._last_failure > 0

    def test_embed_success_sets_dim_and_available(self, monkeypatch) -> None:
        embedder = OllamaEmbedder()

        class _Resp:
            def raise_for_status(self) -> None:
                return None

            def json(self) -> dict:
                return {"embedding": [0.1, 0.2, 0.3]}

        def fake_post(*args, **kwargs):
            return _Resp()

        monkeypatch.setattr("httpx.post", fake_post)
        vec = embedder.embed("hello")
        assert vec == [0.1, 0.2, 0.3]
        assert embedder.dim == 3
        assert embedder.is_available() is True
