"""Regression tests for per-speaker personal memory (issue #1770).

Before the fix:
- ``voice_facts`` and ``voice_turns`` had no ``speaker_id`` column.
- ``memory_context`` / ``memory_search`` returned a global pool — facts
  about "Саша" leaked into the answer when "Денчик" asked
  "что ты обо мне знаешь?" (the live bug from the issue body).

After the fix:
- ``save_turn`` / ``save_fact`` accept ``speaker_id``.
- ``get_facts`` / ``search`` / ``load_recent_turns`` / ``get_context`` scope
  results to the requested ``speaker_id`` plus legacy NULL (global) rows.
- Two registered users never see each other's facts.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.voice_memory import VoiceMemory


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _memory(tmp_path) -> VoiceMemory:
    return VoiceMemory(
        db_path=str(tmp_path / "voice_memory.db"),
        session_id="issue_1770",
    )


@pytest.fixture()
def memory(tmp_path):
    mem = _memory(tmp_path)
    try:
        yield mem
    finally:
        mem.close()


# ---------------------------------------------------------------------------
# save_fact: speaker_id is persisted
# ---------------------------------------------------------------------------


class TestSaveFactWithSpeaker:
    def test_save_fact_persists_speaker_id(self, memory: VoiceMemory) -> None:
        fid = memory.save_fact(
            "Денчик любит кататься на лыжах",
            category="preference",
            speaker_id="denchik-uuid",
        )
        row = memory.conn.execute(
            "SELECT speaker_id, fact FROM voice_facts WHERE id = ?", (fid,)
        ).fetchone()
        assert row["speaker_id"] == "denchik-uuid"
        assert "лыжах" in row["fact"]

    def test_save_fact_without_speaker_id_keeps_legacy_null(
        self, memory: VoiceMemory
    ) -> None:
        fid = memory.save_fact("глобальный факт")
        row = memory.conn.execute(
            "SELECT speaker_id FROM voice_facts WHERE id = ?", (fid,)
        ).fetchone()
        assert row["speaker_id"] is None


# ---------------------------------------------------------------------------
# save_turn: speaker_id is persisted
# ---------------------------------------------------------------------------


class TestSaveTurnWithSpeaker:
    def test_save_turn_persists_speaker_id(self, memory: VoiceMemory) -> None:
        tid = memory.save_turn(
            "user",
            "Привет, я Денчик",
            speaker_id="denchik-uuid",
        )
        row = memory.conn.execute(
            "SELECT speaker_id, content FROM voice_turns WHERE id = ?", (tid,)
        ).fetchone()
        assert row["speaker_id"] == "denchik-uuid"
        assert "Денчик" in row["content"]


# ---------------------------------------------------------------------------
# get_facts: scopes by speaker
# ---------------------------------------------------------------------------


class TestGetFactsScope:
    def test_get_facts_filters_by_speaker(self, memory: VoiceMemory) -> None:
        memory.save_fact("Денчик любит лыжи", speaker_id="denchik")
        memory.save_fact("Саша любит китайскую музыку", speaker_id="sasha")

        denchik = memory.get_facts(speaker_id="denchik")
        sasha = memory.get_facts(speaker_id="sasha")
        assert [f["fact"] for f in denchik] == ["Денчик любит лыжи"]
        assert [f["fact"] for f in sasha] == ["Саша любит китайскую музыку"]

    def test_get_facts_keeps_legacy_global_rows_visible(
        self, memory: VoiceMemory
    ) -> None:
        """Pre-migration facts (``speaker_id IS NULL``) are shared.

        Otherwise we'd silently delete a real user's history the moment
        we deployed the migration.
        """
        memory.save_fact("глобальный факт про обоих")  # speaker_id=None
        memory.save_fact("Денчик любит лыжи", speaker_id="denchik")

        denchik = memory.get_facts(speaker_id="denchik")
        sasha = memory.get_facts(speaker_id="sasha")
        # Both users see the legacy row.
        assert any("глобальный" in f["fact"] for f in denchik)
        assert any("глобальный" in f["fact"] for f in sasha)
        # Only Denchik sees his personal fact.
        assert any("лыжи" in f["fact"] for f in denchik)
        assert not any("лыжи" in f["fact"] for f in sasha)

    def test_get_facts_without_speaker_returns_everything(
        self, memory: VoiceMemory
    ) -> None:
        memory.save_fact("глобальный факт")
        memory.save_fact("Денчик факт", speaker_id="denchik")
        memory.save_fact("Саша факт", speaker_id="sasha")
        all_facts = memory.get_facts()
        assert len(all_facts) == 3


# ---------------------------------------------------------------------------
# search: scopes by speaker
# ---------------------------------------------------------------------------


class TestSearchScope:
    def test_search_filters_by_speaker(self, memory: VoiceMemory) -> None:
        memory.save_turn("user", "Денчик любит кататься на лыжах", speaker_id="denchik")
        memory.save_turn("user", "Саша любит китайскую музыку", speaker_id="sasha")

        denchik_hits = memory.search("любит", limit=10, speaker_id="denchik")
        sasha_hits = memory.search("любит", limit=10, speaker_id="sasha")
        assert any("лыжах" in h["content"] for h in denchik_hits)
        assert not any("китайскую" in h["content"] for h in denchik_hits)
        assert any("китайскую" in h["content"] for h in sasha_hits)
        assert not any("лыжах" in h["content"] for h in sasha_hits)

    def test_search_keeps_legacy_global_rows(self, memory: VoiceMemory) -> None:
        memory.save_turn("user", "общая тема про Ивана")  # speaker_id=None
        memory.save_turn("user", "Денчик про Ивана", speaker_id="denchik")
        memory.save_turn("user", "Саша про Ивана", speaker_id="sasha")

        denchik_hits = memory.search("Иван", limit=10, speaker_id="denchik")
        # Both personal (denchik) and global hits, no sasha hit.
        contents = " | ".join(h["content"] for h in denchik_hits)
        assert "Денчик про Ивана" in contents
        assert "общая тема про Ивана" in contents
        assert "Саша про Ивана" not in contents


# ---------------------------------------------------------------------------
# get_context: scopes both facts and recent turns
# ---------------------------------------------------------------------------


class TestGetContextScope:
    def test_get_context_filters_facts_and_turns(self, memory: VoiceMemory) -> None:
        memory.save_fact("Денчик любит лыжи", speaker_id="denchik")
        memory.save_fact("Саша любит китайскую музыку", speaker_id="sasha")
        memory.save_turn(
            "user",
            "Денчик говорит про лыжи",
            session_id="other",
            speaker_id="denchik",
        )
        memory.save_turn(
            "user",
            "Саша говорит про китайскую музыку",
            session_id="other",
            speaker_id="sasha",
        )

        ctx = memory.get_context(limit=10, speaker_id="denchik")
        fact_texts = [f["fact"] for f in ctx["facts"]]
        turn_texts = [t["content"] for t in ctx["recent_turns"]]
        assert any("лыжи" in t for t in fact_texts)
        assert not any("китайскую" in t for t in fact_texts)
        assert any("лыжи" in t for t in turn_texts)
        assert not any("китайскую" in t for t in turn_texts)


# ---------------------------------------------------------------------------
# load_recent_turns: scopes by speaker
# ---------------------------------------------------------------------------


class TestLoadRecentTurnsScope:
    def test_load_recent_turns_filters_by_speaker(
        self, memory: VoiceMemory
    ) -> None:
        memory.save_turn(
            "user", "Денчик turn 1", session_id="s1", speaker_id="denchik"
        )
        memory.save_turn(
            "user", "Саша turn 1", session_id="s1", speaker_id="sasha"
        )
        memory.save_turn(
            "assistant", "robot reply", session_id="s1", speaker_id="denchik"
        )

        denchik = memory.load_recent_turns(limit=10, speaker_id="denchik")
        sasha = memory.load_recent_turns(limit=10, speaker_id="sasha")
        assert all(t["speaker_id"] == "denchik" for t in denchik)
        assert all(t["speaker_id"] == "sasha" for t in sasha)
        assert any("robot reply" in t["content"] for t in denchik)


# ---------------------------------------------------------------------------
# Migration: columns exist after init
# ---------------------------------------------------------------------------


class TestSchema:
    def test_speaker_id_column_exists_on_voice_facts(
        self, memory: VoiceMemory
    ) -> None:
        cols = {
            row["name"]
            for row in memory.conn.execute(
                "PRAGMA table_info(voice_facts)"
            ).fetchall()
        }
        assert "speaker_id" in cols

    def test_speaker_id_column_exists_on_voice_turns(
        self, memory: VoiceMemory
    ) -> None:
        cols = {
            row["name"]
            for row in memory.conn.execute(
                "PRAGMA table_info(voice_turns)"
            ).fetchall()
        }
        assert "speaker_id" in cols

    def test_speaker_index_exists_on_voice_facts(
        self, memory: VoiceMemory
    ) -> None:
        idx = memory.conn.execute(
            "SELECT name FROM sqlite_master "
            "WHERE type='index' AND name='idx_vf_speaker_updated'"
        ).fetchone()
        assert idx is not None
