"""Regression tests for issue #2793.

Before this fix, ``memory_save`` wrote to ``voice_facts`` while
``VoiceMemory.search()`` only ever read ``voice_turns`` (and its FTS5
index) — a fact saved seconds earlier could never be found by
``memory_search``. Live E2E symptom: ``facts=4 turns=0`` in the run's
database, yet the robot told the user its memory search came back empty.

These tests pin the actual contract requested in the issue:
  1. ``memory_save`` -> ``memory_search`` (by a word from the fact) finds it.
  2. Facts respect ``speaker_id`` scoping the same way ``save_fact`` does.
  3. Results distinguish a "fact" hit from a "turn" hit (``kind`` key).
"""

from __future__ import annotations

from rob_box_voice.core.voice_memory import VoiceMemory


def _memory(tmp_path) -> VoiceMemory:
    return VoiceMemory(
        db_path=str(tmp_path / "voice_memory.db"),
        session_id="issue_2793",
    )


def test_save_fact_then_search_finds_it(tmp_path) -> None:
    """The exact regression from the issue: save a fact, search a word
    from it, expect a non-empty result -- not "поиск упрямо пустой"."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact(
            "Саша пьёт зелёный чай без сахара", category="preference"
        )

        hits = memory.search("чай", limit=5)

        assert hits, (
            "memory_search вернул пусто для только что сохранённого факта"
        )
        assert any("чай" in h["content"] for h in hits)
        assert all(h["kind"] == "fact" for h in hits)
    finally:
        memory.close()


def test_search_distinguishes_fact_from_turn(tmp_path) -> None:
    memory = _memory(tmp_path)
    try:
        memory.save_fact("Борис болеет за Спартак", category="preference")
        memory.save_turn("user", "Борис вчера смотрел матч Спартака")

        hits = memory.search("Спартак", limit=10)
        kinds = {h["kind"] for h in hits}

        assert kinds == {"fact", "turn"}
        fact_hit = next(h for h in hits if h["kind"] == "fact")
        turn_hit = next(h for h in hits if h["kind"] == "turn")
        assert fact_hit["source"] == "fact"
        assert turn_hit["source"] in ("fts", "vec", "hybrid")
    finally:
        memory.close()


def test_fact_search_respects_speaker_scope(tmp_path) -> None:
    """Same personal-vs-global rule as ``save_fact``/``get_facts`` (#1770):
    a registered speaker sees their own facts + legacy global rows, never
    another registered speaker's."""
    memory = _memory(tmp_path)
    try:
        memory.save_fact("Денчик любит лыжи", speaker_id="denchik")
        memory.save_fact("Саша любит китайскую музыку", speaker_id="sasha")
        memory.save_fact("любимый цвет неба — голубой")  # legacy global

        denchik_hits = memory.search("любит", limit=10, speaker_id="denchik")
        sasha_hits = memory.search("любит", limit=10, speaker_id="sasha")

        assert any("лыжи" in h["content"] for h in denchik_hits)
        assert not any("китайскую" in h["content"] for h in denchik_hits)
        assert any("китайскую" in h["content"] for h in sasha_hits)
        assert not any("лыжи" in h["content"] for h in sasha_hits)
    finally:
        memory.close()


def test_facts_take_priority_within_limit(tmp_path) -> None:
    """Facts fill the result budget first; turns only fill what's left --
    this is what makes the just-saved fact actually surface instead of
    being crowded out by unrelated historical turns."""
    memory = _memory(tmp_path)
    try:
        for i in range(5):
            memory.save_turn("user", f"чай эпизод {i}")
        memory.save_fact("Саша пьёт зелёный чай без сахара")

        hits = memory.search("чай", limit=3)

        assert len(hits) == 3
        assert hits[0]["kind"] == "fact"
    finally:
        memory.close()


def test_empty_query_returns_empty_list(tmp_path) -> None:
    memory = _memory(tmp_path)
    try:
        memory.save_fact("факт без запроса не найдётся")
        assert memory.search("", limit=5) == []
        assert memory.search("   ", limit=5) == []
    finally:
        memory.close()
