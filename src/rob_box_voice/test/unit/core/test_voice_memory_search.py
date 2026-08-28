"""Regression tests for VoiceMemory FTS5 search (issue #1355)."""

from __future__ import annotations

from rob_box_voice.core.voice_memory import VoiceMemory


def _memory(tmp_path) -> VoiceMemory:
    return VoiceMemory(
        db_path=str(tmp_path / "voice_memory.db"),
        session_id="issue_1355",
    )


def test_search_is_case_insensitive_for_cyrillic(tmp_path) -> None:
    memory = _memory(tmp_path)
    try:
        memory.save_turn("user", "Иван любит музыку")

        upper_hits = memory.search("ИВАН", limit=5)
        lower_hits = memory.search("иван", limit=5)

        assert [hit["id"] for hit in upper_hits] == [hit["id"] for hit in lower_hits]
        assert any("Иван" in hit["content"] for hit in upper_hits)
    finally:
        memory.close()


def test_startup_repairs_missing_fts_rows(tmp_path) -> None:
    db_path = tmp_path / "voice_memory.db"
    memory = VoiceMemory(db_path=str(db_path), session_id="seed")
    try:
        memory.conn.execute("DROP TRIGGER voice_turns_ai")
        memory.conn.execute(
            "INSERT INTO voice_turns (session_id, role, content, timestamp) "
            "VALUES (?, ?, ?, ?)",
            ("legacy", "user", "Мы говорили про Ивана", 1.0),
        )
        memory.conn.commit()
        assert memory.search("иван", limit=5) == []
    finally:
        memory.close()

    reopened = VoiceMemory(db_path=str(db_path), session_id="reopened")
    try:
        hits = reopened.search("Иван", limit=5)
        assert any("Ивана" in hit["content"] for hit in hits)
    finally:
        reopened.close()


def test_startup_repairs_stale_fts_rows(tmp_path) -> None:
    db_path = tmp_path / "voice_memory.db"
    memory = VoiceMemory(db_path=str(db_path), session_id="seed")
    try:
        turn_id = memory.save_turn("user", "Саша любит музыку")
        memory.conn.execute("DROP TRIGGER voice_turns_au")
        memory.conn.execute(
            "UPDATE voice_turns SET content = ? WHERE id = ?",
            ("Иван любит музыку", turn_id),
        )
        memory.conn.commit()
        assert memory.search("иван", limit=5) == []
    finally:
        memory.close()

    reopened = VoiceMemory(db_path=str(db_path), session_id="reopened")
    try:
        hits = reopened.search("ИВАН", limit=5)
        assert [hit["id"] for hit in hits] == [turn_id]
        assert hits[0]["content"] == "Иван любит музыку"
    finally:
        reopened.close()


def test_startup_repairs_missing_fts_triggers(tmp_path) -> None:
    db_path = tmp_path / "voice_memory.db"
    memory = VoiceMemory(db_path=str(db_path), session_id="seed")
    try:
        memory.conn.execute("DROP TRIGGER voice_turns_ai")
        memory.conn.commit()
    finally:
        memory.close()

    reopened = VoiceMemory(db_path=str(db_path), session_id="reopened")
    try:
        reopened.save_turn("user", "Иван пришёл после перезапуска")
        hits = reopened.search("иван", limit=5)
        assert any("перезапуска" in hit["content"] for hit in hits)
        trigger = reopened.conn.execute(
            "SELECT name FROM sqlite_master "
            "WHERE type = 'trigger' AND name = 'voice_turns_ai'"
        ).fetchone()
        assert trigger is not None
    finally:
        reopened.close()
