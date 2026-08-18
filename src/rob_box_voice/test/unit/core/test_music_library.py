"""Unit tests for :mod:`rob_box_voice.core.music_library`."""

from __future__ import annotations

import json
import os
import sqlite3
import tempfile
from pathlib import Path

import pytest

from rob_box_voice.core.music_library import (
    GeneratedMusicLibrary,
    GeneratedTrack,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def library(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> GeneratedMusicLibrary:
    """A library with an isolated SQLite DB and on-disk root."""
    db_path = tmp_path / "music.db"
    lib_root = tmp_path / "library"
    monkeypatch.setenv("VOICE_MEMORY_DB_PATH", str(db_path))
    monkeypatch.setenv("MUSIC_LIBRARY_ROOT", str(lib_root))
    return GeneratedMusicLibrary(
        library_root=str(lib_root),
        db_path=str(db_path),
    )


# ---------------------------------------------------------------------------
# Smoke / schema
# ---------------------------------------------------------------------------


class TestSchema:
    def test_creates_table_and_fts(self, library: GeneratedMusicLibrary) -> None:
        with library.lock:
            tables = {
                row["name"]
                for row in library.conn.execute(
                    "SELECT name FROM sqlite_master WHERE type IN ('table', 'view')"
                ).fetchall()
            }
        assert "generated_tracks" in tables
        assert "generated_tracks_fts" in tables

    def test_uses_migrations_dir_when_present(
        self, library: GeneratedMusicLibrary, tmp_path: Path
    ) -> None:
        # The fixture already runs against a real migrations/ dir; assert
        # that the resulting user_version is at least 7.
        with library.lock:
            version = library.conn.execute("PRAGMA user_version").fetchone()[0]
        assert version >= 7, f"expected user_version >= 7, got {version}"

    def test_inline_fallback_schema(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        """No migrations dir → library must still work via inline schema."""
        monkeypatch.setenv("VOICE_MEMORY_DB_PATH", str(tmp_path / "vm.db"))
        monkeypatch.setenv("MUSIC_LIBRARY_ROOT", str(tmp_path / "lib"))
        lib = GeneratedMusicLibrary(
            library_root=str(tmp_path / "lib"),
            db_path=str(tmp_path / "vm.db"),
            migrations_dir="/nonexistent/path",
        )
        # CRUD must work
        track = lib.save(prompt="hello", name="t1", mood="chill")
        assert lib.count() == 1
        assert lib.get(track.track_id) is not None


# ---------------------------------------------------------------------------
# CRUD
# ---------------------------------------------------------------------------


class TestCrud:
    def test_save_assigns_uuid_when_track_id_omitted(
        self, library: GeneratedMusicLibrary
    ) -> None:
        t = library.save(prompt="romantic ballad")
        assert t.track_id
        assert len(t.track_id) == 32  # uuid4().hex length
        assert t.generated_at > 0
        assert t.updated_at == t.generated_at
        assert t.play_count == 0

    def test_save_persists_all_fields(self, library: GeneratedMusicLibrary) -> None:
        t = library.save(
            track_id="abc123",
            prompt="romantic piano ballad C minor 80bpm",
            lyrics="[Verse]\nВ каплях дождя\n[Chorus]\nТы со мной",
            model="music-3.0",
            mood="romantic",
            genre="ballad",
            lang="ru",
            tags=["chill", "minor", "rain"],
            name="rain song",
            duration_s=60.0,
        )
        got = library.get("abc123")
        assert got is not None
        assert got.prompt.startswith("romantic piano")
        assert "[Verse]" in got.lyrics
        assert got.model == "music-3.0"
        assert got.mood == "romantic"
        assert got.genre == "ballad"
        assert got.lang == "ru"
        assert got.tags == ["chill", "minor", "rain"]
        assert got.name == "rain song"
        assert got.duration_s == 60.0
        # meta.json sidecar
        meta_path = library.library_root / "abc123" / "meta.json"
        assert meta_path.exists()
        meta = json.loads(meta_path.read_text())
        assert meta["track_id"] == "abc123"

    def test_save_updates_existing(self, library: GeneratedMusicLibrary) -> None:
        library.save(track_id="x", prompt="v1", mood="sad")
        first = library.get("x")
        library.save(track_id="x", prompt="v2", mood="happy", name="renamed")
        second = library.get("x")
        assert second.prompt == "v2"
        assert second.mood == "happy"
        assert second.name == "renamed"
        # generated_at preserved, updated_at bumped
        assert second.generated_at == first.generated_at
        assert second.updated_at >= first.updated_at

    def test_save_resolves_file_size(self, library: GeneratedMusicLibrary) -> None:
        d = library.ensure_track_dir("z")
        f = d / "track.mp3"
        f.write_bytes(b"\x00" * 1024)
        t = library.save(track_id="z", file_path=str(f), prompt="x")
        assert t.file_size == 1024

    def test_get_returns_none_for_missing(self, library: GeneratedMusicLibrary) -> None:
        assert library.get("does-not-exist") is None

    def test_delete_removes_row_and_dir(
        self, library: GeneratedMusicLibrary
    ) -> None:
        library.save(track_id="del", prompt="bye", name="goodbye")
        assert (library.library_root / "del" / "meta.json").exists()
        assert library.delete("del") is True
        assert library.get("del") is None
        assert not (library.library_root / "del").exists()

    def test_delete_missing_returns_false(self, library: GeneratedMusicLibrary) -> None:
        assert library.delete("ghost") is False

    def test_increment_play_count(self, library: GeneratedMusicLibrary) -> None:
        library.save(track_id="pc", prompt="x")
        assert library.increment_play_count("pc") == 1
        assert library.increment_play_count("pc") == 2
        assert library.increment_play_count("pc") == 3
        assert library.get("pc").play_count == 3

    def test_increment_play_count_missing(self, library: GeneratedMusicLibrary) -> None:
        assert library.increment_play_count("nope") == -1


# ---------------------------------------------------------------------------
# Listing
# ---------------------------------------------------------------------------


class TestListing:
    def test_list_recent_orders_by_generated_at(
        self, library: GeneratedMusicLibrary
    ) -> None:
        import time
        library.save(track_id="a", prompt="first")
        time.sleep(0.01)
        library.save(track_id="b", prompt="second")
        time.sleep(0.01)
        library.save(track_id="c", prompt="third")
        recent = library.list_recent(limit=10)
        ids = [r.track_id for r in recent]
        assert ids == ["c", "b", "a"]

    def test_list_popular_orders_by_play_count(
        self, library: GeneratedMusicLibrary
    ) -> None:
        library.save(track_id="a", prompt="x")
        library.save(track_id="b", prompt="y")
        library.save(track_id="c", prompt="z")
        for _ in range(3):
            library.increment_play_count("b")
        library.increment_play_count("a")
        popular = library.list_popular(limit=10)
        ids = [r.track_id for r in popular]
        # 'c' has play_count=0 → excluded by list_popular (only played tracks)
        assert ids == ["b", "a"]

    def test_list_popular_excludes_zero_plays(
        self, library: GeneratedMusicLibrary
    ) -> None:
        library.save(track_id="x", prompt="never played")
        assert library.list_popular() == []

    def test_list_all_dispatches_on_sort_by(
        self, library: GeneratedMusicLibrary
    ) -> None:
        library.save(track_id="a", prompt="x")
        library.save(track_id="b", prompt="y")
        assert library.list_all(sort_by="recent")[0].track_id == "b"
        library.increment_play_count("a")
        assert library.list_all(sort_by="popular")[0].track_id == "a"

    def test_count(self, library: GeneratedMusicLibrary) -> None:
        assert library.count() == 0
        for i in range(3):
            library.save(track_id=f"t{i}", prompt=f"p{i}")
        assert library.count() == 3


# ---------------------------------------------------------------------------
# Search (FTS5 + filters)
# ---------------------------------------------------------------------------


class TestSearch:
    def _seed(self, lib: GeneratedMusicLibrary) -> None:
        lib.save(track_id="t1", prompt="romantic piano ballad C minor 80bpm", lyrics="[Verse]\nВ каплях дождя", mood="romantic", genre="ballad", tags=["minor", "rain", "piano"], name="rain song")
        lib.save(track_id="t2", prompt="energetic summer house 128bpm", lyrics="[Verse]\nHot sun, hot night", mood="energetic", genre="house", tags=["dance", "summer"], name="summer")
        lib.save(track_id="t3", prompt="dark ambient drone minor atmospheric", lyrics="[Verse]\nNo light", mood="dark", genre="ambient", tags=["drone", "minor"], name="dark night")
        lib.save(track_id="t4", prompt="smooth jazz saxophone chill Bb", lyrics="[Verse]\nSlow walk", mood="chill", genre="jazz", tags=["saxophone", "Bb"], name="jazz walk")

    def test_fts_keyword(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        results = library.search("saxophone", limit=5)
        assert len(results) == 1
        assert results[0].track_id == "t4"

    def test_fts_partial_match(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        # FTS5 prefix-style with our wrapper: "piano" → t1 only
        results = library.search("piano", limit=5)
        assert any(r.track_id == "t1" for r in results)

    def test_fts_malformed_query_falls_back_to_like(
        self, library: GeneratedMusicLibrary
    ) -> None:
        self._seed(library)
        # A colon/operator-like string should not crash — must degrade to LIKE.
        results = library.search("hot:sun*", limit=5)
        # Either FTS5 errored and we LIKE'd, or FTS5 still matched — both OK.
        names = [r.name for r in results]
        assert "summer" in names

    def test_filter_by_mood(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        results = library.search("", mood="romantic", limit=5)
        assert len(results) == 1
        assert results[0].track_id == "t1"

    def test_filter_by_genre(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        results = library.search("", genre="jazz", limit=5)
        assert len(results) == 1
        assert results[0].track_id == "t4"

    def test_filter_by_tag(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        results = library.search("", tags=["minor"], limit=5)
        ids = {r.track_id for r in results}
        assert ids == {"t1", "t3"}

    def test_combined_query_and_filters(
        self, library: GeneratedMusicLibrary
    ) -> None:
        self._seed(library)
        results = library.search("jazz", mood="chill", limit=5)
        assert len(results) == 1
        assert results[0].track_id == "t4"

    def test_limit_is_respected(self, library: GeneratedMusicLibrary) -> None:
        self._seed(library)
        results = library.search("", limit=2)
        assert len(results) == 2

    def test_no_query_returns_recent_when_filters_supplied(
        self, library: GeneratedMusicLibrary
    ) -> None:
        self._seed(library)
        results = library.search("", mood="dark")
        assert [r.track_id for r in results] == ["t3"]

    def test_fts5_keeps_in_sync_after_update(
        self, library: GeneratedMusicLibrary
    ) -> None:
        self._seed(library)
        library.save(track_id="t1", prompt="completely different text about cats", mood="cute")
        results = library.search("cats", limit=5)
        assert any(r.track_id == "t1" for r in results)
        # Old text "piano" should be gone from t1
        old = library.search("piano")
        assert all(r.track_id != "t1" for r in old)

    def test_fts5_keeps_in_sync_after_delete(
        self, library: GeneratedMusicLibrary
    ) -> None:
        self._seed(library)
        library.delete("t1")
        results = library.search("piano")
        assert all(r.track_id != "t1" for r in results)


# ---------------------------------------------------------------------------
# Escaping / safety
# ---------------------------------------------------------------------------


class TestFtsEscape:
    def test_escape_wraps_quotes(self) -> None:
        assert GeneratedMusicLibrary._escape_fts('hello "world"') == '"hello" "world"'

    def test_escape_handles_empty(self) -> None:
        assert GeneratedMusicLibrary._escape_fts("") == ""
        assert GeneratedMusicLibrary._escape_fts("   ") == ""

    def test_escape_drops_empty_tokens(self) -> None:
        assert GeneratedMusicLibrary._escape_fts("a  b\tc") == '"a" "b" "c"'

    def test_escape_special_chars(self) -> None:
        # FTS5 operators must be quoted to be safe.
        out = GeneratedMusicLibrary._escape_fts("hot:sun* +foo -bar")
        assert "hot:sun*" in out
        assert "+foo" in out
        assert "-bar" in out
        # All wrapped in quotes
        assert all(t.startswith('"') and t.endswith('"') for t in out.split())
