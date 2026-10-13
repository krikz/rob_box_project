"""
test_gen_library.py — тесты библиотеки сгенерированных треков и gen_* тулов.

Issue #1392 / ADR-0158 PR-1: покрыто
  * engine/gen/library.py — CRUD, поиск, play_count (прежний контракт)
  * tools/gen_library.py — 6 тулов (валидация параметров и пути execute)

Клиент MiniMax Music и тул ``generate_music`` удалены (API 410 Gone с
20.08.2026); новый клиент KIE — ``test_gen_kie_suno.py``, новая схема и
миграция — ``test_gen_library_cache.py``. ROS 2 не нужен.
"""


import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from .._ros_stubs import RosStubs

# ── ROS-free mocking (matches existing test_music.py pattern) ───────────────
_ros = RosStubs()


# ── Helpers ──────────────────────────────────────────────────────────────────


class _FakeLogger:
    def info(self, m): pass
    def warning(self, m): pass
    def error(self, m): pass
    def debug(self, m): pass


def _fake_node():
    node = SimpleNamespace(get_logger=lambda: _FakeLogger())
    node.create_publisher = MagicMock(return_value=MagicMock())
    return node


# ── Imports under test (load gen_library directly, bypassing
#    tools/__init__.py which would pull in ros-bound .music + .navigation) ──

def _load_gen_library_module():
    """Load rob_box_mcp_tools.tools.gen_library as a sub-package so its
    ``from ..base import …`` relative import resolves correctly.

    Avoids importing the full ``tools/__init__.py`` which transitively
    requires ``rob_box_voice`` (unavailable in unit-test environment).
    """
    pkg_name = "_rob_box_mcp_tools_test_isolation"
    tools_pkg = ModuleType(pkg_name + ".tools")
    tools_pkg.__path__ = []  # mark as package
    sys.modules[pkg_name + ".tools"] = tools_pkg

    # Stand-in for rob_box_mcp_tools (parent package)
    parent_pkg = ModuleType(pkg_name)
    parent_pkg.tools = tools_pkg
    parent_pkg.__path__ = [str(Path(__file__).resolve().parents[2] / "rob_box_mcp_tools")]
    sys.modules[pkg_name] = parent_pkg

    # Load gen_library as if it were rob_box_mcp_tools.tools.gen_library
    file_path = (Path(__file__).resolve().parents[2]
                 / "rob_box_mcp_tools" / "tools" / "gen_library.py")
    spec = importlib.util.spec_from_file_location(
        pkg_name + ".tools.gen_library", str(file_path),
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[pkg_name + ".tools.gen_library"] = module
    spec.loader.exec_module(module)
    return module


with _ros:
    from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary

    _mm = _load_gen_library_module()
_ros_stubs = _ros.fixture()
GenListLibraryTool = _mm.GenListLibraryTool
GenSearchLibraryTool = _mm.GenSearchLibraryTool
GenSaveToLibraryTool = _mm.GenSaveToLibraryTool
GenPlayFromLibraryTool = _mm.GenPlayFromLibraryTool
GenDeleteFromLibraryTool = _mm.GenDeleteFromLibraryTool
GenGetTrackInfoTool = _mm.GenGetTrackInfoTool


# ════════════════════════════════════════════════════════════════════════════
# engine/gen/library.py
# ════════════════════════════════════════════════════════════════════════════


@pytest.fixture
def tmp_lib(tmp_path):
    """Fresh GeneratedMusicLibrary per test."""
    lib = GeneratedMusicLibrary(str(tmp_path / "music_library"))
    yield lib
    # cleanup happens automatically with tmp_path


class TestGeneratedMusicLibrary:
    def test_empty_library_count(self, tmp_lib):
        assert tmp_lib.count == 0
        assert tmp_lib.list_tracks()["total"] == 0

    def test_save_track_creates_directory_and_files(self, tmp_lib):
        result = tmp_lib.save_track(
            audio_bytes=b"ID3...fake mp3 bytes...",
            prompt="Indie folk, melancholic, 92 bpm",
            lyrics="[Verse]\nStreetlights flicker...",
            model="music-3.0-free",
            duration_ms=45000,
            tags=["rainy", "love"],
            mood="melancholic",
            genre="indie folk",
            title="Rainy Evening",
        )
        assert result["success"]
        track_id = result["id"]
        assert len(track_id) == 32  # uuid4 hex
        assert Path(result["path"]).exists()
        assert Path(result["meta_json"]).exists()

        meta_on_disk = json.loads(Path(result["meta_json"]).read_text(encoding="utf-8"))
        assert meta_on_disk["title"] == "Rainy Evening"
        assert meta_on_disk["duration_ms"] == 45000
        assert meta_on_disk["tags"] == ["rainy", "love"]

        assert tmp_lib.count == 1

    def test_save_empty_audio_fails(self, tmp_lib):
        result = tmp_lib.save_track(audio_bytes=b"", prompt="x")
        assert not result["success"]
        assert "empty" in result["error"]

    def test_save_empty_prompt_fails(self, tmp_lib):
        result = tmp_lib.save_track(audio_bytes=b"x", prompt="")
        assert not result["success"]

    def test_list_filters_and_sorts(self, tmp_lib):
        for i in range(3):
            tmp_lib.save_track(
                audio_bytes=b"x",
                prompt=f"track {i}",
                tags=["common"],
                mood="happy" if i % 2 else "sad",
                rating=i,
            )
        all_tracks = tmp_lib.list_tracks()
        assert all_tracks["total"] == 3

        # sort_by=rating
        by_rating = tmp_lib.list_tracks(sort_by="rating")
        assert by_rating["tracks"][0]["rating"] == 2  # highest first

        # mood filter
        happy_only = tmp_lib.list_tracks(mood="happy")
        assert happy_only["total"] == 1

        # tag filter
        common = tmp_lib.list_tracks(tag="common")
        assert common["total"] == 3

    def test_search_by_keyword(self, tmp_lib):
        tmp_lib.save_track(audio_bytes=b"x", prompt="Indie folk about rain", lyrics="drip drop")
        tmp_lib.save_track(audio_bytes=b"x", prompt="Synthwave about love", lyrics="neon glow")
        tmp_lib.save_track(audio_bytes=b"x", prompt="Lo-fi beats", lyrics="chill vibes")

        r = tmp_lib.search_tracks("rain")
        assert r["total"] == 1
        assert "rain" in r["tracks"][0]["prompt"].lower()

        # Search across lyrics
        r2 = tmp_lib.search_tracks("neon")
        assert r2["total"] == 1

    def test_search_empty_query_rejected(self, tmp_lib):
        r = tmp_lib.search_tracks("")
        assert not r["success"]
        assert r["total"] == 0

    def test_get_track_info(self, tmp_lib):
        saved = tmp_lib.save_track(
            audio_bytes=b"abc", prompt="x", title="My Track", rating=4,
        )
        info = tmp_lib.get_track_info(saved["id"])
        assert info["success"]
        assert info["title"] == "My Track"
        assert info["exists_on_disk"] is True
        assert info["path"].endswith("track.mp3")

    def test_get_nonexistent_track(self, tmp_lib):
        info = tmp_lib.get_track_info("deadbeef" * 4)
        assert not info["success"]
        assert "не найден" in info["error"]

    def test_delete_track_removes_files(self, tmp_lib):
        saved = tmp_lib.save_track(audio_bytes=b"abc", prompt="x")
        track_id = saved["id"]
        assert tmp_lib.count == 1

        result = tmp_lib.delete_track(track_id)
        assert result["success"]
        assert tmp_lib.count == 0
        assert not (tmp_lib.root_dir / track_id).exists()

    def test_delete_nonexistent(self, tmp_lib):
        result = tmp_lib.delete_track("deadbeef" * 4)
        assert not result["success"]

    def test_play_count_increments(self, tmp_lib):
        saved = tmp_lib.save_track(audio_bytes=b"x", prompt="x")
        track_id = saved["id"]

        r1 = tmp_lib.increment_play_count(track_id)
        assert r1["success"]
        assert r1["play_count"] == 1

        r2 = tmp_lib.increment_play_count(track_id)
        assert r2["play_count"] == 2


# ════════════════════════════════════════════════════════════════════════════
# tools/gen_library.py — 6 MCP tools
# ════════════════════════════════════════════════════════════════════════════


class TestToolBasics:
    """Generic smoke tests for all 6 tools."""

    @pytest.fixture
    def library(self, tmp_path):
        return GeneratedMusicLibrary(str(tmp_path / "lib"))

    def _node(self):
        return _fake_node()

    def test_names_and_descriptions(self, library):
        tools = [
            (GenListLibraryTool, None, library, "gen_list_library"),
            (GenSearchLibraryTool, None, library, "gen_search_library"),
            (GenSaveToLibraryTool, None, library, "gen_save_to_library"),
            (GenPlayFromLibraryTool, None, library, "gen_play_from_library"),
            (GenDeleteFromLibraryTool, None, library, "gen_delete_from_library"),
            (GenGetTrackInfoTool, None, library, "gen_get_track_info"),
        ]
        names = []
        for cls, c, l, expected_name in tools:
            t = cls(self._node(), l)
            assert t.name == expected_name
            assert t.description  # non-empty
            assert len(t.parameters) > 0
            # Each param has a JSON-schema shape
            for p in t.parameters:
                assert p.name
                assert p.type
                assert p.description
            names.append(t.name)
        assert len(names) == 6
        assert len(set(names)) == 6  # all unique

    def test_list_search_info_are_read_only(self, library):
        for cls in (GenListLibraryTool, GenSearchLibraryTool, GenGetTrackInfoTool):
            t = cls(self._node(), library)
            assert t.read_only is True


class TestListSearchTools:
    @pytest.fixture
    def library(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        lib.save_track(audio_bytes=b"x", prompt="happy indie", tags=["happy", "folk"], mood="happy", rating=5)
        lib.save_track(audio_bytes=b"y", prompt="sad ballad", tags=["sad"], mood="sad", rating=3)
        lib.save_track(audio_bytes=b"z", prompt="energetic dance", tags=["dance"], mood="energetic", rating=4)
        return lib

    def test_list_returns_all(self, library):
        tool = GenListLibraryTool(_fake_node(), library)
        result = tool.execute()
        assert result.success
        assert result.data["total"] == 3

    def test_list_limit(self, library):
        tool = GenListLibraryTool(_fake_node(), library)
        result = tool.execute(limit=2)
        assert result.data["total"] == 2

    def test_list_mood_filter(self, library):
        tool = GenListLibraryTool(_fake_node(), library)
        result = tool.execute(mood="sad")
        assert result.data["total"] == 1
        assert result.data["tracks"][0]["mood"] == "sad"

    def test_list_sort_by_rating(self, library):
        tool = GenListLibraryTool(_fake_node(), library)
        result = tool.execute(sort_by="rating")
        ratings = [t["rating"] for t in result.data["tracks"]]
        assert ratings == sorted(ratings, reverse=True)

    def test_search_finds_match(self, library):
        tool = GenSearchLibraryTool(_fake_node(), library)
        result = tool.execute(query="ballad")
        assert result.success
        assert result.data["total"] >= 1

    def test_search_missing_query_fails(self, library):
        tool = GenSearchLibraryTool(_fake_node(), library)
        result = tool.execute(query="")
        assert not result.success


class TestGetTrackInfoTool:
    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(
            audio_bytes=b"x" * 100, prompt="test", title="My Track",
            duration_ms=30000, model="music-3.0-free",
        )
        return SimpleNamespace(
            library=lib,
            track_id=saved["id"],
            tool=GenGetTrackInfoTool(_fake_node(), lib),
        )

    def test_returns_full_metadata(self, setup):
        result = setup.tool.execute(track_id=setup.track_id)
        assert result.success
        assert result.data["title"] == "My Track"
        assert result.data["duration_ms"] == 30000
        assert result.data["exists_on_disk"] is True

    def test_missing_track(self, setup):
        result = setup.tool.execute(track_id="0000000000000000" * 2)
        assert not result.success


class TestDeleteFromLibraryTool:
    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(audio_bytes=b"x", prompt="x")
        return SimpleNamespace(
            library=lib,
            track_id=saved["id"],
            tool=GenDeleteFromLibraryTool(_fake_node(), lib),
        )

    def test_deletes_track(self, setup):
        assert setup.library.count == 1
        result = setup.tool.execute(track_id=setup.track_id)
        assert result.success
        assert setup.library.count == 0

    def test_delete_missing(self, setup):
        result = setup.tool.execute(track_id="ffffffffffffffffffffffffffffffff")
        assert not result.success


class TestPlayFromLibraryTool:
    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(audio_bytes=b"x" * 200, prompt="x", title="Play Me")
        return SimpleNamespace(
            library=lib,
            track_id=saved["id"],
            tool=GenPlayFromLibraryTool(_fake_node(), lib),
        )

    def test_returns_path_and_increments_play_count(self, setup):
        before = setup.library.get_track_info(setup.track_id)["play_count"]
        result = setup.tool.execute(track_id=setup.track_id)
        assert result.success
        assert result.data["track_id"] == setup.track_id
        assert result.data["path"].endswith("track.mp3")
        assert result.data["exists_on_disk"] is True
        after = setup.library.get_track_info(setup.track_id)["play_count"]
        assert after == before + 1

    def test_missing_track_fails(self, setup):
        result = setup.tool.execute(track_id="0000000000000000" * 2)
        assert not result.success

    def test_missing_file_on_disk_fails(self, setup, tmp_path):
        # Delete the mp3 file
        track_dir = setup.library.root_dir / setup.track_id
        (track_dir / "track.mp3").unlink()
        result = setup.tool.execute(track_id=setup.track_id)
        assert not result.success
        assert "mp3" in result.error.lower()

    def test_publishes_path_to_sound_node(self, setup):
        """gen_play_from_library must publish the mp3 path to /voice/sound/play_file."""
        result = setup.tool.execute(track_id=setup.track_id)
        assert result.success
        assert result.data["playback"] == "published"
        assert setup.tool._play_file_pub.publish.called


class TestSaveToLibraryTool:
    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(
            audio_bytes=b"x", prompt="x", title="Original", rating=2,
        )
        return SimpleNamespace(
            library=lib,
            track_id=saved["id"],
            tool=GenSaveToLibraryTool(_fake_node(), lib),
        )

    def test_updates_rating(self, setup):
        result = setup.tool.execute(track_id=setup.track_id, rating=5)
        assert result.success
        info = setup.library.get_track_info(setup.track_id)
        assert info["rating"] == 5

    def test_updates_tags_and_mood(self, setup):
        result = setup.tool.execute(
            track_id=setup.track_id,
            tags="favorite,wedding",
            mood="romantic",
        )
        assert result.success
        info = setup.library.get_track_info(setup.track_id)
        assert info["tags"] == ["favorite", "wedding"]
        assert info["mood"] == "romantic"

    def test_rating_minus_one_keeps_existing(self, setup):
        original_rating = setup.library.get_track_info(setup.track_id)["rating"]
        result = setup.tool.execute(track_id=setup.track_id, title="New Title", rating=-1)
        assert result.success
        info = setup.library.get_track_info(setup.track_id)
        assert info["rating"] == original_rating  # unchanged

    def test_missing_track_fails(self, setup):
        result = setup.tool.execute(track_id="ffffffffffffffffffffffffffffffff", rating=5)
        assert not result.success
