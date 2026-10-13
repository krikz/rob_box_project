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
import wave
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
    """ADR-0158 PR-2: трек кэша играет на деке ``PlayerOwner`` (wav сервера, ``render_audio``), ``ok`` — по started."""

    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(audio_bytes=b"ID3fake", prompt="space", title="Космос", duration_ms=3000)
        plays, closed, decodes = [], [], []

        def play(program, dj=None, once=False):
            plays.append((program, dj, once))
            return {"ok": True, "track_id": program.track_id}

        def run(cmd, **_):  # ffmpeg: пишет wav 16 кГц стерео на 2.5 с в последний аргумент
            decodes.append(cmd)
            with wave.open(cmd[-1], "wb") as out:
                out.setnchannels(2)
                out.setsampwidth(2)
                out.setframerate(16000)
                out.writeframes(b"\x00\x00" * 2 * 40000)

        events = {}
        node = _fake_node()
        node._player_owner = SimpleNamespace(play=play)
        node._dj_set_tool = SimpleNamespace(close_set=closed.append)
        node._music_confirm = lambda track_id: events.get("event", SimpleNamespace(event="started", fields={}))
        tool = GenPlayFromLibraryTool(node, lib, clock=lambda: 1000.0, run=run)
        return SimpleNamespace(library=lib, track_id=saved["id"], tool=tool, plays=plays, closed=closed,
                               decodes=decodes, events=events, node=node)

    def test_plays_on_the_deck_once_after_closing_the_set(self, setup):
        before = setup.library.get_track_info(setup.track_id)["play_count"]
        result = setup.tool.execute(track_id=setup.track_id)
        assert result.success, result.error
        program, dj, once = setup.plays[0]
        assert once is True and setup.closed == ["gen_play_from_library"]
        assert program.track_id == f"gen_{setup.track_id[:8]}_1000" and program.bpm == 60
        assert program.form_beats == 2.5  # длина — из декодированного wav, не из строки библиотеки
        wav = str(setup.library.root_dir / setup.track_id / "track.16k.wav")
        assert program.sample_files == frozenset({wav}) and f"loop({wav!r}" in program.code
        assert dj == {"enabled": False, "title": "Космос", "source": "generated", "library_id": setup.track_id,
                      "bpm": None}
        assert result.message == "Включаю «Космос» из библиотеки — 2 секунды."
        assert setup.library.get_track_info(setup.track_id)["play_count"] == before + 1

    def test_decodes_once(self, setup):
        setup.tool.execute(track_id=setup.track_id)
        setup.tool.execute(track_id=setup.track_id)
        assert len(setup.decodes) == 1
        cmd = setup.decodes[0]
        assert cmd[cmd.index("-ar") + 1] == "16000" and cmd[cmd.index("-ac") + 1] == "2"

    def test_not_started_is_not_ok(self, setup):
        """Владелец отказал (``rejected``) — ни фразы успеха, ни play_count."""
        setup.events["event"] = SimpleNamespace(event="rejected", fields={"reason": "missing_sample", "detail": "x"})
        result = setup.tool.execute(track_id=setup.track_id)
        assert not result.success and "missing_sample" in result.error
        assert setup.library.get_track_info(setup.track_id)["play_count"] == 0

    def test_missing_track_fails(self, setup):
        result = setup.tool.execute(track_id="0000000000000000" * 2)
        assert not result.success and setup.plays == []

    def test_missing_file_on_disk_fails(self, setup):
        (setup.library.root_dir / setup.track_id / "track.mp3").unlink()
        result = setup.tool.execute(track_id=setup.track_id)
        assert not result.success and "no_file" in result.error
        assert setup.plays == []

    def test_no_engine_no_sound_claimed(self, setup):
        setup.node._player_owner = None
        result = setup.tool.execute(track_id=setup.track_id)
        assert not result.success and "движок" in result.error


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
