"""Unit tests for the new generated-music function_tools on :class:`MusicSkill`.

These tests cover the integration of the 6 new function tools with the
``GeneratedMusicLibrary`` + ``MinimaxMusicClient`` pair that the
``MusicSkill`` constructor wires together.  They exercise the tools
*directly* (without going through the LLM loop) by invoking the
underlying closures that ``_make_tools()`` returns.

If the ``agents`` SDK is not available in the test env, the test
module is skipped — the skill itself imports ``from agents import
function_tool`` at the top level.
"""

from __future__ import annotations

import asyncio
import base64
import json
import os
import sys
import tempfile
import types
from pathlib import Path
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import httpx
import pytest

# Skip the whole module when the agents SDK is unavailable.
try:
    import agents  # noqa: F401
    HAS_AGENTS_SDK = True
except ImportError:
    HAS_AGENTS_SDK = False

pytestmark = pytest.mark.skipif(
    not HAS_AGENTS_SDK, reason="openai-agents SDK not installed in this env"
)


# ---------------------------------------------------------------------------
# Fakes / fixtures
# ---------------------------------------------------------------------------


class _FakeAdapter:
    """Stand-in for LLMToolCallAdapter — captures tool calls."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, dict]] = []

    def execute_tool_call_sync(self, name, params, timeout=10.0):
        self.calls.append((name, dict(params)))
        return {"result": "ok"}


def _make_response(status: int, body: dict[str, Any]) -> httpx.Response:
    req = httpx.Request("POST", "https://api.minimax.io/v1/music_generation")
    return httpx.Response(status, json=body, request=req)


@pytest.fixture()
def tmp_env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> dict[str, str]:
    db = tmp_path / "vm.db"
    lib_root = tmp_path / "mlib"
    monkeypatch.setenv("VOICE_MEMORY_DB_PATH", str(db))
    monkeypatch.setenv("MUSIC_LIBRARY_ROOT", str(lib_root))
    monkeypatch.setenv("MINIMAX_API_KEY", "test-key")
    return {"db": str(db), "lib": str(lib_root)}


def _new_music_skill(tmp_env, *, model="music-3.0", api_handler=None):
    """Construct a MusicSkill with a fake adapter + mocked client."""
    from rob_box_voice.skills.music_skill import MusicSkill

    # Inject a mocked client so we don't hit the real API.
    from rob_box_voice.core.minimax_music_client import MinimaxMusicClient

    adapter = _FakeAdapter()
    if api_handler is not None:
        transport = httpx.MockTransport(api_handler)
        http = httpx.AsyncClient(transport=transport, base_url="https://api.minimax.io")
        client = MinimaxMusicClient(api_key="test-key", client=http)
    else:
        client = None  # disable generation calls in tests that don't need them

    skill = MusicSkill(
        adapter=adapter,
        model=object(),
        prompt_template="You are music. {renardo_ref} {music_library_enabled}",
        music_library=None,  # let __init__ build a fresh one from env
        minimax_client=client,
        minimax_model=model,
        music_library_root=tmp_env["lib"],
        voice_memory_db=tmp_env["db"],
        minimax_api_key="test-key",
    )
    return skill, adapter, client


def _run(coro):
    return asyncio.get_event_loop().run_until_complete(coro)  # type: ignore[deprecated]


def _tool_by_name(tools, name: str):
    for t in tools:
        if getattr(t, "__name__", None) == name:
            return t
    raise KeyError(f"tool {name!r} not in {[getattr(t, '__name__', '?') for t in tools]}")


# ---------------------------------------------------------------------------
# Smoke: skill construction
# ---------------------------------------------------------------------------


class TestMusicSkillConstruction:
    def test_constructor_wires_library_and_client(self, tmp_env) -> None:
        skill, _adapter, _client = _new_music_skill(tmp_env)
        assert skill._library is not None
        # Library wrote a sidecar dir
        assert Path(tmp_env["lib"]).exists()
        assert skill._client is not None

    def test_make_tools_includes_all_six_new_tools(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        names = {getattr(t, "__name__", "?") for t in tools}
        for name in {
            "generate_music", "save_to_library", "search_library",
            "list_library", "play_from_library",
            "delete_from_library", "get_track_info",
        }:
            assert name in names, f"missing tool: {name}"

    def test_make_tools_keeps_existing_tools(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        names = {getattr(t, "__name__", "?") for t in tools}
        # Pre-existing tools must still be there.
        for name in {
            "search_samples", "execute_music_code", "stop_music",
            "set_vibe_preset", "get_music_state", "search_artist_style",
            "list_tracks", "save_track", "load_track", "delete_track",
            "set_dj_mode",
        }:
            assert name in names, f"missing existing tool: {name}"


# ---------------------------------------------------------------------------
# generate_music
# ---------------------------------------------------------------------------


class TestGenerateMusic:
    def test_returns_error_when_client_unavailable(
        self, tmp_env, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        # Build skill with NO client
        from rob_box_voice.skills.music_skill import MusicSkill

        skill = MusicSkill(
            adapter=_FakeAdapter(),
            model=object(),
            prompt_template="x {music_library_enabled}",
            minimax_client=None,  # force fallback
            music_library_root=tmp_env["lib"],
            voice_memory_db=tmp_env["db"],
            minimax_api_key="",
        )
        tools = skill._make_tools()
        gen = _tool_by_name(tools, "generate_music")

        # Even with no client, the tool must return a JSON error — not crash.
        result_str = _run(gen.on_invoke_tool_async(
            None, json.dumps({"prompt": "x", "lyrics": "y"})
        )) if hasattr(gen, "on_invoke_tool_async") else None
        # The simpler path: directly call the function. The function_tool
        # decorator wraps the function; calling the underlying callable
        # works because function_tool is just a marker.
        underlying = getattr(gen, "__wrapped__", gen)
        result = _run(underlying(prompt="x", lyrics="y"))
        body = json.loads(result)
        assert "error" in body
        assert "MINIMAX_API_KEY" in body["error"]

    def test_saves_to_library_and_returns_track_id(self, tmp_env) -> None:
        audio = b"mp3 binary 12345"
        audio_hex = audio.hex()

        async def handler(req: httpx.Request) -> httpx.Response:
            return _make_response(
                200,
                {
                    "audio": audio_hex,
                    "audio_format": "mp3",
                    "duration_s": 60.0,
                    "model": "music-3.0",
                },
            )

        skill, _, _ = _new_music_skill(tmp_env, api_handler=handler)
        tools = skill._make_tools()
        gen = _tool_by_name(tools, "generate_music")
        underlying = getattr(gen, "__wrapped__", gen)
        result = _run(underlying(
            prompt="romantic piano ballad C minor 80bpm",
            lyrics="[Verse]\nHello\n[Chorus]\nWorld",
            mood="romantic",
            genre="ballad",
            save_to_lib=True,
        ))
        body = json.loads(result)
        assert body["saved"] is True
        assert body["track_id"]
        assert body["file_path"].endswith(".mp3")
        assert os.path.exists(body["file_path"])
        assert body["duration_s"] == 60.0
        # Verify library has the track
        lib = skill._library
        assert lib.get(body["track_id"]) is not None

    def test_instrumental_flag_overrides_lyrics(self, tmp_env) -> None:
        captured: list[dict[str, Any]] = []

        async def handler(req: httpx.Request) -> httpx.Response:
            captured.append(json.loads(req.content))
            return _make_response(
                200,
                {"audio": b"x".hex(), "audio_format": "mp3", "duration_s": 1.0},
            )

        skill, _, _ = _new_music_skill(tmp_env, api_handler=handler)
        tools = skill._make_tools()
        gen = _tool_by_name(tools, "generate_music")
        underlying = getattr(gen, "__wrapped__", gen)
        _run(underlying(prompt="x", lyrics="should be overridden", instrumental=True))
        assert captured[0]["lyrics"] == "[Instrumental]"

    def test_save_to_lib_false_skips_persistence(self, tmp_env) -> None:
        audio = b"x" * 16

        async def handler(req: httpx.Request) -> httpx.Response:
            return _make_response(
                200,
                {"audio": audio.hex(), "audio_format": "mp3", "duration_s": 1.0},
            )

        skill, _, _ = _new_music_skill(tmp_env, api_handler=handler)
        tools = skill._make_tools()
        gen = _tool_by_name(tools, "generate_music")
        underlying = getattr(gen, "__wrapped__", gen)
        result = _run(underlying(prompt="x", save_to_lib=False))
        body = json.loads(result)
        assert body["saved"] is False
        assert body["track_id"] is None


# ---------------------------------------------------------------------------
# Library CRUD tools
# ---------------------------------------------------------------------------


class TestLibraryTools:
    def _seed(self, skill) -> list[str]:
        ids = []
        for i, (name, mood, genre) in enumerate([
            ("rain song", "romantic", "ballad"),
            ("dark night", "dark", "ambient"),
            ("summer dance", "energetic", "house"),
        ]):
            t = skill._library.save(
                prompt=f"prompt {i}",
                name=name,
                mood=mood,
                genre=genre,
                lyrics="[Verse]\nx",
            )
            ids.append(t.track_id)
        return ids

    def test_save_to_library_updates_tags(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        save = _tool_by_name(tools, "save_to_library")
        underlying = getattr(save, "__wrapped__", save)
        result = _run(underlying(
            track_id=ids[0], tags="chill,rain", name="rainy memory"
        ))
        body = json.loads(result)
        assert body["ok"] is True
        # The track's tags now include the new ones
        track = body["track"]
        assert "chill" in track["tags"]
        assert "rain" in track["tags"]
        assert track["name"] == "rainy memory"

    def test_save_to_library_unknown_track_id(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        save = _tool_by_name(tools, "save_to_library")
        underlying = getattr(save, "__wrapped__", save)
        result = _run(underlying(track_id="does-not-exist"))
        body = json.loads(result)
        assert "error" in body

    def test_search_library_finds_by_mood(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        search = _tool_by_name(tools, "search_library")
        underlying = getattr(search, "__wrapped__", search)
        result = _run(underlying(query="", mood="dark"))
        body = json.loads(result)
        assert body["count"] == 1
        assert body["tracks"][0]["name"] == "dark night"

    def test_search_library_finds_by_keyword(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        search = _tool_by_name(tools, "search_library")
        underlying = getattr(search, "__wrapped__", search)
        result = _run(underlying(query="prompt 0"))
        body = json.loads(result)
        assert body["count"] >= 1
        assert any(t["track_id"] == ids[0] for t in body["tracks"])

    def test_list_library_recent(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        self._seed(skill)
        tools = skill._make_tools()
        lst = _tool_by_name(tools, "list_library")
        underlying = getattr(lst, "__wrapped__", lst)
        result = _run(underlying(limit=10, sort_by="recent"))
        body = json.loads(result)
        assert body["total"] == 3
        assert body["shown"] == 3

    def test_get_track_info(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        info = _tool_by_name(tools, "get_track_info")
        underlying = getattr(info, "__wrapped__", info)
        result = _run(underlying(track_id=ids[0]))
        body = json.loads(result)
        assert body["track_id"] == ids[0]
        assert body["name"] == "rain song"
        assert body["mood"] == "romantic"

    def test_get_track_info_missing(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        info = _tool_by_name(tools, "get_track_info")
        underlying = getattr(info, "__wrapped__", info)
        result = _run(underlying(track_id="ghost"))
        body = json.loads(result)
        assert "error" in body

    def test_delete_from_library(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        delete = _tool_by_name(tools, "delete_from_library")
        underlying = getattr(delete, "__wrapped__", delete)
        result = _run(underlying(track_id=ids[0]))
        body = json.loads(result)
        assert body["ok"] is True
        assert skill._library.get(ids[0]) is None

    def test_delete_from_library_missing(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        delete = _tool_by_name(tools, "delete_from_library")
        underlying = getattr(delete, "__wrapped__", delete)
        result = _run(underlying(track_id="nope"))
        body = json.loads(result)
        assert body["ok"] is False


# ---------------------------------------------------------------------------
# play_from_library
# ---------------------------------------------------------------------------


class TestPlayFromLibrary:
    def test_missing_file_returns_error(self, tmp_env) -> None:
        skill, _adapter, _ = _new_music_skill(tmp_env)
        # Insert a track whose file doesn't exist on disk
        t = skill._library.save(track_id="p1", prompt="x", file_path="/no/such/file.mp3")
        tools = skill._make_tools()
        play = _tool_by_name(tools, "play_from_library")
        underlying = getattr(play, "__wrapped__", play)
        result = _run(underlying(track_id="p1"))
        body = json.loads(result)
        assert "error" in body
        assert "missing" in body["error"].lower() or "not found" in body["error"].lower()

    def test_existing_file_publishes_via_mcp_and_bumps_play_count(
        self, tmp_env
    ) -> None:
        skill, adapter, _ = _new_music_skill(tmp_env)
        # Write a real file to disk
        track_dir = Path(tmp_env["lib"]) / "p2"
        track_dir.mkdir(parents=True, exist_ok=True)
        file_path = track_dir / "track.mp3"
        file_path.write_bytes(b"\x00" * 64)
        skill._library.save(track_id="p2", prompt="x", file_path=str(file_path))
        tools = skill._make_tools()
        play = _tool_by_name(tools, "play_from_library")
        underlying = getattr(play, "__wrapped__", play)
        result = _run(underlying(track_id="p2"))
        body = json.loads(result)
        assert body["ok"] is True
        assert body["play_count"] == 1
        # Adapter was called with play_mp3_file
        names = [c[0] for c in adapter.calls]
        assert "play_mp3_file" in names
        call_params = adapter.calls[0][1]
        assert call_params["track_id"] == "p2"
        assert call_params["file_path"] == str(file_path)

    def test_unknown_track_returns_error(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        play = _tool_by_name(tools, "play_from_library")
        underlying = getattr(play, "__wrapped__", play)
        result = _run(underlying(track_id="ghost"))
        body = json.loads(result)
        assert "error" in body
