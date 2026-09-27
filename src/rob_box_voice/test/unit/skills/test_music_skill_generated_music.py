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
        # Issue #1371 — every AI-library tool was renamed to the ``gen_`` family.
        for name in {
            "generate_music", "gen_save_to_library", "gen_search_library",
            "gen_list_library", "gen_play_from_library",
            "gen_delete_from_library", "gen_get_track_info",
        }:
            assert name in names, f"missing tool: {name}"

    def test_make_tools_keeps_existing_tools(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        names = {getattr(t, "__name__", "?") for t in tools}
        # Pre-existing tools must still be there.  Issue #1371:
        # `search_samples` → `renardo_search_samples`,
        # `list_tracks`    → `renardo_list_tracks`,
        # `save_track`     → `renardo_save_track`,
        # `load_track`     → `renardo_load_track`,
        # `delete_track`   → `renardo_delete_track`.
        for name in {
            "renardo_search_samples", "execute_music_code", "stop_music",
            "set_vibe_preset", "get_music_state", "search_artist_style",
            "renardo_list_tracks", "renardo_save_track", "renardo_load_track",
            "renardo_delete_track", "set_dj_mode",
        }:
            assert name in names, f"missing existing tool: {name}"

    def test_make_tools_renardo_ai_namespaces_do_not_collide(
        self, tmp_env
    ) -> None:
        """Issue #1371 — both namespaces must be present and disjoint.

        Regression guard for the LLM semantic-collision bug: before
        #1371 the LLM could not tell ``list_tracks`` (Renardo) from
        ``list_library`` (AI).  After the fix every ambiguous tool
        name has an explicit ``renardo_`` / ``gen_`` prefix.
        """
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        names = {getattr(t, "__name__", "?") for t in tools}
        # Both prefixes must appear.
        assert any(n.startswith("renardo_") for n in names), \
            "no renardo_* tools found"
        assert any(n.startswith("gen_") for n in names), \
            "no gen_* tools found"
        # No bare (unprefixed) library/track/track-info tools left.
        for forbidden in (
            "search_samples", "list_tracks", "save_track", "load_track",
            "delete_track", "list_library", "search_library",
            "save_to_library", "play_from_library", "delete_from_library",
            "get_track_info",
        ):
            assert forbidden not in names, \
                f"old ambiguous name still exposed: {forbidden}"


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
        save = _tool_by_name(tools, "gen_save_to_library")
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
        save = _tool_by_name(tools, "gen_save_to_library")
        underlying = getattr(save, "__wrapped__", save)
        result = _run(underlying(track_id="does-not-exist"))
        body = json.loads(result)
        assert "error" in body

    def test_search_library_finds_by_mood(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        search = _tool_by_name(tools, "gen_search_library")
        underlying = getattr(search, "__wrapped__", search)
        result = _run(underlying(query="", mood="dark"))
        body = json.loads(result)
        assert body["count"] == 1
        assert body["tracks"][0]["name"] == "dark night"

    def test_search_library_finds_by_keyword(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        search = _tool_by_name(tools, "gen_search_library")
        underlying = getattr(search, "__wrapped__", search)
        result = _run(underlying(query="prompt 0"))
        body = json.loads(result)
        assert body["count"] >= 1
        assert any(t["track_id"] == ids[0] for t in body["tracks"])

    def test_gen_list_library_recent(self, tmp_env, caplog) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        self._seed(skill)
        tools = skill._make_tools()
        lst = _tool_by_name(tools, "gen_list_library")
        underlying = getattr(lst, "__wrapped__", lst)
        import logging
        with caplog.at_level(logging.INFO, logger="rob_box_voice.skills.music_skill"):
            result = _run(underlying(limit=10, sort_by="recent"))
        body = json.loads(result)
        assert body["total"] == 3
        assert body["shown"] == 3
        # e2e acceptance: distinctive log line (issue #1358 + #1371 rename).
        assert any("issue 1358" in r.message and "gen_list_library" in r.message
                   for r in caplog.records), \
            "expected 'issue 1358 gen_list_library' log line"

    def test_get_track_info(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        info = _tool_by_name(tools, "gen_get_track_info")
        underlying = getattr(info, "__wrapped__", info)
        result = _run(underlying(track_id=ids[0]))
        body = json.loads(result)
        assert body["track_id"] == ids[0]
        assert body["name"] == "rain song"
        assert body["mood"] == "romantic"

    def test_get_track_info_missing(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        info = _tool_by_name(tools, "gen_get_track_info")
        underlying = getattr(info, "__wrapped__", info)
        result = _run(underlying(track_id="ghost"))
        body = json.loads(result)
        assert "error" in body

    def test_delete_from_library(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        ids = self._seed(skill)
        tools = skill._make_tools()
        delete = _tool_by_name(tools, "gen_delete_from_library")
        underlying = getattr(delete, "__wrapped__", delete)
        result = _run(underlying(track_id=ids[0]))
        body = json.loads(result)
        assert body["ok"] is True
        assert skill._library.get(ids[0]) is None

    def test_delete_from_library_missing(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        delete = _tool_by_name(tools, "gen_delete_from_library")
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
        play = _tool_by_name(tools, "gen_play_from_library")
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
        play = _tool_by_name(tools, "gen_play_from_library")
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
        play = _tool_by_name(tools, "gen_play_from_library")
        underlying = getattr(play, "__wrapped__", play)
        result = _run(underlying(track_id="ghost"))
        body = json.loads(result)
        assert "error" in body


# ---------------------------------------------------------------------------
# Issue #1373 — DISPATCH routing regression tests
#
# Vague user phrases from the music_library_suite_v1 e2e scenario must map to
# the correct tool (issue #1371 — semantically disambiguate Renardo live-engine
# from AI-generated mp3 library).  We can't unit-test the LLM here, but we CAN
# verify the prompt fixture carries an explicit routing hint for every
# scenario step, plus a tool-name registry check.
# ---------------------------------------------------------------------------


class TestDispatchRouting:
    """Issue #1373 — each music_library_suite_v1 phrase has a correct tool.

    Tests below guard against regression of issue #1371 (LLM semantic
    collision between Renardo ``list_tracks`` and AI ``list_library``).
    They don't invoke an LLM — they assert two static invariants:

      1. The music_skill_prompt.txt DISPATCH section explicitly maps every
         scenario phrase to a tool with the right renardo_*/gen_* prefix.
      2. ``MusicSkill._make_tools()`` exposes the routed tool (and its
         dispatcher) under the exact name referenced in DISPATCH.
    """

    PROMPT_PATH = (
        Path(__file__).resolve().parents[3]
        / "prompts"
        / "skills"
        / "music_skill_prompt.txt"
    )

    def _dispatch_section(self) -> str:
        assert self.PROMPT_PATH.exists(), f"prompt not found: {self.PROMPT_PATH}"
        text = self.PROMPT_PATH.read_text(encoding="utf-8")
        # Take everything from the DISPATCH marker to the next blank-then-non-bullet
        # block (heuristic: stop at the next "### " / "## " or 4-space-indented block
        # heading — pragmatic for the file shape used today).
        start = text.find("⚡ DISPATCH")
        assert start != -1, "DISPATCH marker missing in music_skill_prompt.txt"
        return text[start:]

    @pytest.mark.parametrize(
        "phrase, expected_tool",
        [
            # Phrases from music_library_suite_v1 — each must be reachable
            # through an explicit dispatch rule.
            ("что в библиотеке", "gen_list_library"),
            ("покажи треки", "gen_list_library"),
            ("найди трек про дождь", "gen_search_library"),
            ("что-нибудь романтичное", "gen_search_library"),
            ("сохрани этот трек", "gen_save_to_library"),
            ("запомни как", "gen_save_to_library"),
            ("удали последний", "gen_list_library"),  # two-step dispatch
            ("что за трек", "gen_get_track_info"),
            ("расскажи что это за трек", "gen_get_track_info"),
            # Renardo path — must still be reachable and unambiguous.
            ("играй renardo бит", "renardo_load_track"),
            ("сыграй renardo бит", "renardo_load_track"),
            ("играй сэмпл kick", "renardo_search_samples"),
        ],
    )
    def test_prompt_routes_phrase_to_correct_tool(
        self, phrase: str, expected_tool: str
    ) -> None:
        """User phrase → DISPATCH rule → expected tool name.

        We require the expected tool to appear in DISPATCH near the phrase
        (within ~120 chars), proving the prompt gives the LLM an explicit
        routing hint rather than letting it guess from context.
        """
        dispatch = self._dispatch_section().lower()
        phrase_l = phrase.lower()
        tool_l = expected_tool.lower()
        # Find the phrase position
        pos = dispatch.find(phrase_l)
        assert pos != -1, (
            f"phrase {phrase!r} not present in DISPATCH section — "
            f"the LLM would have to guess"
        )
        # Expected tool must appear close to the phrase (forward direction)
        window = dispatch[pos:pos + 240]
        assert tool_l in window, (
            f"phrase {phrase!r} found in DISPATCH but expected tool "
            f"{expected_tool!r} not present nearby ({window!r})"
        )

    def test_renardo_and_gen_namespaces_disjoint(self, tmp_env) -> None:
        skill, _, _ = _new_music_skill(tmp_env)
        tools = skill._make_tools()
        names = {getattr(t, "__name__", "?") for t in tools}
        renardo = {n for n in names if n.startswith("renardo_")}
        gen = {n for n in names if n.startswith("gen_")}
        # Issue #1371: ambiguous bare names must not leak.
        bare = names & {
            "search_samples", "list_tracks", "save_track", "load_track",
            "delete_track", "list_library", "search_library",
            "save_to_library", "play_from_library", "delete_from_library",
            "get_track_info",
        }
        assert not bare, f"bare ambiguous tool names leaked: {bare}"
        # Sanity: both prefixes populated so DISPATCH has something to route to.
        assert renardo, "no renardo_* tools in skill"
        assert gen, "no gen_* tools in skill"

    def test_dispatch_section_present_in_prompt(self) -> None:
        """Regression guard: prompt must keep the DISPATCH block visible.

        If somebody deletes the DISPATCH section during a future cleanup,
        the LLM will fall back to bare ``list_tracks`` vs ``list_library``
        and bug #1371 will return.  Verify the marker survives.
        """
        text = self.PROMPT_PATH.read_text(encoding="utf-8")
        assert "⚡ DISPATCH" in text
        assert "(issue #1371)" in text
        # Default-rule gate: phrase «библиотека» without clarification
        # must be routed to AI library (gen_*), NOT Renardo (renardo_*).
        idx = text.find("библиотека")
        assert idx != -1
        # Find nearest "gen_" / "renardo_" hit around the «библиотека» mention
        # that points to AI library.
        window = text[idx:idx + 220]
        assert "gen_" in window, (
            "phrase «библиотека» must route to gen_* (AI mp3 library) "
            f"by default — got window: {window!r}"
        )

    def test_get_track_info_resolves_existing_track(self, tmp_env) -> None:
        """ml06_track_info — verify gen_get_track_info returns the full track.

        Sanity check that the e2e pattern ``gen_get_track_info`` actually
        resolves a real track instead of an empty list.
        """
        skill, _, _ = _new_music_skill(tmp_env)
        # Seed one track so gen_get_track_info has something to return
        t = skill._library.save(
            prompt="romantic piano", name="rain memory", mood="romantic"
        )
        tools = skill._make_tools()
        info = _tool_by_name(tools, "gen_get_track_info")
        underlying = getattr(info, "__wrapped__", info)
        result = _run(underlying(track_id=t.track_id))
        body = json.loads(result)
        assert body["track_id"] == t.track_id
        # Mood/name come back, so e2e can grep them
        assert body["mood"] == "romantic"
        assert body["name"] == "rain memory"

    def test_delete_two_step_dispatch_resolves_track_id(self, tmp_env) -> None:
        """ml05_delete_last — list then delete round-trip works on a real track.

        This mirrors the two-step e2e pattern ``[gen_list_library,
        gen_delete_from_library]`` from the scenario: list returns the most
        recent track_id, delete removes it, second list returns one fewer.
        """
        skill, _, _ = _new_music_skill(tmp_env)
        # Seed three tracks
        ids = []
        for i, name in enumerate(("alpha", "beta", "gamma")):
            t = skill._library.save(prompt=f"prompt {i}", name=name)
            ids.append(t.track_id)
        tools = skill._make_tools()
        listing_tool = _tool_by_name(tools, "gen_list_library")
        delete_tool = _tool_by_name(tools, "gen_delete_from_library")
        list_underlying = getattr(listing_tool, "__wrapped__", listing_tool)
        delete_underlying = getattr(delete_tool, "__wrapped__", delete_tool)

        # Step 1: list
        listing = json.loads(_run(list_underlying(limit=10, sort_by="recent")))
        assert listing["total"] == 3
        first_id = listing["tracks"][0]["track_id"]
        assert first_id == ids[-1]  # most recent = last seeded

        # Step 2: delete
        del_result = json.loads(_run(delete_underlying(track_id=first_id)))
        assert del_result["ok"] is True
        assert skill._library.get(first_id) is None

        # Step 3: list again, expect 2
        listing2 = json.loads(_run(list_underlying(limit=10, sort_by="recent")))
        assert listing2["total"] == 2
