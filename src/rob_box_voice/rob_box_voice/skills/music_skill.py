"""
music_skill.py — MusicSkill: sub-agent for live music generation via Renardo/SuperCollider.

Tools exposed to the sub-agent (issue #1371 — namespace split to avoid LLM
semantic collisions between Renardo track-code and the new AI-generated
mp3 library, issue #1361):

  Renardo / SuperCollider live-engine (prefixed ``renardo_``):
    renardo_search_samples — filesystem sample search (finds letter+index for play())
    execute_music_code    — execute Renardo code in the running SC instance
    stop_music            — stop a pattern or all music
    set_vibe_preset       — apply a named vibe preset
    get_music_state       — query current music state
    search_artist_style   — DuckDuckGo search for artist/concept style (issue #1000)
    renardo_list_tracks   — list saved Renardo track-blocks (Python code snapshots)
    renardo_save_track    — save Renardo Python-code snapshot to library
    renardo_load_track    — re-run a saved Renardo code block
    renardo_delete_track  — remove a Renardo track-block

  AI-generated music (MiniMax Music API, prefix ``gen_``) — issue #1358/#1361:
    generate_music         — compose a NEW song via MiniMax (40-160s per track)
    gen_list_library       — list saved AI mp3 tracks
    gen_search_library     — FTS5 / tag search across AI mp3 metadata
    gen_save_to_library    — update tags/name/mood/genre for an AI track
    gen_play_from_library  — play an AI mp3 via the audio backend
    gen_delete_from_library — delete an AI mp3 from the library (irreversible)
    gen_get_track_info     — full metadata for one AI track

  Mode-independent:
    set_dj_mode            — toggle autonomous DJ mode (Renardo-only)

The Renardo reference documentation is loaded at init time from a configurable
path (RENARDO_REF_PATH env var or the renardo_ref_path constructor argument) and
substituted into the prompt template via {renardo_ref}.

Renardo samples are searched in RENARDO_SAMPLES_PATH (default /renardo_samples).
"""

import json
import logging
import os
from pathlib import Path
from typing import Optional

from agents import function_tool

from .base_skill import BaseSkill

# Module logger (used by the new function_tool closures)
logger = logging.getLogger(__name__)

# ── Issue #1358 — generated-music library (MiniMax Music API) ─────────────
# Optional imports: if the new modules aren't present, the new tools
# degrade to "library unavailable" and the rest of the skill still works.
try:
    from rob_box_voice.core.music_library import GeneratedMusicLibrary
except Exception:  # noqa: BLE001
    GeneratedMusicLibrary = None  # type: ignore[assignment,misc]

try:
    from rob_box_voice.core.minimax_music_client import (
        MinimaxMusicClient,
        MinimaxMusicError,
    )
except Exception:  # noqa: BLE001
    MinimaxMusicClient = None  # type: ignore[assignment,misc]
    # Keep MinimaxMusicError as a placeholder Exception subclass; if the
    # real import failed, the import line above will be referenced via
    # its local name which is bound to this fallback class.
    class _MinimaxMusicErrorFallback(Exception):
        status_code = None
        retry_after_s = None

    MinimaxMusicError = _MinimaxMusicErrorFallback  # type: ignore[assignment,misc]

# ── DuckDuckGo search (free, no API key) — issue #1000 ──────────────────────
# Used by search_artist_style for DJ-mode "research first" workflow.
# Tries the new ``ddgs`` package first, then falls back to the legacy
# ``duckduckgo_search`` name (pre-8.0).
try:
    from ddgs import DDGS
    _DDGS_AVAILABLE = True
except ImportError:
    try:
        from duckduckgo_search import DDGS  # legacy name
        _DDGS_AVAILABLE = True
    except ImportError:
        _DDGS_AVAILABLE = False

# ── Default paths ─────────────────────────────────────────────────────────────
_DEFAULT_RENARDO_REF_PATH = os.getenv(
    "RENARDO_REF_PATH",
    "/renardo_ref/RENARDO_REFERENCE.md",
)
_DEFAULT_SAMPLES_PATH = os.getenv(
    "RENARDO_SAMPLES_PATH",
    "/root/.config/renardo/samples",
)


class MusicSkill(BaseSkill):
    """Sub-agent that handles all music-related requests using Renardo/SuperCollider.

    Args:
        adapter:           Shared LLMToolCallAdapter.
        model:             Shared OpenAIChatCompletionsModel.
        prompt_template:   Prompt text with {renardo_ref} placeholder.
        renardo_ref_path:  Path to RENARDO_REFERENCE.md (mounted volume).
        samples_path:      Root directory of renardo sample packs.
        **kwargs:          Forwarded to BaseSkill (temperature, max_tokens, ...).
    """

    def __init__(
        self,
        adapter,
        model,
        prompt_template: str,
        renardo_ref_path: str = _DEFAULT_RENARDO_REF_PATH,
        samples_path: str = _DEFAULT_SAMPLES_PATH,
        # Issue #1358 — generated-music wiring (all optional, see music_skill_prompt.txt)
        music_library=None,
        minimax_client=None,
        music_library_root: Optional[str] = None,
        voice_memory_db: Optional[str] = None,
        minimax_api_key: Optional[str] = None,
        minimax_model: Optional[str] = None,
        **kwargs,
    ) -> None:
        renardo_ref = self._load_renardo_ref(renardo_ref_path)
        if music_library_root is None:
            music_library_root = os.getenv("MUSIC_LIBRARY_ROOT", "/data/music_library")
        if voice_memory_db is None:
            voice_memory_db = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        if minimax_api_key is None:
            minimax_api_key = os.getenv("MINIMAX_API_KEY")
        if minimax_model is None:
            minimax_model = os.getenv("MINIMAX_MUSIC_MODEL", "music-3.0")
        filled_prompt = prompt_template.replace("{renardo_ref}", renardo_ref)
        # Carry any extra-placeholders through the same .replace() chain.
        filled_prompt = filled_prompt.replace(
            "{music_library_enabled}",
            "enabled" if (music_library is not None or GeneratedMusicLibrary is not None) else "disabled",
        )
        super().__init__(
            adapter=adapter,
            model=model,
            prompt=filled_prompt,
            name="MusicSkill",
            **kwargs,
        )
        self._samples_path = samples_path

        # ── Issue #1358 — generated-music wiring (lazy + safe) ──────────
        self._library = music_library
        self._client = minimax_client
        self._minimax_model = minimax_model
        if self._library is None and GeneratedMusicLibrary is not None:
            try:
                self._library = GeneratedMusicLibrary(
                    library_root=music_library_root,
                    db_path=voice_memory_db,
                )
            except Exception as exc:  # noqa: BLE001
                # Library is optional — keep going with the rest of the skill.
                import logging as _log
                _log.getLogger(__name__).warning(
                    "⚠️ [MusicSkill] generated-music library unavailable: %s", exc
                )
                self._library = None
        if self._client is None and MinimaxMusicClient is not None:
            try:
                self._client = MinimaxMusicClient(api_key=minimax_api_key)
            except Exception as exc:  # noqa: BLE001
                import logging as _log
                _log.getLogger(__name__).warning(
                    "⚠️ [MusicSkill] MiniMax client unavailable: %s", exc
                )
                self._client = None

    @staticmethod
    def _load_renardo_ref(path: str) -> str:
        """Load RENARDO_REFERENCE.md from the given path, or return empty string."""
        try:
            p = Path(path)
            if p.exists():
                return p.read_text(encoding="utf-8")
        except Exception:
            pass
        return ""

    def _make_tools(self) -> list:
        # Capture samples_path for renardo_search_samples closure
        samples_root = Path(self._samples_path)

        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        # ── renardo_search_samples ────────────────────────────────────────
        # Filesystem-based: reads renardo sample packs directly.
        # Returns JSON with letter, sample_index, and ready-to-use play_code.
        # Issue #1371: renamed from `search_samples` to disambiguate from
        # the AI-generated ``gen_search_library`` tool. The new name
        # explicitly tags this as a RENARDO live-engine helper.

        @function_tool
        def renardo_search_samples(
            query: str,
            pack: str = "0_foxdot_default",
            case: str = "lower",
        ) -> str:
            """Search Renardo sample packs by keyword (RENARDO ENGINE).

            Use this when you need an UNKNOWN sample letter / index for
            ``play()`` or want to browse the sample library. Do NOT call
            it for known built-ins like vocal ``c`` or standard kick/snare
            letters already covered by the prompt.

            Filenames describe the sonic character of the sample
            (e.g. ``Kick1.wav``, ``Snare_rim.wav``).

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool operates on the RENARDO live-engine sample packs
            (filesystem under RENARDO_SAMPLES_PATH). For searching the
            AI-generated mp3 library (MiniMax Music API) use
            ``gen_search_library`` instead.

            Args:
                query: Keyword to search for (e.g. "kick", "snare", "hat", "bass",
                       "synth", "vocal", "glitch").  Use "*" for a compact overview
                       of all available letters across the pack.
                pack:  Sample pack name: "0_foxdot_default" (standard) or
                       "1_pitchglitch_samples" (extended, includes vocals/FX).
                case:  "lower" → lowercase letter in play();
                       "upper" → uppercase letter in play().

            Returns:
                JSON with found results: letter, sample_index, filename, play_code.
            """
            from rob_box_voice.core.sample_search import search_renardo_samples

            result = search_renardo_samples(samples_root, query, pack, case)

            # Enrich with hints for the LLM before serialising
            if "error" in result:
                if "available_packs" not in result and "hint" not in result:
                    result["hint"] = "Mount RENARDO_SAMPLES_PATH volume in Docker"
            elif "letters" in result:
                # overview mode
                result["hint"] = (
                    'Search by word: renardo_search_samples("kick") or '
                    'renardo_search_samples("synth", pack="1_pitchglitch_samples")'
                )
            elif result.get("found", 0) == 0:
                result["hint"] = (
                    'Try: "kick", "snare", "hat", "bass", "synth", "dist", "loop", "*"'
                )

            return json.dumps(result, ensure_ascii=False, indent=2)

        # ── MCP-backed music tools ─────────────────────────────────────────

        @function_tool
        async def execute_music_code(code: str, pattern_name: str = "p1") -> str:
            """Execute Renardo code to create or update a music pattern.

            Args:
                code:         Valid Renardo Python code string.
                pattern_name: Pattern name for history tracking (e.g. "p1", "drums").
            """
            return await _call(
                "execute_music_code",
                {"code": code, "pattern_name": pattern_name},
                timeout=15.0,
            )

        @function_tool
        async def stop_music(pattern_name: str = "") -> str:
            """Stop a named pattern or all music.

            Args:
                pattern_name: Pattern name (e.g. "p1") or "" / "all" to stop everything.
            """
            params = {"pattern_name": pattern_name} if pattern_name else {}
            return await _call("stop_music", params)

        @function_tool
        async def set_vibe_preset(preset_name: str) -> str:
            """Apply a vibe preset that configures BPM, scale, and root key.

            Args:
                preset_name: One of 'chill', 'energetic', 'ambient', 'jazz',
                             'dark', 'rock', 'latin', 'electronic', 'cinematic',
                             'funk', 'reggae', 'classical'. Must be passed as
                             ``preset_name`` (NOT ``preset`` — issue #935: prior
                             ``preset`` naming made the LLM think the MCP tool
                             expected ``preset`` and got rejected with
                             "Отсутствует обязательный параметр: preset_name").
            """
            return await _call("set_vibe_preset", {"preset_name": preset_name})

        @function_tool
        async def get_music_state() -> str:
            """Get current music state: SC availability, active patterns, preset."""
            return await _call("get_music_state", {})

        # ── search_artist_style (DuckDuckGo, free) — issue #1000 ──────────
        # MANDATORY FIRST STEP in DJ-mode before execute_music_code / handle_music.
        # Research style, genre, BPM, key, instruments and mood by artist name
        # OR concept (DJ themes like "летняя дискотека хаус", "ритуальная музыка").
        @function_tool
        def search_artist_style(artist_name: str, song_names: str = "") -> str:
            """Search for music style, genre, BPM, key, instruments and mood by artist name OR concept.

            Use this for:
            - Artists/bands: "Егор Летов", "Radiohead", "Kraftwerk", "Daft Punk"
            - DJ themes/concepts: "летняя дискотека хаус", "робот-диджей"
            - Performer roles: "Пастырь культа" → "ритуальная музыка хоралы"
            - Any music request: always research the style first!

            Args:
                artist_name: Name of the artist, band, OR concept/persona to research.
                    For concepts: describe the style/mood/associations (e.g. "ритуальная
                    музыка культ хоралы" for a cult preacher persona).
                song_names: Optional comma-separated song/album names if user
                    mentioned specific tracks (e.g. "Русское поле экспериментов,
                    Гражданская оборона"). Searches for chords and structure.
            """
            if not _DDGS_AVAILABLE:
                return json.dumps(
                    {"error": "duckduckgo-search not installed", "artist": artist_name},
                    ensure_ascii=False,
                )

            # Core queries for artist style
            queries = [
                f"{artist_name} жанр стиль музыки",
                f"{artist_name} звучание инструменты темп",
                f"{artist_name} music genre BPM key instruments",
            ]

            # If user mentioned specific songs/albums — search for chords and structure
            if song_names and song_names.strip():
                for song in song_names.split(","):
                    song = song.strip()
                    if song:
                        queries.append(f"{song} {artist_name} аккорды тональность")
                        queries.append(f"{song} {artist_name} song structure tempo")

            snippets = []
            try:
                with DDGS() as ddgs:
                    for q in queries:
                        for r in ddgs.text(q, max_results=3, region="wt-wt"):
                            title = r.get("title", "")
                            body = r.get("body", "")
                            if body:
                                snippets.append(f"**{title}**: {body}")
            except Exception as e:
                return json.dumps(
                    {"error": str(e), "artist": artist_name},
                    ensure_ascii=False,
                )

            if not snippets:
                return json.dumps(
                    {"artist": artist_name, "found": False,
                     "hint": "Try spelling the name differently or use the original language name."},
                    ensure_ascii=False,
                )

            # Trim to keep token usage reasonable
            combined = "\n\n".join(snippets[:10])
            # ~2500 chars max to avoid bloating the context
            if len(combined) > 2500:
                combined = combined[:2500] + "..."

            return json.dumps(
                {
                    "artist": artist_name,
                    "found": True,
                    "research": combined,
                    "instruction": (
                        "Based on the above, adapt your Renardo code: "
                        "match the genre (BPM, scale, rhythm pattern), "
                        "use appropriate instruments/sounds, "
                        "recreate the characteristic mood and energy."
                    ),
                },
                ensure_ascii=False,
                indent=2,
            )

        # ── renardo_list_tracks / renardo_save_track / renardo_load_track / ──
        # ── renardo_delete_track  (RENARDO ENGINE track-block library) ────
        # Issue #1371: renamed from `list_tracks` / `save_track` /
        # `load_track` / `delete_track` to disambiguate from the AI-generated
        # ``gen_*_library`` family.  These tools persist and replay
        # **Renardo Python code snapshots** (track-blocks), NOT mp3 files.
        # The MCP-имя в ``_call()`` оставлено без изменений (это протокол
        # между skill и MCP-сервером — на стороне MCP эти имена зашиты в
        # music_library_backend'е Renardo).

        @function_tool
        async def renardo_list_tracks(tag: str = "", min_rating: int = 0) -> str:
            """List saved Renardo track-blocks (RENARDO ENGINE).

            ALWAYS call this when the user asks about saved Renardo
            tracks / melodies / DJ-set blocks — do NOT answer from memory!

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool lists **Renardo Python code snapshots** (live
            patterns). For listing AI-generated mp3 tracks (MiniMax
            Music API), use ``gen_list_library`` instead.

            Args:
                tag: Filter by tag, e.g. 'full_track', 'robot_authored', 'minor'. Empty = all.
                min_rating: Show only tracks rated this or higher (0-5).
            """
            params: dict = {}
            if tag:
                params["tag"] = tag
            if min_rating:
                params["min_rating"] = min_rating
            return await _call("list_tracks", params)

        @function_tool
        async def renardo_save_track(name: str, title: str = "", description: str = "",
                                     tags: str = "", rating: int = 0, notes: str = "") -> str:
            """Save the current/last Renardo track-block to the persistent library (RENARDO ENGINE).

            Use when the user says 'сохрани этот бит', 'remember this
            melody', etc.  Stores the Renardo Python code snapshot for
            later replay via ``renardo_load_track``.

            ⚠️ DISAMBIGUATION (issue #1371):
            This persists Renardo Python code, NOT an mp3 file. For
            updating metadata of an AI-generated track use
            ``gen_save_to_library``.

            Args:
                name: Unique slug identifier (e.g. 'chill_dnb_v1').
                title: Human-readable title.
                description: Mood, structure, notes about the track.
                tags: Comma-separated tags (e.g. 'chill,minor,90bpm').
                rating: Star rating 0-5.
                notes: Personal notes.
            """
            params: dict = {"name": name}
            if title:
                params["title"] = title
            if description:
                params["description"] = description
            if tags:
                params["tags"] = [t.strip() for t in tags.split(",") if t.strip()]
            if rating:
                params["rating"] = rating
            if notes:
                params["notes"] = notes
            return await _call("save_track", params)

        @function_tool
        async def renardo_load_track(name: str) -> str:
            """Load a saved Renardo track-block and replay it (RENARDO ENGINE).

            Use when the user asks to play a saved Renardo track by
            name.  Call ``renardo_list_tracks()`` first to get the
            exact slug.

            ⚠️ DISAMBIGUATION (issue #1371):
            This replays a Renardo Python code block live in SC. For
            playing an AI-generated mp3 use ``gen_play_from_library``.

            Args:
                name: Track slug (e.g. 'csm_132_full_track').
            """
            return await _call("load_track", {"name": name})

        @function_tool
        async def renardo_delete_track(name: str) -> str:
            """Delete a saved Renardo track-block (irreversible, RENARDO ENGINE).

            Use when the user says 'удали этот renardo-бит'.  For
            deleting AI-generated mp3 tracks use
            ``gen_delete_from_library``.

            Args:
                name: Track slug to delete.
            """
            return await _call("delete_track", {"name": name})

        @function_tool
        async def set_dj_mode(enabled: bool, next_transition_sec: int = 0, theme: str = "") -> str:
            """Enable or disable autonomous DJ mode with optional party theme.

            In DJ mode the robot automatically makes smooth music transitions
            like a live DJ at a party, adapting to the given theme.

            Workflow:
                1. Start thematic music: execute_music_code(...)
                2. Enable DJ mode: set_dj_mode(enabled=True, next_transition_sec=45,
                   theme="8 марта, женский день")
                3. Robot will autonomously evolve patterns AND periodically make
                   thematic announcements (e.g., congratulate women on March 8th).
                4. At the END of every DJ transition call this again with the chosen
                   next_transition_sec (no need to re-send theme — it's remembered).
                5. To stop: set_dj_mode(enabled=False), then stop_music() if needed.

            Args:
                enabled: True to enable, False to disable.
                next_transition_sec: Seconds until next auto-transition (15–300).
                    YOU decide based on the set: fast/energetic → 30–40s,
                    slow/ambient → 60–90s. ALWAYS provide when enabled=True.
                theme: Party theme / context (e.g. '8 марта', 'halloween', 'корпоратив 90-х',
                    'день рождения Антона'). Pass ONLY on first activation — remembered until
                    disabled. Robot will tailor music and occasional speech to this theme.
            """
            params: dict = {"enabled": enabled}
            if next_transition_sec:
                params["next_transition_sec"] = next_transition_sec
            if theme:
                params["theme"] = theme
            return await _call("set_dj_mode", params)

        # ── Issue #1358 — generated-music tools (MiniMax API) ────────────
        # These tools let the LLM compose a brand-new track on demand
        # (slow, 40-160s per track) and manage a persistent library of
        # AI-generated MP3s.  Use Renardo for live-loop / DJ work; use
        # these when the user wants a real, sung, structured song.
        #
        # Issue #1371: every AI-generated library tool was prefixed with
        # ``gen_`` so the LLM can no longer confuse ``list_library`` /
        # ``search_library`` with ``renardo_list_tracks`` /
        # ``renardo_search_samples``.  When the user says «библиотека»
        # without further qualifier, default to the NEW (gen_*) tools —
        # that's the use-case we ship to consumers.
        _library = self._library
        _client = self._client
        _minimax_model = self._minimax_model
        _LOG = logging.getLogger(__name__)

        @function_tool
        async def generate_music(
            prompt: str,
            lyrics: str = "[Instrumental]",
            mood: str = "",
            genre: str = "",
            instrumental: bool = False,
            save_to_lib: bool = True,
            timeout_s: int = 180,
        ) -> str:
            """Generate a NEW song with lyrics + melody via MiniMax Music API.

            Use this when the user wants a real, complete song — a sung
            track with lyrics, a verse/chorus structure, full arrangement
            (NOT a live Renardo loop).  Generation takes 40–160 seconds.

            Workflow (this tool handles all of it):
              1. POST https://api.minimax.io/v1/music_generation
                 (model "music-3.0", sample_rate 44.1kHz, 256kbps MP3).
              2. Save MP3 to /data/music_library/<uuid>/track.mp3.
              3. Persist metadata + tags in the generated-music library.
              4. Return the track_id + file_path so the user can play it.

            Args:
                prompt:     Style / mood / genre / instruments / tempo
                            description in English.  Be SPECIFIC.
                            Example: "warm romantic ballad, soft piano
                            arpeggios, C minor, 80 bpm, female vocal".
                lyrics:     Lyrics with optional section markers
                            ``[Verse] [Chorus] [Bridge]``.  Default
                            ``[Instrumental]`` → purely instrumental track.
                mood:       Optional tag, e.g. "romantic", "energetic".
                genre:      Optional tag, e.g. "ballad", "rock", "jazz".
                instrumental: When True, force ``[Instrumental]`` lyrics.
                save_to_lib: When True (default), persist the track in the
                             library immediately so the user can find it
                             again with gen_search_library().
                timeout_s:  Hard wall-time cap in seconds (default 180).

            Returns:
                JSON with track_id, file_path, duration_s, model, library
                stats.  If generation fails: ``{"error": "..."}``.
            """
            if _client is None:
                return json.dumps(
                    {"error": "MinimaxMusicClient unavailable (MINIMAX_API_KEY not set?)"},
                    ensure_ascii=False,
                )

            if instrumental or not lyrics or not lyrics.strip():
                lyrics = "[Instrumental]"

            try:
                import asyncio as _asyncio

                async def _progress(payload: dict) -> None:
                    _LOG.info(
                        "🎼 [generate_music] %.0fs elapsed — %s",
                        payload.get("elapsed_s", 0),
                        payload.get("hint", ""),
                    )

                result = await _client.generate_with_progress(
                    prompt=prompt,
                    lyrics=lyrics,
                    model=_minimax_model,
                    progress_cb=_progress,
                    timeout_s=float(timeout_s),
                )
            except MinimaxMusicError as exc:
                return json.dumps(
                    {
                        "error": str(exc),
                        "status_code": exc.status_code,
                        "retry_after_s": exc.retry_after_s,
                    },
                    ensure_ascii=False,
                )
            except Exception as exc:  # noqa: BLE001
                return json.dumps(
                    {"error": f"{type(exc).__name__}: {exc}"},
                    ensure_ascii=False,
                )

            track_payload: dict = {
                "track_id": None,
                "file_path": None,
                "duration_s": result.duration_s,
                "model": result.model,
                "wall_time_s": round(result.wall_time_s, 2),
                "prompt": prompt,
                "lyrics": lyrics,
                "mood": mood,
                "genre": genre,
            }

            if save_to_lib and _library is not None:
                try:
                    import uuid as _uuid

                    track_id = _uuid.uuid4().hex
                    file_path = _library.track_file_path(track_id)
                    _library.ensure_track_dir(track_id)
                    with open(file_path, "wb") as fh:
                        fh.write(result.audio_bytes)
                    track = _library.save(
                        track_id=track_id,
                        prompt=prompt,
                        lyrics=lyrics,
                        model=result.model,
                        file_path=file_path,
                        duration_s=result.duration_s,
                        file_size=len(result.audio_bytes),
                        mood=mood,
                        genre=genre,
                        tags=[t for t in (mood, genre) if t],
                    )
                    track_payload["track_id"] = track.track_id
                    track_payload["file_path"] = track.file_path
                    track_payload["saved"] = True
                except Exception as exc:  # noqa: BLE001
                    track_payload["saved"] = False
                    track_payload["save_error"] = f"{type(exc).__name__}: {exc}"
            else:
                track_payload["saved"] = False
                track_payload["save_to_lib"] = False

            return json.dumps(track_payload, ensure_ascii=False, indent=2)

        @function_tool
        def gen_save_to_library(
            track_id: str,
            tags: str = "",
            mood: str = "",
            genre: str = "",
            name: str = "",
        ) -> str:
            """Update metadata for an already-generated track in the AI music library.

            Use this when the user says "сохрани этот трек как 'для
            Ивана'" or wants to add tags / mood / genre to the most
            recent track.

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool updates metadata of an AI-generated mp3. For
            persisting a Renardo Python track-block use
            ``renardo_save_track``.

            Args:
                track_id: UUID4 hex returned by generate_music().
                tags:     Comma-separated tags, e.g. "chill,rainy,minor".
                mood:     Optional mood tag, e.g. "romantic".
                genre:    Optional genre tag, e.g. "jazz".
                name:     Optional human-readable title.
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            existing = _library.get(track_id)
            if existing is None:
                return json.dumps(
                    {"error": f"track_id {track_id!r} not found in library"},
                    ensure_ascii=False,
                )
            tag_list = [t.strip() for t in (tags or "").split(",") if t.strip()]
            merged_tags = list(dict.fromkeys(list(existing.tags) + tag_list))
            saved = _library.save(
                track_id=existing.track_id,
                prompt=existing.prompt,
                lyrics=existing.lyrics,
                model=existing.model,
                file_path=existing.file_path,
                duration_s=existing.duration_s,
                file_size=existing.file_size,
                mood=mood or existing.mood,
                genre=genre or existing.genre,
                lang=existing.lang,
                tags=merged_tags,
                name=name or existing.name,
            )
            return json.dumps(
                {"ok": True, "track": saved.to_dict()}, ensure_ascii=False, indent=2
            )

        @function_tool
        def gen_search_library(
            query: str = "",
            tags: str = "",
            mood: str = "",
            genre: str = "",
            limit: int = 5,
        ) -> str:
            """Search the AI-generated mp3 library by keyword, mood, or genre.

            ALWAYS call this when the user asks for "тот трек про
            дождь", "найди что-то романтичное", "что у нас про лето?"
            — do NOT answer from memory.

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool searches the AI-generated mp3 library (MiniMax
            Music API). For searching Renardo sample packs use
            ``renardo_search_samples``.

            Args:
                query:  Free-text query (searches prompt + lyrics + name).
                tags:   Comma-separated tags to filter by.
                mood:   Filter by mood (substring match).
                genre:  Filter by genre (substring match).
                limit:  Max results to return (default 5).
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            tag_list = [t.strip() for t in (tags or "").split(",") if t.strip()]
            results = _library.search(
                query=query or "",
                tags=tag_list or None,
                mood=mood or None,
                genre=genre or None,
                limit=limit,
            )
            # Issue #1358 e2e: distinctive log line.
            _LOG.info(
                "🎼 [issue 1358] gen_search_library: query=%r mood=%r genre=%r tags=%r → %d hits",
                query, mood, genre, tag_list, len(results),
            )
            return json.dumps(
                {
                    "query": query,
                    "count": len(results),
                    "tracks": [t.to_dict() for t in results],
                },
                ensure_ascii=False,
                indent=2,
            )

        @function_tool
        def gen_list_library(limit: int = 20, sort_by: str = "recent") -> str:
            """List all tracks in the AI-generated mp3 library.

            Use when the user asks "что у нас в библиотеке?", "покажи
            все треки", or before picking one to play.

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool lists AI-generated mp3s (MiniMax Music API).
            For listing saved Renardo track-blocks use
            ``renardo_list_tracks``. When the user says «библиотека»
            without further qualifier, default to THIS tool.

            Args:
                limit:  Max tracks to return (default 20).
                sort_by: "recent" (default) or "popular" (by play_count).
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            tracks = _library.list_all(limit=limit, sort_by=sort_by)
            total = _library.count()
            # Issue #1358 e2e: distinctive log line that the e2e harness
            # can grep for to confirm the new tool was actually called.
            _LOG.info(
                "🎼 [issue 1358] gen_list_library: total=%d shown=%d sort=%s",
                total, len(tracks), sort_by,
            )
            return json.dumps(
                {
                    "sort_by": sort_by,
                    "total": total,
                    "shown": len(tracks),
                    "tracks": [t.to_dict() for t in tracks],
                },
                ensure_ascii=False,
                indent=2,
            )

        @function_tool
        async def gen_play_from_library(track_id: str) -> str:
            """Play a track from the AI-generated mp3 library.

            Locates the MP3 file, increments the play count, and publishes
            a play trigger so the audio node can stream it.

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool plays an AI-generated mp3 (MiniMax Music API).
            For replaying a saved Renardo track-block use
            ``renardo_load_track``.

            Args:
                track_id: UUID4 hex returned by generate_music() or
                          visible in gen_list_library() / gen_search_library().
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            track = _library.get(track_id)
            if track is None:
                return json.dumps(
                    {"error": f"track_id {track_id!r} not found in library"},
                    ensure_ascii=False,
                )
            if not track.file_path or not os.path.exists(track.file_path):
                return json.dumps(
                    {
                        "error": f"audio file missing for {track_id} (file_path={track.file_path!r})",
                    },
                    ensure_ascii=False,
                )
            new_count = _library.increment_play_count(track_id)

            # Publish to MCP for the audio node to pick up.  We use a
            # dedicated tool name (play_mp3_file) so the MCP server can
            # dispatch it to whatever playback backend is wired up.
            play_result = await _call(
                "play_mp3_file",
                {
                    "file_path": track.file_path,
                    "track_id": track_id,
                    "duration_s": track.duration_s,
                    "name": track.name,
                },
                timeout=10.0,
            )

            return json.dumps(
                {
                    "ok": True,
                    "track_id": track_id,
                    "file_path": track.file_path,
                    "play_count": new_count,
                    "playback": play_result,
                },
                ensure_ascii=False,
                indent=2,
            )

        @function_tool
        def gen_delete_from_library(track_id: str) -> str:
            """Delete a track from the AI-generated mp3 library (irreversible).

            Use when the user says "удали тот грустный трек" or
            "удали последний".  ALWAYS call gen_list_library() or
            gen_search_library() first to confirm the track_id — do NOT
            guess from context.

            ⚠️ DISAMBIGUATION (issue #1371):
            This tool deletes an AI-generated mp3 (MiniMax Music API).
            For deleting a saved Renardo track-block use
            ``renardo_delete_track``.

            Args:
                track_id: UUID4 hex of the track to remove.
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            deleted = _library.delete(track_id)
            return json.dumps(
                {"ok": deleted, "track_id": track_id},
                ensure_ascii=False,
            )

        @function_tool
        def gen_get_track_info(track_id: str) -> str:
            """Return full metadata for one AI-generated track in the library.

            Use when the user asks "что за трек?", "покажи информацию",
            or when you need prompt+lyrics+tags before deciding to play
            it.

            Args:
                track_id: UUID4 hex of the track.
            """
            if _library is None:
                return json.dumps({"error": "library unavailable"}, ensure_ascii=False)
            track = _library.get(track_id)
            if track is None:
                return json.dumps(
                    {"error": f"track_id {track_id!r} not found"},
                    ensure_ascii=False,
                )
            return json.dumps(track.to_dict(), ensure_ascii=False, indent=2)

        return [
            # Renardo live-engine tools (issue #1371 — namespaced with renardo_
            # prefix where the name was ambiguous with the new AI library).
            renardo_search_samples, execute_music_code, stop_music,
            set_vibe_preset, get_music_state, search_artist_style,
            renardo_list_tracks, renardo_save_track, renardo_load_track,
            renardo_delete_track, set_dj_mode,
            # AI-generated music tools (issue #1358/#1361; namespaced with
            # gen_ prefix in issue #1371).
            generate_music, gen_save_to_library, gen_search_library,
            gen_list_library, gen_play_from_library, gen_delete_from_library,
            gen_get_track_info,
        ]
