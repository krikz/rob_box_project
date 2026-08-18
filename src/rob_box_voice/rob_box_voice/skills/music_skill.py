"""
music_skill.py — MusicSkill: sub-agent for live music generation via Renardo/SuperCollider.

Tools exposed to the sub-agent:
  search_samples    — filesystem sample search (finds letter + index for play())
  execute_music_code — execute Renardo code in the running SC instance
  stop_music         — stop a pattern or all music
  set_vibe_preset    — apply a named vibe preset (chill/energetic/ambient/jazz/dark)
  get_music_state    — query current music state
  search_artist_style — DuckDuckGo search for artist/concept style (issue #1000)

The Renardo reference documentation is loaded at init time from a configurable
path (RENARDO_REF_PATH env var or the renardo_ref_path constructor argument) and
substituted into the prompt template via {renardo_ref}.

Renardo samples are searched in RENARDO_SAMPLES_PATH (default /renardo_samples).
"""

import json
import os
from pathlib import Path

from agents import function_tool

from .base_skill import BaseSkill

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
        **kwargs,
    ) -> None:
        renardo_ref = self._load_renardo_ref(renardo_ref_path)
        filled_prompt = prompt_template.replace("{renardo_ref}", renardo_ref)
        super().__init__(
            adapter=adapter,
            model=model,
            prompt=filled_prompt,
            name="MusicSkill",
            **kwargs,
        )
        self._samples_path = samples_path

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
        # Capture samples_path for search_samples closure
        samples_root = Path(self._samples_path)

        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        # ── search_samples ─────────────────────────────────────────────────
        # Filesystem-based: reads renardo sample packs directly.
        # Returns JSON with letter, sample_index, and ready-to-use play_code.

        @function_tool
        def search_samples(
            query: str,
            pack: str = "0_foxdot_default",
            case: str = "lower",
        ) -> str:
            """Search for renardo samples by keyword in the filename.

            Call this when you need an UNKNOWN sample letter / index or want to
            browse the sample library. Do NOT call it for known built-ins like
            vocal `c` or standard kick/snare letters already covered by the prompt.
            Filenames describe the sonic character of the sample (e.g. "Kick1.wav", "Snare_rim.wav").

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
                    'Search by word: search_samples("kick") or '
                    'search_samples("synth", pack="1_pitchglitch_samples")'
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

        @function_tool
        async def list_tracks(tag: str = "", min_rating: int = 0) -> str:
            """List saved tracks in the robot's music library.

            ALWAYS call this when the user asks about saved tracks, melodies or
            the music library — do NOT answer from memory!

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
        async def save_track(name: str, title: str = "", description: str = "",
                             tags: str = "", rating: int = 0, notes: str = "") -> str:
            """Save the current or last played track to the persistent music library.

            Use when the user says 'save this track', 'remember this melody', etc.

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
        async def load_track(name: str) -> str:
            """Load a track from the library and play it.

            Use when the user asks to play a saved track by name.
            Call list_tracks() first to get the exact name.

            Args:
                name: Track slug (e.g. 'csm_132_full_track').
            """
            return await _call("load_track", {"name": name})

        @function_tool
        async def delete_track(name: str) -> str:
            """Delete a track from the music library (irreversible).

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

        # ── MiniMax music generation + library tools (issue #1392) ──────
        # Routes through MCP server's minimax_music tools (added in
        # mcp_server.py:_register_minimax_music_tools). Renardo/SuperCollider
        # tools above stay first-choice for live synthesis; MiniMax is the
        # fallback when the user asks for a full vocal track with lyrics.
        @function_tool
        async def generate_music(prompt: str, lyrics: str = "",
                                 is_instrumental: bool = False,
                                 mood: str = "", genre: str = "",
                                 lang: str = "", tags: str = "",
                                 title: str = "") -> str:
            """Generate a brand-new vocal/instrumental track via MiniMax Music API.

            CALL THIS WHEN THE USER ASKS FOR A FULL SONG WITH LYRICS
            ("спой песню про бычка", "сыграй романтичную песню") and there is no
            matching track in the library (call gen_search_library first!).

            Takes 40-160 seconds (avg ~88s). MUST warn the user before calling:
            "Сейчас сгенерирую, это займёт около минуты — подожди, не уходи!"

            Args:
                prompt:    Style/mood description (1-2000 chars). REQUIRED.
                lyrics:    Song lyrics with [Verse]/[Chorus] tags. Required unless
                           is_instrumental=True.
                is_instrumental: True for instrumental-only (no vocals).
                mood:      Mood tag (e.g. 'romantic', 'dark').
                genre:     Genre tag (e.g. 'indie folk', 'synthwave').
                lang:      Language of vocals ('ru', 'en').
                tags:      Extra tags via comma (for library search later).
                title:     Human-readable title for the track.

            Returns:
                JSON with track_id, path to mp3, duration_ms, model used.
                To play it back later: gen_play_from_library(track_id=...).
            """
            params: dict = {"prompt": prompt, "auto_save": True}
            if lyrics:
                params["lyrics"] = lyrics
            if is_instrumental:
                params["is_instrumental"] = True
            if mood:
                params["mood"] = mood
            if genre:
                params["genre"] = genre
            if lang:
                params["lang"] = lang
            if tags:
                params["tags"] = tags
            if title:
                params["title"] = title
            # LONG timeout: MiniMax avg 88s (issue #1358).
            return await _call("generate_music", params, timeout=200.0)

        @function_tool
        async def gen_list_library(limit: int = 20, sort_by: str = "recent",
                                   tag: str = "", mood: str = "") -> str:
            """List tracks in the AI-generated music library (/data/music_library).

            Args:
                limit:    Max results (1-100, default 20).
                sort_by:  'recent' (default) / 'popular' / 'rating'.
                tag:      Filter by tag (exact match).
                mood:     Filter by mood (exact match).

            Returns:
                JSON with tracks[] (each: id, title, prompt, tags, mood, genre,
                duration_ms, rating, play_count, created_at) and total count.
            """
            params: dict = {"limit": limit, "sort_by": sort_by}
            if tag:
                params["tag"] = tag
            if mood:
                params["mood"] = mood
            return await _call("gen_list_library", params)

        @function_tool
        async def gen_search_library(query: str, limit: int = 5) -> str:
            """Search AI-generated music library by keyword.

            Searches title/prompt/lyrics/genre/mood/notes (substring match).

            Args:
                query: Search string (1-200 chars). REQUIRED.
                limit: Max results (1-20, default 5).

            Returns:
                JSON with tracks[] and total count. Empty total = nothing found.
            """
            params: dict = {"query": query, "limit": limit}
            return await _call("gen_search_library", params)

        @function_tool
        async def gen_save_to_library(track_id: str, title: str = "",
                                      tags: str = "", mood: str = "",
                                      genre: str = "", rating: int = -1,
                                      notes: str = "") -> str:
            """Update metadata (tags/rating/notes/mood/genre/title) for a saved track.

            Args:
                track_id: UUID of the track (get from gen_list_library / generate_music).
                title:    New title (empty = don't change).
                tags:     New comma-separated tags (replaces existing).
                mood:     New mood (replaces existing).
                genre:    New genre (replaces existing).
                rating:   New rating 0-5 (-1 = don't change).
                notes:    New notes (replaces existing).
            """
            params: dict = {"track_id": track_id}
            if title:
                params["title"] = title
            if tags:
                params["tags"] = tags
            if mood:
                params["mood"] = mood
            if genre:
                params["genre"] = genre
            if rating >= 0:
                params["rating"] = rating
            if notes:
                params["notes"] = notes
            return await _call("gen_save_to_library", params)

        @function_tool
        async def gen_play_from_library(track_id: str) -> str:
            """Return path to a saved track for playback (path-based).

            NOTE: Full mp3 playback wiring is a separate audio_node task.
            This tool returns the path + duration so you can tell the user
            "трек готов: /data/music_library/<id>/track.mp3 (93 секунды)".

            Args:
                track_id: UUID of the track.

            Returns:
                JSON with track_id, path, title, duration_ms, exists_on_disk.
            """
            return await _call("gen_play_from_library", {"track_id": track_id})

        @function_tool
        async def gen_delete_from_library(track_id: str) -> str:
            """Delete a track from the AI-generated music library (irreversible).

            Args:
                track_id: UUID of the track.
            """
            return await _call("gen_delete_from_library", {"track_id": track_id})

        @function_tool
        async def gen_get_track_info(track_id: str) -> str:
            """Get detailed metadata for a single track.

            Returns: id, title, prompt, lyrics, tags, mood, genre,
            duration_ms, sample_rate, bitrate, play_count, rating,
            created_at, path, exists_on_disk.
            """
            return await _call("gen_get_track_info", {"track_id": track_id})

        return [
            search_samples, execute_music_code, stop_music, set_vibe_preset,
            get_music_state, search_artist_style, list_tracks, save_track,
            load_track, delete_track, set_dj_mode,
            generate_music, gen_list_library, gen_search_library,
            gen_save_to_library, gen_play_from_library, gen_delete_from_library,
            gen_get_track_info,
        ]
