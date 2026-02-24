"""
music_skill.py — MusicSkill: sub-agent for live music generation via Renardo/SuperCollider.

Tools exposed to the sub-agent:
  search_samples    — filesystem sample search (finds letter + index for play())
  execute_music_code — execute Renardo code in the running SC instance
  stop_music         — stop a pattern or all music
  set_vibe_preset    — apply a named vibe preset (chill/energetic/ambient/jazz/dark)
  get_music_state    — query current music state

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

# ── Default paths ─────────────────────────────────────────────────────────────
_DEFAULT_RENARDO_REF_PATH = os.getenv(
    "RENARDO_REF_PATH",
    "/renardo_ref/RENARDO_REFERENCE.md",
)
_DEFAULT_SAMPLES_PATH = os.getenv(
    "RENARDO_SAMPLES_PATH",
    "/renardo_samples",
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

            ALWAYS call this before creating music patterns!  Filenames describe
            the sonic character of the sample (e.g. "Kick1.wav", "Snare_rim.wav").

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
            if not samples_root.exists():
                return json.dumps(
                    {"error": f"Samples dir not found: {samples_root}",
                     "hint": "Mount RENARDO_SAMPLES_PATH volume in Docker"},
                    ensure_ascii=False,
                )

            pack_path = samples_root / pack
            if not pack_path.exists():
                available = [d.name for d in samples_root.iterdir() if d.is_dir()]
                return json.dumps(
                    {"error": f"Pack '{pack}' not found", "available_packs": available},
                    ensure_ascii=False,
                )

            exts = {".wav", ".aif", ".aiff", ".mp3"}

            # Calculate spack index (0-based position in sorted pack list)
            all_packs = sorted([d.name for d in samples_root.iterdir() if d.is_dir()])
            spack_num = all_packs.index(pack) if pack in all_packs else 0
            spack_suffix = f", spack={spack_num}" if spack_num != 0 else ""

            # query="*" → compact overview of letters and counts
            if query.strip() == "*":
                overview = {}
                for folder in sorted(pack_path.iterdir()):
                    if not folder.is_dir() or folder.name.startswith("."):
                        continue
                    sub = folder / case
                    if not sub.exists():
                        sub = folder
                    count = sum(1 for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts)
                    if count:
                        overview[folder.name] = count
                return json.dumps(
                    {
                        "pack": pack,
                        "case": case,
                        "letters": overview,
                        "hint": 'Search by word: search_samples("kick") or search_samples("synth", pack="1_pitchglitch_samples")',
                    },
                    ensure_ascii=False,
                    indent=2,
                )

            # Keyword search
            q = query.lower().strip()
            results = []
            for folder in sorted(pack_path.iterdir()):
                if not folder.is_dir() or folder.name.startswith("."):
                    continue
                sub = folder / case
                if not sub.exists():
                    sub = folder
                files = sorted(
                    [f for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts]
                )
                for idx, f in enumerate(files):
                    if q in f.name.lower():
                        play_letter = folder.name.upper() if case == "upper" else folder.name
                        results.append(
                            {
                                "letter": play_letter,
                                "sample_index": idx,
                                "spack": spack_num,
                                "filename": f.name,
                                "play_code": f'd1 >> play("{play_letter}", sample={idx}{spack_suffix})',
                            }
                        )
                if len(results) >= 30:
                    break

            if not results:
                return json.dumps(
                    {
                        "query": query,
                        "pack": pack,
                        "found": 0,
                        "hint": 'Try: "kick", "snare", "hat", "bass", "synth", "dist", "loop", "*"',
                    },
                    ensure_ascii=False,
                )

            return json.dumps(
                {"query": query, "pack": pack, "case": case, "found": len(results), "results": results},
                ensure_ascii=False,
                indent=2,
            )

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
        async def set_vibe_preset(preset: str) -> str:
            """Apply a vibe preset that configures BPM, scale, and root key.

            Args:
                preset: One of 'chill', 'energetic', 'ambient', 'jazz', 'dark'.
            """
            return await _call("set_vibe_preset", {"preset": preset})

        @function_tool
        async def get_music_state() -> str:
            """Get current music state: SC availability, active patterns, preset."""
            return await _call("get_music_state", {})

        return [search_samples, execute_music_code, stop_music, set_vibe_preset, get_music_state]
