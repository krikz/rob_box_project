"""
sample_search.py — Shared Renardo sample filesystem search.

Pure function: no ROS2, no Renardo runtime, no MCP. Both
``MusicSkill.search_samples`` and ``SearchSamplesTool`` delegate to
:func:`search_renardo_samples` to avoid duplicated filesystem-walk logic.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_SAMPLES_PATH = os.environ.get(
    "RENARDO_SAMPLES_PATH",
    "/root/.config/renardo/samples",
)

AUDIO_EXTENSIONS: set[str] = {".wav", ".aif", ".aiff", ".mp3"}

MAX_RESULTS = 30


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def search_renardo_samples(
    samples_root: Path,
    query: str,
    pack: str = "0_foxdot_default",
    case: str = "lower",
) -> Dict[str, Any]:
    """Search Renardo sample packs by keyword in filename.

    Args:
        samples_root: Root directory of renardo sample packs
            (e.g. ``/root/.config/renardo/samples``).
        query: Keyword to search for (``"kick"``, ``"snare"``, ``"hat"``,
            ``"bass"``, ``"synth"``, ``"vocal"``, ``"glitch"``, ``"dist"``,
            ``"loop"``). Use ``"*"`` for a compact overview of all letters
            and their sample counts.
        pack: Sample pack name — ``"0_foxdot_default"`` (standard) or
            ``"1_pitchglitch_samples"`` (extended, includes vocals/FX).
        case: Letter case in ``play()`` — ``"lower"`` or ``"upper"``.

    Returns:
        Dict with keys:

        *On error:*
        - ``error`` (str)
        - ``hint`` (str, optional)
        - ``available_packs`` (list[str], optional — when pack not found)

        *On success (overview, query="*"):*
        - ``pack`` (str)
        - ``case`` (str)
        - ``letters`` (dict[str, int]) — letter → sample count
        - ``total_samples`` (int)

        *On success (keyword search):*
        - ``query`` (str)
        - ``pack`` (str)
        - ``case`` (str)
        - ``found`` (int)
        - ``results`` (list[dict]) — each with ``letter``, ``sample_index``,
          ``spack``, ``filename``, ``play_code``
    """
    if not samples_root.exists():
        return {
            "error": f"Samples dir not found: {samples_root}",
            "hint": "Mount RENARDO_SAMPLES_PATH volume in Docker",
        }

    pack_path = samples_root / pack
    if not pack_path.exists():
        available = sorted(
            d.name for d in samples_root.iterdir() if d.is_dir()
        )
        return {
            "error": f"Pack '{pack}' not found",
            "available_packs": available,
        }

    # Calculate spack index (0-based position in sorted pack list).
    # Renardo uses ``spack=N`` in ``play("X", sample=N, spack=N)`` to
    # select which sample pack the letter/char maps to.
    all_packs = sorted(
        d.name for d in samples_root.iterdir() if d.is_dir()
    )
    spack_num = all_packs.index(pack) if pack in all_packs else 0
    spack_suffix = f", spack={spack_num}" if spack_num != 0 else ""

    # ── query="*" → compact overview ──────────────────────────────────
    if query.strip() == "*":
        overview: Dict[str, int] = {}
        for folder in sorted(pack_path.iterdir()):
            if not folder.is_dir() or folder.name.startswith("."):
                continue
            sub = folder / case
            if not sub.exists():
                sub = folder
            count = sum(
                1
                for f in sub.iterdir()
                if f.is_file() and f.suffix.lower() in AUDIO_EXTENSIONS
            )
            if count:
                overview[folder.name] = count
        return {
            "pack": pack,
            "case": case,
            "letters": overview,
            "total_samples": sum(overview.values()),
        }

    # ── Keyword search ────────────────────────────────────────────────
    q = query.lower().strip()
    results: List[Dict[str, Any]] = []
    for folder in sorted(pack_path.iterdir()):
        if not folder.is_dir() or folder.name.startswith("."):
            continue
        sub = folder / case
        if not sub.exists():
            sub = folder
        files = sorted(
            f
            for f in sub.iterdir()
            if f.is_file() and f.suffix.lower() in AUDIO_EXTENSIONS
        )
        for idx, f in enumerate(files):
            if q in f.name.lower():
                play_letter = (
                    folder.name.upper() if case == "upper" else folder.name
                )
                results.append(
                    {
                        "letter": play_letter,
                        "sample_index": idx,
                        "spack": spack_num,
                        "filename": f.name,
                        "play_code": (
                            f'd1 >> play("{play_letter}", sample={idx}'
                            f"{spack_suffix})"
                        ),
                    }
                )
                if len(results) >= MAX_RESULTS:
                    break
        if len(results) >= MAX_RESULTS:
            break

    if not results:
        return {
            "query": query,
            "pack": pack,
            "case": case,
            "found": 0,
        }

    return {
        "query": query,
        "pack": pack,
        "case": case,
        "found": len(results),
        "results": results,
    }
