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

#: Шаг поворота окна результатов между вызовами с одним и тем же запросом.
#:
#: Зачем: раньше поиск шёл по ``sorted()`` и обрывался на первых
#: ``MAX_RESULTS`` совпадениях, то есть ``search_samples("kick")`` ВСЕГДА
#: возвращал одни и те же алфавитно первые сэмплы. LLM не «выбирала одно и
#: то же» — ей просто ни разу не показали ничего другого. Поворот окна на
#: каждом вызове раскрывает всю коллекцию.
#:
#: 7 — взаимно простое с типичными размерами наборов, поэтому окно
#: сдвигается по всей выборке, а не топчется на нескольких позициях.
ROTATION_STRIDE = 7


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def search_renardo_samples(
    samples_root: Path,
    query: str,
    pack: str = "0_foxdot_default",
    case: str = "lower",
    rotate: int = 0,
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
        rotate: Which window of the match set to return when there are more
            than :data:`MAX_RESULTS` matches. The function stays pure and
            deterministic — the *caller* supplies an incrementing counter so
            that repeating the same query surfaces different samples. See
            :data:`ROTATION_STRIDE`.

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
        - ``found`` (int) — how many were returned
        - ``total_found`` (int) — how many exist in total
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
    # Собираем ВСЕ совпадения, а не первые MAX_RESULTS: без полного набора
    # нельзя ни сказать, сколько их на самом деле, ни показать разные при
    # повторном запросе.
    q = query.lower().strip()
    matches: List[Dict[str, Any]] = []
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
                matches.append(
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

    if not matches:
        return {
            "query": query,
            "pack": pack,
            "case": case,
            "found": 0,
            "total_found": 0,
        }

    total_found = len(matches)
    if total_found > MAX_RESULTS:
        # Поворачиваем окно, а не срезаем начало: иначе каждый вызов с одним
        # и тем же запросом отдаёт одни и те же сэмплы и вся остальная
        # коллекция для LLM не существует.
        start = (int(rotate) * ROTATION_STRIDE) % total_found
        results = [matches[(start + i) % total_found] for i in range(MAX_RESULTS)]
    else:
        results = matches

    return {
        "query": query,
        "pack": pack,
        "case": case,
        # ``found`` — сколько вернули (контракт не менялся).
        "found": len(results),
        # ``total_found`` — сколько есть на самом деле. Без него LLM видит
        # «найдено 30» при двухстах доступных и считает коллекцию бедной.
        "total_found": total_found,
        "results": results,
    }
