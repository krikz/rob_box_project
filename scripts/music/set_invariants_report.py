#!/usr/bin/env python3
"""Отчёт инвариантов сета H1–H7 (ADR-0157 §5) на сид-серии: стили × сеты × треки, офлайн, без робота.

Хуки тем — из архива RTTTL (как на роботе: ``ThemeProfile.hook_ids`` → мелодии архива), сеты — ``seeded_plan`` по
сиду, треки — ``compose`` подряд с историей. Метрики — :mod:`rob_box_music.set_invariants`; H7 дополнительно по
программе рендера: доля треков, где ``Transition.filter_in`` дал ``hpf=`` входа деки.

    python scripts/music/set_invariants_report.py --sets 4 --tracks 5

``--no-pulse-guard`` — без рантайм-гарда H1 в ``model.validate`` (замер «до» на коде, где гард уже есть).
"""

from __future__ import annotations

import argparse
import gzip
import json
import pathlib
import sys
from typing import Dict, List, Optional, Sequence

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))  # пакет этого чекаута, не editable-установка основного

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music import model  # noqa: E402
from rob_box_music.arrange.compose import compose  # noqa: E402
from rob_box_music.diversity import track_composition  # noqa: E402
from rob_box_music.render.renardo import render  # noqa: E402
from rob_box_music.set_invariants import summary  # noqa: E402
from rob_box_music.set_plan import seeded_plan  # noqa: E402
from rob_box_music.theme import seeded_profile  # noqa: E402

ARCHIVE = REPO / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
THEMES = ("космос", "дождь", "дорога", "киберпанк", "море", "лес")
COLUMNS = ("tracks", "h1", "h3", "h3_checked", "h6", "h2_median", "h4_rest_median", "h4_run16_max", "h7_ducked",
           "with_kick", "h7_hpf")


def load_melodies() -> Dict[str, str]:
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        return {rec["name"]: rec["rtttl"] for rec in map(json.loads, filter(str.strip, fh))}


def style_sets(style: str, n_sets: int, n_tracks: int, melodies: Dict[str, str]) -> List[List[model.Track]]:
    out = []
    for seed in range(n_sets):
        profile = seeded_profile(THEMES[seed % len(THEMES)], style)
        plan = seeded_plan(profile, seed, n_tracks, set_id=f"inv{seed}")
        mel = {i: melodies[i] for i in profile.hook_ids if i in melodies}
        history: List[dict] = []
        run = []
        for no in range(1, n_tracks + 1):
            track = compose(plan, no, melodies=mel, history=history, deck="AB"[no % 2])
            history.insert(0, track_composition(track))
            run.append(track)
        out.append(run)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sets", type=int, default=4)
    ap.add_argument("--tracks", type=int, default=5)
    ap.add_argument("--styles", nargs="*", default=list(kn.STYLES))
    ap.add_argument("--no-pulse-guard", action="store_true")
    args = ap.parse_args(argv)
    if args.no_pulse_guard:
        model._check_pulse = lambda track: None
    melodies = load_melodies()
    sets = {style: style_sets(style, args.sets, args.tracks, melodies) for style in args.styles}
    table = summary(sets)
    for style, runs in sets.items():
        tracks = [t for run in runs for t in run]
        table[style]["h7_hpf"] = sum("hpf=var(" in render(t, "A").code for t in tracks)
    print("| style | " + " | ".join(COLUMNS) + " |")
    print("|---" * (len(COLUMNS) + 1) + "|")
    for style, row in table.items():
        cells = [f"{row[c]:g}" if isinstance(row[c], float) else str(row[c]) for c in COLUMNS]
        print(f"| {style} | " + " | ".join(cells) + " |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
