#!/usr/bin/env python3
"""Политика тонов баса из корпуса партитур — таблица ``knowledge.BASS_TONES`` (ADR-0154 §3.4, Н7, PR-4).

Вход — ``stats.json`` из ``score_material_probe.py`` (поля ``mode``, ``bass_rel``, ``bass_approach``, ``bars``):
нота басового голоса относительно аккорда полутакта — прима/терция/квинта/чужая, подход полутоном к первой доле
такта. Таблица — доли по ладу (мажор/минор лада music21) и подходов на такт; подсчёт, без ГСЧ — одинаковый вход
даёт одинаковые байты.

    python scripts/music/research/score_material_probe.py <каталог> <файлы…> --out stats.json
    python scripts/music/research/score_bass_tones.py stats.json \
        --write-table src/rob_box_music/rob_box_music/data/bass_tones.json
"""

from __future__ import annotations

import argparse
import collections
import datetime
import hashlib
import json
import pathlib
import sys
from typing import Dict, List

RELATIONS = ("root", "third", "fifth", "other")


def mode_table(rows: List[Dict], mode: str) -> Dict[str, float]:
    """Доли тонов баса и подходов на такт по партитурам лада ``mode``."""
    rel = collections.Counter()
    approach = bars = 0
    for r in rows:
        if r["mode"] != mode:
            continue
        rel.update(r["bass_rel"])
        approach += r["bass_approach"]
        bars += r["bars"]
    total = sum(rel[k] for k in RELATIONS)
    out = {k: round(rel[k] / total, 4) for k in RELATIONS}
    out["approach_per_bar"] = round(approach / max(1, bars), 4)
    out["notes"], out["bars"], out["approaches"] = total, bars, approach
    return out


def corpus_digest(rows: List[Dict]) -> str:
    corpus = sorted((r["file"], r["mode"], sorted(r["bass_rel"].items()), r["bass_approach"], r["bars"]) for r in rows)
    return hashlib.sha256(json.dumps(corpus, ensure_ascii=False).encode()).hexdigest()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stats")
    ap.add_argument("--write-table", default="", help="записать таблицу (JSON knowledge.BASS_TONES)")
    ap.add_argument("--corpus-label", default="", help="описание корпуса для провенанса таблицы")
    args = ap.parse_args()
    data = json.load(open(args.stats, encoding="utf-8"))
    rows = [r for r in data["rows"] if "error" not in r and r.get("bass_rel")]
    tables = {mode: mode_table(rows, mode) for mode in ("major", "minor")}
    digest = corpus_digest(rows)
    for mode, t in tables.items():
        print(f"{mode}: " + "  ".join(f"{k} {t[k]:.3f}" for k in RELATIONS)
              + f"  approach/bar {t['approach_per_bar']:.4f}  (нот {t['notes']}, тактов {t['bars']}, "
                f"подходов {t['approaches']})")
    print(f"партитур {len(rows)}, sha256 корпуса {digest[:12]}")
    if args.write_table:
        table = {
            "provenance": {
                "corpus": args.corpus_label or "musetrainer/library (PD) + 2 Interstellar (локально, не в git); "
                                               "docs/music/research_score_material.md",
                "scores": len(rows), "modes": dict(collections.Counter(r["mode"] for r in rows)),
                "corpus_sha256": digest, "stats": pathlib.Path(args.stats).name,
                "date": datetime.date.today().isoformat(),
                "script": "scripts/music/research/score_bass_tones.py --write-table",
                "relations": "низшая звучащая нота онсета против трезвучия полутакта "
                             "(score_material_probe.analyze); approach — полутон к первой доле такта"},
            "tables": tables,
        }
        pathlib.Path(args.write_table).write_text(json.dumps(table, ensure_ascii=False, indent=1) + "\n",
                                                  encoding="utf-8")
        print(f"таблица → {args.write_table}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
