#!/usr/bin/env python3
"""Регрессия фильтра годности партитуры по известным темам: прежнее точное окно vs выравнивание (#3573).

Офлайн, только чтение. Для каждой темы из :data:`THEMES` берёт партитуры из ``score_index.db`` (по названию) и
RTTTL-эталоны архива (``rtttl_melodies.jsonl.gz``), считает для пары «материал x тема» старый балл (точное окно
интервалов, копия кода до #3573 — только для сравнения, в проде её нет) и новый (``hook.theme_match``), решение при
порогах 0.8 (сет) и 0.7 (одиночная пьеса) и печатает флипы и сводку «сколько тем узнаваемо» до/после, а также время
``theme_match`` на самой толстой теме.

    # 1. список файлов партитур (потом скопировать их с робота /opt/rob_box/scores/ в --scores)
    python theme_gate_regression.py --index score_index.db --list-files > files.txt
    #    tar -C /opt/rob_box/scores -T files.txt -cf - | tar -C <scores-dir> -xf -
    # 2. прогон
    python theme_gate_regression.py --index score_index.db --scores <scores-dir>
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import re
import sqlite3
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(_ROOT, "src", "rob_box_music"))

from rob_box_music.arrange import hook as hooks  # noqa: E402
from rob_box_music.material import from_dict  # noqa: E402

ARCHIVE = os.path.join(_ROOT, "src", "rob_box_mcp_tools", "rob_box_mcp_tools", "data", "rtttl_melodies.jsonl.gz")
PER_THEME = 40
#: тема: (метка, regex названия партитуры, regex композитора или "", имена RTTTL-эталонов архива)
THEMES: Tuple[Tuple[str, str, str, Tuple[str, ...]], ...] = (
    ("Grieg: Hall of the Mountain King", r"mountain king", "",
     ("hallofth", "hallofth_2", "hallofth_3", "mountainking")),
    ("Mozart: Eine kleine Nachtmusik", r"nachtmusik|eine kleine", "", ("eineklei",)),
    ("Beethoven: Fur Elise", r"f[uü]r elise|fur elise", "", ("furelise",)),
    ("Beethoven: Ode to Joy", r"ode to joy|freude", "", ("odetojoy", "odetojoy_2")),
    ("Beethoven: 5th Symphony", r"5th symphony|fifth symphony|symphony (no\.? ?)?5", "beethoven", ("5thsymph_2",)),
    ("Vivaldi: Spring", r"spring|primavera", "vivaldi", ("4seasons_3",)),
    ("Tchaikovsky: Swan Lake", r"swan lake", "", ("swanlake", "swanlake_2")),
    ("Tchaikovsky: Sugar Plum Fairy", r"sugar ?plum", "", ("danceoft_2",)),
    ("Tetris / Korobeiniki", r"tetris|korobeiniki", "", ("tetris", "tetris_2", "tetris_3")),
    ("Kalinka", r"kalinka", "", ("kalinkav", "kalinkav_2")),
    ("Star Wars: Imperial March", r"imperial march|star wars", "", ("starwars_3", "starwars_4", "starwars_5")),
    ("Mozart: Turkish March", r"turkish march|alla turca", "", ("turkishm",)),
    ("Pachelbel: Canon", r"canon", "pachelbel", ("canon", "canon_2")),
    ("Rossini: William Tell", r"william tell", "", ("williamt", "williamt_2")),
    ("Rimsky-Korsakov: Bumblebee", r"bumble", "", ("bumblebe", "flightof", "flightof_2")),
    ("Wedding March", r"wedding march", "", ("weddingm", "weddingm_2", "weddingm_3", "weddingm_4")),
    ("Greensleeves", r"greensleeves", "", ("greensle",)),
    ("Bach: Air on the G string", r"air on|g string|g-saite", "bach", ("aironthe",)),
    ("Bach: Toccata and Fugue BWV 565", r"toccata", "bach", ("toccata", "toccata_2", "toccataa", "toccataa_2")),
)
THRESHOLDS = (0.8, 0.7)


def legacy_score(material, refs: Sequence[Tuple[int, ...]]) -> Optional[float]:
    """Копия сравнения до #3573: доля совпавших интервалов в лучшем окне длины эталона (без вставок/пропусков)."""
    best: Optional[float] = None
    for events in hooks.voices(material).values():
        pitches = [e.midi for e in events]
        steps = [b - a for a, b in zip(pitches, pitches[1:])]
        for ref in refs:
            last = len(steps) - len(ref)
            if last < 0:
                continue
            at: Dict[int, List[int]] = {}
            for j, step in enumerate(steps):
                at.setdefault(step, []).append(j)
            hits = [0] * (last + 1)
            for k, step in enumerate(ref):
                for j in at.get(step, ()):
                    if 0 <= j - k <= last:
                        hits[j - k] += 1
            score = max(hits) / len(ref)
            best = score if best is None or score > best else best
    return best


def select(conn: sqlite3.Connection, title_rx: str, composer_rx: str) -> List[Tuple[str, str, str, str]]:
    rx, crx = re.compile(title_rx, re.I), re.compile(composer_rx, re.I)
    rows = conn.execute("SELECT material_id, title, composer, file FROM score_index "
                        "ORDER BY COALESCE(n_views, 0) DESC, material_id").fetchall()
    out = [r for r in rows if rx.search(r[1] or "")
           and (not composer_rx or crx.search((r[2] or "") + " " + (r[1] or "")))]
    return out[:PER_THEME]


def load_refs(names: Sequence[str]) -> List[str]:
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        by_name = {r["name"]: r["rtttl"] for r in (json.loads(line) for line in fh if line.strip())}
    return [by_name[n] for n in names if n in by_name]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--index", required=True)
    ap.add_argument("--scores", default="")
    ap.add_argument("--list-files", action="store_true")
    ap.add_argument("--max-flips", type=int, default=40)
    args = ap.parse_args()
    conn = sqlite3.connect(f"file:{args.index}?mode=ro", uri=True)
    picked = {label: select(conn, trx, crx) for label, trx, crx, _n in THEMES}
    if args.list_files:
        print("\n".join(sorted({r[3] for rows in picked.values() for r in rows})))
        return 0
    flips: List[Tuple[str, str, str, float, float, float]] = []
    recognised = {t: [0, 0] for t in (0.8, 0.7)}
    timing: Optional[Tuple[str, int, float, float]] = None
    skipped = 0
    print(f"{'тема':36} {'партитур':>8} {'узнаётся0.8 до/после':>22} {'0.7 до/после':>14}")
    for label, _t, _c, names in THEMES:
        refs = hooks.reference_contours(load_refs(names))
        mats = []
        for mid, title, composer, fname in picked[label]:
            path = os.path.join(args.scores, fname)
            try:
                with open(path, encoding="utf-8") as fh:
                    mats.append((from_dict(json.load(fh)), title))
            except (OSError, ValueError, KeyError, TypeError):
                skipped += 1
        rec = {0.8: [False, False], 0.7: [False, False]}
        for mat, title in mats:
            old = legacy_score(mat, refs)
            got = hooks.theme_match(mat, refs)
            new = got[0] if got else None
            if old is None or new is None:
                continue
            for thr in THRESHOLDS:
                a, b = old >= thr, new >= thr
                rec[thr][0] |= a
                rec[thr][1] |= b
                if a != b:
                    kind = "принят->отвергнут" if a else "отвергнут->принят"
                    flips.append((label, f"{title[:55]} [{mat.material_id[-8:]}]", f"{thr}: {kind}", thr, old, new))
        for thr in THRESHOLDS:
            recognised[thr][0] += rec[thr][0]
            recognised[thr][1] += rec[thr][1]
        print(f"{label:36} {len(mats):8d} {int(rec[0.8][0])} -> {int(rec[0.8][1]):<18} "
              f"{int(rec[0.7][0])} -> {int(rec[0.7][1])}")
        if mats and (timing is None or len(mats) > timing[1]):
            t0 = time.perf_counter()
            for mat, _t2 in mats:
                hooks._MATCH_CACHE.clear()
                hooks.theme_match(mat, refs)
            new_t = time.perf_counter() - t0
            t0 = time.perf_counter()
            for mat, _t2 in mats:
                legacy_score(mat, refs)
            timing = (label, len(mats), new_t, time.perf_counter() - t0)
    print(f"\nфлипов всего: {len(flips)}; не загружено партитур: {skipped}")
    for thr in THRESHOLDS:
        down = sum(1 for f in flips if f[3] == thr and "принят->отвергнут" in f[2])
        up = sum(1 for f in flips if f[3] == thr and "отвергнут->принят" in f[2])
        print(f"порог {thr}: принят->отвергнут {down}, отвергнут->принят {up}; тем с >=1 проходящим материалом: "
              f"{recognised[thr][0]} -> {recognised[thr][1]} из {len(THEMES)}")
    print("\nфлипы (сначала принят->отвергнут):")
    ordered = sorted(flips, key=lambda f: ("отвергнут->принят" in f[2], f[0]))
    for label, title, kind, _thr, old, new in ordered[:args.max_flips]:
        print(f"  {label[:28]:28} | {title:68} | {kind:26} | {old:.2f} -> {new:.2f}")
    if timing:
        print(f"\nсамая толстая тема: {timing[0]}, {timing[1]} материалов: theme_match {timing[2]:.3f} с "
              f"(старое окно {timing[3]:.3f} с), без кэша совпадений")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
