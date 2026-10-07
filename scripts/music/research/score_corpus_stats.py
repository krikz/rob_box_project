#!/usr/bin/env python3
"""Библиотека ``ScoreMaterial`` (JSON) → ``stats.json`` для перевыучивания таблиц §3.4 (ADR-0154, PR-6).

``score_markov_harmony.py --write-table`` (``PROGRESSION_TRANSITIONS``) и ``score_bass_tones.py --write-table``
(``BASS_TONES``) читают строки ``{file, mode, degree_seq, bass_rel, bass_approach, bars}`` — раньше их давал
``score_material_probe.py`` по MusicXML (71 партитура, один процесс). Этот скрипт берёт то же из готового импорта
(каталог JSON библиотеки, ``score_import.py``/``pdmx_batch.py``), поэтому корпус любого размера считается без повторного
разбора MusicXML:

* ``degree_seq`` — ступени слитых аккордов подряд (``ChordSpan.degree``; ``None`` — недиатонический), как у пробы;
* ``bass_rel`` — нота басового голоса против аккорда, звучащего в её долю: прима/терция/квинта/чужая;
* ``bass_approach`` — подход полутоном к первой доле следующего такта; ``bars`` — тактов с басом.

Только ладов ``major``/``minor`` (другие в таблицы §3.4 не входят). Подсчёт детерминирован: файлы по алфавиту.

    python scripts/music/research/score_corpus_stats.py <каталог библиотеки> --out stats.json [--jobs 4] [--limit N]
"""

from __future__ import annotations

import argparse
import bisect
import json
import multiprocessing
import pathlib
import sys
from typing import Any, Dict, List, Optional, Sequence

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music import material as mt  # noqa: E402

MODES = ("major", "minor")


def bass_relations(m: mt.ScoreMaterial) -> Dict[str, Any]:
    """Тоны баса материала против аккордов и подходы; ``{}`` — у материала нет баса или аккордов."""
    if not m.bass or not m.chords:
        return {}
    starts = [c.beat for c in m.chords]
    bar = mt.bar_beats(m.meter)
    rel = {"root": 0, "third": 0, "fifth": 0, "other": 0}
    approach = 0
    for i, e in enumerate(m.bass):
        k = bisect.bisect_right(starts, e.beat + 1e-6) - 1
        if k >= 0:
            c = m.chords[k]
            if e.beat < c.beat + c.dur_beats - 1e-6:
                ivals = kn.CHORD_INTERVALS[c.quality]
                d = (e.midi - c.root_pc) % 12
                third = ivals[1] if len(ivals) > 1 else None
                fifth = ivals[2] if len(ivals) > 2 else None
                rel["root" if d == 0 else "third" if d == third else "fifth" if d == fifth else "other"] += 1
        if i + 1 < len(m.bass):
            nxt = m.bass[i + 1]
            b, nb = int((e.beat + 1e-6) // bar), int((nxt.beat + 1e-6) // bar)
            if nb == b + 1 and abs(nxt.beat - nb * bar) < 1e-6 and abs(nxt.midi - e.midi) == 1:
                approach += 1
    last = m.bass[-1].beat
    return {"bass_rel": rel, "bass_approach": approach, "bars": int(last // bar) + 1}


def row_of(path: str) -> Optional[Dict[str, Any]]:
    try:
        m = mt.from_json(pathlib.Path(path).read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001 - битый JSON библиотеки считает проверка пака, здесь он просто не в корпусе
        return None
    if m.key.mode not in MODES or not m.chords:
        return None
    seq: List[Optional[int]] = []
    prev = None
    for c in m.chords:
        sig = (c.root_pc, c.quality)
        if sig != prev:
            seq.append(c.degree)
        prev = sig
    row: Dict[str, Any] = {"file": pathlib.Path(path).name, "mode": m.key.mode, "degree_seq": seq,
                           "bass_rel": {}, "bass_approach": 0, "bars": 0}
    row.update(bass_relations(m))
    return row


def collect(lib: pathlib.Path, jobs: int, limit: int = 0) -> List[Dict[str, Any]]:
    files = sorted(p for p in lib.glob("*.json") if p.name != "LIBRARY.json")[:limit or None]
    if jobs <= 1:
        rows = [row_of(str(p)) for p in files]
    else:
        with multiprocessing.Pool(jobs) as pool:
            rows = pool.map(row_of, [str(p) for p in files], chunksize=64)
    return [r for r in rows if r]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("lib", help="каталог библиотеки (JSON материалов)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args(argv)
    rows = collect(pathlib.Path(args.lib).expanduser(), max(1, args.jobs), args.limit)
    pathlib.Path(args.out).write_text(json.dumps({"rows": rows}, ensure_ascii=False) + "\n", encoding="utf-8")
    with_bass = sum(1 for r in rows if r["bass_rel"])
    print(f"материалов в корпусе таблиц {len(rows)} (major {sum(r['mode'] == 'major' for r in rows)}, "
          f"minor {sum(r['mode'] == 'minor' for r in rows)}), с басовым голосом {with_bass} → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
