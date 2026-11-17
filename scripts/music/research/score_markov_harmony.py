#!/usr/bin/env python3
"""Гармония из выученной таблицы переходов против шаблонов стиля — опыт для ADR-0154 §3.3.

Вход — ``stats.json`` из ``score_material_probe.py`` (поля ``degree_seq``, ``mode``, ``our_progression.dump``).
На каждую партитуру таблица переходов ступеней учится на ОСТАЛЬНЫХ партитурах того же лада (leave-one-out,
сглаживание +1), а аккорды 8-тактового окна (4 слота по 2 такта) выбираются Витерби по 7 диатоническим ступеням:
``w × доля длительности мелодии слота в трезвучии + log P(переход)``. Сравнивается доля совпавших слотов с
оригиналом: шаблоны ``harmony.fit_progression`` (из json), Витерби при нескольких ``w`` (все значения печатаются —
подбор на той же выборке честно виден), потолок пула шаблонов, «везде тоника».

    python scripts/music/research/score_markov_harmony.py stats.json
    python scripts/music/research/score_markov_harmony.py stats.json \
        --write-table src/rob_box_music/rob_box_music/data/progression_transitions.json

``--write-table`` — таблица ``knowledge.PROGRESSION_TRANSITIONS`` по ВСЕМ партитурам (ADR-0154 §3.4, PR-3).
Оговорка: эмиссия здесь (:func:`emissions`) считает трезвучия от C, а не от тоники партитуры — вклад мелодии в
этом опыте занижен для не-C; Витерби кода (``arrange.harmony.viterbi``) берёт тонику, его замер —
``score_harmony_m3.py``.
"""

from __future__ import annotations

import argparse
import collections
import datetime
import hashlib
import json
import math
import pathlib
import sys
from typing import Dict, List, Sequence, Tuple

MAJOR = (0, 2, 4, 5, 7, 9, 11)
MINOR = (0, 2, 3, 5, 7, 8, 10)
WEIGHTS = (0.0, 0.5, 1.0, 2.0, 4.0)


def triad(scale: Sequence[int], degree: int) -> set:
    return {scale[(degree + 2 * k) % 7] % 12 for k in range(3)}


def corpus_counts(rows: List[Dict], mode: str, skip: str = "") -> Tuple[List[float], List[List[float]]]:
    """Счёт первых ступеней и переходов лада ``mode`` со сглаживанием +1; ``skip`` — партитура вне обучения."""
    counts = [[1.0] * 7 for _ in range(7)]
    starts = [1.0] * 7
    for r in rows:
        if r["file"] == skip or r["mode"] != mode:
            continue
        seq = r["degree_seq"]
        if seq and seq[0] is not None:
            starts[seq[0]] += 1
        for a, b in zip(seq, seq[1:]):
            if a is not None and b is not None:
                counts[a][b] += 1
    return starts, counts


def transitions(rows: List[Dict], skip: str, mode: str) -> List[List[float]]:
    _starts, counts = corpus_counts(rows, mode, skip)
    return [[math.log(c / sum(row)) for c in row] for row in counts]


def emissions(window: Dict, scale: Sequence[int]) -> List[List[float]]:
    """[слот][ступень] — доля длительности нот слота в трезвучии ступени."""
    out = []
    for slot in range(4):
        notes = [(b, d, m) for b, d, m in window["melody"] if slot * 8 <= b < (slot + 1) * 8]
        total = sum(d for _b, d, _m in notes) or 1.0
        out.append([sum(d for _b, d, m in notes if m % 12 in triad(scale, deg)) / total for deg in range(7)])
    return out


def viterbi(em: List[List[float]], trans: List[List[float]], w: float, prior: Sequence[float]) -> List[int]:
    best = [[(w * em[0][d] + prior[d], -1) for d in range(7)]]
    for slot in range(1, 4):
        row = []
        for d in range(7):
            cand = max((best[-1][p][0] + trans[p][d], p) for p in range(7))
            row.append((cand[0] + w * em[slot][d], cand[1]))
        best.append(row)
    path = [max(range(7), key=lambda d: best[3][d][0])]
    for slot in range(3, 0, -1):
        path.append(best[slot][path[-1]][1])
    return path[::-1]


def corpus_table(rows: List[Dict], mode: str, skip: str = "") -> Dict[str, List]:
    """Таблица лада ``mode`` для ``knowledge.PROGRESSION_TRANSITIONS``: вероятности (не логарифмы) с тем же
    сглаживанием +1, что в leave-one-out опыте; ``skip`` — партитура, исключённая из обучения."""
    starts, counts = corpus_counts(rows, mode, skip)
    return {"start": [round(s / sum(starts), 4) for s in starts],
            "next": [[round(c / sum(row), 4) for c in row] for row in counts]}


def write_table(rows: List[Dict], stats_path: str, out: str, label: str = "") -> None:
    """Выученная таблица — данные пакета (ADR-0154 §3.4). Обучение детерминировано (подсчёт, без ГСЧ); провенанс —
    число партитур, sha256 входа (файлы + лады + последовательности ступеней), дата и команда."""
    corpus = sorted((r["file"], r["mode"], r["degree_seq"]) for r in rows)
    digest = hashlib.sha256(json.dumps(corpus, ensure_ascii=False).encode()).hexdigest()
    table = {
        "provenance": {
            "corpus": label or "musetrainer/library (PD) + 2 Interstellar (локально, не в git); "
                                "docs/music/research_score_material.md",
            "scores": len(rows), "modes": dict(collections.Counter(r["mode"] for r in rows)),
            "corpus_sha256": digest, "stats": pathlib.Path(stats_path).name, "date": datetime.date.today().isoformat(),
            "script": "scripts/music/research/score_markov_harmony.py --write-table", "smoothing": "+1",
            "degrees": "индекс ступени 0..6 в ладу; аккорд полутакта — score_material_probe.chords_per_half_bar"},
        "tables": {mode: corpus_table(rows, mode) for mode in ("major", "minor")},
    }
    pathlib.Path(out).write_text(json.dumps(table, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(f"таблица → {out}: партитур {len(rows)}, sha256 корпуса {digest[:12]}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stats")
    ap.add_argument("--write-table", default="", help="записать таблицу переходов (JSON knowledge) по всему корпусу")
    ap.add_argument("--corpus-label", default="", help="описание корпуса для провенанса таблицы")
    args = ap.parse_args()
    data = json.load(open(args.stats, encoding="utf-8"))
    all_rows = [r for r in data["rows"] if "error" not in r and r.get("degree_seq")]
    if args.write_table:  # таблица — по всему корпусу; ``score_corpus_stats.py`` не считает ``our_progression``
        write_table(all_rows, args.stats, args.write_table, args.corpus_label)
    rows = [r for r in all_rows if r.get("our_progression")]
    score = collections.Counter()
    slots = 0
    for r in rows:
        scale = MAJOR if r["mode"] == "major" else MINOR
        trans = transitions(rows, r["file"], r["mode"])
        # априорная ступень первого слота: как часто ступень открывает последовательность у остальных
        starts, _counts = corpus_counts(rows, r["mode"], r["file"])
        prior = [math.log(s / sum(starts)) for s in starts]
        for win in r["our_progression"]["dump"]:
            em = emissions(win, scale)
            orig = win["orig"]
            known = [i for i, o in enumerate(orig) if o is not None]
            slots += len(known)
            score["template"] += sum(1 for i in known if win["template"][i] == orig[i])
            score["tonic"] += sum(1 for i in known if orig[i] == 0)
            score["melody_only_argmax"] += sum(1 for i in known if max(range(7), key=lambda d: em[i][d]) == orig[i])
            for w in WEIGHTS:
                path = viterbi(em, trans, w, prior)
                score[f"viterbi_w{w}"] += sum(1 for i in known if path[i] == orig[i])
    print(f"партитур {len(rows)}, слотов с диатоническим аккордом оригинала {slots}")
    for k, v in sorted(score.items(), key=lambda kv: -kv[1]):
        print(f"{k:22s} {v:5d}  {v / max(1, slots):.3f}")
    # сводная таблица переходов по корпусу (мажор/минор) — кандидат в knowledge
    for mode in ("major", "minor"):
        counts = collections.Counter()
        for r in all_rows:
            if r["mode"] != mode:
                continue
            seq = r["degree_seq"]
            for a, b in zip(seq, seq[1:]):
                if a is not None and b is not None and a != b:
                    counts[(a, b)] += 1
        total = sum(counts.values())
        print(f"\n{mode}: переходов {total}; топ-12 (ступени 0..6):")
        for (a, b), n in counts.most_common(12):
            print(f"  {a}->{b}: {n} ({n / max(1, total):.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
