#!/usr/bin/env python3
"""Проба партитур (MusicXML/MXL) как материала аранжировщика v2 — исследование ADR-0154.

Что считает по каждой партитуре (music21) и сводит по выборке:

* партии, размер, темп, тональность (ключевые знаки, Крумхансл music21, наш ``tonality.detect_key`` по мелодии);
* мелодия — skyline (верхняя нота на каждом онсете всех партий) и доля онсетов, где она из верхней партии;
* гармония — аккорд полутакта/такта по шаблонам 24 трезвучий (как ``core/harmonize._best_chord``: вес = длительность
  нот), ступень в тональности; гармонический ритм; биграммы переходов ступеней по корпусу;
* бас — низшая нота онсета: тон аккорда (прима/терция/квинта/чужой), нот на такт, доля на долях, подходы полутоном;
* фактура аккомпанемента (всё, что не skyline) по тактам: ``sustained``/``block``/``broken``/``mixed``;
* структура — отпечатки тактов мелодии (повторы, доля уникальных), развитие соседних 4-тактовых фраз
  (``repeat``/``sequence``/``rhythm``/``new``), текстовые метки частей;
* наш аранжировщик на этом материале: ``hook.from_rtttl`` на первых 8 тактах skyline (RTTTL из партитуры),
  ``harmony.fit_progression`` (стиль club) против аккордов оригинала по 2-тактовым окнам.

Разбор партитуры живёт в ``scripts/music/score_import.py`` (ADR-0154 PR-2); здесь — только исследовательская
статистика и сравнение с нашим аранжировщиком.

Запуск (нужен ``pip install music21``; ``PYTHONPATH`` на ``src/rob_box_music``):

    python scripts/music/research/score_material_probe.py <каталог|файл>... [--out stats.json] [--limit N]

Партитуры в git не кладутся (чужие аранжировки); сюда — только числа.
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import sys
import time
import warnings
from typing import Dict, List, Optional, Tuple

warnings.filterwarnings("ignore")

from music21 import converter  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
# Разбор партитуры (skyline, аккорды, бас, фактуры, фразы) — одна реализация, в импортёре ADR-0154 PR-2.
from score_import import (DEGREE_NAMES, MAJOR, MINOR, STEP, TRIADS, bar_fingerprints, bar_map, bar_of,  # noqa: E402,F401
                          bass_line, best_triad, chords_per_half_bar, degree_name, key_signature, load_notes,
                          meters_of, phrase_relation, skyline, tempos_of, texture_of_bar, text_marks,
                          window_weights)

try:
    from rob_box_music import knowledge as kn
    from rob_box_music.arrange import harmony, hook as hooks
    from rob_box_music.model import Key, PitchEvent
    from rob_box_music.tonality import detect_key, key_fit
    HAVE_ARRANGER = True
except ImportError:  # pragma: no cover - без PYTHONPATH сравнение с аранжировщиком пропускается
    HAVE_ARRANGER = False


# ------------------------------------------------------------------ наш аранжировщик

_RTTTL_DUR = ((16, "1"), (12, "2."), (8, "2"), (6, "4."), (4, "4"), (3, "8."), (2, "8"), (1, "16"))
_NAMES = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")


def to_rtttl(mel, bars: int, offs, lens, bpm: int) -> str:
    """Первые ``bars`` тактов skyline → RTTTL (сетка 16-х, паузы ``p``, длинные ноты — несколько токенов)."""
    end = offs[0] + sum(lens[:bars])
    tokens = []
    t = offs[0]
    for off, dur, midi, _pi in mel:
        if off >= end:
            break
        gap = int(round((off - t) / STEP))
        for n in _split(gap):
            tokens.append(f"{n}p")
        steps = int(round(min(dur, end - off) / STEP))
        for i, n in enumerate(_split(max(1, steps))):
            tokens.append(f"{n}{_NAMES[midi % 12]}{midi // 12 - 1}")
        t = off + steps * STEP
    return f"score:d=4,o=5,b={bpm}:" + ",".join(tokens)


def _split(steps: int) -> List[str]:
    out = []
    while steps > 0:
        for n, code in _RTTTL_DUR:
            if n <= steps:
                out.append(code)
                steps -= n
                break
    return out


def our_hook(mel, offs, lens, root: int, mode: str) -> Dict[str, object]:
    rtttl = to_rtttl(mel, 8, offs, lens, 130)
    try:
        hk, key = hooks.from_rtttl(rtttl, "score", 130, root, mode)
    except hooks.HookError as exc:
        return {"ok": False, "reason": str(exc)[:80]}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "reason": f"{type(exc).__name__}: {exc}"[:80]}
    return {"ok": True, "bars": hk.bars, "key_fit": hk.key_fit, "notes": len(hk.notes),
            "key": f"{kn.ROOTS[key.root]} {key.mode}"}


def our_progression_match(mel, chords, offs, lens, tonic: int, mode: str) -> Dict[str, object]:
    """``fit_progression`` (club) по 8-тактовым окнам против ступеней оригинала (аккорд 2-тактового окна —
    по длительности); доля совпавших из 4 слотов; базовая линия — «везде I»."""
    style = kn.STYLES["club"]
    key = Key(tonic, mode if mode in kn.SCALES else "minor")
    scale = kn.SCALES[key.mode]
    hits = total = base_hits = oracle_hits = 0
    windows = 0
    dump: List[Dict[str, object]] = []
    by_bar: Dict[int, List[str]] = collections.defaultdict(list)
    for bar, _h, tri, _name in chords:
        by_bar[bar].append(tri)
    for start in range(0, len(offs) - 7, 8):
        win_end = offs[start] + sum(lens[start:start + 8])
        notes = [PitchEvent(midi, (off - offs[start]) * 4.0 / lens[start], dur * 4.0 / lens[start], 2)
                 for off, dur, midi, _pi in mel if offs[start] <= off < win_end]
        if len(notes) < 6:
            continue
        try:
            degrees = harmony.fit_progression(style, key, notes, 8.0, random.Random(start))
        except Exception:  # noqa: BLE001
            continue
        windows += 1
        orig_degrees: List[Optional[int]] = []
        for slot in range(4):
            tris = by_bar.get(start + 2 * slot, []) + by_bar.get(start + 2 * slot + 1, [])
            if not tris:
                orig_degrees.append(None)
                continue
            orig = collections.Counter(tris).most_common(1)[0][0]
            orig_deg = (orig[0] - tonic) % 12
            total += 1
            deg = scale.index(orig_deg) if orig_deg in scale else None
            orig_degrees.append(deg)
            if deg is not None and deg == degrees[slot]:
                hits += 1
            if orig_deg == 0:
                base_hits += 1
        # лучший из пула шаблонов стиля — потолок шаблонного подхода на этом окне
        oracle_hits += max(sum(1 for d, o in zip(p, orig_degrees) if o is not None and d == o)
                           for p in style.progressions)
        dump.append({"melody": [(e.beat, e.dur_beats, e.midi) for e in notes], "orig": orig_degrees,
                     "template": list(degrees)})
    return {"windows": windows, "slots": total, "hits": hits, "tonic_baseline": base_hits, "pool_oracle": oracle_hits,
            "dump": dump}


# ------------------------------------------------------------------ одна партитура

def analyze(path: pathlib.Path) -> Dict[str, object]:
    t0 = time.time()
    score = converter.parse(str(path))
    if not score.parts:
        return {"file": path.name, "error": "no parts"}
    notes = load_notes(score)
    offs, lens = bar_map(score)
    if not notes or not offs:
        return {"file": path.name, "error": "empty"}
    mel = skyline(notes)
    top_share = sum(1 for n in mel if n[3] == 0) / len(mel)

    k21 = score.analyze("key")
    tonic = k21.tonic.pitchClass
    mode = "major" if k21.mode == "major" else "minor"
    ours = {"root": None, "mode": None, "agree": None}
    if HAVE_ARRANGER:
        r, m = detect_key([n[2] for n in mel], [n[1] for n in mel])
        ours = {"root": r, "mode": m, "agree": kn.ROOTS.index(r) == tonic and (m == "major") == (mode == "major")}
        fit = key_fit([(n[2], n[1]) for n in mel], kn.ROOTS[tonic], mode)
    else:
        fit = None

    chords = chords_per_half_bar(notes, offs, lens, tonic)
    degrees = [name for _b, _h, _t, name in chords]
    changes = sum(1 for i in range(1, len(chords)) if chords[i][2] != chords[i - 1][2])
    bars_with_two = sum(1 for i in range(0, len(chords) - 1, 2)
                        if chords[i][0] == chords[i + 1][0] and chords[i][2] != chords[i + 1][2])
    by_bar_tri: Dict[int, set] = collections.defaultdict(set)
    for b, _h, tri, _n in chords:
        by_bar_tri[b].add(tri)
    two_bar_single = sum(1 for s in range(0, len(offs) - 1, 2)
                         if len(by_bar_tri.get(s, set()) | by_bar_tri.get(s + 1, set())) == 1)
    bigrams = collections.Counter()
    merged = [name for i, name in enumerate(degrees) if i == 0 or chords[i][2] != chords[i - 1][2]]
    for a, b in zip(merged, merged[1:]):
        bigrams[f"{a}>{b}"] += 1

    # бас
    bl = bass_line(notes)
    chord_at = {(b, h): tri for b, h, tri, _n in chords}
    rel = collections.Counter()
    on_beat = 0
    approach = 0
    jumps = 0
    for i, (off, dur, midi) in enumerate(bl):
        b = bar_of(offs, off)
        h = 0 if off - offs[b] < lens[b] / 2 else 1
        tri = chord_at.get((b, h))
        if tri:
            pc = midi % 12
            rel["root" if pc == tri[0] else "third" if pc == tri[1] else "fifth" if pc == tri[2] else "other"] += 1
        beat = (off - offs[b]) % 1.0
        on_beat += beat < 1e-6
        if i + 1 < len(bl):
            nxt = bl[i + 1]
            nb = bar_of(offs, nxt[0])
            if nb == b + 1 and abs(nxt[0] - offs[nb]) < 1e-6 and abs(nxt[2] - midi) == 1:
                approach += 1
            jumps += abs(nxt[2] - midi) >= 12
    bass_notes_per_bar = len(bl) / len(offs)

    # фактура аккомпанемента
    mel_set = {(off, midi) for off, _d, midi, _pi in mel}
    acc_by_bar: Dict[int, List[Tuple[float, float, int]]] = collections.defaultdict(list)
    for off, dur, midi, _pi in notes:
        q = round(off / STEP) * STEP
        if (q, midi) in mel_set:
            continue
        acc_by_bar[bar_of(offs, off)].append((off, dur, midi))
    textures = collections.Counter(texture_of_bar(acc_by_bar.get(i, [])) for i in range(len(offs)))

    # структура
    fps = bar_fingerprints(mel, offs, lens)
    nonempty = [f for f in fps if f]
    unique = len(set(nonempty))
    seen: Dict[tuple, int] = {}
    repeat_gaps = []
    for i, f in enumerate(fps):
        if not f:
            continue
        if f in seen:
            repeat_gaps.append(i - seen[f])
        else:
            seen[f] = i
    relations = collections.Counter()
    for s in range(0, len(fps) - 7, 4):
        relations[phrase_relation(fps[s:s + 4], fps[s + 4:s + 8])] += 1
    mel_notes_per_bar = len(mel) / len(offs)

    out = {
        "file": path.name, "parts": len(score.parts), "bars": len(offs), "meters": meters_of(score),
        "tempos": tempos_of(score), "key_sig": key_signature(score),
        "key21": f"{k21.tonic.name} {k21.mode}", "key21_corr": round(float(k21.correlationCoefficient), 3),
        "our_key": ours, "melody_key_fit": fit,
        "skyline_top_part_share": round(top_share, 3), "melody_notes_per_bar": round(mel_notes_per_bar, 2),
        "chord_changes_per_bar": round(changes / len(offs), 2),
        "bars_with_two_chords": round(bars_with_two / len(offs), 2),
        "two_bar_single_chord_share": round(two_bar_single / max(1, len(offs) // 2), 2),
        "degrees_top": collections.Counter(merged).most_common(8), "bigrams": bigrams.most_common(12),
        "bass_rel": dict(rel), "bass_notes_per_bar": round(bass_notes_per_bar, 2),
        "bass_on_beat_share": round(on_beat / max(1, len(bl)), 3), "bass_approach": approach,
        "bass_octave_jumps": round(jumps / max(1, len(bl)), 3),
        "textures": dict(textures), "unique_bar_share": round(unique / max(1, len(nonempty)), 3),
        "repeat_gaps_top": collections.Counter(repeat_gaps).most_common(5), "phrase_relations": dict(relations),
        "text_marks": text_marks(score, offs)[:20], "seconds": round(time.time() - t0, 1),
    }
    scale = MAJOR if mode == "major" else MINOR
    out["mode"] = mode
    # последовательность ступеней (слитые окна): индекс в ладу 0..6, None — недиатонический аккорд
    out["degree_seq"] = [scale.index((tri[0] - tonic) % 12) if (tri[0] - tonic) % 12 in scale else None
                         for i, (_b, _h, tri, _n) in enumerate(chords) if i == 0 or chords[i][2] != chords[i - 1][2]]
    if HAVE_ARRANGER:
        out["our_hook"] = our_hook(mel, offs, lens, tonic, mode)
        out["our_progression"] = our_progression_match(mel, chords, offs, lens, tonic, mode)
    return out


# ------------------------------------------------------------------ сводка

def summarize(rows: List[Dict[str, object]]) -> Dict[str, object]:
    ok = [r for r in rows if "error" not in r]
    bigrams = collections.Counter()
    degrees = collections.Counter()
    textures = collections.Counter()
    relations = collections.Counter()
    bass_rel = collections.Counter()
    gaps = collections.Counter()
    for r in ok:
        for k, v in r["bigrams"]:
            bigrams[k] += v
        for k, v in r["degrees_top"]:
            degrees[k] += v
        for k, v in r["textures"].items():
            textures[k] += v
        for k, v in r["phrase_relations"].items():
            relations[k] += v
        for k, v in r["bass_rel"].items():
            bass_rel[k] += v
        for k, v in r["repeat_gaps_top"]:
            gaps[k] += v

    def med(key, sub=None):
        vals = sorted((r[key] if sub is None else r[key][sub]) for r in ok if r.get(key) is not None)
        return vals[len(vals) // 2] if vals else None

    summary = {
        "scores": len(rows), "parsed": len(ok),
        "meters": collections.Counter(m for r in ok for m in r["meters"]).most_common(),
        "key_agree_ours_vs_music21": sum(1 for r in ok if r["our_key"]["agree"]),
        "melody_key_fit_median": med("melody_key_fit"),
        "melody_key_fit_ge_0.6": sum(1 for r in ok if (r["melody_key_fit"] or 0) >= 0.6),
        "skyline_top_part_share_median": med("skyline_top_part_share"),
        "melody_notes_per_bar_median": med("melody_notes_per_bar"),
        "chord_changes_per_bar_median": med("chord_changes_per_bar"),
        "bars_with_two_chords_median": med("bars_with_two_chords"),
        "two_bar_single_chord_share_median": med("two_bar_single_chord_share"),
        "degrees_total": degrees.most_common(12), "bigrams_total": bigrams.most_common(20),
        "bass_rel_total": dict(bass_rel), "bass_notes_per_bar_median": med("bass_notes_per_bar"),
        "bass_on_beat_share_median": med("bass_on_beat_share"),
        "bass_approach_total": sum(r["bass_approach"] for r in ok),
        "textures_total": dict(textures), "unique_bar_share_median": med("unique_bar_share"),
        "repeat_gaps_total": gaps.most_common(8), "phrase_relations_total": dict(relations),
        "scores_with_text_marks": sum(1 for r in ok if r["text_marks"]),
    }
    if ok and "our_hook" in ok[0]:
        hooks_ok = [r for r in ok if r["our_hook"]["ok"]]
        summary["our_hook_ok"] = len(hooks_ok)
        summary["our_hook_reasons"] = collections.Counter(
            r["our_hook"]["reason"].split(":")[0] for r in ok if not r["our_hook"]["ok"]).most_common()
        summary["our_hook_key_fit_median"] = (sorted(r["our_hook"]["key_fit"] for r in hooks_ok)[len(hooks_ok) // 2]
                                              if hooks_ok else None)
        slots = sum(r["our_progression"]["slots"] for r in ok)
        summary["our_progression"] = {
            "windows": sum(r["our_progression"]["windows"] for r in ok), "slots": slots,
            "hit_share": round(sum(r["our_progression"]["hits"] for r in ok) / max(1, slots), 3),
            "tonic_baseline_share": round(sum(r["our_progression"]["tonic_baseline"] for r in ok) / max(1, slots), 3),
            "pool_oracle_share": round(sum(r["our_progression"]["pool_oracle"] for r in ok) / max(1, slots), 3),
        }
    return summary


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")  # Windows: cp1252 не печатает русские причины отказа хука
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--out", default=None)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()
    files: List[pathlib.Path] = []
    for p in args.paths:
        pp = pathlib.Path(p)
        if pp.is_dir():
            files += sorted(x for x in pp.iterdir() if x.suffix.lower() in (".mxl", ".xml", ".musicxml"))
        else:
            files.append(pp)
    if args.limit:
        files = files[:args.limit]
    rows = []
    for f in files:
        try:
            row = analyze(f)
        except Exception as exc:  # noqa: BLE001
            row = {"file": f.name, "error": f"{type(exc).__name__}: {exc}"[:120]}
        rows.append(row)
        brief = {k: row.get(k) for k in ("file", "error", "bars", "meters", "key21", "our_key",
                                         "skyline_top_part_share", "our_hook", "seconds")}
        if row.get("our_progression"):
            brief["our_progression"] = {k: v for k, v in row["our_progression"].items() if k != "dump"}
        print(json.dumps(brief, ensure_ascii=False), flush=True)
    summary = summarize(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps({"rows": rows, "summary": summary}, ensure_ascii=False, indent=1),
                                          encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
