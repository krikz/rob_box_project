#!/usr/bin/env python3
"""Замер M3 ADR-0154 (PR-3): гармония трека из материала партитуры против аккордов оригинала.

На каждую партитуру (music21, разбор — функции ``score_material_probe.py``) строится ``ScoreMaterial``: skyline,
аккорды полутактов (слитые) со ступенью по приме в ладу music21, такты после затакта. Дальше по 8-тактовым окнам
(слот — ``--slot-bars`` тактов):

* ``material`` — ``harmony.from_material`` (код аранжировщика) против ступени оригинала слота (аккорд большинства
  полутактов, ступень по приме — как ``our_progression_match`` пробы); раскладка потерь: адаптация недиатоники,
  фраза на одном аккорде (Витерби), каденция;
* ``viterbi_loo`` — ``harmony.viterbi`` без материала (только мелодия окна) по таблице, выученной на ОСТАЛЬНЫХ
  партитурах (leave-one-out, ``score_markov_harmony.corpus_counts``); ``template`` — ``harmony.fit_progression``;
  ``tonic`` — «везде I»;
* ``compose`` — сквозной путь: ``compose`` с ``TrackPlan.material`` (хук и гармония из материала), ступени дропа
  против оригинала первых 8 тактов (только множитель темпа 1 и хук на 8 тактов — иначе слоты не сравнимы).

    PYTHONPATH=src/rob_box_music python scripts/music/research/score_harmony_m3.py <каталог|файл>... [--slot-bars 1]

Партитуры в git не кладутся; печатаются только числа.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import pathlib
import random
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import score_markov_harmony as smh  # noqa: E402
import score_material_probe as probe  # noqa: E402
from music21 import converter  # noqa: E402

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music import material as mt  # noqa: E402
from rob_box_music.arrange import compose as cp, harmony, hook as hooks  # noqa: E402
from rob_box_music.model import Key, PitchEvent  # noqa: E402
from rob_box_music.set_plan import seeded_plan  # noqa: E402
from rob_box_music.theme import ThemeProfile  # noqa: E402


MIN_RUN_BARS = 8


def meter_of(bar: float):
    """Размер по длине такта в четвертях: 3.0 → (3, 4), 1.5 → (3, 8)."""
    num, den = bar, 4
    while abs(num - round(num)) > 1e-6:
        num, den = num * 2, den * 2
    return int(round(num)), den


def runs(offs, nominal):
    """Куски пьесы ``[a, b)`` из ≥ ``MIN_RUN_BARS`` ровных тактов одной длины ≤ 4/4 (ADR-0154 §3.1: смена размера
    режет материал на части; затакт и вольты — вне кусков). Длина такта — разность начал: ``bar_map`` даёт
    номинальную (``barDuration``), затакт там выглядит полным."""
    lens = [b - a for a, b in zip(offs, offs[1:])] + nominal[-1:]
    out, a = [], 0
    for i in range(1, len(lens) + 1):
        if i == len(lens) or abs(lens[i] - lens[a]) > 1e-6 or abs(lens[i] - nominal[i]) > 1e-6:
            if i - a >= MIN_RUN_BARS and abs(lens[a] - nominal[a]) < 1e-6 and lens[a] <= 4.0:
                out.append((a, i))
            a = i
    return lens, out


def segment(path, key, notes, offs, lens, chords, a, b):
    """Материал куска ``[a, b)``: аккорды полутактов (слитые), ступень по приме; оригинал по тактам куска."""
    scale = kn.SCALES[key.mode]
    base = offs[a]
    spans, by_bar = [], collections.defaultdict(list)
    for bar, half, tri, _name in chords:
        if not a <= bar < b:
            continue
        rel = (tri[0] - key.root) % 12
        degree = scale.index(rel) if rel in scale else None
        by_bar[bar - a].append((tri, degree))
        quality = "maj" if (tri[1] - tri[0]) % 12 == 4 else "min"
        beat = offs[bar] + half * lens[bar] / 2 - base
        if spans and (spans[-1].root_pc, spans[-1].quality) == (tri[0], quality) and \
                abs(spans[-1].beat + spans[-1].dur_beats - beat) < 1e-6:
            spans[-1] = replace(spans[-1], dur_beats=spans[-1].dur_beats + lens[bar] / 2)
        else:
            spans.append(mt.ChordSpan(beat, lens[bar] / 2, tri[0], quality, degree))
    end = offs[b - 1] + lens[b - 1]
    melody = tuple(PitchEvent(m, round(off - base, 6), dur, 2) for off, dur, m, _p in probe.skyline(notes)
                   if base <= off < end)
    sha = hashlib.sha256(f"{path.name}:{a}".encode()).hexdigest()[:8]
    material = mt.ScoreMaterial(f"local:{sha}", f"{path.stem} [{a}:{b}]", "", "file", "PD", meter_of(lens[a]), None,
                                key, melody, tuple(spans))
    mt.validate_material(material)
    return material, b - a, by_bar


def build(path: pathlib.Path):
    """Куски-материалы партитуры и последовательность ступеней всей пьесы (для таблицы LOO) или причина отказа."""
    score = converter.parse(str(path))
    notes = probe.load_notes(score)
    offs, nominal = probe.bar_map(score)
    if not notes or not offs:
        return "пусто"
    lens, parts = runs(offs, nominal)
    if not parts:
        return "нет ровного куска ≥ 8 тактов ≤ 4/4"
    k21 = score.analyze("key")
    key = Key(k21.tonic.pitchClass, "major" if k21.mode == "major" else "minor")
    chords = probe.chords_per_half_bar(notes, offs, lens, key.root)
    scale = kn.SCALES[key.mode]
    seq = [(tri, scale.index((tri[0] - key.root) % 12) if (tri[0] - key.root) % 12 in scale else None)
           for _b, _h, tri, _n in chords]
    merged = [d for i, (tri, d) in enumerate(seq) if i == 0 or tri != seq[i - 1][0]]  # как degree_seq пробы
    return [segment(path, key, notes, offs, lens, chords, a, b) for a, b in parts], merged


def orig_slots(by_bar, start: int, slot_bars: int, slots: int):
    """Ступень оригинала слота: аккорд большинства полутактов (ничья — раньше), ступень по приме; None — нет."""
    out = []
    for s in range(slots):
        tris = [x for b in range(start + s * slot_bars, start + (s + 1) * slot_bars) for x in by_bar.get(b, [])]
        out.append(collections.Counter(tris).most_common(1)[0][0][1] if tris else None)
    return out


def window_notes(material, start: int):
    """Мелодия окна в долях трека (4/4 клуба, от первой ноты окна — как хук)."""
    bar = mt.bar_beats(material.meter)
    inside = [e for e in material.melody if start * bar <= e.beat < (start + 8) * bar]
    if not inside:
        return []
    zero = mt.club_beat(material.meter, inside[0].beat - start * bar)
    return [PitchEvent(e.midi, mt.club_beat(material.meter, e.beat - start * bar) - zero, e.dur_beats, 2)
            for e in inside]


def loo_table(rows, skip: str, mode: str):
    starts, counts = smh.corpus_counts(rows, mode, skip)
    return {"start": tuple(s / sum(starts) for s in starts),
            "next": tuple(tuple(c / sum(r) for c in r) for r in counts)}


def measure_windows(name, material, bars, by_bar, rows, slot_bars, score):
    slots, beats = 8 // slot_bars, slot_bars * 4.0
    table = loo_table(rows, name, material.key.mode)
    for start in range(0, bars - 7, 8):
        notes = window_notes(material, start)
        orig = orig_slots(by_bar, start, slot_bars, slots)
        if len(notes) < 6:
            continue
        phrase = mt.Phrase(start, 8, "new", 1)
        got = harmony.from_material(material, phrase, material.key, notes, beats, slots)
        raw = [None if c is None else harmony._adapt(c, material.key)
               for c in harmony.material_slots(material, phrase, beats, slots)]
        vit = harmony.viterbi(material.key, notes, beats, slots, (), table)
        tpl = harmony.fit_progression(kn.STYLES["club"], material.key, notes, beats, random.Random(start)) \
            if slots == 4 else None
        for i, o in enumerate(orig):
            score["slots_all"] += 1
            if o is None:
                continue
            score["slots"] += 1
            score["material"] += got[i] == o
            score["material_raw_adapted"] += raw[i] == o
            score["viterbi_loo"] += vit[i] == o
            score["tonic"] += o == 0
            if tpl is not None:
                score["template"] += tpl[i] == o


def measure_compose(material, by_bar, score):
    profile = ThemeProfile("", kn.DEFAULT_STYLE, 132, material.key.root, material.key.mode, (), None)
    plan = seeded_plan(profile, 1)
    plan = replace(plan, tracks=(replace(plan.track(1), material=material.material_id),))
    track = cp.compose(plan, 1, materials={material.material_id: material})
    score["compose_tracks"] += 1
    if track.hook is None or track.hook.source != material.material_id:
        try:  # причина отказа — тем же путём, что в compose (лог I12 здесь не читаем)
            hooks.from_material(material, plan.bpm, plan.root(1), profile.mode, cp.hook_register(plan.table))
            reason = "пэд/лад гармонии"
        except ValueError as exc:
            reason = str(exc).split(":")[0][:40]
        score[f"compose_refused: {reason}"] += 1
        return
    score["compose_from_material"] += 1
    if track.hook.bars != 8:
        return
    drop = [c.degree for c in track.harmony.progression["drop"]]
    for o, d in zip(orig_slots(by_bar, 0, 2, 4), drop):
        if o is not None:
            score["compose_slots"] += 1
            score["compose_match"] += d == o


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--slot-bars", type=int, default=2, choices=(1, 2))
    args = ap.parse_args()
    files = []
    for p in map(pathlib.Path, args.paths):
        scores = (x for x in p.iterdir() if x.suffix.lower() in (".mxl", ".xml", ".musicxml")) if p.is_dir() else [p]
        files += sorted(scores)
    built, skipped = {}, collections.Counter()
    for f in files:
        try:
            out = build(f)
        except Exception as exc:  # noqa: BLE001 — отказ парсера/валидатора идёт в отчёт
            out = f"{type(exc).__name__}: {getattr(exc, 'reason', '')[:25]}"
        if isinstance(out, str):
            skipped[out.split(":")[0][:40]] += 1
        else:
            built[f.name] = out
    rows = [{"file": n, "mode": parts[0][0].key.mode, "degree_seq": seq} for n, (parts, seq) in built.items()]
    score = collections.Counter()
    for name, (parts, _seq) in built.items():
        for material, bars, by_bar in parts:
            measure_windows(name, material, bars, by_bar, rows, args.slot_bars, score)
        if args.slot_bars == 2:
            measure_compose(parts[0][0], parts[0][2], score)  # трек — по первому куску пьесы
    n_parts = sum(len(parts) for parts, _seq in built.values())
    print(f"файлов {len(files)}, партитур с материалом {len(built)} (кусков {n_parts}), пропущено {dict(skipped)}; "
          f"слот = {args.slot_bars} такт(а)")
    n = max(1, score["slots"])
    print(f"слотов с диатоническим аккордом оригинала {score['slots']} (всех слотов {score['slots_all']})")
    for k in ("material", "material_raw_adapted", "viterbi_loo", "template", "tonic"):
        if k != "template" or args.slot_bars == 2:
            print(f"  {k:22s} {score[k]:5d}  {score[k] / n:.3f}")
    if args.slot_bars == 2:
        print(f"compose: треков {score['compose_tracks']}, "
              f"хук и гармония из материала {score['compose_from_material']}, "
              f"слотов дропа {score['compose_slots']}, совпало {score['compose_match']} "
              f"({score['compose_match'] / max(1, score['compose_slots']):.3f})")
        for k in sorted(k for k in score if k.startswith("compose_refused")):
            print(f"  {k}: {score[k]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
