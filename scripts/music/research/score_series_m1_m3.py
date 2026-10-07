#!/usr/bin/env python3
"""Серия M1–M3 ADR-0154 §5 на N случайных материалах библиотеки (PR-6): один сид — одна выборка, печатаются только числа.

Вход — каталог библиотеки (JSON ``ScoreMaterial`` из ``pdmx_batch.py``), манифест батча и ``stats.json``
(``score_corpus_stats.py``, для таблицы Витерби leave-one-out). Выборка — ``random.Random(seed).sample`` по
отсортированным ``material_id`` принятых материалов.

* **M1** — доля принятых файлов манифеста (весь батч) и в случайной выборке строк манифеста; причины отказов.
* **M2** — на выборке материалов: ``compose`` с ``TrackPlan.material`` (тот же путь, что в роботе) даёт хук из
  материала? доля и медиана ``key_fit`` хука; причины отказа — по исключению ``hooks.from_material``.
* **M3** — окно выбранной хук-фразы (до 8 тактов, 4 слота): ступени слотов трека (``harmony.from_material``) против
  ступени автора слота (``material_slots``, диатоническая); рядом — Витерби без материала на таблице leave-one-out
  (``score_markov_harmony.corpus_counts`` без этой партитуры), шаблоны ``fit_progression`` и «везде I».
  Оговорка: ``from_material`` берёт таблицу переходов ``knowledge`` (выучена на всём корпусе, включая выборку) — только
  для слотов без аккорда и каденции; Витерби-база честная (leave-one-out).

    python scripts/music/research/score_series_m1_m3.py <lib> --manifest m.jsonl --stats stats.json --seed 20261007 -n 100
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import statistics
import sys
from dataclasses import replace
from typing import Any, Dict, List, Optional, Sequence

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "src" / "rob_box_music"))
sys.path.insert(0, str(HERE))

import score_markov_harmony as smh  # noqa: E402

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music import material as mt  # noqa: E402
from rob_box_music.arrange import compose as cp, harmony, hook as hooks  # noqa: E402
from rob_box_music.model import PitchEvent  # noqa: E402
from rob_box_music.set_plan import seeded_plan  # noqa: E402
from rob_box_music.theme import ThemeProfile  # noqa: E402

SLOTS, SLOT_BARS = 4, 2


def window_notes(material: mt.ScoreMaterial, start_bar: int) -> List[PitchEvent]:
    """Мелодия окна в долях трека (4/4 клуба, от первой ноты окна — как хук)."""
    bar = mt.bar_beats(material.meter)
    inside = [e for e in material.melody if start_bar * bar <= e.beat < (start_bar + 8) * bar]
    if not inside:
        return []
    zero = mt.club_beat(material.meter, inside[0].beat - start_bar * bar)
    return [PitchEvent(e.midi, mt.club_beat(material.meter, e.beat - start_bar * bar) - zero, e.dur_beats, 2)
            for e in inside]


def loo_table(rows: Sequence[Dict[str, Any]], skip: str, mode: str) -> Dict[str, Any]:
    starts, counts = smh.corpus_counts(list(rows), mode, skip)
    return {"start": tuple(s / sum(starts) for s in starts),
            "next": tuple(tuple(c / sum(r) for c in r) for r in counts)}


def read_manifest(path: pathlib.Path) -> List[Dict[str, Any]]:
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def m1(manifest: List[Dict[str, Any]], rng: random.Random, n: int) -> None:
    def line(label: str, ms: List[Dict[str, Any]]) -> None:
        ok = sum(m["status"] == "ok" for m in ms)
        codes = collections.Counter(m["code"] for m in ms if m["status"] != "ok")
        print(f"M1 {label}: файлов {len(ms)}, принято {ok} ({ok / max(1, len(ms)):.1%}; порог ≥ 95 %); отказы: "
              + (", ".join(f"{c} ×{k}" for c, k in codes.most_common()) or "нет"))

    line("весь батч", manifest)
    line(f"случайные {n} строк манифеста", rng.sample(manifest, min(n, len(manifest))))


def measure(material: mt.ScoreMaterial, name: str, rows: Sequence[Dict[str, Any]], score: collections.Counter,
            fits: List[float]) -> None:
    profile = ThemeProfile("", kn.DEFAULT_STYLE, 132, material.key.root, material.key.mode, (), None)
    plan = seeded_plan(profile, 1)
    plan = replace(plan, tracks=(replace(plan.track(1), material=material.material_id),))
    score["m2_materials"] += 1
    try:
        track = cp.compose(plan, 1, materials={material.material_id: material})
    except Exception as exc:  # noqa: BLE001 - причина отказа в отчёт
        score[f"m2_compose_error: {type(exc).__name__}"] += 1
        return
    if track.hook is None or track.hook.source != material.material_id:
        try:
            hooks.from_material(material, plan.bpm, plan.root(1), profile.mode, cp.hook_register(plan.table))
            reason = "пэд/лад гармонии"
        except Exception as exc:  # noqa: BLE001
            reason = str(exc).split(":")[0][:50]
        score[f"m2_refused: {reason}"] += 1
        return
    score["m2_hook_from_material"] += 1
    fits.append(float(track.hook.key_fit))
    # M3 — окно хук-фразы материала
    phrase = hooks.pick_phrase(material)
    notes = window_notes(material, phrase.bar)
    if len(notes) < 6:
        score["m3_windows_skipped_few_notes"] += 1
        return
    beats = SLOT_BARS * 4.0
    window = mt.Phrase(phrase.bar, 8, "new", 1)
    key = material.key
    author = [None if c is None else c.degree for c in harmony.material_slots(material, window, beats, SLOTS)]
    got = harmony.from_material(material, window, key, notes, beats, SLOTS)
    vit = harmony.viterbi(key, notes, beats, SLOTS, (), loo_table(rows, name, key.mode))
    tpl = harmony.fit_progression(kn.STYLES["club"], key, notes, beats, random.Random(phrase.bar))
    for i, a in enumerate(author):
        score["m3_slots_all"] += 1
        if a is None:
            continue  # слот без аккорда или недиатонический аккорд автора — не с чем сравнивать
        score["m3_slots"] += 1
        score["m3_material"] += got[i] == a
        score["m3_viterbi_loo"] += vit[i] == a
        score["m3_template"] += tpl[i] == a
        score["m3_tonic"] += a == 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("lib")
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--stats", required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("-n", type=int, default=100)
    args = ap.parse_args(argv)
    lib = pathlib.Path(args.lib).expanduser()
    manifest = read_manifest(pathlib.Path(args.manifest))
    rng = random.Random(args.seed)
    print(f"сид {args.seed}, выборка {args.n}")
    m1(manifest, rng, args.n)
    ok_ids = sorted(m["material_id"] for m in manifest if m["status"] == "ok")
    sample = random.Random(args.seed).sample(ok_ids, min(args.n, len(ok_ids)))
    rows = json.loads(pathlib.Path(args.stats).read_text(encoding="utf-8"))["rows"]
    score: collections.Counter = collections.Counter()
    fits: List[float] = []
    for mid in sample:
        name = mid.replace(":", "_") + ".json"
        measure(mt.from_json((lib / name).read_text(encoding="utf-8")), name, rows, score, fits)
    n = max(1, score["m2_materials"])
    hooked = score["m2_hook_from_material"]
    print(f"M2: материалов {score['m2_materials']}, хук из материала {hooked} ({hooked / n:.1%}; порог ≥ 80 %); "
          f"медиана key_fit хука {statistics.median(fits) if fits else float('nan'):.3f} (порог ≥ 0.9)")
    for k in sorted(k for k in score if k.startswith(("m2_refused", "m2_compose_error"))):
        print(f"  {k}: {score[k]}")
    s = max(1, score["m3_slots"])
    print(f"M3: слотов с диатоническим аккордом автора {score['m3_slots']} (всех {score['m3_slots_all']}), "
          f"окон без нот пропущено {score['m3_windows_skipped_few_notes']}")
    for k, label in (("m3_material", "material (from_material)"), ("m3_viterbi_loo", "viterbi_loo (без материала)"),
                     ("m3_template", "template (шаблоны стиля)"), ("m3_tonic", "tonic (везде I)")):
        print(f"  {label:30s} {score[k]:5d}  {score[k] / s:.3f}" + ("   порог ≥ 0.80" if k == "m3_material" else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
