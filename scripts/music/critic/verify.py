#!/usr/bin/env python3
"""verify.py — верификатор утверждений судьи по ``Track``-модели (ADR-0157 §2.5, J1).

Каждое утверждение ``judge.Claim`` сводится к проверке **кодом**: ``confirmed`` / ``refuted`` / ``unverifiable``.

* hole / overlap / level / sustain / phrasing — детерминированная арифметика по секциям, нотам и ``Mix`` трека.
* out_of_key / chord_clash — по нотам и аккордам трека, через ``arranger_theory_audit`` (``section_starts``,
  ``section_of``, ``declared``): та же теория, что в аудите Ф1–Ф3, без копии.
* mix / masking — ``unverifiable``: нужен замер записи (A9/A10/LR/band), по модели не проверить.
* param_reading — чтение числа параметра кода; сверка с таблицей знания ``knowledge`` (``LPF_OPEN``, ``PAD_DETUNE``…).

Факт «нота вне лада» ещё не ошибка: нота хука (авторская хроматика), тон объявленного аккорда и аккорд с
качеством не из лада — по замыслу; такое утверждение ``refuted`` с причиной (ADR-0157 §1.4).
Пороги ниже — отчётные, не приёмочные (H4: цифр нет). Запуск: ``python verify.py <fixture.json>... --tracks <dir> --out report.md``.
"""

from __future__ import annotations

import argparse
import collections
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))  # пакет этого чекаута, не editable-установка основного
sys.path.insert(0, str(REPO / "scripts" / "music" / "research"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import arranger_theory_audit as audit  # noqa: E402
import bundle  # noqa: E402
import judge as jd  # noqa: E402
from rob_box_music import knowledge as kn  # noqa: E402

CONFIRMED, REFUTED, UNVERIFIABLE = "confirmed", "refuted", "unverifiable"
CLASSES = {"structural": ("hole", "overlap", "level", "sustain", "phrasing"), "notes": ("out_of_key", "chord_clash"),
           "mix": ("mix", "masking"), "param": ("param_reading",)}
HOLE_MIN_BEATS = 8.0      # паузы короче 2 тактов — дыхание, не дыра
HOLE_SHARE = 0.5          # дыра подтверждена, если ≥ половины окна без роли
OVERLAP_MIN_S = 2.0       # нахлёст дек короче — не нахлёст
LEVEL_GAP_DB = 6.0        # порог H5 (ADR-0157 §5): ниже/выше соседей более чем на 6 дБ
SUSTAIN_MAX_BEATS = 8.0   # H6: sus > 2 тактов допустим только у pad
PHRASING_REST_MAX = 0.10  # доля пауз в окне, ниже которой «без дыхания» подтверждено
BEATS_PER_BAR = 4
STEPS_PER_BAR = 16
STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
TOL_S = 5.0               # точность таймкодов судьи (ADR §2.5)


@dataclass(frozen=True)
class Verdict:
    claim: jd.Claim
    verdict: str
    reason: str
    style: str = ""

    @property
    def klass(self) -> str:
        return next(c for c, kinds in CLASSES.items() if self.claim.kind in kinds)


@dataclass(frozen=True)
class Placed:
    """Трек на временной оси записи: ``start_s`` — секунда файла, когда он встал на плеер."""

    track: Any
    start_s: float

    @property
    def spb(self) -> float:
        return 60.0 / self.track.bpm

    @property
    def end_s(self) -> float:
        return self.at(self.track.form.bars_total * BEATS_PER_BAR)

    def at(self, beat: float) -> float:
        return self.start_s + beat * self.spb

    def beat_of(self, t: float) -> float:
        return (t - self.start_s) / self.spb

    def active(self, t0: float, t1: float) -> bool:
        return self.start_s < t1 and self.end_s > t0


def _merge(spans: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    out: List[List[float]] = []
    for a, b in sorted(spans):
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(a, b) for a, b in out]


def played(track, role: str) -> List[Any]:
    """Ноты роли, которые реально звучат: в секциях, где роль объявлена (вне их рендер держит ``amp=0``)."""
    part = track.parts.get(role)
    starts = audit.section_starts(track)
    return [e for e in (part.pitches or ()) if role in audit.section_of(starts, int(e.beat // BEATS_PER_BAR))[1]] if part else []


def role_spans(p: Placed, role: str) -> List[Tuple[float, float]]:
    """Интервалы секунд, когда роль звучит: у тональной — по нотам секций с ролью, у ударной — по ударам сетки."""
    part = p.track.parts.get(role)
    if part is None:
        return []
    if part.pitches:
        return [(p.at(e.beat), p.at(e.beat + e.dur_beats)) for e in played(p.track, role)]
    steps, starts, spans = part.grid.steps, audit.section_starts(p.track), []
    for g in range(p.track.form.bars_total * STEPS_PER_BAR):  # удары сетки роли в секциях, где она объявлена
        if steps[g % len(steps)].on and role in audit.section_of(starts, g // STEPS_PER_BAR)[1]:
            spans.append((p.at(g * STEP_BEATS), p.at((g + 1) * STEP_BEATS)))
    return spans


def silence_s(spans: Sequence[Tuple[float, float]], t0: float, t1: float, min_gap_s: float) -> float:
    """Секунды окна без звука, считая только паузы не короче ``min_gap_s``."""
    gaps, cur = 0.0, t0
    for a, b in _merge([(max(a, t0), min(b, t1)) for a, b in spans if b > t0 and a < t1]) + [(t1, t1)]:
        if a - cur >= min_gap_s:
            gaps += a - cur
        cur = max(cur, b)
    return gaps


def _active(plan: Sequence[Placed], c: jd.Claim) -> List[Placed]:
    return [p for p in plan if p.active(c.t0, c.t1)]


def _window_notes(plan: Sequence[Placed], c: jd.Claim, role: str):
    for p in _active(plan, c):
        for e in played(p.track, role):
            if c.t0 <= p.at(e.beat) < c.t1:
                yield p, e


def check_hole(plan, c):
    role = c.args.get("role", "lead")
    act = _active(plan, c)
    if not act:
        return UNVERIFIABLE, "в окне нет ни одного трека модели"
    spans = [s for p in act for s in role_spans(p, role)]
    mute = silence_s(spans, c.t0, c.t1, HOLE_MIN_BEATS * act[0].spb)
    share = mute / max(c.t1 - c.t0, 1e-9)
    ok = share >= HOLE_SHARE
    return (CONFIRMED if ok else REFUTED), f"роль {role}: без звука {mute:.1f} с из {c.t1 - c.t0:.1f} с ({share:.0%}, порог {HOLE_SHARE:.0%})"


def check_overlap(plan, c):
    cuts = sorted({c.t0, c.t1, *(x for p in plan for x in (p.start_s, p.end_s) if c.t0 < x < c.t1)})
    over = sum(b - a for a, b in zip(cuts, cuts[1:]) if sum(p.active(a, b) for p in plan) >= 2)
    claimed = c.args.get("seconds")
    why = f"два трека одновременно {over:.1f} с в окне {c.t0:.0f}–{c.t1:.0f} с"
    if claimed is not None and abs(float(claimed) - over) > TOL_S:
        return REFUTED, f"{why}; судья назвал {claimed} с (допуск ±{TOL_S:.0f})"
    return (CONFIRMED if over >= OVERLAP_MIN_S else REFUTED), why


def check_level(plan, c):
    role, direction = c.args.get("role"), c.args.get("direction")
    act = _active(plan, c)
    if not role or direction not in ("low", "high") or not act:
        return UNVERIFIABLE, "нужны args.role и args.direction=low|high"
    p = act[0]
    mid = (c.t0 + c.t1) / 2
    first = max(p.beat_of(mid), 0) // BEATS_PER_BAR
    _n, roles, _f = audit.section_of(audit.section_starts(p.track), int(first))
    db = p.track.mix.level_db
    others = [db[r] for r in roles if r != role and r in db] if not c.args.get("other") else [db[c.args["other"]]]
    if role not in db or not others:
        return UNVERIFIABLE, f"роли {role} нет в Mix.level_db или нет соседей в секции"
    gap = db[role] - max(others)
    ok = gap <= -LEVEL_GAP_DB if direction == "low" else gap >= LEVEL_GAP_DB
    return (CONFIRMED if ok else REFUTED), f"level_db {role}={db[role]:.1f} против соседей {max(others):.1f}: разница {gap:+.1f} дБ (порог ±{LEVEL_GAP_DB:.0f}, по модели, не по записи)"


def check_sustain(plan, c):
    role = c.args.get("role")
    if not role:
        return UNVERIFIABLE, "нужна args.role"
    longest = max((e.dur_beats for _p, e in _window_notes(plan, c, role)), default=0.0)
    if role == "pad":
        return REFUTED, f"pad держит аккорд до {longest:g} долей по замыслу (H6 разрешает sus > 2 тактов только pad); sus — доли, не секунды"
    return (CONFIRMED if longest > SUSTAIN_MAX_BEATS else REFUTED), f"{role}: самая длинная нота окна {longest:g} долей (порог H6 {SUSTAIN_MAX_BEATS:g})"


def check_phrasing(plan, c):
    role = c.args.get("role", "lead")
    notes = sorted((p.at(e.beat), p.at(e.beat + e.dur_beats), e.dur_beats) for p, e in _window_notes(plan, c, role))
    if not notes:
        return UNVERIFIABLE, f"в окне нет нот роли {role}"
    rest = silence_s([(a, b) for a, b, _d in notes], c.t0, c.t1, 1e-6) / max(c.t1 - c.t0, 1e-9)
    run = best = 0
    for prev, cur in zip(notes, notes[1:]):
        run = run + 1 if cur[0] - prev[1] < 1e-6 and cur[2] <= 0.25 else 0
        best = max(best, run + 1)
    return (CONFIRMED if rest < PHRASING_REST_MAX else REFUTED), f"{role}: пауз {rest:.0%} окна (порог {PHRASING_REST_MAX:.0%}), самая длинная сплошная серия 16-х — {best} нот"


def _scale_pcs(track) -> frozenset:
    return frozenset((track.key.root + i) % 12 for i in kn.SCALES[track.key.mode])


def _chord_at(track, bar: int):
    starts = audit.section_starts(track)
    name, _roles, first = audit.section_of(starts, bar)
    chords, rel = track.harmony.progression[name], bar - first
    sec_bars = next(b for f, b, n, _r in starts if f == first)
    return chords[rel if len(chords) >= sec_bars else (rel // 2) % len(chords)]


def _hook_midis(track) -> frozenset:
    hook = track.hook
    return frozenset(e.midi for e in (*hook.notes, *hook.answer, *hook.theme)) if hook else frozenset()


def check_out_of_key(plan, c):
    role, want = c.args.get("role"), set(c.args.get("midi", ()))
    if not role:
        return UNVERIFIABLE, "нужна args.role"
    found, authored, chordal = [], [], []
    for p, e in _window_notes(plan, c, role):
        if want and e.midi not in want:
            continue
        if e.midi % 12 in _scale_pcs(p.track):
            continue
        found.append(e.midi)
        chord = _chord_at(p.track, int(e.beat // BEATS_PER_BAR))
        if e.midi in _hook_midis(p.track):
            authored.append(e.midi)
        elif e.midi % 12 in audit.declared(kn.STYLES[p.track.style], p.track.key, chord):
            chordal.append(e.midi)
    if not found:
        keys = {f"{kn.ROOTS[p.track.key.root]} {p.track.key.mode}" for p in _active(plan, c)}
        return REFUTED, f"{role}: нот {sorted(want) or 'вне лада'} в окне нет — лад {', '.join(sorted(keys))}"
    bad = len(found) - len(authored) - len(chordal)
    why = f"{role}: нот вне лада {len(found)} (хук/авторская хроматика {len(authored)}, тон объявленного аккорда {len(chordal)}, остальные {bad})"
    return (CONFIRMED if bad else REFUTED), why


def check_chord_clash(plan, c):
    role = c.args.get("role", "pad")
    act = _active(plan, c)
    if not act:
        return UNVERIFIABLE, "в окне нет трека модели"
    hits, seen = [], 0
    for p in act:
        a, b = int(max(p.beat_of(c.t0), 0) // BEATS_PER_BAR), int(min(p.beat_of(c.t1), p.track.form.bars_total * BEATS_PER_BAR) // BEATS_PER_BAR)
        for bar in range(a, max(b, a + 1)):
            chord = _chord_at(p.track, bar)
            seen += 1
            if role == "pad":
                v = sorted(chord.voicing)
                if any(y - x == 1 for x, y in zip(v, v[1:])):
                    hits.append(f"такт {bar}: {[audit.nm(x) for x in v]}")
            elif role == "bass":
                pcs = audit.declared(kn.STYLES[p.track.style], p.track.key, chord)
                for q, e in _window_notes([p], c, "bass"):
                    if int(e.beat // BEATS_PER_BAR) == bar and e.beat % BEATS_PER_BAR == 0 and e.midi % 12 not in pcs \
                            and any((e.midi - t) % 12 == 1 for t in pcs):
                        hits.append(f"такт {bar}: {audit.nm(e.midi)} на малую секунду выше тона аккорда")
    if role not in ("pad", "bass"):
        return UNVERIFIABLE, f"роль {role}: проверка конфликта есть только для pad и bass"
    return (CONFIRMED if hits else REFUTED), f"{role}: тактов в окне {seen}, конфликтов (м2) {len(hits)}" + (f"; пример {hits[0]}" if hits else "")


#: Параметр кода → (проверка, пояснение). Чтение числа «как сырого» — ложное, когда единица в таблице знания иная.
PARAM_NOTES = {
    "lpf": (lambda v: v == kn.LPF_OPEN, f"lpf=0 — «фильтр снят» (LPF_OPEN={kn.LPF_OPEN}), а не закрытый"),
    "pshift": (lambda v: abs(v - kn.PAD_DETUNE) < 1e-9, f"pshift={kn.PAD_DETUNE} — намеренный PAD_DETUNE (1/8 полутона)"),
    "delay": (lambda v: True, "delay — в долях (свинг/Хаас), не секунды"),
    "sus": (lambda v: True, "sus — в долях, не секунды"),
    "amp": (lambda v: True, f"amp между синтами и сэмплами несопоставим: единица — LAYER_MEASURED_DB, потолок MAX_LAYER_AMP={kn.MAX_LAYER_AMP}"),
    "tempo": (lambda v: True, "tempo= у лупа — родной темп сэмпла для стретча, не BPM трека"),
    "scale": (lambda v: True, "ноты — абсолютный MIDI (Scale.chromatic, root=0 в renardo.py); лад держит модель Track и её валидатор"),
}


def check_param(plan, c):
    param = c.args.get("param")
    if param not in PARAM_NOTES:
        return UNVERIFIABLE, f"параметр {param!r} не в таблице знания"
    test, note = PARAM_NOTES[param]
    value = c.args.get("value")
    if value is not None and not test(value):
        return UNVERIFIABLE, f"значение {value} не совпадает с табличным — по таблице не решить"
    return REFUTED, f"ложное чтение единицы: {note}"


def check_measure(plan, c):
    if c.args.get("param") == "sidechain":  # «нет сайдчейна у роли» — это поле Mix.duck_roles, проверяемо по модели
        role, act = c.args.get("role", "bass"), _active(plan, c)
        if not act:
            return UNVERIFIABLE, "в окне нет трека модели"
        ducked = [role in p.track.mix.duck_roles for p in act]
        return (REFUTED if any(ducked) else CONFIRMED), f"Mix.duck_roles: {role} под сайдчейном у {sum(ducked)} из {len(act)} треков окна (по модели; слышимость — замер записи)"
    return UNVERIFIABLE, "не проверяемо по модели: нужен замер записи (A9/A10/LR/band) — слух Шифу с таймкодом"


CHECKS = {"hole": check_hole, "overlap": check_overlap, "level": check_level, "sustain": check_sustain,
          "phrasing": check_phrasing, "out_of_key": check_out_of_key, "chord_clash": check_chord_clash,
          "mix": check_measure, "masking": check_measure, "param_reading": check_param}


def verify(plan: Sequence[Placed], claims: Sequence[jd.Claim], style: str = "") -> List[Verdict]:
    out = []
    for c in claims:
        try:
            verdict, why = CHECKS[c.kind](plan, c)
        except (KeyError, StopIteration) as exc:  # нет роли/секции в модели — честно «не проверяемо», а не молчаливый провал
            verdict, why = UNVERIFIABLE, f"модель не содержит нужного: {type(exc).__name__} {exc}"
        out.append(Verdict(c, verdict, why, style))
    return out


def load_fixture(path: Path, tracks_dir: Path) -> Tuple[Dict[str, Any], List[Placed], List[jd.Claim]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    plan = [Placed(bundle.load_track_file(tracks_dir / t["model"]), float(t["start_s"])) for t in data["tracks"]]
    return data, plan, jd.parse_claims(data["claims"])


def j1_table(verdicts: Sequence[Verdict]) -> Dict[str, Dict[str, int]]:
    """J1: по классам и по видам — счётчики вердиктов."""
    tab: Dict[str, Dict[str, int]] = collections.defaultdict(lambda: collections.Counter())
    for v in verdicts:
        for key in (v.klass, f"  {v.claim.kind}", "ВСЕГО"):
            tab[key][v.verdict] += 1
    return tab


def render_report(per_style: Mapping[str, Sequence[Verdict]]) -> str:
    allv = [v for vs in per_style.values() for v in vs]
    lines = ["# Отчёт верификатора судьи (J1)", "", f"Утверждений: {len(allv)}, стилей: {len(per_style)}.", "",
             "## J1 по классам", "", "| класс / вид | всего | подтв. | опроверг. | не проверяемо | J1 (подтв./всего) | подтв./решённых |",
             "|---|---|---|---|---|---|---|"]
    for key, cnt in sorted(j1_table(allv).items(), key=lambda kv: (kv[0] != "ВСЕГО", kv[0].strip() != kv[0], kv[0])):
        n, ok, bad = sum(cnt.values()), cnt[CONFIRMED], cnt[REFUTED]
        dec = ok + bad
        lines.append(f"| {key} | {n} | {ok} | {bad} | {cnt[UNVERIFIABLE]} | {ok / n:.0%} | {f'{ok / dec:.0%}' if dec else '—'} |")
    for style, vs in per_style.items():
        lines += ["", f"## {style}", "", "| t0–t1 | вид | вердикт | утверждение судьи | основание кода |", "|---|---|---|---|---|"]
        lines += [f"| {v.claim.t0:.0f}–{v.claim.t1:.0f} | {v.claim.kind} | {v.verdict} | {v.claim.claim.replace('|', '/')} | {v.reason.replace('|', '/')} |"
                  for v in vs]
    return "\n".join(lines) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Верификатор утверждений судьи по Track-модели")
    ap.add_argument("fixtures", nargs="+", type=Path, help="JSON стиля: tracks[{model,start_s}] + claims[]")
    ap.add_argument("--tracks", type=Path, required=True, help="каталог моделей треков (music_v2_tracks_dir)")
    ap.add_argument("--out", type=Path, default=None, help="куда писать report.md")
    args = ap.parse_args(argv)
    per_style = {}
    for fx in args.fixtures:
        data, plan, claims = load_fixture(fx, args.tracks)
        per_style[data["style"]] = verify(plan, claims, data["style"])
    text = render_report(per_style)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
