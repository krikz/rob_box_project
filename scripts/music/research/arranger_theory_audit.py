#!/usr/bin/env python3
"""Аудит выхода аранжировщика v2 по теории музыки (поручение Шифу 07.10: «что получается у аранжировщика, как это по
теории музыки и гармонии и как это звучит»). Отчёт — ``docs/music/arranger_theory_audit_2026-10-07.md``.

Режимы:

* ``notes`` — офлайн ``compose()`` по матрице стили × темы × сиды (модель ``Track`` — ноты всех ролей) и проверки:
  мелодия против аккорда на сильных долях, неаккордовые тоны (проходящие/вспомогательные/неразрешённые), бас против
  аккорда, параллели бас–мелодия, голосоведение пэда, качество аккордов (ум./ув.), регистры, лад ролей против лада
  трека и плана, сетка, бас против бочки, сильные доли материала партитуры после перевода (#3520), форма.
* ``wav`` — записи с робота: хрома против заявленной тональности, доля энергии вне лада, онсеты против сетки 16-х при
  известном темпе, полосы низ/середина/верх, корреляция L/R.
* ``live`` — трек 1 живого сета офлайн по материалу из лога против его записи (хрома нот против хромы записи).
* ``chords`` — корпус партитур: доля аккордов, чьё качество у автора не совпадает с диатонической триадой пэда.
* ``weights`` — оценка фикса «гармония Витерби под хук» при разном весе мелодии, без правки кода.

Только чтение: ничего не пишет в репо и не меняет таблицы. Без ГСЧ вне сидов плана — одинаковый вход даёт одинаковый
вывод.

    python scripts/music/research/arranger_theory_audit.py notes --materials <каталог JSON партитур> \\
        --seeds 5 --tracks 6 --json audit.json
    python scripts/music/research/arranger_theory_audit.py wav hk_1.wav lead_club.wav:132:E:minor

Материалы — JSON ``ScoreMaterial`` (пак ``~/pr6/lib`` на katana); без ``--materials`` тема «материал» идёт по хукам.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import gzip
import json
import logging
import pathlib
import statistics
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))  # пакет этого чекаута, не editable-установка основного

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music.arrange import harmony  # noqa: E402
from rob_box_music.arrange import hook as hooks  # noqa: E402
from rob_box_music.arrange.compose import compose  # noqa: E402
from rob_box_music.diversity import track_history  # noqa: E402
from rob_box_music.material import from_json, meter_map  # noqa: E402
from rob_box_music.model import BEATS_PER_BAR, Track  # noqa: E402
from rob_box_music.set_plan import seeded_plan  # noqa: E402
from rob_box_music.theme import seeded_profile  # noqa: E402

#: Секция темы целиком (ADR-0154 PR-7, #3523); до #3523 константы нет — первый дроп (скрипт сравнивает и старый код).
THEME_SECTION = getattr(kn, "THEME_SECTION", "drop")
#: #3529 (Ф1): ``Harmony.progression`` — аккорды секции ПО ТАКТАМ (гармония под мелодию каждой секции), а не петля по
#: 2 такта. Старый формат читается как раньше.
PER_BAR = hasattr(kn, "HOOK_HARMONY")
ARCHIVE = REPO / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
STYLES = ("club", "rave", "synthwave", "chiptune", "breaks", "dnb")
NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
EPS = 1e-6
#: Темы матрицы: (текст, найденные мелодии, части перечисления, брать ли материалы).
MOUNTAIN = ("hallofth_2", "mountainking", "hallofth", "hallofth_3", "kingofth_2")  # находки поиска на роботе 07.10
ENUM_PARTS = (("supermar", "supermar_4", "mariobro"), ("tetris", "tetris_2", "tetris_3"), ("zelda", "zelda_2", "zelda64"))
THEMES: Mapping[str, Tuple[str, Tuple[str, ...], Tuple[Tuple[str, ...], ...], bool]] = {
    "rtttl_found": ("в пещере горного короля", MOUNTAIN, (), False),
    "rtttl_table": ("космическая вечеринка", (), (), False),  # строка таблицы ``space``
    "enum": ("марио, тетрис и зельда", tuple(h for trio in zip(*ENUM_PARTS) for h in trio), ENUM_PARTS, False),
    "material": ("", (), (), True),  # по теме на каждую партитуру ``--materials``: :func:`material_themes`
}


def nm(midi: int) -> str:
    return f"{NAMES[midi % 12]}{midi // 12 - 1}"


def load_melodies() -> Dict[str, str]:
    out = {}
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                rec = json.loads(line)
                out[rec["name"]] = rec["rtttl"]
    return out


def load_materials(folder: Optional[str]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    if not folder:
        return out
    for path in sorted(pathlib.Path(folder).glob("*.json")):
        m = from_json(path.read_text(encoding="utf-8"))
        out[m.material_id] = m
    return out


# ── разбор трека ─────────────────────────────────────────────────────────────────────────────────────────────

def section_starts(track: Track) -> List[Tuple[int, int, str, frozenset]]:
    out, bar = [], 0
    for s in track.form.sections:
        out.append((bar, s.bars, s.name, s.roles))
        bar += s.bars
    return out


def section_of(starts, bar: int) -> Tuple[str, frozenset, int]:
    for first, bars, name, roles in starts:
        if first <= bar < first + bars:
            return name, roles, first
    return "?", frozenset(), 0


def triad_quality(pcs: Sequence[int]) -> str:
    a, b = (pcs[1] - pcs[0]) % 12, (pcs[2] - pcs[1]) % 12
    return {(4, 3): "maj", (3, 4): "min", (3, 3): "dim", (4, 4): "aug"}.get((a, b), f"{a}{b}")


def sounding(events, beat: float):
    return [e for e in events if e.beat <= beat + EPS and beat < e.beat + e.dur_beats - EPS]


def lead_line(events) -> Tuple[List[Any], List[Any]]:
    """Основной голос лида (по онсетам — ближайшая к предыдущей нота) и добавочные голоса (терции ``drop2``)."""
    by_onset: Dict[float, List[Any]] = collections.defaultdict(list)
    for e in events:
        by_onset[round(e.beat, 4)].append(e)
    line, extra, prev = [], [], None
    for beat in sorted(by_onset):
        notes = by_onset[beat]
        main = min(notes, key=lambda e: (abs(e.midi - prev) if prev is not None else 0, e.midi))
        line.append(main)
        extra += [e for e in notes if e is not main]
        prev = main.midi
    return line, extra


def nct_kind(prev, cur, nxt) -> str:
    """Вид неаккордового тона по соседям основного голоса: passing / neighbor / appoggiatura / escape / free."""
    step_in = prev is not None and 0 < abs(cur.midi - prev.midi) <= 2
    step_out = nxt is not None and 0 < abs(nxt.midi - cur.midi) <= 2
    if step_in and step_out:
        same_dir = (cur.midi - prev.midi) * (nxt.midi - cur.midi) > 0
        return "passing" if same_dir else "neighbor"
    if step_out:
        return "appoggiatura"
    if step_in:
        return "escape"
    return "free"


def loop_and_theme(track: Track) -> Tuple[Tuple[Any, ...], Tuple[Any, ...]]:
    """(петля аккордов, аккорды темы целиком): ``compose`` пишет в секцию ``knowledge.THEME_SECTION`` аккорды темы
    перед петлёй, в остальные секции — петлю."""
    prog = track.harmony.progression
    loop = next((v for k, v in prog.items() if k != THEME_SECTION), None) or prog[THEME_SECTION]
    head = prog.get(THEME_SECTION, loop)
    return tuple(loop), tuple(head[:len(head) - len(loop)]) if len(head) > len(loop) else ()


def analyze(track: Track, plan, tno: int, materials: Mapping[str, Any],
            refs: Optional[Mapping[str, Sequence[str]]] = None) -> Dict[str, Any]:
    style = plan.table
    key = track.key
    starts = section_starts(track)
    if PER_BAR:
        return analyze_per_bar(track, plan, tno, materials, refs)
    chords, theme_chords = loop_and_theme(track)
    loop_bars = len(chords) * 2
    span = len(theme_chords) * 2  # тактов темы целиком в секции ``knowledge.THEME_SECTION`` (ADR-0154 PR-7)

    def chord_at(bar: int):
        """Аккорд такта формы так же, как ``compose._bar_chords``: в секции темы первые ``span`` тактов — аккорды
        темы, остаток — петля с начала; в остальных секциях — петля по такту формы."""
        name, _roles, first = section_of(starts, bar)
        if name == THEME_SECTION and span:
            at = bar - first
            c = theme_chords[at // 2] if at < span else chords[((at - span) % loop_bars) // 2]
        else:
            c = chords[(bar % loop_bars) // 2]
        return c, harmony.chord_pcs(style, key, c.degree)

    every = list(theme_chords) + list(chords)
    pairs = [(chords[i - 1], chords[i]) for i in range(len(chords))] + list(zip(theme_chords, theme_chords[1:]))
    return analyze_body(track, plan, tno, materials, refs, chord_at, span, loop_bars, every, pairs,
                        [c.degree for c in chords])


def analyze_per_bar(track: Track, plan, tno: int, materials, refs) -> Dict[str, Any]:
    """Новый формат: аккорд такта — ``progression[секция][такт - начало]``; «аккорды» трека для качества и
    голосоведения — цепочка тактов формы без повторов подряд; петля — прогрессия истории трека."""
    style, key = plan.table, track.key
    starts = section_starts(track)
    prog = track.harmony.progression

    def chord_at(bar: int):
        name, _roles, first = section_of(starts, bar)
        c = prog[name][bar - first]
        return c, harmony.chord_pcs(style, key, c.degree)

    seq = [chord_at(b)[0] for b in range(track.form.bars_total)]
    every = [c for i, c in enumerate(seq) if i == 0 or c != seq[i - 1]]
    pairs = list(zip(every, every[1:])) or [(every[0], every[0])]  # весь трек на одном аккорде
    loop = [int(d) for d in track.history_key.progression.split("-")]
    span = track.hook.theme_bars if track.hook else 0
    return analyze_body(track, plan, tno, materials, refs, chord_at, span,
                        len(loop) * kn.HOOK_HARMONY.slot_bars, every, pairs, loop)


def analyze_body(track: Track, plan, tno: int, materials, refs, chord_at, span: int, loop_bars: int, every, pairs,
                 progression) -> Dict[str, Any]:
    style = plan.table
    key = track.key
    scale = set(kn.scale_pitch_classes(key.root, key.mode))
    starts = section_starts(track)
    ex: Dict[str, List[str]] = collections.defaultdict(list)
    m: Dict[str, Any] = {"theme_bars": span}

    # ── лад, аккорды, голосоведение пэда (петля и аккорды темы)
    quals = [triad_quality(harmony.chord_pcs(style, key, c.degree)) for c in every]
    m["chord_qualities"] = quals
    m["dim_aug_chords"] = sum(q in ("dim", "aug") for q in quals)
    if m["dim_aug_chords"]:
        bad = [(c.degree, q, [nm(v) for v in c.voicing]) for c, q in zip(every, quals) if q in ("dim", "aug")]
        ex["dim_aug"].append(f"{key.mode} {NAMES[key.root]} ступени {[c.degree for c in every]}: {bad[:2]}")
    leaps = [max(abs(x - y) for x, y in zip(sorted(a.voicing), sorted(b.voicing))) for a, b in pairs]
    m["pad_max_leap"] = max(leaps)
    pad_moves = [sum(abs(x - y) for x, y in zip(sorted(a.voicing), sorted(b.voicing))) for a, b in pairs]
    m["pad_mean_move"] = statistics.mean(pad_moves)

    lead = track.parts.get("lead")
    bass = track.parts.get("bass")
    pad = track.parts.get("pad")
    lead_ev = list(lead.pitches) if lead and lead.pitches else []
    bass_ev = list(bass.pitches) if bass and bass.pitches else []
    pad_ev = list(pad.pitches) if pad and pad.pitches else []

    # ── мелодия против аккорда
    line, extra = lead_line(lead_ev)
    per_sec: Dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
    strong_n = strong_ct = 0
    nct_kinds: collections.Counter = collections.Counter()
    clash_b9 = clash_tritone = lead_bass_m2 = long_strong_nct = off_key = 0
    for i, e in enumerate(line):
        bar = int(e.beat // BEATS_PER_BAR + EPS)
        sec, _roles, _first = section_of(starts, bar)
        kind = sec
        _c, pcs = chord_at(bar)
        pc = e.midi % 12
        in_chord = pc in pcs
        strong = abs((e.beat % BEATS_PER_BAR) % 2) < EPS  # доли 1 и 3
        c = per_sec[kind]
        c["notes"] += 1
        c["dur"] += e.dur_beats
        c["ct_dur"] += e.dur_beats if in_chord else 0
        if pc not in scale:
            off_key += 1
        if strong:
            strong_n += 1
            strong_ct += in_chord
            c["strong"] += 1
            c["strong_ct"] += in_chord
        if not in_chord:
            prev = line[i - 1] if i > 0 and line[i - 1].beat > e.beat - 2 * BEATS_PER_BAR else None
            nxt = line[i + 1] if i + 1 < len(line) and line[i + 1].beat < e.beat + 2 * BEATS_PER_BAR else None
            k = nct_kind(prev, e, nxt)
            nct_kinds[k] += 1
            if strong:
                c["strong_nct_" + k] += 1
            if strong and e.dur_beats >= 1.0 - EPS:
                long_strong_nct += 1
                if len(ex["long_strong_nct"]) < 3:
                    ex["long_strong_nct"].append(
                        f"такт {bar} ({sec}) {nm(e.midi)} {e.dur_beats:g} доли над {[NAMES[p] for p in pcs]} [{k}]")
            if strong and any((pc - p) % 12 == 1 for p in pcs):
                clash_b9 += 1
                c["strong_b9"] += 1
                if len(ex["b9"]) < 3:
                    ex["b9"].append(f"такт {bar} ({sec}) доля {e.beat % 4 + 1:g}: {nm(e.midi)} над "
                                    f"{[NAMES[p] for p in pcs]} — малая нона/секунда к тону аккорда [{k}]")
            if strong and (pc - pcs[0]) % 12 == 6 and k not in ("passing", "neighbor"):
                clash_tritone += 1
        bs = [b for b in sounding(bass_ev, e.beat)]
        if bs and any((e.midi - b.midi) % 12 in (1, 11) for b in bs):
            lead_bass_m2 += 1
            if len(ex["lead_bass_m2"]) < 3:
                ex["lead_bass_m2"].append(f"такт {bar} ({sec}) {nm(e.midi)} против баса {nm(bs[0].midi)}")
    m["lead_notes"] = len(line)
    m["lead_extra"] = len(extra)
    m["strong_ct_share"] = strong_ct / strong_n if strong_n else None
    m["ct_dur_share"] = (sum(c["ct_dur"] for c in per_sec.values()) / sum(c["dur"] for c in per_sec.values())
                         if line else None)
    m["by_section"] = {k: {"ct_dur": round(v["ct_dur"] / v["dur"], 3) if v["dur"] else None,
                           "strong_ct": round(v["strong_ct"] / v["strong"], 3) if v["strong"] else None,
                           "strong_b9": v["strong_b9"], "notes": v["notes"]} for k, v in per_sec.items()}
    m["nct_kinds"] = dict(nct_kinds)
    m["strong_b9"] = clash_b9
    m["strong_tritone_unres"] = clash_tritone
    m["long_strong_nct"] = long_strong_nct
    m["lead_bass_m2"] = lead_bass_m2
    m["lead_off_key"] = off_key
    # терции drop2: добавочный голос против аккорда
    ex_ct = sum(chord_at(int(e.beat // 4))[1].count(e.midi % 12) > 0 for e in extra)
    m["extra_ct_share"] = ex_ct / len(extra) if extra else None

    m.update(drop_oracle(track, style, line, starts, chord_at, span))
    m["hook_bars"] = track.hook.bars if track.hook else None
    m["b9_chromatic"] = sum(1 for e in line if e.midi % 12 not in scale and abs(e.beat % 2) < EPS
                            and any((e.midi - p) % 12 == 1 for p in chord_at(int(e.beat // 4))[1]))
    # минор: вводный тон мелодии (VII#) против натуральной VII в аккорде — переченье (аккорды строятся по натуральному
    # минору, классика в миноре берёт гармонический)
    lt, b7 = (key.root + 11) % 12, (key.root + 10) % 12
    m["leading_tone_clash"] = (sum(1 for e in line if e.midi % 12 == lt and b7 in chord_at(int(e.beat // 4))[1])
                               if kn.SCALES[key.mode][2] == 3 else 0)
    m["lead_leading_tone"] = sum(1 for e in line if e.midi % 12 == lt) if kn.SCALES[key.mode][2] == 3 else 0

    # ── бас против аккорда, бочки и мелодии
    rel = collections.Counter()
    first_rel = collections.Counter()
    seen_bars = set()
    for b in bass_ev:
        bar = int(b.beat // BEATS_PER_BAR + EPS)
        _c, pcs = chord_at(bar)
        r = {pcs[0]: "root", pcs[1]: "third", pcs[2]: "fifth"}.get(b.midi % 12, "other")
        rel[r] += 1
        if bar not in seen_bars:
            seen_bars.add(bar)
            first_rel[r] += 1
    m["bass_rel"] = dict(rel)
    m["bass_tritone"] = sum(1 for b in bass_ev if (b.midi - harmony.chord_pcs(style, key, chord_at(
        int(b.beat // 4))[0].degree)[0]) % 12 == 6)
    m["bass_first_rel"] = dict(first_rel)
    m["bass_off_key"] = sum(b.midi % 12 not in scale for b in bass_ev)
    kick = track.parts.get("kick")
    kick_on = {i for i, st in enumerate(kick.grid.steps) if st.on} if kick else set()
    bass_steps = [round(b.beat * 4) for b in bass_ev]
    m["bass_on_kick"] = sum(s in kick_on for s in bass_steps)
    m["bass_onsets"] = len(bass_steps)
    # параллельные квинты/октавы: соседние онсеты лида, где звучит бас и оба голоса сдвинулись в одну сторону
    par5 = par8 = moves = 0
    prev_pair = None
    for e in line:
        bs = sounding(bass_ev, e.beat)
        pair = (min(b.midi for b in bs), e.midi) if bs else None
        if pair and prev_pair and e.beat - prev_pair[2] <= 1.0 + EPS:
            db, dl = pair[0] - prev_pair[0], pair[1] - prev_pair[1]
            if db and dl and db * dl > 0:
                moves += 1
                i0, i1 = (prev_pair[1] - prev_pair[0]) % 12, (pair[1] - pair[0]) % 12
                if i0 == i1 == 7:
                    par5 += 1
                    if len(ex["par5"]) < 2:
                        ex["par5"].append(f"доля {e.beat:g}: бас {nm(prev_pair[0])}→{nm(pair[0])}, "
                                          f"лид {nm(prev_pair[1])}→{nm(pair[1])}")
                elif i0 == i1 == 0:
                    par8 += 1
        prev_pair = (pair[0], pair[1], e.beat) if pair else None
    m["par5"], m["par8"], m["same_dir_moves"] = par5, par8, moves

    # ── регистры
    if pad_ev and line:
        pad_top = max(e.midi for e in pad_ev)
        lead_min = min(e.midi for e in lead_ev)
        m["pad_top"], m["lead_min"], m["lead_max"] = pad_top, lead_min, max(e.midi for e in lead_ev)
        under = 0
        for e in lead_ev:
            ps = sounding(pad_ev, e.beat)
            if ps and e.midi <= max(p.midi for p in ps):
                under += 1
        m["lead_under_pad"] = under
    if pad_ev and bass_ev:
        m["bass_max"], m["pad_min"] = max(b.midi for b in bass_ev), min(p.midi for p in pad_ev)
        m["bass_pad_overlap"] = m["bass_max"] >= m["pad_min"]

    # ── сетка
    m["off_grid"] = {r: sum(abs(e.beat * 4 - round(e.beat * 4)) > 1e-3 for e in ev)
                     for r, ev in (("lead", lead_ev), ("bass", bass_ev), ("pad", pad_ev))}

    # ── форма
    lead_bars = sorted({int(e.beat // 4) for e in lead_ev})
    total = track.form.bars_total
    m["form"] = [(n, b) for _f, b, n, _r in starts]
    m["bars_total"] = total
    m["lead_bar_share"] = len(lead_bars) / total
    m["first_lead_bar"] = lead_bars[0] if lead_bars else None
    m["first_lead_s"] = lead_bars[0] * 4 * 60 / track.bpm if lead_bars else None
    m["track_s"] = total * 4 * 60 / track.bpm
    misaligned = [(n, f) for f, b, n, roles in starts if "lead" in roles and f % loop_bars]
    m["lead_sections_off_loop"] = misaligned

    # ── источник хука и лад
    step = plan.track(tno)
    src = "motif" if track.hook is None else ("material" if step.material and track.hook.source == step.material
                                              else "rtttl")
    m["source"] = src
    m["hook"] = track.hook.source if track.hook else None
    m["key"] = f"{NAMES[key.root]} {key.mode}"
    m["mode"] = key.mode
    m["plan_mode"] = plan.profile.mode
    m["style_modes"] = list(kn.STYLES[plan.style].modes)
    m["major_in_minor_style"] = (kn.SCALES[key.mode][2] == 4 and "major" not in kn.STYLES[plan.style].modes)
    m["mode_outside_style"] = key.mode not in kn.STYLES[plan.style].modes
    m["bpm"], m["genre"], m["swing"] = track.bpm, plan.genre, plan.swing
    m["pad_figure"], m["bass_figure"] = track.history_key.pad_figure, track.history_key.bass_figure
    m["progression"] = list(progression)

    # ── сильные доли материала после перевода (#3520)
    if src == "material":
        mat = materials[step.material]
        m.update(material_downbeats(mat, track, (refs or {}).get(step.material, ())))
        m["material_mode"] = mat.key.mode
        m["material_meter"] = f"{mat.meter[0]}/{mat.meter[1]}"
    m["examples"] = dict(ex)
    return m


def drop_oracle(track: Track, style, line, starts, chord_at, span: int) -> Dict[str, Any]:
    """Первый дроп (секция темы ``knowledge.THEME_SECTION``) целиком: доля длительности мелодии в тонах звучащего
    аккорда; то же у среднего шаблона стиля, у Витерби по таблице переходов (слот 2 такта и 1 такт) и у лучшей
    диатонической триады на слот (потолок). ``span`` — тактов темы целиком: CT темы и CT хука после неё; без темы —
    половины 8-тактовой петли (хук в 4 такта во второй половине повторяет мелодию на других аккордах)."""
    key = track.key
    drop = next(((f, b) for f, b, n, _r in starts if n == THEME_SECTION), None)
    if drop is None:
        return {}
    f, b = drop
    notes = [e for e in line if f * 4 <= e.beat < (f + b) * 4]
    total = sum(e.dur_beats for e in notes) or 1.0
    pcs = {d: set(harmony.chord_pcs(style, key, d)) for d in range(7)}

    def ct(sub) -> Optional[float]:
        d = sum(e.dur_beats for e in sub)
        return sum(e.dur_beats for e in sub if e.midi % 12 in chord_at(int(e.beat // 4))[1]) / d if d else None

    def cover(prog: Sequence[int], slot_bars: int = 2) -> float:
        return sum(e.dur_beats for e in notes
                   if e.midi % 12 in pcs[prog[int((e.beat - f * 4) // (slot_bars * 4)) % len(prog)]]) / total

    def oracle(slot_bars: int) -> float:
        best = 0.0
        for k in range(-(-b // slot_bars)):
            inside = [e for e in notes if k * slot_bars * 4 <= e.beat - f * 4 < (k + 1) * slot_bars * 4]
            best += max(sum(e.dur_beats for e in inside if e.midi % 12 in pcs[d]) for d in range(7)) if inside else 0
        return best / total

    rel = [dataclasses.replace(e, beat=e.beat - f * 4) for e in notes]
    try:
        vit2 = cover(harmony.viterbi(key, rel, 8.0, -(-b // 2)), 2)
        vit1 = cover(harmony.viterbi(key, rel, 4.0, b), 1)
    except (ValueError, KeyError):
        vit2 = vit1 = None
    out = {"drop_bars": b, "drop_ct_actual": ct(notes),
           "drop_ct_templates": statistics.mean(cover(p) for p in style.progressions),
           "drop_ct_viterbi2": vit2, "drop_ct_viterbi1": vit1, "drop_ct_oracle2": oracle(2), "drop_ct_oracle1": oracle(1),
           "drop_in_key": sum(e.dur_beats for e in notes
                              if e.midi % 12 in kn.scale_pitch_classes(key.root, key.mode)) / total}
    if span:
        out["drop_ct_theme"] = ct([e for e in notes if e.beat < (f + span) * 4])
        out["drop_ct_after_theme"] = ct([e for e in notes if e.beat >= (f + span) * 4])
    else:
        out["drop_ct_half1"] = ct([e for e in notes if e.beat < (f + 4) * 4])
        out["drop_ct_half2"] = ct([e for e in notes if (f + 4) * 4 <= e.beat < (f + 8) * 4])
    return out


def material_downbeats(mat, track: Track, refs: Sequence[str] = ()) -> Dict[str, Any]:
    """Где оказались ноты сильных долей материала (такт автора) в хуке трека: доля 1 / доли 1–3 клуба. Голос и такт
    главного мотива — как в ``compose`` (``hook.for_theme`` по RTTTL-эталонам темы)."""
    try:
        mat, anchor = hooks.for_theme(mat, refs) if hasattr(hooks, "for_theme") else (mat, None)
    except hooks.HookError:
        return {"mat_map_ok": False}
    phrase = hooks.pick_phrase(mat, anchor) if anchor is not None else hooks.pick_phrase(mat)
    mm = meter_map(mat.meter)
    first, last = phrase.bar * mm.bar, (phrase.bar + phrase.bars) * mm.bar
    notes = [e for e in mat.melody if first <= e.beat < last]
    hk = track.hook.notes
    n = min(len(notes), len(hk))
    shifts = {(hk[i].midi - notes[i].midi) % 12 for i in range(n)}
    if len(shifts) != 1:
        return {"mat_map_ok": False}
    down = [i for i in range(n) if abs(((notes[i].beat - first) % mm.bar)) < EPS]
    on1 = sum(abs(hk[i].beat % 4) < EPS for i in down)
    on13 = sum(abs(hk[i].beat % 2) < EPS for i in down)
    pickup = notes[0].beat - first if notes else 0.0
    return {"mat_map_ok": True, "mat_downbeat_notes": len(down), "mat_on1": on1, "mat_on13": on13,
            "mat_pickup_beats": round(pickup, 3)}


# ── матрица ──────────────────────────────────────────────────────────────────────────────────────────────────

#: Слово названия партитуры → слово названия RTTTL-записей того же произведения (эталоны ``hook.for_theme``): тема
#: «материал» — как на роботе: поиск находит и партитуру, и RTTTL по одному названию.
MATERIAL_REFS = {"mountain": "mountain king", "canon": "canon", "mario": "mario", "korobe": "tetris",
                 "greensleeve": "greensleeve", "elise": "elise"}


class _Refusals(logging.Handler):
    """Отказы материала в ``compose`` (лог ``🎵 [music v2] material=… отказ: …``) — причина по материалу."""

    def __init__(self) -> None:
        super().__init__()
        self.seen: List[Tuple[str, str]] = []

    def emit(self, record: logging.LogRecord) -> None:
        msg = record.getMessage()
        if "material=" in msg and "отказ:" in msg:
            mid = msg.split("material=", 1)[1].split()[0]
            self.seen.append((mid, msg.split("отказ:", 1)[1].split(" — ")[0].strip()))


def material_themes(materials: Mapping[str, Any], archive: Mapping[str, str]) -> List[Tuple[str, Tuple[str, ...], str]]:
    """(текст темы, найденные RTTTL, material_id) на каждую партитуру: тема — её название."""
    titles = {}
    with gzip.open(ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                rec = json.loads(line)
                titles[rec["name"]] = (rec.get("title") or "").lower()
    out = []
    for mid, mat in materials.items():
        low = mat.title.lower()
        word = next((v for k, v in MATERIAL_REFS.items() if k in low), None)
        found = tuple(n for n, t in titles.items() if word and word in t and n in archive)[:5]
        out.append((mat.title, found, mid))
    return out


def run_notes(args) -> int:
    melodies = load_melodies()
    materials = load_materials(args.materials)
    rows: List[Dict[str, Any]] = []
    refused: Dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
    catcher = _Refusals()
    logging.getLogger("rob_box_music").addHandler(catcher)
    logging.getLogger("rob_box_music").setLevel(logging.INFO)
    runs = [(theme, text, found, parts, None, args.tracks)
            for theme, (text, found, parts, use_mat) in THEMES.items() if not use_mat]
    runs += [("material", text, found, (), mid, args.material_tracks)
             for text, found, mid in material_themes(materials, melodies)]
    for style in STYLES:
        for theme, text, found, parts, mid, n_tracks in runs:
            for seed in range(args.seeds):
                mats = (mid,) if mid else ()
                profile = seeded_profile(text, style, found=found, parts=parts, materials=mats)
                rejected: Dict[str, str] = {}
                plan = seeded_plan(profile, seed, n_tracks, set_id=f"aud{seed}",
                                   materials=materials if mid else None, rejected=rejected)
                mel = {i: melodies[i] for i in profile.hook_ids if i in melodies}
                history: List[Dict[str, Any]] = []
                for no in range(1, n_tracks + 1):
                    catcher.seen.clear()
                    track = compose(plan, no, melodies=mel, history=history, materials=materials)
                    history.insert(0, track_history(track, plan.set_id))
                    refs = {mid: [melodies[i] for i in profile.theme_hooks if i in melodies]} if mid else {}
                    row = analyze(track, plan, no, materials, refs)
                    row.update(style=style, theme=theme, seed=seed, no=no, rejected=len(rejected))
                    rows.append(row)
                    for m_id, why in catcher.seen:
                        mat = materials[m_id]
                        refused[f"{mat.title[:40]} {mat.meter[0]}/{mat.meter[1]} (compose)"][why[:80]] += 1
                for m_id, why in rejected.items():
                    mat = materials[m_id]
                    refused[f"{mat.title[:40]} {mat.meter[0]}/{mat.meter[1]} (план)"][why[:80]] += 1
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(rows, ensure_ascii=False, indent=1, default=str), "utf-8")
    report(rows)
    if refused:
        print("\n## Материалы, отвергнутые планом или компоновкой (треков × причина)")
        for title, why in refused.items():
            print(f"  {title}: " + "; ".join(f"{k} ×{v}" for k, v in why.items()))
    return 0


def pct(n: int, d: int) -> str:
    return f"{100 * n / d:.0f}% ({n}/{d})" if d else "—"


def share(rows, cond) -> str:
    return pct(sum(1 for r in rows if cond(r)), len(rows))


def mean(xs) -> str:
    xs = [x for x in xs if x is not None]
    return f"{statistics.mean(xs):.2f}" if xs else "—"


def report(rows: List[Dict[str, Any]]) -> None:
    n = len(rows)
    print(f"Треков: {n}; стили {STYLES}; темы {list(THEMES)}")
    print("\n## Источник хука по темам")
    for theme in THEMES:
        sub = [r for r in rows if r["theme"] == theme]
        print(f"  {theme:<12} " + " ".join(f"{k}={v}" for k, v in collections.Counter(r['source'] for r in sub).items()))

    print("\n## Таблица по стилям (средние на трек / доля треков)")
    hdr = ("стиль", "треков", "CT сильн.", "CT длит.", "b9 сильн.≥1", "дл.NCT сильн.≥1", "лид–бас м2≥1",
           "ум/ув аккорд", "мажор при минор-стиле", "лад вне стиля", "бас на бочке", "пар.5/8≥1", "скачок пэда>7")
    print(" | ".join(hdr))
    for style in STYLES + ("ВСЕ",):
        sub = rows if style == "ВСЕ" else [r for r in rows if r["style"] == style]
        print(" | ".join(str(x) for x in (
            style, len(sub), mean(r["strong_ct_share"] for r in sub), mean(r["ct_dur_share"] for r in sub),
            share(sub, lambda r: r["strong_b9"] >= 1), share(sub, lambda r: r["long_strong_nct"] >= 1),
            share(sub, lambda r: r["lead_bass_m2"] >= 1), share(sub, lambda r: r["dim_aug_chords"] >= 1),
            share(sub, lambda r: r["major_in_minor_style"]), share(sub, lambda r: r["mode_outside_style"]),
            share(sub, lambda r: r["bass_on_kick"] >= 1), share(sub, lambda r: r["par5"] + r["par8"] >= 1),
            share(sub, lambda r: r["pad_max_leap"] > 7))))

    print("\n## По источнику хука")
    for src in ("rtttl", "material", "motif"):
        sub = [r for r in rows if r["source"] == src]
        if sub:
            print(f"  {src:<9} n={len(sub)} CT сильн.={mean(r['strong_ct_share'] for r in sub)} "
                  f"CT длит.={mean(r['ct_dur_share'] for r in sub)} b9≥1={share(sub, lambda r: r['strong_b9'] >= 1)} "
                  f"лид–бас м2 на трек={mean(r['lead_bass_m2'] for r in sub)} вне лада лид={mean(r['lead_off_key'] for r in sub)}")

    print("\n## CT по секциям (доля длительности лида в тонах аккорда; доля сильных долей)")
    agg: Dict[str, List[Tuple[float, float]]] = collections.defaultdict(list)
    for r in rows:
        for k, v in r["by_section"].items():
            agg[k].append((v["ct_dur"], v["strong_ct"]))
    for k, vs in sorted(agg.items()):
        print(f"  {k:<8} n={len(vs)} CT длит.={mean(a for a, _ in vs)} CT сильн.={mean(b for _, b in vs)}")

    print("\n## Потолок гармонии в первом дропе (доля длительности мелодии в тонах аккорда)")
    for src in ("rtttl", "material", "motif"):
        sub = [r for r in rows if r["source"] == src and "drop_ct_actual" in r]
        if not sub:
            continue
        print(f"  {src:<9} n={len(sub)} выбранная {mean(r['drop_ct_actual'] for r in sub)} | средний шаблон стиля "
              f"{mean(r['drop_ct_templates'] for r in sub)} | лучшая триада лада на 2 такта "
              f"{mean(r['drop_ct_oracle2'] for r in sub)} | на 1 такт {mean(r['drop_ct_oracle1'] for r in sub)} | "
              f"в ладу {mean(r['drop_in_key'] for r in sub)}")
        print(f"            Витерби по таблице переходов: на 2 такта {mean(r['drop_ct_viterbi2'] for r in sub)}, "
              f"на 1 такт {mean(r['drop_ct_viterbi1'] for r in sub)}")
        th = [r for r in sub if r["theme_bars"]]
        print(f"            тема целиком: {len(th)} треков, тактов темы {mean(r['theme_bars'] for r in th)}, дроп "
              f"{mean(r['drop_bars'] for r in th)} тактов — CT темы {mean(r.get('drop_ct_theme') for r in th)}, "
              f"хука после темы {mean(r.get('drop_ct_after_theme') for r in th)}")
        for bars in (4, 8):
            hb = [r for r in sub if r["hook_bars"] == bars and not r["theme_bars"]]
            print(f"            без темы, хук {bars} тактов: {len(hb)} — первая/вторая половина петли "
                  f"{mean(r.get('drop_ct_half1') for r in hb)} / {mean(r.get('drop_ct_half2') for r in hb)}")
    print(f"  b9 на сильной доле от хроматики мелодии: {sum(r['b9_chromatic'] for r in rows)} из "
          f"{sum(r['strong_b9'] for r in rows)}")
    minor = [r for r in rows if r["mode"] == "minor"]
    print(f"  минор: вводный тон VII# в мелодии {share(minor, lambda r: r['lead_leading_tone'] >= 1)} треков; "
          f"он же над аккордом с натуральной VII: {share(minor, lambda r: r['leading_tone_clash'] >= 1)} треков, нот "
          f"{sum(r['leading_tone_clash'] for r in minor)}/{sum(r['lead_leading_tone'] for r in minor)}")
    print(f"  бас на тритоне от примы аккорда (квинта ум. трезвучия): {share(rows, lambda r: r['bass_tritone'] >= 1)}"
          f" треков, нот {sum(r['bass_tritone'] for r in rows)}")

    print("\n## Неаккордовые тоны лида (основной голос, все треки)")
    kinds = collections.Counter()
    for r in rows:
        kinds.update(r["nct_kinds"])
    tot = sum(kinds.values())
    print("  " + ", ".join(f"{k} {pct(v, tot)}" for k, v in kinds.most_common()))
    print(f"  терции drop2 (добавочный голос) в аккорде: {mean(r['extra_ct_share'] for r in rows)}")

    print("\n## Аккорды: качество (все треки, слоты петли)")
    q = collections.Counter(x for r in rows for x in r["chord_qualities"])
    print("  " + ", ".join(f"{k} {pct(v, sum(q.values()))}" for k, v in q.most_common()))
    bymode = collections.defaultdict(collections.Counter)
    for r in rows:
        bymode[r["mode"]].update(r["chord_qualities"])
    for mode, c in sorted(bymode.items()):
        print(f"  {mode:<14} " + ", ".join(f"{k}={v}" for k, v in c.most_common()))

    print("\n## Лад трека против плана")
    print("  лады треков: " + ", ".join(f"{k}={v}" for k, v in collections.Counter(r['mode'] for r in rows).most_common()))
    for style in STYLES:
        sub = [r for r in rows if r["style"] == style]
        print(f"  {style:<10} лады {dict(collections.Counter(r['mode'] for r in sub))}; стиль {sub[0]['style_modes']}")
    mat = [r for r in rows if r["source"] == "material"]
    if mat:
        print(f"  материал: лад материала {dict(collections.Counter(r['material_mode'] for r in mat))} → лад трека "
              f"{dict(collections.Counter(r['mode'] for r in mat))}")

    print("\n## Бас")
    rel = collections.Counter()
    first = collections.Counter()
    for r in rows:
        rel.update(r["bass_rel"])
        first.update(r["bass_first_rel"])
    print("  все ноты: " + ", ".join(f"{k} {pct(v, sum(rel.values()))}" for k, v in rel.most_common()))
    print("  первая нота такта: " + ", ".join(f"{k} {pct(v, sum(first.values()))}" for k, v in first.most_common()))
    print(f"  бас вне лада (нот на трек): {mean(r['bass_off_key'] for r in rows)}")
    print(f"  бас на шаге бочки: {share(rows, lambda r: r['bass_on_kick'] >= 1)} треков; "
          f"нот {sum(r['bass_on_kick'] for r in rows)}/{sum(r['bass_onsets'] for r in rows)}")
    print(f"  параллельные квинты {sum(r['par5'] for r in rows)}, октавы {sum(r['par8'] for r in rows)} на "
          f"{sum(r['same_dir_moves'] for r in rows)} прямых движений бас–лид")

    print("\n## Регистры")
    print(f"  лид под верхом пэда (нот): {share(rows, lambda r: r.get('lead_under_pad', 0) >= 1)} треков")
    print(f"  бас заходит в пэд: {share(rows, lambda r: r.get('bass_pad_overlap'))}")
    print(f"  низ пэда: {collections.Counter(r.get('pad_min') for r in rows).most_common(6)}")
    print(f"  зазор пэд→лид, полутонов: {collections.Counter(r['lead_min'] - r['pad_top'] for r in rows if 'pad_top' in r).most_common(6)}")
    print(f"  скачок голоса пэда, макс: {collections.Counter(r['pad_max_leap'] for r in rows).most_common(8)}")

    print("\n## Ритм и сетка")
    off = collections.Counter()
    for r in rows:
        off.update(r["off_grid"])
    print(f"  нот вне сетки 16-х: {dict(off)}")
    if mat:
        ok = [r for r in mat if r.get("mat_map_ok")]
        d = sum(r["mat_downbeat_notes"] for r in ok)
        print(f"  материал: нот на сильной доле автора {d}; в клубе на доле 1: {pct(sum(r['mat_on1'] for r in ok), d)}, "
              f"на 1/3: {pct(sum(r['mat_on13'] for r in ok), d)}; затакт фразы ≠ 0: "
              f"{share(ok, lambda r: r['mat_pickup_beats'] > 0)}")
    print(f"  свинг плана: {collections.Counter(r['swing'] for r in rows).most_common(5)}")

    print("\n## Форма")
    print(f"  формы: {collections.Counter(tuple(n for n, _ in r['form']) for r in rows).most_common(4)}")
    print(f"  доля тактов с лидом: {mean(r['lead_bar_share'] for r in rows)}; первая нота лида, с: "
          f"{mean(r['first_lead_s'] for r in rows)}; трек, с: {mean(r['track_s'] for r in rows)}")
    print(f"  секции лида не с начала петли аккордов: {share(rows, lambda r: r['lead_sections_off_loop'])}")

    print("\n## Примеры")
    for keyname in ("dim_aug", "b9", "long_strong_nct", "lead_bass_m2", "par5"):
        shown = 0
        for r in rows:
            for line in r["examples"].get(keyname, [])[:1]:
                print(f"  [{keyname}] {r['style']}/{r['theme']}/seed{r['seed']}/трек{r['no']} {r['key']} "
                      f"hook={r['hook']}: {line}")
                shown += 1
                break
            if shown >= 4:
                break


# ── аудио ────────────────────────────────────────────────────────────────────────────────────────────────────

MAJ = (6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88)
MIN = (6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17)


def run_wav(args) -> int:
    import wave

    import numpy as np
    from scipy.signal import find_peaks, stft

    for spec in args.files:
        path, *rest = spec.split(":")
        bpm = float(rest[0]) if rest else None
        declared = (NAMES.index(rest[1]), rest[2]) if len(rest) >= 3 else None
        w = wave.open(path)
        sr, ch = w.getframerate(), w.getnchannels()
        a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(float).reshape(-1, ch) / 32768
        mono = a.mean(1)
        lr = float(np.corrcoef(a[:, 0], a[:, 1])[0, 1]) if ch == 2 else 1.0
        f, _t, Z = stft(mono, sr, nperseg=8192, noverlap=4096)
        P = (np.abs(Z) ** 2).mean(1)
        bands = {"низ<250": P[(f > 20) & (f < 250)].sum(), "середина 250–2к": P[(f >= 250) & (f < 2000)].sum(),
                 "верх≥2к": P[f >= 2000].sum()}
        tot = sum(bands.values())
        ok = (f > 60) & (f < 2500)
        pc = np.round(12 * np.log2(f[ok] / 440.0) + 9).astype(int) % 12
        c = np.bincount(pc, weights=P[ok], minlength=12)
        c = c / c.sum()
        best = max(((np.corrcoef(c, np.roll(prof, r))[0, 1], r, nm_) for r in range(12)
                    for nm_, prof in (("major", MAJ), ("minor", MIN))))
        diat = np.array([1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1], float)
        oos_best = min(1 - (c * np.roll(diat, r)).sum() for r in range(12))
        line = (f"== {pathlib.Path(path).name}: {len(mono) / sr:.0f} с, {sr} Гц, LR corr {lr:.2f}; полосы "
                + ", ".join(f"{k} {v / tot:.2f}" for k, v in bands.items())
                + f"; хрома → {NAMES[best[1]]} {best[2]} (r={best[0]:.2f}); вне лучшего диатонич. лада {oos_best:.2f}")
        if declared:
            scale = kn.scale_pitch_classes(*declared)
            mask = np.array([1.0 if i in scale else 0.0 for i in range(12)])
            prof = MAJ if kn.SCALES[declared[1]][2] == 4 else MIN
            r_decl = np.corrcoef(c, np.roll(prof, declared[0]))[0, 1]
            line += (f"; заявлено {NAMES[declared[0]]} {declared[1]}: r={r_decl:.2f}, вне лада трека "
                     f"{1 - (c * mask).sum():.2f}")
        top = np.argsort(c)[::-1][:5]
        line += "; топ хромы " + " ".join(f"{NAMES[i]}:{c[i]:.2f}" for i in top)
        if bpm:
            _f, _t, Z2 = stft(mono, sr, nperseg=1024, noverlap=1024 - 128)
            M = np.log1p(np.abs(Z2) * 50)
            flux = np.maximum(np.diff(M, axis=1), 0).sum(0)
            fs = sr / 128
            p, _ = find_peaks(flux, height=np.percentile(flux, 85), distance=int(fs * 0.06))
            tt = p / fs
            step = 60 / bpm / 4
            z = np.exp(2j * np.pi * tt / step).mean()
            dev = ((tt - np.angle(z) / (2 * np.pi) * step + step / 2) % step - step / 2) * 1000
            line += (f"; онсетов {len(tt) / (len(mono) / sr):.1f}/с, R сетки 16-х @ {bpm:g} = {abs(z):.2f}, "
                     f"|откл| медиана {np.median(np.abs(dev)):.0f} мс, >25 мс {np.mean(np.abs(dev) > 25):.0%}")
        print(line)
    return 0


def run_live(args) -> int:
    """Трек 1 живого сета офлайн по материалу из лога (стиль, окно, темп, тоника плана) — ноты против записи: та же
    проверка нот, что в ``notes``, и хрома нот (длительность по ролям лид/пэд/бас, каждая роль — равный вес) против
    хромы записи за первые ``--seconds`` с (трек 1 до блэнда). Сид живого сета в логе не записан — фигуры и форма
    могут отличаться, ступени и тоны баса материала от сида не зависят."""
    import dataclasses

    import numpy as np

    from rob_box_music.set_plan import SetPlan

    mat = from_json(pathlib.Path(args.material).read_text(encoding="utf-8"))
    melodies = load_melodies()
    found = tuple(args.found or ())
    profile = seeded_profile(args.theme, args.style, found=found, materials=(mat.material_id,))
    plan = seeded_plan(profile, 0, 2, set_id="live", genre=args.genre, materials={mat.material_id: mat})
    root = NAMES.index(args.root)
    plan = dataclasses.replace(plan, profile=dataclasses.replace(plan.profile, root=root), bpm=args.bpm)
    assert isinstance(plan, SetPlan)
    mel = {i: melodies[i] for i in profile.hook_ids if i in melodies}
    track = compose(plan, 1, melodies=mel, materials={mat.material_id: mat})
    refs = {mat.material_id: [melodies[i] for i in profile.theme_hooks if i in melodies]}
    row = analyze(track, plan, 1, {mat.material_id: mat}, refs)
    print(f"материал: {mat.title[:50]} {mat.meter[0]}/{mat.meter[1]} {NAMES[mat.key.root]} {mat.key.mode}")
    print(f"офлайн: {row['key']} источник={row['source']} ступени={row['progression']} {row['chord_qualities']} "
          f"CT сильн.={row['strong_ct_share']:.2f} CT длит.={row['ct_dur_share']:.2f} b9={row['strong_b9']} "
          f"лид–бас м2={row['lead_bass_m2']} вне лада лид={row['lead_off_key']} пэд={row['pad_figure']} "
          f"бас={row['bass_figure']} форма={row['form']} тема={row['theme_bars']} тактов "
          f"LT-переченье={row['leading_tone_clash']}/{row['lead_leading_tone']}")
    if "drop_ct_actual" in row:
        print(f"  дроп: выбранная {row['drop_ct_actual']:.2f}, лучшая триада/2 такта {row['drop_ct_oracle2']:.2f}, "
              f"на такт {row['drop_ct_oracle1']:.2f}")
    expect = np.zeros(12)
    for role in ("lead", "pad", "bass"):
        part = track.parts.get(role)
        if part and part.pitches:
            c = np.zeros(12)
            for e in part.pitches:
                c[e.midi % 12] += e.dur_beats
            expect += c / c.sum()
    expect /= expect.sum()
    import wave

    from scipy.signal import stft
    w = wave.open(args.wav)
    sr, ch = w.getframerate(), w.getnchannels()
    a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(float).reshape(-1, ch).mean(1) / 32768
    a = a[:int(args.seconds * sr)]
    f, _t, Z = stft(a, sr, nperseg=8192, noverlap=4096)
    P = (np.abs(Z) ** 2).mean(1)
    ok = (f > 60) & (f < 2500)
    pc = np.round(12 * np.log2(f[ok] / 440.0) + 9).astype(int) % 12
    heard = np.bincount(pc, weights=P[ok], minlength=12)
    heard /= heard.sum()
    print("  хрома нот   " + " ".join(f"{NAMES[i]}:{expect[i]:.2f}" for i in range(12)))
    print("  хрома записи " + " ".join(f"{NAMES[i]}:{heard[i]:.2f}" for i in range(12)))
    print(f"  корреляция нот и записи r={np.corrcoef(expect, heard)[0, 1]:.2f}; сдвиги ±1 полутон: "
          f"{np.corrcoef(np.roll(expect, 1), heard)[0, 1]:.2f} / {np.corrcoef(np.roll(expect, -1), heard)[0, 1]:.2f}")
    return 0


def run_weights(args) -> int:
    """Оценка фикса Ф1 без правки кода: ``harmony.viterbi`` на нотах первого дропа RTTTL-треков при разном весе
    мелодии (``VITERBI_MELODY_WEIGHT``; с #3529 — ``knowledge.HOOK_HARMONY.melody_weight``). С ADR-0154 PR-7 (#3523) от веса зависит и сама компоновка темы целиком
    (``harmony.melody_progression``) — «как скомпонован» тоже печатается."""
    melodies = load_melodies()
    saved = harmony.VITERBI_MELODY_WEIGHT if not PER_BAR else kn.HOOK_HARMONY
    try:
        for w in args.weights:
            if PER_BAR:
                kn.HOOK_HARMONY = dataclasses.replace(saved, melody_weight=w)
            else:
                harmony.VITERBI_MELODY_WEIGHT = w
            v1, v2, got = [], [], []
            for style in args.styles:
                for _theme, (text, found, parts, use_mat) in THEMES.items():
                    if use_mat:
                        continue
                    for seed in range(args.seeds):
                        profile = seeded_profile(text, style, found=found, parts=parts)
                        plan = seeded_plan(profile, seed, args.tracks, set_id=f"w{seed}")
                        history: List[Dict[str, Any]] = []
                        mel = {i: melodies[i] for i in profile.hook_ids if i in melodies}
                        for no in range(1, args.tracks + 1):
                            track = compose(plan, no, melodies=mel, history=history)
                            history.insert(0, track_history(track, plan.set_id))
                            r = analyze(track, plan, no, {})
                            v1.append(r["drop_ct_viterbi1"])
                            v2.append(r["drop_ct_viterbi2"])
                            got.append(r["drop_ct_actual"])
            print(f"w={w}: дроп как скомпонован {statistics.mean(got):.2f}; витерби 1 такт {statistics.mean(v1):.2f}, "
                  f"2 такта {statistics.mean(v2):.2f}, n={len(v1)}")
    finally:
        if PER_BAR:
            kn.HOOK_HARMONY = saved
        else:
            harmony.VITERBI_MELODY_WEIGHT = saved
    return 0


def run_chords(args) -> int:
    """Корпус партитур: доля длительности аккордов со ступенью, у которых качество автора отличается от диатонической
    триады лада материала — то, что сыграет пэд (``harmony._adapt`` берёт ступень, ``chord_pcs`` — триаду лада)."""
    import glob
    import random

    triad = {"maj": "maj", "dom7": "maj", "maj7": "maj", "min": "min", "min7": "min", "dim": "dim", "aug": "aug"}
    files = sorted(glob.glob(str(pathlib.Path(args.folder) / "*.json")))
    random.Random(args.seed).shuffle(files)
    stat = {m: collections.Counter() for m in ("minor", "major")}
    top = {m: collections.Counter() for m in stat}
    count = collections.Counter()
    for path in files[:args.limit]:
        try:
            data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        mode = data["key"]["mode"]
        if mode not in stat:
            continue
        count[mode] += 1
        sc = kn.SCALES[mode]
        for _beat, dur, _root, quality, degree in data["chords"]:
            if degree is None or quality not in triad:
                continue
            shape = ((sc[(degree + 2) % 7] - sc[degree]) % 12, (sc[(degree + 4) % 7] - sc[degree]) % 12)
            diat = {(4, 7): "maj", (3, 7): "min", (3, 6): "dim", (4, 8): "aug"}[shape]
            same = triad[quality] == diat
            stat[mode]["same" if same else "changed"] += dur
            if not same:
                top[mode][(degree, f"{diat}->{triad[quality]}")] += dur
    print("материалов", dict(count))
    for mode, c in stat.items():
        total = sum(c.values()) or 1.0
        print(f"{mode} длительность аккордов со ступенью: {round(total)} качество автора != диатоническому: "
              f"{c['changed'] / total:.3f}")
        print("   топ:", [(k, round(v / total, 3)) for k, v in top[mode].most_common(6)])
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    a = sub.add_parser("notes")
    a.add_argument("--materials")
    a.add_argument("--seeds", type=int, default=5)
    a.add_argument("--tracks", type=int, default=6)
    a.add_argument("--material-tracks", type=int, default=2, help="треков в сете темы-партитуры (трек 1 — материал)")
    a.add_argument("--json")
    b = sub.add_parser("wav")
    b.add_argument("files", nargs="+", help="file.wav[:bpm[:тоника:лад]]")
    c = sub.add_parser("live")
    c.add_argument("--material", required=True)
    c.add_argument("--style", required=True)
    c.add_argument("--genre")
    c.add_argument("--bpm", type=int, required=True)
    c.add_argument("--root", required=True, help="тоника плана трека 1 из лога (root)")
    c.add_argument("--theme", default="в пещере горного короля")
    c.add_argument("--found", nargs="*", default=list(MOUNTAIN), help="RTTTL-находки темы из лога (эталоны мотива)")
    c.add_argument("--wav", required=True)
    c.add_argument("--seconds", type=float, default=80.0)
    d = sub.add_parser("chords")
    d.add_argument("folder")
    d.add_argument("--limit", type=int, default=6000)
    d.add_argument("--seed", type=int, default=7)
    e = sub.add_parser("weights")
    e.add_argument("--weights", type=float, nargs="+", default=[1.0, 2.0, 4.0, 8.0])
    e.add_argument("--styles", nargs="+", default=["club", "rave", "synthwave"])
    e.add_argument("--seeds", type=int, default=3)
    e.add_argument("--tracks", type=int, default=4)
    args = ap.parse_args(argv)
    modes = {"notes": run_notes, "wav": run_wav, "live": run_live, "chords": run_chords, "weights": run_weights}
    return modes[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
