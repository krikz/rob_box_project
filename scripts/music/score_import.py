#!/usr/bin/env python3
"""Офлайн-импортёр партитур MusicXML/MXL → ``ScoreMaterial`` (ADR-0154 §3.2, PR-2).

Партитура → skyline-мелодия, аккорды полутакта (24 трезвучия, как ``core/harmonize._best_chord``), басовый голос,
фразы/секции, тональность и размер из партитуры → ``rob_box_music.material.ScoreMaterial`` (модель, валидатор и JSON —
оттуда, здесь их нет). Результат:

* JSON-материал на партитуру в каталог ``--out-dir`` (библиотека **вне git**: партитуры и аранжировки чужие);
* строка ``score_index`` в SQLite ``--db`` (та же, что реестр ``works``; запись и лицензионный гейт —
  ``rob_box_music.works.write_score_index``); партитура подвязывается к произведению как источник ``pdmx:<id>``;
* отчёт ``--report``: сколько файлов принято, кто и почему отказан (M1), файлов/с (M7).

Только офлайн, на хосте/katana: ``pip install music21`` (зависимость скрипта; в образ робота и в ``requirements``
рантайма она не входит). ``PYTHONPATH`` — на ``src/rob_box_music`` (editable-пакет может смотреть в другой чекаут).

    python scripts/music/score_import.py <каталог|файл>... --out-dir lib/ --db voice_memory.db \\
        [--pdmx-csv PDMX.csv] [--license PD] [--jobs 8] [--report]

Файл из ``--pdmx-csv`` берётся как ``pdmx:<имя файла без расширения>`` с лицензией/рейтингом из CSV; строка с
``license_conflict=True`` отклоняется до разбора. Файл вне CSV — локальный, ``local:<sha8>``, лицензию обязан указать
``--license``: без неё он отклонён, а не принят «как есть». Размер: берётся самый длинный участок одного размера из
полных тактов (затакт и участки другого размера отбрасываются; см. «bars_used» в отчёте).
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import hashlib
import json
import pathlib
import sqlite3
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

warnings.filterwarnings("ignore")

from music21 import chord as m21chord, converter, expressions, key as m21key  # noqa: E402
from music21 import meter as m21meter, note as m21note, percussion as m21percussion, tempo as m21tempo  # noqa: E402

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music import material as mt  # noqa: E402
from rob_box_music import works  # noqa: E402
from rob_box_music.model import Key, PitchEvent  # noqa: E402
from rob_box_music.tonality import key_fit  # noqa: E402

MAJOR = kn.SCALES["major"]
MINOR = kn.SCALES["minor"]
DEGREE_NAMES = ("I", "bII", "II", "bIII", "III", "IV", "bV", "V", "bVI", "VI", "bVII", "VII")
STEP = 0.25  # 16-я в четвертях
TRIADS = [((r, (r + t) % 12, (r + 7) % 12), t == 4) for r in range(12) for t in (4, 3)]
MIN_BARS = 4  # короче — не материал темы
MIN_MELODY_NOTES = 8
PHRASE_BARS = 4
#: Слова текстовых меток, которые считаются названием части (остальное — темп/динамика); регистр не важен.
SECTION_WORDS = ("intro", "verse", "chorus", "bridge", "outro", "coda", "refrain", "interlude", "theme", "solo",
                 "вступление", "куплет", "припев", "проигрыш", "кода", "тема")
EXTENSIONS = (".mxl", ".musicxml", ".xml")


class Refusal(Exception):
    """Партитура не стала материалом: ``code`` — группа для отчёта (M1), ``reason`` — причина."""

    def __init__(self, code: str, reason: str) -> None:
        super().__init__(f"{code}: {reason}")
        self.code = code
        self.reason = reason


# ── ноты партитуры ─────────────────────────────────────────────────────────────────────────────────────────────

def load_notes(score) -> List[Tuple[float, float, int, int]]:
    """Все звучащие ноты: ``(offset_ql, dur_ql, midi, индекс партии)``, по возрастанию."""
    out = []
    for pi, part in enumerate(score.parts):
        for el in part.flatten().notes:
            if el.isRest:
                continue
            off, dur = float(el.offset), float(el.duration.quarterLength)
            if dur <= 0:
                continue
            if isinstance(el, (m21note.Unpitched, m21percussion.PercussionChord)):
                continue  # ударные без высоты (барабанная партия PDMX) — не мелодия и не бас, а не причина отказа файла
            pitches = el.pitches if isinstance(el, m21chord.Chord) else (el.pitch,)
            out += [(off, dur, int(p.midi), pi) for p in pitches]
    out.sort()
    return out


def bar_map(score) -> Tuple[List[float], List[float]]:
    """``(начала тактов, длины тактов в четвертях)`` по первой партии, как они записаны (без затакта и смен размера)."""
    measures = list(score.parts[0].getElementsByClass("Measure"))
    return [float(m.offset) for m in measures], [float(m.barDuration.quarterLength) for m in measures]


def bar_of(offs: Sequence[float], off: float) -> int:
    return max(0, bisect.bisect_right(offs, off + 1e-6) - 1)


def meters_of(score) -> List[str]:
    return sorted({ts.ratioString for ts in score.flatten().getElementsByClass(m21meter.TimeSignature)})


def tempos_of(score) -> List[float]:
    return sorted({float(t.number) for t in score.flatten().getElementsByClass(m21tempo.MetronomeMark) if t.number})


def key_signature(score) -> Optional[str]:
    for ks in score.flatten().getElementsByClass(m21key.KeySignature):
        try:
            k = ks.asKey()
            return f"{k.tonic.name} {k.mode}"
        except Exception:  # noqa: BLE001 - music21 бросает разное на экзотических знаках
            return f"{ks.sharps} sharps"
    return None


def first_bpm(score) -> Optional[int]:
    """Первая метрономная метка в четвертях/мин; вне ``BPM_RANGE`` или нет — None (temp не выдумываем)."""
    for t in score.flatten().getElementsByClass(m21tempo.MetronomeMark):
        try:
            bpm = round(float(t.getQuarterBPM()))
        except Exception:  # noqa: BLE001
            continue
        return bpm if mt.BPM_RANGE[0] <= bpm <= mt.BPM_RANGE[1] else None
    return None


def text_marks(score, offs: Sequence[float]) -> List[Tuple[int, str]]:
    """Текстовые метки партитуры ``(такт с 1, текст)``; ``offs`` — начала тактов."""
    out = []
    for el in score.flatten().getElementsByClass((expressions.TextExpression, expressions.RehearsalMark)):
        text = getattr(el, "content", None)
        if text:
            out.append((bar_of(offs, float(el.offset)) + 1, str(text)))
    return out


# ── мелодия, гармония, бас ─────────────────────────────────────────────────────────────────────────────────────

def skyline(notes) -> List[Tuple[float, float, int, int]]:
    """Верхняя нота на каждом онсете (сетка 16-х); длительность — до следующего онсета skyline."""
    by_onset: Dict[float, List[Tuple[int, float, int]]] = collections.defaultdict(list)
    for off, dur, midi, pi in notes:
        by_onset[round(off / STEP) * STEP].append((midi, dur, pi))
    out = []
    for q in sorted(by_onset):
        midi, dur, pi = max(by_onset[q])
        out.append([q, dur, midi, pi])
    for i in range(len(out) - 1):
        out[i][1] = min(out[i][1], out[i + 1][0] - out[i][0])
    return [tuple(x) for x in out if x[1] > 0]


def window_weights(notes, start: float, end: float, *, starts: Optional[Sequence[float]] = None,
                   max_dur: float = 0.0) -> Dict[int, float]:
    """Длительность каждого pitch class в окне. ``starts`` (онсеты, по возрастанию) и ``max_dur`` — ускорение: ноты,
    начавшиеся раньше ``start - max_dur``, окно не достигают."""
    w: Dict[int, float] = collections.defaultdict(float)
    lo = bisect.bisect_left(starts, start - max_dur) if starts is not None else 0
    hi = bisect.bisect_left(starts, end) if starts is not None else len(notes)
    for off, dur, midi, _pi in notes[lo:hi]:
        a, b = max(off, start), min(off + dur, end)
        if b > a:
            w[midi % 12] += b - a
    return w


def best_triad(weights: Mapping[int, float], previous: Optional[Tuple[int, ...]]) -> Optional[Tuple[int, ...]]:
    """Лучшее из 24 трезвучий (+50 % вес приме, +15 % инерция к предыдущему); пустое окно — предыдущее."""
    if not weights:
        return previous
    best, best_score = None, -1.0
    total = sum(weights.values())
    for pcs, _major in TRIADS:
        score = sum(weights.get(pc, 0.0) for pc in pcs) + 0.5 * weights.get(pcs[0], 0.0)
        if previous is not None and pcs == previous:
            score += 0.15 * total
        if score > best_score:
            best, best_score = pcs, score
    return best


def degree_name(root_pc: int, tonic: int) -> str:
    return DEGREE_NAMES[(root_pc - tonic) % 12]


def chords_per_half_bar(notes, offs: Sequence[float], lens: Sequence[float], tonic: int):
    """Аккорд каждой половины такта: ``(такт, половина, трезвучие, имя ступени)``."""
    starts = [n[0] for n in notes]
    max_dur = max((n[1] for n in notes), default=0.0)
    out = []
    prev = None
    for i, (o, ln) in enumerate(zip(offs, lens)):
        for h in range(2):
            w = window_weights(notes, o + h * ln / 2, o + (h + 1) * ln / 2, starts=starts, max_dur=max_dur)
            tri = best_triad(w, prev)
            if tri is None:
                continue
            out.append((i, h, tri, degree_name(tri[0], tonic) + ("" if (tri[1] - tri[0]) % 12 == 4 else "m")))
            prev = tri
    return out


def bass_line(notes) -> List[Tuple[float, float, int]]:
    """Басовый голос: на онсете — низшая ЗВУЧАЩАЯ нота; берётся, только если начинается здесь."""
    by_onset: Dict[float, List[Tuple[int, float]]] = collections.defaultdict(list)
    for off, dur, midi, _pi in notes:
        by_onset[round(off / STEP) * STEP].append((midi, dur))
    out = []
    active: List[Tuple[float, int]] = []
    for q in sorted(by_onset):
        active = [(end, m) for end, m in active if end > q + 1e-6]
        lowest = min(by_onset[q])
        if not active or lowest[0] <= min(m for _e, m in active):
            out.append((q, lowest[1], lowest[0]))
        active += [(q + dur, m) for m, dur in by_onset[q]]
    return out


# ── фактура и структура ────────────────────────────────────────────────────────────────────────────────────────

def texture_of_bar(acc_notes) -> str:
    """``sustained``/``block``/``broken``/``mixed``/``none`` по нотам аккомпанемента такта ``(off, dur, midi)``."""
    if not acc_notes:
        return "none"
    onsets: Dict[int, List[int]] = collections.defaultdict(list)
    for off, _dur, midi in acc_notes:
        onsets[round(off / STEP)].append(midi)
    n_on = len(onsets)
    simul = sum(len(v) for v in onsets.values()) / n_on
    if n_on <= 2 and simul >= 2:
        return "sustained"
    if simul >= 2.5:
        return "block"
    if n_on >= 4 and simul < 1.5:
        return "broken"
    return "mixed"


def bar_fingerprints(mel, offs: Sequence[float], lens: Sequence[float]) -> List[tuple]:
    """Отпечаток такта мелодии: ``(шаг 16-х от начала такта, длительность в 16-х, midi)``."""
    bars: Dict[int, List[Tuple[int, int, int]]] = collections.defaultdict(list)
    for off, dur, midi, _pi in mel:
        b = bar_of(offs, off)
        bars[b].append((int(round((off - offs[b]) / STEP)), int(round(dur / STEP)), midi))
    return [tuple(bars.get(i, ())) for i in range(len(offs))]


def phrase_relation(a, b) -> str:
    """Отношение соседних фраз (списки отпечатков тактов): ``repeat``/``sequence``/``rhythm``/``new``/``empty``."""
    fa = [n for bar in a for n in bar]
    fb = [n for bar in b for n in bar]
    if not fa or not fb:
        return "empty"
    if a == b:
        return "repeat"
    if len(fa) == len(fb) and [(s, d) for s, d, _m in fa] == [(s, d) for s, d, _m in fb]:
        return "sequence" if len({mb - ma for (_s, _d, ma), (_s2, _d2, mb) in zip(fa, fb)}) == 1 else "rhythm"
    return "new"


# ── рамка: какой участок партитуры берём ───────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Frame:
    """Участок из полных тактов одного размера; ``offs`` — начала тактов относительно ``origin`` (четверти)."""

    origin: float
    offs: Tuple[float, ...]
    lens: Tuple[float, ...]
    meter: Tuple[int, int]
    bars_total: int

    @property
    def end(self) -> float:
        return self.offs[-1] + self.lens[-1]


def _measure_rows(score) -> List[Tuple[float, float, Any]]:
    """``(начало, фактическая длина, размер)`` каждого такта первой партии; размер тянется до смены."""
    ms = list(score.parts[0].getElementsByClass("Measure"))
    if not ms:
        raise Refusal("no_measures", "в первой партии нет тактов")
    ts = next(iter(score.flatten().getElementsByClass(m21meter.TimeSignature)), None)
    rows = []
    for i, m in enumerate(ms):
        if m.timeSignature is not None:
            ts = m.timeSignature
        if ts is None:
            raise Refusal("no_time_signature", "размер не задан")
        off = float(m.offset)
        nxt = float(ms[i + 1].offset) if i + 1 < len(ms) else off + float(m.highestTime)
        rows.append((off, nxt - off, ts))
    return rows


def pick_frame(score) -> Frame:
    """Самый длинный (при равенстве — ранний) участок полных тактов одного размера; полный такт — длина равна
    размеру (последний такт может быть короче: конец пьесы). Затакт и участки других размеров отбрасываются."""
    rows = _measure_rows(score)
    runs: List[List[int]] = []
    for i, (_off, ln, ts) in enumerate(rows):
        nominal = float(ts.barDuration.quarterLength)
        full = abs(ln - nominal) < 1e-6 or (i == len(rows) - 1 and 0 < ln <= nominal + 1e-6)
        if full and runs and runs[-1] and rows[runs[-1][-1]][2].ratioString == ts.ratioString \
                and runs[-1][-1] == i - 1:
            runs[-1].append(i)
        elif full:
            runs.append([i])
    if not runs:
        raise Refusal("no_full_bars", "нет ни одного полного такта")
    best = max(runs, key=lambda r: (len(r), -r[0]))
    if len(best) < MIN_BARS:
        raise Refusal("too_short", f"самый длинный участок одного размера — {len(best)} т. (< {MIN_BARS})")
    ts = rows[best[0]][2]
    origin = rows[best[0]][0]
    return Frame(origin, tuple(rows[i][0] - origin for i in best), tuple(rows[i][1] for i in best),
                 (int(ts.numerator), int(ts.denominator)), len(rows))


def _in_frame(notes, frame: Frame):
    lo, hi = frame.origin, frame.origin + frame.end
    return [(off - lo, min(dur, hi - off), midi, pi) for off, dur, midi, pi in notes if lo - 1e-6 <= off < hi - 1e-6]


# ── сборка материала ───────────────────────────────────────────────────────────────────────────────────────────

def _accent(q: float, offs: Sequence[float]) -> int:
    """3 — первая доля такта, 2 — другая целая доля, 1 — между долями."""
    if abs(offs[bar_of(offs, q)] - q) < 1e-6:
        return 3
    return 2 if abs(q - round(q)) < 1e-6 else 1


def _events(rows: Iterable[Tuple[float, float, int]], offs: Sequence[float]) -> Tuple[PitchEvent, ...]:
    return tuple(PitchEvent(midi, round(q, 4), round(dur, 4), _accent(q, offs)) for q, dur, midi in rows)


def _chord_spans(chords, offs: Sequence[float], lens: Sequence[float], tonic: int, mode: str) -> Tuple[mt.ChordSpan, ...]:
    """Соседние полутакты с одним трезвучием сливаются в один ``ChordSpan``."""
    scale = kn.SCALES[mode]
    spans: List[List[Any]] = []
    for bar, half, tri, _name in chords:
        beat = offs[bar] + half * lens[bar] / 2
        dur = lens[bar] / 2
        if spans and spans[-1][2] == tri and abs(spans[-1][0] + spans[-1][1] - beat) < 1e-6:
            spans[-1][1] += dur
        else:
            spans.append([beat, dur, tri])
    out = []
    for beat, dur, tri in spans:
        rel = (tri[0] - tonic) % 12
        out.append(mt.ChordSpan(round(beat, 4), round(dur, 4), tri[0], "maj" if (tri[1] - tri[0]) % 12 == 4 else "min",
                                scale.index(rel) if rel in scale else None))
    return tuple(out)


def _phrases(fps: List[tuple]) -> Tuple[mt.Phrase, ...]:
    """4-тактовые блоки мелодии: отношение к предыдущему блоку с нотами и сколько раз такой блок звучит в пьесе."""
    blocks = [(s, fps[s:s + PHRASE_BARS]) for s in range(0, len(fps), PHRASE_BARS)]
    blocks = [(s, b) for s, b in blocks if any(b) and (len(b) == PHRASE_BARS or len(b) >= 2)]
    counts = collections.Counter(tuple(b) for _s, b in blocks)
    out, prev = [], None
    for s, b in blocks:
        rel = "new" if prev is None else phrase_relation(prev, b)
        out.append(mt.Phrase(s, len(b), "new" if rel == "empty" else rel, counts[tuple(b)]))
        prev = b
    return tuple(out)


def _sections(score, frame: Frame) -> Tuple[mt.ScoreSection, ...]:
    """Секции из текстовых меток-названий частей (``SECTION_WORDS``) и ``RehearsalMark``; без меток — пусто."""
    offs_abs = [frame.origin + o for o in frame.offs]
    seen: Dict[int, str] = {}
    for el in score.flatten().getElementsByClass((expressions.TextExpression, expressions.RehearsalMark)):
        text = str(getattr(el, "content", "") or "").strip()
        off = float(el.offset)
        is_part = isinstance(el, expressions.RehearsalMark) or any(w in text.lower() for w in SECTION_WORDS)
        if text and is_part and frame.origin - 1e-6 <= off < frame.origin + frame.end:
            seen.setdefault(bar_of(offs_abs, off), text[:40])
    bars = sorted(seen)
    return tuple(mt.ScoreSection(seen[b], b, (bars[i + 1] if i + 1 < len(bars) else len(frame.offs)) - b, "text")
                 for i, b in enumerate(bars))


def _key_of(score) -> Tuple[int, str]:
    try:
        k = score.analyze("key")
    except Exception as exc:  # noqa: BLE001
        raise Refusal("key_failed", f"{type(exc).__name__}: {exc}"[:100]) from exc
    return k.tonic.pitchClass, ("major" if k.mode == "major" else "minor")


def build_material(score, meta: Mapping[str, Any]) -> Tuple[mt.ScoreMaterial, Dict[str, Any]]:
    """Партитура music21 + метаданные (``material_id``, ``title``, ``composer``, ``source``, ``license``, ``rating``,
    ``n_views``, ``complexity``) → ``(ScoreMaterial, сведения для отчёта и индекса)``. Не годится — :class:`Refusal`."""
    if not score.parts:
        raise Refusal("no_parts", "в партитуре нет партий")
    frame = pick_frame(score)
    notes = _in_frame(load_notes(score), frame)
    mel = skyline(notes)
    if len(mel) < MIN_MELODY_NOTES:
        raise Refusal("melody_too_short", f"в мелодии {len(mel)} нот (< {MIN_MELODY_NOTES})")
    tonic, mode = _key_of(score)
    offs, lens = list(frame.offs), list(frame.lens)

    mel_set = {(off, midi) for off, _d, midi, _pi in mel}
    acc_by_bar: Dict[int, List[Tuple[float, float, int]]] = collections.defaultdict(list)
    for off, dur, midi, _pi in notes:
        if (round(off / STEP) * STEP, midi) not in mel_set:
            acc_by_bar[bar_of(offs, off)].append((off, dur, midi))
    textures = collections.Counter(texture_of_bar(acc_by_bar.get(i, [])) for i in range(len(offs)))
    fps = bar_fingerprints(mel, offs, lens)
    filled = [f for f in fps if f]
    phrases = _phrases(fps)
    best = max(phrases, key=lambda p: (p.repeats, -p.bar), default=None)
    stats = mt.MaterialStats(
        rating=meta.get("rating"), n_views=meta.get("n_views"), complexity=meta.get("complexity"),
        key_fit=key_fit([(n[2], n[1]) for n in mel], kn.ROOTS[tonic], mode),
        unique_bar_share=round(len(set(filled)) / max(1, len(filled)), 3),
        textures={k: round(v / len(offs), 3) for k, v in sorted(textures.items()) if k != "none"})
    try:
        material = mt.ScoreMaterial(
            material_id=meta["material_id"], title=str(meta.get("title") or "").strip(),
            composer=str(meta.get("composer") or "").strip(), source=str(meta.get("source") or ""),
            license=str(meta.get("license") or ""), meter=frame.meter, bpm=first_bpm(score), key=Key(tonic, mode),
            melody=_events(((q, d, m) for q, d, m, _pi in mel), offs),
            chords=_chord_spans(chords_per_half_bar(notes, offs, lens, tonic), offs, lens, tonic, mode),
            bass=_events(bass_line(notes), offs), phrases=phrases, sections=_sections(score, frame), stats=stats)
        mt.validate_material(material)
    except mt.MaterialError as exc:
        raise Refusal("license" if exc.path == "license" else "invalid_material", str(exc)) from exc
    info = {"bars_total": frame.bars_total, "bars_used": len(offs), "keysig": key_signature(score),
            "hook_phrase_bar": best.bar if best else None}
    return material, info


# ── метаданные PDMX и обработка одного файла ───────────────────────────────────────────────────────────────────

def _num(value: Any, kind: type) -> Optional[Any]:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return None if x != x else kind(x)


def load_pdmx_csv(path: str) -> Dict[str, Dict[str, Any]]:
    """``PDMX.csv`` → ``{имя файла без расширения: метаданные}`` (только нужные колонки; файл — только для отбора)."""
    csv.field_size_limit(min(sys.maxsize, 2 ** 31 - 1))
    out: Dict[str, Dict[str, Any]] = {}
    with open(path, encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            stem = pathlib.PurePosixPath(str(row.get("mxl") or row.get("path") or "").replace("\\", "/")).stem
            if not stem:
                continue
            na = {"", "NA", "nan", "None"}
            out[stem] = {
                "license": "" if row.get("license") in na else row.get("license", ""),
                "license_conflict": str(row.get("license_conflict")).strip().lower() == "true",
                "rating": _num(row.get("rating"), float), "n_ratings": _num(row.get("n_ratings"), int),
                "n_views": _num(row.get("n_views"), int), "complexity": _num(row.get("complexity"), int),
                "genres": "" if row.get("genres") in na else row.get("genres", ""),
                "title": row.get("title") or row.get("song_name") or "",
                "composer": row.get("composer_name") or row.get("artist_name") or ""}
    return out


def meta_for(path: pathlib.Path, pdmx: Optional[Mapping[str, Mapping[str, Any]]], license_arg: str,
             source_arg: str) -> Dict[str, Any]:
    """Метаданные файла: строка PDMX по имени файла, иначе локальный материал с лицензией из ``--license``."""
    row = (pdmx or {}).get(path.stem.replace(".mxl", ""))
    if row is not None:
        if row["license_conflict"]:
            raise Refusal("license_conflict", "PDMX: license_conflict=True — вне подмножества no_license_conflict")
        return {**row, "material_id": f"pdmx:{path.stem}", "source": f"PDMX {path.stem}"}
    if pdmx is not None:
        raise Refusal("not_in_csv", f"файла {path.name} нет в PDMX.csv")
    sha8 = hashlib.sha256(path.read_bytes()).hexdigest()[:8]
    return {"material_id": f"local:{sha8}", "license": license_arg, "source": source_arg or f"local {path.name}",
            "genres": "", "n_ratings": None}


def file_name_of(material_id: str) -> str:
    """Имя JSON в библиотеке: ``pdmx:123`` → ``pdmx_123.json`` (двоеточие в имени ломает Windows)."""
    return material_id.replace(":", "_") + ".json"


def process_file(args: Tuple[str, Optional[Mapping[str, Mapping[str, Any]]], str, str]) -> Dict[str, Any]:
    """Один файл → результат (picklable, для пула): ``status`` ok/refused, JSON материала и строка индекса или причина."""
    path_s, pdmx, license_arg, source_arg = args
    path = pathlib.Path(path_s)
    t0 = time.time()
    res: Dict[str, Any] = {"file": path.name, "status": "refused"}
    try:
        meta = meta_for(path, pdmx, license_arg, source_arg)
        score = converter.parse(str(path))
        meta.setdefault("title", "")
        if not str(meta.get("title") or "").strip():
            md = score.metadata
            meta["title"] = (getattr(md, "title", None) or getattr(md, "movementName", None) or path.stem) if md else path.stem
        if not str(meta.get("composer") or "").strip():
            meta["composer"] = (getattr(score.metadata, "composer", None) or "") if score.metadata else ""
        material, info = build_material(score, meta)
        row = works.score_index_row(material, bars=info["bars_used"], file=file_name_of(material.material_id),
                                    genres=str(meta.get("genres") or ""), n_ratings=meta.get("n_ratings"),
                                    keysig=info["keysig"])
        res.update(status="ok", material_id=material.material_id, json=mt.to_json(material), index=row, **info)
    except Refusal as exc:
        res.update(code=exc.code, reason=exc.reason)
    except Exception as exc:  # noqa: BLE001 - любой отказ парсера music21 — в отчёт, не в падение батча
        res.update(code="parse_error", reason=f"{type(exc).__name__}: {exc}"[:160])
    res["seconds"] = round(time.time() - t0, 2)
    return res


# ── пакет, отчёт, запись ───────────────────────────────────────────────────────────────────────────────────────

def collect_files(paths: Sequence[str]) -> List[pathlib.Path]:
    files: List[pathlib.Path] = []
    for p in map(pathlib.Path, paths):
        files += sorted(x for x in p.rglob("*") if x.suffix.lower() in EXTENSIONS) if p.is_dir() else [p]
    return files


def run_batch(files: Sequence[pathlib.Path], pdmx: Optional[Mapping[str, Mapping[str, Any]]], license_arg: str,
              source_arg: str, jobs: int) -> List[Dict[str, Any]]:
    jobs_args = [(str(f), pdmx, license_arg, source_arg) for f in files]
    if jobs <= 1:
        return [process_file(a) for a in jobs_args]
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(process_file, jobs_args, chunksize=4))


def store(results: Sequence[Mapping[str, Any]], out_dir: Optional[pathlib.Path], db: Optional[str]) -> Dict[str, Any]:
    """Записать принятые материалы в каталог и их строки в ``score_index``; вернуть итог записи (с отказами гейта)."""
    ok = [r for r in results if r["status"] == "ok"]
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        for r in ok:
            (out_dir / file_name_of(r["material_id"])).write_text(r["json"], encoding="utf-8")
    written, gate_rejected = len(ok), []
    if db:
        conn = sqlite3.connect(db)
        try:
            written, gate_rejected = works.write_score_index(conn, [r["index"] for r in ok])
        finally:
            conn.close()
    return {"json_written": len(ok) if out_dir is not None else 0, "index_written": written,
            "index_rejected": gate_rejected}


def report(results: Sequence[Mapping[str, Any]], seconds: float, jobs: int, written: Optional[Mapping[str, Any]] = None) -> str:
    """Отчёт M1 (доля принятых и причины отказов) и M7 (файлов/с); медианы времени и доля использованных тактов."""
    ok = [r for r in results if r["status"] == "ok"]
    refused = [r for r in results if r["status"] != "ok"]
    n = len(results)
    codes = collections.Counter(r["code"] for r in refused)
    times = sorted(r["seconds"] for r in results)
    used = sum(r["bars_used"] for r in ok)
    total = sum(r["bars_total"] for r in ok)
    lines = [f"файлов {n}; принято {len(ok)} ({len(ok) / n:.1%} — M1, порог ≥ 95 %)" if n else "файлов 0",
             "отказы по причинам: " + (", ".join(f"{c} ×{k}" for c, k in codes.most_common()) or "нет"),
             f"время: всего {seconds:.1f} с, jobs={jobs}, {n / seconds:.2f} файл/с (M7, порог ≥ 1 файл/с на katana); "
             f"медиана на файл {times[len(times) // 2] if times else 0:.2f} с, максимум {times[-1] if times else 0:.2f} с"]
    if ok:
        lines.append(f"тактов использовано {used} из {total} ({used / max(1, total):.0%}; остальное — затакт и участки "
                     f"других размеров)")
    if written is not None:
        lines.append(f"записано: JSON {written['json_written']}, score_index {written['index_written']}, "
                     f"отклонено гейтом индекса {len(written['index_rejected'])}")
    lines += [f"  отказ {r['file']}: {r['code']}: {r['reason']}" for r in refused[:50]]
    if len(refused) > 50:
        lines.append(f"  … ещё {len(refused) - 50} отказов")
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")  # Windows: cp1252 не печатает русские причины
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="+", help="файлы или каталоги (рекурсивно): .mxl .musicxml .xml")
    ap.add_argument("--out-dir", help="каталог JSON-библиотеки (вне git)")
    ap.add_argument("--db", help="SQLite (та же, что реестр works): сюда пишется score_index")
    ap.add_argument("--pdmx-csv", help="PDMX.csv: метаданные и отбор по license_conflict (файл не коммитится)")
    ap.add_argument("--license", default="", help="лицензия локальных файлов; без неё они отклоняются")
    ap.add_argument("--source", default="", help="источник локальных файлов (для атрибуции)")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--report", action="store_true", help="напечатать отчёт M1/M7")
    ap.add_argument("--report-json", help="сырые результаты по файлам в JSON (без тел материалов)")
    args = ap.parse_args(argv)
    files = collect_files(args.paths)[:args.limit or None]
    pdmx = load_pdmx_csv(args.pdmx_csv) if args.pdmx_csv else None
    t0 = time.time()
    results = run_batch(files, pdmx, args.license, args.source, max(1, args.jobs))
    seconds = time.time() - t0
    written = store(results, pathlib.Path(args.out_dir) if args.out_dir else None, args.db) \
        if (args.out_dir or args.db) else None
    if args.report or not (args.out_dir or args.db):
        print(report(results, seconds, max(1, args.jobs), written))
    if args.report_json:
        slim = [{k: v for k, v in r.items() if k not in ("json", "index")} for r in results]
        pathlib.Path(args.report_json).write_text(json.dumps(slim, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
