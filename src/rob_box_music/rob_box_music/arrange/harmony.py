"""Гармония: аккорды лада и голосоведение пэда (ADR-0149 §3.6, research 4.2); пул прогрессий и размер аккорда —
из стиля (``knowledge.Style``, ADR-0153 §2.2). Есть материал партитуры — ступени из его аккордов
(:func:`from_material`, ADR-0154 §3.3); пробелы материала — Витерби по выученной таблице переходов
(``knowledge.PROGRESSION_TRANSITIONS``, :func:`viterbi`)."""

from __future__ import annotations

import bisect
import itertools
import math
import random
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..diversity import weighted_pick
from ..material import ChordSpan, Phrase, ScoreMaterial, meter_map
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, PitchEvent

#: Одна прогрессия — не больше ``PROGRESSION_CAP`` раз за ``PROGRESSION_WINDOW`` треков подряд (ADR-0149 A13).
PROGRESSION_CAP, PROGRESSION_WINDOW = 3, 10
#: Сколько скомпонованных, но не сыгранных треков может затесаться в окно (#3460, A13: 4/10 на случайной серии): N+1
#: компонуется заранее и в ``history`` попадает, а сет остановили/сменили до его старта — в записи его нет, и окно из
#: 9 прошлых записей по ``history`` покрывает меньше сыгранных треков. По одному на сет, окно из 10 — до трёх сетов.
PROGRESSION_SKIPPED = 3
#: Сколько прошлых треков ``history`` видит подбор прогрессии (и сколько хранит ``SetMemory``).
PROGRESSION_LOOKBACK = PROGRESSION_WINDOW - 1 + PROGRESSION_SKIPPED


def progression_name(degrees: Sequence[int]) -> str:
    """Имя прогрессии в ``music_history.progression``: ступени через дефис."""
    return "-".join(str(d) for d in degrees)


def chord_pcs(style: kn.Style, key: Key, degree: int) -> Tuple[int, ...]:
    """Аккорд ступени ``degree`` из ``style.chord_size`` звуков лада терциями (не хроматика): (тоника, терция,
    квинта, …)."""
    scale = kn.SCALES[key.mode]
    return tuple((key.root + scale[(degree + 2 * k) % len(scale)]) % 12 for k in range(style.chord_size))


def voicings(pcs: Sequence[int], register: Tuple[int, int]) -> List[Tuple[int, ...]]:
    """Все тесные расположения (обращения) аккорда внутри регистра."""
    lo, hi = register
    out = []
    for inv in range(len(pcs)):
        order = list(pcs[inv:]) + list(pcs[:inv])
        for base in (m for m in range(lo, hi + 1) if m % 12 == order[0]):
            notes = [base]
            for pc in order[1:]:
                notes.append(notes[-1] + (pc - notes[-1]) % 12)
            if notes[-1] <= hi:
                out.append(tuple(notes))
    return out


def _movement(a: Sequence[int], b: Sequence[int]) -> int:
    return sum(abs(x - y) for x, y in zip(a, b))


def pad_chords(style: kn.Style, key: Key, degrees: Sequence[int], register: Tuple[int, int]) -> Tuple[Chord, ...]:
    """Петля прогрессии → обращения с минимальным движением голосов, включая стык «последний → первый».

    Цепочка «каждый от предыдущего» уплывает, и на повторе петли пэд прыгает (до 17 полутонов у
    тестов PR-2); поэтому перебор всех обращений петли (4 аккорда × ≤ 9 вариантов) по сумме
    движения по кругу, при равенстве — ближе к середине регистра.
    """
    options = [voicings(chord_pcs(style, key, d), register) for d in degrees]
    if not all(options):
        raise ValueError(f"прогрессия {tuple(degrees)} не помещается в регистр {register}")
    mid = sum(register) / 2

    def cost(combo: Tuple[Tuple[int, ...], ...]) -> Tuple[int, float]:
        ring = sum(_movement(combo[i - 1], combo[i]) for i in range(len(combo)))
        return ring, sum(abs(sum(v) / len(v) - mid) for v in combo)

    best = min(itertools.product(*options), key=lambda combo: (cost(combo), combo))
    return tuple(Chord(d, v) for d, v in zip(degrees, best))


def fit_progression(style: kn.Style, key: Key, notes: Sequence[PitchEvent], chord_beats: float, rng: random.Random,
                    recent: Sequence[str] = ()) -> Tuple[int, ...]:
    """Прогрессия из ``style.progressions``, аккорды которой покрывают больше всего звучания хука.

    ``notes`` — хук в долях от начала петли, аккорд держится ``chord_beats`` долей. ``recent`` — прогрессии
    прошлых треков (свежие первыми): сыгранная ``PROGRESSION_CAP`` раз за ``PROGRESSION_LOOKBACK`` не берётся, при ничьей —
    выбор сидом со штрафом за недавнее (``diversity.weighted_pick``).
    """
    def score(degrees: Tuple[int, ...]) -> float:
        triads = [set(chord_pcs(style, key, d)) for d in degrees]
        return sum(e.dur_beats for e in notes if e.midi % 12 in triads[int(e.beat // chord_beats) % len(triads)])

    window = list(recent)[:PROGRESSION_LOOKBACK]
    pool = style.progressions
    allowed = [d for d in pool if window.count(progression_name(d)) < PROGRESSION_CAP] or list(pool)
    scores = {degrees: score(degrees) for degrees in allowed}
    best = max(scores.values())
    top = {progression_name(d): d for d in allowed if scores[d] == best}
    return top[weighted_pick(list(top), window, rng)]


# ── Гармония из материала партитуры и запасной Витерби (ADR-0154 §3.3, §3.4, Н6) ──────────────────────────────

#: Вес мелодии против log P перехода (Н6: w = 1 — лучший из опробованных на 71 партитуре).
VITERBI_MELODY_WEIGHT = 1.0
#: Петля из ≥ 2 разных аккордов: переход «последний → первый» реже этого — последний слот становится каденцией
#: по таблице (ступень, лучше всех ведущая из предпоследнего в первый).
CADENCE_MIN_P = 0.05
#: Недиатонический аккорд материала становится диатоническим, если у них столько общих звуков (2 из 3).
COMMON_TONES_MIN = 2
_SAMPLE_BEATS = BEATS_PER_BAR / STEPS_PER_BAR  # аккорд слота — по длительности на сетке 16-х трека
Table = Mapping[str, tuple]


def table_mode(mode: str) -> str:
    """Лад таблиц корпуса (переходы, тоны баса): семиступенные с большой терцией — ``major``, остальные — ``minor``;
    лад не из семи ступеней (пентатоника, хроматика) — ``ValueError`` (ступени 0..6 там не определены)."""
    scale = kn.SCALES.get(mode, ())
    if len(scale) != 7:
        raise ValueError(f"лад {mode!r} не семиступенный — таблицы переходов нет")
    return "major" if scale[2] == 4 else "minor"


def transition_table(mode: str) -> Table:
    """Таблица переходов лада трека (:func:`table_mode`)."""
    return kn.PROGRESSION_TRANSITIONS[table_mode(mode)]


def _triad(key: Key, degree: int) -> set:
    scale = kn.SCALES[key.mode]
    return {(key.root + scale[(degree + 2 * k) % 7]) % 12 for k in range(3)}


def _emissions(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int) -> List[List[float]]:
    """[слот][ступень] — доля длительности мелодии слота в трезвучии ступени (0, если слот пуст)."""
    out = []
    for slot in range(slots):
        inside = [e for e in notes if slot * chord_beats <= e.beat < (slot + 1) * chord_beats]
        total = sum(e.dur_beats for e in inside) or 1.0
        out.append([sum(e.dur_beats for e in inside if e.midi % 12 in _triad(key, d)) / total for d in range(7)])
    return out


def viterbi(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int,
            fixed: Sequence[Optional[int]] = (), table: Optional[Table] = None) -> Tuple[int, ...]:
    """Ступени ``slots`` слотов по ``w × доля мелодии в трезвучии + log P(переход)`` (HMM, ADR-0154 альтернатива F).

    ``fixed`` — ступени, известные заранее (аккорды материала), ``None`` — слот выбирает Витерби. ``table`` — для
    опытов leave-one-out; по умолчанию ``knowledge.PROGRESSION_TRANSITIONS`` лада ``key``. Ничья — меньшая ступень.
    """
    table = table or transition_table(key.mode)
    em = _emissions(key, notes, chord_beats, slots)
    pinned = list(fixed) + [None] * (slots - len(fixed))

    def allowed(slot: int) -> Sequence[int]:
        return range(7) if pinned[slot] is None else (pinned[slot],)

    def gain(slot: int, d: int) -> float:
        return VITERBI_MELODY_WEIGHT * em[slot][d]

    def top(cands: Iterable[Tuple[float, Tuple[int, ...]]]) -> Tuple[float, Tuple[int, ...]]:
        return max(cands, key=lambda sp: (sp[0], tuple(-d for d in sp[1])))

    best = {d: (math.log(table["start"][d]) + gain(0, d), (d,)) for d in allowed(0)}
    for slot in range(1, slots):
        best = {d: top((score + math.log(table["next"][p][d]) + gain(slot, d), path + (d,))
                       for p, (score, path) in best.items())
                for d in allowed(slot)}
    return top(best.values())[1]


def _adapt(chord: ChordSpan, key: Key) -> Optional[int]:
    """Ступень аккорда материала: диатоническая — как есть; недиатоническая — ступень с наибольшим числом общих
    звуков (≥ :data:`COMMON_TONES_MIN`; ничья — та же прима, затем меньшая ступень), иначе ``None``."""
    if chord.degree is not None:
        return chord.degree
    pcs = {(chord.root_pc + i) % 12 for i in kn.CHORD_INTERVALS[chord.quality]}
    scale = kn.SCALES[key.mode]
    common, _same_root, degree = max((len(pcs & _triad(key, d)), (key.root + scale[d]) % 12 == chord.root_pc, -d)
                                     for d in range(7))
    return -degree if common >= COMMON_TONES_MIN else None


def _chord_at(material: ScoreMaterial, starts: Sequence[float], beat: float) -> Optional[ChordSpan]:
    i = bisect.bisect_right(starts, beat) - 1
    if i < 0:
        return None
    chord = material.chords[i]
    return chord if beat < chord.beat + chord.dur_beats else None


def material_beat(material: ScoreMaterial, phrase: Phrase, scale: float = 1.0) -> Callable[[float], Optional[float]]:
    """Доля трека ``t`` → доля материала: перевод размера в 4/4 (``material.meter_map``, режим
    ``knowledge.TRIPLE_METER_MODE``) от первой ноты фразы (хук срезает начальную паузу, ``hook._onsets``) и множитель
    темпа ``scale`` (``hook.material_scale``). Доля в паузе перевода (4-я доля 3/4 в режиме ``pause``) — ``None``.
    Одно отображение на гармонию и бас материала."""
    mm = meter_map(material.meter)
    if mm is None:
        raise ValueError(f"размер {material.meter[0]}/{material.meter[1]} в 4/4 клуба не переводится")
    first = phrase.bar * mm.bar
    lead_in = next((e.beat for e in material.melody if e.beat >= first), first) - first
    origin = mm.to_club(lead_in)

    def at(t: float) -> Optional[float]:
        beat = mm.from_club(origin + t / scale)
        return None if beat is None else first + beat
    return at


def material_slots(material: ScoreMaterial, phrase: Phrase, chord_beats: float, slots: int,
                   scale: float = 1.0) -> List[Optional[ChordSpan]]:
    """Аккорд материала каждого слота трека — самый долгий на сетке 16-х трека (ничья — раньше в слоте); доля
    трека → доля материала — :func:`material_beat`. Слот без аккорда (пауза 3/4, нет разметки) — ``None``."""
    at = material_beat(material, phrase, scale)
    starts = [c.beat for c in material.chords]
    out: List[Optional[ChordSpan]] = []
    for slot in range(slots):
        weight: Dict[ChordSpan, int] = {}
        for k in range(int(round(chord_beats / _SAMPLE_BEATS))):
            beat = at(slot * chord_beats + k * _SAMPLE_BEATS)
            chord = _chord_at(material, starts, beat) if beat is not None else None
            if chord is not None:
                weight[chord] = weight.get(chord, 0) + 1
        out.append(max(weight, key=weight.__getitem__) if weight else None)
    return out


def _cadence(degrees: List[int], table: Table) -> List[int]:
    """Последний слот петли, из которого нет хода в первый (P < :data:`CADENCE_MIN_P`), — ступень, лучше всех
    ведущая из предпоследнего в первый."""
    first, last = degrees[0], degrees[-1]
    if len(set(degrees)) < 2 or last == first or table["next"][last][first] >= CADENCE_MIN_P:
        return degrees
    prev = degrees[-2]
    best = max(range(7), key=lambda d: (table["next"][prev][d] * table["next"][d][first], -d))
    return degrees[:-1] + [best]


def from_material(material: ScoreMaterial, phrase: Phrase, key: Key, notes: Sequence[PitchEvent],
                  chord_beats: float, slots: int, scale: float = 1.0) -> Tuple[int, ...]:
    """Ступени слотов трека из аккордов фразы материала (ADR-0154 §3.3) — гармония автора, а не шаблон стиля.

    Аккорд слота — :func:`material_slots`; недиатонический — :func:`_adapt`. Слоты без аккорда (и материал без
    разметки целиком) выбирает :func:`viterbi` по ``notes`` (хук в долях трека, тональность ``key``) с известными
    слотами как опорой; фраза на одном аккорде — тоже Витерби от первого слота (Н8: петля без движения). Петля
    закрывается каденцией (:func:`_cadence`). Лад ``key`` — лад материала (``hook.from_material``), ступени не
    зависят от тоники. Лад не семиступенный — ``ValueError``.
    """
    table = transition_table(key.mode)
    if transition_table(material.key.mode) is not table:
        raise ValueError(f"лад трека {key.mode} и лад материала {material.key.mode} разные — ступени не переносятся")
    fixed = [None if c is None else _adapt(c, material.key)
             for c in material_slots(material, phrase, chord_beats, slots, scale)]
    known = [d for d in fixed if d is not None]
    if slots > 1 and len(set(known)) == 1:
        fixed = [known[0]] + [None] * (slots - 1)
    degrees = list(viterbi(key, notes, chord_beats, slots, fixed, table))
    return tuple(_cadence(degrees, table)) if slots > 1 else tuple(degrees)


def melody_progression(key: Key, notes: Sequence[PitchEvent], chord_beats: float, slots: int) -> Tuple[int, ...]:
    """Ступени слотов под мелодию без аккордов автора (тема RTTTL целиком, ADR-0154 PR-7): :func:`viterbi` по
    выученной таблице лада и каденция к первому слоту (:func:`_cadence`). Лад не семиступенный — ``ValueError``."""
    table = transition_table(key.mode)
    degrees = list(viterbi(key, notes, chord_beats, slots, (), table))
    return tuple(_cadence(degrees, table)) if slots > 1 else tuple(degrees)


def chain_chords(style: kn.Style, key: Key, degrees: Sequence[int], register: Tuple[int, int],
                 loop: int = 4) -> Tuple[Chord, ...]:
    """Обращения длинной последовательности (тема целиком): по :func:`pad_chords` на каждые ``loop`` аккордов —
    перебор всех обращений длинной темы комбинаторно не считается."""
    return tuple(c for i in range(0, len(degrees), loop) for c in pad_chords(style, key, degrees[i:i + loop], register))


__all__ = ["CADENCE_MIN_P", "COMMON_TONES_MIN", "PROGRESSION_CAP", "PROGRESSION_LOOKBACK", "PROGRESSION_SKIPPED",
           "PROGRESSION_WINDOW", "VITERBI_MELODY_WEIGHT", "chain_chords", "chord_pcs", "melody_progression", "fit_progression", "from_material",
           "material_beat", "material_slots", "pad_chords", "progression_name", "table_mode", "transition_table", "viterbi", "voicings"]
