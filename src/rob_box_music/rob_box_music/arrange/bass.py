"""Бас: фигуры баса — генераторы ``(style, key, bar_chords, synth, register) -> Part`` (ADR-0153 §2.2).

Рисунки — ``knowledge.BASS_FIGURES`` (ADR-0152 §3.3), генератор выбирается по ключу из реестра
``arrange.compose.BASS_GENERATORS``:

* ``offbeat`` — 4 ноты на такт в оффбит, тоника/квинта аккорда (ADR-0149 §3.5, research 4.1).
* ``rolling8`` — 8 нот на такт тоникой, «и» и «а» каждой доли.
* ``broken`` — 4 ноты мимо шагов ломаной бочки ``breakbeat`` (окно ``breaks``, ADR-0152 PR-8).
* ``octave8`` — шаги ``rolling8``, каждая вторая нота октавой выше: 8-битный бас ``pulse`` (ADR-0153 S2).
* ``acid16`` — 12 16-х на такт мимо долей с акцентами, октавными прыжками и срезом фильтра на каждую ноту; только
  ``tb303`` семьи ``hard`` (``knowledge.BASS_FIGURE_SYNTHS``, ADR-0152 PR-9).

Все комплементарны бочке (прямой или ломаной): ни одной ноты на доле, нота кончается к доле (ADR-0149 §3.4).

Линия ``walking`` (ADR-0153 S4, ``knowledge.BASS_LINES``) — четверти на долях: доля 1 — тон аккорда такта, 2–3 — тоны
аккорда (терция, квинта, септима — по чётности такта) ближе к предыдущей ноте, 4 — подход полутоном к первой ноте
следующего такта (хроматика — целая доля, ``Style.approach_max_beats``); следующего такта баса нет — тон аккорда.

Линия ``riff`` (ADR-0153 S5, рок) — восьмые по риффу стиля (``Style.bass_riffs``, такты по кругу): ``R`` — тон такта
(прима аккорда или тон материала), ``F`` — чистая квинта аккорда над ним (квинта ступени не чистая — тон такта), ``O`` —
октава тона такта, ``.`` — нота тянется. Тон такта — ближайший к тону прошлого такта (бас не прыгает по регистру).

Тон такта (ADR-0154 §3.3, PR-4): без материала — тоника ×(n−1) + квинта, как всегда. С материалом партитуры —
:func:`material_tones`: такт стоит на том тоне аккорда, на котором стоит басовый голос автора (обращение — терция,
квинта), последняя нота такта — подход полутоном к следующему, если он есть у автора; политика — выученная таблица
``knowledge.BASS_TONES`` (тон по умолчанию, потолок подходов на петлю). Шаги рисунка те же — бас мимо бочки.
"""

from __future__ import annotations

import bisect
import collections
import math
from dataclasses import replace
from typing import Dict, List, Mapping, NamedTuple, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..material import Phrase, ScoreMaterial
from ..model import APPROACH_MAX_BEATS, BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import harmony, rhythm

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
STEPS_PER_BEAT = STEPS_PER_BAR // BEATS_PER_BAR
FIFTH = 7  # полутонов: бас берёт только чистую квинту аккорда (у ум. и ув. трезвучия — приму, аудит П6)
TRITONE = 6  # уменьшённая квинта
AUG_FIFTH = 8  # увеличенная квинта


class BassTone(NamedTuple):
    """Тон такта баса: ``anchor`` — индекс тона аккорда, на котором стоит такт (0 прима, 1 терция, 2 квинта);
    ``approach`` — последняя нота такта на полутон от первой ноты следующего: −1 снизу, +1 сверху, 0 — без подхода."""

    anchor: int = 0
    approach: int = 0


ROOT_TONE = BassTone()
#: Тоны аккорда, на которые встаёт бас (``knowledge.BASS_TONE_RELATIONS`` без ``other`` — чужой ноты).
ANCHORS: Mapping[str, int] = {"root": 0, "third": 1, "fifth": 2}
#: Сколько долей назад искать звучащую ноту баса материала (дольше 4 тактов 4/4 нота баса не тянется).
_LOOKBACK_BEATS = 4 * BEATS_PER_BAR
Tones = Optional[Mapping[int, BassTone]]


def bar_notes(root_pc: int, fifth_pc: int, register: Tuple[int, int], count: int = 4,
              fifth_last: bool = True) -> Tuple[int, ...]:
    """``count`` нот такта: тоника, последняя — квинта аккорда (``fifth_last``), ближайшая к тонике внутри регистра.
    Квинта только чистая: у уменьшённого и увеличенного трезвучия третий тон — не квинта, такт стоит на приме (аудит
    П6)."""
    root = next(m for m in range(register[0], register[1] + 1) if m % 12 == root_pc)
    up = root + (fifth_pc - root) % 12
    fifth = up if up <= register[1] else up - 12
    if fifth < register[0] or not fifth_last or (fifth_pc - root_pc) % 12 in (TRITONE, AUG_FIFTH):
        fifth = root
    return (root,) * (count - 1) + (fifth,)


def perfect_fifth(pcs: Sequence[int]) -> int:
    """Квинта аккорда для баса: третий тон, если он чистая квинта от примы, иначе прима (ум. и ув. трезвучие)."""
    return pcs[2] if (pcs[2] - pcs[0]) % 12 == FIFTH else pcs[0]


def note_beats(steps: Sequence[int], i: int) -> float:
    """Длина ноты шага ``steps[i]``: до следующего шага рисунка (по кругу такта) или до доли — что раньше."""
    step = steps[i]
    gap = (steps[(i + 1) % len(steps)] - step) % STEPS_PER_BAR or STEPS_PER_BAR
    to_beat = STEPS_PER_BEAT - step % STEPS_PER_BEAT
    return min(gap, to_beat) * STEP_BEATS


def figure_bass(figure: kn.BassFigure, bars: Sequence[Tuple[int, Tuple[int, ...]]],
                register: Tuple[int, int], tones: Tones = None) -> Tuple[PitchEvent, ...]:
    """``bars`` — (номер такта формы, трезвучие аккорда в pitch class); ноты на шагах рисунка ``figure``. ``tones`` —
    тон такта формы (:class:`BassTone`, материал партитуры); такта нет — тоника ×(n−1) + квинта, как без материала."""
    steps = figure.steps
    out: List[PitchEvent] = []
    firsts: Dict[int, int] = {}
    for bar, pcs in bars:
        anchor = (tones or {}).get(bar, ROOT_TONE).anchor
        fifth = perfect_fifth(pcs)
        if anchor == ANCHORS["fifth"] and fifth == pcs[0]:
            anchor = ANCHORS["root"]  # квинта ум./ув. трезвучия не чистая: бас стоит на приме (аудит П6)
        notes = bar_notes(pcs[anchor], pcs[0] if anchor == ANCHORS["fifth"] else fifth, register, len(steps),
                          figure.fifth_last)
        firsts[bar] = len(out)
        for i, (step, midi) in enumerate(zip(steps, notes)):
            accent = figure.accents[i] if figure.accents else 3 if i == 0 else 2
            lifted = midi + 12 if i in figure.lift and midi + 12 <= register[1] else midi
            lpf = figure.lpf[(bar % 2) * len(steps) + i] if figure.lpf else 0.0
            out.append(PitchEvent(lifted, bar * BEATS_PER_BAR + step * STEP_BEATS, note_beats(steps, i), accent, lpf))
    if tones:
        _approaches(out, firsts, tones, steps, register)
    return tuple(out)


def _approaches(out: List[PitchEvent], firsts: Mapping[int, int], tones: Mapping[int, BassTone],
                steps: Sequence[int], register: Tuple[int, int]) -> None:
    """Последняя нота такта с подходом — полутон к первой ноте следующего такта (со стороны автора; не помещается в
    регистр — с другой). Только если нота коротка для хроматики (``model.APPROACH_MAX_BEATS``) и бас звучит в
    следующем такте."""
    last = len(steps) - 1
    if note_beats(steps, last) > APPROACH_MAX_BEATS:
        return
    for bar, start in firsts.items():
        side = tones.get(bar, ROOT_TONE).approach
        if not side or bar + 1 not in firsts:
            continue
        target = out[firsts[bar + 1]].midi
        midi = next((target + s for s in (side, -side) if register[0] <= target + s <= register[1]), None)
        if midi is not None:
            out[start + last] = replace(out[start + last], midi=midi)


def _sounding(bass: Sequence[PitchEvent], starts: Sequence[float], beat: float) -> Optional[PitchEvent]:
    """Низшая нота баса материала, звучащая на доле ``beat``."""
    notes = [bass[j] for j in range(bisect.bisect_right(starts, beat) - 1, -1, -1)
             if starts[j] > beat - _LOOKBACK_BEATS and beat < bass[j].beat + bass[j].dur_beats]
    return min(notes, key=lambda e: e.midi) if notes else None


def _author_approach(bass: Sequence[PitchEvent], starts: Sequence[float], target: Optional[float]) -> int:
    """Подход автора к доле ``target``: нота, взятая на ней, и предыдущая нота баса — на полутон (Н7, как
    ``score_material_probe``); ответ — сторона предыдущей от целевой (−1 снизу, +1 сверху), иначе 0."""
    if target is None:
        return 0
    j = bisect.bisect_left(starts, target - 1e-6)
    if j == 0 or j >= len(bass) or abs(starts[j] - target) > 1e-6:
        return 0
    goal = min(e.midi for e in bass[j:bisect.bisect_right(starts, target + 1e-6)])
    prev = min(e.midi for e in bass[bisect.bisect_left(starts, starts[j - 1]):j])
    return prev - goal if abs(prev - goal) == 1 else 0


def material_tones(style: kn.Style, material: ScoreMaterial, phrase: Phrase, degrees: Sequence[harmony.AnyChord],
                   chord_beats: float, scale: float = 1.0) -> Tuple[BassTone, ...]:
    """Тон каждого такта петли ``degrees`` (аккорды: ступень или ступень с качеством, Ф2; аккорд держится
    ``chord_beats`` долей) по басовому голосу материала
    (ADR-0154 §3.3, Н7). Доля трека → доля материала — ``harmony.material_beat`` (та же фраза и множитель темпа,
    3/4 — пауза на 4-й доле).

    Тон такта — нота баса автора, звучащая дольше всех на сетке 16-х такта, если она прима/терция/квинта
    сыгранного аккорда (ступень трека в ладу материала); чужая нота или баса нет — самый частый тон корпуса лада
    (``knowledge.BASS_TONES``, сегодня прима). Подход — если он есть у автора на стыке тактов; не больше
    ``round(approach_per_bar × тактов петли)`` на петлю, ранние раньше.
    """
    policy = kn.BASS_TONES[harmony.table_mode(material.key.mode)]
    default = ANCHORS[max(ANCHORS, key=lambda rel: policy[rel])]
    at = harmony.material_beat(material, phrase, scale)
    starts = [e.beat for e in material.bass]
    bars = int(round(len(degrees) * chord_beats / BEATS_PER_BAR))
    cap = round(policy["approach_per_bar"] * bars)
    out: List[BassTone] = []
    for b in range(bars):
        chord = harmony.sym(material.key, degrees[int(b * BEATS_PER_BAR // chord_beats)])
        pcs = harmony.chord_pcs(style, material.key, chord.degree, chord.quality)[:3]
        votes: Dict[int, int] = collections.Counter()
        for k in range(STEPS_PER_BAR):
            beat = at(b * BEATS_PER_BAR + k * STEP_BEATS)
            note = _sounding(material.bass, starts, beat) if beat is not None else None
            if note is not None:
                votes[note.midi % 12] += 1
        pc = max(votes, key=lambda p: (votes[p], -p)) if votes else None
        approach = _author_approach(material.bass, starts, at((b + 1) * BEATS_PER_BAR)) if cap > 0 else 0
        cap -= bool(approach)
        out.append(BassTone(pcs.index(pc) if pc in pcs else default, approach))
    return tuple(out)


def _part(name: str, style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
          register: Tuple[int, int], tones: Tones = None) -> Part:
    figure = kn.BASS_FIGURES[name]
    bars = [(bar, _tones(style, key, chord)) for bar, chord in bar_chords]
    return Part("bass", synth, rhythm.grid(figure.steps), figure_bass(figure, bars, register, tones), 0.0, register)


def _tones(style: kn.Style, key: Key, chord: Chord) -> Tuple[int, ...]:
    """Тоны объявленного аккорда такта (``model.chord_tones``: качество автора, гармонический V; Ф2)."""
    return harmony.chord_pcs(style, key, chord.degree, chord.quality or None)


def offbeat(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int], tones: Tones = None) -> Part:
    """Бас в оффбит по тактам формы ``bar_chords`` — (такт, аккорд); уровень ставит ``arrange.mix``."""
    return _part("offbeat", style, key, bar_chords, synth, register, tones)


def rolling8(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
             register: Tuple[int, int], tones: Tones = None) -> Part:
    """Ролл тоникой на «и» и «а» каждой доли по тактам формы ``bar_chords``; уровень ставит ``arrange.mix``."""
    return _part("rolling8", style, key, bar_chords, synth, register, tones)


def acid16(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
           register: Tuple[int, int], tones: Tones = None) -> Part:
    """Кислотная линия ``tb303``: 12 16-х на такт мимо долей, акцент первой 16-й доли, последняя 16-я доли — октавой
    выше; у каждой ноты свой срез фильтра (``PitchEvent.lpf``), волна в два такта; уровень ставит ``arrange.mix``."""
    return _part("acid16", style, key, bar_chords, synth, register, tones)


#: Тоны аккорда на долях 2 и 3 такта walking по чётности такта: индексы ``harmony.chord_pcs`` (1 — терция, 2 — квинта,
#: 3 — септима; у трезвучия септима — терция).
WALK_TONES: Tuple[Tuple[int, int], ...] = ((1, 2), (2, 3))


def _near(pc: int, ref: int, register: Tuple[int, int]) -> int:
    """Нота pitch class ``pc`` внутри регистра, ближайшая к ``ref`` (ничья — ниже)."""
    options = [m for m in range(register[0], register[1] + 1) if m % 12 == pc]
    return min(options, key=lambda m: (abs(m - ref), m))


def _approach(target: int, prev: int, register: Tuple[int, int]) -> int:
    """Полутон к ``target`` со стороны предыдущей ноты (снизу, если она ниже), не помещается — с другой стороны."""
    side = -1 if prev <= target else 1
    return next(target + s for s in (side, -side) if register[0] <= target + s <= register[1])


def walking(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int], tones: Tones = None) -> Part:
    """Walking-бас по тактам ``bar_chords``: четверти, подход к следующему такту — на 4-й доле (одна доля ≤
    ``style.approach_max_beats``); ``tones`` — тон 1-й доли такта из материала (иначе прима)."""
    bars = [(bar, _tones(style, key, chord)) for bar, chord in bar_chords]
    sounding = {bar for bar, _pcs in bars}
    firsts: Dict[int, int] = {}
    prev = register[0] + 6
    for bar, pcs in bars:
        anchor = (tones or {}).get(bar, ROOT_TONE).anchor
        if anchor == ANCHORS["fifth"] and perfect_fifth(pcs) == pcs[0]:
            anchor = ANCHORS["root"]
        prev = firsts[bar] = _near(pcs[anchor], prev, register)
    out: List[PitchEvent] = []
    for bar, pcs in bars:
        line = [firsts[bar]]
        fifth = perfect_fifth(pcs)
        for idx in WALK_TONES[bar % 2]:
            line.append(_near(fifth if idx == 2 else pcs[idx % len(pcs)], line[-1], register))
        nxt = firsts.get(bar + 1) if bar + 1 in sounding else None
        line.append(_approach(nxt, line[-1], register) if nxt is not None else _near(fifth, line[-1], register))
        out += [PitchEvent(m, bar * BEATS_PER_BAR + beat, 1.0, 3 if beat == 0 else 2) for beat, m in enumerate(line)]
    return Part("bass", synth, rhythm.grid(range(0, STEPS_PER_BAR, STEPS_PER_BEAT)), tuple(out), 0.0, register)


def broken(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
           register: Tuple[int, int], tones: Tones = None) -> Part:
    """Бас в обход ломаной бочки (``breaks``): тоника ×3 + квинта на шагах 1, 6, 9, 14, мимо шагов
    рисунка ``breakbeat``; уровень ставит ``arrange.mix``."""
    return _part("broken", style, key, bar_chords, synth, register, tones)


def octave8(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int], tones: Tones = None) -> Part:
    """8-битный бас: восемь нот на «и» и «а» каждой доли, «а» — октавой выше (если влезает в регистр); только синтом
    ``knowledge.BASS_FIGURE_SYNTHS["octave8"]``; уровень ставит ``arrange.mix``."""
    return _part("octave8", style, key, bar_chords, synth, register, tones)


#: Восьмых в такте риффа (``Style.bass_riffs``: строка на такт).
RIFF_EIGHTHS = 8


def _riff_notes(riff: str, tone: int, fifth: Optional[int], register: Tuple[int, int]) -> List[Tuple[int, int, int]]:
    """(восьмая, длина в восьмых, MIDI) нот такта риффа ``riff`` от тона такта ``tone``."""
    up = {"R": tone, "F": tone + ((fifth - tone) % 12 if fifth is not None else 0), "O": tone + 12}
    notes: List[Tuple[int, int, int]] = []
    for i, ch in enumerate(riff):
        if ch == ".":
            if notes:
                notes[-1] = (notes[-1][0], notes[-1][1] + 1, notes[-1][2])
            continue
        midi = up[ch] if up[ch] <= register[1] else up[ch] - 12
        notes.append((i, 1, midi if midi >= register[0] else tone))
    return notes


def riff(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
         register: Tuple[int, int], tones: Tones = None) -> Part:
    """Бас-рифф рока по тактам ``bar_chords``: восьмые по риффу ``style.bass_riffs[bar % n]`` на тоне такта (прима или
    тон материала ``tones``), квинте и октаве; уровень ставит ``arrange.mix``."""
    riffs = style.bass_riffs
    if not riffs or any(len(r) != RIFF_EIGHTHS or set(r) - set("RFO.") or r[0] == "." for r in riffs):
        raise ValueError(f"риффы баса {riffs!r}: не {RIFF_EIGHTHS} восьмых из R F O . (первая — нота)")
    out: List[PitchEvent] = []
    prev = register[0] + 6
    eighth = BEATS_PER_BAR / RIFF_EIGHTHS
    for bar, chord in bar_chords:
        pcs = _tones(style, key, chord)  # тоны объявленного аккорда (качество автора, гармонический V; Ф2)
        anchor = (tones or {}).get(bar, ROOT_TONE).anchor
        if anchor == ANCHORS["fifth"] and perfect_fifth(pcs) == pcs[0]:
            anchor = ANCHORS["root"]
        prev = tone = _near(pcs[anchor], prev, register)
        fifth = perfect_fifth(pcs) if perfect_fifth(pcs) != pcs[0] else None  # квинта только чистая (аудит П6)
        out += [PitchEvent(m, bar * BEATS_PER_BAR + i * eighth, n * eighth, 3 if i % 2 == 0 else 2)
                for i, n, m in _riff_notes(riffs[bar % len(riffs)], tone, fifth, register)]
    return Part("bass", synth, rhythm.grid(range(0, STEPS_PER_BAR, 2)), tuple(out), 0.0, register)


# ── Голосоведение баса против лида (#3549, аудит теории S7: м2/м9 лид–бас у 100 % треков джаза, параллели — 62 %) ──

#: Цены выбора ноты баса (:func:`against_lead`): нота, звучащая вместе с нотой лида на интервале
#: ``Style.bass_lead_clash``, и параллель на ``Style.bass_lead_parallels`` — дороже любой замены; замена ноты генератора
#: — дёшево и тем дороже, чем дальше; скачок больше :data:`LEAP_MAX` и повтор ноты — мелкие штрафы линии.
CLASH_COST = 10.0
PARALLEL_COST = 10.0
CHANGE_COST = 0.5
CHANGE_COST_PER_SEMITONE = 0.05
LEAP_MAX = 9
LEAP_COST = 1.0
REPEAT_COST = 0.5
#: Подход ступенью лада (целый тон к следующей ноте) вместо полутона — когда оба полутона трутся о лид.
STEP_APPROACH_COST = 1.0
#: Замена ноты — тон аккорда такта не дальше этого от ноты генератора (бас не прыгает по регистру).
SWAP_RANGE = 7


def _overlaps(a: PitchEvent, b: PitchEvent) -> bool:
    return a.beat < b.beat + b.dur_beats - 1e-6 and b.beat < a.beat + a.dur_beats - 1e-6


def _is_approach(notes: Sequence[PitchEvent], i: int) -> bool:
    """Нота ``i`` — подход: полутон к следующей ноте, встык."""
    return (i + 1 < len(notes) and abs(notes[i].midi - notes[i + 1].midi) == 1
            and abs(notes[i].beat + notes[i].dur_beats - notes[i + 1].beat) < 1e-6)


def _candidates(notes: Sequence[PitchEvent], approach: Sequence[bool], pcs: Mapping[int, Sequence[int]],
                scale: frozenset, register: Tuple[int, int]) -> List[List[int]]:
    """Кандидаты нот (с конца: подход строится от кандидатов следующей) — нота генератора первой."""
    lo, hi = register
    cands: List[List[int]] = [[] for _ in notes]
    for i in reversed(range(len(notes))):
        e = notes[i]
        if approach[i]:
            options = {f + s for f in cands[i + 1] for s in (-1, 1)}
            options |= {f + s for f in cands[i + 1] for s in (-2, 2) if (f + s) % 12 in scale}
        else:
            tones = pcs.get(int(e.beat // BEATS_PER_BAR + 1e-9), ())
            options = {m for m in range(e.midi - SWAP_RANGE, e.midi + SWAP_RANGE + 1) if m % 12 in tones}
        cands[i] = sorted({e.midi} | {m for m in options if lo <= m <= hi},
                          key=lambda m, e=e: (m != e.midi, abs(m - e.midi), m))
    return cands


def _lead_moves(notes: Sequence[PitchEvent], lead: Sequence[PitchEvent]
                ) -> Dict[int, List[Tuple[Tuple[int, ...], Tuple[int, ...]]]]:
    """Соседние онсеты лида не дальше доли, между которыми бас сменил ноту: индекс новой ноты баса → (высоты лида
    прошлого онсета, высоты текущего)."""
    at: Dict[float, List[int]] = collections.defaultdict(list)
    for n in lead:
        at[n.beat].append(n.midi)
    starts = [e.beat for e in notes]

    def bass_at(t: float) -> Optional[int]:
        i = bisect.bisect_right(starts, t + 1e-6) - 1
        return i if i >= 0 and t < notes[i].beat + notes[i].dur_beats - 1e-6 else None

    onsets = sorted(at)
    moves: Dict[int, List[Tuple[Tuple[int, ...], Tuple[int, ...]]]] = collections.defaultdict(list)
    for t0, t1 in zip(onsets, onsets[1:]):
        j0, j1 = bass_at(t0), bass_at(t1)
        if j0 is not None and j1 == j0 + 1 and t1 - t0 <= 1.0 + 1e-6:
            moves[j1].append((tuple(at[t0]), tuple(at[t1])))
    return moves


def _parallels(moves: Sequence[Tuple[Tuple[int, ...], Tuple[int, ...]]], p: int, m: int, parallels: frozenset) -> int:
    """Параллелей бас ``p → m`` с лидом на интервалах ``parallels``: оба сдвинулись в одну сторону, интервал тот же."""
    return sum(1 for before, after in moves for a in before for b in after
               if (b - a) * (m - p) > 0 and (a - p) % 12 == (b - m) % 12 in parallels)


def _step_cost(p: int, m: int, approach: bool, scale: frozenset) -> float:
    """Форма линии ``p → m``: скачок и повтор; после подхода — только полутон или ступень лада (иначе ∞)."""
    cost = (LEAP_COST if abs(m - p) > LEAP_MAX else 0.0) + (REPEAT_COST if m == p else 0.0)
    if approach and abs(m - p) != 1:
        if abs(m - p) != 2 or p % 12 not in scale:
            return math.inf
        cost += STEP_APPROACH_COST
    return cost


def _clash_costs(notes: Sequence[PitchEvent], cands: Sequence[Sequence[int]], lead: Sequence[PitchEvent],
                 clash: frozenset) -> List[Dict[int, float]]:
    """Цена кандидата ноты: замена (дальше — дороже) + :data:`CLASH_COST` за каждую ноту лида, звучащую вместе с ней
    на интервале ``clash``."""
    out: List[Dict[int, float]] = []
    for e, options in zip(notes, cands):
        over = [n.midi for n in lead if _overlaps(n, e)]
        out.append({m: (0.0 if m == e.midi else CHANGE_COST + CHANGE_COST_PER_SEMITONE * abs(m - e.midi))
                    + CLASH_COST * sum((n - m) % 12 in clash for n in over) for m in options})
    return out


def _viterbi(cands: Sequence[Sequence[int]], unary, pair) -> Optional[List[int]]:
    """Путь наименьшей цены по кандидатам; все пути бесконечны — ``None``."""
    best = [{m: (unary(0, m), None) for m in cands[0]}]
    for i in range(1, len(cands)):
        row = {}
        for m in cands[i]:
            p, c = min(((p, c + pair(i, p, m)) for p, (c, _b) in best[-1].items()), key=lambda x: x[1])
            row[m] = (c + unary(i, m), p)
        best.append(row)
    m = min(best[-1], key=lambda k: best[-1][k][0])
    if math.isinf(best[-1][m][0]):
        return None
    line = [m]
    for i in range(len(cands) - 1, 0, -1):
        m = best[i][m][1]
        line.append(m)
    return line[::-1]


def against_lead(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], part: Part,
                 lead: Sequence[PitchEvent]) -> Part:
    """Бас ``part`` с нотами, переставленными против лида ``lead`` (правила стиля ``bass_lead_clash``/
    ``bass_lead_parallels``; пусто — партия как есть).

    Витерби по нотам баса: кандидаты ноты — нота генератора и тоны аккорда такта в регистре не дальше
    :data:`SWAP_RANGE`; нота-подход (полутон к следующей ноте встык) остаётся подходом к выбранной следующей —
    полутоном или, дороже (:data:`STEP_APPROACH_COST`), ступенью лада. Цена — :data:`CLASH_COST` за каждую ноту лида,
    звучащую вместе на запретном интервале, :data:`PARALLEL_COST` за параллель (соседние онсеты лида, бас и лид
    сдвинулись в одну сторону, интервал до и после — запретный), плюс замена и форма линии. Ритм, длины и акценты —
    генератора; не нашлось чистого — меньшее зло."""
    clash, parallels = frozenset(style.bass_lead_clash), frozenset(style.bass_lead_parallels)
    notes = sorted(part.pitches or (), key=lambda e: e.beat)
    if not (clash or parallels) or not notes or not lead or len({e.beat for e in notes}) != len(notes):
        return part
    scale = kn.scale_pitch_classes(key.root, key.mode)
    approach = [_is_approach(notes, i) for i in range(len(notes))]
    cands = _candidates(notes, approach, {bar: _tones(style, key, c) for bar, c in bar_chords}, scale, part.register)
    unary = _clash_costs(notes, cands, lead, clash)
    moves = _lead_moves(notes, lead)

    def pair(i: int, p: int, m: int) -> float:
        return _step_cost(p, m, approach[i - 1], scale) + PARALLEL_COST * _parallels(moves.get(i, ()), p, m, parallels)

    line = _viterbi(cands, lambda i, m: unary[i][m], pair)
    if line is None:
        return part
    return replace(part, pitches=tuple(replace(e, midi=m) for e, m in zip(notes, line)))


__all__ = ["ANCHORS", "BassTone", "RIFF_EIGHTHS", "ROOT_TONE", "WALK_TONES", "acid16", "against_lead", "bar_notes", "broken",
           "figure_bass", "material_tones", "note_beats", "octave8", "offbeat", "perfect_fifth", "riff", "rolling8",
           "walking"]
