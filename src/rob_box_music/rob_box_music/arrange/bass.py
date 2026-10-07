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

Тон такта (ADR-0154 §3.3, PR-4): без материала — тоника ×(n−1) + квинта, как всегда. С материалом партитуры —
:func:`material_tones`: такт стоит на том тоне аккорда, на котором стоит басовый голос автора (обращение — терция,
квинта), последняя нота такта — подход полутоном к следующему, если он есть у автора; политика — выученная таблица
``knowledge.BASS_TONES`` (тон по умолчанию, потолок подходов на петлю). Шаги рисунка те же — бас мимо бочки.
"""

from __future__ import annotations

import bisect
import collections
from dataclasses import replace
from typing import Dict, List, Mapping, NamedTuple, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..material import Phrase, ScoreMaterial
from ..model import APPROACH_MAX_BEATS, BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import harmony, rhythm

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
STEPS_PER_BEAT = STEPS_PER_BAR // BEATS_PER_BAR


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
    """``count`` нот такта: тоника, последняя — квинта аккорда (``fifth_last``), ближайшая к тонике внутри регистра."""
    root = next(m for m in range(register[0], register[1] + 1) if m % 12 == root_pc)
    up = root + (fifth_pc - root) % 12
    fifth = up if up <= register[1] else up - 12
    if fifth < register[0] or not fifth_last:
        fifth = root
    return (root,) * (count - 1) + (fifth,)


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
        notes = bar_notes(pcs[anchor], pcs[0] if anchor == ANCHORS["fifth"] else pcs[2], register, len(steps),
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


def material_tones(style: kn.Style, material: ScoreMaterial, phrase: Phrase, degrees: Sequence[int],
                   chord_beats: float, scale: float = 1.0) -> Tuple[BassTone, ...]:
    """Тон каждого такта петли ``degrees`` (аккорд держится ``chord_beats`` долей) по басовому голосу материала
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
        pcs = harmony.chord_pcs(style, material.key, degrees[int(b * BEATS_PER_BAR // chord_beats)])[:3]
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
    bars = [(bar, harmony.chord_pcs(style, key, chord.degree)) for bar, chord in bar_chords]
    return Part("bass", synth, rhythm.grid(figure.steps), figure_bass(figure, bars, register, tones), 0.0, register)


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
    bars = [(bar, harmony.chord_pcs(style, key, chord.degree)) for bar, chord in bar_chords]
    sounding = {bar for bar, _pcs in bars}
    firsts: Dict[int, int] = {}
    prev = register[0] + 6
    for bar, pcs in bars:
        prev = firsts[bar] = _near(pcs[(tones or {}).get(bar, ROOT_TONE).anchor], prev, register)
    out: List[PitchEvent] = []
    for bar, pcs in bars:
        line = [firsts[bar]]
        for idx in WALK_TONES[bar % 2]:
            line.append(_near(pcs[idx % len(pcs)], line[-1], register))
        nxt = firsts.get(bar + 1) if bar + 1 in sounding else None
        line.append(_approach(nxt, line[-1], register) if nxt is not None else _near(pcs[2], line[-1], register))
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


__all__ = ["ANCHORS", "BassTone", "ROOT_TONE", "WALK_TONES", "acid16", "bar_notes", "broken", "figure_bass",
           "material_tones", "note_beats", "octave8", "offbeat", "rolling8", "walking"]
