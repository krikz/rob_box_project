"""Бас: фигуры баса — генераторы ``(style, key, bar_chords, synth, register) -> Part`` (ADR-0153 §2.2).

Рисунки — ``knowledge.BASS_FIGURES`` (ADR-0152 §3.3), генератор выбирается по ключу из реестра
``arrange.compose.BASS_GENERATORS``:

* ``offbeat`` — 4 ноты на такт в оффбит, тоника/квинта аккорда (ADR-0149 §3.5, research 4.1).
* ``rolling8`` — 8 нот на такт тоникой, «и» и «а» каждой доли.
* ``broken`` — 4 ноты мимо шагов ломаной бочки ``breakbeat`` (окно ``breaks``, ADR-0152 PR-8).
* ``acid16`` — 12 16-х на такт мимо долей с акцентами, октавными прыжками и срезом фильтра на каждую ноту; только
  ``tb303`` семьи ``hard`` (``knowledge.BASS_FIGURE_SYNTHS``, ADR-0152 PR-9).

Все комплементарны бочке (прямой или ломаной): ни одной ноты на доле, нота кончается к доле (ADR-0149 §3.4).
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import harmony, rhythm

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
STEPS_PER_BEAT = STEPS_PER_BAR // BEATS_PER_BAR


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
                register: Tuple[int, int]) -> Tuple[PitchEvent, ...]:
    """``bars`` — (номер такта формы, трезвучие аккорда в pitch class); ноты на шагах рисунка ``figure``."""
    steps = figure.steps
    out: List[PitchEvent] = []
    for bar, pcs in bars:
        notes = bar_notes(pcs[0], pcs[2], register, len(steps), figure.fifth_last)
        for i, (step, midi) in enumerate(zip(steps, notes)):
            accent = figure.accents[i] if figure.accents else 3 if i == 0 else 2
            lifted = midi + 12 if i in figure.lift and midi + 12 <= register[1] else midi
            lpf = figure.lpf[(bar % 2) * len(steps) + i] if figure.lpf else 0.0
            out.append(PitchEvent(lifted, bar * BEATS_PER_BAR + step * STEP_BEATS, note_beats(steps, i), accent, lpf))
    return tuple(out)


def _part(name: str, style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
          register: Tuple[int, int]) -> Part:
    figure = kn.BASS_FIGURES[name]
    bars = [(bar, harmony.chord_pcs(style, key, chord.degree)) for bar, chord in bar_chords]
    return Part("bass", synth, rhythm.grid(figure.steps), figure_bass(figure, bars, register), 0.0, register)


def offbeat(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int]) -> Part:
    """Бас в оффбит по тактам формы ``bar_chords`` — (такт, аккорд); уровень ставит ``arrange.mix``."""
    return _part("offbeat", style, key, bar_chords, synth, register)


def rolling8(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
             register: Tuple[int, int]) -> Part:
    """Ролл тоникой на «и» и «а» каждой доли по тактам формы ``bar_chords``; уровень ставит ``arrange.mix``."""
    return _part("rolling8", style, key, bar_chords, synth, register)


def acid16(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
           register: Tuple[int, int]) -> Part:
    """Кислотная линия ``tb303``: 12 16-х на такт мимо долей, акцент первой 16-й доли, последняя 16-я доли — октавой
    выше; у каждой ноты свой срез фильтра (``PitchEvent.lpf``), волна в два такта; уровень ставит ``arrange.mix``."""
    return _part("acid16", style, key, bar_chords, synth, register)


def broken(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
           register: Tuple[int, int]) -> Part:
    """Бас в обход ломаной бочки (``breaks``): тоника ×3 + квинта на шагах 1, 6, 9, 14, мимо шагов
    рисунка ``breakbeat``; уровень ставит ``arrange.mix``."""
    return _part("broken", style, key, bar_chords, synth, register)


__all__ = ["acid16", "bar_notes", "broken", "figure_bass", "note_beats", "offbeat", "rolling8"]
