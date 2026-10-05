"""Пэд: фигуры аккомпанемента — генераторы ``(style, key, bar_chords, synth, register) -> Part`` (ADR-0153 §2.2).

``pumped16`` — аккорд на каждой 16-й (``sus`` — шаг): сайдчейн-огибающая рендера живёт только на событиях, поэтому
клубный пэд звучит под ней каждой 16-й (ADR-0149 §3.6, §3.8). Обращения аккордов — ``harmony.pad_chords``.
"""

from __future__ import annotations

from typing import Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import rhythm

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR


def pumped16(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
             register: Tuple[int, int]) -> Part:
    """Аккорд такта на каждой 16-й по тактам формы ``bar_chords`` — (такт, аккорд); уровень ставит ``arrange.mix``."""
    events = tuple(
        PitchEvent(m, bar * BEATS_PER_BAR + step * STEP_BEATS, STEP_BEATS, 3)
        for bar, chord in bar_chords for step in range(STEPS_PER_BAR) for m in chord.voicing
    )
    return Part("pad", synth, rhythm.grid(range(STEPS_PER_BAR)), events, 0.0, register)


__all__ = ["pumped16"]
