"""Бас: фигуры баса — генераторы ``(style, key, bar_chords, synth, register) -> Part`` (ADR-0153 §2.2).

``offbeat`` — 4 ноты на такт в оффбит, тоника/квинта аккорда (ADR-0149 §3.5, research 4.1).
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, Part, PitchEvent
from . import harmony, rhythm
from .rhythm import OFFBEAT_STEPS

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
NOTE_BEATS = 0.5  # половина доли: нота баса кончается до следующей бочки


def bar_notes(root_pc: int, fifth_pc: int, register: Tuple[int, int]) -> Tuple[int, ...]:
    """Тоника ×3 и квинта аккорда — ближайшая к тонике внутри регистра."""
    root = next(m for m in range(register[0], register[1] + 1) if m % 12 == root_pc)
    up = root + (fifth_pc - root) % 12
    fifth = up if up <= register[1] else up - 12
    if fifth < register[0]:
        fifth = root
    return (root, root, root, fifth)


def offbeat_bass(bars: Sequence[Tuple[int, Tuple[int, ...]]], register: Tuple[int, int]) -> Tuple[PitchEvent, ...]:
    """``bars`` — (номер такта формы, трезвучие аккорда в pitch class); ноты на «и» каждой доли."""
    out: List[PitchEvent] = []
    for bar, pcs in bars:
        for step, midi in zip(OFFBEAT_STEPS, bar_notes(pcs[0], pcs[2], register)):
            accent = 3 if step == OFFBEAT_STEPS[0] else 2
            out.append(PitchEvent(midi, bar * BEATS_PER_BAR + step * STEP_BEATS, NOTE_BEATS, accent))
    return tuple(out)


def offbeat(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, Chord]], synth: str,
            register: Tuple[int, int]) -> Part:
    """Бас в оффбит по тактам формы ``bar_chords`` — (такт, аккорд); уровень ставит ``arrange.mix``."""
    bars = [(bar, harmony.chord_pcs(style, key, chord.degree)) for bar, chord in bar_chords]
    return Part("bass", synth, rhythm.grid(OFFBEAT_STEPS), offbeat_bass(bars, register), 0.0, register)


__all__ = ["NOTE_BEATS", "bar_notes", "offbeat", "offbeat_bass"]
