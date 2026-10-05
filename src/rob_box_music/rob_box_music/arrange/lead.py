"""Лид: мотив «вопрос 2 такта / ответ 2 такта» из пула 4 нот (ADR-0149 §3.7, research 1.1, 4.3).

Пул — VII ступень под тоникой и тоническое трезвучие лада (I, III, V) в верхней части регистра лида; контур
один на весь трек и под аккорды не переназначается. Вопрос кончается не на тонике, ответ —
на тонике. Во втором такте каждой половины лид молчит: паузы, а не блуждание по 16-м.
"""

from __future__ import annotations

import random
from typing import List, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Key, PitchEvent

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
MOTIF_BARS = 4
#: Ритмы первого такта фразы (шаги 16-х): 3–4 ноты, с синкопой.
RHYTHMS: Tuple[Tuple[int, ...], ...] = ((0, 3, 6, 10), (0, 3, 8, 11), (2, 6, 10), (0, 6, 10, 12))
HOLD_LAST_BEATS = 2.0  # последняя нота фразы тянется в тишину второго такта


def pool(key: Key, register: Tuple[int, int]) -> Tuple[int, ...]:
    """4 ноты: тоника, III, V и VII под тоникой; тоника — в верхней октаве, где квинта ещё в регистре."""
    scale = kn.SCALES[key.mode]
    top = register[1] - scale[4]
    tonic = next(m for m in range(top - 11, top + 1) if m % 12 == key.root)
    return (tonic, tonic + scale[2], tonic + scale[4], tonic + scale[-1] - 12)


def motif(style: kn.Style, key: Key, rng: random.Random) -> Tuple[PitchEvent, ...]:
    """Мотив на ``MOTIF_BARS`` тактов в регистре лида стиля, доли от начала мотива."""
    notes = pool(key, style.registers["lead"])
    rhythm = rng.choice(RHYTHMS)
    contour = [rng.choice(notes) for _ in rhythm[:-1]]
    question = contour + [rng.choice(notes[1:])]
    answer = contour + [notes[0]]
    out: List[PitchEvent] = []
    for half, phrase in ((0, question), (2, answer)):
        for i, (step, midi) in enumerate(zip(rhythm, phrase)):
            last = i == len(rhythm) - 1
            gap = (rhythm[i + 1] - step) * STEP_BEATS if not last else HOLD_LAST_BEATS
            dur = min(gap, 0.75) if not last else gap
            accent = 3 if i == 0 else (2 if not last else 1)
            out.append(PitchEvent(midi, half * BEATS_PER_BAR + step * STEP_BEATS, dur, accent))
    return tuple(out)


__all__ = ["MOTIF_BARS", "RHYTHMS", "motif", "pool"]
