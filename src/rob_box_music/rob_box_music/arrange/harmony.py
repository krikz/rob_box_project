"""Гармония: трезвучия лада и голосоведение пэда (ADR-0149 §3.6, research 4.2)."""

from __future__ import annotations

import itertools
from typing import List, Sequence, Tuple

from .. import knowledge as kn
from ..model import Chord, Key

#: Прогрессии club по ступеням лада, аккорд на 2 такта (8-тактовая петля).
PROGRESSIONS: Tuple[Tuple[int, ...], ...] = ((0, 5, 2, 6), (0, 3, 5, 4), (0, 5, 3, 4), (0, 6, 5, 6))


def triad_pcs(key: Key, degree: int) -> Tuple[int, ...]:
    """Трезвучие ступени ``degree`` из звуков лада (терции лада, не хроматика): (тоника, терция, квинта)."""
    scale = kn.SCALES[key.mode]
    return tuple((key.root + scale[(degree + k) % len(scale)]) % 12 for k in (0, 2, 4))


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


def pad_chords(key: Key, degrees: Sequence[int], register: Tuple[int, int]) -> Tuple[Chord, ...]:
    """Петля прогрессии → обращения с минимальным движением голосов, включая стык «последний → первый».

    Цепочка «каждый от предыдущего» уплывает, и на повторе петли пэд прыгает (до 17 полутонов у
    тестов PR-2); поэтому перебор всех обращений петли (4 аккорда × ≤ 9 вариантов) по сумме
    движения по кругу, при равенстве — ближе к середине регистра.
    """
    options = [voicings(triad_pcs(key, d), register) for d in degrees]
    if not all(options):
        raise ValueError(f"прогрессия {tuple(degrees)} не помещается в регистр {register}")
    mid = sum(register) / 2

    def cost(combo: Tuple[Tuple[int, ...], ...]) -> Tuple[int, float]:
        ring = sum(_movement(combo[i - 1], combo[i]) for i in range(len(combo)))
        return ring, sum(abs(sum(v) / len(v) - mid) for v in combo)

    best = min(itertools.product(*options), key=lambda combo: (cost(combo), combo))
    return tuple(Chord(d, v) for d, v in zip(degrees, best))


__all__ = ["PROGRESSIONS", "pad_chords", "triad_pcs", "voicings"]
