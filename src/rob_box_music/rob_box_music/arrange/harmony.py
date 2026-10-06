"""Гармония: аккорды лада и голосоведение пэда (ADR-0149 §3.6, research 4.2); пул прогрессий и размер аккорда —
из стиля (``knowledge.Style``, ADR-0153 §2.2)."""

from __future__ import annotations

import itertools
import random
from typing import List, Sequence, Tuple

from .. import knowledge as kn
from ..diversity import weighted_pick
from ..model import Chord, Key, PitchEvent

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


__all__ = ["PROGRESSION_CAP", "PROGRESSION_LOOKBACK", "PROGRESSION_SKIPPED", "PROGRESSION_WINDOW", "chord_pcs", "fit_progression", "pad_chords", "progression_name",
           "voicings"]
