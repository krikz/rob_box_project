"""Лид: мотив «вопрос 2 такта / ответ 2 такта» из пула 4 нот (ADR-0149 §3.7, research 1.1, 4.3).

Пул — VII ступень под тоникой и тоническое трезвучие лада (I, III, V) в верхней части регистра лида; контур
один на весь трек и под аккорды не переназначается. Вопрос кончается не на тонике, ответ —
на тонике. Во втором такте каждой половины лид молчит: паузы, а не блуждание по 16-м.
"""

from __future__ import annotations

import random
from typing import Dict, List, Mapping, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Key, PitchEvent
from . import harmony

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


# ── Соло по сменам (ADR-0153 S6, джаз) ───────────────────────────────────────────────────────────────────────────

#: 16-х в доле: нота на доле (``шаг % BEAT_STEPS == 0``) — сильная, солист берёт тон аккорда такта.
BEAT_STEPS = STEPS_PER_BAR // BEATS_PER_BAR
#: Фраза соло — 2 такта (``Style.solo_phrases``).
PHRASE_BARS = 2
#: Контур: нота на доле тянется на столько полутонов от прошлой ноты в сторону движения линии.
CONTOUR_STEP = 3
#: Скачок соло — не больше октавы (между соседними нотами).
MAX_LEAP = 12
#: Смена направления линии в начале фразы — вероятность (у краёв регистра — всегда к середине).
TURN_P = 0.3


def _phrase_notes(style: kn.Style, bars: Sequence[int], rng: random.Random) -> List[Tuple[int, int, bool]]:
    """(шаг 16-х от начала формы, длина в 16-х, последняя нота фразы) — фразы ``style.solo_phrases`` подряд по
    :data:`PHRASE_BARS` такта от первого такта ``bars``; нота не выходит за последний такт."""
    end = (bars[-1] + 1) * STEPS_PER_BAR
    out: List[Tuple[int, int, bool]] = []
    for start in range(bars[0], bars[-1] + 1, PHRASE_BARS):
        phrase = [(start * STEPS_PER_BAR + step, length) for step, length in rng.choice(style.solo_phrases)
                  if start * STEPS_PER_BAR + step < end]
        out += [(at, min(length, end - at), i == len(phrase) - 1) for i, (at, length) in enumerate(phrase)]
    return out


def _chord_tone(pcs: Sequence[int], prev: int, aim: int, register: Tuple[int, int]) -> int:
    """Тон аккорда ``pcs`` в регистре не дальше октавы от ``prev``, ближайший к ``aim`` (повтор — последним)."""
    options = [m for m in range(register[0], register[1] + 1) if m % 12 in pcs and abs(m - prev) <= MAX_LEAP]
    return min(options, key=lambda m: (m == prev, abs(m - aim), m))


def _scale_step(scale: Sequence[int], midi: int, direction: int, register: Tuple[int, int]) -> int:
    """Соседний звук лада от ``midi`` в сторону ``direction`` (у края регистра — в обратную)."""
    for d in (direction, -direction):
        m = midi + d
        while register[0] <= m <= register[1]:
            if m % 12 in scale:
                return m
            m += d
    return midi


class _Soloist:
    """Линия солиста (:func:`solo`): прошлая нота, направление контура и нота на доле, к которой слабая уже подошла."""

    def __init__(self, pcs: Mapping[int, frozenset], scale: Sequence[int], register: Tuple[int, int],
                 rng: random.Random, chromatic: float) -> None:
        self.pcs, self.scale, self.register, self.rng, self.chromatic = pcs, scale, register, rng, chromatic
        self.prev, self.direction = sum(register) // 2, rng.choice((-1, 1))
        self.pending: Dict[int, int] = {}

    def _aim(self) -> int:
        return self.prev + self.direction * CONTOUR_STEP

    def _steer(self, phrase_start: bool) -> None:
        """В начале фразы линия может развернуться; у края регистра — всегда к середине."""
        if phrase_start and self.rng.random() < TURN_P:
            self.direction = -self.direction
        if not self.register[0] + CONTOUR_STEP <= self._aim() <= self.register[1] - CONTOUR_STEP:
            self.direction = -self.direction

    def _approach(self, target: int) -> int:
        """Подход к ``target`` со стороны прошлой ноты: полутоном (доля ``chromatic``) или ступенью лада."""
        side = -1 if self.prev <= target else 1
        lo, hi = self.register
        chromatic = target + side if lo <= target + side <= hi else target - side
        midi = chromatic if self.rng.random() < self.chromatic else _scale_step(self.scale, target, side, self.register)
        return chromatic if midi == target else midi

    def _weak(self, i: int, notes: Sequence[Tuple[int, int, bool]]) -> int:
        """Слабая нота: перед нотой на доле встык — подход к её тону аккорда (он запоминается), иначе ступень лада."""
        at, length, _last = notes[i]
        nxt = notes[i + 1][0] if i + 1 < len(notes) else None
        if nxt is None or nxt != at + length or nxt % BEAT_STEPS:
            return _scale_step(self.scale, self.prev, self.direction, self.register)
        self.pending[i + 1] = _chord_tone(self.pcs[nxt // STEPS_PER_BAR], self.prev, self._aim(), self.register)
        return self._approach(self.pending[i + 1])

    def note(self, i: int, notes: Sequence[Tuple[int, int, bool]]) -> PitchEvent:
        at, length, last = notes[i]
        self._steer(i == 0 or notes[i - 1][2])
        tones = self.pcs[at // STEPS_PER_BAR]
        if i in self.pending:
            midi = self.pending.pop(i)
        elif at % BEAT_STEPS == 0 or last:
            midi = _chord_tone(tones, self.prev, self._aim(), self.register)
        else:
            midi = self._weak(i, notes)
            if abs(midi - self.prev) > MAX_LEAP:
                midi = _chord_tone(tones, self.prev, self.prev, self.register)
        accent = 3 if at % (2 * BEAT_STEPS) == 0 else (2 if at % BEAT_STEPS == 0 else 1)
        if midi != self.prev:
            self.direction = 1 if midi > self.prev else -1
        self.prev = midi
        return PitchEvent(midi, at * STEP_BEATS, length * STEP_BEATS, accent)


def solo(style: kn.Style, key: Key, bar_chords: Sequence[Tuple[int, object]], register: Tuple[int, int],
         rng: random.Random) -> Tuple[PitchEvent, ...]:
    """Импровизация солиста по аккордам тактов ``bar_chords`` — (такт формы, аккорд с ``degree``/``quality``), доли от
    начала формы. Ритм — фразы ``style.solo_phrases`` (2 такта, хвост — пауза). Нота на доле и последняя нота фразы —
    тон аккорда своего такта (``harmony.chord_pcs``: септ/нона стиля), ближайший к контуру линии; слабая нота перед
    нотой на доле — подход к ней: полутоном (доля ``style.solo_chromatic``, хроматика) или ступенью лада; иные слабые —
    ступень лада по направлению. Скачок между соседними нотами ≤ :data:`MAX_LEAP`. Детерминировано ``rng``."""
    chords = dict(bar_chords)
    if not chords or not style.solo_phrases:
        return ()
    pcs = {bar: frozenset(harmony.chord_pcs(style, key, c.degree, c.quality or None)) for bar, c in chords.items()}
    notes = _phrase_notes(style, sorted(chords), rng)
    line = _Soloist(pcs, sorted(kn.scale_pitch_classes(key.root, key.mode)), register, rng, style.solo_chromatic)
    return tuple(line.note(i, notes) for i in range(len(notes)))


__all__ = ["BEAT_STEPS", "MAX_LEAP", "MOTIF_BARS", "PHRASE_BARS", "RHYTHMS", "motif", "pool", "solo"]
