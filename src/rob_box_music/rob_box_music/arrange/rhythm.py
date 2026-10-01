"""Ритм-сетки 16 шагов × такт (ADR-0149 §3.4): жанр решает бочку, хэты в оффбит."""

from __future__ import annotations

from typing import Iterable, Mapping, Optional

from .. import knowledge as kn
from ..model import STEPS_PER_BAR, Grid, Step

#: Шаги «и» каждой доли: оффбит для хэтов и баса при прямой бочке (research 4.1).
OFFBEAT_STEPS = (2, 6, 10, 14)
#: Клэп на 2 и 4.
BACKBEAT_STEPS = (4, 12)


def grid(on_steps: Iterable[int], length: int = STEPS_PER_BAR, accents: Optional[Mapping[int, int]] = None) -> Grid:
    """Сетка длиной ``length`` шагов; ``accents`` — акцент 0..3 по шагу (по умолчанию 2)."""
    on = set(on_steps)
    acc = accents or {}
    return Grid(tuple(Step(i in on, acc.get(i, 2) if i in on else 0) for i in range(length)))


def kick_grid(kick: str) -> Grid:
    """Бочка из ``knowledge.KICK_PATTERNS``; доли 1 и 3 — с акцентом 3."""
    pattern = kn.KICK_PATTERNS[kick]
    if len(pattern) != STEPS_PER_BAR:
        raise ValueError(f"рисунок бочки {kick!r} не 16 простых шагов")
    return grid((i for i, ch in enumerate(pattern) if ch == "X"), accents={0: 3, 8: 3})


def hats_grid() -> Grid:
    """Оффбит-хэт; «и» второй и четвёртой доли тише — качание, а не метроном."""
    return grid(OFFBEAT_STEPS, accents={2: 3, 6: 2, 10: 3, 14: 2})


def clap_grid() -> Grid:
    return grid(BACKBEAT_STEPS, accents={4: 3, 12: 3})


__all__ = ["BACKBEAT_STEPS", "OFFBEAT_STEPS", "clap_grid", "grid", "hats_grid", "kick_grid"]
