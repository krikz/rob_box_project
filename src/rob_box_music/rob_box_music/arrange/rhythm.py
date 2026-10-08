"""Ритм-сетки 16 шагов × такт (ADR-0149 §3.4): стиль решает бочку, хэты в оффбит, свинг, fill-ы.

Fill — последний такт секции с ``Section.fill_last_bar`` (8-тактовая фраза перед дропом): ролл клэпа
16-ми на третьей и четвёртой долях с нарастающим акцентом, бочка снимается на последней доле. Свинг —
``offset_ms`` ударов и нот на шагах ``Style.swing_steps`` (:func:`swing`, :func:`swing_part`): у клуба — нечётные 16-е
хэтов (гоуст-ноты), сильные доли и бочка не сдвигаются (сайдчейн, research 1.4); у lo-fi (ADR-0153 S4) — оффбит-восьмые
хэтов и ролей ``Style.swing_roles`` (бочка, малый, бас, пэд, лид — грув один на всех).
"""

from __future__ import annotations

from dataclasses import replace
from typing import Callable, Iterable, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Grid, Part, PitchEvent, Section, Step

#: Шаги «и» каждой доли: оффбит для хэтов и баса при прямой бочке (research 4.1).
OFFBEAT_STEPS = (2, 6, 10, 14)
#: Клэп на 2 и 4.
BACKBEAT_STEPS = (4, 12)
#: Гоуст-хэты на последней 16-й доли 2 и 4 — нечётные шаги, их и качает свинг.
GHOST_HAT_STEPS = (7, 15)
#: Ролл клэпа в такте fill-а: 16-е третьей и четвёртой долей, акцент растёт к дропу.
ROLL_STEPS = tuple(range(8, 16))
ROLL_ACCENTS = (1, 1, 2, 2, 2, 3, 3, 3)
#: С этого шага такта fill-а бочка молчит: последняя доля перед дропом.
KICK_CUT_STEP = 12
#: Ролл клэпа перед дропом, такты: предпоследний — восьмые, последний — 16-е; акцент растёт к дропу.
ROLL_BARS = 2
_ROLL_EIGHTHS = {step: 1 for step in range(0, STEPS_PER_BAR, 2)}
_ROLL_SIXTEENTHS = {step: (1, 2, 2, 3)[step // 4] for step in range(STEPS_PER_BAR)}


def grid(on_steps: Iterable[int], length: int = STEPS_PER_BAR, accents: Optional[Mapping[int, int]] = None) -> Grid:
    """Сетка длиной ``length`` шагов; ``accents`` — акцент 0..3 по шагу (по умолчанию 2)."""
    on = set(on_steps)
    acc = accents or {}
    return Grid(tuple(Step(i in on, acc.get(i, 2) if i in on else 0) for i in range(length)))


def kick_grid(pattern: str) -> Grid:
    """Такт бочки по рисунку 16 шагов (``knowledge.KICK_PATTERNS``, вид секции ``Style.looks``); доли 1 и 3 —
    акцент 3."""
    if len(pattern) != STEPS_PER_BAR or set(pattern) - {"X", "."}:
        raise ValueError(f"рисунок бочки {pattern!r} не 16 простых шагов")
    return grid((i for i, ch in enumerate(pattern) if ch == "X"), accents={0: 3, 8: 3})


def swing_offset_ms(swing: float, bpm: int) -> int:
    """Опоздание нечётной 16-й: ``swing`` доли восьмой при темпе ``bpm``, мс."""
    return int(round(swing * 30000.0 / bpm))


#: Символ рисунка каркаса (``Style.kits``) → акцент; ``.`` — пауза.
_KIT_ACCENT = {"X": 3, "x": 2, "g": 0}


def pattern_grid(pattern: str) -> Grid:
    """Такт по рисунку каркаса: ``X``/``x``/``g`` — акцент 3/2/0; без свинга (его кладёт :func:`swing`)."""
    if len(pattern) != STEPS_PER_BAR or set(pattern) - set(_KIT_ACCENT) - {"."}:
        raise ValueError(f"рисунок каркаса {pattern!r} не 16 шагов из X x g .")
    return Grid(tuple(Step(False) if ch == "." else Step(True, _KIT_ACCENT[ch]) for ch in pattern))


def swing(grid: Grid, swing_ms: int, steps: Sequence[int]) -> Grid:
    """Удары сетки на шагах такта ``steps`` (``Style.swing_steps``) опаздывают на ``swing_ms``; остальные — как были."""
    on = frozenset(steps)
    return Grid(tuple(replace(st, offset_ms=swing_ms) if st.on and i % STEPS_PER_BAR in on else st
                      for i, st in enumerate(grid.steps)))


def _step_in_bar(ev: PitchEvent) -> Optional[int]:
    """Шаг 16-х такта ноты; нота вне сетки 16-х — ``None`` (её свинг не трогает)."""
    step = ev.beat * STEPS_PER_BAR / BEATS_PER_BAR
    return int(round(step)) % STEPS_PER_BAR if abs(step - round(step)) < 1e-9 else None


def swing_pitches(events: Sequence[PitchEvent], swing_ms: int, steps: Sequence[int]) -> Tuple[PitchEvent, ...]:
    """Ноты на шагах такта ``steps`` опаздывают на ``swing_ms`` (``PitchEvent.offset_ms``); доли модели не меняются."""
    on = frozenset(steps)
    return tuple(replace(e, offset_ms=swing_ms) if _step_in_bar(e) in on else e for e in events)


def swing_part(part: Part, swing_ms: int, steps: Sequence[int]) -> Part:
    """Партия со свингом (``Style.swing_roles``): сетка ударов и ноты — на шагах ``steps``."""
    pitches = None if part.pitches is None else swing_pitches(part.pitches, swing_ms, steps)
    return replace(part, grid=swing(part.grid, swing_ms, steps), pitches=pitches)


def hats_grid(style: kn.Style, swing_ms: int = 0, kit: str = "offbeat") -> Grid:
    """Хэты каркаса ``style.kits``: удары на шагах ``style.swing_steps`` опаздывают на ``swing_ms`` (у клуба —
    гоуст-ноты на нечётных 16-х: других ударов там в каркасах нет)."""
    return swing(pattern_grid(style.kits[kit]["hats"]), swing_ms, style.swing_steps)


def clap_grid() -> Grid:
    return grid(BACKBEAT_STEPS, accents={4: 3, 12: 3})


def clap_fill(bar: Grid) -> Grid:
    """Такт fill-а: первая половина такта ``bar`` как есть, вторая — ролл 16-ми с нарастающим акцентом."""
    head = bar.steps[:ROLL_STEPS[0]]
    return Grid(head + tuple(Step(True, a) for a in ROLL_ACCENTS))


def clap_roll(bar_no: int) -> Grid:
    """Такт ``bar_no`` (0..``ROLL_BARS`` − 1) ролла перед дропом: восьмые, затем 16-е."""
    accents = _ROLL_EIGHTHS if bar_no < ROLL_BARS - 1 else _ROLL_SIXTEENTHS
    return grid(accents, accents=accents)


def kick_fill(bar: Grid) -> Grid:
    """Такт fill-а бочки: последняя доля перед дропом пустая."""
    return Grid(tuple(Step(False) if i >= KICK_CUT_STEP else st for i, st in enumerate(bar.steps)))


#: Такт fill-а малого по ключу ``Style.clap_fill``: ``roll`` — ролл 16-ми второй половины (клуб), ``cut`` — малый
#: молчит с последней доли, как бочка (рок: там филл томами, ADR-0153 S5).
CLAP_FILLS: Mapping[str, Callable[[Grid], Grid]] = {"roll": clap_fill, "cut": kick_fill}


def tom_fill(pattern: str) -> Grid:
    """Такт филла томами по рисунку 16 шагов (``Style.tom_fills``): ``X``/``x`` — акцент 3/2, ``.`` — пауза."""
    if len(pattern) != STEPS_PER_BAR or set(pattern) - {"X", "x", "."}:
        raise ValueError(f"филл томами {pattern!r} не 16 шагов из X x .")
    return grid((i for i, ch in enumerate(pattern) if ch != "."),
                accents={i: _KIT_ACCENT[ch] for i, ch in enumerate(pattern) if ch != "."})


def form_grid(sections: Sequence[Section], bar_of: Callable[[Section, bool], Grid]) -> Grid:
    """Сетка на всю форму: ``bar_of(секция, это такт fill-а)`` даёт такт (16 шагов)."""
    steps = []
    for sec in sections:
        for i in range(sec.bars):
            steps += bar_of(sec, sec.fill_last_bar and i == sec.bars - 1).steps
    return Grid(tuple(steps))


def form_bars(sections: Sequence[Section], bar_of: Callable[[Section, int], Grid]) -> Grid:
    """Сетка на всю форму: ``bar_of(секция, тактов до конца секции, считая этот)`` даёт такт (16 шагов)."""
    steps = []
    for sec in sections:
        for i in range(sec.bars):
            steps += bar_of(sec, sec.bars - i).steps
    return Grid(tuple(steps))


__all__ = ["CLAP_FILLS", "tom_fill", "BACKBEAT_STEPS", "GHOST_HAT_STEPS", "KICK_CUT_STEP", "OFFBEAT_STEPS", "ROLL_ACCENTS", "ROLL_BARS",
           "ROLL_STEPS", "clap_fill", "clap_grid", "clap_roll", "form_bars", "form_grid", "grid", "hats_grid",
           "kick_fill", "kick_grid", "pattern_grid", "swing", "swing_offset_ms", "swing_part", "swing_pitches"]
