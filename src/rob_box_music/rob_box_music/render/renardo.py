"""``render(track, deck) -> Program`` — одна чистая функция рендера модели в Renardo (ADR-0149 §3.2).

Подмножество Renardo: шесть присваиваний плеерам деки. Ударные — ``play("<сетка>")``
по 16-м, тональные — список MIDI по 16-м (``None`` — пауза, кортеж — аккорд) в
``Scale.chromatic`` с ``root=0, oct=0``. Секции — ``amp=var([...], [доли])`` по
``Section.roles``; акценты — ``amplify=[...]``. Список нот свёрнут до наименьшего
периода в тактах, на котором модель совпадает во всех звучащих секциях, — поэтому
программа короткая, а события нот равны модели (тест на ``render.events``).

Не умеет (честная ошибка :class:`RenderError`, а не тихая потеря): свинг ``offset_ms``,
роли ``sample``/``fx``, ноты вне сетки 16-х, ноту в секции, где роль молчит.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, STEPS_PER_BAR, Part, Track, validate
from .program import Program

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
#: Роль → индекс слота в ``knowledge.DECK_SLOTS[deck]``: d-слоты ударным, p-слоты тональным.
ROLE_SLOT: Dict[str, int] = {"kick": 0, "hats": 1, "clap": 2, "perc": 2, "bass": 3, "pad": 4, "lead": 5}
_UNSET = object()

Cell = Tuple[Optional[Tuple[int, ...]], float, int]  # (ноты шага, sus в долях, акцент)


class RenderError(ValueError):
    """Модель валидна, но рендер её не выражает — громко, до exec (I25)."""


def _num(value: float) -> str:
    rounded = round(float(value), 3)
    return str(int(rounded)) if rounded.is_integer() else f"{rounded:.3f}".rstrip("0").rstrip(".")


def _list(values: Sequence[object]) -> str:
    return "[" + ", ".join(str(v) for v in values) + "]"


def _active_steps(track: Track, role: str) -> List[bool]:
    out: List[bool] = []
    for sec in track.form.sections:
        out += [role in sec.roles] * (sec.bars * STEPS_PER_BAR)
    return out


def _gate(track: Track, role: str, amp: float) -> str:
    """``amp=var`` по секциям: уровень роли там, где она заказана, иначе 0."""
    segments: List[List] = []
    for sec in track.form.sections:
        value = _num(amp) if role in sec.roles else "0"
        if segments and segments[-1][0] == value:
            segments[-1][1] += sec.bars * BEATS_PER_BAR
        else:
            segments.append([value, sec.bars * BEATS_PER_BAR])
    return f"var({_list(v for v, _ in segments)}, {_list(_num(b) for _, b in segments)})"


def _amp(part: Part) -> float:
    return 10.0 ** (part.level_db / 20.0)


def _tail(track: Track, role: str, part: Part, accents: Sequence[int]) -> List[str]:
    opts = [f"amp={_gate(track, role, _amp(part))}"]
    if len(set(accents)) > 1:
        opts.append(f"amplify={_list(_num(kn.ACCENT_AMPLIFY[a]) for a in accents)}")
    pan = track.mix.pan.get(role, 0.0)
    if pan:
        opts.append(f"pan={_num(pan)}")
    return opts


def _drum_line(slot: str, role: str, part: Part, track: Track) -> str:
    if any(st.offset_ms for st in part.grid.steps):
        raise RenderError(f"parts.{role}.grid: свинг offset_ms рендер ещё не выражает (ADR-0149 PR-3)")
    symbol = kn.DRUM_SYMBOLS[role]
    pattern = "".join(symbol if st.on else "." for st in part.grid.steps)
    accents = [st.accent for st in part.grid.steps if st.on]
    full = [st.accent if st.on else 0 for st in part.grid.steps]
    opts = _tail(track, role, part, full if len(set(accents)) > 1 else [0])
    return f'{slot} >> play("{pattern}", dur=1/4, ' + ", ".join(opts) + ")"


def _cells(role: str, part: Part, track: Track) -> List[Optional[Cell]]:
    """Ноты партии по 16-м всей формы; проверка сетки и секций."""
    active = _active_steps(track, role)
    grouped: Dict[int, List] = {}
    for ev in part.pitches or ():
        step = ev.beat / STEP_BEATS
        if abs(step - round(step)) > 1e-9:
            raise RenderError(f"parts.{role}: нота на доле {ev.beat} вне сетки 16-х")
        step = int(round(step))
        if not active[step]:
            raise RenderError(f"parts.{role}: нота на доле {ev.beat} в секции, где роль молчит")
        grouped.setdefault(step, []).append(ev)
    cells: List[Optional[Cell]] = [None] * len(active)
    for step, evs in grouped.items():
        if len({(e.dur_beats, e.accent) for e in evs}) != 1:
            raise RenderError(f"parts.{role}: аккорд на доле {step * STEP_BEATS} с разными sus/акцентом")
        cells[step] = (tuple(sorted(e.midi for e in evs)), evs[0].dur_beats, evs[0].accent)
    return cells


def _fold(cells: Sequence[Optional[Cell]], active: Sequence[bool], bars_total: int) -> List[Optional[Cell]]:
    """Наименьший период в тактах, на котором звучащие шаги модели совпадают."""
    for bars in (b for b in range(1, bars_total + 1) if bars_total % b == 0):
        period = bars * STEPS_PER_BAR
        pattern: List[object] = [_UNSET] * period
        for i, cell in enumerate(cells):
            if not active[i]:
                continue
            if pattern[i % period] is _UNSET:
                pattern[i % period] = cell
            elif pattern[i % period] != cell:
                break
        else:
            return [None if c is _UNSET else c for c in pattern]  # type: ignore[misc]
    raise RenderError("свёртка невозможна")  # период = вся форма всегда подходит


def _note(cell: Optional[Cell]) -> str:
    if cell is None:
        return "None"
    notes = cell[0]
    return str(notes[0]) if len(notes) == 1 else "(" + ", ".join(str(n) for n in notes) + ")"


def _tonal_line(slot: str, role: str, part: Part, track: Track) -> str:
    pattern = _fold(_cells(role, part, track), _active_steps(track, role), track.form.bars_total)
    sus = [c[1] if c else STEP_BEATS for c in pattern]
    accents = [c[2] if c else 0 for c in pattern]
    sus_text = _num(sus[0]) if len(set(sus)) == 1 else _list(_num(s) for s in sus)
    opts = [f"dur=1/4, sus={sus_text}, scale=Scale.chromatic, root=0, oct=0"]
    onsets = {c[2] for c in pattern if c}
    opts += _tail(track, role, part, accents if len(onsets) > 1 else [0])
    return f"{slot} >> {part.synth_or_sample}({_list(_note(c) for c in pattern)}, " + ", ".join(opts) + ")"


def _slots(track: Track, deck: str) -> Dict[str, str]:
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    unknown = sorted(set(track.parts) - set(ROLE_SLOT))
    if unknown:
        raise RenderError(f"роли {unknown} рендер ещё не выражает (ADR-0149 PR-3)")
    slots = {role: kn.DECK_SLOTS[deck][ROLE_SLOT[role]] for role in track.parts}
    if len(set(slots.values())) != len(slots):
        raise RenderError(f"две роли в одном слоте: {slots}")
    return slots


def render(track: Track, deck: str) -> Program:
    """Модель трека → программа деки ``deck``. Детерминирована: одна модель — одна строка."""
    validate(track)
    slots = _slots(track, deck)
    key = track.key
    lines = [f"# {track.track_id} {kn.ROOTS[key.root]} {key.mode} {track.bpm} BPM deck {deck}"]
    for role in sorted(track.parts, key=ROLE_SLOT.__getitem__):
        part = track.parts[role]
        line = _tonal_line if role in kn.TONAL_ROLES else _drum_line
        lines.append(line(slots[role], role, part, track))
    tonal = {p.synth_or_sample for r, p in track.parts.items() if r in kn.TONAL_ROLES}
    drums = {kn.DRUM_SYMBOLS[r] for r in track.parts if r not in kn.TONAL_ROLES}
    return Program(
        code="\n".join(lines) + "\n", track_id=track.track_id, deck=deck, bpm=track.bpm,
        form_beats=float(track.form.bars_total * BEATS_PER_BAR), slots=slots,
        synths=frozenset(tonal), samples=frozenset(drums),
    )


__all__ = ["ROLE_SLOT", "RenderError", "render"]
