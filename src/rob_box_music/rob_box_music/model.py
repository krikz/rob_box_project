"""Модель трека аранжировщика v2 и валидатор (ADR-0149 §3.1).

Музыка — данные: ``Track`` неизменяем, длительности в долях такта (никаких
секунд, I22), уровни в дБ — другой тип поля, поэтому «регекс по списку чисел»
невозможен. Валидатор :func:`validate` бросает :class:`TrackError` с путём до
ошибочного поля (``parts.bass.pitches[3].midi``) и причиной.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

from . import knowledge as kn

BEATS_PER_BAR = 4
STEPS_PER_BAR = 16
BARS_TOTAL = (32, 48, 64)
OUTRO_MIN_BARS = 8
PHRASE_BARS = (8, 16, 32)
APPROACH_MAX_BEATS = 0.5  # хроматический подход баса (ADR-0149 §3.5)


class TrackError(ValueError):
    """Модель трека нарушает инвариант: ``path`` — поле, ``reason`` — почему."""

    def __init__(self, path: str, reason: str) -> None:
        super().__init__(f"{path}: {reason}")
        self.path = path
        self.reason = reason


@dataclass(frozen=True)
class Key:
    root: int  # 0..11, индекс в ``knowledge.ROOTS``
    mode: str  # ключ ``knowledge.SCALES`` (не chromatic)


@dataclass(frozen=True)
class Section:
    name: str
    bars: int
    energy: int  # 0..10
    roles: frozenset  # frozenset[str]: роли, звучащие в секции
    fill_last_bar: bool = False


@dataclass(frozen=True)
class Form:
    sections: Tuple[Section, ...]

    @property
    def bars_total(self) -> int:
        return sum(s.bars for s in self.sections)


@dataclass(frozen=True)
class Step:
    on: bool
    accent: int = 0  # 0..3
    offset_ms: int = 0  # свинг


@dataclass(frozen=True)
class Grid:
    steps: Tuple[Step, ...]  # 16 × такты цикла


@dataclass(frozen=True)
class PitchEvent:
    midi: int
    beat: float  # доля от начала трека
    dur_beats: float
    accent: int = 0


@dataclass(frozen=True)
class Part:
    role: str
    synth_or_sample: str
    grid: Grid
    pitches: Optional[Tuple[PitchEvent, ...]]  # только у тональных ролей
    level_db: float
    register: Tuple[int, int]


@dataclass(frozen=True)
class Chord:
    degree: int  # ступень лада 0..6
    voicing: Tuple[int, ...]  # MIDI, уже «проведённое» обращение


@dataclass(frozen=True)
class Harmony:
    progression: Mapping[str, Tuple[Chord, ...]]  # имя секции → аккорды по тактам


@dataclass(frozen=True)
class Hook:
    notes: Tuple[PitchEvent, ...]  # доли от начала мотива
    bars: int  # 4..8
    source: Optional[str]  # id мелодии RTTTL или None


@dataclass(frozen=True)
class Mix:
    level_db: Mapping[str, float]
    pan: Mapping[str, float]  # -1..1
    duck_depth: float  # 0..1
    fx: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)  # имя секции → эффекты


@dataclass(frozen=True)
class Transition:
    phrase_bars: int
    bass_swap_bar: int
    filter_in: bool


@dataclass(frozen=True)
class HistoryKey:
    kit: str
    progression: str
    hook: Optional[str]
    sample: Optional[str]
    root: int


@dataclass(frozen=True)
class Track:
    track_id: str
    seed: int
    bpm: int
    key: Key
    form: Form
    parts: Mapping[str, Part]
    harmony: Harmony
    hook: Optional[Hook]
    mix: Mix
    energy: int  # 1..5
    transition_in: Transition
    transition_out: Transition
    history_key: HistoryKey


def _require(ok: bool, path: str, reason: str) -> None:
    if not ok:
        raise TrackError(path, reason)


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _check_key(track: Track) -> None:
    key = track.key
    _require(isinstance(key.root, int) and 0 <= key.root <= 11, "key.root", f"тоника {key.root!r} вне 0..11")
    _require(key.mode in kn.SCALES and key.mode != kn.CHROMATIC, "key.mode", f"лад {key.mode!r} не из knowledge.SCALES")
    lo, hi = kn.BPM_RANGE
    _require(isinstance(track.bpm, int) and lo <= track.bpm <= hi, "bpm", f"bpm {track.bpm!r} вне {lo}..{hi}")
    _require(bool(track.track_id), "track_id", "пустой id")
    _require(track.energy in kn.ENERGY_LEVELS, "energy", f"энергия {track.energy!r} вне 1..5")


def _check_form(form: Form) -> None:
    _require(bool(form.sections), "form.sections", "нет секций")
    _require(form.bars_total in BARS_TOTAL, "form.bars_total", f"{form.bars_total} тактов не из {BARS_TOTAL}")
    for i, sec in enumerate(form.sections):
        path = f"form.sections[{i}]"
        _require(sec.bars > 0 and sec.bars % 4 == 0, f"{path}.bars", f"{sec.bars} тактов: нужно кратное 4")
        _require(0 <= sec.energy <= 10, f"{path}.energy", f"энергия {sec.energy} вне 0..10")
        unknown = sorted(set(sec.roles) - set(kn.ROLES))
        _require(not unknown, f"{path}.roles", f"неизвестные роли {unknown}")
    last = form.sections[-1]
    _require(last.name == "outro" and last.bars >= OUTRO_MIN_BARS, "form.sections[-1]",
             f"последняя секция — outro ≥ {OUTRO_MIN_BARS} тактов (DJ-friendly)")
    _require("lead" not in last.roles, "form.sections[-1].roles", "в outro нет лида")


def _check_grid(role: str, grid: Grid, bars_total: int) -> None:
    n = len(grid.steps)
    path = f"parts.{role}.grid.steps"
    _require(n > 0 and n % STEPS_PER_BAR == 0, path, f"длина {n} не кратна такту ({STEPS_PER_BAR})")
    _require((STEPS_PER_BAR * bars_total) % n == 0, path, f"длина {n} не делит форму ({bars_total} тактов)")
    for i, st in enumerate(grid.steps):
        _require(0 <= st.accent <= 3, f"{path}[{i}].accent", f"акцент {st.accent} вне 0..3")
        _require(isinstance(st.offset_ms, int), f"{path}[{i}].offset_ms", "сдвиг не в целых мс")


def _check_pitch(role: str, i: int, ev: PitchEvent, key: Key, limit_beats: float, part: Part) -> None:
    path = f"parts.{role}.pitches[{i}]"
    _require(_finite(ev.beat) and _finite(ev.dur_beats), path, "доли не конечные числа")
    _require(ev.dur_beats > 0 and 0 <= ev.beat and ev.beat + ev.dur_beats <= limit_beats + 1e-9,
             f"{path}.beat", f"нота {ev.beat}+{ev.dur_beats} вне формы ({limit_beats} долей)")
    lo, hi = part.register
    _require(lo <= ev.midi <= hi, f"{path}.midi", f"MIDI {ev.midi} вне регистра партии {lo}..{hi}")
    in_scale = ev.midi % 12 in kn.scale_pitch_classes(key.root, key.mode)
    approach = role == "bass" and ev.dur_beats <= APPROACH_MAX_BEATS
    _require(in_scale or approach, f"{path}.midi", f"MIDI {ev.midi} не в ладе {kn.ROOTS[key.root]} {key.mode}")


def _check_tonal(role: str, part: Part, track: Track) -> None:
    corridor = kn.REGISTERS[role]
    lo, hi = part.register
    _require(corridor[0] <= lo <= hi <= corridor[1], f"parts.{role}.register",
             f"регистр {part.register} вне коридора {corridor}")
    _require(hi <= kn.LEAD_MAX_MIDI, f"parts.{role}.register", f"верх {hi} выше {kn.LEAD_MAX_MIDI}")
    _require(bool(part.pitches), f"parts.{role}.pitches", "тональная партия без нот")
    limit = float(track.form.bars_total * BEATS_PER_BAR)
    for i, ev in enumerate(part.pitches or ()):
        _check_pitch(role, i, ev, track.key, limit, part)
    palette = kn.SYNTH_PALETTE.get(role, ())
    _require(part.synth_or_sample in palette or part.synth_or_sample in kn.SYNTH_TRAITS,
             f"parts.{role}.synth_or_sample", f"синт {part.synth_or_sample!r} не из палитры")


def _check_parts(track: Track) -> None:
    for role, part in track.parts.items():
        _require(role in kn.ROLES and part.role == role, f"parts.{role}.role", "роль не совпадает с ключом")
        _check_grid(role, part.grid, track.form.bars_total)
        if role in kn.TONAL_ROLES:
            _check_tonal(role, part, track)
        else:
            _require(part.pitches is None, f"parts.{role}.pitches", "у ударной роли нет высот")
        _require(_finite(part.level_db) and part.level_db <= kn.role_ceiling(role), f"parts.{role}.level_db",
                 f"{part.level_db} дБ выше потолка роли {kn.role_ceiling(role)}")
        _require(track.mix.level_db.get(role) == part.level_db, f"mix.level_db.{role}",
                 "уровень в Mix расходится с партией (одна ось громкости)")
    for i, sec in enumerate(track.form.sections):
        missing = sorted(set(sec.roles) - set(track.parts))
        _require(not missing, f"form.sections[{i}].roles", f"роли без партии: {missing}")


def _check_levels(track: Track) -> None:
    limit = 10.0 ** (kn.LEVEL_CEILINGS["master_peak_db"] / 10.0)
    for i, sec in enumerate(track.form.sections):
        power = sum(10.0 ** (track.parts[r].level_db / 10.0) for r in sec.roles)
        _require(power <= limit, f"form.sections[{i}].roles",
                 f"сумма пиков {10 * math.log10(power):.1f} дБ выше потолка {kn.LEVEL_CEILINGS['master_peak_db']}")
    _require(0.0 <= track.mix.duck_depth <= 1.0, "mix.duck_depth", "глубина сайдчейна вне 0..1")
    for role, pan in track.mix.pan.items():
        _require(-1.0 <= pan <= 1.0, f"mix.pan.{role}", f"панорама {pan} вне -1..1")
    for role in ("kick", "bass"):
        _require(track.mix.pan.get(role, 0.0) == 0.0, f"mix.pan.{role}", "низ — строго в центре (ADR-0149 §3.9)")


def _check_hook_and_harmony(track: Track) -> None:
    names = {s.name for s in track.form.sections}
    for name, chords in track.harmony.progression.items():
        _require(name in names, f"harmony.progression.{name}", "секции нет в форме")
        for i, chord in enumerate(chords):
            _require(0 <= chord.degree <= 6 and bool(chord.voicing), f"harmony.progression.{name}[{i}]",
                     "ступень вне 0..6 или пустое обращение")
    hook = track.hook
    if hook is None:
        return
    _require(4 <= hook.bars <= 8, "hook.bars", f"хук {hook.bars} тактов вне 4..8")
    for i, ev in enumerate(hook.notes):
        _require(ev.dur_beats > 0 and 0 <= ev.beat and ev.beat + ev.dur_beats <= hook.bars * BEATS_PER_BAR + 1e-9,
                 f"hook.notes[{i}].beat", "нота вне длины мотива")


def _check_transitions(track: Track) -> None:
    for name in ("transition_in", "transition_out"):
        tr = getattr(track, name)
        _require(tr.phrase_bars in PHRASE_BARS, f"{name}.phrase_bars", f"фраза {tr.phrase_bars} не из {PHRASE_BARS}")
        _require(0 <= tr.bass_swap_bar < tr.phrase_bars, f"{name}.bass_swap_bar", "своп баса вне фразы")


def validate(track: Track) -> None:
    """Проверить инварианты трека; нарушение — :class:`TrackError` (путь + причина)."""
    _check_key(track)
    _check_form(track.form)
    _check_parts(track)
    _check_levels(track)
    _check_hook_and_harmony(track)
    _check_transitions(track)


__all__ = [
    "Chord", "Form", "Grid", "Harmony", "HistoryKey", "Hook", "Key", "Mix", "Part", "PitchEvent", "Section",
    "Step", "Track", "TrackError", "Transition", "validate",
]
