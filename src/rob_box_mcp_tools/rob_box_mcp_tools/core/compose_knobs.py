"""Ручки ``compose_music`` → опции ядра аранжировщика (ADR-0132, PR-4).

Модель передаёт ручки отдельными плоскими параметрами тула (вариант A
ADR-0132 §5, выбран экспериментом 23.09.2026) — строками из JSON-вызова.
Этот модуль переводит их в :class:`core.harmonize.HarmonizeOptions` (ноты
партий) и :class:`core.arranger.ArrangeOptions` (сборка слоёв). Проверку
значений делают сами классы опций (PR-3) — здесь только разбор строкового
формата тула (``"Am|F|C|G"``, ``"55-67"``, ``"bass=0.5,pad=0.8"``, ``"+1"``)
и сообщения об ошибке, которые модель может исправить следующим вызовом.

Незаданная ручка (``None``/пустая строка) = значение по умолчанию = ``auto``
(у ``lead_outliers`` — ``fix``): сегодняшнее поведение байт-в-байт
(``test_arranger_golden``). Неизвестное значение — ``ValueError`` со
списком допустимых, никогда не тихая замена на умолчание.

Модуль без ROS, как и остальной ``core``.
"""

from __future__ import annotations

import dataclasses
import re
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple, Union

from .arranger import ArrangeOptions
from .harmonize import (
    AUTO,
    LEAD_OCTAVE_RANGE,
    LEAD_OCTAVE_WORDS,
    PAD_REGISTER_LIMITS,
    PAD_REGISTERS,
    HarmonizeOptions,
    parse_chord,
)

__all__ = [
    "ComposeKnobs",
    "KNOB_PARAMS",
    "build_knobs",
    "lead_octave_choices",
    "parse_chords",
    "parse_lead_octave",
    "parse_levels",
    "parse_pad_register",
    "parse_theme_octaves",
]

#: Ручки-слова ``HarmonizeOptions`` (значения — ``harmonize.KNOB_VALUES``).
_HARMONIZE_WORDS: Tuple[str, ...] = (
    "key_detection",
    "harmonic_rhythm",
    "density",
    "bass_style",
    "bass_approach",
    "pad_style",
    "lead_outliers",
)

#: Все ручки-параметры ``compose_music`` (порядок — как в схеме тула).
KNOB_PARAMS: Tuple[str, ...] = _HARMONIZE_WORDS + (
    "chords",
    "pad_register",
    "lead_octave",
    "counter",
    "theme_octaves",
    "levels",
)

_CHORDS_FORMAT = "аккорды по тактам через «|», по кругу, напр. Am|F|C|G"
_LEVELS_FORMAT = "роль=множитель через запятую, напр. bass=0.5,pad=0.8"
_RANGE_RE = re.compile(r"(\d+)\s*-\s*(\d+)")
_INT_RE = re.compile(r"[+-]?\d+")


def _unset(value: object) -> bool:
    return value is None or (isinstance(value, str) and not value.strip())


def _word(value: object, default: str) -> object:
    """Слово ручки: регистр и пробелы не важны; не строка — как есть (ошибку даст проверка)."""
    if _unset(value):
        return default
    return value.strip().lower() if isinstance(value, str) else value


def lead_octave_choices() -> List[str]:
    """Значения ``lead_octave`` в схеме тула: ``auto, keep, -2, -1, 0, +1, +2``."""
    lo, hi = LEAD_OCTAVE_RANGE
    return list(LEAD_OCTAVE_WORDS) + [f"{n:+d}" if n else "0" for n in range(lo, hi + 1)]


def parse_chords(value: object) -> Optional[Tuple[str, ...]]:
    """``"Am|F|C|G"`` → ``("Am", "F", "C", "G")``; не задано/``auto`` → ``None``."""
    if _unset(value) or _word(value, AUTO) == AUTO:
        return None
    names = tuple(part.strip() for part in str(value).split("|"))
    for name in names:
        try:
            parse_chord(name)
        except ValueError as exc:
            raise ValueError(f"chords={value!r}: {exc} Формат: {_CHORDS_FORMAT}.") from None
    return names


def parse_pad_register(value: object) -> Union[str, Tuple[int, int]]:
    """``auto|low|mid|high`` или ``"низ-верх"`` MIDI (``"55-67"``) → значение ручки."""
    word = _word(value, AUTO)
    if word == AUTO or word in PAD_REGISTERS:
        return word  # type: ignore[return-value]
    match = _RANGE_RE.fullmatch(str(word))
    if match:
        register = (int(match.group(1)), int(match.group(2)))
        try:
            HarmonizeOptions(pad_register=register)
        except ValueError:
            raise ValueError(_pad_register_error(value)) from None
        return register
    raise ValueError(_pad_register_error(value))


def _pad_register_error(value: object) -> str:
    bands = ", ".join(f"{name} ({lo}–{hi})" for name, (lo, hi) in PAD_REGISTERS.items())
    lo_limit, hi_limit = PAD_REGISTER_LIMITS
    return (
        f"pad_register={value!r}: допустимо auto, {bands} или диапазон «низ-верх» "
        f"MIDI в {lo_limit}..{hi_limit} не уже октавы, напр. 55-67."
    )


def parse_lead_octave(value: object) -> Union[str, int]:
    """``auto|keep`` или сдвиг октавами ``"-2".."+2"`` (строкой или числом)."""
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    word = _word(value, AUTO)
    if isinstance(word, str) and _INT_RE.fullmatch(word):
        return int(word)
    return word  # type: ignore[return-value]  # слово проверит HarmonizeOptions


def parse_levels(value: object) -> Dict[str, float]:
    """``"bass=0.5,pad=0.8"`` → ``{"bass": 0.5, "pad": 0.8}``; роли и 0..2 проверит ArrangeOptions."""
    if isinstance(value, Mapping):
        return dict(value)
    if _unset(value) or _word(value, AUTO) == AUTO:
        return {}
    levels: Dict[str, float] = {}
    for item in (part.strip() for part in str(value).split(",")):
        if not item:
            continue
        role, sep, number = item.partition("=")
        try:
            if not sep:
                raise ValueError
            levels[role.strip().lower()] = float(number)
        except ValueError:
            raise ValueError(f"levels: {item!r} — нужен формат {_LEVELS_FORMAT}.") from None
    return levels


def parse_theme_octaves(value: object) -> Tuple[object, bool]:
    """``theme_octaves`` → ``(режим ручки, флаг spec_from_flat)``.

    ``auto|on|off`` — ручка ADR-0132. Старый булев контракт (#2463) сохранён
    байт-в-байт: ``true`` = ``auto`` (удвоение по правилу плотности),
    ``false`` = прежний флаг ``theme_octaves=False``.
    """
    if isinstance(value, bool):
        return AUTO, value
    word = _word(value, AUTO)
    legacy = {"true": (AUTO, True), "false": (AUTO, False)}
    if word in legacy:
        return legacy[word]  # type: ignore[index]
    return word, True


@dataclass(frozen=True)
class ComposeKnobs:
    """Разобранные ручки одного вызова ``compose_music``.

    Attributes:
        harmonize: ноты партий (используются только с ``name=``).
        arrange: сборка слоёв (``levels`` — и для сочинённого трека).
        theme_octaves: флаг ``spec_from_flat(theme_octaves=)``.
    """

    harmonize: HarmonizeOptions
    arrange: ArrangeOptions
    theme_octaves: bool

    def name_only_set(self) -> List[str]:
        """Заданные (не по умолчанию) ручки, которые действуют только с ``name=``.

        Выведенную из темы аранжировку строит только путь ``name=``; у
        сочинённого трека бас/пэд/аккорды задаются нотами, а не ручками.
        """
        default_h, default_a = HarmonizeOptions(), ArrangeOptions()
        changed = [
            f.name for f in dataclasses.fields(HarmonizeOptions)
            if f.name not in ("drums", "hats")
            and getattr(self.harmonize, f.name) != getattr(default_h, f.name)
        ]
        changed += [
            knob for knob in ("counter", "theme_octaves")
            if getattr(self.arrange, knob) != getattr(default_a, knob)
        ]
        return changed


def build_knobs(
    knobs: Mapping[str, object],
    *,
    drums: Optional[str] = None,
    hats: Optional[str] = None,
) -> ComposeKnobs:
    """Параметры-ручки тула → :class:`ComposeKnobs`.

    ``knobs`` — ``{имя из KNOB_PARAMS: значение из вызова}``, отсутствующие
    = умолчание. ``drums``/``hats`` — явные рисунки ударных вместо
    выведенных из темы (передаются только при ``name=``).

    Raises:
        ValueError: неизвестное значение или формат — текст называет ручку
            и допустимые значения (``ArrangementError`` — подкласс).
    """
    defaults = {f.name: f.default for f in dataclasses.fields(HarmonizeOptions)}
    words = {knob: _word(knobs.get(knob), defaults[knob]) for knob in _HARMONIZE_WORDS}
    mode, flag = parse_theme_octaves(knobs.get("theme_octaves"))
    harmonize = HarmonizeOptions(
        **words,  # type: ignore[arg-type]
        chords=parse_chords(knobs.get("chords")),
        pad_register=parse_pad_register(knobs.get("pad_register")),
        lead_octave=parse_lead_octave(knobs.get("lead_octave")),
        drums=drums,
        hats=hats,
    )
    arrange = ArrangeOptions(
        counter=_word(knobs.get("counter"), AUTO),  # type: ignore[arg-type]
        theme_octaves=mode,  # type: ignore[arg-type]
        levels=parse_levels(knobs.get("levels")),
    )
    return ComposeKnobs(harmonize=harmonize, arrange=arrange, theme_octaves=flag)
