"""track_spec.py — структурированная спека трека (ADR-0142 §4-§5, issue #3136).

Первый кодовый шаг ADR-0142 (там — PR-2): спека как ДАННЫЕ, генерируемая
схема, валидатор с путём ошибки и детерминированный рендер спеки через
существующий клубный аранжировщик. LLM здесь нет.

Модуль чистый (без Renardo/ROS), как :mod:`core.arrangement_matrix`.

Спека v1 (``style="club"``) честно сужена до того, что рендер УЖЕ умеет
(ADR-0142 §5.2, урок ``_club_ignored``: схема не обещает того, чего не
делает рендер)::

    {
      "version": 1,
      "style": "club",
      "bpm": 124, "root": "A#", "scale": "minor",
      "seed": 0,                                  # риф (и прогрессия по умолчанию)
      "form": {"template": "dj_dave_32"},          # ключ SECTION_TEMPLATES
      "progression": "VI-III-VII-i",               # имя из PROGRESSIONS
      "groove": {"kick": "by_design", "hats": "by_design"},
      "timbre": {"lead": "pluck", "bass": "bass", "pad": "sinepad"},
      "levels": {"lead": 0.9},                     # необяз., множители 0..1
      "notes": "..."                               # необяз., в рендер не идёт
    }

Чего в v1 НЕТ (и что валидатор отклоняет как неизвестные поля): ``style
="classic"``, ``melody``, ``sections``, ``matrix`` (своя матрица),
``groove.bass``/``groove.lead``, ``passes``, ``hype_line``, ``code``. Они
появятся вместе с поддержкой в рендере (ADR-0142 §11, PR-3 и дальше).

Расхождение с ADR-0142 §5.2 (описано в PR): поле ``seed`` — в ADR его нет,
но без него рендер не может выбрать риф лида; ADR-правило «сумма пиков ≤
1.2» к клубу не применимо как есть — у эталона «By Design» сумма пиков
слоёв уже ≈1.6 (``peak_levels``), поэтому v1 держит бюджет иначе:
множители ``levels`` только 0..1, то есть сумма пиков не выше эталонной.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from .arrangement_matrix import SECTION_TEMPLATES
from .arranger import BPM_RANGE, VALID_ROOTS
from .club_arranger import (
    HATS_PATTERNS,
    KICK_PATTERNS,
    LAYER_LEVELS,
    PROGRESSIONS,
    ROLE_SYNTHS,
    SUPPORTED_SCALES,
    club_form_beats,
    club_kit,
    render_club_kit,
)

SPEC_VERSION = 1
SUPPORTED_STYLES: Tuple[str, ...] = ("club",)
SUPPORTED_ENTRIES: Tuple[str, ...] = ("fresh",)
PROGRESSION_NAMES: Tuple[str, ...] = tuple(name for name, _ in PROGRESSIONS)
SEED_RANGE: Tuple[int, int] = (0, 2**31 - 1)
NOTES_MAX_CHARS = 2000

_TOP_REQUIRED = ("version", "style", "bpm", "root", "scale", "seed", "form", "progression", "groove", "timbre")
_TOP_OPTIONAL = ("levels", "notes")


class SpecError(ValueError):
    """Спека не прошла валидацию: ``path`` — где (``"timbre.lead"``), ``reason`` — почему."""

    def __init__(self, path: str, reason: str) -> None:
        super().__init__(f"{path}: {reason}")
        self.path = path
        self.reason = reason


@dataclass(frozen=True)
class SpecAnchors:
    """Поля, которые ризонер НЕ может менять (ADR-0142 §4, §7.3).

    ``None`` — якоря нет. ``form_beats`` — длина формы в долях (F превью).
    """

    bpm: Optional[float] = None
    root: Optional[str] = None
    scale: Optional[str] = None
    form_beats: Optional[int] = None
    pad: Optional[str] = None
    hats: Optional[str] = None


@dataclass(frozen=True)
class TrackSpec:
    """Прошедшая валидацию спека трека v1. Только данные, без побочных эффектов."""

    bpm: float
    root: str
    scale: str
    seed: int
    template: str
    progression: str
    kick: str
    hats: str
    lead: str
    bass: str
    pad: str
    levels: Mapping[str, float] = field(default_factory=dict)
    notes: str = ""
    style: str = "club"
    version: int = SPEC_VERSION

    def kit(self) -> Dict[str, str]:
        """Каркас в формате :func:`core.club_arranger.club_kit`."""
        return {
            "template": self.template, "kick": self.kick, "hats": self.hats,
            "lead": self.lead, "bass": self.bass, "pad": self.pad,
        }

    def to_dict(self) -> Dict[str, Any]:
        """JSON-вид спеки; ``validate_spec(spec.to_dict()) == spec``."""
        out: Dict[str, Any] = {
            "version": self.version, "style": self.style,
            "bpm": self.bpm, "root": self.root, "scale": self.scale, "seed": self.seed,
            "form": {"template": self.template},
            "progression": self.progression,
            "groove": {"kick": self.kick, "hats": self.hats},
            "timbre": {"lead": self.lead, "bass": self.bass, "pad": self.pad},
        }
        if self.levels:
            out["levels"] = dict(self.levels)
        if self.notes:
            out["notes"] = self.notes
        return out


# ---------------------------------------------------------------------------
# Схема (генерируется из констант кода, ADR-0142 §5.1)
# ---------------------------------------------------------------------------


def _enum(values) -> Dict[str, Any]:
    return {"type": "string", "enum": list(values)}


def _object(properties: Dict[str, Any], required) -> Dict[str, Any]:
    return {
        "type": "object",
        "properties": properties,
        "required": list(required),
        "additionalProperties": False,
    }


def track_spec_schema() -> Dict[str, Any]:
    """JSON-схема спеки v1 из констант кода. Руками схема не пишется.

    Снимок — ``test/fixtures/track_spec.schema.json``; тест сверяет его с
    генерацией, так схема не разъедется с палитрой аранжировщика.
    """
    level = {"type": "number", "minimum": 0.0, "maximum": 1.0}
    properties = {
        "version": {"type": "integer", "const": SPEC_VERSION},
        "style": _enum(SUPPORTED_STYLES),
        "bpm": {"type": "number", "minimum": BPM_RANGE[0], "maximum": BPM_RANGE[1]},
        "root": _enum(VALID_ROOTS),
        "scale": _enum(SUPPORTED_SCALES),
        "seed": {"type": "integer", "minimum": SEED_RANGE[0], "maximum": SEED_RANGE[1]},
        "form": _object({"template": _enum(SECTION_TEMPLATES)}, ["template"]),
        "progression": _enum(PROGRESSION_NAMES),
        "groove": _object({"kick": _enum(KICK_PATTERNS), "hats": _enum(HATS_PATTERNS)}, ["kick", "hats"]),
        "timbre": _object({role: _enum(synths) for role, synths in ROLE_SYNTHS.items()}, list(ROLE_SYNTHS)),
        "levels": _object({lane: dict(level) for lane in LAYER_LEVELS}, []),
        "notes": {"type": "string", "maxLength": NOTES_MAX_CHARS},
    }
    schema = _object(properties, _TOP_REQUIRED)
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["title"] = "TrackSpec"
    return schema


# ---------------------------------------------------------------------------
# Валидатор (чистый Python, без jsonschema — его нет в зависимостях пакета)
# ---------------------------------------------------------------------------


def _require_object(value: Any, path: str, required, optional=()) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SpecError(path, f"ожидался объект, получено {type(value).__name__}")
    unknown = sorted(set(value) - set(required) - set(optional))
    if unknown:
        where = f"{path}.{unknown[0]}" if path != "$" else unknown[0]
        raise SpecError(where, "неизвестное поле (в спеке v1 его нет)")
    for key in required:
        if key not in value:
            raise SpecError(f"{path}.{key}" if path != "$" else key, "обязательное поле отсутствует")
    return value


def _one_of(value: Any, path: str, allowed) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise SpecError(path, f"{value!r} не из допустимых: {', '.join(allowed)}")
    return value


def _number(value: Any, path: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SpecError(path, f"ожидалось число, получено {value!r}")
    if not low <= float(value) <= high:
        raise SpecError(path, f"{value} вне диапазона {low:g}..{high:g}")
    return value


def _integer(value: Any, path: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SpecError(path, f"ожидалось целое, получено {value!r}")
    _number(value, path, low, high)
    return value


def _levels(raw: Any) -> Dict[str, float]:
    if raw is None:
        return {}
    levels = _require_object(raw, "levels", (), tuple(LAYER_LEVELS))
    return {lane: _number(v, f"levels.{lane}", 0.0, 1.0) for lane, v in levels.items()}


def _notes(raw: Any) -> str:
    if raw is None:
        return ""
    if not isinstance(raw, str):
        raise SpecError("notes", f"ожидалась строка, получено {type(raw).__name__}")
    if len(raw) > NOTES_MAX_CHARS:
        raise SpecError("notes", f"длиннее {NOTES_MAX_CHARS} символов")
    return raw


def _parse(raw: Any) -> TrackSpec:
    top = _require_object(raw, "$", _TOP_REQUIRED, _TOP_OPTIONAL)
    if isinstance(top["version"], bool) or top["version"] != SPEC_VERSION:
        raise SpecError("version", f"поддержана только версия {SPEC_VERSION}, получено {top['version']!r}")
    form = _require_object(top["form"], "form", ("template",))
    groove = _require_object(top["groove"], "groove", ("kick", "hats"))
    timbre = _require_object(top["timbre"], "timbre", tuple(ROLE_SYNTHS))
    return TrackSpec(
        style=_one_of(top["style"], "style", SUPPORTED_STYLES),
        bpm=_number(top["bpm"], "bpm", *BPM_RANGE),
        root=_one_of(top["root"], "root", VALID_ROOTS),
        scale=_one_of(top["scale"], "scale", SUPPORTED_SCALES),
        seed=_integer(top["seed"], "seed", *SEED_RANGE),
        template=_one_of(form["template"], "form.template", tuple(SECTION_TEMPLATES)),
        progression=_one_of(top["progression"], "progression", PROGRESSION_NAMES),
        kick=_one_of(groove["kick"], "groove.kick", tuple(KICK_PATTERNS)),
        hats=_one_of(groove["hats"], "groove.hats", tuple(HATS_PATTERNS)),
        lead=_one_of(timbre["lead"], "timbre.lead", ROLE_SYNTHS["lead"]),
        bass=_one_of(timbre["bass"], "timbre.bass", ROLE_SYNTHS["bass"]),
        pad=_one_of(timbre["pad"], "timbre.pad", ROLE_SYNTHS["pad"]),
        levels=_levels(top.get("levels")),
        notes=_notes(top.get("notes")),
    )


def _check_anchors(spec: TrackSpec, anchors: SpecAnchors) -> None:
    pairs = (
        ("bpm", anchors.bpm, spec.bpm),
        ("root", anchors.root, spec.root),
        ("scale", anchors.scale, spec.scale),
        ("timbre.pad", anchors.pad, spec.pad),
        ("groove.hats", anchors.hats, spec.hats),
    )
    for path, anchor, value in pairs:
        if anchor is not None and value != anchor:
            raise SpecError(path, f"якорь {anchor!r} менять нельзя, получено {value!r}")
    if anchors.form_beats is not None:
        beats = club_form_beats(spec.template)
        if beats != anchors.form_beats:
            raise SpecError(
                "form.template",
                f"длина формы {beats} долей ≠ якорю {anchors.form_beats} ({spec.template!r})",
            )


def validate_spec(raw: Any, anchors: Optional[SpecAnchors] = None) -> TrackSpec:
    """Сырой JSON (dict) → :class:`TrackSpec` или :class:`SpecError` с путём.

    Проверяет типы, перечисления из палитры кода, диапазоны, неизвестные
    поля и якоря (``anchors``). Выдумать синт/шаблон/прогрессию нельзя.
    """
    spec = _parse(raw)
    if anchors is not None:
        _check_anchors(spec, anchors)
    return spec


# ---------------------------------------------------------------------------
# Seeded-спека и рендер
# ---------------------------------------------------------------------------


def seeded_progression(seed: int) -> str:
    """Имя прогрессии, которую выбрал бы ``render_club(seed=seed)``."""
    return PROGRESSIONS[random.Random(seed).randrange(len(PROGRESSIONS))][0]


def seeded_spec(
    seed: int = 0,
    bpm: float = 124,
    root: str = "A#",
    scale: str = "minor",
    template: Optional[str] = None,
    kick: Optional[str] = None,
) -> TrackSpec:
    """Детерминированная спека без сети — то, что сегодня играет ``render_club``.

    Регресс-якорь ADR-0142 §4: ``render_spec(seeded_spec(seed=s)) ==
    render_club(seed=s)`` побайтно. Результат проходит :func:`validate_spec`.
    """
    kit = club_kit(seed, template, kick)
    raw = {
        "version": SPEC_VERSION, "style": "club",
        "bpm": bpm, "root": root, "scale": scale, "seed": seed,
        "form": {"template": kit["template"]},
        "progression": seeded_progression(seed),
        "groove": {"kick": kit["kick"], "hats": kit["hats"]},
        "timbre": {"lead": kit["lead"], "bass": kit["bass"], "pad": kit["pad"]},
    }
    return validate_spec(raw)


def render_spec(
    spec: TrackSpec, *, entry: str = "fresh", repeat: bool = False, align_clock: bool = False,
) -> str:
    """Спека → Renardo-программа. Детерминированно: одна спека — один код.

    ``entry="fresh"`` — полная программа (``Clock.clear`` → [выравнивание
    #3112] → ``Clock.bpm`` → плееры → ``Clock.future``), как сейчас у
    ``render_club``. ``entry="reveal"`` (переприсваивание плееров для
    замены превью, ADR-0142 §5.4) — следующий шаг (PR-3), здесь отказ.

    Raises:
        SpecError: неподдержанный ``entry``.
    """
    if entry not in SUPPORTED_ENTRIES:
        raise SpecError("entry", f"{entry!r} не поддержан (сейчас только: {', '.join(SUPPORTED_ENTRIES)})")
    return render_club_kit(
        spec.kit(),
        bpm=spec.bpm, root=spec.root, scale=spec.scale, seed=spec.seed,
        progression=spec.progression, levels=spec.levels or None,
        repeat=repeat, align_clock=align_clock,
    )


__all__ = [
    "SPEC_VERSION",
    "SpecAnchors",
    "SpecError",
    "TrackSpec",
    "render_spec",
    "seeded_progression",
    "seeded_spec",
    "track_spec_schema",
    "validate_spec",
]
