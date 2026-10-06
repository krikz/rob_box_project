"""``ScoreMaterial`` — материал темы, независимый от источника (ADR-0154 §3.1).

Партитура (MusicXML/PDMX) несёт то, чего нет в RTTTL: гармонию автора, басовый голос, структуру фраз и повторов,
размер и темп. Модуль — только данные, валидатор и JSON; импортёр (``scripts/music/score_import.py``, PR-2) пишет
эти JSON офлайн, аранжировщик читает (``arrange.hook.from_material``).

Единицы — **четверти** от начала пьесы в исходном размере (``meter``), абсолютный MIDI, никаких секунд (I22). Такт
исходного размера длится ``meter[0] * 4 / meter[1]`` четвертей (:func:`bar_beats`). Знание о ладах — из
``knowledge`` (одна таблица), здесь не дублируется. Нарушение инварианта — :class:`MaterialError` с путём поля и
причиной, как :class:`rob_box_music.model.TrackError`: материал без лицензии, с кривой мелодией или хаосом в
аккордах не доходит до аранжировщика молча (ADR-0018).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from . import knowledge as kn
from .model import BEATS_PER_BAR, Key, PitchEvent

SCHEMA_VERSION = 1
#: Качества аккордов материала (ADR-0154 §3.1); ``degree`` — ступень диатонического аккорда в ``key`` или None.
CHORD_QUALITIES: Tuple[str, ...] = ("maj", "min", "dim", "aug", "sus", "dom7", "maj7", "min7", "other")
#: Отношение фразы к предыдущей (Н10): буквальный повтор, секвенция, тот же ритм с другим контуром, новая.
PHRASE_RELATIONS: Tuple[str, ...] = ("repeat", "sequence", "rhythm", "new")
#: Происхождение секции: текстовая метка партитуры или найдена по повторам.
SECTION_ORIGINS: Tuple[str, ...] = ("text", "repeat")
_ID_RE = re.compile(r"^(pdmx|local):[A-Za-z0-9_.\-]+$")
BPM_RANGE = (20, 400)


class MaterialError(ValueError):
    """Материал нарушает инвариант: ``path`` — поле, ``reason`` — почему."""

    def __init__(self, path: str, reason: str) -> None:
        super().__init__(f"{path}: {reason}")
        self.path = path
        self.reason = reason


@dataclass(frozen=True)
class ChordSpan:
    beat: float
    dur_beats: float
    root_pc: int  # 0..11
    quality: str  # из CHORD_QUALITIES
    degree: Optional[int] = None  # 0..6 в key материала; None — недиатонический (заимствованный)


@dataclass(frozen=True)
class Phrase:
    bar: int  # такт начала (с 0, в исходном размере)
    bars: int
    relation: str  # из PHRASE_RELATIONS
    repeats: int = 1  # сколько раз фраза встречается в пьесе, включая эту (Н9)


@dataclass(frozen=True)
class ScoreSection:
    name: str
    bar: int
    bars: int
    origin: str  # из SECTION_ORIGINS


@dataclass(frozen=True)
class MaterialStats:
    rating: Optional[float] = None
    n_views: Optional[int] = None
    complexity: Optional[int] = None
    key_fit: Optional[float] = None  # доля длительности мелодии в ладу материала, 0..1
    unique_bar_share: Optional[float] = None  # 0..1 (Н9)
    textures: Mapping[str, float] = field(default_factory=dict)  # доли фактур аккомпанемента (Н8)


@dataclass(frozen=True)
class ScoreMaterial:
    material_id: str  # "pdmx:<id>" | "local:<sha8 файла>"
    title: str
    composer: str
    source: str
    license: str  # "CC-BY-4.0" | "PD" | …; пусто/"unknown"/"…conflict" — не используется
    meter: Tuple[int, int]
    bpm: Optional[int]
    key: Key
    melody: Tuple[PitchEvent, ...]  # skyline, по возрастанию ``beat``
    chords: Tuple[ChordSpan, ...] = ()
    bass: Tuple[PitchEvent, ...] = ()
    phrases: Tuple[Phrase, ...] = ()
    sections: Tuple[ScoreSection, ...] = ()
    stats: MaterialStats = field(default_factory=MaterialStats)


def bar_beats(meter: Tuple[int, int]) -> float:
    """Длина такта размера ``meter`` в четвертях."""
    return meter[0] * 4 / meter[1]


def club_beat(meter: Tuple[int, int], beat: float) -> float:
    """Доля материала → доля в тактах 4/4 клуба (ADR-0154 §3.6, В5 (б)): такт материала встаёт в такт 4/4 с начала,
    короткий (3/4) дополняется паузой. Одна формула для хука и гармонии."""
    bar = bar_beats(meter)
    bar_idx, offset = divmod(beat, bar)
    return bar_idx * BEATS_PER_BAR + offset


# ── валидатор ──────────────────────────────────────────────────────────────────────────────────────────────

def _require(cond: bool, path: str, reason: str) -> None:
    if not cond:
        raise MaterialError(path, reason)


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _check_identity(m: ScoreMaterial) -> None:
    _require(isinstance(m.material_id, str) and bool(_ID_RE.match(m.material_id)), "material_id",
             f"{m.material_id!r} — ожидается 'pdmx:<id>' или 'local:<id>'")
    _require(isinstance(m.title, str) and bool(m.title.strip()), "title", "пустое название")
    lic = m.license.strip().lower() if isinstance(m.license, str) else ""
    _require(lic not in ("", "unknown") and "conflict" not in lic, "license",
             f"лицензия {m.license!r} не позволяет использовать материал (ADR-0154 §3.7)")


def _check_meter_key(m: ScoreMaterial) -> None:
    ok = (isinstance(m.meter, tuple) and len(m.meter) == 2 and all(_is_int(x) and x > 0 for x in m.meter)
          and m.meter[1] & (m.meter[1] - 1) == 0)
    _require(ok, "meter", f"{m.meter!r} — ожидается (числитель > 0, знаменатель — степень двойки)")
    _require(m.bpm is None or (_is_int(m.bpm) and BPM_RANGE[0] <= m.bpm <= BPM_RANGE[1]), "bpm",
             f"{m.bpm!r} вне {BPM_RANGE}")
    _require(isinstance(m.key, Key) and 0 <= m.key.root < 12 and m.key.mode in kn.SCALES, "key",
             f"{m.key!r} — лад не из knowledge.SCALES")


def _check_pitches(events: Sequence[PitchEvent], path: str, *, monophonic: bool) -> None:
    last = -1.0
    for i, e in enumerate(events):
        at = f"{path}[{i}]"
        _require(_is_int(e.midi) and 0 <= e.midi <= 127, f"{at}.midi", f"{e.midi!r} вне 0..127")
        _require(e.beat >= 0 and e.dur_beats > 0, at, f"beat={e.beat}, dur_beats={e.dur_beats}: нужны beat ≥ 0, dur > 0")
        _require(e.beat > last if monophonic else e.beat >= last, f"{at}.beat",
                 f"{e.beat} не после предыдущей ноты ({last}): ноты идут по возрастанию")
        last = e.beat


def _check_chords(m: ScoreMaterial) -> None:
    end = 0.0
    for i, c in enumerate(m.chords):
        at = f"chords[{i}]"
        _require(_is_int(c.root_pc) and 0 <= c.root_pc < 12, f"{at}.root_pc", f"{c.root_pc!r} вне 0..11")
        _require(c.quality in CHORD_QUALITIES, f"{at}.quality", f"{c.quality!r} не из {CHORD_QUALITIES}")
        _require(c.degree is None or (_is_int(c.degree) and 0 <= c.degree <= 6), f"{at}.degree",
                 f"{c.degree!r} — ожидается 0..6 или None")
        _require(c.beat >= end - 1e-9 and c.dur_beats > 0, at, f"аккорды не по порядку или без длительности "
                 f"(beat={c.beat}, dur={c.dur_beats}, конец предыдущего {end})")
        end = c.beat + c.dur_beats


def _check_structure(m: ScoreMaterial) -> None:
    last = -1
    for i, p in enumerate(m.phrases):
        at = f"phrases[{i}]"
        _require(_is_int(p.bar) and p.bar >= 0 and _is_int(p.bars) and p.bars > 0, at,
                 f"bar={p.bar!r}, bars={p.bars!r}: нужны bar ≥ 0, bars > 0")
        _require(p.relation in PHRASE_RELATIONS, f"{at}.relation", f"{p.relation!r} не из {PHRASE_RELATIONS}")
        _require(_is_int(p.repeats) and p.repeats >= 1, f"{at}.repeats", f"{p.repeats!r} — нужно ≥ 1")
        _require(p.bar >= last, f"{at}.bar", f"{p.bar} раньше предыдущей фразы ({last})")
        last = p.bar
    for i, s in enumerate(m.sections):
        at = f"sections[{i}]"
        _require(isinstance(s.name, str) and bool(s.name.strip()), f"{at}.name", "пустое имя секции")
        _require(s.origin in SECTION_ORIGINS, f"{at}.origin", f"{s.origin!r} не из {SECTION_ORIGINS}")
        _require(_is_int(s.bar) and s.bar >= 0 and _is_int(s.bars) and s.bars > 0, at,
                 f"bar={s.bar!r}, bars={s.bars!r}: нужны bar ≥ 0, bars > 0")


def _check_stats(m: ScoreMaterial) -> None:
    for name in ("key_fit", "unique_bar_share"):
        v = getattr(m.stats, name)
        _require(v is None or 0.0 <= v <= 1.0, f"stats.{name}", f"{v!r} вне 0..1")
    bad = [k for k, v in m.stats.textures.items() if not 0.0 <= v <= 1.0]
    _require(not bad, "stats.textures", f"доли вне 0..1: {bad}")


def validate_material(m: ScoreMaterial) -> None:
    """Проверить инварианты материала; нарушение — :class:`MaterialError` (путь + причина)."""
    _check_identity(m)
    _check_meter_key(m)
    _require(len(m.melody) > 0, "melody", "пустая мелодия")
    _check_pitches(m.melody, "melody", monophonic=True)
    _check_pitches(m.bass, "bass", monophonic=False)
    _check_chords(m)
    _check_structure(m)
    _check_stats(m)


# ── JSON ───────────────────────────────────────────────────────────────────────────────────────────────────

def _event_json(e: PitchEvent) -> list:
    return [e.midi, e.beat, e.dur_beats, e.accent]


def to_dict(m: ScoreMaterial) -> Dict[str, Any]:
    """JSON-совместимый словарь материала (после :func:`validate_material`); ноты — компактные ``[midi, beat, dur, accent]``."""
    validate_material(m)
    return {
        "schema": SCHEMA_VERSION, "material_id": m.material_id, "title": m.title, "composer": m.composer,
        "source": m.source, "license": m.license, "meter": list(m.meter), "bpm": m.bpm,
        "key": {"root": m.key.root, "mode": m.key.mode},
        "melody": [_event_json(e) for e in m.melody], "bass": [_event_json(e) for e in m.bass],
        "chords": [[c.beat, c.dur_beats, c.root_pc, c.quality, c.degree] for c in m.chords],
        "phrases": [[p.bar, p.bars, p.relation, p.repeats] for p in m.phrases],
        "sections": [[s.name, s.bar, s.bars, s.origin] for s in m.sections],
        "stats": {"rating": m.stats.rating, "n_views": m.stats.n_views, "complexity": m.stats.complexity,
                  "key_fit": m.stats.key_fit, "unique_bar_share": m.stats.unique_bar_share,
                  "textures": dict(sorted(m.stats.textures.items()))},
    }


def to_json(m: ScoreMaterial) -> str:
    """Детерминированный JSON материала (одинаковый материал — одинаковые байты)."""
    return json.dumps(to_dict(m), ensure_ascii=False, separators=(",", ":"))


def _field(data: Mapping[str, Any], name: str, kind: type, path: str = "") -> Any:
    _require(isinstance(data, Mapping) and name in data, f"{path}{name}", "поле отсутствует")
    value = data[name]
    _require(isinstance(value, kind) or (kind is float and _is_int(value)), f"{path}{name}",
             f"ожидается {kind.__name__}, пришло {type(value).__name__}")
    return value


def _row(rows: Any, name: str, size: int) -> list:
    _require(isinstance(rows, list), name, "ожидается список")
    for i, r in enumerate(rows):
        _require(isinstance(r, list) and len(r) == size, f"{name}[{i}]", f"ожидается список из {size} полей")
    return rows


def _events(data: Mapping[str, Any], name: str) -> Tuple[PitchEvent, ...]:
    return tuple(PitchEvent(r[0], r[1], r[2], r[3]) for r in _row(_field(data, name, list), name, 4))


def from_dict(data: Mapping[str, Any]) -> ScoreMaterial:
    """Материал из словаря :func:`to_dict`; кривая форма или нарушенный инвариант — :class:`MaterialError`."""
    try:
        return _build(data)
    except (TypeError, ValueError) as exc:
        if isinstance(exc, MaterialError):
            raise
        raise MaterialError("", f"поле неверного типа: {exc}") from exc


def _build(data: Mapping[str, Any]) -> ScoreMaterial:
    _require(isinstance(data, Mapping), "", f"ожидается объект, пришло {type(data).__name__}")
    _require(data.get("schema") == SCHEMA_VERSION, "schema", f"версия {data.get('schema')!r}, понимаю {SCHEMA_VERSION}")
    key = _field(data, "key", dict)
    stats = _field(data, "stats", dict)
    material = ScoreMaterial(
        material_id=_field(data, "material_id", str), title=_field(data, "title", str),
        composer=_field(data, "composer", str), source=_field(data, "source", str),
        license=_field(data, "license", str), meter=tuple(_field(data, "meter", list)),  # type: ignore[arg-type]
        bpm=data.get("bpm"), key=Key(_field(key, "root", int, "key."), _field(key, "mode", str, "key.")),
        melody=_events(data, "melody"), bass=_events(data, "bass"),
        chords=tuple(ChordSpan(*r) for r in _row(_field(data, "chords", list), "chords", 5)),
        phrases=tuple(Phrase(*r) for r in _row(_field(data, "phrases", list), "phrases", 4)),
        sections=tuple(ScoreSection(*r) for r in _row(_field(data, "sections", list), "sections", 4)),
        stats=MaterialStats(rating=stats.get("rating"), n_views=stats.get("n_views"),
                            complexity=stats.get("complexity"), key_fit=stats.get("key_fit"),
                            unique_bar_share=stats.get("unique_bar_share"),
                            textures=dict(_field(stats, "textures", dict, "stats."))))
    validate_material(material)
    return material


def from_json(text: str) -> ScoreMaterial:
    """Материал из JSON-строки; не JSON — тоже :class:`MaterialError`."""
    try:
        data = json.loads(text)
    except ValueError as exc:
        raise MaterialError("", f"не JSON: {exc}") from exc
    return from_dict(data)


__all__ = ["BPM_RANGE", "CHORD_QUALITIES", "ChordSpan", "MaterialError", "MaterialStats", "PHRASE_RELATIONS",
           "Phrase", "SCHEMA_VERSION", "SECTION_ORIGINS", "ScoreMaterial", "ScoreSection", "bar_beats", "club_beat",
           "from_dict",
           "from_json", "to_dict", "to_json", "validate_material"]
