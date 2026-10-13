"""Разбор RTTTL (Nokia ringtone) в ноты: ``parse_rtttl`` (ADR-0149 §3.3, §8.1).

Перенесено из ``rob_box_mcp_tools/core/rtttl.py`` без изменений (ADR-0149 PR-3): одна реализация,
старый модуль импортирует ``parse_rtttl`` отсюда.

Формат: ``Name:d=4,o=5,b=140:8c5,8c5,8g5,...``
  * ``d`` — длительность по умолчанию (4 = четверть, 8 = восьмая…);
  * ``o`` — октава по умолчанию (4-7);
  * ``b`` — темп;
  * нота — ``[длительность][a-g/p][#|_][октава][.]``; ``p`` — пауза, ``.`` —
    точка (увеличивает длительность в полтора раза).

Альтерации: ``#`` и ``_`` — оба обозначают диез (в коллекциях PICAXE ``_`` —
историческая замена ``#``; это доказано парой файлов одной песни в двух
тональностях — V1 с ``_`` и V2 с ``#``, ноты совпадают со сдвигом на полутон).
"""

from __future__ import annotations

import logging
import re
from collections import Counter
from typing import Any, Dict, List, Optional, Sequence, Tuple

__all__ = ["CONTOUR_NOTES", "THEME_HOOKS", "consensus_order", "contour", "fix_octaves", "parse_rtttl"]

_LOG = logging.getLogger(__name__)

#: Смещение буквы ноты в полутонах от C.
_NOTE_SEMITONE = {"c": 0, "d": 2, "e": 4, "f": 5, "g": 7, "a": 9, "b": 11}

#: Токен: [длительность][нота][#|_][октава][.], напр. ``8c5``, ``2e.``,
#: ``8g#5``, ``8a_``, ``16p``. Октава и длительность опциональны (берутся из
#: заголовка). Точка может стоять и ДО октавы (диалект PICAXE: ``8c.7``,
#: ``16d#.6``) — оба варианта понимаются.
_TOKEN_RE = re.compile(
    r"^(?P<dur>\d*)(?P<note>[a-gp])(?P<acc>[#_]?)"
    r"(?:(?P<oct>\d+)(?P<dot>\.?)|(?P<dot2>\.)(?P<oct2>\d*))?$",
    re.I,
)


def _to_midi(note: str, acc: str, octave: int) -> int:
    """Буква ноты + октава → MIDI (C4 = 60, как в научной нотации).

    ``acc`` — альтерация: ``#`` или ``_`` → +1 полутон (диез), пустая строка
    → 0. В диалекте PICAXE ``_`` — синоним ``#``.
    """
    shift = 1 if acc in ("#", "_") else 0
    return 12 * (octave + 1) + _NOTE_SEMITONE[note.lower()] + shift


def parse_rtttl(rtttl: str) -> Tuple[str, int, List[Tuple[Optional[int], float]]]:
    """Разобрать RTTTL-строку в ``(имя, bpm, [(midi|None, доли)])``.

    ``None`` вместо midi — пауза (в Renardo это rest в списке нот).
    Длительности — в долях такта (четверть = 1.0), точка = ×1.5.

    Raises:
        ValueError: строка не похожа на RTTTL (нет двух двоеточий или
            токен ноты не распознаётся).
    """
    try:
        name, settings, data = rtttl.strip().split(":", 2)
    except ValueError as exc:
        raise ValueError(
            f"RTTTL должен содержать 'name:d=…,o=…,b=…:ноты', получено {rtttl[:60]!r}"
        ) from exc

    defaults: dict = {}
    for kv in settings.split(","):
        key, _, value = kv.partition("=")
        key = key.strip().lower()
        if key:
            defaults[key] = value.strip()
    default_dur = int(defaults.get("d", "4"))
    default_oct = int(defaults.get("o", "5"))
    bpm = int(defaults.get("b", "120"))

    notes: List[Tuple[Optional[int], float]] = []
    for raw in data.split(","):
        token = raw.strip()
        if not token:
            continue
        # Arduino-вариант «4.f5» — точка сразу после длительности: переносим
        # её в конец токена, чтобы один regex понимал оба диалекта.
        if re.match(r"^\d+\.", token):
            token = token.replace(".", "", 1) + "."
        match = _TOKEN_RE.match(token)
        if not match:
            raise ValueError(f"Не удалось разобрать токен RTTTL: {token!r}")
        dur = int(match.group("dur") or default_dur)
        if dur <= 0:
            raise ValueError(f"Недопустимая длительность {dur} в токене RTTTL: {token!r}")
        note = match.group("note").lower()
        acc = match.group("acc")
        octave = int(match.group("oct") or match.group("oct2") or default_oct)
        dotted = bool(match.group("dot") or match.group("dot2"))
        beats = 4.0 / dur * (1.5 if dotted else 1.0)
        midi = None if note == "p" else _to_midi(note, acc, octave)
        notes.append((midi, beats))
    return name, bpm, notes


#: Доля нот с явной октавой, с которой мелодия «с явными октавами» (#3580): остальные 30 % — вероятно, пропуск автора.
OCTAVE_EXPLICIT_SHARE = 0.7
#: Скачок в полутонах, с которого нота без октавы считается ошибкой: целая октава и больше, с возвратом.
OCTAVE_LEAP = 12
#: Сколько подряд идущих нот без октавы правится как одна ошибка (пара ``32d_,e`` на октаву выше соседей).
OCTAVE_RUN_MAX = 2
#: Больше этой доли звучащих нот править нельзя: столько «выбросов» — уже приём автора (перекличка октав), не опечатки.
OCTAVE_FIX_MAX_SHARE = 0.2


def _sounding_tokens(rtttl: str):
    """``(имя, настройки, [токены], default_oct)``: токены — ``None`` (пустой) или ``(raw, match)``; ``None``, если
    строка не RTTTL (вызывающий оставляет её как есть — ошибку даст ``parse_rtttl``)."""
    try:
        name, settings, data = rtttl.strip().split(":", 2)
    except ValueError:
        return None
    default_oct = 5
    for kv in settings.split(","):
        key, _, value = kv.partition("=")
        if key.strip().lower() == "o" and value.strip().isdigit():
            default_oct = int(value)
    tokens = []
    for raw in data.split(","):
        token = raw.strip()
        if re.match(r"^\d+\.", token):
            token = token.replace(".", "", 1) + "."
        match = _TOKEN_RE.match(token) if token else None
        if token and not match:
            return None
        tokens.append(match)
    return name, settings, tokens, default_oct


def _octave_of(match) -> Optional[int]:
    raw = match.group("oct") or match.group("oct2")
    return int(raw) if raw else None


def _spiked(pitches: Sequence[int], first: int, last: int) -> bool:
    """Нота (или пара) ``pitches[first..last]`` — выброс: скачок ≥ октавы от предыдущей ноты, и следующая нота
    вернулась в пределы октавы от предыдущей (вверх и назад — или вниз и назад)."""
    if first < 1 or last + 1 >= len(pitches):
        return False
    into, back = pitches[first] - pitches[first - 1], pitches[last + 1] - pitches[first - 1]
    return abs(into) >= OCTAVE_LEAP and abs(back) < OCTAVE_LEAP


def _implicit_runs(sounding) -> List[Tuple[int, int]]:
    """Серии подряд идущих звучащих нот без октавы: ``[(первая, последняя)]``."""
    runs: List[Tuple[int, int]] = []
    for n, match in enumerate(sounding):
        if _octave_of(match) is not None:
            continue
        if runs and runs[-1][1] == n - 1:
            runs[-1] = (runs[-1][0], n)
        else:
            runs.append((n, n))
    return runs


def _octave_fixes(sounding, default_oct: int) -> dict:
    """``{id(токен): октава}`` для нот, где пропуск октавы — выброс (пусто — править нечего или нельзя)."""
    implicit = sum(_octave_of(m) is None for m in sounding)
    if not implicit or len(sounding) - implicit < OCTAVE_EXPLICIT_SHARE * len(sounding):
        return {}
    pitches = [_to_midi(m.group("note").lower(), m.group("acc"), _octave_of(m) or default_oct) for m in sounding]
    found = []
    for first, last in _implicit_runs(sounding):
        signature = frozenset(pitches[first:last + 1])
        found.append((signature, _fix_run(sounding, pitches, first, last)))
    fixes: dict = {}
    for signature, fix in found:
        # «выброс» той же высоты (класс ноты и октава) в двух и более местах — приём автора, опечатка одиночна
        if fix and not any(sum(p in sig and bool(f) for sig, f in found) > 1 for p in signature):
            fixes.update(fix)
    return fixes if len(fixes) <= OCTAVE_FIX_MAX_SHARE * len(sounding) else {}


def fix_octaves(rtttl: str, label: str = "") -> str:
    """Исправить в RTTTL ноты без октавы, которые автор пропустил (#3580): у них октава по умолчанию ``o=``, а
    соседи записаны явно и в другой («Celebrat»: ``o=6``, вся мелодия ``…e5,32c_5,32d5``, но ``32d_`` и ``c`` без
    октавы — на октаву выше, в хуке скачок 66→81→70).

    Правка — только если у ≥ :data:`OCTAVE_EXPLICIT_SHARE` звучащих нот октава явная, нот без октавы подряд не больше
    :data:`OCTAVE_RUN_MAX`, они стоят после явной ноты и дают выброс (скачок ≥ :data:`OCTAVE_LEAP` полутонов от
    предыдущей ноты, а следующая нота возвращается в пределы октавы от неё), который исчезает от октавы предыдущей
    ноты. Не меняются: мелодии, где октава по умолчанию — норма (почти все ноты без октавы); ноты без выброса;
    «выброс» той же высоты, повторённый в мелодии ≥ 2 раз (педаль, перекличка — приём автора, опечатка одиночна);
    мелодия, где правок больше :data:`OCTAVE_FIX_MAX_SHARE` нот. Возвращает строку RTTTL (та же, если править нечего);
    в лог — сколько нот исправлено.
    """
    parsed = _sounding_tokens(rtttl)
    if parsed is None:
        return rtttl
    name, settings, tokens, default_oct = parsed
    fixes = _octave_fixes([m for m in tokens if m is not None and m.group("note").lower() != "p"], default_oct)
    if not fixes:
        return rtttl
    _LOG.info("🎵 [music v2] hook melody=%s исправил октаву %d нот", label or name, len(fixes))
    return f"{name}:{settings}:" + ",".join(_with_octave(m, fixes) for m in tokens if m is not None)


def _fix_run(sounding, pitches: list, first: int, last: int) -> dict:
    """Октавы для серии нот без октавы ``first..last`` (``{id(токен): октава}``; пусто — править нельзя). ``pitches``
    обновляется на месте, когда правка принята."""
    if first < 1 or last - first + 1 > OCTAVE_RUN_MAX or not _spiked(pitches, first, last):
        return {}
    target = _octave_of(sounding[first - 1])
    trial = list(pitches)
    for n in range(first, last + 1):
        trial[n] = _to_midi(sounding[n].group("note").lower(), sounding[n].group("acc"), target)
    if trial[first] == pitches[first] or _spiked(trial, first, last):
        return {}
    pitches[first:last + 1] = trial[first:last + 1]
    return {id(sounding[n]): target for n in range(first, last + 1)}


def _with_octave(match, octaves: dict) -> str:
    """Токен как был; у исправленного — с проставленной октавой."""
    if id(match) not in octaves:
        return match.group(0)
    dot = "." if (match.group("dot") or match.group("dot2")) else ""
    return f"{match.group('dur')}{match.group('note')}{match.group('acc')}{octaves[id(match)]}{dot}"


def contour(rtttl: str, notes: int) -> Optional[Tuple[int, ...]]:
    """Контур начала мелодии (#3427): интервалы между первыми ``notes`` нотами — без транспозиции, пауз и
    длительностей: у двух версий одной темы в разных тональностях и ритмической записи он один. Нот меньше
    ``notes`` или строка не RTTTL — ``None``."""
    try:
        _name, _bpm, events = parse_rtttl(rtttl)
    except ValueError:
        return None
    pitches = [midi for midi, _beats in events if midi is not None][:notes]
    if len(pitches) < notes:
        return None
    return tuple(b - a for a, b in zip(pitches, pitches[1:]))


#: Сколько найденных по теме мелодий получает профиль сета (кандидаты хука и LLM); у темы-перечисления — на часть.
THEME_HOOKS = 8
#: Нот в контуре начала для «консенсуса версий» (#3427): при 7 версии «Terminator» theme_177/theme_178 совпадают,
#: а повторные ноты «Space Quest» и «Exploration Of Space» уже различаются (при 6 — нет; при 8 расходятся 177/178).
CONTOUR_NOTES = 7


def consensus_order(hits: Sequence[Tuple[float, Dict[str, Any]]], limit: int = THEME_HOOKS) -> List[str]:
    """Первые ``limit`` найденных записей ``(confidence, запись с rtttl)`` — в порядке хуков темы (#3427, «консенсус
    версий»); набор тот же, меняется только порядок.

    Внутри одной доли слов: сначала мелодии, чей контур начала (:func:`rob_box_music.rtttl.contour`,
    :data:`CONTOUR_NOTES` нот) есть ещё хотя бы у одной найденной записи (считаются все ``hits``), — по одной версии
    на контур, затем их повторные версии, затем одиночные; при равенстве — порядок поиска (ближе к названию).
    Узнаваемая тема лежит в архиве в нескольких версиях («Terminator» theme_177/theme_178: d e f e c f), случайный
    рингтон с тем же словом в названии — в одной (terminat)."""
    contours = [contour(str(r.get("rtttl") or ""), CONTOUR_NOTES) for _c, r in hits]
    copies = Counter(c for c in contours if c is not None)
    versions: Counter = Counter()
    keys = []
    for i, ((confidence, _r), shape) in enumerate(zip(hits[:limit], contours)):
        single = shape is None or copies[shape] < 2
        keys.append((-confidence, single, 0 if single else versions[shape], i))
        versions[shape] += 1
    return [hits[i][1]["name"] for *_rank, i in sorted(keys)]
