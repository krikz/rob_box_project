#!/usr/bin/env python3
"""scene_spec.py — разбор ``scene.yaml`` набора сцен (ADR-0144 §4.2).

Чистая логика, без ROS: участники, таймлайн событий, визиты (пары
``enter..leave``), окна речи (``say``), разрешённые имена во времени
(ADR-0144 §6.1). Этим модулем пользуются считалка метрик (``metrics.py``)
и дирижёр (``conductor.py``, при сборке ``scene.yaml``).

Ошибка формата — ``SceneSpecError`` с указанием поля: разметка, которую
нельзя прочитать однозначно, не должна молча превращаться в цифры.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

SCHEMA_VERSION = 1

KINDS = ("person", "mask", "photo")
#: Участники этих видов в кадре законно стоят на месте (подставка / рука
#: с телефоном): для них задаётся коридор ``anchor_cx``.
STATIC_KINDS = ("mask", "photo")

PRESENCE_TYPES = ("enter", "leave")
#: Типы событий, у которых обязателен ``who``.
PARTICIPANT_EVENT_TYPES = ("enter", "leave", "say", "introduce")
#: Справочные типы — пишутся дирижёром, в метрики не входят.
INFO_EVENT_TYPES = (
    "glasses_on", "glasses_off", "hat_on", "hat_off",
    "turn_back", "turn_front", "note", "consent",
)
EVENT_TYPES = ("empty",) + PARTICIPANT_EVENT_TYPES + INFO_EVENT_TYPES

#: Сцены, которые нельзя снять без живого второго человека (ADR-0144 §3.2).
#: Отчёт печатает их как ``not covered`` всегда.
NOT_COVERED = (
    "guest_glasses: гость в очках",
    "two_live_crossing: двое живых людей пересекаются",
    "guest_from_behind: гость со спины",
)


class SceneSpecError(ValueError):
    """Разметку сцены нельзя прочитать однозначно."""


@dataclass(frozen=True)
class Participant:
    label: str
    truth: str
    kind: str = "person"
    known_before: bool = False
    present_at_start: bool = False
    anchor_cx: Optional[Tuple[float, float]] = None

    @property
    def is_static(self) -> bool:
        return self.kind in STATIC_KINDS


@dataclass(frozen=True)
class Event:
    t: float
    type: str
    who: str = ""
    until: Optional[float] = None
    name: str = ""
    text: str = ""


@dataclass(frozen=True)
class Visit:
    """Одно пребывание участника в кадре: ``start..end`` (секунды бэга)."""

    who: str
    start: float
    end: float
    #: Перед входом кадр был проверенно пуст (метка ``empty``, ADR-0144 §4.1).
    verified_empty: bool

    def contains(self, t: float, grace: float = 0.0) -> bool:
        return self.start - grace <= t <= self.end + grace


@dataclass(frozen=True)
class NameWindow:
    start: float
    names: Tuple[str, ...]


@dataclass
class Scene:
    scene: str
    participants: Dict[str, Participant]
    events: List[Event]
    duration: float
    expected: Dict[str, Any] = field(default_factory=dict)
    consent: List[Dict[str, Any]] = field(default_factory=list)
    recording: Dict[str, Any] = field(default_factory=dict)


# ── разбор ───────────────────────────────────────────────────────────────────


def _require(d: Mapping[str, Any], key: str, where: str) -> Any:
    if key not in d or d[key] in (None, ""):
        raise SceneSpecError(f"{where}: нет поля {key!r}")
    return d[key]


def _parse_anchor(raw: Any, where: str) -> Optional[Tuple[float, float]]:
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise SceneSpecError(f"{where}.anchor_cx: ожидается [lo, hi]")
    lo, hi = float(raw[0]), float(raw[1])
    if not 0.0 <= lo < hi <= 1.0:
        raise SceneSpecError(f"{where}.anchor_cx: нужно 0 <= lo < hi <= 1, дано {raw}")
    return lo, hi


def parse_participant(raw: Mapping[str, Any], idx: int) -> Participant:
    where = f"participants[{idx}]"
    kind = str(raw.get("kind", "person"))
    if kind not in KINDS:
        raise SceneSpecError(f"{where}.kind: {kind!r} не из {KINDS}")
    return Participant(
        label=str(_require(raw, "label", where)),
        truth=str(_require(raw, "truth", where)),
        kind=kind,
        known_before=bool(raw.get("known_before", False)),
        present_at_start=bool(raw.get("present_at_start", False)),
        anchor_cx=_parse_anchor(raw.get("anchor_cx"), where),
    )


def parse_event(raw: Mapping[str, Any], idx: int, labels: Sequence[str]) -> Event:
    where = f"events[{idx}]"
    etype = str(_require(raw, "type", where))
    if etype not in EVENT_TYPES:
        raise SceneSpecError(f"{where}.type: {etype!r} неизвестен")
    t = float(_require(raw, "t", where))
    if not math.isfinite(t) or t < 0:
        raise SceneSpecError(f"{where}.t: {t} — нужно конечное число >= 0")
    who = str(raw.get("who") or "")
    if etype in PARTICIPANT_EVENT_TYPES and who not in labels:
        raise SceneSpecError(f"{where}.who: {who!r} нет среди участников {list(labels)}")
    until = raw.get("until")
    if etype == "say":
        if until is None or float(until) < t:
            raise SceneSpecError(f"{where}: у say нужно until >= t")
        until = float(until)
    name = str(raw.get("name") or "")
    if etype == "introduce" and not name:
        raise SceneSpecError(f"{where}: у introduce нужно name")
    return Event(t=t, type=etype, who=who, until=until, name=name, text=str(raw.get("text") or ""))


def parse_scene(raw: Mapping[str, Any]) -> Scene:
    """dict из ``scene.yaml`` → ``Scene``. Все проверки формата — здесь."""
    if not isinstance(raw, Mapping):
        raise SceneSpecError("scene.yaml: ожидается словарь")
    schema = int(raw.get("schema", SCHEMA_VERSION))
    if schema != SCHEMA_VERSION:
        raise SceneSpecError(f"schema: {schema}, поддерживается {SCHEMA_VERSION}")
    name = str(_require(raw, "scene", "scene.yaml"))
    plist = [parse_participant(p, i) for i, p in enumerate(raw.get("participants") or [])]
    if not plist:
        raise SceneSpecError("participants: пусто")
    participants: Dict[str, Participant] = {}
    for p in plist:
        if p.label in participants:
            raise SceneSpecError(f"participants: label {p.label!r} повторяется")
        participants[p.label] = p
    events = sorted(
        (parse_event(e, i, list(participants)) for i, e in enumerate(raw.get("events") or [])),
        key=lambda e: e.t,
    )
    last_t = max([e.t for e in events] + [e.until or 0.0 for e in events] + [0.0])
    duration = float(raw.get("duration") or last_t)
    if duration < last_t:
        raise SceneSpecError(f"duration {duration} < последнего события {last_t}")
    expected = dict(raw.get("expected") or {})
    _check_expected(expected, participants)
    return Scene(
        scene=name,
        participants=participants,
        events=events,
        duration=duration,
        expected=expected,
        consent=list(raw.get("consent") or []),
        recording=dict(raw.get("recording") or {}),
    )


def _check_expected(expected: Mapping[str, Any], participants: Mapping[str, Participant]) -> None:
    for key in ("allowed_names", "max_greetings_per_visit", "max_records"):
        for label in (expected.get(key) or {}):
            if label not in participants:
                raise SceneSpecError(f"expected.{key}: участника {label!r} нет")


def load_scene(path: str) -> Scene:
    import yaml  # лениво: чистые функции выше yaml не требуют

    with open(path, encoding="utf-8") as f:
        return parse_scene(yaml.safe_load(f))


# ── производные: визиты, окна речи, разрешённые имена ────────────────────────


def _verified_before(events: Sequence[Event], enter: Event, static: Sequence[str]) -> bool:
    """Был ли перед этим входом проверенно пустой кадр.

    Ищется метка ``empty`` не позже входа, после которой до этого входа
    не входил никто из нестатичных участников (иначе «пусто» относилось
    к другому моменту сцены).
    """
    for e in reversed([e for e in events if e.t <= enter.t and e is not enter]):
        if e.type == "empty":
            return True
        if e.type == "enter" and e.who not in static:
            return False
    return False


def visits(scene: Scene) -> List[Visit]:
    """Пары ``enter..leave`` каждого участника; открытый визит закрывается концом сцены.

    Двойной ``enter`` без ``leave`` и ``leave`` без ``enter`` — ошибка
    разметки (дирижёр так не пишет), а не повод угадывать.
    """
    static = [p.label for p in scene.participants.values() if p.is_static]
    open_at: Dict[str, Tuple[float, bool]] = {
        p.label: (0.0, False) for p in scene.participants.values() if p.present_at_start
    }
    out: List[Visit] = []
    for e in scene.events:
        if e.type == "enter":
            if e.who in open_at:
                raise SceneSpecError(f"enter {e.who!r} t={e.t}: участник уже в кадре")
            open_at[e.who] = (e.t, _verified_before(scene.events, e, static))
        elif e.type == "leave":
            if e.who not in open_at:
                raise SceneSpecError(f"leave {e.who!r} t={e.t}: участника нет в кадре")
            start, verified = open_at.pop(e.who)
            out.append(Visit(e.who, start, e.t, verified))
    for who, (start, verified) in open_at.items():
        out.append(Visit(who, start, scene.duration, verified))
    return sorted(out, key=lambda v: (v.start, v.who))


def say_windows(scene: Scene) -> List[Event]:
    return [e for e in scene.events if e.type == "say"]


def name_windows(scene: Scene, label: str) -> List[NameWindow]:
    """Разрешённые имена участника как ступенчатая функция времени (ADR-0144 §6.1).

    ``expected.allowed_names[label]`` заменяет правила целиком; иначе
    ``known_before`` → ``{truth}`` с нуля, ``introduce`` добавляет имя с
    момента события. ``photo`` без явного ``expected`` не называется никак.
    """
    explicit = (scene.expected.get("allowed_names") or {}).get(label)
    if explicit is not None:
        wins = [NameWindow(float(w.get("from", 0.0)), tuple(w.get("names") or ())) for w in explicit]
        return sorted(wins, key=lambda w: w.start)
    p = scene.participants[label]
    if p.kind == "photo":
        return [NameWindow(0.0, ())]
    names: List[str] = [p.truth] if p.known_before else []
    wins = [NameWindow(0.0, tuple(names))]
    for e in scene.events:
        if e.type == "introduce" and e.who == label and e.name not in names:
            names = names + [e.name]
            wins.append(NameWindow(e.t, tuple(names)))
    return wins


def allowed_names(windows: Sequence[NameWindow], t: float) -> Tuple[str, ...]:
    current: Tuple[str, ...] = ()
    for w in windows:
        if w.start <= t:
            current = w.names
    return current


def first_allowed_at(windows: Sequence[NameWindow], start: float, end: float) -> Optional[float]:
    """Первый момент в ``[start, end]``, когда участнику разрешено хоть одно имя."""
    if allowed_names(windows, start):
        return start
    for w in windows:
        if start < w.start <= end and w.names:
            return w.start
    return None


def expected_int(scene: Scene, key: str, label: str, default: int) -> int:
    return int((scene.expected.get(key) or {}).get(label, default))
