#!/usr/bin/env python3
"""Маркер Встречи лицом из ``VisionEvent`` (ADR-0123 §3) — для шва лица ADR-0139.

Раньше модуль ещё и выбирал приветствие («Денис вошёл → робот здоровается первым», issue #2599). По решению
товарища Шифу В12′ (ADR-0150 §18.3, 21.10) робот действует только по вейк-слову и сам в разговор не вступает:
приветствие Встречи удалено вместе с выбором фразы (#3695). Остался разбор маркера: ``dialogue_node`` кладёт
лицо Встречи в шов «Знакомый» (``note_face_seen``, ADR-0139 §2.3).

Модуль без ROS — тестируется напрямую.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MeetingMarker:
    """Разобранный маркер Встречи из ``VisionEvent.attributes_json``."""

    person_id: str
    name: str
    is_new: bool
    similarity: float
    encounter_count: int
    face_px: float
    privacy_mode: str
    #: Реальный источник кадра из VisionEvent. Нужен не для красоты:
    #: ``OccasionGate`` держит СВОЙ стаб-фильтр по ``source_camera``, и
    #: если подставить сюда постоянное «oak_d», этот фильтр перестанет
    #: работать — останется только наш собственный (см.
    #: ``parse_meeting_marker``). Два независимых заслона от выдуманных
    #: людей лучше одного.
    source_camera: str = ''

    @property
    def is_named(self) -> bool:
        return bool(self.name.strip())


def parse_meeting_marker(
    event_type: str,
    attributes_json: str,
    source_camera: str = '',
) -> Optional[MeetingMarker]:
    """Вернуть маркер Встречи или ``None``, если это рядовое событие.

    Возвращает ``None`` (а не бросает) на любом мусоре: топик
    ``/vision/hailo/events`` общий с person-детекцией, и сюда приезжает
    много того, что Встречей не является.

    **Выдумка не проходит** (ADR-0089 §2.2, #2583): ``event_type="stub"``
    отсекается здесь же — иначе Личность начнёт здороваться с
    несуществующими людьми, а это хуже, чем молчать.
    """
    if event_type != 'face':
        return None
    # Лицевая нода в real-режиме всегда проставляет frame_id камеры;
    # «stub»/«unknown» — признак выдуманного события (тот же стаб-фильтр,
    # что у OccasionGate для person). Пустая строка означает «источник не
    # передали» — это не утверждение о стабе, поэтому не отсекаем.
    if source_camera in ('stub', 'unknown'):
        return None
    if not attributes_json:
        return None
    try:
        payload = json.loads(attributes_json)
    except (ValueError, TypeError):
        return None
    if not isinstance(payload, dict) or payload.get('encounter') != 'start':
        return None
    person_id = str(payload.get('person_id') or '')
    if not person_id:
        return None
    return MeetingMarker(
        person_id=person_id,
        name=str(payload.get('name') or ''),
        is_new=bool(payload.get('is_new', False)),
        similarity=float(payload.get('similarity') or 0.0),
        encounter_count=int(payload.get('encounter_count') or 0),
        face_px=float(payload.get('face_px') or 0.0),
        privacy_mode=str(payload.get('privacy_mode') or ''),
        source_camera=source_camera,
    )
