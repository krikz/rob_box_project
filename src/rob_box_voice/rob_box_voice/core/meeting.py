#!/usr/bin/env python3
"""Встреча лицом (ADR-0123 §3): маркер из ``VisionEvent`` и приветствие знакомого после долгой разлуки.

Приветствие вернули по решению товарища Шифу 21.10 (~11:10Z): «когда он меня не видел давно, он должен
поздороваться» — это главная фича, и она ИСКЛЮЧЕНИЕ из В12′ (робот сам в разговор не вступает; ADR-0150 §18.3).
Рамки исключения: одна фраза по имени и времени суток, без вейка, без вопроса и без продолжения разговора.
Незнакомое и безымянное лицо не приветствуется вовсе: вопрос об имени — только по В17 (после N фраз диалога).
Фразу строит код (ADR-0148), не LLM: приветствие обязано звучать, даже когда у облака кончились деньги.

Модуль без ROS — тестируется напрямую.
"""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

#: Порог «давно не виделись». Ниже — тот же день, молчим.
DEFAULT_LONG_ABSENCE_SEC = 20 * 3600.0

#: Кулдаун повода ``meeting`` в ``OccasionGate`` (как до #3760): двое вошедших вместе не слипаются в одно.
MEETING_SOURCE_COOLDOWN_SEC = 5.0


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


def time_of_day_greeting(hour: int) -> str:
    """«Доброе утро» / «Добрый день» / «Добрый вечер» / «Доброй ночи»."""
    if 5 <= hour < 12:
        return 'Доброе утро'
    if 12 <= hour < 18:
        return 'Добрый день'
    if 18 <= hour < 23:
        return 'Добрый вечер'
    return 'Доброй ночи'


#: Знакомый, которого давно не было. Только такие фразы: «обычного» приветствия нет (не повод заговорить).
_LONG_ABSENCE_PHRASES = (
    '{tod}, {name}! Сколько лет, сколько зим.',
    '{name}! Давно тебя не видел. {tod}.',
    'О, {name}! А я уж думал, ты про меня забыл. {tod}.',
)


class MeetingGreeter:
    """Решает, здороваться ли с этим человеком и какими словами.

    Здороваемся только с ЗНАКОМЫМ ПО ИМЕНИ, которого не видели ``long_absence_sec``. «Видели» — любой маркер
    Встречи, не только приветствие: вышел и вернулся через час — это тот же день, молчим. После
    перезапуска робота памяти нет: знакомый с ``encounter_count > 1`` считается «давно не виделись»
    (один раз за запуск). Безымянные и новые лица — никогда.
    """

    def __init__(
        self,
        *,
        long_absence_sec: float = DEFAULT_LONG_ABSENCE_SEC,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], Any] = time.localtime,
        rng: Any = None,
    ) -> None:
        self._long_absence_sec = float(long_absence_sec)
        self._clock = clock
        self._wall_clock = wall_clock
        self._rng = rng or random.Random()
        self._last_seen_at: Dict[str, float] = {}

    def due(self, marker: MeetingMarker) -> bool:
        """Пора ли здороваться (без побочных эффектов)."""
        if not marker.is_named:
            return False
        last = self._last_seen_at.get(marker.person_id)
        if last is None:
            return marker.encounter_count > 1
        return (self._clock() - last) >= self._long_absence_sec

    def note_seen(self, marker: MeetingMarker) -> None:
        """Запомнить, что видели человека сейчас."""
        self._last_seen_at[marker.person_id] = self._clock()

    def compose(self, marker: MeetingMarker) -> str:
        """Фраза по имени и времени суток."""
        tod = time_of_day_greeting(self._wall_clock().tm_hour)
        return self._rng.choice(_LONG_ABSENCE_PHRASES).format(tod=tod, name=marker.name.strip())


def greet_on_meeting(
    greeter: MeetingGreeter,
    marker: MeetingMarker,
    *,
    idle: bool,
    allowed: Callable[[MeetingMarker], bool],
    speak: Callable[[str], Any],
) -> Optional[str]:
    """Встреча → максимум одна фраза-приветствие. Вернуть сказанное или ``None``.

    ``idle`` — диалог не идёт (перебивать говорящего хуже, чем промолчать). ``allowed`` — шов «можно ли
    заговорить» (``OccasionGate``). Если приветствие положено, но заблокировано, человека НЕ отмечаем
    «виденным» — следующая Встреча попробует снова. Во всех остальных случаях отмечаем.
    """
    if not greeter.due(marker):
        greeter.note_seen(marker)
        return None
    if not idle or not allowed(marker):
        return None
    phrase = greeter.compose(marker)
    greeter.note_seen(marker)
    speak(phrase)
    return phrase


def greet_node_on_meeting(node: Any, marker: MeetingMarker) -> None:
    """Обвязка :func:`greet_on_meeting` для ``DialogueNode`` (держит ROS-часть вне класса, ADR-0145).

    Ошибка приветствия не должна ронять колбэк лица (ADR-0139 §2.5): логируем и идём дальше.
    """
    from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
    from rob_box_voice.core.greeting_window import open_window
    from rob_box_voice.core.occasion import Occasion, VerdictKind

    def allowed(m: MeetingMarker) -> bool:
        occasion = Occasion(kind='meeting', is_user_initiated=False, payload={
            'event_type': 'face', 'source_camera': m.source_camera, 'person_id': m.person_id, 'name': m.name})
        return node._occasion.may_speak(occasion).kind == VerdictKind.ALLOW

    spoken: Dict[str, str] = {}

    def speak(text: str) -> None:
        spoken['speech_id'] = node._publish_response(text, animation='happy')

    try:
        phrase = greet_on_meeting(
            node._meeting_greeter, marker,
            idle=node._dsm.current_state == DialogueStateKind.IDLE, allowed=allowed, speak=speak,
        )
        if phrase:
            # #3776: после приветствия окно 15 с — «привет» в ответ без «робот» (кадр ждёт конца речи).
            open_window(node, marker.person_id, marker.name.strip(), str(spoken.get('speech_id') or ''))
            node.get_logger().info(f'🗣 [встреча] {marker.name} (person={marker.person_id[:8]}): {phrase!r}')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warning(f'⚠️ [встреча] приветствие упало: {exc!r}')
