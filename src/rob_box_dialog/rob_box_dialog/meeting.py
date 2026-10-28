"""Встреча лицом (ADR-0123 §3): маркер из ``VisionEvent`` и приветствие знакомого после долгой разлуки.

Перенесено из v1 ``rob_box_voice/core/meeting.py`` как есть (решение Шифу 23.10 D1): v1 импортирует отсюда,
реализация одна. Приветствие вернули по решению товарища Шифу 21.10 (~11:10Z): «когда он меня не видел давно, он
должен поздороваться» — это главная фича, и она ИСКЛЮЧЕНИЕ из В12′ (робот сам в разговор не вступает; ADR-0150
§18.3). Рамки исключения: одна фраза по имени и времени суток, без вейка, без вопроса и без продолжения разговора.
Незнакомое и безымянное лицо не приветствуется вовсе: вопрос об имени — только по В17 (после N фраз диалога).
Фразу строит код (ADR-0148, тексты — ``phrases/ru.yaml``), не LLM: приветствие обязано звучать, даже когда у облака
кончились деньги.
"""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases

#: Порог «давно не виделись» (``knowledge.MEETING_LONG_ABSENCE_S``). Ниже — тот же день, молчим.
DEFAULT_LONG_ABSENCE_SEC = kn.MEETING_LONG_ABSENCE_S

#: Кулдаун повода ``meeting`` в v1 ``OccasionGate`` (как раньше). В v1 он не срабатывал: ``mark_consumed`` по
#: встрече не звался. Держится для v1 до PR-15.
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
    #: Реальный источник кадра из VisionEvent (стаб-фильтр v1 ``OccasionGate`` и :func:`parse_meeting_marker`).
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

    Возвращает ``None`` (а не бросает) на любом мусоре: топик ``/vision/hailo/events`` общий с person-детекцией, и
    сюда приезжает много того, что Встречей не является.

    **Выдумка не проходит** (ADR-0089 §2.2): ``source_camera`` «stub»/«unknown» отсекается здесь же — иначе
    Личность начнёт здороваться с несуществующими людьми, а это хуже, чем молчать. Пустая строка означает «источник
    не передали» — это не утверждение о стабе, поэтому не отсекаем.
    """
    if event_type != 'face':
        return None
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
        part = 'morning'
    elif 12 <= hour < 18:
        part = 'day'
    elif 18 <= hour < 23:
        part = 'evening'
    else:
        part = 'night'
    return phrases.text(f'meeting.tod.{part}')


class MeetingGreeter:
    """Решает, здороваться ли с этим человеком и какими словами.

    Здороваемся только с ЗНАКОМЫМ ПО ИМЕНИ, которого не видели ``long_absence_sec``. «Видели» — любой маркер
    Встречи, не только приветствие: вышел и вернулся через час — это тот же день, молчим. После перезапуска робота
    памяти нет: знакомый с ``encounter_count > 1`` считается «давно не виделись» (один раз за запуск). Безымянные и
    новые лица — никогда.
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
        text, _ = phrases.pick('meeting.long_absence', rng=self._rng, tod=tod, name=marker.name.strip())
        return text


def greet_on_meeting(
    greeter: MeetingGreeter,
    marker: MeetingMarker,
    *,
    idle: bool,
    allowed: Callable[[MeetingMarker], bool],
    speak: Callable[[str], Any],
) -> Optional[str]:
    """Встреча → максимум одна фраза-приветствие. Вернуть сказанное или ``None``.

    ``idle`` — диалог не идёт (перебивать говорящего хуже, чем промолчать). ``allowed`` — шов «можно ли заговорить».
    Если приветствие положено, но заблокировано, человека НЕ отмечаем «виденным» — следующая Встреча попробует
    снова. Во всех остальных случаях отмечаем.
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


def unprompted_quiet(snapshot: Any, turn_busy: bool) -> bool:
    """Роботу сейчас нельзя заговорить самому (v2-аналог «автомат диалога не в IDLE» v1): идёт ход или в стеке сцен
    тишина/удержание оператором (``knowledge.UNPROMPTED_QUIET_SCENES``; в v1 оба — SILENCED)."""
    return bool(turn_busy) or any(snapshot.top(kind) is not None for kind in kn.UNPROMPTED_QUIET_SCENES)


__all__ = ["DEFAULT_LONG_ABSENCE_SEC", "MEETING_SOURCE_COOLDOWN_SEC", "MeetingGreeter", "MeetingMarker",
           "greet_on_meeting", "parse_meeting_marker", "time_of_day_greeting", "unprompted_quiet"]
