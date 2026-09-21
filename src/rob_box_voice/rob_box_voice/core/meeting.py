#!/usr/bin/env python3
"""Встреча лицом — повод заговорить первым (ADR-0102 «Повод», ADR-0123).

Сценарий-заказчик issue #2599 целиком: «Денис входит в мастерскую →
робот замечает событие → поднимает биометрию → понимает, кто это →
знает, что давно не виделись → **заговаривает первым**, с учётом времени
суток». До этого модуля последний шаг был единственным неработающим:
``Occasion(kind="meeting")`` был описан в ``core/occasion.py``, но его
никто не поднимал — робот молчал, пока с ним не заговорят.

Модуль отвечает на два вопроса и больше ни на что:

1. **Это вообще Встреча?** — ``parse_meeting_marker``. Лицевая нода
   публикует ``VisionEvent`` по ~5 штук в секунду на человека, но
   маркер ``{"encounter": "start"}`` в ``attributes_json`` ставит ровно
   один раз на трек (ADR-0123 §3). Реагировать надо на маркер, иначе
   робот поздоровается пять раз в секунду.

2. **Что сказать?** — :class:`MeetingGreeter`. Фраза зависит от того,
   знаем ли мы имя, впервые ли видим человека и который час.

Ни ROS, ни LLM, ни TTS здесь нет: это чистая логика, которую можно
прогнать тестами без робота. Фраза намеренно НЕ идёт через LLM —
приветствие обязано звучать, даже когда у облака кончились деньги
(голосовой стек уже деградировал молча, ADR-0123 §7 прямо требует так
не делать).
"""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

#: Сколько молчать про одного и того же человека. Он вышел за дверь и
#: вернулся через минуту — это не новая встреча, это тот же разговор.
DEFAULT_PERSON_COOLDOWN_SEC = 900.0

#: Порог «давно не виделись» — сутки. Ниже — обычное приветствие.
DEFAULT_LONG_ABSENCE_SEC = 20 * 3600.0


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
    """«Доброе утро» / «Добрый день» / «Добрый вечер» / «Доброй ночи».

    Сценарий issue #2599 требует «с учётом времени суток» явно.
    """
    if 5 <= hour < 12:
        return 'Доброе утро'
    if 12 <= hour < 18:
        return 'Добрый день'
    if 18 <= hour < 23:
        return 'Добрый вечер'
    return 'Доброй ночи'


#: Знакомый, виделись недавно.
_KNOWN_PHRASES = (
    '{tod}, {name}!',
    'О, {name}! {tod}.',
    '{name}, привет!',
    'А вот и {name}. {tod}!',
)

#: Знакомый, которого давно не было.
_KNOWN_LONG_ABSENCE_PHRASES = (
    '{tod}, {name}! Сколько лет, сколько зим.',
    '{name}! Давно тебя не видел. {tod}.',
    'О, {name}! А я уж думал, ты про меня забыл. {tod}.',
)

#: Лицо знакомое, имени не знаем (ADR-0123: в мастерской пишутся все
#: подряд, имя привязывается потом).
_UNNAMED_RETURNING_PHRASES = (
    'О, снова ты! {tod}. Мы ведь уже виделись, а я так и не знаю, как тебя зовут.',
    '{tod}! Твоё лицо мне знакомо, но имя ты мне так и не сказал.',
    'Опять ты! {tod}. Может, всё-таки представишься?',
)

#: Совсем новый человек.
_NEW_PERSON_PHRASES = (
    '{tod}! Кажется, мы ещё не знакомы. Как тебя зовут?',
    'О, новое лицо! {tod}. Я тебя раньше не видел — как тебя звать?',
    '{tod}! Мы не встречались, верно? Представься, я запомню.',
)


class MeetingGreeter:
    """Решает, здороваться ли с этим человеком и какими словами.

    Кулдаун — per-person, а не глобальный: если в мастерскую зашли двое,
    робот должен поздороваться с обоими, но не здороваться с каждым по
    десять раз, пока они ходят туда-сюда мимо камеры.

    ``OccasionGate`` (ADR-0102) остаётся выше по стеку и решает вопрос
    «а вообще можно ли сейчас говорить» (идёт ли диалог, играет ли
    музыка, не частим ли). Здесь — только «стоит ли оно того».

    Args:
        person_cooldown_sec: сколько молчать про одного человека.
        long_absence_sec: с какого перерыва фраза становится «давно не виделись».
        clock: источник времени (монотонный) — для тестов.
        wall_clock: источник локального времени суток — для тестов.
        rng: генератор для выбора фразы — для тестов.
    """

    def __init__(
        self,
        *,
        person_cooldown_sec: float = DEFAULT_PERSON_COOLDOWN_SEC,
        long_absence_sec: float = DEFAULT_LONG_ABSENCE_SEC,
        clock: Any = time.monotonic,
        wall_clock: Any = time.localtime,
        rng: Any = None,
    ) -> None:
        self._person_cooldown_sec = float(person_cooldown_sec)
        self._long_absence_sec = float(long_absence_sec)
        self._clock = clock
        self._wall_clock = wall_clock
        self._rng = rng or random.Random()
        self._last_greeted_at: Dict[str, float] = {}
        self._greeted_total = 0
        self._suppressed_total = 0

    # ------------------------------------------------------------------

    def should_greet(self, marker: MeetingMarker) -> bool:
        """Не частим ли мы с этим конкретным человеком."""
        last = self._last_greeted_at.get(marker.person_id)
        if last is None:
            return True
        return (self._clock() - last) >= self._person_cooldown_sec

    def greet(self, marker: MeetingMarker) -> Optional[str]:
        """Фраза приветствия, или ``None``, если здороваться не время.

        Побочный эффект — отметка «поздоровались»: вызывать ровно тогда,
        когда фраза действительно пойдёт в TTS.
        """
        if not self.should_greet(marker):
            self._suppressed_total += 1
            return None
        phrase = self.compose(marker)
        self._last_greeted_at[marker.person_id] = self._clock()
        self._greeted_total += 1
        return phrase

    def compose(self, marker: MeetingMarker) -> str:
        """Собрать фразу, ничего не запоминая (удобно для тестов и логов)."""
        tod = time_of_day_greeting(self._wall_clock().tm_hour)

        if marker.is_named:
            since = self._since_last_greeting(marker.person_id)
            if since is not None and since >= self._long_absence_sec:
                template = self._rng.choice(_KNOWN_LONG_ABSENCE_PHRASES)
            elif since is None and marker.encounter_count > 1:
                # Знакомый, но в этом запуске ещё не здоровались —
                # робота перезапускали, человека не было.
                template = self._rng.choice(_KNOWN_LONG_ABSENCE_PHRASES)
            else:
                template = self._rng.choice(_KNOWN_PHRASES)
            return template.format(tod=tod, name=marker.name.strip())

        if marker.is_new:
            template = self._rng.choice(_NEW_PERSON_PHRASES)
        else:
            template = self._rng.choice(_UNNAMED_RETURNING_PHRASES)
        return template.format(tod=tod)

    # ------------------------------------------------------------------

    def _since_last_greeting(self, person_id: str) -> Optional[float]:
        last = self._last_greeted_at.get(person_id)
        if last is None:
            return None
        return self._clock() - last

    def note_greeted(self, person_id: str) -> None:
        """Отметить приветствие, сказанное не через :meth:`greet`."""
        self._last_greeted_at[person_id] = self._clock()

    def stats(self) -> Dict[str, Any]:
        return {
            'greeted_total': self._greeted_total,
            'suppressed_total': self._suppressed_total,
            'known_people': len(self._last_greeted_at),
        }
