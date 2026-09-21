"""Unit-тесты :mod:`rob_box_voice.core.meeting` (issue #2599 сценарий-заказчик,
ADR-0102 «Повод», ADR-0123).

Покрывают:

- ``parse_meeting_marker`` — что считается маркером Встречи, а что —
  рядовым мусором из общего топика ``/vision/hailo/events``. Отдельно —
  анти-выдумочное правило (ADR-0089 §2.2, #2583): ``source_camera`` в
  ``{"stub", "unknown"}`` не порождает маркер, даже если JSON валиден.
- ``time_of_day_greeting`` — границы часов.
- ``MeetingGreeter`` — какая фраза и когда, per-person кулдаун.

Чистые unit-тесты: без ROS2, без LLM/TTS. Время — только через
инъекцию (``clock``/``wall_clock``), выбор фразы — через сидированный
``random.Random`` (никакой недетерминированности).
"""

from __future__ import annotations

import time

import pytest

from rob_box_voice.core.meeting import (
    MeetingGreeter,
    MeetingMarker,
    parse_meeting_marker,
    time_of_day_greeting,
)


# ---------------------------------------------------------------------------
# parse_meeting_marker
# ---------------------------------------------------------------------------


class TestParseMeetingMarker:
    """Разбор маркера Встречи из ``VisionEvent.attributes_json``."""

    def _good_payload(self, **overrides) -> str:
        import json

        payload = {
            'encounter': 'start',
            'person_id': 'p1',
            'name': 'Денис',
            'is_new': False,
            'similarity': 0.87,
            'encounter_count': 3,
            'face_px': 120.0,
            'privacy_mode': 'workshop',
        }
        payload.update(overrides)
        return json.dumps(payload, ensure_ascii=False)

    def test_wrong_event_type_returns_none(self) -> None:
        marker = parse_meeting_marker('person', self._good_payload(), source_camera='main_camera')
        assert marker is None

    def test_empty_attributes_json_returns_none(self) -> None:
        marker = parse_meeting_marker('face', '', source_camera='main_camera')
        assert marker is None

    def test_malformed_json_returns_none(self) -> None:
        marker = parse_meeting_marker('face', '{not valid json', source_camera='main_camera')
        assert marker is None

    def test_non_json_garbage_returns_none(self) -> None:
        marker = parse_meeting_marker('face', 'просто строка', source_camera='main_camera')
        assert marker is None

    def test_json_but_not_a_dict_returns_none(self) -> None:
        marker = parse_meeting_marker('face', '[1, 2, 3]', source_camera='main_camera')
        assert marker is None

    def test_valid_json_without_encounter_start_returns_none(self) -> None:
        marker = parse_meeting_marker(
            'face', self._good_payload(encounter='update'), source_camera='main_camera'
        )
        assert marker is None

    def test_missing_encounter_key_returns_none(self) -> None:
        import json

        payload = {'person_id': 'p1', 'name': 'Денис'}
        marker = parse_meeting_marker('face', json.dumps(payload), source_camera='main_camera')
        assert marker is None

    def test_marker_without_person_id_returns_none(self) -> None:
        marker = parse_meeting_marker('face', self._good_payload(person_id=''), source_camera='main_camera')
        assert marker is None

    def test_missing_person_id_key_returns_none(self) -> None:
        import json

        payload = {'encounter': 'start', 'name': 'Денис'}
        marker = parse_meeting_marker('face', json.dumps(payload), source_camera='main_camera')
        assert marker is None

    def test_good_payload_returns_populated_marker(self) -> None:
        marker = parse_meeting_marker('face', self._good_payload(), source_camera='main_camera')
        assert marker == MeetingMarker(
            person_id='p1',
            name='Денис',
            is_new=False,
            similarity=0.87,
            encounter_count=3,
            face_px=120.0,
            privacy_mode='workshop',
        )
        assert marker.is_named is True

    def test_good_payload_unnamed_person(self) -> None:
        marker = parse_meeting_marker('face', self._good_payload(name=''), source_camera='main_camera')
        assert marker is not None
        assert marker.is_named is False

    @pytest.mark.parametrize('bad_camera', ['stub', 'unknown'])
    def test_stub_or_unknown_camera_returns_none(self, bad_camera: str) -> None:
        """Анти-выдумочное правило: робот не здоровается с несуществующими людьми."""
        marker = parse_meeting_marker('face', self._good_payload(), source_camera=bad_camera)
        assert marker is None

    def test_empty_source_camera_is_not_treated_as_stub(self) -> None:
        """Пустая строка значит «источник не передали», а не «выдумано» —
        это НЕ должно отсекаться тем же правилом, что stub/unknown."""
        marker = parse_meeting_marker('face', self._good_payload(), source_camera='')
        assert marker is not None
        assert marker.person_id == 'p1'

    def test_default_source_camera_argument_is_empty_and_allowed(self) -> None:
        """``source_camera`` по умолчанию — пустая строка, не stub."""
        marker = parse_meeting_marker('face', self._good_payload())
        assert marker is not None


# ---------------------------------------------------------------------------
# time_of_day_greeting
# ---------------------------------------------------------------------------


class TestTimeOfDayGreeting:
    """Границы часов — доброе утро/день/вечер/ночь."""

    @pytest.mark.parametrize(
        'hour,expected',
        [
            (0, 'Доброй ночи'),
            (4, 'Доброй ночи'),
            (5, 'Доброе утро'),
            (11, 'Доброе утро'),
            (12, 'Добрый день'),
            (17, 'Добрый день'),
            (18, 'Добрый вечер'),
            (22, 'Добрый вечер'),
            (23, 'Доброй ночи'),
        ],
    )
    def test_boundaries(self, hour: int, expected: str) -> None:
        assert time_of_day_greeting(hour) == expected


# ---------------------------------------------------------------------------
# MeetingGreeter
# ---------------------------------------------------------------------------


class _FakeClock:
    """Управляемые вручную монотонные часы для тестов ``MeetingGreeter``."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now


class _FakeWallClock:
    """Управляемый вручную источник ``time.localtime()`` — только ``tm_hour``."""

    def __init__(self, hour: int = 12) -> None:
        self.hour = hour

    def __call__(self) -> time.struct_time:
        # struct_time требует все 9 полей; важен только tm_hour.
        return time.struct_time((2026, 9, 22, self.hour, 0, 0, 1, 265, 0))


def make_marker(
    person_id: str = 'p1',
    name: str = '',
    is_new: bool = False,
    similarity: float = 0.9,
    encounter_count: int = 1,
    face_px: float = 100.0,
    privacy_mode: str = 'workshop',
) -> MeetingMarker:
    return MeetingMarker(
        person_id=person_id,
        name=name,
        is_new=is_new,
        similarity=similarity,
        encounter_count=encounter_count,
        face_px=face_px,
        privacy_mode=privacy_mode,
    )


class TestMeetingGreeterPhrases:
    """Какую фразу выбирает ``compose``/``greet`` в зависимости от того,
    знаем ли имя и виделись ли раньше."""

    def test_named_person_phrase_contains_name(self) -> None:
        import random

        greeter = MeetingGreeter(
            clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        marker = make_marker(name='Денис', encounter_count=1)
        phrase = greeter.greet(marker)
        assert phrase is not None
        assert 'Денис' in phrase

    def test_unnamed_brand_new_person_asks_for_name(self) -> None:
        import random

        greeter = MeetingGreeter(
            clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        marker = make_marker(name='', is_new=True, encounter_count=1)

        # Проверяем намерение (просьба представиться), а не конкретную
        # строку — шаблонов несколько, гоняем через все возможные seed'ы.
        found_intent = False
        for seed in range(20):
            g = MeetingGreeter(clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(seed))
            phrase = g.greet(marker)
            assert phrase is not None
            if 'зовут' in phrase or 'Представься' in phrase or 'представишься' in phrase or 'как тебя звать' in phrase:
                found_intent = True
        assert found_intent, 'ни один шаблон нового человека не просит представиться'

    def test_unnamed_returning_person_gets_we_met_before_flavour(self) -> None:
        import random

        marker = make_marker(name='', is_new=False, encounter_count=2)

        found_intent = False
        for seed in range(20):
            g = MeetingGreeter(clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(seed))
            phrase = g.greet(marker)
            assert phrase is not None
            if 'виделись' in phrase or 'знакомо' in phrase or 'Опять' in phrase or 'снова' in phrase.lower():
                found_intent = True
            # Не должно быть фразы "мы ещё не знакомы"/"новое лицо" — это шаблон новичка.
            assert 'мы ещё не знакомы' not in phrase
            assert 'новое лицо' not in phrase
        assert found_intent, 'ни один шаблон возвращающегося незнакомца не звучит как "мы уже виделись"'

    def test_named_person_not_greeted_yet_but_returning_gets_long_absence_flavour(self) -> None:
        """Знакомый (encounter_count > 1), но в этом запуске процесса ещё
        не здоровались — трактуется как "давно не виделись" (перезапуск ноды)."""
        import random

        marker = make_marker(name='Денис', encounter_count=5)

        found_intent = False
        for seed in range(20):
            g = MeetingGreeter(clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(seed))
            phrase = g.greet(marker)
            assert phrase is not None
            if 'Сколько лет' in phrase or 'Давно тебя не видел' in phrase or 'забыл' in phrase:
                found_intent = True
        assert found_intent, 'знакомый с encounter_count>1, не встреченный в этом процессе, должен звучать как "давно не виделись"'

    def test_named_person_first_ever_encounter_gets_plain_known_phrase(self) -> None:
        """encounter_count == 1 и ещё не здоровались в этом процессе — это
        первая Встреча вообще, не "давно не виделись"."""
        import random

        found_long_absence = False
        for seed in range(20):
            g = MeetingGreeter(clock=_FakeClock(0.0), wall_clock=_FakeWallClock(10), rng=random.Random(seed))
            marker = make_marker(name='Денис', encounter_count=1)
            phrase = g.greet(marker)
            assert phrase is not None
            if 'Сколько лет' in phrase or 'Давно тебя не видел' in phrase or 'забыл' in phrase:
                found_long_absence = True
        assert not found_long_absence, 'первая встреча вообще не должна звучать как "давно не виделись"'


class TestMeetingGreeterCooldown:
    """Per-person кулдаун — не глобальный."""

    def test_second_greet_within_cooldown_returns_none(self) -> None:
        import random

        clock = _FakeClock(0.0)
        greeter = MeetingGreeter(
            person_cooldown_sec=900.0, clock=clock, wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        marker = make_marker(person_id='p1', name='Денис')

        first = greeter.greet(marker)
        assert first is not None

        clock.now = 100.0  # внутри кулдауна (900с)
        second = greeter.greet(marker)
        assert second is None

    def test_greet_after_cooldown_expires_greets_again(self) -> None:
        import random

        clock = _FakeClock(0.0)
        greeter = MeetingGreeter(
            person_cooldown_sec=900.0, clock=clock, wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        marker = make_marker(person_id='p1', name='Денис')

        assert greeter.greet(marker) is not None

        clock.now = 900.0  # ровно на границе — кулдаун исчерпан
        assert greeter.greet(marker) is not None

    def test_two_different_people_in_a_row_both_greeted(self) -> None:
        """Регресс-риск: кулдаун должен быть per-person, а не глобальный."""
        import random

        clock = _FakeClock(0.0)
        greeter = MeetingGreeter(
            person_cooldown_sec=900.0, clock=clock, wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        first_marker = make_marker(person_id='denis', name='Денис')
        second_marker = make_marker(person_id='olga', name='Ольга')

        phrase1 = greeter.greet(first_marker)
        clock.now = 1.0  # секунда спустя, второй человек зашёл следом
        phrase2 = greeter.greet(second_marker)

        assert phrase1 is not None
        assert phrase2 is not None
        assert 'Денис' in phrase1
        assert 'Ольга' in phrase2

    def test_stats_counts_greeted_and_suppressed(self) -> None:
        import random

        clock = _FakeClock(0.0)
        greeter = MeetingGreeter(
            person_cooldown_sec=900.0, clock=clock, wall_clock=_FakeWallClock(10), rng=random.Random(1)
        )
        marker = make_marker(person_id='p1', name='Денис')

        greeter.greet(marker)  # greeted
        clock.now = 10.0
        greeter.greet(marker)  # suppressed — в окне кулдауна
        clock.now = 20.0
        greeter.greet(marker)  # suppressed

        stats = greeter.stats()
        assert stats['greeted_total'] == 1
        assert stats['suppressed_total'] == 2
        assert stats['known_people'] == 1
