"""Unit-тесты :mod:`rob_box_dialog.meeting` — перенесены из v1 ``core/meeting.py`` вместе с модулем (#3912;
issue #2599 сценарий-заказчик, ADR-0102 «Повод», ADR-0123).

Покрывают:

- ``parse_meeting_marker`` — что считается маркером Встречи, а что —
  рядовым мусором из общего топика ``/vision/hailo/events``. Отдельно —
  анти-выдумочное правило (ADR-0089 §2.2, #2583): ``source_camera`` в
  ``{"stub", "unknown"}`` не порождает маркер, даже если JSON валиден.

Приветствие знакомого после долгой разлуки — исключение Шифу 21.10 из В12′
(ADR-0150 §18.3; #3695): одна фраза по имени, без вопросов; незнакомец молчит.
"""

from __future__ import annotations

import time

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.meeting import (
    MeetingGreeter,
    MeetingMarker,
    greet_on_meeting,
    parse_meeting_marker,
    time_of_day_greeting,
    unprompted_quiet,
)
from rob_box_dialog.session import Session


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
            # Источник кадра переносится в маркер как есть: у OccasionGate
            # свой стаб-фильтр по этому полю (ADR-0089 §2.2, #2583).
            source_camera='main_camera',
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


HOUR = 3600.0


class _Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


def _marker(name: str = 'Денис', person_id: str = 'p1', is_new: bool = False, count: int = 3) -> MeetingMarker:
    return MeetingMarker(person_id=person_id, name=name, is_new=is_new, similarity=0.9,
                         encounter_count=count, face_px=100.0, privacy_mode='workshop')


class _Meet:
    """Обвязка: часы, греттер, список сказанного."""

    def __init__(self, hour: int = 10, idle: bool = True, allowed: bool = True) -> None:
        self.clock = _Clock()
        wall = time.struct_time((2026, 10, 21, hour, 0, 0, 2, 294, 0))
        self.greeter = MeetingGreeter(clock=self.clock, wall_clock=lambda: wall)
        self.said: list = []
        self.idle = idle
        self.allowed = allowed

    def meet(self, marker: MeetingMarker):
        return greet_on_meeting(self.greeter, marker, idle=self.idle,
                                allowed=lambda m: self.allowed, speak=self.said.append)


@pytest.mark.parametrize('hour,expected', [(4, 'Доброй ночи'), (5, 'Доброе утро'), (12, 'Добрый день'),
                                           (18, 'Добрый вечер'), (23, 'Доброй ночи')])
def test_time_of_day(hour: int, expected: str) -> None:
    assert time_of_day_greeting(hour) == expected


class TestGreetAfterLongAbsence:
    def test_known_after_21_hours_gets_one_phrase_with_name(self) -> None:
        m = _Meet()
        m.meet(_marker(count=1))  # первая встреча в запуске: просто запомнили
        assert m.said == []
        m.clock.now += 21 * HOUR
        phrase = m.meet(_marker())
        assert phrase is not None and 'Денис' in phrase
        assert m.said == [phrase]

    def test_phrase_has_time_of_day(self) -> None:
        m = _Meet(hour=19)
        m.meet(_marker(count=1))
        m.clock.now += 21 * HOUR
        m.meet(_marker())
        assert m.said and 'Добрый вечер' in m.said[0]

    def test_again_after_one_hour_is_silent(self) -> None:
        m = _Meet()
        m.meet(_marker(count=1))
        m.clock.now += 21 * HOUR
        m.meet(_marker())
        m.clock.now += HOUR
        assert m.meet(_marker()) is None
        assert len(m.said) == 1

    def test_phrase_is_not_a_question(self) -> None:
        m = _Meet()
        for seed in range(30):
            m.greeter._rng.seed(seed)
            assert '?' not in m.greeter.compose(_marker())

    def test_after_restart_known_person_greeted_once(self) -> None:
        m = _Meet()
        assert m.meet(_marker(count=5)) is not None
        m.clock.now += 60
        assert m.meet(_marker(count=6)) is None

    def test_first_ever_encounter_is_silent(self) -> None:
        m = _Meet()
        assert m.meet(_marker(count=1)) is None

    def test_fake_face_with_name_after_20_hours_greets_once(self) -> None:
        m = _Meet()
        m.meet(_marker(name='Литвин', person_id='fake', count=1))
        m.clock.now += 20 * HOUR
        assert 'Литвин' in m.meet(_marker(name='Литвин', person_id='fake'))
        m.clock.now += 60
        assert m.meet(_marker(name='Литвин', person_id='fake')) is None


class TestNoGreeting:
    def test_stranger_gets_no_phrase_and_no_question(self) -> None:
        m = _Meet()
        assert m.meet(_marker(name='', is_new=True, count=1)) is None
        m.clock.now += 48 * HOUR
        assert m.meet(_marker(name='', is_new=True, count=1)) is None
        assert m.said == []

    def test_unnamed_returning_face_is_silent(self) -> None:
        m = _Meet()
        m.clock.now += 48 * HOUR
        assert m.meet(_marker(name='', count=7)) is None

    def test_dialogue_active_does_not_interrupt_and_retries_later(self) -> None:
        m = _Meet(idle=False)
        assert m.meet(_marker(count=5)) is None
        assert m.said == []
        m.idle = True
        assert m.meet(_marker(count=5)) is not None

    def test_gate_refusal_does_not_burn_the_greeting(self) -> None:
        m = _Meet(allowed=False)
        assert m.meet(_marker(count=5)) is None
        m.allowed = True
        assert m.meet(_marker(count=5)) is not None

    def test_two_people_both_greeted(self) -> None:
        m = _Meet()
        assert 'Денис' in m.meet(_marker(count=5))
        assert 'Ольга' in m.meet(_marker(name='Ольга', person_id='p2', count=5))


class TestThresholdAndQuiet:
    """Порог «давно» — одна константа знания; «нельзя заговорить самому» — по v2-владельцам (ход, сцены сессии)."""

    def test_just_under_20_hours_is_silent_and_20_hours_greets(self) -> None:
        assert kn.MEETING_LONG_ABSENCE_S == 20 * HOUR
        m = _Meet()
        m.meet(_marker(count=1))
        m.clock.now += 20 * HOUR - 1
        assert m.meet(_marker()) is None
        m.clock.now += 20 * HOUR
        assert m.meet(_marker()) is not None

    def test_quiet_while_turn_runs_or_silenced_or_held(self) -> None:
        session = Session("personality", clock=lambda: 0.0)
        assert unprompted_quiet(session.snapshot, turn_busy=False) is False
        assert unprompted_quiet(session.snapshot, turn_busy=True) is True
        for kind, setter in (("Silence", "user"), ("OperatorHold", "operator")):
            session.push_scene(kind, setter, ttl_s=60.0)
            assert unprompted_quiet(session.snapshot, turn_busy=False) is True, kind
            session.pop_scene(kind)
        session.push_scene("DJ", "code")  # музыка/сет в v1 не глушили приветствие — и здесь не глушат
        assert unprompted_quiet(session.snapshot, turn_busy=False) is False
