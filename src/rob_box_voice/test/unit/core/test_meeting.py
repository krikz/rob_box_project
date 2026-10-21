"""Unit-тесты :mod:`rob_box_voice.core.meeting` (issue #2599 сценарий-заказчик,
ADR-0102 «Повод», ADR-0123).

Покрывают:

- ``parse_meeting_marker`` — что считается маркером Встречи, а что —
  рядовым мусором из общего топика ``/vision/hailo/events``. Отдельно —
  анти-выдумочное правило (ADR-0089 §2.2, #2583): ``source_camera`` в
  ``{"stub", "unknown"}`` не порождает маркер, даже если JSON валиден.

Приветствие Встречи (``MeetingGreeter``) удалено: робот сам в разговор не
вступает (В12′, ADR-0150 §18.3; #3695).
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.meeting import (
    MeetingMarker,
    parse_meeting_marker,
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
