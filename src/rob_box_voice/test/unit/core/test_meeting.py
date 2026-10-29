"""v1 ``dialogue_node``: обвязка встречи ``greet_node_on_meeting`` (DSM + ``OccasionGate``).

Правила встречи (кого, когда, какой фразой) тестирует ``src/rob_box_dialog/test/test_meeting.py`` (#3912);
здесь — только что v1-хост говорит через них. Уходит вместе с v1 в PR-15.
"""

from __future__ import annotations

import time

from rob_box_voice.core.meeting import MeetingGreeter, MeetingMarker


class _Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


def _marker(name: str = 'Денис', person_id: str = 'p1', count: int = 3) -> MeetingMarker:
    return MeetingMarker(person_id=person_id, name=name, is_new=False, similarity=0.9,
                         encounter_count=count, face_px=100.0, privacy_mode='workshop')


class _Meet:
    def __init__(self) -> None:
        wall = time.struct_time((2026, 10, 21, 10, 0, 0, 2, 294, 0))
        self.greeter = MeetingGreeter(clock=_Clock(), wall_clock=lambda: wall)


class TestV1Glue:
    def test_node_glue_speaks_once_through_real_occasion_gate(self) -> None:
        from types import SimpleNamespace

        from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
        from rob_box_voice.core.meeting import greet_node_on_meeting
        from rob_box_voice.core.occasion import OccasionGate

        m = _Meet()
        spoken: list = []
        logger = SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)
        node = SimpleNamespace(
            _occasion=OccasionGate(source_cooldowns={'meeting': 5.0}), _meeting_greeter=m.greeter,
            _dsm=SimpleNamespace(current_state=DialogueStateKind.IDLE),
            _publish_response=lambda text, animation='neutral': spoken.append((text, animation)),
            get_logger=lambda: logger)
        greet_node_on_meeting(node, _marker(count=5))
        greet_node_on_meeting(node, _marker(count=5))
        assert len(spoken) == 1 and 'Денис' in spoken[0][0]
        node._dsm.current_state = DialogueStateKind.SILENCED
        greet_node_on_meeting(node, _marker(name='Ольга', person_id='p2', count=5))
        assert len(spoken) == 1
