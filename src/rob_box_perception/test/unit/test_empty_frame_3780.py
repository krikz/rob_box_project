"""Факт «кадр пуст» (#3780 C): реальный прогон без людей — не детекция и не выдумка."""

from rob_box_perception.vision_hailo_loader import (
    EMPTY_FRAME_EVENT_TYPE,
    empty_frame_event,
    frame_clear,
    is_stub_event,
)


def test_a_ran_frame_with_nobody_is_clear():
    assert frame_clear([]) is True
    assert frame_clear([{'event_type': 'object', 'class_name': 'cup'}]) is True


def test_a_person_or_a_face_is_not_clear_even_below_the_confidence_filter():
    assert frame_clear([{'event_type': 'person', 'confidence': 0.1}]) is False
    assert frame_clear([{'event_type': 'face'}]) is False


def test_a_stub_does_not_clear_the_frame_and_is_not_a_person():
    stub = {'event_type': 'stub', 'source_camera': 'stub', 'class_name': 'person'}
    assert is_stub_event(stub) is True
    assert frame_clear([stub]) is False


def test_empty_frame_event_is_not_a_detection():
    event = empty_frame_event('oak_d')
    assert event['event_type'] == EMPTY_FRAME_EVENT_TYPE
    assert event['event_type'] not in ('person', 'face', 'object', 'scene', 'stub')
    assert event['bbox_w'] < 0
    assert is_stub_event(event) is False
