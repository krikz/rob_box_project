"""Честный маркер выдуманных событий vision_hailo (ADR-0089 §2.2).

Зачем:
    Заглушка ``StubHEFLoader`` публикует выдуманное «person, conf 0.92, 1 м»
    каждые ``stub_period_sec``. Пока у этого события нет маркера, downstream
    не может отличить его от настоящей детекции — а значит нельзя безопасно
    подключать ``vision_events_json`` к LLM-контексту: Личность начнёт
    рассказывать про несуществующего человека рядом.

    Раньше маркера не было вовсе:
      * ``event_type`` был ``'person'`` (ADR предписывал ``'stub'``);
      * ``source_camera`` падал на ``'stub'`` только при пустом ``frame_id``,
        но узел зовёт ``infer`` с ``frame_id='unknown'``
        (``vision_hailo_node._tick``) — то есть никогда.

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_stub_marker.py -v
"""

from __future__ import annotations

import importlib
import sys
import time
from pathlib import Path

import pytest


# Этот файл: src/rob_box_perception/test/unit/test_vision_stub_marker.py
#   parents[2] = src/rob_box_perception → содержит пакет rob_box_perception/
# Относительный путь, без хардкода worktree (см. #2497 / #2505).
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

_VISION_EVENT_MSG = (
    _HERE.parents[3] / 'rob_box_perception_msgs' / 'msg' / 'VisionEvent.msg'
)

loader_mod = importlib.import_module('rob_box_perception.vision_hailo_loader')


def _emit_stub_event() -> dict:
    """Один выдуманный event от заглушки."""
    stub = loader_mod.StubHEFLoader(period_sec=0.01)
    time.sleep(0.02)
    events = stub.infer(frame_id='unknown', image=None)
    assert len(events) == 1
    return events[0]


# ---------- маркер присутствует в данных -----------------------------------


def test_stub_event_type_is_stub_not_person():
    """ADR-0089 §2.2 предписывает event_type == 'stub'."""
    ev = _emit_stub_event()
    assert ev['event_type'] == 'stub'
    assert ev['event_type'] != 'person', (
        'event_type="person" у выдуманного события — ровно тот дефект, '
        'из-за которого stub нельзя было отличить от детекции.'
    )


def test_stub_source_camera_is_marked_even_when_frame_id_given():
    """frame_id='unknown' приходит из _tick — маркер обязан пережить это."""
    ev = _emit_stub_event()
    assert ev['source_camera'] == 'stub'


def test_stub_marker_survives_any_frame_id():
    """Какой бы frame_id ни подсунули, выдумка остаётся помеченной."""
    for frame_id in ('', 'unknown', 'oak-d', 'ceiling_camera'):
        stub = loader_mod.StubHEFLoader(period_sec=0.01)
        time.sleep(0.02)
        ev = stub.infer(frame_id=frame_id, image=None)[0]
        assert loader_mod.is_stub_event(ev) is True, (
            f'Выдуманное событие потеряло маркер при frame_id={frame_id!r}'
        )


# ---------- предикат -------------------------------------------------------


def test_is_stub_event_true_for_stub_payload():
    assert loader_mod.is_stub_event(_emit_stub_event()) is True


def test_is_stub_event_false_for_real_detection():
    """Реальная детекция из постпроцессинга маркера не несёт."""
    real = {
        'source_camera': 'oak-d',
        'event_type': 'person',
        'class_name': 'person',
        'class_id': 0,
        'confidence': 0.87,
    }
    assert loader_mod.is_stub_event(real) is False


def test_is_stub_event_detects_either_marker_alone():
    """Достаточно любого из двух маркеров — предикат не требует обоих."""
    assert loader_mod.is_stub_event(
        {'event_type': 'stub', 'source_camera': 'oak-d'}
    ) is True
    assert loader_mod.is_stub_event(
        {'event_type': 'person', 'source_camera': 'stub'}
    ) is True


def test_is_stub_event_tolerates_missing_keys():
    """Частичный dict не должен ронять предикат."""
    assert loader_mod.is_stub_event({}) is False
    assert loader_mod.is_stub_event({'confidence': 0.5}) is False


# ---------- реальный путь маркера не получает ------------------------------


def test_post_process_never_marks_real_detections_as_stub():
    """Детекции из HEF обязаны оставаться неотличимо «настоящими»."""
    np = pytest.importorskip('numpy')
    # Layout как у YOLOv8n Hailo HEF: (4 + n_classes, n_anchors).
    # n_anchors = 8400 как у настоящего HEF: при малом числе якорей
    # декодер не может различить (channels, anchors) и (anchors, channels).
    tensor = np.zeros(
        (4 + loader_mod.DEFAULT_NUM_CLASSES, 8400), dtype=np.float32
    )
    tensor[0, 0] = 320.0   # cx
    tensor[1, 0] = 320.0   # cy
    tensor[2, 0] = 200.0   # w
    tensor[3, 0] = 400.0   # h
    tensor[4, 0] = 0.85    # score для class=0 (person)

    events = loader_mod._post_process_detections(
        raw_output=tensor,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events, 'постпроцессинг не вернул ни одной детекции'
    for ev in events:
        assert loader_mod.is_stub_event(ev) is False


# ---------- IDL документирует значение -------------------------------------


def test_idl_documents_stub_event_type():
    """Значение 'stub' должно быть описано в VisionEvent.msg."""
    text = _VISION_EVENT_MSG.read_text(encoding='utf-8')
    assert '"stub"' in text, (
        'VisionEvent.msg не документирует допустимое значение "stub" для '
        'event_type — контракт разойдётся с кодом.'
    )
