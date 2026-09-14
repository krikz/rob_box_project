"""Unit-тесты для Phase 1.5 pure-функций `vision_hailo_loader` (ADR-0089).

Запуск:
    PYTHONPATH=src/rob_box_perception \
        python3 -m pytest src/rob_box_perception/test/unit/test_vision_hailo_phase15.py -v

Что покрываем (Phase 1.5 pure-Python, БЕЗ hailo_platform):
  1. `_post_process_detections` — shape (84, 8400), (8400, 84),
     unknown shape (→ return []).
  2. `_post_process_detections` — confidence thresholding: ниже порога →
     drop, ровно порог → keep.
  3. `_post_process_detections` — bbox normalization (px → [0,1]).
  4. `_post_process_detections` — class mapping: class_id=0 → 'person',
     иначе 'object'.
  5. `_post_process_detections` — distance_m = -1.0 (Phase 2 будет fusion).
  6. `_nms_per_class` — простой кейс: 3 одинаковых бокса → 1 остаётся.
  7. `_nms_per_class` — разные классы → не подавляют друг друга.
  8. `_nms_per_class` — empty input → [].
  9. `make_loader` — hailo_enabled=True + hef_path → RealHEFLoader
     (instance type, без init).
 10. RealHEFLoader.infer(image=None) → [] (без raise) — safety guard.

Эти тесты НЕ требуют rclpy / HailoRT / железа — pure-numpy.
"""

from __future__ import annotations

import importlib
import sys
from typing import List

import numpy as np
import pytest


# ---------- import target under test ------------------------------------

def _import_module():
    pkg_root = (
        '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
        'src/rob_box_perception'
    )
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)
    return importlib.import_module('rob_box_perception.vision_hailo_loader')


loader_mod = _import_module()


# ============================================================================
# _post_process_detections
# ============================================================================

def _make_fake_output(
    n_anchors: int = loader_mod.DEFAULT_NUM_ANCHORS,
    n_classes: int = loader_mod.DEFAULT_NUM_CLASSES,
    seed: int = 42,
) -> np.ndarray:
    """Сгенерировать фейковый (84, 8400) output с одной 'person' детекцией.

    Box при class=0 ('person'): cx=320, cy=320, w=200, h=400 (в letterbox 640×640).
    Confidence = 0.85.
    """
    rng = np.random.default_rng(seed)
    out = rng.uniform(0, 0.1, size=(4 + n_classes, n_anchors)).astype(np.float32)
    # Anchor 0 → strong "person" detection.
    out[0, 0] = 320.0  # cx
    out[1, 0] = 320.0  # cy
    out[2, 0] = 200.0  # w
    out[3, 0] = 400.0  # h
    # Класс 0 ('person') получает score 0.85, остальные классы — низкие.
    out[4:, :] = 0.0
    out[4, 0] = 0.85  # score для class=0 (person)
    return out


def test_post_process_shape_84_8400():
    """Layout (84, 8400) — стандартный YOLOv8n Hailo HEF output."""
    raw = _make_fake_output()
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    # Только одна детекция выше threshold=0.5.
    assert len(events) == 1
    ev = events[0]
    assert ev['source_camera'] == 'oak-d'
    assert ev['event_type'] == 'person'
    assert ev['class_name'] == 'person'
    assert ev['class_id'] == 0
    assert ev['confidence'] == pytest.approx(0.85, abs=0.01)
    # Bbox normalized: cx=320/640=0.5, cy=320/640=0.5, w=200/640≈0.3125, h=400/640=0.625.
    assert ev['bbox_cx'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_cy'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_w'] == pytest.approx(0.3125, abs=0.01)
    assert ev['bbox_h'] == pytest.approx(0.625, abs=0.01)
    assert ev['distance_m'] == -1.0


def test_post_process_shape_8400_84_transposed():
    """Layout (8400, 84) — HEF иногда выдаёт transposed output."""
    raw = _make_fake_output()
    raw = raw.T  # (8400, 84)
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    assert events[0]['event_type'] == 'person'


def test_post_process_unknown_shape_returns_empty():
    """Неизвестный shape → return [] (capability-honest, не raise)."""
    bad = np.zeros((100, 100), dtype=np.float32)  # не 84x8400 и не 8400x84
    events = loader_mod._post_process_detections(
        raw_output=bad,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events == []


def test_post_process_confidence_threshold_drops_low():
    """Confidence 0.4 < threshold 0.5 → drop."""
    raw = _make_fake_output()
    raw[4, 0] = 0.4  # ниже threshold
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events == []


def test_post_process_confidence_at_threshold_keeps():
    """Confidence == threshold → keep (>= contract)."""
    raw = _make_fake_output()
    raw[4, 0] = 0.5  # ровно threshold
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1


def test_post_process_class_mapping_object():
    """class_id != 0 → event_type='object', class_name=COCO name."""
    raw = _make_fake_output()
    # Anchor 0 теперь 'cup' (class 41 в COCO).
    raw[4, 0] = 0.0
    raw[4 + 41, 0] = 0.9
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    ev = events[0]
    assert ev['class_id'] == 41
    assert ev['class_name'] == 'cup'
    assert ev['event_type'] == 'object'


def test_post_process_source_camera_propagated():
    """source_camera из входа → source_camera в каждом event'е."""
    raw = _make_fake_output()
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='mjpg-720p',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events[0]['source_camera'] == 'mjpg-720p'


def test_post_process_empty_input():
    """raw_output=None → return []. Используется при init failure."""
    events = loader_mod._post_process_detections(
        raw_output=None,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events == []


# ============================================================================
# _nms_per_class
# ============================================================================

def test_nms_drops_overlapping_same_class():
    """3 одинаковых бокса одного класса → 1 остаётся."""
    boxes = np.array([
        [10, 10, 50, 50],
        [12, 12, 52, 52],   # ~overlapping
        [11, 11, 51, 51],   # ~overlapping
    ], dtype=np.float32)
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    classes = np.array([0, 0, 0], dtype=np.int32)
    keep = loader_mod._nms_per_class(boxes, scores, classes, iou_threshold=0.45)
    assert keep == [0]


def test_nms_keeps_different_classes():
    """Одинаковый bbox, но разные классы — НЕ подавляют друг друга."""
    boxes = np.array([
        [10, 10, 50, 50],
        [10, 10, 50, 50],
    ], dtype=np.float32)
    scores = np.array([0.9, 0.8], dtype=np.float32)
    classes = np.array([0, 1], dtype=np.int32)  # разные классы
    keep = loader_mod._nms_per_class(boxes, scores, classes, iou_threshold=0.45)
    assert set(keep) == {0, 1}


def test_nms_empty_boxes():
    """Empty input → empty output."""
    boxes = np.zeros((0, 4), dtype=np.float32)
    scores = np.zeros((0,), dtype=np.float32)
    classes = np.zeros((0,), dtype=np.int32)
    keep = loader_mod._nms_per_class(boxes, scores, classes, iou_threshold=0.45)
    assert keep == []


def test_nms_low_iou_threshold_keeps_far_boxes():
    """iou_threshold=0.45 + boxes с IoU < 0.45 → все сохраняются."""
    # Два бокса далеко друг от друга, IoU близок к 0.
    # xyxy: x1, y1, x2, y2 (x2 > x1, y2 > y1)
    boxes = np.array([
        [10, 10, 50, 50],
        [200, 200, 300, 300],  # далеко, не пересекается
    ], dtype=np.float32)
    scores = np.array([0.9, 0.8], dtype=np.float32)
    classes = np.array([0, 0], dtype=np.int32)
    keep = loader_mod._nms_per_class(boxes, scores, classes, iou_threshold=0.45)
    assert len(keep) == 2


def test_nms_low_iou_threshold_suppresses_overlapping():
    """iou_threshold=0.45 + boxes с IoU > 0.45 → второй подавляется."""
    boxes = np.array([
        [10, 10, 50, 50],
        [12, 12, 52, 52],   # ~overlapping, IoU ~ 0.97
    ], dtype=np.float32)
    scores = np.array([0.9, 0.8], dtype=np.float32)
    classes = np.array([0, 0], dtype=np.int32)
    keep = loader_mod._nms_per_class(boxes, scores, classes, iou_threshold=0.45)
    assert keep == [0]


# ============================================================================
# RealHEFLoader: contract (не init, не run)
# ============================================================================

def test_real_loader_infer_with_none_image_returns_empty():
    """RealHEFLoader.infer(image=None) → [] (safety guard).

    Узел вызывает infer с image=None если _latest_image ещё нет.
    Это НЕ должно raise — Node продолжает heartbeat.
    """
    real = loader_mod.RealHEFLoader(hef_path='/nonexistent.hef')
    # С image=None не вызывается _ensure_initialized (lazy).
    events = real.infer(frame_id='oak-d', image=None)
    assert events == []


def test_real_loader_init_failure_is_available_false():
    """Без hailo_platform + missing HEF → is_available() = False."""
    real = loader_mod.RealHEFLoader(hef_path='/nonexistent.hef')
    # Не делаем mock — на CI без hailo_platform упадёт ImportError → False.
    assert real.is_available() is False


def test_real_loader_with_missing_hef_raises_on_init():
    """infer(image=fake) без HEF → raise (capability-honest)."""
    real = loader_mod.RealHEFLoader(hef_path='/nonexistent.hef')
    fake_img = np.zeros((480, 640, 3), dtype=np.uint8)
    with pytest.raises((RuntimeError, ImportError, FileNotFoundError)):
        real.infer(frame_id='oak-d', image=fake_img)


# ============================================================================
# make_loader factory
# ============================================================================

def test_make_loader_routing_for_real():
    """hailo_enabled=True + hef_path=non-empty → RealHEFLoader instance."""
    loader = loader_mod.make_loader(
        hailo_enabled=True,
        hef_path='/tmp/yolov8n.hef',
        stub_period_sec=2.0,
    )
    assert isinstance(loader, loader_mod.RealHEFLoader)


def test_make_loader_routing_for_stub():
    """hailo_enabled=False → StubHEFLoader."""
    loader = loader_mod.make_loader(
        hailo_enabled=False,
        hef_path=None,
        stub_period_sec=2.0,
    )
    assert isinstance(loader, loader_mod.StubHEFLoader)


def test_make_loader_routing_for_empty_hef():
    """hailo_enabled=True но hef_path=None → StubHEFLoader (per contract)."""
    loader = loader_mod.make_loader(
        hailo_enabled=True,
        hef_path=None,
        stub_period_sec=2.0,
    )
    assert isinstance(loader, loader_mod.StubHEFLoader)


# ============================================================================
# Integration: post_process на синтетическом YOLO-выходе с multiple detections
# ============================================================================

def test_post_process_multiple_detections_with_nms():
    """Несколько детекций одного класса + NMS → подавляются overlapping.

    Генерируем 4 'person' детекции: 2 overlapping pair + 2 одиночных
    далеко друг от друга. Ожидаем: NMS оставит 3 (по одному от каждого
    кластера + одиночные).
    """
    raw = _make_fake_output(seed=1)
    # Anchor 0: person@0.9 в позиции (320, 320), w=200, h=400
    # xyxy = [220, 120, 420, 520]
    raw[0, 0], raw[1, 0], raw[2, 0], raw[3, 0] = 320, 320, 200, 400
    raw[4, 0] = 0.9
    # Anchor 1: person@0.8 в позиции (321, 322) — overlapping с Anchor 0
    # xyxy = [221, 122, 421, 522]
    raw[0, 1], raw[1, 1], raw[2, 1], raw[3, 1] = 321, 322, 200, 400
    raw[4, 1] = 0.8
    # Anchor 2: person@0.7 в позиции (100, 100), w=20, h=40
    # xyxy = [90, 80, 110, 120] — далеко от Anchor 0/1, IoU ~ 0
    raw[0, 2], raw[1, 2], raw[2, 2], raw[3, 2] = 100, 100, 20, 40
    raw[4, 2] = 0.7
    # Anchor 3: person@0.6 в позиции (600, 600), w=20, h=40
    # xyxy = [590, 580, 610, 620] — далеко от Anchor 0/1/2
    raw[0, 3], raw[1, 3], raw[2, 3], raw[3, 3] = 600, 600, 20, 40
    raw[4, 3] = 0.6

    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    # Anchor 0 и 1 схлопнутся (overlap) → остаётся Anchor 0.
    # Anchor 2 и 3 остаются (далеко).
    classes = [ev['class_id'] for ev in events]
    assert all(c == 0 for c in classes), 'Все должны быть person (class_id=0)'
    assert len(events) == 3, f'NMS должен оставить 3, получил {len(events)}'


def test_post_process_keeps_multiple_classes():
    """'person' и 'cup' детекции оба проходят (разные классы)."""
    raw = _make_fake_output(seed=2)
    # Anchor 0: person@0.9
    raw[0, 0], raw[1, 0] = 320, 320
    raw[2, 0], raw[3, 0] = 200, 400
    raw[4, 0] = 0.9
    # Anchor 1: cup@0.85 (class 41) на другой позиции
    raw[0, 1], raw[1, 1] = 100, 100
    raw[2, 1], raw[3, 1] = 100, 100
    raw[4 + 41, 1] = 0.85

    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    classes = sorted(ev['class_name'] for ev in events)
    assert classes == ['cup', 'person']
