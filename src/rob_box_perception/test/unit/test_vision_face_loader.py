"""Unit-тесты для vision_face_loader (ADR-0089 Phase 2, issue #2599 PR-A).

Запуск:
    pytest src/rob_box_perception/test/unit/test_vision_face_loader.py -v

Что покрываем (pure-Python, БЕЗ hailo_platform / rclpy / железа):
  1. _build_anchors — форма (38640, 4), порядок, нормализация [0, 1].
  2. _decode_boxes — RetinaFace box regression -> xyxy.
  3. _softmax — face-score через индекс 1.
  4. _normalize_output_layout — NHWC / NCHW / flat -> (H, W, C).
  5. _post_process_faces — synthetic 9-тензорный выход с одним лицом ->
     event_type="face", bbox normalized, embedding пустой (PR-A).
  6. _post_process_faces — порог confidence ниже которого drop.
  7. make_face_loader — factory routing (RetinaFace vs Stub).
  8. Stub-режим — event_type="stub", is_stub_event=True (выдумка помечена).
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest


# ---------- import target under test ------------------------------------

# `parents[2]` = .../src/rob_box_perception → содержит пакет rob_box_perception/.
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
loader_mod = importlib.import_module('rob_box_perception.vision_face_loader')
yolo_loader_mod = importlib.import_module(
    'rob_box_perception.vision_hailo_loader'
)


# ---------- helpers ------------------------------------------------------

def _build_synthetic_outputs():
    """9 тензоров retinaface: одно лицо на branch 0, cell (46, 80), anchor 0.

    Box regression = [0,0,0,0] -> декодированный box == anchor (норм.).
    Class = [0, 10] на цели (face-score ~1.0), [10, 0] везде (face-score ~0).
    """
    outputs = []
    for idx, (fh, fw) in enumerate(loader_mod.FEATURE_MAP_SIZES):
        box_t = np.zeros((fh, fw, loader_mod._BOX_CHANNELS), dtype=np.float32)
        cls_t = np.zeros((fh, fw, loader_mod._CLASS_CHANNELS), dtype=np.float32)
        cls_t[:, :, 0] = 10.0  # bg strong
        cls_t[:, :, 1] = 0.0   # face weak
        # 3-й тензор (landmarks) — PR-A не декодирует, но обязан быть в списке.
        lmk_t = np.zeros((fh, fw, loader_mod._LANDMARK_CHANNELS), dtype=np.float32)
        outputs.extend([box_t, cls_t, lmk_t])

    # Лицо на branch 0 (fh=92, fw=160), cell (46, 80), anchor 0.
    outputs[0][46, 80, 0:4] = [0.0, 0.0, 0.0, 0.0]   # box regression = 0
    outputs[1][46, 80, 0] = 0.0                       # bg
    outputs[1][46, 80, 1] = 10.0                      # face
    return outputs


# ---------- tests ---------------------------------------------------------


def test_build_anchors_shape_and_normalization():
    anchors = loader_mod._build_anchors()
    total = sum(fh * fw * 2 for fh, fw in loader_mod.FEATURE_MAP_SIZES)
    assert anchors.shape == (total, 4)
    assert np.all(anchors >= 0.0) and np.all(anchors <= 1.0)
    # Первый анкер: branch 0, cell (0, 0), min_size 16.
    assert anchors[0][0] == pytest.approx(0.5 / 160, abs=1e-6)   # cx
    assert anchors[0][1] == pytest.approx(0.5 / 92, abs=1e-6)    # cy
    assert anchors[0][2] == pytest.approx(16 / 1280, abs=1e-6)   # w
    assert anchors[0][3] == pytest.approx(16 / 736, abs=1e-6)    # h


def test_decode_boxes_identity_regression():
    anchors = np.array([[0.5, 0.5, 0.1, 0.2]], dtype=np.float32)
    box = np.array([[0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
    xyxy = loader_mod._decode_boxes(box, anchors)
    assert xyxy.shape == (1, 4)
    assert xyxy[0][0] == pytest.approx(0.45, abs=1e-6)  # x1
    assert xyxy[0][1] == pytest.approx(0.40, abs=1e-6)  # y1
    assert xyxy[0][2] == pytest.approx(0.55, abs=1e-6)  # x2
    assert xyxy[0][3] == pytest.approx(0.60, abs=1e-6)  # y2


def test_decode_boxes_positive_regression_shifts_center():
    anchors = np.array([[0.5, 0.5, 0.1, 0.2]], dtype=np.float32)
    # dx=10 -> cx += 10/10 * 0.1 = 0.1; dw=5 -> w *= exp(1) ~ 2.718.
    box = np.array([[10.0, 0.0, 5.0, 0.0]], dtype=np.float32)
    xyxy = loader_mod._decode_boxes(box, anchors)
    assert xyxy[0][0] + xyxy[0][2] == pytest.approx(1.2, abs=1e-5)  # cx*2 = 0.6*2
    assert xyxy[0][1] + xyxy[0][3] == pytest.approx(1.0, abs=1e-5)  # cy*2 = 0.5*2


def test_softmax_face_index_one():
    logits = np.array([[10.0, 0.0], [0.0, 10.0]], dtype=np.float32)
    probs = loader_mod._softmax(logits, axis=1)
    assert probs[0][1] == pytest.approx(0.0, abs=1e-3)
    assert probs[1][1] == pytest.approx(1.0, abs=1e-3)
    assert np.allclose(np.sum(probs, axis=1), 1.0, atol=1e-6)


def test_normalize_output_layout_variants():
    arr_nhwc = np.arange(1 * 2 * 3 * 4, dtype=np.float32).reshape(1, 2, 3, 4)
    out = loader_mod._normalize_output_layout(arr_nhwc, 2, 3, 4)
    assert out.shape == (2, 3, 4)

    arr_nchw = np.arange(1 * 4 * 2 * 3, dtype=np.float32).reshape(1, 4, 2, 3)
    out = loader_mod._normalize_output_layout(arr_nchw, 2, 3, 4)
    assert out.shape == (2, 3, 4)
    # NCHW[0, :, 0, 0] == NHWC[0, 0, :].
    assert np.allclose(out[0, 0], arr_nchw[0, :, 0, 0])

    arr_flat = np.arange(24, dtype=np.float32)
    out = loader_mod._normalize_output_layout(arr_flat, 2, 3, 4)
    assert out.shape == (2, 3, 4)


def test_post_process_faces_single_detection():
    events = loader_mod._post_process_faces(
        raw_outputs=_build_synthetic_outputs(),
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    ev = events[0]
    assert ev['event_type'] == 'face'
    assert ev['class_name'] == 'face'
    assert ev['source_camera'] == 'oak-d'
    assert ev['confidence'] == pytest.approx(1.0, abs=0.01)
    # bbox == anchor (regression=0): cx=(80.5)/160, cy=(46.5)/92, w=16/1280, h=16/736.
    assert ev['bbox_cx'] == pytest.approx(80.5 / 160, abs=1e-3)
    assert ev['bbox_cy'] == pytest.approx(46.5 / 92, abs=1e-3)
    assert ev['bbox_w'] == pytest.approx(16 / 1280, abs=1e-3)
    assert ev['bbox_h'] == pytest.approx(16 / 736, abs=1e-3)
    # PR-A: embedding_id / display_name пустые.
    assert ev['embedding_id'] == ''
    assert ev['display_name'] == ''


def test_post_process_faces_confidence_threshold_drops_all():
    # Все class-логиты -> face-score ~0 (face weak), порог 0.6 -> [].
    events = loader_mod._post_process_faces(
        raw_outputs=_build_synthetic_outputs(),
        source_camera='oak-d',
        confidence_threshold=0.999,
        nms_iou_threshold=0.45,
    )
    # Целевая детекция имеет face-score ~1.0 > 0.999? Нет: 1.0 > 0.999 → keep.
    # Поэтому отдельный кейс: занижаем face-score целевой детекции.
    outputs = _build_synthetic_outputs()
    outputs[1][46, 80, 1] = 0.0   # face weak на цели
    outputs[1][46, 80, 0] = 10.0  # bg strong
    events = loader_mod._post_process_faces(
        raw_outputs=outputs,
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert events == []


def test_post_process_faces_empty_outputs():
    assert loader_mod._post_process_faces(
        raw_outputs=[],
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    ) == []


def test_make_face_loader_factory_routing():
    stub = loader_mod.make_face_loader(
        hailo_enabled=False, hef_path=None, stub_period_sec=1.0
    )
    assert isinstance(stub, yolo_loader_mod.StubHEFLoader)

    real_no_hef = loader_mod.make_face_loader(
        hailo_enabled=True, hef_path=None, stub_period_sec=1.0
    )
    assert isinstance(real_no_hef, yolo_loader_mod.StubHEFLoader)

    real = loader_mod.make_face_loader(
        hailo_enabled=True,
        hef_path='/tmp/retinaface_mobilenet_v1.hef',
        stub_period_sec=1.0,
    )
    assert isinstance(real, loader_mod.RetinaFaceLoader)


def test_stub_face_mode_marks_events_as_stub():
    """Stub-режим лицевой ноды обязан помечать выдумку (ADR-0089 §2.2)."""
    stub = loader_mod.make_face_loader(
        hailo_enabled=False, hef_path=None, stub_period_sec=0.0
    )
    events = stub.infer(frame_id='oak_rgb_frame', image=None)
    assert len(events) == 1
    ev = events[0]
    # НЕ "face" — выдуманное событие помечено "stub", чтобы Личность
    # не здоровалась с несуществующими людьми (issue #2599 §3).
    assert ev['event_type'] == yolo_loader_mod.STUB_EVENT_TYPE
    assert yolo_loader_mod.is_stub_event(ev) is True


def test_retinaface_loader_returns_empty_on_none_image():
    real = loader_mod.RetinaFaceLoader(hef_path='/tmp/nonexistent.hef')
    # image=None -> [] БЕЗ инициализации HailoRT (heartbeat-контракт).
    assert real.infer(frame_id='x', image=None) == []
