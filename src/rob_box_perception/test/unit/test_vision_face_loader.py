"""Unit-тесты для vision_face_loader (ADR-0089 Phase 2, issue #2599 PR-A,
issue #2773 — декод landmarks).

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
  9. _decode_landmarks — RetinaFace landmark regression -> normalized
     (x, y) x 5 точек, letterbox-space (issue #2773).
  10. _unproject_landmarks_normalized — letterbox-space -> исходный кадр,
      формула 1:1 с _xyxy_to_cxcywh_normalized.
  11. _post_process_faces — ключ 'landmarks' в каждом событии: декод
      согласован с боксом того же события через тот же confidence-filter
      и NMS-отбор, unproject через LetterboxInfo, None-фолбек когда
      landmark-тензоров нет на входе или декод дал не-конечные числа.

ЧЕСТНО (issue #2773 acceptance): масштабный делитель декодера landmarks
(10, тот же, что у box-центра) сверен с эталонной реализацией
biubug6/Pytorch_Retinaface, а НЕ измерен на реальном HEF — тесты ниже
проверяют формулу, которую код реализует, а не то, что декод совпадает
с фактическим выходом retinaface_mobilenet_v1.hef на роботе (это можно
проверить только на живом железе, см. отчёт по issue).
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


def _build_synthetic_outputs_with_landmarks():
    """Как ``_build_synthetic_outputs``, плюс ненулевая landmark-регрессия.

    Точка 0 (left_eye) детекции branch0/cell(46,80)/anchor0 сдвинута по
    x на dx=10 (-> x = a_cx + a_w, см. ``_decode_landmarks``). Остальные
    4 точки — регрессия 0 (совпадают с центром анкера).
    """
    outputs = _build_synthetic_outputs()
    lmk_t = outputs[2]  # branch 0: (box=outputs[0], class=outputs[1], landmarks=outputs[2])
    lmk_t[46, 80, 0] = 10.0  # left_eye dx
    lmk_t[46, 80, 1] = 0.0   # left_eye dy
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


# ---------- landmarks decode (issue #2773) ---------------------------------


def test_decode_landmarks_identity_regression():
    """Регрессия 0 -> все 5 точек совпадают с центром анкера."""
    anchors = np.array([[0.5, 0.5, 0.1, 0.2]], dtype=np.float32)
    landm = np.zeros((1, 10), dtype=np.float32)
    decoded = loader_mod._decode_landmarks(landm, anchors)
    assert decoded.shape == (1, 10)
    for k in range(5):
        assert decoded[0, 2 * k] == pytest.approx(0.5, abs=1e-6)
        assert decoded[0, 2 * k + 1] == pytest.approx(0.5, abs=1e-6)


def test_decode_landmarks_positive_regression_shifts_point():
    """dx/dy у ОДНОЙ точки сдвигают только её, остальные точки не трогает."""
    anchors = np.array([[0.5, 0.5, 0.1, 0.2]], dtype=np.float32)
    landm = np.zeros((1, 10), dtype=np.float32)
    # dx=10 у точки 0 (left_eye): x = a_cx + 10/10 * a_w = 0.5 + 0.1 = 0.6.
    landm[0, 0] = 10.0
    landm[0, 1] = 0.0
    # dy=10 у точки 2 (nose): y = a_cy + 10/10 * a_h = 0.5 + 0.2 = 0.7.
    landm[0, 5] = 10.0
    decoded = loader_mod._decode_landmarks(landm, anchors)
    assert decoded[0, 0] == pytest.approx(0.6, abs=1e-6)   # left_eye x
    assert decoded[0, 1] == pytest.approx(0.5, abs=1e-6)   # left_eye y
    assert decoded[0, 2] == pytest.approx(0.5, abs=1e-6)   # right_eye x (untouched)
    assert decoded[0, 3] == pytest.approx(0.5, abs=1e-6)   # right_eye y (untouched)
    assert decoded[0, 4] == pytest.approx(0.5, abs=1e-6)   # nose x (untouched)
    assert decoded[0, 5] == pytest.approx(0.7, abs=1e-6)   # nose y (shifted)


# ---------- unproject landmarks (issue #2773) -------------------------------


def test_unproject_landmarks_normalized_none_letterbox_is_identity():
    landmarks = np.array(
        [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.05]], dtype=np.float32
    )
    out = loader_mod._unproject_landmarks_normalized(landmarks, None)
    assert np.allclose(out, landmarks)


def test_unproject_landmarks_normalized_matches_manual_formula():
    """Формула обязана быть 1:1 с ``_xyxy_to_cxcywh_normalized`` для bbox'а."""
    info = yolo_loader_mod.LetterboxInfo(
        scale=0.5,
        pad_left=100,
        pad_top=50,
        orig_w=1000,
        orig_h=800,
        letterbox_w=1280,
        letterbox_h=736,
    )
    # Одна и та же точка (0.2, 0.3) во всех 5 позициях — достаточно для
    # проверки формулы по каждой координате.
    point = [0.2, 0.3]
    landmarks = np.array([point * 5], dtype=np.float32)
    out = loader_mod._unproject_landmarks_normalized(landmarks, info)

    expected_x = (0.2 * 1280 - 100) / 0.5 / 1000
    expected_y = (0.3 * 736 - 50) / 0.5 / 800
    for k in range(5):
        assert out[0, 2 * k] == pytest.approx(expected_x, abs=1e-6)
        assert out[0, 2 * k + 1] == pytest.approx(expected_y, abs=1e-6)


# ---------- _post_process_faces: контракт ключа 'landmarks' (issue #2773) --


def test_post_process_faces_includes_landmarks_key():
    outputs = _build_synthetic_outputs_with_landmarks()
    events = loader_mod._post_process_faces(
        raw_outputs=outputs,
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    ev = events[0]
    assert 'landmarks' in ev
    lmk = ev['landmarks']
    assert lmk is not None
    assert len(lmk) == 10

    a_cx = 80.5 / 160
    a_cy = 46.5 / 92
    a_w = 16 / 1280
    # left_eye (точка 0): dx=10 -> x = a_cx + a_w, y = a_cy (dy=0).
    assert lmk[0] == pytest.approx(a_cx + a_w, abs=1e-3)
    assert lmk[1] == pytest.approx(a_cy, abs=1e-3)
    # Остальные 4 точки без регрессии -> совпадают с центром анкера.
    for k in range(1, 5):
        assert lmk[2 * k] == pytest.approx(a_cx, abs=1e-3)
        assert lmk[2 * k + 1] == pytest.approx(a_cy, abs=1e-3)


def test_post_process_faces_landmarks_match_correct_detection_after_nms():
    """Два непересекающихся лица — landmarks каждого события обязаны
    относиться к ЕГО ЖЕ bbox, а не переставиться местами при NMS-отборе
    (issue #2773: "точки должны пройти тот же отбор... что и боксы").
    """
    outputs = _build_synthetic_outputs()

    # Второе лицо: branch 0, cell (10, 20), anchor 1 (min_size=32) —
    # далеко от первого (cell (46, 80), anchor 0), NMS их не тронет.
    outputs[0][10, 20, 4:8] = [0.0, 0.0, 0.0, 0.0]   # box regression anchor1 = 0
    outputs[1][10, 20, 2] = 0.0                       # anchor1 bg
    outputs[1][10, 20, 3] = 10.0                      # anchor1 face

    lmk_t = outputs[2]
    lmk_t[46, 80, 0:2] = [10.0, 0.0]     # face #1, left_eye: dx=10
    lmk_t[10, 20, 10:12] = [0.0, 10.0]   # face #2 (anchor1 -> channels 10:20), left_eye: dy=10

    events = loader_mod._post_process_faces(
        raw_outputs=outputs,
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 2

    face1_cx = 80.5 / 160
    face1_cy = 46.5 / 92
    face1_w = 16 / 1280
    face2_cx = 20.5 / 160
    face2_cy = 10.5 / 92
    face2_h = 32 / 736

    ev1 = next(ev for ev in events if ev['bbox_cx'] == pytest.approx(face1_cx, abs=1e-3))
    ev2 = next(ev for ev in events if ev['bbox_cx'] == pytest.approx(face2_cx, abs=1e-3))

    assert ev1['landmarks'][0] == pytest.approx(face1_cx + face1_w, abs=1e-3)
    assert ev1['landmarks'][1] == pytest.approx(face1_cy, abs=1e-3)

    assert ev2['landmarks'][0] == pytest.approx(face2_cx, abs=1e-3)
    assert ev2['landmarks'][1] == pytest.approx(face2_cy + face2_h, abs=1e-3)


def test_post_process_faces_landmarks_unprojected_through_letterbox():
    """Идентичный letterbox (scale=1, pad=0, orig==letterbox) обязан давать
    те же координаты, что и letterbox_info=None — проверка, что unproject
    реально вызывается и не ломает identity-случай.
    """
    info = yolo_loader_mod.LetterboxInfo(
        scale=1.0,
        pad_left=0,
        pad_top=0,
        orig_w=loader_mod.DEFAULT_INPUT_W,
        orig_h=loader_mod.DEFAULT_INPUT_H,
        letterbox_w=loader_mod.DEFAULT_INPUT_W,
        letterbox_h=loader_mod.DEFAULT_INPUT_H,
    )

    events_none = loader_mod._post_process_faces(
        raw_outputs=_build_synthetic_outputs_with_landmarks(),
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
        letterbox_info=None,
    )
    events_identity = loader_mod._post_process_faces(
        raw_outputs=_build_synthetic_outputs_with_landmarks(),
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
        letterbox_info=info,
    )
    assert len(events_none) == len(events_identity) == 1
    assert events_none[0]['landmarks'] == pytest.approx(
        events_identity[0]['landmarks'], abs=1e-5
    )


def test_post_process_faces_landmarks_none_when_tensors_missing():
    """Обратная совместимость: вход без landmark-тензоров (6, а не 9) ->
    ключ 'landmarks' присутствует со значением None (issue #2773:
    "если landmarks недоступны... ключ должен присутствовать со
    значением None, а не отсутствовать")."""
    outputs = _build_synthetic_outputs()
    # Оставляем только (box, class) на branch, выкидываем landmarks
    # (индексы 2, 5, 8 в порядке (box, class, landmarks) x 3 branches).
    box_class_only = [t for i, t in enumerate(outputs) if i % 3 != 2]
    assert len(box_class_only) == 6

    events = loader_mod._post_process_faces(
        raw_outputs=box_class_only,
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    assert 'landmarks' in events[0]
    assert events[0]['landmarks'] is None


def test_post_process_faces_landmarks_none_on_non_finite_decode():
    """Сломанная (не-конечная) landmark-регрессия -> None, а не NaN наружу."""
    outputs = _build_synthetic_outputs()
    outputs[2][46, 80, 0] = float('inf')  # left_eye dx = inf

    events = loader_mod._post_process_faces(
        raw_outputs=outputs,
        source_camera='oak-d',
        confidence_threshold=0.6,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    assert events[0]['landmarks'] is None
