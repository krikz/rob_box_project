"""Unit-тесты для асимметричного letterbox + ADR-0104 bbox-денормализации.

Зачем этот файл (issue #2531 acceptance #9):

Старый `_post_process_detections` делил bbox на input_w/input_h
(letterbox-space 640×640) БЕЗ учёта letterbox-pad'а. На несимметричном
кадре (640×480 — OAK-D, ceiling-camera) letterbox добавляет вертикальный
паддинг, и bbox оказывался смещённым и сжатым относительно объекта.

ADR-0104 (этот issue) вводит LetterboxInfo и unproject: bbox сначала
переводится из letterbox-space в resized-space (вычитаем pad_left/top),
потом из resized-space в исходный кадр (делим на scale), и только
потом нормализуется на orig_w/orig_h.

Эти тесты — единственный существующий код, который способен поймать
эту ошибку. Старый ``test_vision_hailo_phase15.py`` использует
квадратный синтетический вход (640×640), где letterbox-pad = 0,
поэтому расхождение паддинга физически не проявляется.

Запуск:
    PYTHONPATH=src/rob_box_perception \\
        python3 -m pytest \\
        src/rob_box_perception/test/unit/test_vision_hailo_letterbox_unproject.py \\
        -v
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest


# ---------- import target under test ------------------------------------

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]


def _import_module():
    if str(_PKG_ROOT) not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT))
    return importlib.import_module('rob_box_perception.vision_hailo_loader')


loader_mod = _import_module()


# ============================================================================
# LetterboxInfo.unproject — pure function, 1 кейс на корректность формулы
# ============================================================================

def test_letterbox_info_unproject_symmetric_no_pad():
    """640×640 → scale=1.0, pad=0. unproject — no-op."""
    info = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=0,
        orig_w=640, orig_h=640,
        letterbox_w=640, letterbox_h=640,
    )
    cx, cy, bw, bh = info.unproject(100.0, 200.0, 50.0, 80.0)
    assert cx == pytest.approx(100.0)
    assert cy == pytest.approx(200.0)
    assert bw == pytest.approx(50.0)
    assert bh == pytest.approx(80.0)


def test_letterbox_info_unproject_640x480_with_vertical_pad():
    """640×480 → letterbox 640×640: scale=1.0, pad_top=80 (вертикальный).

    YOLOv8n выдаёт bbox в letterbox-space. Без unproject старый код делил
    бы cx/cy/bw/bh на 640 (input_h), что:
      - сдвигало бы cy вверх на 80/640=0.125 (относительно исходного кадра),
      - сжимало бы bh на 80/640=0.125 (падинг уменьшал видимую высоту).

    С unproject: (cx - pad_top) → в resized 640×480, /scale → в исходный
    кадр, нормализация на orig_h=480.
    """
    info = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=80,
        orig_w=640, orig_h=480,
        letterbox_w=640, letterbox_h=640,
    )
    # Bbox в letterbox-space: cx=320 (центр), cy=320 (центр letterbox),
    # bw=100, bh=200. В исходном 640×480 кадре он должен быть
    # cx=320 (центр), cy=320-80=240 (центр), bw=100, bh=200.
    cx, cy, bw, bh = info.unproject(320.0, 320.0, 100.0, 200.0)
    assert cx == pytest.approx(320.0)
    assert cy == pytest.approx(240.0)
    assert bw == pytest.approx(100.0)
    assert bh == pytest.approx(200.0)


def test_letterbox_info_unproject_320x240_scaled():
    """320×240 → letterbox 640×640: scale=2.0, pad_left=0, pad_top=0.

    Маленький кадр растягивается в 2×, bbox в letterbox-space будет в 2×
    больше чем в исходном. unproject должен восстановить исходный размер.
    """
    info = loader_mod.LetterboxInfo(
        scale=2.0, pad_left=0, pad_top=0,
        orig_w=320, orig_h=240,
        letterbox_w=640, letterbox_h=640,
    )
    # bbox в letterbox-space: cx=160, cy=120, bw=80, bh=60.
    # В исходном 320×240: cx=80, cy=60, bw=40, bh=30.
    cx, cy, bw, bh = info.unproject(160.0, 120.0, 80.0, 60.0)
    assert cx == pytest.approx(80.0)
    assert cy == pytest.approx(60.0)
    assert bw == pytest.approx(40.0)
    assert bh == pytest.approx(30.0)


def test_letterbox_info_unproject_scale_zero_keeps_input():
    """scale=0 (degenerate) — no division by zero, return as-is."""
    info = loader_mod.LetterboxInfo(
        scale=0.0, pad_left=10, pad_top=20,
        orig_w=640, orig_h=480,
        letterbox_w=640, letterbox_h=640,
    )
    cx, cy, bw, bh = info.unproject(100.0, 200.0, 50.0, 80.0)
    assert (cx, cy, bw, bh) == (100.0, 200.0, 50.0, 80.0)


# ============================================================================
# _post_process_detections с letterbox_info — асимметричный кейс
# ============================================================================

def _make_fake_output_asymmetric(
    n_anchors: int = loader_mod.DEFAULT_NUM_ANCHORS,
    n_classes: int = loader_mod.DEFAULT_NUM_CLASSES,
    seed: int = 42,
) -> np.ndarray:
    """Сгенерировать фейковый (84, 8400) output с одной 'person' детекцией
    в центре letterbox-тензора 640×640.
    """
    rng = np.random.default_rng(seed)
    out = rng.uniform(0, 0.1, size=(4 + n_classes, n_anchors)).astype(np.float32)
    # Anchor 0 → strong "person" detection в центре letterbox (320, 320).
    out[0, 0] = 320.0  # cx
    out[1, 0] = 320.0  # cy
    out[2, 0] = 100.0  # w
    out[3, 0] = 200.0  # h
    out[4:, :] = 0.0
    out[4, 0] = 0.85  # score для class=0 (person)
    return out


def test_post_process_asymmetric_letterbox_unprojects_to_orig():
    """640×480 кадр → letterbox с pad_top=80, scale=1.0.

    HEF выдаёт bbox в letterbox-space (cx=320, cy=320, bw=100, bh=200).
    После unproject в исходный кадр 640×480:
        cx = (320 - 0) / 1.0 = 320     → normalized: 320/640 = 0.5
        cy = (320 - 80) / 1.0 = 240     → normalized: 240/480 = 0.5
        bw = 100 / 1.0 = 100            → normalized: 100/640 ≈ 0.15625
        bh = 200 / 1.0 = 200            → normalized: 200/480 ≈ 0.4167

    Старый код (БЕЗ letterbox_info) делил бы на input_h=640 и получил
    cy=0.5 (случайно правильно — но только потому что мы в центре),
    bh=0.3125 (на 25% меньше — bbox сжат). Этот кейс ловит расхождение.
    """
    raw = _make_fake_output_asymmetric()
    info = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=80,
        orig_w=640, orig_h=480,
        letterbox_w=640, letterbox_h=640,
    )
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
        letterbox_info=info,
    )
    assert len(events) == 1
    ev = events[0]
    assert ev['source_camera'] == 'oak-d'
    assert ev['event_type'] == 'person'
    # bbox в координатах исходного кадра 640×480.
    assert ev['bbox_cx'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_cy'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_w'] == pytest.approx(0.15625, abs=0.01)
    assert ev['bbox_h'] == pytest.approx(0.4167, abs=0.01)


def test_post_process_320x240_scaled_letterbox():
    """320×240 кадр → letterbox 640×640, scale=2.0, без паддинга.

    HEF выдаёт bbox в letterbox-space (cx=320, cy=240, bw=100, bh=200).
    После unproject:
        cx = (320 - 0) / 2.0 = 160        → normalized: 160/320 = 0.5
        cy = (240 - 0) / 2.0 = 120        → normalized: 120/240 = 0.5
        bw = 100 / 2.0 = 50               → normalized: 50/320 ≈ 0.15625
        bh = 200 / 2.0 = 100              → normalized: 100/240 ≈ 0.4167

    Без unproject (back-compat): cx=0.5, cy=0.375, bw=0.15625, bh=0.3125.
    """
    raw = _make_fake_output_asymmetric()
    raw[1, 0] = 240.0  # cy=240 в letterbox-space (а не 320)
    info = loader_mod.LetterboxInfo(
        scale=2.0, pad_left=0, pad_top=0,
        orig_w=320, orig_h=240,
        letterbox_w=640, letterbox_h=640,
    )
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
        letterbox_info=info,
    )
    assert len(events) == 1
    ev = events[0]
    assert ev['bbox_cx'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_cy'] == pytest.approx(0.5, abs=0.01)
    assert ev['bbox_w'] == pytest.approx(0.15625, abs=0.01)
    assert ev['bbox_h'] == pytest.approx(0.4167, abs=0.01)


def test_post_process_no_letterbox_info_keeps_backward_compat():
    """letterbox_info=None → bbox нормализуется на input_w/h (старое поведение).

    Это back-compat для тестов с квадратным синтетическим входом, где
    pad=0 и scale=1, поэтому старый и новый путь дают одинаковый ответ.
    """
    raw = _make_fake_output_asymmetric()
    events_no_info = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    info_identity = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=0,
        orig_w=640, orig_h=640,
        letterbox_w=640, letterbox_h=640,
    )
    events_with_identity = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=640,
        input_h=640,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
        letterbox_info=info_identity,
    )
    assert len(events_no_info) == len(events_with_identity) == 1
    ev_no = events_no_info[0]
    ev_yes = events_with_identity[0]
    for field in ('bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h'):
        assert ev_no[field] == pytest.approx(ev_yes[field], abs=1e-5), (
            f'back-compat: {field}={ev_no[field]} != {ev_yes[field]}'
        )
