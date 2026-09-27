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
import logging
import sys
from pathlib import Path

import numpy as np
import pytest


# ---------- import target under test ------------------------------------

# `parents[2]` = .../src/rob_box_perception → содержит пакет rob_box_perception/.
# (тот же приём, что и в test_vision_event_parity.py; ранее был захардкод
# `/home/builder/.worktrees/t_40ee0b73/` — ломалось после cleanup этого worktree).
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]


def _import_module():
    if str(_PKG_ROOT) not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT))
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
# Ragged on-chip NMS output (issue #2703)
# ============================================================================
#
# raw-лог 17.09.2026 (vision_hailo_loader.py:581): VisibleDeprecationWarning
# "Creating an ndarray from ragged nested sequences" на np.asarray(raw_output).
# Гипотеза: HEF с on-chip NMS postprocessing (штатная поставка yolov8n.hef
# из hailo_model_zoo) отдаёт get_buffer() как List[np.ndarray] длины
# num_classes, где raw_output[class_id] — (n, 5) [y_min, x_min, y_max,
# x_max, score], разной длины по классам -> ragged. Старый код падал в
# ветку "unknown shape" -> детекции молча схлопывались в []. Формат НЕ
# проверен эмпирически на живом HAT+ (нет доступа к железу в этой среде) —
# см. docstring _is_nms_postprocessed_output/_post_process_nms_output.

def _make_empty_ragged_output(n_classes: int = loader_mod.DEFAULT_NUM_CLASSES):
    """Список из n_classes пустых (0, 5) массивов — валидный "0 детекций"."""
    return [np.zeros((0, 5), dtype=np.float32) for _ in range(n_classes)]


def test_post_process_ragged_nms_output_is_detected():
    """Эвристика _is_nms_postprocessed_output различает ragged list от dense tensor."""
    ragged = _make_empty_ragged_output()
    ragged[0] = np.array([[0.1, 0.1, 0.5, 0.5, 0.9]], dtype=np.float32)
    assert loader_mod._is_nms_postprocessed_output(ragged) is True

    dense = _make_fake_output()  # (84, 8400) numpy ndarray
    assert loader_mod._is_nms_postprocessed_output(dense) is False

    # Все-пустой список — тоже валидный NMS output (0 детекций везде).
    assert loader_mod._is_nms_postprocessed_output(_make_empty_ragged_output()) is True

    # Не список/не ndarray (например, обычный int) — не NMS output.
    assert loader_mod._is_nms_postprocessed_output(42) is False
    assert loader_mod._is_nms_postprocessed_output([]) is False


def test_post_process_ragged_nms_output_yolov8n_person():
    """Регресс issue #2703: ragged NMS output НЕ должен схлопываться в []."""
    raw = _make_empty_ragged_output()
    # person (class_id=0): letterbox-space [0,1] bbox + score.
    raw[0] = np.array([[0.2, 0.3, 0.8, 0.7, 0.87]], dtype=np.float32)

    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1, (
        f'ragged NMS output схлопнулся: events={events!r} '
        f'(issue #2703 regression — np.asarray() на per-class ragged list '
        f'терял детекции без предупреждения вызывающему коду).'
    )
    ev = events[0]
    assert ev['class_name'] == 'person'
    assert ev['event_type'] == 'person'
    assert ev['class_id'] == 0
    assert ev['confidence'] == pytest.approx(0.87, abs=1e-4)
    assert 0.0 < ev['bbox_cx'] < 1.0
    assert 0.0 < ev['bbox_w'] < 1.0
    assert ev['distance_m'] == -1.0


def test_post_process_ragged_nms_output_confidence_filter():
    """Ragged NMS путь тоже уважает confidence_threshold."""
    raw = _make_empty_ragged_output()
    raw[0] = np.array([[0.2, 0.3, 0.8, 0.7, 0.3]], dtype=np.float32)  # ниже 0.5
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events == []


def test_post_process_ragged_nms_output_all_empty_returns_empty():
    """Ragged NMS output, все классы без детекций -> [] (не raise, не warning)."""
    raw = _make_empty_ragged_output()
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert events == []


def test_post_process_ragged_nms_output_multi_class():
    """Несколько классов с детекциями в ragged NMS output — все доходят."""
    raw = _make_empty_ragged_output()
    raw[0] = np.array([[0.1, 0.1, 0.5, 0.5, 0.9]], dtype=np.float32)    # person
    raw[41] = np.array([[0.6, 0.6, 0.9, 0.9, 0.75]], dtype=np.float32)  # cup
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    names = sorted(ev['class_name'] for ev in events)
    assert names == ['cup', 'person']


def test_post_process_ragged_nms_output_multi_detection_same_class():
    """Несколько детекций одного класса в ragged NMS output — обе доходят.

    NMS уже применён HailoRT на чипе — постпроцессинг здесь НЕ должен
    повторно подавлять пересекающиеся боксы (в отличие от dense-пути,
    где _nms_per_class ещё нужен).
    """
    raw = _make_empty_ragged_output()
    raw[0] = np.array([
        [0.1, 0.1, 0.4, 0.4, 0.9],
        [0.5, 0.5, 0.9, 0.9, 0.8],
    ], dtype=np.float32)
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 2
    confidences = sorted(ev['confidence'] for ev in events)
    assert confidences == pytest.approx([0.8, 0.9], abs=1e-4)


def test_post_process_ragged_nms_output_with_letterbox_unproject():
    """Ragged NMS путь unproject'ит bbox через LetterboxInfo так же, как dense-путь."""
    raw = _make_empty_ragged_output()
    # Детекция занимает весь letterbox-кадр (0..1 по обеим осям).
    raw[0] = np.array([[0.0, 0.0, 1.0, 1.0, 0.9]], dtype=np.float32)
    letterbox_info = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=80, pad_top=0,
        orig_w=480, orig_h=640, letterbox_w=640, letterbox_h=640,
    )
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
        letterbox_info=letterbox_info,
    )
    assert len(events) == 1
    ev = events[0]
    # letterbox px: x in [0,640], cx_px=320; unproject: (320-80)/1=240;
    # normalize: 240/480=0.5.
    assert ev['bbox_cx'] == pytest.approx(0.5, abs=1e-3)


def test_post_process_ragged_nms_output_row_as_plain_list():
    """raw_output элементы могут прийти как обычные Python list (не ndarray)."""
    raw = [[] for _ in range(loader_mod.DEFAULT_NUM_CLASSES)]
    raw[0] = [[0.2, 0.3, 0.8, 0.7, 0.87]]
    events = loader_mod._post_process_detections(
        raw_output=raw,
        source_camera='oak-d',
        input_w=loader_mod.DEFAULT_INPUT_W,
        input_h=loader_mod.DEFAULT_INPUT_H,
        confidence_threshold=0.5,
        nms_iou_threshold=0.45,
    )
    assert len(events) == 1
    assert events[0]['class_name'] == 'person'


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
# RealHEFLoader: путь инициализации и run() с моком hailo_platform
# ----------------------------------------------------------------------------
# Issue #2538: тесты в блоке «contract (не init, не run)» выше НЕ доводили
# до строк 230-258 (init) и 300-315 (run). Это значит, что баг
# «смешение двух поколений HailoRT API» мог пройти CI незамеченным.
#
# Здесь — моки, которые доводят _ensure_initialized() до конца (включая
# create_bindings) и проверяют, что infer() возвращает numpy в
# _post_process_detections, а не binding-объект. При «откате» фикса
# (возврате к input_vstreams/get_input_binding или к передаче списка
# outputs в run()) эти тесты падают намеренно.
# ============================================================================

class _FakeInferStream:
    """Минимальный мок для InferModel.input(name)/output(name).

    Нужен shape (для np.empty в init) и set_format_type (YOLOv8n HEF).
    """

    def __init__(self, shape):
        self._shape = list(shape)

    @property
    def shape(self):
        return list(self._shape)

    def set_format_type(self, _fmt):
        # no-op — HailoRT реально дергает C-API, в моке безвредно.
        return None


class _FakeInferModel:
    """Mock InferModel.create_infer_model().configure() chain."""

    def __init__(self, input_shape=(1, 640, 640, 3), output_shape=(1, 84, 8400)):
        self._input_shape = input_shape
        self._output_shape = output_shape

    def set_batch_size(self, _n):
        return None

    def input(self, _name=""):
        return _FakeInferStream(self._input_shape)

    def output(self, _name=""):
        return _FakeInferStream(self._output_shape)

    @property
    def input_names(self):
        return ['images']

    @property
    def output_names(self):
        return ['outputs']

    def configure(self):
        return _FakeConfigured(self._input_shape, self._output_shape)


class _FakeBindingStream:
    """Mock для Bindings.input(name).set_buffer(np) и .output(name).get_buffer()."""

    def __init__(self, np_array):
        self._buf = np_array

    def set_buffer(self, np_array):
        self._buf = np_array

    def get_buffer(self, *_args, **_kwargs):
        return self._buf


class _FakeBindings:
    """Mock ConfiguredInferModel.create_bindings().Bindings."""

    def __init__(self, input_buffers, output_buffers):
        # Сохраняем исходные numpy-массивы; после run() модифицируем.
        self._input_buffers = dict(input_buffers)
        self._output_buffers = dict(output_buffers)
        self._in_streams = {
            n: _FakeBindingStream(arr) for n, arr in input_buffers.items()
        }
        self._out_streams = {
            n: _FakeBindingStream(arr) for n, arr in output_buffers.items()
        }

    def input(self, name=""):
        return self._in_streams[name]

    def output(self, name=""):
        return self._out_streams[name]


class _FakeConfigured:
    """Mock ConfiguredInferModel.run([bindings], timeout)."""

    def __init__(self, input_shape, output_shape):
        self._input_shape = input_shape
        self._output_shape = output_shape
        self.run_calls: list = []
        self._output_filler = None  # задаётся в тесте через setattr

    def create_bindings(self, input_buffers=None, output_buffers=None):
        # Современный API принимает dict'ы numpy и возвращает Bindings.
        return _FakeBindings(input_buffers or {}, output_buffers or {})

    def activate(self):
        return None

    def run(self, bindings, timeout):
        """Синхронный run: заполняем output_buffer «детекциями»."""
        self.run_calls.append({'bindings': bindings, 'timeout': timeout})
        # Эмулируем поведение HailoRT: после run() output-буферы заполнены.
        for binding in bindings:
            for name, stream in binding._out_streams.items():
                if self._output_filler is not None:
                    stream.set_buffer(self._output_filler())


class _FakeVDevice:
    """Mock VDevice: create_infer_model(hef_path) → InferModel."""

    def __init__(self, infer_model):
        self._infer_model = infer_model

    def create_infer_model(self, hef_path):
        assert hef_path  # Smoke-check: путь передан.
        return self._infer_model


class _FakeHailoPlatform:
    """Заглушка модуля hailo_platform.

    Подменяется в sys.modules под именем 'hailo_platform' для одного теста.
    Содержит минимальный публичный API, который зовёт RealHEFLoader.
    """

    VDevice = None  # filled by _install()
    HailoSchedulingAlgorithm = None
    FormatType = None


def _install_fake_hailo_platform(
    monkeypatch,
    fake_vdevice,
    with_scheduling=True,
    with_format=True,
):
    """Подменить `hailo_platform` в sys.modules временным модулем."""
    import types

    fake_mod = types.ModuleType('hailo_platform')
    fake_mod.VDevice = type('VDevice', (), {
        'create_params': staticmethod(lambda: types.SimpleNamespace(
            scheduling_algorithm=None,
            device_id=None,
        )),
        '__init__': lambda self, params=None: None,
        '__new__': lambda cls, *a, **kw: super().__new__(cls),
    }) if with_scheduling else None

    # Реальный VDevice-объект сфабрикован снаружи (для контроля поведения).
    fake_mod.VDevice = lambda *a, **kw: fake_vdevice

    if with_scheduling:
        fake_mod.HailoSchedulingAlgorithm = types.SimpleNamespace(
            ROUND_ROBIN='ROUND_ROBIN',
        )
    if with_format:
        fake_mod.FormatType = types.SimpleNamespace(UINT8='UINT8')

    # Также подменяем submodule hailo_platform.pyhailort (lazy import).
    fake_sub = types.ModuleType('hailo_platform.pyhailort')
    fake_mod.pyhailort = fake_sub

    monkeypatch.setitem(sys.modules, 'hailo_platform', fake_mod)
    monkeypatch.setitem(sys.modules, 'hailo_platform.pyhailort', fake_sub)
    return fake_mod


def test_real_loader_init_with_mock_hailo_platform_succeeds(
    tmp_path, monkeypatch,
):
    """Регрессионный тест issue #2538.

    С моком `hailo_platform` _ensure_initialized() должен дойти до конца:
        VDevice.create_infer_model(hef) → InferModel.configure() →
        create_bindings(input_buffers, output_buffers).
    До фикса этот путь падал на отсутствии input_vstreams/get_input_binding.
    """
    # HEF-файл должен существовать (FileNotFoundError иначе до mock'а).
    hef = tmp_path / 'yolov8n.hef'
    hef.write_bytes(b'\x00')

    # Создаём согласованную цепочку моков.
    infer_model = _FakeInferModel(
        input_shape=(1, 640, 640, 3),
        output_shape=(1, 84, 8400),
    )
    vdevice = _FakeVDevice(infer_model)
    _install_fake_hailo_platform(monkeypatch, vdevice)

    real = loader_mod.RealHEFLoader(hef_path=str(hef))

    # Lazy-init: должен пройти БЕЗ ошибок.
    real._ensure_initialized()

    # Состояние сохранено: bindings есть, output_name выставлен.
    assert real._bindings is not None, (
        'create_bindings() не вызвана — init упал раньше. '
        'Скорее всего вернулись к input_vstreams/get_input_binding API.'
    )
    assert real._output_name == 'outputs'
    assert real._infer_model is not None
    assert real._configured is not None
    assert real._init_failed is None

    # is_available() теперь True (нет fallback на Exception).
    assert real.is_available() is True


def test_real_loader_infer_with_mock_returns_postprocessed_events(
    tmp_path, monkeypatch,
):
    """Полный pipeline с моком: infer() → run() → numpy → events.

    Проверяет, что в post_process_detections уходит numpy.ndarray
    формы (1, 84, 8400), а НЕ объект binding'а (это и был баг:
    raw_output = output_list[0] возвращал binding-объект).
    """
    hef = tmp_path / 'yolov8n.hef'
    hef.write_bytes(b'\x00')

    # Симулируем «детекцию» в выходном тензоре.
    fake_output = np.zeros((1, 84, 8400), dtype=np.float32)
    # Поставим одну уверенную детекцию person в (320, 320) листа.
    # boxes: cx=320, cy=320, w=200, h=400
    fake_output[0, 0, 0] = 320.0
    fake_output[0, 1, 0] = 320.0
    fake_output[0, 2, 0] = 200.0
    fake_output[0, 3, 0] = 400.0
    # Class score = 0.95 для class 0 (person).
    fake_output[0, 4, 0] = 0.95

    infer_model = _FakeInferModel(
        input_shape=(1, 640, 640, 3),
        output_shape=(1, 84, 8400),
    )
    vdevice = _FakeVDevice(infer_model)
    _install_fake_hailo_platform(monkeypatch, vdevice)

    real = loader_mod.RealHEFLoader(hef_path=str(hef))
    real._ensure_initialized()

    # Подменяем filler в configured._FakeConfigured, чтобы run() заполнил
    # output-буфер fake_output (а не np.zeros по умолчанию).
    real._configured._output_filler = lambda: fake_output.copy()

    # Мок _preprocess: реальный требует cv2, в CI-сборке его нет. Для
    # теста run()→numpy→postprocess этого достаточно — нам важно, чтобы
    # входной tensor попал в bindings.input().set_buffer() и output
    # был прочитан из bindings.output().get_buffer().
    #
    # ADR-0104 (issue #2531 acceptance #9): _preprocess возвращает
    # (tensor, LetterboxInfo). Mock тоже должен вернуть tuple — иначе
    # infer() упадёт на «not enough values to unpack».
    preprocessed = np.zeros((1, 640, 640, 3), dtype=np.uint8)
    fake_letterbox = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=0,
        orig_w=640, orig_h=480,
        letterbox_w=640, letterbox_h=640,
    )
    monkeypatch.setattr(
        real, '_preprocess',
        lambda _img: (preprocessed, fake_letterbox),
    )

    # Запускаем infer() с синтетическим кадром.
    fake_img = np.zeros((480, 640, 3), dtype=np.uint8)
    events = real.infer(frame_id='oak-d', image=fake_img)

    # run() был вызван ровно один раз и с правильной сигнатурой.
    assert len(real._configured.run_calls) == 1
    call = real._configured.run_calls[0]
    # bindings — list с одним объектом; timeout — int (мс).
    assert isinstance(call['bindings'], list) and len(call['bindings']) == 1
    assert isinstance(call['timeout'], int)

    # Output — список событий с person@0.95.
    assert isinstance(events, list)
    assert len(events) == 1
    ev = events[0]
    assert ev['source_camera'] == 'oak-d'
    assert ev['class_name'] == 'person'
    assert ev['class_id'] == 0
    assert abs(ev['confidence'] - 0.95) < 1e-5
    # bbox_cx / bbox_w нормализованы в [0, 1].
    assert 0.0 < ev['bbox_cx'] < 1.0
    assert 0.0 < ev['bbox_w'] < 1.0


def test_real_loader_init_failure_caches_attribute_error(monkeypatch):
    """is_available() возвращает False при AttributeError в init.

    До фикса `except (ImportError, FileNotFoundError, RuntimeError, OSError)`
    НЕ ловил AttributeError — это и был сценарий «тишина + зелёный
    healthcheck» в issue #2538 (Vision Pi: API-drift → AttributeError →
    пролёт наружу → Node продолжает heartbeat).
    """
    # Мок, у которого create_infer_model возвращает объект без input_names
    # (имитация API-drift). is_available() должен вернуть False, не raise.
    class _BrokenInferModel:
        def set_batch_size(self, _n):
            return None

        def input(self, _name=""):
            raise AttributeError(
                "module 'hailo_platform' has no attribute 'input_names' "
                '(fake drift scenario)',
            )

        def configure(self):
            return self

    class _BrokenVDevice:
        def create_infer_model(self, _hef):
            return _BrokenInferModel()

    import types
    fake_mod = types.ModuleType('hailo_platform')
    fake_mod.VDevice = lambda *a, **kw: _BrokenVDevice()
    monkeypatch.setitem(sys.modules, 'hailo_platform', fake_mod)

    real = loader_mod.RealHEFLoader(hef_path='/nonexistent.hef')
    # Должен вернуть False, а НЕ raise AttributeError.
    assert real.is_available() is False
    # И ошибка закеширована (fail-fast при повторных вызовах).
    assert real._init_failed is not None


def test_real_loader_init_failure_is_not_silent_log_spam(tmp_path, monkeypatch):
    """Инициализация с broken HEF → is_available() False, но init_failed
    зафиксирован. Вызов infer() c фейковым кадром даст ОДНУ ошибку в логе,
    а не бесконечный цикл «init-fail + reinit» (см. issue #2538 п.6).
    """
    hef = tmp_path / 'yolov8n.hef'
    hef.write_bytes(b'\x00')

    # Мок где configure() падает (имитация битого HEF).
    class _BrokenInferModel:
        def set_batch_size(self, _n):
            return None

        def input(self, _name=""):
            return _FakeInferStream((1, 640, 640, 3))

        def configure(self):
            raise RuntimeError('HEF parse failed (fake)')

    class _BrokenVDevice:
        def create_infer_model(self, _hef):
            return _BrokenInferModel()

    import types
    fake_mod = types.ModuleType('hailo_platform')
    fake_mod.VDevice = lambda *a, **kw: _BrokenVDevice()
    fake_mod.FormatType = types.SimpleNamespace(UINT8='UINT8')
    monkeypatch.setitem(sys.modules, 'hailo_platform', fake_mod)

    real = loader_mod.RealHEFLoader(hef_path=str(hef))
    assert real.is_available() is False

    # Повторный вызов — должна кидаться та же ошибка (кеш), а не reinit.
    exc1 = real._init_failed
    assert exc1 is not None, (
        'is_available() не закешировал _init_failed — будет reinit-цикл '
        'на каждом тике (см. issue #2538 п.6).'
    )
    # Второй вызов должен re-raise ту же ошибку из кеша.
    with pytest.raises(RuntimeError, match='HEF parse failed'):
        real._ensure_initialized()
    assert real._init_failed is exc1


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


# ============================================================================
# HAILO_STREAM_ABORT(63) recovery (issue #2625)
# ============================================================================
#
# ADR-0112 §5 п.1: hailort.service не изолирует клиентов друг от друга —
# грязная смерть ЛЮБОГО другого процесса на /dev/hailo0 утаскивает пайплайн
# живой ноды в HAILO_STREAM_ABORT(63). До этого фикса `_init_failed`
# кешировался навсегда после ЛЮБОГО сбоя — включая abort уже поднятого
# пайплайна — и переинициализация не происходила в принципе (процесс жив,
# контейнер healthy, событий нет, лечится только `docker restart`).
#
# ЧЕСТНОСТЬ (ADR-0018): тесты ниже используют фейковый `hailo_platform` —
# ни реального железа, ни реального класса исключения HailoRT здесь нет
# (см. docstring `_is_recoverable_stream_abort`). Симулируем сигнатуру
# через `RuntimeError('...HAILO_STREAM_ABORT(63)...')`, ровно как
# `RealHEFLoader.infer()` реально видит её после оборачивания в
# `except Exception as exc` — это НЕ проверено на живом HAT+.
#
# Карточка issue #2625 указывает как "фейк уже есть" на
# `test_hailo_device.py`, но тот файл фейкает только `hailo_device.open_device()`
# (выбор service/exclusive/legacy) — там нет ни `run()`, ни `InferModel`,
# ни `Configured`. Фейк, который умеет `create_infer_model → configure →
# create_bindings → run()`, живёт здесь, в `test_vision_hailo_phase15.py`
# (`_FakeInferModel`/`_FakeConfigured`/`_FakeBindings`/
# `_install_fake_hailo_platform`, см. выше) — тесты abort-recovery
# расширяют ИМЕННО этот, уже существующий набор фейков, а не заводят
# новый. См. PR-описание issue #2625 для развёрнутого объяснения.


class _AbortOnceInferModel(_FakeInferModel):
    """Fake InferModel, чей `run()` кидает `HAILO_STREAM_ABORT(63)` один раз.

    `configure_count` считает вызовы `configure()` (= число (ре)инициализаций
    пайплайна). `_total_runs` — сквозной (через reinit) счётчик вызовов
    `run()`; после `abort_after_n_runs`-го успешного вызова кидается ОДИН
    abort, дальше `run()` снова успешен — эмулирует «сосед по /dev/hailo0
    умер один раз, дальше железо снова доступно».
    """

    def __init__(
        self,
        *,
        abort_after_n_runs: int,
        output_shape=(1, 84, 8400),
        input_shape=(1, 640, 640, 3),
    ) -> None:
        super().__init__(input_shape=input_shape, output_shape=output_shape)
        self.configure_count = 0
        self._abort_after_n_runs = abort_after_n_runs
        self._total_runs = 0
        self._abort_raised = False

    def configure(self):
        self.configure_count += 1
        configured = _FakeConfigured(self._input_shape, self._output_shape)
        model = self
        original_run = configured.run

        def run(bindings, timeout):
            model._total_runs += 1
            if (
                not model._abort_raised
                and model._total_runs == model._abort_after_n_runs
            ):
                model._abort_raised = True
                raise RuntimeError(
                    '[HailoRT] [error] CHECK_SUCCESS failed with '
                    'status=HAILO_STREAM_ABORT(63) - Can\'t handle '
                    'inference request since pipeline status is '
                    'HAILO_STREAM_ABORT(63).'
                )
            return original_run(bindings, timeout)

        configured.run = run
        return configured


class _FlakyVDeviceForReinit:
    """VDevice, чей `create_infer_model()` падает на заданных по номеру вызовах.

    Используется для проверки backoff'а reinit-попыток после abort'а:
    первый вызов (исходная успешная инициализация) не в `fail_on_calls`,
    последующие reinit-попытки — падают, пока не «починится».
    """

    def __init__(self, infer_model, fail_on_calls) -> None:
        self._infer_model = infer_model
        self._fail_on_calls = set(fail_on_calls)
        self.create_calls = 0

    def create_infer_model(self, hef_path):
        assert hef_path
        self.create_calls += 1
        if self.create_calls in self._fail_on_calls:
            raise RuntimeError(
                f'fake create_infer_model failure at call #{self.create_calls}'
            )
        return self._infer_model


def _real_loader_for_abort_test(tmp_path, monkeypatch, vdevice):
    """Общий setup: HEF-файл + RealHEFLoader с замоканным `_preprocess`."""
    hef = tmp_path / 'yolov8n.hef'
    hef.write_bytes(b'\x00')
    _install_fake_hailo_platform(monkeypatch, vdevice)

    real = loader_mod.RealHEFLoader(hef_path=str(hef))
    preprocessed = np.zeros((1, 640, 640, 3), dtype=np.uint8)
    fake_letterbox = loader_mod.LetterboxInfo(
        scale=1.0, pad_left=0, pad_top=0,
        orig_w=640, orig_h=480,
        letterbox_w=640, letterbox_h=640,
    )
    monkeypatch.setattr(
        real, '_preprocess', lambda _img: (preprocessed, fake_letterbox)
    )
    return real


def test_is_recoverable_stream_abort_detects_hailort_signature():
    """`_is_recoverable_stream_abort` матчит формат HailoRT из issue #2625."""
    exc = RuntimeError(
        "[HailoRT] [error] CHECK_SUCCESS failed with "
        "status=HAILO_STREAM_ABORT(63)"
    )
    assert loader_mod._is_recoverable_stream_abort(exc) is True

    # Оборачивание в infer()'овский RuntimeError(f'... {exc!r}') не должно
    # ломать детекцию — repr() сохраняет исходный текст внутри.
    wrapped = RuntimeError(f'HailoRT run() failed: {exc!r}')
    assert loader_mod._is_recoverable_stream_abort(wrapped) is True

    # Другие ошибки — НЕ abort (не должны триггерить recovery-путь).
    assert loader_mod._is_recoverable_stream_abort(
        RuntimeError('HAILO_OUT_OF_PHYSICAL_DEVICES(74)')
    ) is False
    assert loader_mod._is_recoverable_stream_abort(FileNotFoundError('x')) is False
    assert loader_mod._is_recoverable_stream_abort(RuntimeError('boom')) is False


def test_next_reinit_backoff_sec_grows_and_caps():
    """Backoff: 0 → INITIAL, дальше геометрический рост до MAX (issue #2625 п.2)."""
    b0 = loader_mod._next_reinit_backoff_sec(0.0)
    assert b0 == loader_mod.REINIT_BACKOFF_INITIAL_SEC

    b1 = loader_mod._next_reinit_backoff_sec(b0)
    assert b1 == pytest.approx(
        loader_mod.REINIT_BACKOFF_INITIAL_SEC * loader_mod.REINIT_BACKOFF_MULTIPLIER
    )

    # Рост зажат потолком.
    huge = loader_mod._next_reinit_backoff_sec(loader_mod.REINIT_BACKOFF_MAX_SEC * 10)
    assert huge == loader_mod.REINIT_BACKOFF_MAX_SEC


def test_real_loader_recovers_after_stream_abort(tmp_path, monkeypatch):
    """issue #2625 п.1/4: abort на N-ном `run()` → следующий кадр
    переинициализируется, а НЕ уходит в вечный `_init_failed`-degraded.
    """
    infer_model = _AbortOnceInferModel(abort_after_n_runs=2)
    vdevice = _FakeVDevice(infer_model)
    real = _real_loader_for_abort_test(tmp_path, monkeypatch, vdevice)
    fake_img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Кадр 1: run() успешен (total_runs=1). Инициализация — раз.
    events1 = real.infer(frame_id='oak-d', image=fake_img)
    assert isinstance(events1, list)
    assert infer_model.configure_count == 1
    assert real._init_failed is None
    assert real._recovering_from_abort is False

    # Кадр 2: run() бросает abort (total_runs=2) → RuntimeError наружу
    # (кадр пропущен, как и раньше — узел это уже умеет логировать/считать).
    with pytest.raises(RuntimeError, match=r'HailoRT run\(\) failed'):
        real.infer(frame_id='oak-d', image=fake_img)

    # ГЛАВНАЯ ПРОВЕРКА (issue #2625 п.1): abort — НЕ фатальный init failure.
    assert real._init_failed is None, (
        'abort не должен кешироваться в _init_failed — иначе '
        '_ensure_initialized() re-raise\'ит его навсегда, и пайплайн '
        'никогда не переинициализируется (issue #2625, "лечится только '
        'docker restart").'
    )
    assert real._recovering_from_abort is True
    assert real._configured is None
    assert real._bindings is None
    assert real._infer_model is None

    # Кадр 3: следующий вызов ДОЛЖЕН переинициализироваться (configure() #2)
    # и успешно инференснуть, а не повторно упасть в degraded.
    events3 = real.infer(frame_id='oak-d', image=fake_img)
    assert isinstance(events3, list)
    assert infer_model.configure_count == 2, (
        'lazy init не переинициализировался на следующем кадре после abort '
        '(issue #2625 п.1 — "дать ленивой инициализации подняться заново").'
    )
    assert real._recovering_from_abort is False
    assert real._init_failed is None


def test_real_loader_logs_explicit_recovery_after_abort(tmp_path, monkeypatch, caplog):
    """issue #2625 п.3: явный лог восстановления, не тихое состояние."""
    infer_model = _AbortOnceInferModel(abort_after_n_runs=1)
    vdevice = _FakeVDevice(infer_model)
    real = _real_loader_for_abort_test(tmp_path, monkeypatch, vdevice)
    fake_img = np.zeros((480, 640, 3), dtype=np.uint8)

    with caplog.at_level(
        logging.WARNING, logger='rob_box_perception.vision_hailo_loader'
    ):
        with pytest.raises(RuntimeError, match=r'HailoRT run\(\) failed'):
            real.infer(frame_id='oak-d', image=fake_img)
        real.infer(frame_id='oak-d', image=fake_img)  # reinit + успешный run()

    messages = [rec.message for rec in caplog.records]
    assert any('abort' in m.lower() for m in messages), (
        f'нет лога обнаружения abort: {messages!r}'
    )
    assert any('переподнят после' in m for m in messages), (
        f'нет явного лога восстановления пайплайна: {messages!r} '
        '(issue #2625 п.3 — иначе снова тихое состояние, "тихо-хорошее").'
    )


def test_real_loader_reinit_backoff_gates_retries_and_grows(tmp_path, monkeypatch):
    """issue #2625 п.2: reinit после abort не молотит каждый кадр — backoff.

    Сценарий: abort → 1-я reinit-попытка немедленная и падает (backoff
    растёт до INITIAL) → повторный кадр ДО истечения backoff НЕ должен
    трогать железо снова → после истечения backoff — 2-я попытка (снова
    падает, backoff растёт до INITIAL*MULTIPLIER) → после истечения —
    3-я попытка успешна → recovery.
    """
    infer_model = _AbortOnceInferModel(abort_after_n_runs=2)
    # Call #1 = исходная успешная инициализация (не в fail-списке).
    # Call #2, #3 = первые две reinit-попытки после abort — падают.
    # Call #4 = третья reinit-попытка — успешна.
    vdevice = _FlakyVDeviceForReinit(infer_model, fail_on_calls={2, 3})
    real = _real_loader_for_abort_test(tmp_path, monkeypatch, vdevice)
    fake_img = np.zeros((480, 640, 3), dtype=np.uint8)

    fake_now = [1_000.0]
    monkeypatch.setattr(loader_mod.time, 'monotonic', lambda: fake_now[0])

    # Кадр 1: успешная инициализация (create_infer_model call #1).
    real.infer(frame_id='oak-d', image=fake_img)
    assert vdevice.create_calls == 1

    # Кадр 2: run() абортит → reset, recovering=True, backoff=0 (немедленно).
    with pytest.raises(RuntimeError, match=r'HailoRT run\(\) failed'):
        real.infer(frame_id='oak-d', image=fake_img)
    assert real._recovering_from_abort is True
    assert real._reinit_backoff_sec == 0.0

    # Кадр 3: backoff=0 → немедленная попытка reinit (call #2) → падает
    # (fake create_infer_model failure), backoff становится INITIAL.
    with pytest.raises(RuntimeError, match='fake create_infer_model failure'):
        real.infer(frame_id='oak-d', image=fake_img)
    assert vdevice.create_calls == 2
    assert real._reinit_backoff_sec == pytest.approx(
        loader_mod.REINIT_BACKOFF_INITIAL_SEC
    )
    not_before_1 = real._reinit_not_before

    # Кадр 4, часы НЕ продвинуты: backoff активен → железо НЕ трогаем снова.
    with pytest.raises(RuntimeError, match='backoff'):
        real.infer(frame_id='oak-d', image=fake_img)
    assert vdevice.create_calls == 2, (
        'reinit не должен молотить каждый кадр внутри backoff-окна '
        '(issue #2625 п.2 — "переинициализация не молотит каждые ~0.5с").'
    )

    # Продвигаем часы за not_before_1 → 2-я reinit-попытка (call #3),
    # снова падает, backoff растёт геометрически.
    fake_now[0] = not_before_1 + 0.01
    with pytest.raises(RuntimeError, match='fake create_infer_model failure'):
        real.infer(frame_id='oak-d', image=fake_img)
    assert vdevice.create_calls == 3
    assert real._reinit_backoff_sec == pytest.approx(
        loader_mod.REINIT_BACKOFF_INITIAL_SEC * loader_mod.REINIT_BACKOFF_MULTIPLIER
    )
    not_before_2 = real._reinit_not_before
    assert not_before_2 > not_before_1

    # Продвигаем часы за not_before_2 → 3-я reinit-попытка (call #4) успешна.
    fake_now[0] = not_before_2 + 0.01
    events = real.infer(frame_id='oak-d', image=fake_img)
    assert isinstance(events, list)
    assert vdevice.create_calls == 4
    assert real._recovering_from_abort is False
    assert real._reinit_backoff_sec == 0.0
    assert real._init_failed is None


def test_real_loader_init_failure_still_caches_permanently(tmp_path, monkeypatch):
    """Контроль: ПЕРВАЯ (never-succeeded) инициализация НЕ путается с
    abort-recovery — остаётся permanent `_init_failed` (issue #2625:
    "разведи два случая честно"). Регресс-дублёр
    `test_real_loader_init_failure_is_not_silent_log_spam`, но с фокусом
    именно на неизменности старого контракта после этого фикса.
    """
    hef = tmp_path / 'yolov8n.hef'
    hef.write_bytes(b'\x00')

    class _AlwaysBrokenVDevice:
        def create_infer_model(self, _hef):
            raise RuntimeError('HEF parse failed (fake, not an abort)')

    import types
    fake_mod = types.ModuleType('hailo_platform')
    fake_mod.VDevice = lambda *a, **kw: _AlwaysBrokenVDevice()
    monkeypatch.setitem(sys.modules, 'hailo_platform', fake_mod)

    real = loader_mod.RealHEFLoader(hef_path=str(hef))
    assert real.is_available() is False
    assert real._init_failed is not None
    assert real._recovering_from_abort is False

    # Повторный вызов — re-raise закешированной ошибки, БЕЗ повторной
    # попытки создать infer_model (в отличие от abort-recovery пути).
    with pytest.raises(RuntimeError, match='HEF parse failed'):
        real._ensure_initialized()
