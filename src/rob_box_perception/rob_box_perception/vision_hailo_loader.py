"""HEF loader interface для vision_hailo_node (ADR-0089).

Зачем отдельный модуль: класс `HEFLoader` + его реализации (Stub, Real)
НЕ должны зависеть от rclpy — иначе unit-тесты без colcon-build
падают с ModuleNotFoundError. Этот модуль — чистый Python, без ROS,
импортируется и тестируется в любом окружении.

Сам ROS-узел (`vision_hailo_node.py`) импортирует `make_loader`
отсюда. На Pi с rclpy + HailoRT — работает real loader; в CI — stub.

Phase 1.5 (issue #2398): `RealHEFLoader.infer()` реализует полный
real-inference pipeline поверх HailoRT modern async API:

    JPEG/PNG bytes → cv2 decode → letterbox 640×640 → NHWC uint8
        → HailoRT VDevice.run() (YOLOv8n HEF)
        → (1, 80, 8400) tensor → _post_process_detections (pure Python)
        → List[VisionEvent-dict]

Failure policy (ADR-0018 capability-honest): любой сбой HailoRT
(init / run / post-process) propagates как exception. Узел
логирует и пропускает кадр — НЕ silent fallback на stub.

Binding install strategy — см. ADR-0099:
  - Phase 1 (current default): `hailo_platform` опциональный, lazy import.
    На CI без Hailo apt-repo / Developer Zone wheel — ImportError → путь
    stub. Тесты (test_real_loader_init_failure_is_available_false в
    test_vision_hailo_phase15.py:292-296) зафиксированы на этот контракт.
  - Phase 1.5 (отдельная карточка): install через Hailo Developer Zone
    .deb (HailoRT) + .whl (Python binding). Тогда же делается fatal
    `import hailo_platform` в Dockerfile и acceptance §9 расширяется
    проверкой `is_available() → True`.
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple


# Default HailoRT VDevice id. На Vision Pi с одним AI HAT+ всегда 0.
DEFAULT_VDEVICE_ID = 0


# YOLOv8n входной размер (hailo_model_zoo даёт HEF именно под 640×640).
# Менять только если берём другой HEF (yolov8s → 640×640, yolov8m → 640×640).
DEFAULT_INPUT_W = 640
DEFAULT_INPUT_H = 640

# YOLOv8n output: 80 классов COCO, 8400 якорей (80×80 + 40×40 + 20×20 = 8400).
DEFAULT_NUM_CLASSES = 80
DEFAULT_NUM_ANCHORS = 8400

# Padding value (gray) для letterbox. YOLOv8n training использовал 114.
LETTERBOX_PAD_VALUE = 114

# COCO class names — нужны для маппинга class_id → class_name.
# Полный список из hailo_model_zoo / COCO dataset.
COCO_CLASS_NAMES: Tuple[str, ...] = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
    'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
    'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
    'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
    'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
    'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
    'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush',
)


# ============================================================================
# Интерфейс и реализации
# ============================================================================

class HEFLoader:
    """Абстракция над HailoRT VDevice + HEF.

    Узел `vision_hailo_node` общается с HEF через этот интерфейс, чтобы
    тесты могли подменять его `MockHEFLoader` без условных `if CI`.
    """

    def is_available(self) -> bool:
        """True если HailoRT инициализирован и есть хотя бы одно устройство."""
        raise NotImplementedError

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        """Запуск инференса на одном кадре.

        Args:
            frame_id: ROS header.frame_id источника (например "oak_rgb_frame").
            image: numpy.ndarray с картинкой (формат зависит от модели).
                Для stub передаётся None.

        Returns:
            List[dict]: список детекций в формате VisionEvent-полей
            (без stamp — его проставляет ROS-нода).
        """
        raise NotImplementedError


class StubHEFLoader(HEFLoader):
    """Детерминированный loader для CI / pre-HEF smoke tests.

    Публикует ровно один event каждые `period_sec` секунд:
    "person at 1.0m, confidence 0.92". Это даёт downstream'у
    стабильный сигнал для проверки, что topic жив и формат
    корректный, без зависимости от железа.
    """

    def __init__(self, period_sec: float = 2.0) -> None:
        self._period_sec = period_sec
        self._last_emit = 0.0

    def is_available(self) -> bool:
        return True

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        now = time.monotonic()
        if now - self._last_emit < self._period_sec:
            return []
        self._last_emit = now
        return [{
            'source_camera': frame_id or 'stub',
            'event_type': 'person',
            'class_name': 'person',
            'class_id': 0,
            'confidence': 0.92,
            'bbox_cx': 0.5,
            'bbox_cy': 0.5,
            'bbox_w': 0.4,
            'bbox_h': 0.6,
            'distance_m': 1.0,
            'embedding_id': '',
            'display_name': '',
            'attributes_json': '',
        }]


class RealHEFLoader(HEFLoader):
    """Реальный HEF loader через `hailort` Python API.

    Импорт `hailo_platform` ленивый — пакет доступен только на Vision Pi
    с установленным HailoRT. Это сохраняет CI-сборку без зависимости от
    ARM64-specific deb-пакета.

    Phase 1.5 (issue #2398): полный inference pipeline.

    Attributes:
        _hef_path: путь к .hef файлу.
        _vdevice_id: ID VDevice (на Pi5 с одним HAT+ всегда 0).
        _input_w / _input_h: ожидаемый HEF input shape (640×640 для YOLOv8n).
        _confidence_threshold: фильтр на уровне loader'а (по умолчанию 0.5).
        _nms_iou_threshold: IoU threshold для NMS (по умолчанию 0.45).
        _vdevice: HailoRT VDevice (lazy).
        _infer_model: HailoRT InferModel (lazy).
        _configured: HailoRT configured model context (lazy).
        _bindings_input / _bindings_output: numpy-массивы для I/O (lazy).
    """

    def __init__(
        self,
        hef_path: str,
        vdevice_id: int = DEFAULT_VDEVICE_ID,
        input_w: int = DEFAULT_INPUT_W,
        input_h: int = DEFAULT_INPUT_H,
        confidence_threshold: float = 0.5,
        nms_iou_threshold: float = 0.45,
    ) -> None:
        self._hef_path = hef_path
        self._vdevice_id = vdevice_id
        self._input_w = input_w
        self._input_h = input_h
        self._confidence_threshold = confidence_threshold
        self._nms_iou_threshold = nms_iou_threshold
        self._vdevice = None
        self._infer_model = None
        self._configured = None
        self._bindings_input = None
        self._bindings_output = None

    # ----------------------------------------------------------------
    # Lazy initialization
    # ----------------------------------------------------------------

    def _ensure_initialized(self) -> None:
        """Ленивая инициализация HailoRT VDevice + HEF.

        Raises:
            ImportError: hailort не установлен (не на том хосте).
            FileNotFoundError: HEF файл не найден.
            RuntimeError: VDevice не доступен / HEF не компилируется.
        """
        if self._configured is not None:
            return
        try:
            from hailo_platform import (  # type: ignore[import-not-found]
                VDevice,
            )
        except ImportError as exc:
            raise ImportError(
                'hailo_platform не установлен. Установите HailoRT на Vision Pi '
                'перед включением hailo_enabled=True (см. docker/vision/vision-hailo/Dockerfile).'
            ) from exc
        if not os.path.isfile(self._hef_path):
            raise FileNotFoundError(
                f'HEF не найден: {self._hef_path}. Скачайте из hailo_model_zoo '
                f'(https://github.com/hailo-ai/hailo_model_zoo).'
            )

        # Modern async API (HailoRT 4.18+). Используем ROUND_ROBIN scheduling
        # для однородной latency (single-stream инференс).
        try:
            from hailo_platform import (  # type: ignore[import-not-found]
                HailoSchedulingAlgorithm,
            )
            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            params.device_id = str(self._vdevice_id)
            self._vdevice = VDevice(params=params)
        except (ImportError, TypeError):
            # Fallback для версий HailoRT < 4.18 (без scheduling_algorithm).
            self._vdevice = VDevice()

        self._infer_model = self._vdevice.create_infer_model(self._hef_path)
        self._infer_model.set_batch_size(1)

        # Установим формат входа/выхода. YOLOv8n HEF ожидает UINT8 NHWC.
        # Методы set_format_type могут отсутствовать в разных версиях HailoRT.
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]
            try:
                self._infer_model.input().set_format_type(FormatType.UINT8)
            except AttributeError:
                # HEF может требовать другой формат — оставляем как есть.
                pass
            # Выход обычно FLOAT32 — class-agnostic decode уже в HEF.
        except ImportError:
            pass

        self._configured = self._infer_model.configure()

        # Pre-allocate I/O buffers (нужно для repeated run()).
        self._bindings_input = {}
        self._bindings_output = {}
        for in_name in self._configured.input_vstreams.keys():
            self._bindings_input[in_name] = self._configured.get_input_binding(
                name=in_name,
            )
        for out_name in self._configured.output_vstreams.keys():
            self._bindings_output[out_name] = self._configured.get_output_binding(
                name=out_name,
            )

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------

    def is_available(self) -> bool:
        try:
            self._ensure_initialized()
            return True
        except (ImportError, FileNotFoundError, RuntimeError, OSError):
            return False

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        """Запуск real inference на одном кадре.

        Args:
            frame_id: ROS header.frame_id источника.
            image: numpy.ndarray (H×W×3, BGR или RGB). Stub вызывает
                с image=None, real — с numpy.

        Returns:
            List[dict] в формате VisionEvent-полей.

        Raises:
            RuntimeError: HEF loader не инициализирован или run() упал.
            ImportError: hailort не установлен (propagated).

        Note:
            `image=None` — это сигнал «нет реального кадра», возвращаем
            [] БЕЗ инициализации HailoRT. Это позволяет heartbeat'ить
            в ноде даже когда upstream кадров ещё нет (см. Node._tick).
        """
        if image is None:
            return []
        self._ensure_initialized()
        # 1. Pre-process: BGR/RGB → NHWC uint8, letterboxed 640×640.
        input_tensor = self._preprocess(image)

        # Type narrowing после _ensure_initialized() — dict'ы и configured
        # гарантированно не None.
        assert self._bindings_input is not None
        assert self._bindings_output is not None
        assert self._configured is not None

        # 2. HailoRT run(). Передаём подготовленный tensor как binding.
        # Single-input/output binding — берём первый (у YOLOv8n один вход).
        first_in_name = next(iter(self._bindings_input))
        first_in_binding = self._bindings_input[first_in_name]
        try:
            first_in_binding.set_buffer(input_tensor)
            output_list = list(self._bindings_output.values())
            self._configured.run(
                [first_in_binding],
                output_list,
            )
            # Output — list[np.ndarray] в порядке output_vstreams.
            raw_output = output_list[0]
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f'HailoRT run() failed: {exc!r}',
            ) from exc

        # 3. Post-process: tensor → List[VisionEvent-dict].
        return _post_process_detections(
            raw_output=raw_output,
            source_camera=frame_id or 'unknown',
            input_w=self._input_w,
            input_h=self._input_h,
            confidence_threshold=self._confidence_threshold,
            nms_iou_threshold=self._nms_iou_threshold,
        )

    # ----------------------------------------------------------------
    # Pre-processing
    # ----------------------------------------------------------------

    def _preprocess(self, image: Any) -> Any:
        """BGR/RGB → NHWC uint8, letterboxed под (_input_w, _input_h).

        Letterbox (а не plain resize) сохраняет aspect ratio — иначе
        bbox'ы на выходе модели искажены.
        """
        # Ленивый import cv2 — не требуется для CI/тестов.
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                'Real inference requires opencv-python + numpy. '
                'Установите: pip install opencv-python numpy',
            ) from exc

        h, w = image.shape[:2]
        scale = min(self._input_w / w, self._input_h / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))

        # Resize.
        if (w, h) != (new_w, new_h):
            resized = cv2.resize(image, (new_w, new_h))
        else:
            resized = image

        # Pad до 640×640 (gray fill).
        pad_w = self._input_w - new_w
        pad_h = self._input_h - new_h
        pad_left = pad_w // 2
        pad_top = pad_h // 2
        padded = cv2.copyMakeBorder(
            resized,
            pad_top,
            pad_h - pad_top,
            pad_left,
            pad_w - pad_left,
            cv2.BORDER_CONSTANT,
            value=(LETTERBOX_PAD_VALUE, LETTERBOX_PAD_VALUE, LETTERBOX_PAD_VALUE),
        )

        # NHWC uint8. HailoRT YOLOv8n HEF ожидает именно этот формат.
        if padded.dtype != np.uint8:
            padded = padded.astype(np.uint8)
        if len(padded.shape) == 3:
            tensor = np.expand_dims(padded, axis=0)  # NHWC
        else:
            tensor = padded
        return np.ascontiguousarray(tensor)


# ============================================================================
# Post-processing (pure-function, тестируется без hailo_platform)
# ============================================================================

def _post_process_detections(
    raw_output: Any,
    source_camera: str,
    input_w: int,
    input_h: int,
    confidence_threshold: float,
    nms_iou_threshold: float,
) -> List[Dict[str, Any]]:
    """YOLOv8n output tensor → List[VisionEvent-dict].

    YOLOv8n Hailo HEF (из hailo_model_zoo) даёт single output tensor формы
    `(1, 84, 8400)` где:
        - первые 4 канала: `cx, cy, w, h` (pixel coords в letterbox space)
        - каналы 4..84: per-class scores (sigmoid'ed или linear)

    Args:
        raw_output: numpy.ndarray формы (1, 84, 8400) или (84, 8400).
        source_camera: подставляется в VisionEvent.source_camera.
        input_w / input_h: letterbox space (= 640 для YOLOv8n).
        confidence_threshold: фильтр confidence.
        nms_iou_threshold: IoU threshold для NMS.

    Returns:
        List[dict] в формате VisionEvent-полей. Bbox'ы в normalized
        [0,1] coords (cx, cy, w, h).
    """
    try:
        import numpy as np  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            'post-processing requires numpy',
        ) from exc

    if raw_output is None:
        return []

    arr = np.asarray(raw_output)
    if arr.ndim == 3:
        arr = arr[0]  # (84, 8400)
    # Если модель выдаёт уже (1, 84, 8400) с sigmoid'нутыми scores:
    # обычно layout = (1, 80+c, anchors) либо (anchors, 80+c). Поддержим оба.

    num_classes = DEFAULT_NUM_CLASSES
    expected_total = 4 + num_classes  # 84 для YOLOv8n

    # Case A: (84, 8400) — class-agnostic decode в HEF.
    if arr.shape == (expected_total, DEFAULT_NUM_ANCHORS):
        boxes = arr[:4, :]              # (4, 8400)
        scores = arr[4:, :]             # (80, 8400)
    # Case B: (8400, 84) — transpose.
    elif arr.shape == (DEFAULT_NUM_ANCHORS, expected_total):
        arr = arr.T
        boxes = arr[:4, :]
        scores = arr[4:, :]
    else:
        # Unknown layout — return empty (capability-honest).
        return []

    # Лучший class per anchor.
    class_ids = np.argmax(scores, axis=0)
    confidences = scores[class_ids, np.arange(scores.shape[1])]

    # Confidence filter.
    keep_mask = confidences >= confidence_threshold
    if not np.any(keep_mask):
        return []
    boxes = boxes[:, keep_mask]
    confidences = confidences[keep_mask]
    class_ids = class_ids[keep_mask]

    # cxcywh → xyxy (NMS требует xyxy).
    xyxy = np.empty_like(boxes)
    xyxy[0] = boxes[0] - boxes[2] / 2  # x1
    xyxy[1] = boxes[1] - boxes[3] / 2  # y1
    xyxy[2] = boxes[0] + boxes[2] / 2  # x2
    xyxy[3] = boxes[1] + boxes[3] / 2  # y2

    # NMS (per-class).
    try:
        keep_idx = _nms_per_class(
            xyxy.T,  # (N, 4)
            confidences,
            class_ids,
            iou_threshold=nms_iou_threshold,
        )
    except (ValueError, IndexError, TypeError):
        # Если NMS упал на некорректных данных (NaN, shape mismatch) —
        # capability-honest: return empty, не падать.
        return []
    if not keep_idx:
        return []

    xyxy = xyxy[:, keep_idx]
    boxes = boxes[:, keep_idx]  # cxcywh (для финального вывода)
    confidences = confidences[keep_idx]
    class_ids = class_ids[keep_idx]

    # Нормализация bbox'ов в [0, 1] для VisionEvent (cxcywh / input_w_h).
    cx = boxes[0, :] / input_w
    cy = boxes[1, :] / input_h
    bw = boxes[2, :] / input_w
    bh = boxes[3, :] / input_h

    events: List[Dict[str, Any]] = []
    for i in range(boxes.shape[1]):
        cls_id = int(class_ids[i])
        cls_name = (
            COCO_CLASS_NAMES[cls_id]
            if 0 <= cls_id < len(COCO_CLASS_NAMES) else ''
        )
        # event_type — phase 1 использует "person"/"object"/"scene" дискриминатор.
        if cls_name == 'person':
            ev_type = 'person'
        elif cls_name:
            ev_type = 'object'
        else:
            ev_type = 'scene'

        events.append({
            'source_camera': source_camera,
            'event_type': ev_type,
            'class_name': cls_name,
            'class_id': cls_id,
            'confidence': float(confidences[i]),
            'bbox_cx': float(cx[i]),
            'bbox_cy': float(cy[i]),
            'bbox_w': float(bw[i]),
            'bbox_h': float(bh[i]),
            # distance_m — Phase 2 fusion с OAK-D depth; сейчас -1.
            'distance_m': -1.0,
            'embedding_id': '',
            'display_name': '',
            'attributes_json': '',
        })
    return events


def _nms_per_class(
    boxes: Any,
    scores: Any,
    class_ids: Any,
    iou_threshold: float,
) -> List[int]:
    """Простой per-class NMS (pure numpy, без cython-cv2).

    Args:
        boxes: (N, 4) в формате [x1, y1, x2, y2].
        scores: (N,)
        class_ids: (N,)
        iou_threshold: drop если IoU >= threshold.

    Returns:
        List[int] индексов, которые нужно оставить.
    """
    import numpy as np  # type: ignore[import-not-found]

    if len(boxes) == 0:
        return []
    order = np.argsort(-scores)
    keep: List[int] = []
    suppressed = np.zeros(len(boxes), dtype=bool)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)

    while len(order) > 0:
        i = int(order[0])
        if suppressed[i]:
            order = order[1:]
            continue
        keep.append(i)
        if len(order) == 1:
            break
        rest = order[1:]
        # Same class only.
        same_class = class_ids[rest] == class_ids[i]
        if not np.any(same_class):
            order = rest
            continue
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / union, 0.0)
        # Per-element boolean mask для `rest` (ДЛИНА = len(rest)).
        # iou >= threshold AND same_class → suppress.
        suppress_mask = (iou >= iou_threshold) & same_class
        suppressed[rest[suppress_mask]] = True
        # Сохраняем то, что НЕ подавлено.
        order = rest[~suppress_mask]
    return keep


# ============================================================================
# Factory
# ============================================================================

def make_loader(
    hailo_enabled: bool,
    hef_path: Optional[str],
    stub_period_sec: float,
) -> HEFLoader:
    """Вернуть правильный loader по launch-параметрам.

    Args:
        hailo_enabled: launch-параметр, разрешает использовать HailoRT.
        hef_path: путь к .hef файлу (None или пустая строка = stub).
        stub_period_sec: период публикации stub-событий.
    """
    if hailo_enabled and hef_path:
        return RealHEFLoader(hef_path=hef_path)
    return StubHEFLoader(period_sec=stub_period_sec)


# ============================================================================
# Filter helper (без rclpy, чистый Python — тестируется отдельно)
# ============================================================================

def filter_by_confidence(
    events: Iterable[Dict[str, Any]],
    threshold: float,
) -> List[Dict[str, Any]]:
    """Фильтр VisionEvent-диктов по confidence_threshold.

    Контракт: `confidence >= threshold` -> keep, иначе drop.
    threshold ровно 0.5 — keep (>=).
    """
    return [
        ev for ev in events
        if float(ev.get('confidence', 0.0)) >= threshold
    ]


# ============================================================================
# VisionEvent-dict normalizer (используется и в Node, и в тестах)
# ============================================================================

VISION_EVENT_FIELDS = (
    'source_camera', 'event_type', 'class_name', 'class_id',
    'confidence', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h',
    'distance_m', 'embedding_id', 'display_name', 'attributes_json',
)


def normalize_event_dict(event_dict: Dict[str, Any]) -> Dict[str, Any]:
    """Вернуть dict с типизированными полями VisionEvent.

    Defaults:
        source_camera='', event_type='scene', class_name='', class_id=-1,
        confidence=0.0, bbox_*=-1.0, distance_m=-1.0,
        embedding_id='', display_name='', attributes_json=''.

    Используется в vision_hailo_node._publish_event (там же
    проставляется stamp), в тестах build_event round-trip, и
    в context_aggregator при маппинге на PerceptionEvent JSON.
    """
    return {
        'source_camera': str(event_dict.get('source_camera', '')),
        'event_type': str(event_dict.get('event_type', 'scene')),
        'class_name': str(event_dict.get('class_name', '')),
        'class_id': int(event_dict.get('class_id', -1)),
        'confidence': float(event_dict.get('confidence', 0.0)),
        'bbox_cx': float(event_dict.get('bbox_cx', -1.0)),
        'bbox_cy': float(event_dict.get('bbox_cy', -1.0)),
        'bbox_w': float(event_dict.get('bbox_w', -1.0)),
        'bbox_h': float(event_dict.get('bbox_h', -1.0)),
        'distance_m': float(event_dict.get('distance_m', -1.0)),
        'embedding_id': str(event_dict.get('embedding_id', '')),
        'display_name': str(event_dict.get('display_name', '')),
        'attributes_json': str(event_dict.get('attributes_json', '')),
    }
