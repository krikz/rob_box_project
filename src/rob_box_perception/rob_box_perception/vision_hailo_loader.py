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
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

from rob_box_perception.hailo_device import open_device


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

#: Маркер выдуманного события. ADR-0089 §2.2 предписывает ровно это значение
#: как ключ, по которому stub-события отсекаются до попадания в LLM-контекст.
#: Значение НЕ входит в набор "person"|"face"|"object"|"scene" намеренно:
#: заглушка — это не детекция, и downstream не должен спутать её ни с чем.
STUB_EVENT_TYPE: str = 'stub'

#: source_camera выдуманного события. Камеры у него нет по определению.
STUB_SOURCE_CAMERA: str = 'stub'


@dataclass(frozen=True)
class LetterboxInfo:
    """Метаданные letterbox, нужные для обратной проекции bbox (ADR-0104).

    Соглашение о паддинге (issue #2584, то же самое что и в
    ``gaze.py::Frame.as_letterbox``): **симметричный** letterbox — паддинг
    делится поровну между противоположными сторонами (``pad // 2`` на одну
    сторону, остаток — на другую), а не top-left. ``_preprocess`` ниже и
    ``Frame.as_letterbox`` обязаны считать pad_left/pad_top одной и той же
    формулой — иначе ``unproject`` сдвинет bbox на величину рассогласования
    (регрессия к дефекту issue #2531 acceptance #9).

    Attributes:
        scale: коэффициент resize ДО паддинга (orig_w * scale = new_w).
        pad_left: пикселей паддинга слева в letterbox-тензоре.
        pad_top: пикселей паддинга сверху в letterbox-тензоре.
        orig_w / orig_h: размер исходного кадра до letterbox.
        letterbox_w / letterbox_h: размер letterbox-тензора (= input_w/h).
    """

    scale: float
    pad_left: int
    pad_top: int
    orig_w: int
    orig_h: int
    letterbox_w: int
    letterbox_h: int

    def unproject(self, cx: float, cy: float, bw: float, bh: float) -> Tuple[float, float, float, float]:
        """Снять letterbox с bbox: вернуть (cx, cy, bw, bh) в исходном кадре.

        YOLOv8n выдаёт bbox в letterbox-space (640×640). Чтобы получить
        bbox в исходном кадре:
            1. (cx - pad_left, cy - pad_top) → координаты в resized-frame.
            2. / scale → координаты в исходном кадре.
        Это фикс issue #2531 acceptance #9 — без unproject bbox уехал
        на размер паддинга и систематически сжат относительно объекта.
        """
        if self.scale <= 0.0:
            return (cx, cy, bw, bh)
        u_cx = (cx - self.pad_left) / self.scale
        u_cy = (cy - self.pad_top) / self.scale
        u_bw = bw / self.scale
        u_bh = bh / self.scale
        return (u_cx, u_cy, u_bw, u_bh)


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
        # ВАЖНО (ADR-0089 §2.2): событие выдуманное, и это обязано быть
        # видно в данных. Раньше здесь стояло event_type='person', а
        # source_camera падал на 'stub' только при пустом frame_id — но
        # узел зовёт infer с frame_id='unknown' (vision_hailo_node._tick),
        # так что маркера не оставалось вообще. Любой downstream, решающий
        # «показывать ли это Личности», обязан иметь способ отличить
        # выдумку от детекции — см. STUB_EVENT_TYPE / is_stub_event.
        return [{
            'source_camera': STUB_SOURCE_CAMERA,
            'event_type': STUB_EVENT_TYPE,
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
        # Lazy-initialized state. Заполняется в _ensure_initialized().
        # Тип намеренно Any (не Optional): после init() поле не None,
        # а pyright-typing «Optional[Any]» приводит к спам-warning'ам
        # про member access на каждом self._infer_model.input() — здесь
        # нам важнее runtime-корректность, а не статический type narrowing.
        self._vdevice: Any = None
        self._infer_model: Any = None
        self._configured: Any = None
        self._bindings: Any = None
        # Имя выходного stream, из которого берём результат (по дефолту первый).
        self._output_name: Optional[str] = None
        # Флаг фатальной ошибки init (для _tick vs init различения).
        self._init_failed: Optional[BaseException] = None

    # ----------------------------------------------------------------
    # Lazy initialization
    # ----------------------------------------------------------------

    def _ensure_initialized(self) -> None:
        """Ленивая инициализация HailoRT VDevice + HEF (modern async API).

        Использует **новое** поколение HailoRT Python API (>=4.18):
            `VDevice.create_infer_model(hef) → InferModel.configure() →
            ConfiguredInferModel.create_bindings(...) → run([bindings], timeout)`.
        Старый API (input_vstreams / get_input_binding / get_output_binding)
        в этом поколении **отсутствует** — см. upstream
        `hailo-ai/hailort/hailort/libhailort/bindings/python/platform/
        hailo_platform/pyhailort/pyhailort.py:2948-3013` (master).

        Raises:
            ImportError: hailort не установлен (не на том хосте).
            FileNotFoundError: HEF файл не найден.
            RuntimeError: VDevice не доступен / HEF не компилируется /
                любой сбой инициализации API.

        Note:
            Метод идемпотентен: после первой успешной инициализации
            последующие вызовы — no-op. При ошибке инициализации
            `_init_failed` запоминается, повторные вызовы re-raise
            ту же ошибку (без reinit-цикла на каждый кадр).
        """
        if self._configured is not None:
            return
        if self._init_failed is not None:
            raise self._init_failed
        try:
            self._init_locked()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            # Запоминаем для fail-fast + DI.
            self._init_failed = exc
            raise

    def _init_locked(self) -> None:
        """Собственно инициализация (без кеширования ошибки)."""
        if not os.path.isfile(self._hef_path):
            raise FileNotFoundError(
                f'HEF не найден: {self._hef_path}. Скачайте из hailo_model_zoo '
                f'(https://github.com/hailo-ai/hailo_model_zoo).'
            )

        # Устройство берём через шов «Ускоритель» (hailo_device): владелец
        # железа — hailort.service, когда его сокет достижим. Прямой
        # VDevice здесь забирал Hailo единолично, и лицевая нода рядом
        # падала с HAILO_OUT_OF_PHYSICAL_DEVICES(74) — issue #2599.
        self._device = open_device(self._vdevice_id)
        self._vdevice = self._device.vdevice

        # Создаём InferModel и конфигурируем (новое поколение API).
        self._infer_model = self._vdevice.create_infer_model(self._hef_path)
        self._infer_model.set_batch_size(1)

        # YOLOv8n HEF ожидает UINT8 NHWC на входе. Метод может отсутствовать
        # в других моделях/версиях — игнорируем AttributeError.
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]
            try:
                self._infer_model.input().set_format_type(FormatType.UINT8)
            except AttributeError:
                pass
        except ImportError:
            pass

        self._configured = self._infer_model.configure()

        # ----------------------------------------------------------------
        # Pre-allocate I/O buffers + create bindings (новое поколение API).
        #
        # В современном Python-API `create_bindings(input_buffers, output_buffers)`
        # принимает dict'ы numpy-массивов; сами bindings.input(name) и
        # .output(name) дают InferStream с .set_buffer(np) / .get_buffer().
        # ----------------------------------------------------------------
        import numpy as np  # type: ignore[import-not-found]

        input_buffers: Dict[str, Any] = {}
        for in_name in self._infer_model.input_names:
            shape = list(self._infer_model.input(in_name).shape)
            # YOLOv8n: (1, 640, 640, 3) uint8.
            input_buffers[in_name] = np.empty(shape, dtype=np.uint8)

        output_buffers: Dict[str, Any] = {}
        for out_name in self._infer_model.output_names:
            shape = list(self._infer_model.output(out_name).shape)
            # YOLOv8n: (1, 84, 8400) float32 (class scores, post-decoded).
            output_buffers[out_name] = np.empty(shape, dtype=np.float32)

        self._bindings = self._configured.create_bindings(
            input_buffers=input_buffers,
            output_buffers=output_buffers,
        )
        # Активируем async-inference pipeline. ОБЯЗАТЕЛЬНО в новом API
        # (HailoRT 4.18+) при единоличном владении устройством: без
        # activate() первый run() падает HAILO_STREAM_NOT_ACTIVATED(72),
        # и pipeline уходит в abort (#2398).
        #
        # Под сервисом/планировщиком — НАОБОРОТ запрещено, HailoRT отвечает
        # дословно: "ConfiguredNetworkGroup::activate function is not
        # supported when using multi-process service or HailoRT Scheduler"
        # → HAILO_INVALID_OPERATION(6). Решает шов, не лоадер.
        if self._device.must_activate:
            self._configured.activate()
        # Запоминаем имя первого output'а — YOLOv8n имеет один,
        # для multi-output моделей это контрактно первый.
        self._output_name = self._infer_model.output_names[0]

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------

    def is_available(self) -> bool:
        """True если HailoRT инициализирован успешно (lazy).

        Ловит **любой** сбой фазы инициализации (ImportError, OSError,
        RuntimeError, AttributeError от API-drift, и т.п.) — важно
        для pre-flight check из PR #2524: необработанный AttributeError
        ломает launch.
        """
        try:
            self._ensure_initialized()
            return True
        except BaseException:  # noqa: BLE001 (capability-honest init phase)
            return False

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        """Запуск real inference на одном кадре.

        Args:
            frame_id: ROS header.frame_id источника.
            image: numpy.ndarray (H×W×3, **RGB**). Stub вызывает
                с image=None, real — с numpy. RGB-контракт
                (ADR-0104, issue #2531 acceptance #8) гарантируется
                швом «Взгляд» (``gaze.py``); loader НЕ делает
                cvtColor (cv2.imdecode отдавал бы BGR).

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
        # 1. Pre-process: RGB → NHWC uint8, letterboxed 640×640.
        # ADR-0104 (issue #2531 acceptance #8): image приходит RGB
        # (gaze.py делает cvtColor ДО сюда). YOLOv8n HEF обучен на RGB.
        # Возвращает (tensor, LetterboxInfo) — последнее нужно для
        # обратной проекции bbox в _post_process_detections.
        input_tensor, letterbox_info = self._preprocess(image)

        # Type narrowing после _ensure_initialized().
        assert self._bindings is not None
        assert self._configured is not None
        assert self._output_name is not None
        assert self._infer_model is not None

        # 2. HailoRT run(): modern API — bindings через create_bindings,
        # set_buffer на input stream, синхронный run([bindings], timeout_ms).
        in_name = self._infer_model.input_names[0]
        try:
            self._bindings.input(in_name).set_buffer(input_tensor)
            self._configured.run([self._bindings], timeout=1000)
            # Output читается из ТОГО ЖЕ binding (numpy уже заполнен run'ом).
            raw_output = self._bindings.output(self._output_name).get_buffer()
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f'HailoRT run() failed: {exc!r}',
            ) from exc

        # 3. Post-process: tensor → List[VisionEvent-dict].
        # ADR-0104: передаём letterbox_info чтобы bbox'ы
        # денормализовались в координаты исходного кадра (а не letterbox).
        return _post_process_detections(
            raw_output=raw_output,
            source_camera=frame_id or 'unknown',
            input_w=self._input_w,
            input_h=self._input_h,
            confidence_threshold=self._confidence_threshold,
            nms_iou_threshold=self._nms_iou_threshold,
            letterbox_info=letterbox_info,
        )

    # ----------------------------------------------------------------
    # Pre-processing
    # ----------------------------------------------------------------

    def _preprocess(self, image: Any) -> Tuple[Any, LetterboxInfo]:
        """BGR/RGB → NHWC uint8, letterboxed под (_input_w, _input_h).

        Letterbox (а не plain resize) сохраняет aspect ratio — иначе
        bbox'ы на выходе модели искажены.

        ADR-0104 (issue #2531 acceptance #8): image передаётся как
        RGB (YOLOv8n обучен на RGB). cv2.imdecode отдаёт BGR,
        но модуль ``gaze.py`` (новый шов) делает перестановку ДО
        сюда — поэтому мы НЕ делаем cvtColor в _preprocess, а
        доверяем контракту: ``image`` уже в RGB. Это документировано
        в docstring ``infer``.

        Returns:
            (tensor, LetterboxInfo): тензор формы (1, input_h, input_w, 3)
            uint8 + метаданные letterbox для обратной проекции bbox.

        Raises:
            ImportError: opencv-python / numpy недоступны.
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

        info = LetterboxInfo(
            scale=float(scale),
            pad_left=int(pad_left),
            pad_top=int(pad_top),
            orig_w=int(w),
            orig_h=int(h),
            letterbox_w=int(self._input_w),
            letterbox_h=int(self._input_h),
        )
        return np.ascontiguousarray(tensor), info


# ============================================================================
# Post-processing (pure-function, тестируется без hailo_platform)
# ============================================================================

def _is_nms_postprocessed_output(raw_output: Any) -> bool:
    """True если ``raw_output`` — уже NMS'нутый (on-chip) выход HailoRT.

    Issue #2703 (raw-лог 17.09.2026, ``vision_hailo_loader.py:581``):
    ``VisibleDeprecationWarning: Creating an ndarray from ragged nested
    sequences`` на ``np.asarray(raw_output)``. Причина: HEF, скомпилированный
    с on-chip NMS postprocessing (штатная поставка ``yolov8n.hef`` из
    ``hailo_model_zoo``), возвращает ``get_buffer()`` не как единый тензор,
    а как ``List[np.ndarray]`` длины ``num_classes``, где
    ``raw_output[class_id]`` — массив формы ``(n_detections, 5)``:
    ``[y_min, x_min, y_max, x_max, score]`` (нормализовано [0, 1],
    letterbox-space). Число детекций отличается по классам → список
    ragged по построению.

    ПРИМЕЧАНИЕ (честность, ADR-0018): точный layout HailoRT-буфера для
    on-chip NMS не был эмпирически проверен на живом AI HAT+ — в этой
    среде нет доступа к железу/hailo_platform. Формат
    ``[y_min, x_min, y_max, x_max, score]`` — задокументированное лучшее
    понимание по публичному описанию hailo_model_zoo NMS postprocessing
    (тот же TF-style конвеншн, что и стандартный NMS box-order). Issue
    #2703 acceptance #3 ("человек в кадре → confidence >= 0.5") остаётся
    открытым до живой проверки на роботе.

    Эвристика: ``raw_output`` НЕ ndarray, а list/tuple, где каждый
    непустой элемент приводится к 2D-массиву с последней размерностью 5.
    """
    try:
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        return False

    if isinstance(raw_output, np.ndarray):
        return False
    if not isinstance(raw_output, (list, tuple)):
        return False
    if len(raw_output) == 0:
        return False
    for cls_entry in raw_output:
        entry_arr = (
            cls_entry if isinstance(cls_entry, np.ndarray)
            else np.asarray(cls_entry)
        )
        if entry_arr.size == 0:
            continue
        if entry_arr.ndim != 2 or entry_arr.shape[-1] != 5:
            return False
    return True


def _post_process_nms_output(
    raw_output: Any,
    source_camera: str,
    confidence_threshold: float,
    letterbox_info: Optional[LetterboxInfo],
    input_w: int,
    input_h: int,
) -> List[Dict[str, Any]]:
    """Разбор уже-NMS'нутого (on-chip) выхода HailoRT в VisionEvent-dict'ы.

    ``raw_output[class_id]`` — ndarray ``(n, 5)`` с
    ``[y_min, x_min, y_max, x_max, score]``, нормализовано [0, 1] в
    letterbox-space. NMS уже применён HailoRT на чипе — здесь только
    confidence-фильтр и unproject bbox в координаты исходного кадра
    (переиспользует ``LetterboxInfo.unproject``, ту же формулу, что и
    dense-путь в ``_post_process_detections``, — см. docstring
    ``_is_nms_postprocessed_output`` про честность формата).
    """
    import numpy as np  # type: ignore[import-not-found]

    lb_w = letterbox_info.letterbox_w if letterbox_info is not None else input_w
    lb_h = letterbox_info.letterbox_h if letterbox_info is not None else input_h

    events: List[Dict[str, Any]] = []
    for cls_id, cls_entry in enumerate(raw_output):
        entry_arr = (
            cls_entry if isinstance(cls_entry, np.ndarray)
            else np.asarray(cls_entry, dtype=float)
        )
        if entry_arr.size == 0:
            continue
        entry_arr = entry_arr.reshape(-1, 5)

        cls_name = (
            COCO_CLASS_NAMES[cls_id]
            if 0 <= cls_id < len(COCO_CLASS_NAMES) else ''
        )
        if cls_name == 'person':
            ev_type = 'person'
        elif cls_name:
            ev_type = 'object'
        else:
            ev_type = 'scene'

        for row in entry_arr:
            y_min, x_min, y_max, x_max, score = (float(v) for v in row[:5])
            if score < confidence_threshold:
                continue

            # Normalized [0,1] letterbox-space → letterbox-space pixels.
            x_min_px, x_max_px = x_min * lb_w, x_max * lb_w
            y_min_px, y_max_px = y_min * lb_h, y_max * lb_h
            cx_px = (x_min_px + x_max_px) / 2.0
            cy_px = (y_min_px + y_max_px) / 2.0
            bw_px = x_max_px - x_min_px
            bh_px = y_max_px - y_min_px

            if letterbox_info is not None and letterbox_info.scale > 0.0:
                u_cx, u_cy, u_bw, u_bh = letterbox_info.unproject(
                    cx_px, cy_px, bw_px, bh_px
                )
                cx = u_cx / letterbox_info.orig_w
                cy = u_cy / letterbox_info.orig_h
                bw = u_bw / letterbox_info.orig_w
                bh = u_bh / letterbox_info.orig_h
            else:
                cx, cy = cx_px / lb_w, cy_px / lb_h
                bw, bh = bw_px / lb_w, bh_px / lb_h

            events.append({
                'source_camera': source_camera,
                'event_type': ev_type,
                'class_name': cls_name,
                'class_id': cls_id,
                'confidence': score,
                'bbox_cx': float(cx),
                'bbox_cy': float(cy),
                'bbox_w': float(bw),
                'bbox_h': float(bh),
                'distance_m': -1.0,
                'embedding_id': '',
                'display_name': '',
                'attributes_json': '',
            })
    return events


def _post_process_detections(
    raw_output: Any,
    source_camera: str,
    input_w: int,
    input_h: int,
    confidence_threshold: float,
    nms_iou_threshold: float,
    letterbox_info: Optional[LetterboxInfo] = None,
) -> List[Dict[str, Any]]:
    """YOLOv8n output tensor → List[VisionEvent-dict].

    YOLOv8n Hailo HEF (из hailo_model_zoo) даёт single output tensor формы
    `(1, 84, 8400)` где:
        - первые 4 канала: `cx, cy, w, h` (pixel coords в letterbox space)
        - каналы 4..84: per-class scores (sigmoid'ed или linear)

    Args:
        raw_output: numpy.ndarray формы (1, 84, 8400) или (84, 8400) —
            "сырой" (без on-chip NMS) HEF-выход. Либо, если HEF собран
            с on-chip NMS postprocessing (issue #2703): list/tuple длины
            num_classes с per-class ndarray (n, 5) — маршрутизируется в
            ``_post_process_nms_output`` через ``_is_nms_postprocessed_output``.
        source_camera: подставляется в VisionEvent.source_camera.
        input_w / input_h: letterbox space (= 640 для YOLOv8n).
        confidence_threshold: фильтр confidence.
        nms_iou_threshold: IoU threshold для NMS.
        letterbox_info: метаданные letterbox (ADR-0104, issue #2531
            acceptance #9). Если None — bbox'ы нормализуются в
            letterbox-space (старое поведение, обратно совместимо для
            unit-тестов с квадратным синтетическим входом).

    Returns:
        List[dict] в формате VisionEvent-полей. Bbox'ы в normalized
        [0,1] coords (cx, cy, w, h) **исходного кадра** (если
        letterbox_info передан и scale>0) или letterbox-space (если нет).
    """
    try:
        import numpy as np  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            'post-processing requires numpy',
        ) from exc

    if raw_output is None:
        return []

    # issue #2703: HEF со встроенным (on-chip) NMS postprocessing (штатная
    # поставка yolov8n.hef из hailo_model_zoo) отдаёт НЕ единый (1, 84, 8400)
    # тензор, а per-class список разной длины — см. _is_nms_postprocessed_output
    # docstring. np.asarray() на нём в старом коде кидал
    # VisibleDeprecationWarning и падал в ветку "unknown shape" → детекции
    # молча схлопывались в []. Раз-регресс: test_post_process_ragged_nms_*
    # в test_vision_hailo_phase15.py.
    if _is_nms_postprocessed_output(raw_output):
        return _post_process_nms_output(
            raw_output,
            source_camera=source_camera,
            confidence_threshold=confidence_threshold,
            letterbox_info=letterbox_info,
            input_w=input_w,
            input_h=input_h,
        )

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

    # Нормализация bbox'ов в [0, 1] для VisionEvent.
    #
    # ADR-0104 (issue #2531 acceptance #9): bbox из HEF — в letterbox-space.
    # Чтобы получить bbox в исходном кадре, нужно сначала unproject
    # (вычесть паддинг и разделить на scale), а потом нормализовать
    # на (orig_w, orig_h). Если letterbox_info не передан (None) —
    # обратная совместимость со старым unit-тестом с квадратным
    # синтетическим входом: нормализация на letterbox (input_w, input_h).
    if letterbox_info is not None and letterbox_info.scale > 0.0:
        # unproject: letterbox → resized → orig, делим на (orig_w, orig_h).
        u_cx = (boxes[0, :] - letterbox_info.pad_left) / letterbox_info.scale
        u_cy = (boxes[1, :] - letterbox_info.pad_top) / letterbox_info.scale
        u_bw = boxes[2, :] / letterbox_info.scale
        u_bh = boxes[3, :] / letterbox_info.scale
        cx = u_cx / letterbox_info.orig_w
        cy = u_cy / letterbox_info.orig_h
        bw = u_bw / letterbox_info.orig_w
        bh = u_bh / letterbox_info.orig_h
    else:
        # Backward-compat: нормализация в letterbox-space (старые unit-тесты).
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
    nms_iou_threshold: float = 0.45,
) -> HEFLoader:
    """Вернуть правильный loader по launch-параметрам.

    Args:
        hailo_enabled: launch-параметр, разрешает использовать HailoRT.
        hef_path: путь к .hef файлу (None или пустая строка = stub).
        stub_period_sec: период публикации stub-событий.
        nms_iou_threshold: IoU порог NMS (real-mode, YOLOv8n).
    """
    if hailo_enabled and hef_path:
        return RealHEFLoader(
            hef_path=hef_path,
            nms_iou_threshold=nms_iou_threshold,
        )
    return StubHEFLoader(period_sec=stub_period_sec)


# ============================================================================
# Filter helper (без rclpy, чистый Python — тестируется отдельно)
# ============================================================================

def is_stub_event(event: Dict[str, Any]) -> bool:
    """True, если событие выдумано заглушкой, а не получено из кадра.

    Единственное место, где живёт этот вопрос. Потребители (проекция в
    LLM-контекст, safety-логика) обязаны спрашивать здесь, а не сравнивать
    строки у себя — иначе маркер снова разъедется с данными, как это уже
    было с event_type='person'.
    """
    return (
        event.get('event_type') == STUB_EVENT_TYPE
        or event.get('source_camera') == STUB_SOURCE_CAMERA
    )


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
