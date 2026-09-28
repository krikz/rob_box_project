"""observation_geometry — bbox + aligned depth → 3D-наблюдение (ADR-0138).

Этап 1 ADR-0130: у детекции появляется положение в optical frame камеры.
Модуль чистый — без rclpy и без cv2 (numpy импортируется лениво), чтобы
контракт ``Observation`` проверялся в CI без colcon-build, как
``vision_hailo_loader``.

Почему отдельный модуль, а не ``gaze.py`` / ``vision_hailo_node.py``:

* ``gaze.py`` — шов получения кадра (топики, декодирование, синхронизация
  depth с RGB). Геометрия детекции к кадру не относится: она зависит от
  bbox, которого у шва нет.
* ``vision_hailo_node.py`` импортирует ``rclpy`` на уровне модуля, и
  без ROS его тесты пропускаются (skip) — контракт геометрии тогда не
  проверялся бы в CI вовсе.

Что здесь:

* ``STATUS_*`` — словарь ``Observation.position_status`` (единственное
  место, ``gaze.py`` берёт статусы синхронизации отсюда).
* :func:`estimate_position` — медиана валидных глубин по центральной части
  бокса, отсечение выбросов, перевод пикселя через K в optical frame.
* :func:`build_observation` — VisionEvent-dict + кадр → поля
  ``Observation.msg`` (``OBSERVATION_FIELDS``) без имени (ADR-0130 §5.1).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

# ============================================================================
# Словарь Observation.position_status (ADR-0018 capability-honest)
# ============================================================================

#: Глубина есть, точка посчитана.
STATUS_OK = 'ok'
#: У источника нет потока глубины (потолочная камера, stub) или ни один
#: depth-кадр ещё не пришёл.
STATUS_NO_DEPTH_STREAM = 'no_depth_stream'
#: Depth-кадр есть, но ближайший по stamp дальше допуска синхронизации.
STATUS_DEPTH_OUT_OF_SYNC = 'depth_out_of_sync'
#: Размер depth не совпадает с RGB — выравнивание (i_align_depth) сломано.
STATUS_DEPTH_SIZE_MISMATCH = 'depth_size_mismatch'
#: Кодировка depth не 16UC1/mono16 (мм).
STATUS_DEPTH_ENCODING_UNSUPPORTED = 'depth_encoding_unsupported'
#: Нет camera_info (или K нулевая) — пиксель нельзя перевести в метры.
STATUS_NO_CAMERA_INFO = 'no_camera_info'
#: camera_info описывает кадр другого размера, чем RGB.
STATUS_CAMERA_INFO_SIZE_MISMATCH = 'camera_info_size_mismatch'
#: Bbox пустой или целиком вне кадра.
STATUS_BBOX_OUTSIDE_FRAME = 'bbox_outside_frame'
#: В центральной части бокса нет (почти) валидных глубин.
STATUS_DEPTH_INVALID = 'depth_invalid'

#: Классы детекций, для которых публикуется Observation (ADR-0130 §2.2:
#: реализуется только профиль person; «face» — детекция части человека).
OBSERVATION_CLASSES = ('person', 'face')

#: Поля Observation.msg без ``header`` (его ставит нода) — Python-сторона
#: контракта, сверяется с IDL в test_observation_parity.py.
OBSERVATION_FIELDS = (
    'source', 'class_name', 'confidence',
    'bbox_cx_px', 'bbox_cy_px', 'bbox_w_px', 'bbox_h_px',
    'image_width', 'image_height',
    'position', 'distance_m', 'position_valid', 'position_status',
    'position_stddev_m', 'candidate_id', 'candidate_similarity',
)

# Параметры оценки. Не калиброваны на роботе — разумные значения для
# OAK-D Lite (рабочий диапазон стерео ~0.2–10 м); меняются здесь.
DEFAULT_CORE_FRACTION = 0.5
DEFAULT_MIN_DEPTH_M = 0.1
DEFAULT_MAX_DEPTH_M = 20.0
DEFAULT_MIN_VALID_FRACTION = 0.05
# Выброс — дальше k·MAD (MAD·1.4826 ≈ σ для нормального шума) от медианы,
# но не строже _MIN_INLIER_BAND_M: при почти постоянной глубине MAD → 0.
_OUTLIER_MAD_K = 3.0
_MIN_INLIER_BAND_M = 0.05

_NAN_POINT = (math.nan, math.nan, math.nan)


@dataclass(frozen=True)
class PositionEstimate:
    """Результат :func:`estimate_position`.

    Attributes:
        position: (x, y, z) в optical frame, м; NaN×3 при ``valid=False``.
        distance_m: |position|, м; -1.0 при ``valid=False``.
        valid: True только при ``status == STATUS_OK``.
        status: одно из ``STATUS_*``.
        stddev_m: σ глубины инлаеров, м; -1.0 при ``valid=False``.
    """

    position: Tuple[float, float, float]
    distance_m: float
    valid: bool
    status: str
    stddev_m: float


def invalid_estimate(status: str) -> PositionEstimate:
    """Честное «глубины нет» с причиной."""
    return PositionEstimate(
        position=_NAN_POINT,
        distance_m=-1.0,
        valid=False,
        status=status,
        stddev_m=-1.0,
    )


def estimate_position(
    depth_mm: Any,
    intrinsics: Optional[Sequence[float]],
    bbox_px: Tuple[float, float, float, float],
    *,
    upstream_status: str = STATUS_OK,
    core_fraction: float = DEFAULT_CORE_FRACTION,
    min_depth_m: float = DEFAULT_MIN_DEPTH_M,
    max_depth_m: float = DEFAULT_MAX_DEPTH_M,
    min_valid_fraction: float = DEFAULT_MIN_VALID_FRACTION,
) -> PositionEstimate:
    """Bbox + выровненная глубина → точка в optical frame камеры.

    Args:
        depth_mm: HxW uint16 (мм, 0 = нет данных), выровнен по RGB; или None.
        intrinsics: (fx, fy, cx, cy) в пикселях RGB-кадра; или None.
        bbox_px: (cx, cy, w, h) в пикселях исходного RGB-кадра.
        upstream_status: статус глубины от шва «Взгляд»
            (``Frame.depth_status``); не ``ok`` → сразу invalid с ним же.
        core_fraction: доля ширины/высоты бокса вокруг центра, по которой
            берётся глубина (края бокса — фон).
        min_depth_m / max_depth_m: вне диапазона — невалидные пиксели.
        min_valid_fraction: меньше этой доли валидных пикселей в ядре —
            ``depth_invalid``.

    Глубина OAK-D — Z-глубина (REP-118), поэтому
    x = (u − cx)·Z/fx, y = (v − cy)·Z/fy, z = Z; u, v — центр бокса.
    """
    if upstream_status != STATUS_OK:
        return invalid_estimate(upstream_status)
    if depth_mm is None:
        return invalid_estimate(STATUS_NO_DEPTH_STREAM)
    if intrinsics is None:
        return invalid_estimate(STATUS_NO_CAMERA_INFO)
    fx, fy, cx0, cy0 = (float(v) for v in intrinsics)
    if fx <= 0.0 or fy <= 0.0:
        return invalid_estimate(STATUS_NO_CAMERA_INFO)

    import numpy as np  # type: ignore[import-not-found]

    u, v, bw, bh = (float(x) for x in bbox_px)
    if not all(math.isfinite(x) for x in (u, v, bw, bh)) or bw <= 0 or bh <= 0:
        return invalid_estimate(STATUS_BBOX_OUTSIDE_FRAME)

    img_h, img_w = int(depth_mm.shape[0]), int(depth_mm.shape[1])
    half_w = 0.5 * bw * core_fraction
    half_h = 0.5 * bh * core_fraction
    x0 = max(0, int(math.floor(u - half_w)))
    x1 = min(img_w, int(math.ceil(u + half_w)))
    y0 = max(0, int(math.floor(v - half_h)))
    y1 = min(img_h, int(math.ceil(v + half_h)))
    if x1 <= x0 or y1 <= y0:
        return invalid_estimate(STATUS_BBOX_OUTSIDE_FRAME)

    core = np.asarray(depth_mm[y0:y1, x0:x1], dtype=np.float64) / 1000.0
    total = core.size
    values = core[(core >= min_depth_m) & (core <= max_depth_m)]
    if values.size == 0 or values.size < min_valid_fraction * total:
        return invalid_estimate(STATUS_DEPTH_INVALID)

    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    band = max(_OUTLIER_MAD_K * 1.4826 * mad, _MIN_INLIER_BAND_M)
    inliers = values[np.abs(values - median) <= band]
    z = float(np.median(inliers))
    stddev = float(np.std(inliers))

    x = (u - cx0) * z / fx
    y = (v - cy0) * z / fy
    return PositionEstimate(
        position=(x, y, z),
        distance_m=math.sqrt(x * x + y * y + z * z),
        valid=True,
        status=STATUS_OK,
        stddev_m=stddev,
    )


def _candidate_similarity(event: Dict[str, Any]) -> float:
    """Сходство из маркера Встречи в attributes_json; -1.0 — не посчитано.

    Покадровое сходство ``FaceRecognizer`` в VisionEvent не кладёт (только
    ``embedding_id``); на кадре Встречи оно есть в ``attributes_json``.
    """
    raw = event.get('attributes_json') or ''
    if not raw:
        return -1.0
    try:
        attrs = json.loads(raw)
    except (TypeError, ValueError):
        return -1.0
    if not isinstance(attrs, dict):
        return -1.0
    if attrs.get('person_id') not in (None, event.get('embedding_id')):
        return -1.0
    try:
        return float(attrs['similarity'])
    except (KeyError, TypeError, ValueError):
        return -1.0


def build_observation(
    event: Dict[str, Any],
    *,
    source: str,
    image_width: int,
    image_height: int,
    depth_mm: Any = None,
    intrinsics: Optional[Sequence[float]] = None,
    depth_status: str = STATUS_NO_DEPTH_STREAM,
) -> Optional[Dict[str, Any]]:
    """VisionEvent-dict (bbox в долях кадра) → поля Observation.msg.

    Returns:
        dict с ключами ``OBSERVATION_FIELDS`` (``position`` — кортеж
        (x, y, z)); None — если класс вне ``OBSERVATION_CLASSES`` или у
        кадра нет размера (наблюдение не из кадра).

    Имени (``display_name``) в результате нет и быть не должно
    (ADR-0130 §5.1 п.1); ``candidate_id`` — ссылка на запись хранилища
    примет, а не id личности (§5.1 п.5).
    """
    class_name = str(event.get('class_name', ''))
    if class_name not in OBSERVATION_CLASSES:
        return None
    if image_width <= 0 or image_height <= 0:
        return None

    bbox_px = (
        float(event.get('bbox_cx', -1.0)) * image_width,
        float(event.get('bbox_cy', -1.0)) * image_height,
        float(event.get('bbox_w', -1.0)) * image_width,
        float(event.get('bbox_h', -1.0)) * image_height,
    )
    estimate = estimate_position(
        depth_mm, intrinsics, bbox_px, upstream_status=depth_status,
    )
    candidate_id = str(event.get('embedding_id', '') or '')
    return {
        'source': str(source),
        'class_name': class_name,
        'confidence': float(event.get('confidence', 0.0)),
        'bbox_cx_px': bbox_px[0],
        'bbox_cy_px': bbox_px[1],
        'bbox_w_px': bbox_px[2],
        'bbox_h_px': bbox_px[3],
        'image_width': int(image_width),
        'image_height': int(image_height),
        'position': estimate.position,
        'distance_m': estimate.distance_m,
        'position_valid': estimate.valid,
        'position_status': estimate.status,
        'position_stddev_m': estimate.stddev_m,
        'candidate_id': candidate_id,
        'candidate_similarity': (
            _candidate_similarity(event) if candidate_id else -1.0
        ),
    }
