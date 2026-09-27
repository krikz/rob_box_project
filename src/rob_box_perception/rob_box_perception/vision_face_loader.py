#!/usr/bin/env python3
"""vision_face_loader.py — RetinaFace HEF loader (ADR-0089 Phase 2, PR-A).

Почему отдельный модуль, а не расширение ``vision_hailo_loader.py``:
RetinaFace — принципиально другой декодер (anchors + box-regression +
landmarks + softmax + NMS), а не YOLO anchor-decode. Примешивать его к
``RealHEFLoader`` значило бы завести второй код-путь в одном классе, где
сейчас каждый метод заточен под YOLOv8n. Issue #2599 PR-A фиксирует
архитектуру «отдельный ``vision_face_node``» — модуль следует тому же
решению.

Модуль — чистый Python без rclpy (импортируется и тестируется в CI без
colcon-build), по тому же контракту, что ``vision_hailo_loader``.

PR-A scope: **только детекция лица** — ``event_type="face"`` + bbox.
``embedding_id`` / ``display_name`` остаются пустыми (это PR-B, ArcFace).
Эмбеддинги не считаются, БД нет → privacy-review для PR-A не требуется.

Issue #2773 (после PR-B, "PR-C"): landmarks-тензоры, которые HEF УЖЕ
отдавал, но PR-A их выбрасывал, теперь декодируются тем же анкерным
декодером, что и боксы (``_decode_landmarks``), проходят тот же
confidence-фильтр и NMS, что и боксы (СОГЛАСОВАННО — одни и те же
индексы отбора), и unproject'ятся в координаты исходного кадра тем же
``LetterboxInfo``, что и bbox. Landmarks кладутся в VisionEvent-dict
под ключом ``landmarks`` — контракт с ``face_embedding.align_face``,
см. докстринг ``_post_process_faces``. Причина: ArcFace на невыровненном
кропе (bbox + запас + анизотропный resize) даёт внутриперсонный разброс
эмбеддингов 0.3–0.85, который перекрывает межперсонный (issue #2773,
живые замеры на роботе 22.09.2026) — выравнивание по 5 точкам это чинит.

Спецификация модели (hailo_model_zoo retinaface_mobilenet_v1, проверено
по ``cfg/networks/retinaface_mobilenet_v1.yaml`` и
``core/postprocessing/face_detection_postprocessing.py``):
  - input_shape:  736x1280x3 (HxWxC).
  - output_shape: 9 тензоров, по 3 на feature-map (stride 8/16/32):
      box (2 anchors x 4), class (2 anchors x 2 logits),
      landmarks (2 anchors x 10). Перечисление в yaml идёт
      (box, class, landmarks) ПОДРЯД на каждый feature-map.
  - anchors (biubug6/Pytorch_Retinaface cfg_mnet):
      min_sizes [[16,32],[64,128],[256,512]], steps [8,16,32].
  - variances: box dx/dy /10, box dw/dh /5 (SCALE_FACTORS=(10,5)).
  - score: softmax(class_logits)[..., 1] — индекс 1 = лицо.

Поля, требующие проверки на живом железе (вынесены в PR-A acceptance):
  - входной формат HEF (uint8 NHWC — принято по образцу YOLOv8n),
  - порядок output_names в HailoRT (принято по yaml output_shape),
  - фактический FPS при двух HEF параллельно.

Failure policy (ADR-0018 capability-honest): любой сбой HailoRT
(init / run / post-process) propagates как exception. Узел логирует
и пропускает кадр — НЕ silent fallback на stub.

Touchpoints:
- ADR-0089 §2.1 Phase 2 (retinaface_mobilenet_v1.hef).
- Issue #2599 PR-A (детекция лица без идентификации).
- Issue #2773 (landmarks decode, выравнивание ArcFace-входа).
- rob_box_perception.vision_hailo_loader (StubHEFLoader, LetterboxInfo,
  is_stub_event, filter_by_confidence — переиспользуются).
- rob_box_perception.face_embedding.align_face (потребитель ключа
  ``landmarks``).
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from rob_box_perception.hailo_device import open_device
from rob_box_perception.vision_hailo_loader import (
    HEFLoader,
    LETTERBOX_PAD_VALUE,
    LetterboxInfo,
    StubHEFLoader,
    _is_recoverable_stream_abort,
    _next_reinit_backoff_sec,
)

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Константы модели (hailo_model_zoo retinaface_mobilenet_v1)
# ---------------------------------------------------------------------------

#: Входной размер HEF: 736 высота x 1280 ширина (input_shape: 736x1280x3).
DEFAULT_INPUT_H = 736
DEFAULT_INPUT_W = 1280

#: Anchors: 3 feature-map'а, 2 анкера на клетку (biubug6 cfg_mnet).
ANCHOR_STEPS: Tuple[int, ...] = (8, 16, 32)
ANCHOR_MIN_SIZES: Tuple[Tuple[int, int], ...] = (
    (16, 32),
    (64, 128),
    (256, 512),
)

#: Variances RetinaFace (SCALE_FACTORS=(10.0, 5.0) в hailo_model_zoo).
BOX_VARIANCE_CENTER = 10.0
BOX_VARIANCE_SIZE = 5.0

#: Число каналов выходов каждого feature-map (NHWC, channel-last).
#: box = 2 anchors x 4, class = 2 anchors x 2, landmarks = 2 anchors x 10.
_BOX_CHANNELS = 8
_CLASS_CHANNELS = 4
_LANDMARK_CHANNELS = 20

#: Порядок выходов в HEF (по yaml output_shape): (box, class, landmarks)
#: ПОДРЯД для каждого из 3 feature-map'ов. index // 3 = feature-map,
#: index % 3 = тип выхода.
_OUTPUT_CHANNELS_PER_TYPE: Tuple[int, int, int] = (
    _BOX_CHANNELS,
    _CLASS_CHANNELS,
    _LANDMARK_CHANNELS,
)


def _feature_map_sizes() -> Tuple[Tuple[int, int], ...]:
    """Размеры feature-map'ов (H, W) для stride 8/16/32 при входе 736x1280."""
    return (
        (DEFAULT_INPUT_H // 8, DEFAULT_INPUT_W // 8),    # 92 x 160
        (DEFAULT_INPUT_H // 16, DEFAULT_INPUT_W // 16),  # 46 x 80
        (DEFAULT_INPUT_H // 32, DEFAULT_INPUT_W // 32),  # 23 x 40
    )


FEATURE_MAP_SIZES: Tuple[Tuple[int, int], ...] = _feature_map_sizes()


# ============================================================================
# Anchors + декодер (pure numpy, тестируется без железа)
# ============================================================================

def _build_anchors() -> Any:
    """Построить (N, 4) normalized anchors [cx, cy, w, h] в [0, 1].

    Порядок обязан совпадать с порядком flatten'а выходов HEF:
    feature-map (stride 8 → 16 → 32), внутри — row-major (i, j),
    внутри клетки — 2 анкера (min_sizes[0], min_sizes[1]).

    Формула — 1:1 из hailo_model_zoo ``FaceDetectionPostProc.extract_anchors``
    (image_dims=(736, 1280)):
        s_kx = min_size / W ; s_ky = min_size / H
        cx = (j + 0.5) / fmap_w ; cy = (i + 0.5) / fmap_h
    """
    import numpy as np  # type: ignore[import-not-found]

    anchors: List[List[float]] = []
    for idx, (fh, fw) in enumerate(FEATURE_MAP_SIZES):
        min_sizes = ANCHOR_MIN_SIZES[idx]
        for i in range(fh):
            for j in range(fw):
                for min_size in min_sizes:
                    anchors.append([
                        (j + 0.5) / fw,          # cx (normalized)
                        (i + 0.5) / fh,          # cy (normalized)
                        min_size / DEFAULT_INPUT_W,  # w (normalized)
                        min_size / DEFAULT_INPUT_H,  # h (normalized)
                    ])
    return np.asarray(anchors, dtype=np.float32)


def _decode_boxes(box_predictions: Any, anchors: Any) -> Any:
    """RetinaFace box decode: regression -> normalized xyxy в letterbox-space.

    box_predictions: (N, 4) [dx, dy, dw, dh]; anchors: (N, 4) [cx, cy, w, h].
    Формула — 1:1 из ``FaceDetectionPostProc._decode_boxes``:
        cx = a_cx + dx / 10 * a_w ; cy = a_cy + dy / 10 * a_h
        w  = a_w * exp(dw / 5) ; h = a_h * exp(dh / 5)
    затем cxcywh -> xyxy (x1, y1, x2, y2).
    """
    import numpy as np  # type: ignore[import-not-found]

    cx = anchors[:, 0] + box_predictions[:, 0] / BOX_VARIANCE_CENTER * anchors[:, 2]
    cy = anchors[:, 1] + box_predictions[:, 1] / BOX_VARIANCE_CENTER * anchors[:, 3]
    w = anchors[:, 2] * np.exp(box_predictions[:, 2] / BOX_VARIANCE_SIZE)
    h = anchors[:, 3] * np.exp(box_predictions[:, 3] / BOX_VARIANCE_SIZE)
    x1 = cx - w / 2.0
    y1 = cy - h / 2.0
    x2 = cx + w / 2.0
    y2 = cy + h / 2.0
    return np.stack([x1, y1, x2, y2], axis=1)


def _decode_landmarks(landmark_predictions: Any, anchors: Any) -> Any:
    # НЕ ПРОВЕРЕНО НА РЕАЛЬНОМ HEF (issue #2773 честно): делитель 10 ниже
    # и порядок 5 точек (left_eye, right_eye, nose, mouth_left,
    # mouth_right) взяты по сверке с эталонной biubug6/Pytorch_Retinaface
    # реализацией, а НЕ измерены на фактическом выходе
    # retinaface_mobilenet_v1.hef — hailo_model_zoo postprocessing config
    # в этом репозитории не лежит. Если на живом роботе landmarks
    # окажутся систематически смещены или точки перепутаны местами —
    # первое место для перепроверки именно здесь, а не в unproject/NMS.
    """RetinaFace landmark decode: regression -> normalized (x, y) x 5 точек, letterbox-space.

    landmark_predictions: (N, 10) [dx1, dy1, dx2, dy2, ..., dx5, dy5] —
    5 точек лица в фиксированном порядке biubug6/Pytorch_Retinaface
    (совпадает с порядком, который требует issue #2773 для контракта
    VisionEvent): left_eye, right_eye, nose, mouth_left, mouth_right.
    anchors: (N, 4) [cx, cy, w, h] — те же анкеры, что и для боксов.

    Масштабный делитель (ЧЕСТНО — не проверено на реальном HEF, только
    сверено с эталонной реализацией, а не измерено): в эталонном
    biubug6/Pytorch_Retinaface ``utils/box_utils.py::decode_landm``
    landmarks декодируются как::

        landm_k = anchor_center + pre_k * variances[0] * anchor_wh

    где ``variances = (0.1, 0.2)`` (тот же cfg_mnet, на который уже
    ссылается докстринг ``_build_anchors`` для anchors и который
    задаёт ``BOX_VARIANCE_CENTER = 10.0`` для box-центра: ``variances[0]
    = 0.1 == 1 / BOX_VARIANCE_CENTER``). То есть у landmarks используется
    ТОТ ЖЕ делитель 10, что и у box-центра (dx/dy), а не делитель 5,
    которым box декодирует размер (dw/dh) — у landmarks аналога
    "размера" нет, регрессия только по смещению точки. hailo_model_zoo
    компилирует эту же эталонную реализацию в HEF (см. модульный
    docstring про ``retinaface_mobilenet_v1``), поэтому формула должна
    совпадать, но: hailo_model_zoo postprocessing config
    (``core/postprocessing/face_detection_postprocessing.py``) в этом
    репозитории не лежит и в рамках issue #2773 не читался напрямую —
    если на живом HEF landmarks выйдут смещёнными сильнее, чем боксы,
    это первое место для перепроверки (сверить с фактическим
    ``face_detection_postprocessing.py`` из hailo_model_zoo на роботе).

    Формула по каждой из 5 точек:
        x_k = a_cx + dx_k / 10 * a_w
        y_k = a_cy + dy_k / 10 * a_h

    Returns:
        (N, 10) в том же порядке каналов, что вход: [x1, y1, ..., x5, y5],
        normalized в letterbox-space (до unproject через LetterboxInfo).
    """
    import numpy as np  # type: ignore[import-not-found]

    a_cx = anchors[:, 0]
    a_cy = anchors[:, 1]
    a_w = anchors[:, 2]
    a_h = anchors[:, 3]

    decoded = np.empty_like(landmark_predictions, dtype=np.float32)
    for k in range(5):
        dx = landmark_predictions[:, 2 * k]
        dy = landmark_predictions[:, 2 * k + 1]
        decoded[:, 2 * k] = a_cx + dx / BOX_VARIANCE_CENTER * a_w
        decoded[:, 2 * k + 1] = a_cy + dy / BOX_VARIANCE_CENTER * a_h
    return decoded


def _softmax(logits: Any, axis: int) -> Any:
    """Численно стабильный softmax вдоль заданной оси."""
    import numpy as np  # type: ignore[import-not-found]

    shifted = logits - np.max(logits, axis=axis, keepdims=True)
    exp = np.exp(shifted)
    return exp / np.sum(exp, axis=axis, keepdims=True)


def _xyxy_to_cxcywh_normalized(
    xyxy: Any,
    letterbox_info: Optional[LetterboxInfo],
) -> Tuple[Any, Any, Any, Any]:
    """xyxy в letterbox-space -> normalized (cx, cy, w, h) исходного кадра.

    RetinaFace отдаёт bbox'ы уже normalized в letterbox-space [0, 1]
    (anchors normalized). Чтобы вернуть их в исходный кадр:
        letterbox px = xyxy * (letterbox_w, letterbox_h)
        unproject (вычесть pad, разделить на scale) -> исходный кадр
        нормализовать на (orig_w, orig_h).
    Если ``letterbox_info`` None (синтетический квадратный вход в тестах) —
    считаем bbox'ы уже normalized в model-input space, возвращаем как есть.
    """
    x1 = xyxy[:, 0]
    y1 = xyxy[:, 1]
    x2 = xyxy[:, 2]
    y2 = xyxy[:, 3]

    if letterbox_info is not None and letterbox_info.scale > 0.0:
        lw = letterbox_info.letterbox_w
        lh = letterbox_info.letterbox_h
        ow = letterbox_info.orig_w
        oh = letterbox_info.orig_h
        # letterbox px -> исходный кадр -> [0, 1].
        x1 = (x1 * lw - letterbox_info.pad_left) / letterbox_info.scale / ow
        x2 = (x2 * lw - letterbox_info.pad_left) / letterbox_info.scale / ow
        y1 = (y1 * lh - letterbox_info.pad_top) / letterbox_info.scale / oh
        y2 = (y2 * lh - letterbox_info.pad_top) / letterbox_info.scale / oh

    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    w = x2 - x1
    h = y2 - y1
    return cx, cy, w, h


def _unproject_landmarks_normalized(
    landmarks: Any,
    letterbox_info: Optional[LetterboxInfo],
) -> Any:
    """5 точек лица в letterbox-space -> normalized координаты исходного кадра.

    Тот же unproject, что ``_xyxy_to_cxcywh_normalized`` делает для
    bbox'а (issue #2773 требует ИМЕННО эту обратную проекцию для
    landmarks — точки обязаны быть в системе координат исходного кадра,
    а не letterbox-тензора 736x1280, иначе ``align_face`` в
    ``face_embedding.py`` посчитает similarity-transform по неверным
    координатам и выравнивание будет хуже, чем его отсутствие):
        letterbox px = xy * (letterbox_w, letterbox_h)
        unproject (вычесть pad, разделить на scale) -> исходный кадр px
        нормализовать на (orig_w, orig_h)
    Если ``letterbox_info`` None (синтетический вход без паддинга в
    юнит-тестах) — точки уже в model-input space, возвращаются как есть
    — та же конвенция, что и у bbox.

    Args:
        landmarks: (N, 10) [x1, y1, ..., x5, y5], normalized letterbox-space.
        letterbox_info: метаданные letterbox или None.

    Returns:
        (N, 10) в исходном кадре, normalized [0, 1] (может выйти за
        пределы [0, 1] для точек у самой границы кадра — вызывающий код
        это не клампит, ровно как и bbox).
    """
    if letterbox_info is None or letterbox_info.scale <= 0.0:
        return landmarks

    lw = letterbox_info.letterbox_w
    lh = letterbox_info.letterbox_h
    ow = letterbox_info.orig_w
    oh = letterbox_info.orig_h

    out = landmarks.copy()
    for k in range(5):
        x = landmarks[:, 2 * k]
        y = landmarks[:, 2 * k + 1]
        out[:, 2 * k] = (x * lw - letterbox_info.pad_left) / letterbox_info.scale / ow
        out[:, 2 * k + 1] = (y * lh - letterbox_info.pad_top) / letterbox_info.scale / oh
    return out


def _normalize_output_layout(
    buf: Any,
    expected_h: int,
    expected_w: int,
    expected_c: int,
) -> Any:
    """Привести выходной буфер HEF к (H, W, C) NHWC.

    HailoRT отдаёт буфер в компоновке, которую диктует скомпилированный
    граф. Model zoo описывает output как HxWxC (channel-last). Здесь мы
    нормализуем любую из типичных компоновок к (H, W, C):
      - (1, H, W, C) NHWC        -> [0] -> (H, W, C)
      - (1, C, H, W) NCHW        -> transpose -> (H, W, C)
      - flat (H*W*C,)            -> reshape -> (H, W, C)
    """
    import numpy as np  # type: ignore[import-not-found]

    arr = np.asarray(buf)
    total = expected_h * expected_w * expected_c
    if arr.ndim == 4:
        arr = arr[0]
    if arr.size != total:
        raise ValueError(
            f'Некорректный размер выходного тензора: {arr.size} != '
            f'{total} (ожидалось H={expected_h} W={expected_w} C={expected_c}).'
        )
    if arr.shape == (expected_h, expected_w, expected_c):
        return arr
    if arr.shape == (expected_c, expected_h, expected_w):
        return arr.transpose(1, 2, 0)
    if arr.size == total:
        return arr.reshape(expected_h, expected_w, expected_c)
    raise ValueError(
        f'Некомпонуемый выходной тензор: shape={arr.shape}, '
        f'ожидалось ({expected_h}, {expected_w}, {expected_c}).'
    )


def _post_process_faces(
    raw_outputs: List[Any],
    source_camera: str,
    confidence_threshold: float,
    nms_iou_threshold: float,
    letterbox_info: Optional[LetterboxInfo] = None,
) -> List[Dict[str, Any]]:
    """RetinaFace HEF outputs -> List[VisionEvent-dict] (event_type="face").

    Args:
        raw_outputs: 9 numpy-тензоров в порядке yaml output_shape:
            (box, class, landmarks) x (stride 8, stride 16, stride 32).
            Каждый — произвольной компоновки (см. _normalize_output_layout).
        source_camera: подставляется в VisionEvent.source_camera.
        confidence_threshold: порог face-score (softmax[...,1]).
        nms_iou_threshold: IoU порог NMS (single-class).
        letterbox_info: метаданные letterbox для обратной проекции bbox
            в исходный кадр (ADR-0104, как в YOLOv8n). None = квадратный
            синтетический вход без паддинга (юнит-тесты).

    Returns:
        List[dict] в формате VisionEvent-полей. ``event_type="face"``,
        ``embedding_id`` / ``display_name`` пустые (PR-A). ``landmarks``
        (issue #2773, PR-C) — плоский список из 10 normalized float'ов
        исходного кадра ``[left_eye_x, left_eye_y, right_eye_x,
        right_eye_y, nose_x, nose_y, mouth_left_x, mouth_left_y,
        mouth_right_x, mouth_right_y]``, либо ``None``, если landmarks
        не удалось декодировать (нет landmark-тензоров на входе —
        обратная совместимость с 6-тензорным PR-A выходом синтетических
        тестов — либо декод дал не-конечные числа). Ключ ``landmarks``
        присутствует ВСЕГДА, даже когда значение ``None`` — контракт с
        ``face_embedding.align_face``, который ждёт ключ, а не его
        отсутствие.
    """
    import numpy as np  # type: ignore[import-not-found]

    if not raw_outputs:
        return []

    anchors = _build_anchors()

    n_branches = len(FEATURE_MAP_SIZES)
    # Обратная совместимость: некоторые синтетические входы (PR-A,
    # старые фикстуры) несут только (box, class) на branch, без
    # landmarks. Полный retinaface-выход — 3 тензора на branch.
    has_landmarks = len(raw_outputs) >= 3 * n_branches

    boxes_list: List[Any] = []
    scores_list: List[Any] = []
    landmarks_list: List[Any] = []
    offset = 0
    for idx in range(n_branches):
        fh, fw = FEATURE_MAP_SIZES[idx]
        n_anchors = fh * fw * 2
        stride = 3 if has_landmarks else 2

        box_t = _normalize_output_layout(
            raw_outputs[idx * stride + 0], fh, fw, _BOX_CHANNELS
        ).reshape(-1, _BOX_CHANNELS // 2)  # (N, 4)  [dx, dy, dw, dh]
        cls_t = _normalize_output_layout(
            raw_outputs[idx * stride + 1], fh, fw, _CLASS_CHANNELS
        ).reshape(-1, _CLASS_CHANNELS // 2)  # (N, 2)  [bg, face]

        branch_anchors = anchors[offset:offset + n_anchors]
        offset += n_anchors

        decoded = _decode_boxes(box_t.astype(np.float32), branch_anchors)
        probs = _softmax(cls_t.astype(np.float32), axis=1)
        face_scores = probs[:, 1]

        boxes_list.append(decoded)
        scores_list.append(face_scores)

        if has_landmarks:
            lmk_t = _normalize_output_layout(
                raw_outputs[idx * stride + 2], fh, fw, _LANDMARK_CHANNELS
            ).reshape(-1, _LANDMARK_CHANNELS // 2)  # (N, 10)
            landmarks_list.append(
                _decode_landmarks(lmk_t.astype(np.float32), branch_anchors)
            )

    boxes = np.concatenate(boxes_list, axis=0)
    scores = np.concatenate(scores_list, axis=0)
    landmarks = np.concatenate(landmarks_list, axis=0) if has_landmarks else None

    # Confidence filter до NMS.
    keep = scores >= confidence_threshold
    if not np.any(keep):
        return []
    boxes = boxes[keep]
    scores = scores[keep]
    if landmarks is not None:
        landmarks = landmarks[keep]

    # NMS. Все боксы — один класс ("face"), поэтому class_ids = 0.
    # Переиспользуем _nms_per_class из vision_hailo_loader (single-class
    # там работает без изменений).
    from rob_box_perception.vision_hailo_loader import _nms_per_class

    try:
        keep_idx = _nms_per_class(
            boxes,
            scores,
            np.zeros(len(boxes), dtype=int),
            iou_threshold=nms_iou_threshold,
        )
    except (ValueError, IndexError, TypeError):
        return []
    if not keep_idx:
        return []

    boxes = boxes[keep_idx]
    scores = scores[keep_idx]
    if landmarks is not None:
        # Те же индексы NMS-отбора, что и для боксов (issue #2773:
        # точки обязаны пройти СОГЛАСОВАННЫЙ с боксами отбор, иначе
        # landmarks[i] будет относиться к другому лицу, чем bbox[i]).
        landmarks = landmarks[keep_idx]

    cx, cy, w, h = _xyxy_to_cxcywh_normalized(boxes, letterbox_info)
    if landmarks is not None:
        landmarks = _unproject_landmarks_normalized(landmarks, letterbox_info)

    events: List[Dict[str, Any]] = []
    for i in range(len(scores)):
        landmarks_out: Optional[List[float]] = None
        if landmarks is not None:
            row = [float(v) for v in landmarks[i]]
            if all(np.isfinite(v) for v in row):
                landmarks_out = row

        events.append({
            'source_camera': source_camera,
            'event_type': 'face',
            'class_name': 'face',
            'class_id': 0,
            'confidence': float(scores[i]),
            'bbox_cx': float(cx[i]),
            'bbox_cy': float(cy[i]),
            'bbox_w': float(w[i]),
            'bbox_h': float(h[i]),
            'distance_m': -1.0,
            'embedding_id': '',
            'display_name': '',
            'attributes_json': '',
            'landmarks': landmarks_out,
        })
    return events


# ============================================================================
# Real loader
# ============================================================================

class RetinaFaceLoader(HEFLoader):
    """Реальный retinaface HEF loader через ``hailort`` Python API.

    Инициализация повторяет modern async API из ``RealHEFLoader``
    (VDevice.create_infer_model -> configure -> create_bindings -> run),
    но под 9 выходов retinaface и вход 736x1280x3.

    Attributes:
        _hef_path: путь к retinaface_mobilenet_v1.hef.
        _vdevice_id: ID VDevice (на Pi5 с одним HAT+ всегда 0).
        _confidence_threshold: фильтр face-score (по умолчанию 0.6).
        _nms_iou_threshold: IoU порог NMS (по умолчанию 0.45).
    """

    def __init__(
        self,
        hef_path: str,
        vdevice_id: int = 0,
        confidence_threshold: float = 0.6,
        nms_iou_threshold: float = 0.45,
    ) -> None:
        self._hef_path = hef_path
        self._vdevice_id = vdevice_id
        self._confidence_threshold = confidence_threshold
        self._nms_iou_threshold = nms_iou_threshold
        self._vdevice: Any = None
        self._infer_model: Any = None
        self._configured: Any = None
        self._bindings: Any = None
        self._output_names: List[str] = []
        self._init_failed: Optional[BaseException] = None
        # ---- HAILO_STREAM_ABORT(63) recovery state (issue #2625) ----
        # Тот же паттерн, что RealHEFLoader (vision_hailo_loader.py) —
        # см. docstring там для полного обоснования. Общий код (детекция
        # abort'а + расчёт backoff) переиспользован через импорт
        # `_is_recoverable_stream_abort`/`_next_reinit_backoff_sec`;
        # само reset/retry-состояние копируется по месту (ADR-0121
        # предлагает вынести общий lazy-init в миксин, но остаётся
        # Proposed — сейчас код следует уже принятому в модуле паттерну
        # дублирования `_init_failed`/`_ensure_initialized`/`_init_locked`).
        self._recovering_from_abort: bool = False
        self._reinit_backoff_sec: float = 0.0
        self._reinit_not_before: float = 0.0

    # ----------------------------------------------------------------
    # Lazy initialization
    # ----------------------------------------------------------------

    def _ensure_initialized(self) -> None:
        if self._configured is not None:
            return
        if self._init_failed is not None:
            raise self._init_failed
        if self._recovering_from_abort:
            self._retry_after_abort()
            return
        try:
            self._init_locked()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            self._init_failed = exc
            raise

    def _retry_after_abort(self) -> None:
        """Reinit после `HAILO_STREAM_ABORT(63)` с backoff (issue #2625).

        1:1 паттерн ``RealHEFLoader._retry_after_abort`` — см. его docstring.
        Не кеширует неудачу в ``_init_failed`` (transient, не фатальный
        отказ инициализации).
        """
        now = time.monotonic()
        if now < self._reinit_not_before:
            remaining = self._reinit_not_before - now
            raise RuntimeError(
                f'HailoRT pipeline recovering from HAILO_STREAM_ABORT(63): '
                f'backoff активен ещё {remaining:.1f}s (issue #2625).'
            )
        try:
            self._init_locked()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            self._reinit_backoff_sec = _next_reinit_backoff_sec(
                self._reinit_backoff_sec
            )
            self._reinit_not_before = time.monotonic() + self._reinit_backoff_sec
            _LOG.warning(
                'Переинициализация RetinaFace после HAILO_STREAM_ABORT(63) '
                f'снова не удалась: {exc!r}. Следующая попытка через '
                f'{self._reinit_backoff_sec:.1f}s (issue #2625).'
            )
            raise
        _LOG.warning(
            'RetinaFace HailoRT пайплайн переподнят после '
            'HAILO_STREAM_ABORT(63) (issue #2625) — реинициализация '
            'прошла успешно.'
        )
        self._recovering_from_abort = False
        self._reinit_backoff_sec = 0.0
        self._reinit_not_before = 0.0

    def _reset_after_stream_abort(self, exc: BaseException) -> None:
        """Сбросить lazy-init состояние после abort'а (issue #2625 п.1).

        1:1 паттерн ``RealHEFLoader._reset_after_stream_abort``.
        """
        _LOG.warning(
            'RetinaFace HailoRT pipeline abort обнаружен '
            f'(HAILO_STREAM_ABORT(63) pattern): {exc!r}. Сбрасываю '
            'состояние лоадера для переинициализации на следующем кадре '
            '(issue #2625, ADR-0112 §5 п.1).'
        )
        self._configured = None
        self._bindings = None
        self._infer_model = None
        self._init_failed = None
        self._vdevice = None
        self._device = None
        self._output_names = []
        self._recovering_from_abort = True
        self._reinit_backoff_sec = 0.0
        self._reinit_not_before = 0.0

    def _init_locked(self) -> None:
        if not os.path.isfile(self._hef_path):
            raise FileNotFoundError(
                f'HEF не найден: {self._hef_path}. Кладёт его на хост Ресурсный '
                f'пак (docker/vision/scripts/resource_pack/apply_resource_pack.sh '
                f'--only retinaface-hef, docker/vision/scripts/resource_pack/manifest.yaml).'
            )

        # Устройство — через шов «Ускоритель» (hailo_device). Прямой
        # VDevice тут означал «кто первый встал, тот и владеет Hailo»:
        # 15.09.2026 на роботе person-нода и лицевая по очереди забирали
        # устройство, а проигравшая молча жила в degraded (issue #2599).
        self._device = open_device(self._vdevice_id)
        self._vdevice = self._device.vdevice

        self._infer_model = self._vdevice.create_infer_model(self._hef_path)
        self._infer_model.set_batch_size(1)

        # Вход retinaface HEF: uint8 NHWC 736x1280x3. Принято по образцу
        # YOLOv8n; фактический формат обязан подтвердиться на живом железе
        # (PR-A acceptance).
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]
            try:
                self._infer_model.input().set_format_type(FormatType.UINT8)
            except AttributeError:
                pass
        except ImportError:
            pass

        # Выходы retinaface квантованы (uint8). Буферы ниже мы
        # аллоцируем float32, поэтому формат обязаны запросить явно —
        # иначе HailoRT считает размер в 4 раза меньше и отвечает
        # HAILO_INVALID_OPERATION(6):
        #   "Output buffer size 471040 is different than expected 117760
        #    for output 'retinaface_mobilenet_v1/conv41'"
        # (живой робот, 15.09.2026 — допущение «по образцу YOLOv8n»
        # не подтвердилось).
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]
            for out_name in self._infer_model.output_names:
                try:
                    self._infer_model.output(out_name).set_format_type(
                        FormatType.FLOAT32
                    )
                except AttributeError:
                    pass
        except ImportError:
            pass

        self._configured = self._infer_model.configure()

        import numpy as np  # type: ignore[import-not-found]

        input_buffers: Dict[str, Any] = {}
        for in_name in self._infer_model.input_names:
            shape = list(self._infer_model.input(in_name).shape)
            input_buffers[in_name] = np.empty(shape, dtype=np.uint8)

        output_buffers: Dict[str, Any] = {}
        for out_name in self._infer_model.output_names:
            shape = list(self._infer_model.output(out_name).shape)
            output_buffers[out_name] = np.empty(shape, dtype=np.float32)

        self._bindings = self._configured.create_bindings(
            input_buffers=input_buffers,
            output_buffers=output_buffers,
        )
        # Активация нужна ТОЛЬКО когда устройство наше единолично
        # (иначе первый run() → HAILO_STREAM_NOT_ACTIVATED(72), #2398).
        # Под сервисом активацией владеет планировщик, и ручной вызов
        # даёт HAILO_INVALID_OPERATION(6) — решение живёт в шве.
        if self._device.must_activate:
            self._configured.activate()

        # Порядок выходов принимаем равным yaml output_shape. Если HailoRT
        # отдаёт их в другом порядке — это выявляется на живом железе
        # (PR-A acceptance: «измерить и зафиксировать фактическое»).
        self._output_names = list(self._infer_model.output_names)

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------

    def is_available(self) -> bool:
        try:
            self._ensure_initialized()
            return True
        except BaseException:  # noqa: BLE001 (capability-honest init phase)
            return False

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        """Запуск real face-detection на одном кадре.

        Args:
            frame_id: ROS header.frame_id источника.
            image: numpy.ndarray (HxWx3, RGB). None = «нет кадра» → [].

        Returns:
            List[dict] VisionEvent-полей, event_type="face".

        Raises:
            RuntimeError / ImportError / ValueError: init/run/post-process.
        """
        if image is None:
            return []
        self._ensure_initialized()

        input_tensor, letterbox_info = self._preprocess(image)

        assert self._bindings is not None
        assert self._configured is not None
        assert self._infer_model is not None

        in_name = self._infer_model.input_names[0]
        try:
            self._bindings.input(in_name).set_buffer(input_tensor)
            self._configured.run([self._bindings], timeout=1000)
        except Exception as exc:  # noqa: BLE001
            if _is_recoverable_stream_abort(exc):
                self._reset_after_stream_abort(exc)
            raise RuntimeError(f'HailoRT run() failed: {exc!r}') from exc

        raw_outputs: List[Any] = []
        for idx, out_name in enumerate(self._output_names):
            buf = self._bindings.output(out_name).get_buffer()
            raw_outputs.append(buf)

        return _post_process_faces(
            raw_outputs=raw_outputs,
            source_camera=frame_id or 'unknown',
            confidence_threshold=self._confidence_threshold,
            nms_iou_threshold=self._nms_iou_threshold,
            letterbox_info=letterbox_info,
        )

    # ----------------------------------------------------------------
    # Pre-processing
    # ----------------------------------------------------------------

    def _preprocess(self, image: Any) -> Tuple[Any, LetterboxInfo]:
        """RGB -> NHWC uint8, letterboxed 736x1280 (симметричный паддинг).

        Конвенция паддинга обязана совпадать с ``Frame.as_letterbox`` /
        ``vision_hailo_loader._preprocess`` (issue #2584): pad делится
        поровну между противоположными сторонами. RetinaFace-декодер
        отдаёт normalized bbox'ы, поэтому unproject выполняется в
        ``_xyxy_to_cxcywh_normalized`` через ``LetterboxInfo``.
        """
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                'Real face inference requires opencv-python + numpy.',
            ) from exc

        h, w = image.shape[:2]
        scale = min(DEFAULT_INPUT_W / w, DEFAULT_INPUT_H / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))

        if (w, h) != (new_w, new_h):
            resized = cv2.resize(image, (new_w, new_h))
        else:
            resized = image

        pad_w = DEFAULT_INPUT_W - new_w
        pad_h = DEFAULT_INPUT_H - new_h
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

        if padded.dtype != np.uint8:
            padded = padded.astype(np.uint8)
        if len(padded.shape) == 3:
            tensor = np.expand_dims(padded, axis=0)
        else:
            tensor = padded

        info = LetterboxInfo(
            scale=float(scale),
            pad_left=int(pad_left),
            pad_top=int(pad_top),
            orig_w=int(w),
            orig_h=int(h),
            letterbox_w=int(DEFAULT_INPUT_W),
            letterbox_h=int(DEFAULT_INPUT_H),
        )
        return np.ascontiguousarray(tensor), info


# ============================================================================
# Factory
# ============================================================================

def make_face_loader(
    hailo_enabled: bool,
    hef_path: Optional[str],
    stub_period_sec: float,
    confidence_threshold: float = 0.6,
    nms_iou_threshold: float = 0.45,
) -> HEFLoader:
    """Вернуть правильный face loader по launch-параметрам.

    Stub-режим переиспользует ``StubHEFLoader``: он публикует
    ``event_type="stub"`` (а НЕ "face") — выдуманное событие обязано
    быть помечено (ADR-0089 §2.2, #2583), иначе Личность начнёт
    здороваться с несуществующими людьми.
    """
    if hailo_enabled and hef_path:
        return RetinaFaceLoader(
            hef_path=hef_path,
            confidence_threshold=confidence_threshold,
            nms_iou_threshold=nms_iou_threshold,
        )
    return StubHEFLoader(period_sec=stub_period_sec)
