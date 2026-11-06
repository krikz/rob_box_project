"""``Observation`` (ADR-0138, этап 1) → :class:`Detection` трекера.

Без импорта ROS: сообщение читается по полям (утиная типизация), так что
модуль проверяется на ``SimpleNamespace`` и годится узлу как есть.

Трекер ведёт треки в одном фрейме (``map`` или ``base_link``,
:data:`FRAMES`; какой — решает :class:`~.frame_policy.FramePolicy`,
ADR-0161 §1). Наблюдение, у которого точка в этом фрейме не ``ok``
(``observation_frames.FRAME_STATUS_*``), трекеру не отдаётся, а причина
возвращается вызывающему — счётчик по причинам ведёт узел
(ADR-0130 §5.1 п.7: отказ канала виден).
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple

from rob_box_perception.tracking.tracker import Detection

#: Фрейм → (поле точки, поле статуса) в Observation.msg.
FRAMES = {
    'map': ('position_map', 'map_status'),
    'base_link': ('position_base_link', 'base_link_status'),
}

#: Статус «точка в нужном фрейме есть» (= observation_frames.FRAME_STATUS_OK).
_OK = 'ok'


def stamp_seconds(msg: Any) -> float:
    """Момент наблюдения ``header.stamp`` в секундах (ADR-0130 §2.5)."""
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def to_detection(msg: Any, frame: str) -> Tuple[Optional[Detection], str]:
    """Перевести наблюдение; ``(Detection, 'ok')`` или ``(None, причина)``.

    Причина — статус точки в фрейме из Observation (``no_transform``,
    ``transform_stale``, ...), либо ``'non_finite'``, если статус ``ok``,
    а координаты не конечны (нарушение контракта источника).
    """
    if frame not in FRAMES:
        raise ValueError(f'неизвестный фрейм трекера: {frame!r}')
    point_field, status_field = FRAMES[frame]
    status = getattr(msg, status_field)
    if status != _OK:
        return None, status
    point = getattr(msg, point_field)
    if not (math.isfinite(point.x) and math.isfinite(point.y)):
        return None, 'non_finite'
    det = Detection(
        class_name=msg.class_name,
        x=float(point.x),
        y=float(point.y),
        confidence=float(msg.confidence),
        position_std_m=float(msg.position_stddev_m),
        candidate_id=msg.candidate_id,
        source=msg.source,
    )
    return det, _OK
