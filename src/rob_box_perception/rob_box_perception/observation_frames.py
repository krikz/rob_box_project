"""observation_frames — точка наблюдения из optical frame в base_link / map (ADR-0130 §7 этап 1).

ADR-0138 довёл ``Observation`` до 3D-точки в optical frame камеры
(``header.frame_id``). Здесь та же точка переводится через tf2 в фреймы
робота: ``base_link`` и ``map``. Для каждого фрейма — своя точка и свой
честный статус (ADR-0018 capability-honest): нет трансформа — точка NaN
и причина, а не нули и не «последняя известная поза».

Замер 03.11.2026 (Vision Pi, одноразовый контейнер из образа
vision-hailo, тот же zenoh-конфиг): ``tf2_echo base_link
camera_color_optical_frame`` отдаёт статический трансформ от
``robot_state_publisher`` Main Pi, ``tf2_echo map base_link`` —
динамический от ``rtabmap``. Межпайный TF через Zenoh виден (ADR-0130
§8 п.5).

Модуль без rclpy и tf2 на уровне импорта: :class:`FrameResolver` —
чистая логика над протоколом «найди трансформ». Адаптер к
``tf2_ros.Buffer`` (:class:`Tf2Lookup`), :class:`RigidTransform` и ошибки
поиска живут в ``rob_box_core.tf_lookup`` (второй потребитель — поза робота
на карте для точек навигации, #4390) и реэкспортируются отсюда.

Политика ожидания. Сначала ``can_transform`` на последний момент без
ожидания: если цепочки ``target ← source`` в дереве нет вовсе (нет
``map``, потому что rtabmap лежит), ждать бессмысленно —
``no_transform`` сразу, кадр не задерживается. Если цепочка есть —
``lookup_transform`` на ``stamp`` КАДРА с таймаутом: данные TF через
Zenoh могут опоздать на десятки мс. Не дождались — ``transform_stale``.
Трансформ на другой момент (latest) вместо момента кадра не
подставляется: для ``map`` это сдвиг на ход робота.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence, Tuple

from rob_box_core.tf_lookup import (
    LookupFn,
    NoTransform,
    Quat,
    RigidTransform,
    Tf2Lookup,
    TransformStale,
    Vec3,
)

__all__ = [
    'LookupFn', 'NoTransform', 'Quat', 'RigidTransform', 'Tf2Lookup',
    'TransformStale', 'Vec3', 'FrameResolver',
]

# ============================================================================
# Словарь Observation.{base_link,map}_status
# ============================================================================

#: Точка переведена.
FRAME_STATUS_OK = 'ok'
#: Исходной точки нет (position_valid=false) — переводить нечего.
FRAME_STATUS_NO_POSITION = 'no_position'
#: Перевод выключен параметром ноды или tf2_ros не импортируется.
FRAME_STATUS_TF_DISABLED = 'tf_disabled'
#: В дереве TF нет цепочки target ← source (фрейма нет, дерево разорвано).
FRAME_STATUS_NO_TRANSFORM = 'no_transform'
#: Цепочка есть, но на момент кадра данных TF нет (не дождались за
#: таймаут или момент вне буфера).
FRAME_STATUS_TRANSFORM_STALE = 'transform_stale'

FRAME_STATUSES = (
    FRAME_STATUS_OK, FRAME_STATUS_NO_POSITION, FRAME_STATUS_TF_DISABLED,
    FRAME_STATUS_NO_TRANSFORM, FRAME_STATUS_TRANSFORM_STALE,
)

BASE_FRAME = 'base_link'
MAP_FRAME = 'map'

#: Поля Observation.msg, которые заполняет этот модуль (порядок — как в IDL).
FRAME_FIELDS = (
    'position_base_link', 'base_link_status',
    'position_map', 'map_status',
)

#: Сколько ждать данные TF на момент кадра. Задержка TF через Zenoh
#: между Pi не измерена (ADR-0130 §8 п.5) — значение с запасом на
#: опоздание, но меньше периода кадра.
DEFAULT_TF_TIMEOUT_SEC = 0.05

_NAN_POINT = (math.nan, math.nan, math.nan)


class FrameResolver:
    """Точка наблюдения → поля ``FRAME_FIELDS`` Observation.msg.

    ``lookup=None`` — перевод выключен: все фреймы ``tf_disabled``.
    Трансформы кэшируются на (source, stamp, target): детекции одного
    кадра ждут TF один раз, а не каждая.
    """

    def __init__(
        self,
        lookup: Optional[LookupFn],
        timeout_sec: float = DEFAULT_TF_TIMEOUT_SEC,
    ) -> None:
        self._lookup = lookup
        self._timeout_sec = max(0.0, float(timeout_sec))
        self._cache_key: Optional[Tuple[str, float]] = None
        self._cache: Dict[str, Tuple[Optional[RigidTransform], str]] = {}

    @property
    def enabled(self) -> bool:
        return self._lookup is not None

    def resolve(
        self,
        source_frame: str,
        stamp_sec: float,
        position: Sequence[float],
        position_valid: bool,
    ) -> Dict[str, Any]:
        """Поля FRAME_FIELDS для одной детекции кадра (source_frame, stamp_sec)."""
        out: Dict[str, Any] = {}
        for target, point_field, status_field in (
            (BASE_FRAME, 'position_base_link', 'base_link_status'),
            (MAP_FRAME, 'position_map', 'map_status'),
        ):
            point, status = self._point_in(
                target, source_frame, stamp_sec, position, position_valid,
            )
            out[point_field] = point
            out[status_field] = status
        return out

    def _point_in(
        self, target: str, source: str, stamp_sec: float,
        position: Sequence[float], position_valid: bool,
    ) -> Tuple[Vec3, str]:
        if self._lookup is None:
            return _NAN_POINT, FRAME_STATUS_TF_DISABLED
        if not position_valid:
            return _NAN_POINT, FRAME_STATUS_NO_POSITION
        transform, status = self._transform(target, source, stamp_sec)
        if transform is None:
            return _NAN_POINT, status
        return transform.apply(position), FRAME_STATUS_OK

    def _transform(
        self, target: str, source: str, stamp_sec: float,
    ) -> Tuple[Optional[RigidTransform], str]:
        key = (source, float(stamp_sec))
        if key != self._cache_key:
            self._cache_key = key
            self._cache = {}
        if target not in self._cache:
            self._cache[target] = self._lookup_once(target, source, stamp_sec)
        return self._cache[target]

    def _lookup_once(
        self, target: str, source: str, stamp_sec: float,
    ) -> Tuple[Optional[RigidTransform], str]:
        assert self._lookup is not None
        try:
            return self._lookup(target, source, stamp_sec, self._timeout_sec), FRAME_STATUS_OK
        except NoTransform:
            return None, FRAME_STATUS_NO_TRANSFORM
        except TransformStale:
            return None, FRAME_STATUS_TRANSFORM_STALE
