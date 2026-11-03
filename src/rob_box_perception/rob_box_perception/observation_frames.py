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
чистая логика над протоколом «найди трансформ», :class:`Tf2Lookup` —
адаптер к ``tf2_ros.Buffer``, импорты ROS — лениво в конструкторе.

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
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

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

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


class NoTransform(Exception):
    """В дереве TF нет цепочки target ← source."""


class TransformStale(Exception):
    """Цепочка есть, но данных на нужный момент нет."""


@dataclass(frozen=True)
class RigidTransform:
    """Трансформ target ← source: p_target = R(rotation)·p_source + translation."""

    translation: Vec3
    rotation: Quat

    def apply(self, point: Sequence[float]) -> Vec3:
        """Перевести точку source → target (кватернион нормируется)."""
        qx, qy, qz, qw = self.rotation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise ValueError('нулевой кватернион в трансформе')
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        px, py, pz = (float(v) for v in point)
        # v' = v + 2w(q×v) + 2 q×(q×v)
        cx = qy * pz - qz * py
        cy = qz * px - qx * pz
        cz = qx * py - qy * px
        ccx = qy * cz - qz * cy
        ccy = qz * cx - qx * cz
        ccz = qx * cy - qy * cx
        tx, ty, tz = self.translation
        return (
            px + 2.0 * (qw * cx + ccx) + tx,
            py + 2.0 * (qw * cy + ccy) + ty,
            pz + 2.0 * (qw * cz + ccz) + tz,
        )


#: Протокол поиска: (target, source, stamp_sec, timeout_sec) → трансформ;
#: бросает NoTransform / TransformStale.
LookupFn = Callable[[str, str, float, float], RigidTransform]


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


class Tf2Lookup:
    """Адаптер ``tf2_ros.Buffer`` → :data:`LookupFn`.

    ``exceptions`` — (lookup, connectivity, extrapolation) классы tf2;
    ``to_time`` / ``to_duration`` — float сек → rclpy Time / Duration.
    По умолчанию берутся из tf2_ros / rclpy (лениво), тесты подставляют
    дублёры вместе с фейковым буфером.
    """

    def __init__(
        self,
        buffer: Any,
        *,
        exceptions: Optional[Tuple[type, type, type]] = None,
        to_time: Optional[Callable[[float], Any]] = None,
        to_duration: Optional[Callable[[float], Any]] = None,
    ) -> None:
        self._buffer = buffer
        if exceptions is None:
            import tf2_ros  # noqa: PLC0415 — лениво: модуль без ROS в CI
            exceptions = (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            )
        if to_time is None or to_duration is None:
            from rclpy.duration import Duration  # noqa: PLC0415
            from rclpy.time import Time  # noqa: PLC0415
            to_time = to_time or (lambda s: Time(nanoseconds=int(round(s * 1e9))))
            to_duration = to_duration or (
                lambda s: Duration(nanoseconds=int(round(s * 1e9)))
            )
        self._lookup_exc, self._connectivity_exc, self._extrapolation_exc = exceptions
        self._to_time = to_time
        self._to_duration = to_duration

    def __call__(
        self, target: str, source: str, stamp_sec: float, timeout_sec: float,
    ) -> RigidTransform:
        latest = self._to_time(0.0)
        if not self._buffer.can_transform(target, source, latest, self._to_duration(0.0)):
            raise NoTransform(f'{target} <- {source}')
        try:
            stamped = self._buffer.lookup_transform(
                target, source, self._to_time(stamp_sec), self._to_duration(timeout_sec),
            )
        except self._extrapolation_exc as exc:
            raise TransformStale(str(exc)) from exc
        except (self._lookup_exc, self._connectivity_exc) as exc:
            raise NoTransform(str(exc)) from exc
        t = stamped.transform.translation
        r = stamped.transform.rotation
        return RigidTransform(
            translation=(float(t.x), float(t.y), float(t.z)),
            rotation=(float(r.x), float(r.y), float(r.z), float(r.w)),
        )
