"""tf_lookup — поиск трансформа tf2 с честными ошибками: единственное место знания.

Вынесено из ``rob_box_perception.observation_frames`` (PR #4304, ADR-0130 §7
этап 1), чтобы второй потребитель — поза робота на карте для точек
навигации в ``rob_box_mcp_tools`` (#4390) — брал тот же адаптер, а не
держал свою копию (ADR-0148: знание в одном модуле). ``observation_frames``
реэкспортирует имена отсюда.

Модуль без rclpy и tf2 на уровне импорта: :class:`Tf2Lookup` —
адаптер к ``tf2_ros.Buffer``, импорты ROS — лениво в конструкторе, тесты
подставляют фейковый буфер и дублёры исключений.

Два режима поиска:

* ``__call__(target, source, stamp_sec, timeout_sec)`` — на момент КАДРА
  (наблюдения восприятия). Сначала ``can_transform`` на latest без
  ожидания: цепочки нет вовсе — :class:`NoTransform` сразу, кадр не
  задерживается. Цепочка есть — ``lookup_transform`` на stamp с таймаутом.
* :meth:`Tf2Lookup.latest` — последний трансформ с ожиданием до таймаута
  (поза робота «сейчас»). Буфер может быть только что создан и пуст, поэтому
  здесь ждём, а не отказываем по пустому дереву.

Ошибки: в дереве нет цепочки target ← source → :class:`NoTransform`;
цепочка есть, данных на нужный момент нет → :class:`TransformStale`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence, Tuple

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
        qx, qy, qz, qw = self._unit_rotation()
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

    @property
    def yaw(self) -> float:
        """Курс (поворот вокруг z) трансформа, рад, в (-π, π]."""
        qx, qy, qz, qw = self._unit_rotation()
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _unit_rotation(self) -> Quat:
        qx, qy, qz, qw = self.rotation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise ValueError('нулевой кватернион в трансформе')
        return qx / norm, qy / norm, qz / norm, qw / norm


#: Протокол поиска: (target, source, stamp_sec, timeout_sec) → трансформ;
#: бросает NoTransform / TransformStale.
LookupFn = Callable[[str, str, float, float], RigidTransform]


class Tf2Lookup:
    """Адаптер ``tf2_ros.Buffer`` → :data:`LookupFn` и :meth:`latest`.

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
        """Трансформ на момент ``stamp_sec``; цепочки нет — отказ без ожидания."""
        latest = self._to_time(0.0)
        if not self._buffer.can_transform(target, source, latest, self._to_duration(0.0)):
            raise NoTransform(f'{target} <- {source}')
        return self._lookup(target, source, stamp_sec, timeout_sec)

    def latest(self, target: str, source: str, timeout_sec: float) -> RigidTransform:
        """Последний трансформ, ждать данных до ``timeout_sec`` (буфер может быть пуст)."""
        return self._lookup(target, source, 0.0, timeout_sec)

    def _lookup(
        self, target: str, source: str, stamp_sec: float, timeout_sec: float,
    ) -> RigidTransform:
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
