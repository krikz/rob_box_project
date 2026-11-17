"""Фильтр Калмана с постоянной скоростью на плоскости (ADR-0130 §2.2, профиль ``person``).

Оси x и y независимы: состояние оси — ``[положение, скорость]``,
ковариация 2×2, шум процесса — белое ускорение со спектральной
плотностью ``q``. Без numpy: на десяток треков при 10 Гц хватает
скаляров, и модуль живёт в любом контейнере.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class _Axis:
    """Одна ось: положение, скорость и ковариация 2×2."""

    p: float
    v: float
    pp: float
    pv: float
    vv: float

    def predict(self, dt: float, q: float) -> None:
        """Шаг модели постоянной скорости с белым шумом ускорения."""
        self.p += self.v * dt
        self.pp += 2.0 * dt * self.pv + dt * dt * self.vv + q * dt ** 3 / 3.0
        self.pv += dt * self.vv + q * dt * dt / 2.0
        self.vv += q * dt

    def update(self, z: float, r: float) -> None:
        """Учесть измерение положения ``z`` с дисперсией ``r``."""
        s = self.pp + r
        kp = self.pp / s
        kv = self.pv / s
        innovation = z - self.p
        self.p += kp * innovation
        self.v += kv * innovation
        pp, pv = self.pp, self.pv
        self.pp = (1.0 - kp) * pp
        self.pv = (1.0 - kp) * pv
        self.vv -= kv * pv


class ConstantVelocity2D:
    """Положение и скорость объекта на плоскости пола."""

    def __init__(self, x: float, y: float, pos_var: float,
                 vel_var: float) -> None:
        """Начать с точки (x, y), скорость 0 с дисперсией ``vel_var``."""
        self._x = _Axis(x, 0.0, pos_var, 0.0, vel_var)
        self._y = _Axis(y, 0.0, pos_var, 0.0, vel_var)

    def predict(self, dt: float, accel_noise: float) -> None:
        """Продвинуть состояние на ``dt`` секунд (``dt <= 0`` — без изменений)."""
        if dt <= 0.0:
            return
        self._x.predict(dt, accel_noise)
        self._y.predict(dt, accel_noise)

    def update(self, x: float, y: float, meas_var: float) -> None:
        """Учесть измерение положения с дисперсией ``meas_var``."""
        self._x.update(x, meas_var)
        self._y.update(y, meas_var)

    def stop(self, vel_var: float) -> None:
        """Обнулить скорость (объект пропал — экстраполировать дальше нельзя)."""
        for axis in (self._x, self._y):
            axis.v = 0.0
            axis.pv = 0.0
            axis.vv = vel_var

    @property
    def position(self) -> Tuple[float, float]:
        """Текущая оценка положения (x, y), м."""
        return self._x.p, self._y.p

    @property
    def velocity(self) -> Tuple[float, float]:
        """Текущая оценка скорости (vx, vy), м/с."""
        return self._x.v, self._y.v

    @property
    def position_variance(self) -> Tuple[float, float]:
        """Дисперсии положения по осям (σx², σy²), м²."""
        return self._x.pp, self._y.pp
