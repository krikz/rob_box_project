"""Заряд батареи робота по напряжению шины: единственное место знания (#4001, #4002).

Потребители импортируют отсюда, а не держат свои копии (ADR-0148, «знание живёт
в одном модуле»): ``rob_box_perception.battery_state`` (топик ``/battery_state``)
и ``rob_box_mcp_tools.perception_projection`` (``battery_percent`` в контексте
Личности). Модуль ROS-free, только stdlib.

Железо (этикетка, фото от товарища Шифу, 28.10.2026): две гироскутерные
батареи smartONE «Lithium-ion Power Battery», Model 10S2P, 36 V nominal,
Limited Charging Voltage 42 V, соединены ПАРАЛЛЕЛЬНО, то есть 10S4P. Химия
Li-ion (ячейки 18650), НЕ LiPo. Параллель напряжение не меняет: шина 10S,
32..42 В (3.2..4.2 В на банку). Замер на роботе 24.10.2026: 41.0 В.

Кривая. Напряжение на банку -> процент заряда, кусочно-линейная интерполяция
по таблице ``OCV_TABLE``. ВНИМАНИЕ: таблица ориентировочная (типичная кривая
напряжения разомкнутой цепи NMC 18650), на НАШЕЙ батарее она не замерялась.
Под нагрузкой напряжение проседает, поэтому процент в движении занижен;
сглаживание провалов делает публикатор (``rob_box_perception.battery_state``).
Число банок (``BATTERY_CELLS = 10``) - константа модуля. Порог отсечки VESC
в репозитории неизвестен, 0 % здесь означает 3.2 В на банку, а не отсечку VESC.

Контракт ``voltage_to_percent``: ``volts`` пусто / NaN / ``<= 0`` -> ``None``
(«нет данных», это НЕ «разряжена»); вне диапазона таблицы - зажим 0..100.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

#: Банок последовательно в шине (10S; параллельные ряды 2P x 2 напряжение не меняют).
BATTERY_CELLS: int = 10

#: (вольт на банку, процент). По возрастанию напряжения; ориентир, не замер.
OCV_TABLE: Tuple[Tuple[float, float], ...] = (
    (3.20, 0.0),
    (3.30, 2.0),
    (3.45, 5.0),
    (3.55, 10.0),
    (3.60, 15.0),
    (3.63, 20.0),
    (3.67, 30.0),
    (3.70, 35.0),
    (3.74, 40.0),
    (3.79, 50.0),
    (3.85, 60.0),
    (3.92, 70.0),
    (4.00, 80.0),
    (4.10, 90.0),
    (4.20, 100.0),
)

#: Напряжение шины при 100 % (4.2 В x 10 = 42 В) и при 0 % (3.2 В x 10 = 32 В).
BATTERY_VOLTAGE_FULL: float = OCV_TABLE[-1][0] * BATTERY_CELLS
BATTERY_VOLTAGE_EMPTY: float = OCV_TABLE[0][0] * BATTERY_CELLS


def valid_voltage(value) -> Optional[float]:
    """Вольты -> float либо ``None``, если это не осмысленное напряжение."""
    try:
        volts = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(volts) or math.isinf(volts) or volts <= 0.0:
        return None
    return volts


def cell_voltage_to_percent(cell_volts: float) -> float:
    """Вольты на банку -> проценты 0..100 по ``OCV_TABLE`` (зажим по краям)."""
    if cell_volts <= OCV_TABLE[0][0]:
        return OCV_TABLE[0][1]
    if cell_volts >= OCV_TABLE[-1][0]:
        return OCV_TABLE[-1][1]
    for (v0, p0), (v1, p1) in zip(OCV_TABLE, OCV_TABLE[1:]):
        if cell_volts <= v1:
            return p0 + (p1 - p0) * (cell_volts - v0) / (v1 - v0)
    return OCV_TABLE[-1][1]  # недостижимо: нижние/верхние границы отсечены выше


def voltage_to_percent(volts) -> Optional[float]:
    """Вольты шины -> проценты 0..100; ``None`` - нет данных (см. контракт модуля)."""
    volts = valid_voltage(volts)
    if volts is None:
        return None
    return cell_voltage_to_percent(volts / BATTERY_CELLS)


def voltage_to_fraction(volts) -> Optional[float]:
    """Вольты шины -> доля заряда 0..1 (ROS-конвенция ``BatteryState.percentage``)."""
    percent = voltage_to_percent(volts)
    return None if percent is None else percent / 100.0
