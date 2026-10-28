"""Батарея из ros2_control: разбор DynamicJointState и поля BatteryState (#4001, #4002).

Чистые функции без ROS-импортов (тестируются обычным ``pytest``): узлы
(``context_aggregator_node``, ``battery_state_node``) только достают сообщение
и публикуют результат.

Откуда напряжение. ``vesc_nexus`` отдаёт батарею в ros2_control как joint
``battery`` с state-интерфейсом ``voltage`` (вольты шины, 10S LiPo). В
``control_msgs/DynamicJointState`` имя joint лежит в ``joint_names[i]``, а
``interface_values[i].interface_names`` содержит только имя интерфейса
(``voltage``), поэтому строки ``battery/voltage`` там не бывает.

Шкала процентов. ``sensor_msgs/BatteryState.percentage`` по конвенции ROS
лежит в 0..1 (потребители: ``face_node`` ``battery_percent_scale: 100``,
``GetRobotStatusTool`` умножает на 100). Формула вольты → доля заряда
повторяет ``rob_box_mcp_tools.perception_projection.voltage_to_percent``
(контейнер perception не содержит rob_box_mcp_tools, импортировать её оттуда
нельзя); совпадение констант и формулы держит тест паритета
``test/unit/test_battery_state.py``.

«Нет данных» — не «полная батарея»: нет напряжения (нет joint, ``<= 0``, NaN,
устарело) → ``present=False`` и ``NaN`` в числовых полях, а не выдуманные 0
или 1.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

#: joint и интерфейс батареи в ros2_control (vesc_nexus).
BATTERY_JOINT = 'battery'
BATTERY_INTERFACE = 'voltage'

#: Границы 10S LiPo (docs/architecture/HARDWARE.md): 10 x 4.2 В и 10 x 3.0 В.
BATTERY_VOLTAGE_FULL = 42.0
BATTERY_VOLTAGE_EMPTY = 30.0

#: Через сколько секунд без свежего напряжения считаем данные устаревшими.
DEFAULT_STALE_S = 5.0

_NAN = float('nan')


class BatteryFields(NamedTuple):
    """Что публиковать в ``sensor_msgs/BatteryState`` (NaN = неизвестно)."""

    present: bool
    voltage: float
    percentage: float


#: Нет данных: батарея не «present», числа неизвестны.
NO_BATTERY = BatteryFields(present=False, voltage=_NAN, percentage=_NAN)


def valid_voltage(value) -> Optional[float]:
    """Вольты → float либо ``None``, если это не осмысленное напряжение."""
    try:
        volts = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(volts) or math.isinf(volts) or volts <= 0.0:
        return None
    return volts


def extract_battery_voltage(joint_names, interface_values) -> Optional[float]:
    """Напряжение из полей ``DynamicJointState`` (``None`` — батареи нет).

    ``joint_names[i]`` соответствует ``interface_values[i]``; порядок joints
    произвольный, ищем по имени.
    """
    for joint, iv in zip(joint_names, interface_values):
        if joint != BATTERY_JOINT:
            continue
        for name, value in zip(iv.interface_names, iv.values):
            if name == BATTERY_INTERFACE:
                return valid_voltage(value)
    return None


def voltage_to_fraction(volts) -> Optional[float]:
    """Вольты → доля заряда 0..1 (ROS-конвенция); ``None`` — нет данных."""
    volts = valid_voltage(volts)
    if volts is None:
        return None
    span = BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY
    return max(0.0, min(1.0, (volts - BATTERY_VOLTAGE_EMPTY) / span))


def battery_fields(volts, age_s: Optional[float],
                   stale_s: float = DEFAULT_STALE_S) -> BatteryFields:
    """Поля BatteryState по последнему напряжению и его возрасту в секундах."""
    volts = valid_voltage(volts)
    if volts is None or age_s is None or age_s > stale_s:
        return NO_BATTERY
    return BatteryFields(True, volts, voltage_to_fraction(volts))
