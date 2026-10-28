"""Батарея из ros2_control: разбор DynamicJointState и поля BatteryState (#4001, #4002).

Чистые функции без ROS-импортов (тестируются обычным ``pytest``): узлы
(``context_aggregator_node``, ``battery_state_node``) только достают сообщение
и публикуют результат.

Откуда напряжение. ``vesc_nexus`` отдаёт батарею в ros2_control как joint
``battery`` с state-интерфейсом ``voltage`` (вольты шины, 10S Li-ion). В
``control_msgs/DynamicJointState`` имя joint лежит в ``joint_names[i]``, а
``interface_values[i].interface_names`` содержит только имя интерфейса
(``voltage``), поэтому строки ``battery/voltage`` там не бывает.

Шкала и кривая. ``sensor_msgs/BatteryState.percentage`` по конвенции ROS
лежит в 0..1. Кривая «вольты -> заряд» и пороги живут в ОДНОМ месте,
``rob_box_core.battery`` (его же использует ``rob_box_mcp_tools``); здесь
только разбор сообщения, сглаживание и сборка полей.

Сглаживание. Под нагрузкой (езда) напряжение проседает на вольты, а процент
по кривой прыгает. Публикуемый процент считается от экспоненциально
сглаженного напряжения (постоянная времени ``DEFAULT_SMOOTHING_TAU_S``);
первая выборка принимается без задержки.

«Нет данных» - не «полная батарея»: нет напряжения (нет joint, ``<= 0``, NaN,
устарело) -> ``present=False`` и ``NaN`` в числовых полях.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

from rob_box_core.battery import valid_voltage, voltage_to_fraction

#: joint и интерфейс батареи в ros2_control (vesc_nexus).
BATTERY_JOINT = 'battery'
BATTERY_INTERFACE = 'voltage'

#: Через сколько секунд без свежего напряжения считаем данные устаревшими.
DEFAULT_STALE_S = 5.0

#: Постоянная времени EMA напряжения, секунды (0 - сглаживание выключено).
DEFAULT_SMOOTHING_TAU_S = 45.0

_NAN = float('nan')


class BatteryFields(NamedTuple):
    """Что публиковать в ``sensor_msgs/BatteryState`` (NaN = неизвестно)."""

    present: bool
    voltage: float
    percentage: float


#: Нет данных: батарея не «present», числа неизвестны.
NO_BATTERY = BatteryFields(present=False, voltage=_NAN, percentage=_NAN)


def extract_battery_voltage(joint_names, interface_values) -> Optional[float]:
    """Напряжение из полей ``DynamicJointState`` (``None`` - батареи нет).

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


def ema_step(previous: Optional[float], sample: float, dt_s: float, tau_s: float) -> float:
    """Один шаг EMA с постоянной времени ``tau_s`` (не зависит от частоты выборок).

    ``previous is None`` (первая выборка) или ``tau_s <= 0`` - выборка как есть,
    без задержки. ``dt_s <= 0`` (повторная выборка в тот же момент) - без изменений.
    """
    if previous is None or tau_s <= 0.0:
        return sample
    if dt_s <= 0.0:
        return previous
    alpha = 1.0 - math.exp(-dt_s / tau_s)
    return previous + alpha * (sample - previous)


class VoltageSmoother:
    """Сглаживатель напряжения: ``update(volts, now_s)`` -> сглаженные вольты."""

    def __init__(self, tau_s: float = DEFAULT_SMOOTHING_TAU_S):
        """Создать сглаживатель с постоянной времени ``tau_s`` секунд."""
        self.tau_s = float(tau_s)
        self.value: Optional[float] = None
        self._at: Optional[float] = None

    def update(self, volts: float, now_s: float) -> float:
        """Принять выборку в момент ``now_s`` и вернуть сглаженное значение."""
        dt = 0.0 if self._at is None else now_s - self._at
        self.value = ema_step(self.value, volts, dt, self.tau_s)
        self._at = now_s
        return self.value

    def reset(self) -> None:
        """Забыть историю (данные устарели): следующая выборка - без задержки."""
        self.value = None
        self._at = None


def battery_fields(volts, age_s: Optional[float], stale_s: float = DEFAULT_STALE_S,
                   smoothed=None) -> BatteryFields:
    """Поля BatteryState по последнему напряжению и его возрасту в секундах.

    ``voltage`` - последнее измеренное; ``percentage`` - от ``smoothed``
    (если передано и осмысленно), иначе от ``volts``.
    """
    volts = valid_voltage(volts)
    if volts is None or age_s is None or age_s > stale_s:
        return NO_BATTERY
    basis = valid_voltage(smoothed)
    return BatteryFields(True, volts, voltage_to_fraction(volts if basis is None else basis))
