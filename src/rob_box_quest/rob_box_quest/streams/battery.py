"""Разбор батарейной телеметрии для robot_status (Wave 3.A / R8).

Источника «процентов заряда» в проекте сегодня нет: подписка
``/device/snapshot`` в ``context_aggregator_node`` закомментирована
(ADR-0010 §4 — firmware сенсор-борда не готов), а ``perception_bridge``
публикует сырой JSON от MCU на ``/sensors/data``. Единственное, что
реально живо на роботе — напряжение с VESC
(``/sensors/motor_state/<label>``, поле ``voltage_input``).

Поэтому:

- ``parse_battery_json`` разбирает JSON-снапшот (std_msgs/String) и
  достаёт проценты и/или вольты по нескольким возможным именам полей —
  чтобы заработать сразу, как только источник появится, без правок кода;
- ``voltage_to_pct`` конвертирует вольты в проценты ТОЛЬКО если заданы
  границы (ROS-параметры). Без границ — не выдумываем цифру, отдаём
  ``None``, а HUD показывает вольты.

Чистая логика без ROS — тестируется напрямую.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional, Tuple

# Имена полей, под которыми разные источники отдают заряд.
_PCT_KEYS = ("battery_pct", "battery_percentage", "battery_percent", "soc_pct")
_VOLT_KEYS = ("battery_v", "battery_voltage", "voltage_input", "battery")


def _as_float(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            return None
    return None


def parse_battery_json(data: Mapping[str, Any]) -> Tuple[Optional[float], Optional[float]]:
    """Достать ``(battery_pct, battery_v)`` из JSON-снапшота.

    Оба значения опциональны — источник может отдавать только вольты
    (VESC) или только проценты (умный BMS).

    ``battery`` трактуется как вольты: так его кладёт
    ``context_aggregator_node.on_device_snapshot`` (``msg.battery_voltage``).
    """
    pct: Optional[float] = None
    volts: Optional[float] = None
    for key in _PCT_KEYS:
        if key in data:
            pct = _as_float(data[key])
            if pct is not None:
                break
    for key in _VOLT_KEYS:
        if key in data:
            volts = _as_float(data[key])
            if volts is not None:
                break
    return pct, volts


def voltage_to_pct(
    volts: Optional[float],
    v_empty: Optional[float],
    v_full: Optional[float],
) -> Optional[int]:
    """Линейно перевести вольты в проценты по границам пакета.

    Границы задаются ROS-параметрами ``battery_voltage_empty`` /
    ``battery_voltage_full`` — они зависят от химии и числа банок, и
    угадывать их нельзя. Пока границы не заданы (``<= 0``) — возвращаем
    ``None``, и HUD показывает вольты вместо выдуманных процентов.

    Линейная модель напряжение→заряд грубая (у Li-ion полка в середине),
    но для «сколько осталось кататься» её хватает; точный SoC — задача
    BMS, а не телеметрии мостика.
    """
    if volts is None or not v_empty or not v_full:
        return None
    if v_full <= v_empty:
        return None
    ratio = (volts - v_empty) / (v_full - v_empty)
    return int(round(max(0.0, min(1.0, ratio)) * 100))
