"""robot_status aggregator (1 Hz): собирает battery/wifi/vel → msgpack payload.

Источник истины: docs/architecture/meta-quest-api.md §4 (robot_status 0x1201).

Phase 1.4: только odom-velocity + фиксированный mode. Батарея/Wi-Fi —
Phase 1.6 (источники: BatteryState + diagnostics msgs).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from ..protocol.topics import encode_robot_status


@dataclass
class StatusAggregator:
    """Хранит свежие данные для формирования robot_status payload.

    Обновляется из ROS-подписок (см. quest_node.py).
    Периодический tick (1 Hz) → publish encode_robot_status(...).
    """

    # Из /odom (nav_msgs/Odometry): линейная и угловая скорость.
    vel_linear: float = 0.0
    vel_angular: float = 0.0

    # Mode — выставляется quest_node из FSM (Phase 1: "teleop_active"|"idle"|"emergency").
    mode: str = "idle"

    # Wave 3.A: battery — из JSON-снапшота (/device/snapshot, /sensors/data)
    # или из VESC (voltage_input); wifi — из /proc/net/wireless на Vision Pi.
    battery_pct: Optional[int] = None
    battery_v: Optional[float] = None
    wifi_rssi: Optional[int] = None

    def update_velocity(self, linear: float, angular: float) -> None:
        self.vel_linear = float(linear)
        self.vel_angular = float(angular)

    def set_mode(self, mode: str) -> None:
        self.mode = str(mode)

    def update_battery(self, pct: Optional[float] = None, volts: Optional[float] = None) -> None:
        """Обновить заряд. ``None`` не затирает уже известное значение —
        источники приходят вразнобой (проценты из BMS, вольты из VESC)."""
        if pct is not None:
            self.battery_pct = int(round(float(pct)))
        if volts is not None:
            self.battery_v = float(volts)

    def update_wifi(self, rssi: Optional[int]) -> None:
        """Обновить RSSI. ``None`` — источник недоступен (нет /proc/net/wireless)."""
        self.wifi_rssi = int(rssi) if rssi is not None else None

    def payload(self) -> bytes:
        """Сформировать msgpack-payload. Неопциональные поля заполняются
        значениями по умолчанию если source ещё не подключен."""
        return encode_robot_status(
            battery_pct=self.battery_pct if self.battery_pct is not None else -1,
            battery_v=self.battery_v,
            wifi_rssi=self.wifi_rssi if self.wifi_rssi is not None else 0,
            mode=self.mode,
            vel_linear=self.vel_linear,
            vel_angular=self.vel_angular,
            ts_ms=int(time.time() * 1000),
        )
