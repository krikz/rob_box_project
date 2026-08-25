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

    # Phase 1.6: подписки на BatteryState + diagnostic_msgs.
    battery_pct: Optional[int] = None
    wifi_rssi: Optional[int] = None

    def update_velocity(self, linear: float, angular: float) -> None:
        self.vel_linear = float(linear)
        self.vel_angular = float(angular)

    def set_mode(self, mode: str) -> None:
        self.mode = str(mode)

    def payload(self) -> bytes:
        """Сформировать msgpack-payload. Неопциональные поля заполняются
        значениями по умолчанию если source ещё не подключен."""
        return encode_robot_status(
            battery_pct=self.battery_pct if self.battery_pct is not None else -1,
            wifi_rssi=self.wifi_rssi if self.wifi_rssi is not None else 0,
            mode=self.mode,
            vel_linear=self.vel_linear,
            vel_angular=self.vel_angular,
            ts_ms=int(time.time() * 1000),
        )
