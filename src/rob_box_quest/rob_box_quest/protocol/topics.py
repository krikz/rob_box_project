"""Topic registry + payload-кодеки rob_box_quest.

Источник истины: docs/architecture/meta-quest-api.md §4.

Этот модуль — чистая логика без зависимостей от ROS/Zenoh. Все форматы —
ровно те, что в meta-quest-api.md. Phase 2/3 расширят payload'ы
(person_detections, lidar_3d) без поломки существующих контрактов.
"""

from __future__ import annotations

import struct
from typing import Sequence

import msgpack


# --- Topic ID registry (meta-quest-api.md §4) -----------------------------
TOPIC_IDS: dict[str, int] = {
    # Server-initiated streams (0x1000..0xFFFF).
    "camera_rear": 0x1001,
    "camera_front": 0x1002,
    "lidar_2d": 0x1101,
    "lidar_3d": 0x1102,
    "robot_status": 0x1201,
    "voice_state": 0x1202,
    "person_detections": 0x1301,
}


# --- lidar_2d payload (meta-quest-api.md §4) -----------------------------
# Формат: little-endian float32
#   [angle_min, angle_max, angle_inc, range_min, range_max,
#    time_increment, scan_time, n_points]
# + n_points × float32 ranges
# + n_points × float32 intensities
# Соответствует sensor_msgs/LaserScan.
_LIDAR_HEADER_FMT = "<ffffffff"  # 8 × float32 LE = 32 байта
_LIDAR_HEADER_STRUCT = struct.Struct(_LIDAR_HEADER_FMT)


def encode_lidar_2d(
    *,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    time_increment: float,
    scan_time: float,
    ranges: Sequence[float],
    intensities: Sequence[float],
) -> bytes:
    """Encode sensor_msgs/LaserScan → bytes (little-endian float32)."""
    n = len(ranges)
    if len(intensities) != n:
        raise ValueError(f"ranges ({n}) and intensities ({len(intensities)}) length mismatch")
    header = _LIDAR_HEADER_STRUCT.pack(
        float(angle_min),
        float(angle_max),
        float(angle_increment),
        float(range_min),
        float(range_max),
        float(time_increment),
        float(scan_time),
        float(n),
    )
    ranges_bytes = struct.pack(f"<{n}f", *ranges)
    intensities_bytes = struct.pack(f"<{n}f", *intensities)
    return header + ranges_bytes + intensities_bytes


# --- robot_status payload (meta-quest-api.md §4) -------------------------
def encode_robot_status(
    *,
    battery_pct: int,
    wifi_rssi: int,
    mode: str,
    vel_linear: float,
    vel_angular: float,
    ts_ms: int,
) -> bytes:
    """Encode robot_status (1 Hz) → MessagePack."""
    return msgpack.packb(
        {
            "battery_pct": int(battery_pct),
            "wifi_rssi": int(wifi_rssi),
            "mode": str(mode),
            "vel_linear": float(vel_linear),
            "vel_angular": float(vel_angular),
            "ts_ms": int(ts_ms),
        },
        use_bin_type=True,
    )


# --- person_detections payload (Phase 2, контракт в Phase 1) -------------
def encode_person_detections(
    *,
    ts_ms: int,
    detections: list[dict],
) -> bytes:
    """Encode person_detections → MessagePack.

    Контракт задаётся в Phase 1, источник данных появится в Phase 2 (R11):
    OAK-D/depthai или отдельный YOLO (Q10).
    """
    return msgpack.packb(
        {
            "ts_ms": int(ts_ms),
            "detections": list(detections),
        },
        use_bin_type=True,
    )
