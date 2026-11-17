"""LiDAR 2D payload: sensor_msgs/LaserScan → bytes.

Источник истины: docs/architecture/meta-quest-api.md §4 (topic_id 0x1101).
Формат: little-endian float32, header (8 × float32) + ranges + intensities.
"""

from __future__ import annotations

from typing import Sequence

from ..protocol.topics import encode_lidar_2d


def scan_to_payload(
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
    """Прямая обёртка над encode_lidar_2d (см. protocol.topics).

    Принимает **именованные** поля — для прямого маппинга из
    sensor_msgs/LaserScan в quest_node.py через **attrs.asdict(msg)**
    или вручную.
    """
    return encode_lidar_2d(
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        time_increment=time_increment,
        scan_time=scan_time,
        ranges=ranges,
        intensities=intensities,
    )
