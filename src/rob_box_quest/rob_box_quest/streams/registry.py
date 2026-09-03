"""Registry доступных стримов rob_box_quest.

Источник истины: docs/architecture/meta-quest-api.md §4 (topic_id map)
+ docs/plans/2026-08-24-meta-quest-telepresence.md §1.4 (Phase 1.4 v2:
видео мимо ROS2, мульти-камера).

Стримы делятся на 2 класса:
- ROS2 стримы (lidar_2d, map_2d, robot_status, voice_state,
  person_detections, camera_rear, camera_ceiling): идут через Zenoh,
  payload формируется в protocol/topics.py и streams/*.
- Камеры мимо ROS (camera_oak_color, camera_oak_depth): читаются
  CameraProvider'ом в Vision Pi напрямую (depthai SDK), payload =
  JPEG/H.264 bytes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class StreamKind(str, Enum):
    """Источник данных: ROS2-топик или прямое чтение с устройства."""

    ROS_TOPIC = "ros_topic"
    CAMERA_DIRECT = "camera_direct"


@dataclass(frozen=True)
class StreamSpec:
    """Описание одного стрима — известно серверу статически."""

    ui_name: str  # "camera_oak_color"
    topic_id: int  # 0x1003 для camera_oak_color
    kind: StreamKind
    # Для ROS_TOPIC: имя ROS-топика для подписки.
    # Для CAMERA_DIRECT: device_id ("oak_color"/"oak_depth"/"ceiling")
    #                    или URL.
    source: str
    # Quality → bandwidth hint для UI (Phase 2).
    default_quality: str = "med"  # low/med/high
    # Описание для UI (человек-читаемое).
    description: str = ""


# Каталог стримов Phase 1.4 v2.
# topic_id продолжает нумерацию из protocol/topics.py §4 (0x1001/0x1101/0x1201/0x1301).
STREAM_CATALOG: dict[str, StreamSpec] = {
    # --- ROS-стримы (через Zenoh, payload из protocol.topics) ----------
    "camera_rear": StreamSpec(
        ui_name="camera_rear",
        topic_id=0x1001,
        kind=StreamKind.ROS_TOPIC,
        source="/camera/camera/color/image_raw",
        default_quality="med",
        description="OAK-D colour (Phase 1 fallback через ROS)",
    ),
    "lidar_2d": StreamSpec(
        ui_name="lidar_2d",
        topic_id=0x1101,
        kind=StreamKind.ROS_TOPIC,
        source="/scan",
        default_quality="high",
        description="2D LiDAR (sensor_msgs/LaserScan)",
    ),
    # Потолочная камера. Раньше стояла как CAMERA_DIRECT на /dev/video0 и
    # никогда не отдавала ни кадра: устройство держит эксклюзивно контейнер
    # `ceiling-camera` (usb_cam), а в `rob-box-quest` /dev/video0 вообще не
    # прокинут (`docker inspect … .HostConfig.Devices` → null). В логах это
    # видно как «cannot open /dev/video0 → camera camera_ceiling unavailable
    # — thread exits» на каждом старте. Берём кадры оттуда, где они уже
    # есть — из ROS-топика usb_cam'а, тем же лёгким путём, что camera_rear
    # (JPEG от image_transport форвардится as-is, без перекодирования).
    "camera_ceiling": StreamSpec(
        ui_name="camera_ceiling",
        topic_id=0x1005,
        kind=StreamKind.ROS_TOPIC,
        source="/ceiling_camera/image_raw/compressed",
        default_quality="med",
        description="USB ceiling camera (usb_cam → image_transport JPEG)",
    ),
    "map_2d": StreamSpec(
        ui_name="map_2d",
        topic_id=0x1103,
        kind=StreamKind.ROS_TOPIC,
        source="/rtabmap/map",
        default_quality="low",
        description="SLAM occupancy grid (RGBA PNG + поза робота)",
    ),
    "robot_status": StreamSpec(
        ui_name="robot_status",
        topic_id=0x1201,
        kind=StreamKind.ROS_TOPIC,
        source="aggregation",
        default_quality="med",
        description="1 Hz battery/wifi/mode/vel",
    ),
    "voice_state": StreamSpec(
        ui_name="voice_state",
        topic_id=0x1202,
        kind=StreamKind.ROS_TOPIC,
        source="/voice/dialogue/state",
        default_quality="med",
        description="Dialogue FSM state",
    ),
    "person_detections": StreamSpec(
        ui_name="person_detections",
        topic_id=0x1301,
        kind=StreamKind.ROS_TOPIC,
        source="phase2_source",
        default_quality="med",
        description="Phase 2 (R11)",
    ),
    # --- Камеры мимо ROS (читаются CameraProvider напрямую) ------------
    "camera_oak_color": StreamSpec(
        ui_name="camera_oak_color",
        topic_id=0x1003,
        kind=StreamKind.CAMERA_DIRECT,
        source="oak:color",
        default_quality="med",
        description="OAK-D color (depthai SDK, in-process)",
    ),
    "camera_oak_depth": StreamSpec(
        ui_name="camera_oak_depth",
        topic_id=0x1004,
        kind=StreamKind.CAMERA_DIRECT,
        source="oak:depth",
        default_quality="med",
        description="OAK-D depth (depthai SDK, colormapped)",
    ),
}


def get_stream(ui_name: str) -> Optional[StreamSpec]:
    """Lookup стрима по UI-name (None если нет в каталоге)."""
    return STREAM_CATALOG.get(ui_name)


def list_streams() -> list[StreamSpec]:
    """Все стримы — для stream_select list cmd (Phase 2 R10)."""
    return list(STREAM_CATALOG.values())


def topic_id_for(ui_name: str) -> Optional[int]:
    """Shortcut: stream_spec.topic_id или None."""
    spec = STREAM_CATALOG.get(ui_name)
    return spec.topic_id if spec else None
