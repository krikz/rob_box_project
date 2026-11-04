"""AprilTag на потолочной камере: кормушка (5 Гц, mono8) + apriltag_node.

Запускается из start_ceiling_camera.sh, только когда CEILING_APRILTAG_ENABLED=true
(docker/vision/docker-compose.yaml, по умолчанию выключено).

Отдельно от AprilTag OAK-D (oakd_with_apriltag.launch.py → /detections):
у камер разные frame_id, а rtabmap и context_aggregator ждут в /detections
только OAK-D. Поэтому здесь свои топики:

* /ceiling_camera/apriltag/detections — apriltag_msgs/AprilTagDetectionArray;
* /ceiling_camera/apriltag/tf — позы тегов относительно
  ceiling_camera_optical_frame. apriltag_node шлёт их в /tf с именами
  ``tag36h11:<id>``, теми же, что у OAK-D: в общем /tf у кадра тега оказалось
  бы два родителя. Ремап держит общее дерево TF чистым.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

NS = "/ceiling_camera/apriltag"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                # Скрипт смонтирован из репо (./scripts/ceiling-camera:/scripts),
                # бит исполнения на томе не гарантирован — запускаем через python3.
                executable="python3",
                arguments=["/scripts/ceiling_tag_feed.py"],
                name="ceiling_tag_feed",
                output="screen",
                parameters=[{"rate_hz": 5.0}],
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="ceiling_apriltag",
                output="screen",
                parameters=["/config/apriltag/ceiling_apriltag.yaml"],
                remappings=[
                    ("image_rect", f"{NS}/image"),
                    ("camera_info", f"{NS}/camera_info"),
                    ("detections", f"{NS}/detections"),
                    ("/tf", f"{NS}/tf"),
                ],
            ),
        ]
    )
