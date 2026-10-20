"""LED matrix of Vision Pi: driver + compositor (ros2leds) and the face_node next to them.

Started by ``docker/vision/scripts/led_matrix/start_led_matrix.sh`` in the
``led-matrix`` container (ADR-0150 §5.5.2, decision В14). Face parameters —
``/config/led_matrix/face_node.yaml`` from the volume (mouth design, thinking
scene, side test frame), the node defaults otherwise.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

FACE_CONFIG = '/config/led_matrix/face_node.yaml'


def generate_launch_description():
    leds = os.path.join(get_package_share_directory('led_matrix_compositor'), 'launch', 'led_matrix_compositor_launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(leds)),
        Node(
            package='rob_box_animations',
            executable='face_node.py',
            name='face_node',
            output='screen',
            parameters=[FACE_CONFIG] if os.path.exists(FACE_CONFIG) else [],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
