"""LED matrix of Vision Pi: driver + compositor (ros2leds) and the face_node next to them.

Started by ``docker/vision/scripts/led_matrix/start_led_matrix.sh`` in the
``led-matrix`` container (ADR-0150 §5.5.2, decision В14). Face parameters —
``/config/led_matrix/face_node.yaml`` from the volume (mouth design, thinking
scene, side test frame), the node defaults otherwise.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

FACE_CONFIG = "/config/led_matrix/face_node.yaml"


def _shutdown_on_matrix_exit(event: ProcessExited, context: LaunchContext) -> list[EmitEvent]:
    """Exit the container when an essential LED process dies so Docker can restart it."""
    if context.is_shutdown or not isinstance(event.action, Node):
        return []
    if event.action.node_package not in {"led_matrix_driver", "led_matrix_compositor"}:
        return []
    return [EmitEvent(event=Shutdown(reason=f"{event.process_name} exited with code {event.returncode}"))]


def generate_launch_description() -> LaunchDescription:
    leds = os.path.join(
        get_package_share_directory("led_matrix_compositor"), "launch", "led_matrix_compositor_launch.py"
    )
    return LaunchDescription(
        [
            RegisterEventHandler(OnProcessExit(on_exit=_shutdown_on_matrix_exit)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(leds)),
            Node(
                package="rob_box_animations",
                executable="face_node.py",
                name="face_node",
                output="screen",
                parameters=[FACE_CONFIG] if os.path.exists(FACE_CONFIG) else [],
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
