from launch import LaunchDescription
from launch_ros.actions import Node

# OAK-D launch for depthai_ros_driver v2 (v2.12.2-humble, arm64+amd64)
# Publishes:
#   /camera/rgb/image_raw       — colour image
#   /camera/rgb/camera_info     — colour camera_info
#   /camera/depth/image_rect_raw — aligned depth
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=['/config/oak_d_config.yaml'],
        ),
    ])
