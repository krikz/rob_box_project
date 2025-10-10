from launch import LaunchDescription
from launch_ros.actions import Node

# Minimal launch for OAK-D: only publishes color image and camera_info for AprilTag

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=['/config/oak_d_config.yaml'],
            remappings=[
                ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
                ('/camera/color/camera_info', '/camera/camera/color/camera_info'),
            ],
        ),
    ])
