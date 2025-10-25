from launch import LaunchDescription
from launch_ros.actions import Node

# OAK-D with integrated AprilTag detection
# This combines camera and AprilTag detection in a single container
# to reduce resource usage on Vision Pi

def generate_launch_description():
    return LaunchDescription([
        # OAK-D Camera Node
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=['/config/oak-d/oak_d_config.yaml'],
            remappings=[
                ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
                ('/camera/color/camera_info', '/camera/camera/color/camera_info'),
            ],
        ),
        
        # AprilTag Detection Node (runs in the same container)
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=['/config/apriltag/apriltag_config.yaml'],
            remappings=[
                ('image_rect', '/camera/rgb/image_raw'),
                ('camera_info', '/camera/rgb/camera_info'),
            ],
        ),
    ])
