from launch import LaunchDescription
from launch_ros.actions import Node

# OAK-D with integrated AprilTag detection
# This combines camera and AprilTag detection in a single container
# to reduce resource usage on Vision Pi

def generate_launch_description():
    return LaunchDescription([
        # OAK-D Camera Node
        # namespace='camera' makes node path /camera/camera which matches
        # the /camera/camera: key in oak_d_config.yaml, so all params apply.
        # Topics are still under /camera/ prefix (e.g. /camera/rgb/image_raw).
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=['/config/oak-d/oak_d_config.yaml'],
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
                ('detections', '/detections'),
            ],
        ),
    ])
