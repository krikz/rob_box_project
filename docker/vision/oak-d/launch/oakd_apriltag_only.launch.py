from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# OAK-D launch for depthai_ros_driver_v3 (humble-arm64-latest)
# Publishes:
#   /oak/rgb/image_raw        — colour image
#   /oak/rgb/camera_info      — colour camera_info
#   /oak/stereo/image_raw     — aligned depth image
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver_v3'),
                    'launch', 'rgbd_pcl.launch.py'
                ])
            ]),
            launch_arguments={
                'name': 'oak',
                'params_file': '/config/oak_d_config.yaml',
            }.items(),
        ),
    ])
