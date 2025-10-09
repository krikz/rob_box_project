#!/usr/bin/python3
# LSLIDAR N10 Headless Launch - без RViz2 для Vision Pi
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

import os

def generate_launch_description():
    """
    Headless launch для LSLIDAR N10
    Только драйвер, без визуализации RViz2
    """
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir],
    )

    return LaunchDescription([
        driver_node,
    ])
