#!/usr/bin/env python3
"""
Launch файл для RTABMap Manager Node
Запускается вместе с rtabmap в одном контейнере
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Генерация launch описания"""
    return LaunchDescription(
        [
            Node(
                package="rob_box_rtabmap_manager",
                executable="rtabmap_manager_node",
                name="rtabmap_manager",
                output="screen",
                parameters=[
                    {"db_path": "/maps/rtabmap.db"},
                    {"backup_dir": "/maps/deleted_backups"},
                    {"auto_backup": True},
                ],
            )
        ]
    )
