#!/usr/bin/env python3
"""
Rob Box Display Launch - Визуализация в RViz

Запускает минимальную систему для визуализации робота:
- robot_state_publisher: публикует TF дерево из URDF
- dummy_joint_state_publisher: симулирует вращение колес
- RViz2: 3D визуализация робота и TF frames

Использование:
    ros2 launch rob_box_bringup display_simple.launch.py

См. docs/guides/visualization.md для деталей
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Путь к URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare('rob_box_description'),
        'urdf',
        'rob_box.xacro'
    ])
    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 30.0,
            'ignore_timestamp': False
        }],
    )
    
    # Dummy Joint State Publisher - публикует нулевые значения для колес
    dummy_joint_state_publisher_node = Node(
        package='rob_box_bringup',
        executable='dummy_joint_state_publisher.py',
        name='dummy_joint_state_publisher',
        output='screen'
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rob_box_description'),
        'rviz',
        'rob_box.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        dummy_joint_state_publisher_node,
        rviz_node
    ])
