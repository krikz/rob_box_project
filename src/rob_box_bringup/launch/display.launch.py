#!/usr/bin/env python3
"""
Rob Box Display Launch - Визуализация робота в RViz
Запускает robot_state_publisher и RViz для просмотра модели
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Аргументы
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Запустить joint_state_publisher GUI для управления joints'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Запустить RViz для визуализации'
    )
    
    # Путь к URDF
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('rob_box_description'),
                'urdf',
                'rob_box.xacro'
            ])
        ]),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher (для тестирования)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Joint State Publisher GUI (для ручного управления колесами)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
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
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
