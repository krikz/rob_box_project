import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Объявление аргументов
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Путь к URDF файлу
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare('rob_box_description'),
        'urdf', 'rob_box.urdf'
    ])

    # Запуск описания робота
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_state_publisher'),
                'launch', 'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file_path])
        }.items()
    )

    # Запуск SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Запуск навигации
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Запуск управления роботом
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rob_box_control'),
                'launch', 'control.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        # Аргументы
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Узлы и подсистемы
        robot_state_publisher,
        slam_launch,
        navigation_launch,
        control_launch
    ])