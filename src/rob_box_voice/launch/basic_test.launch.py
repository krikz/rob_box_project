"""
Launch file для базового тестирования AudioNode и LEDNode.

Issue #1004 fix (ADR-0004): каждый Node грузит СВОЙ per-node YAML
(audio_node.yaml / led_node.yaml), а не общий voice_assistant.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Путь к per-node конфигам (issue #1004).
    pkg_dir = get_package_share_directory('rob_box_voice')
    config_dir = os.path.join(pkg_dir, 'config')
    audio_node_yaml = os.path.join(config_dir, 'audio_node.yaml')
    led_node_yaml = os.path.join(config_dir, 'led_node.yaml')

    # Аргументы
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # AudioNode
    audio_node = Node(
        package='rob_box_voice',
        executable='audio_node',
        name='audio_node',
        output='screen',
        parameters=[
            audio_node_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Добавить при необходимости
        ]
    )

    # LEDNode
    led_node = Node(
        package='rob_box_voice',
        executable='led_node',
        name='led_node',
        output='screen',
        parameters=[
            led_node_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg=['Запуск rob_box_voice базовых нод...']),
        audio_node,
        led_node,
    ])
