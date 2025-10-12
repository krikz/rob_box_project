"""
Launch file для базового тестирования AudioNode и LEDNode
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Путь к конфигурации
    pkg_dir = get_package_share_directory('rob_box_voice')
    config_file = os.path.join(pkg_dir, 'config', 'voice_assistant.yaml')
    
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
            config_file,
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
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg=['Запуск rob_box_voice базовых нод...']),
        audio_node,
        led_node,
    ])
