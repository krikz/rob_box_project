#!/usr/bin/env python3
"""
internal_dialogue_docker.launch.py - Запуск для Docker контейнера

Упрощённая версия для production:
- Без vision_stub (будет отдельный контейнер с AI HAT)
- Только reflection_node
- Использует параметры из environment variables
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Параметры из environment или defaults
    reflection_rate = float(os.getenv('REFLECTION_RATE', '1.0'))
    dialogue_timeout = float(os.getenv('DIALOGUE_TIMEOUT', '10.0'))
    memory_window = int(os.getenv('MEMORY_WINDOW', '60'))
    enable_speech = os.getenv('ENABLE_SPEECH', 'true').lower() == 'true'
    system_prompt_file = os.getenv('SYSTEM_PROMPT_FILE', 'reflection_prompt.txt')
    
    return LaunchDescription([
        # Reflection Node - главная нода размышлений
        Node(
            package='rob_box_perception',
            executable='reflection_node',
            name='reflection_node',
            output='screen',
            parameters=[{
                'reflection_rate': reflection_rate,
                'dialogue_timeout': dialogue_timeout,
                'memory_window': memory_window,
                'enable_speech': enable_speech,
                'system_prompt_file': system_prompt_file,
            }],
        ),
    ])
