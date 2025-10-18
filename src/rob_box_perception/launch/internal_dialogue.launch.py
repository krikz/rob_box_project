#!/usr/bin/env python3
"""
internal_dialogue.launch.py - Запуск Internal Dialogue Agent v2.0

НОВАЯ АРХИТЕКТУРА (Event-Driven):

Запускает:
1. vision_stub_node - заглушка для vision context (пока нет AI HAT)
2. context_aggregator - сборщик контекста (MPC lite)
3. reflection_node - ядро внутреннего диалога (event-driven)

Поток данных:
  [Sensors] → context_aggregator → [PerceptionEvent] → reflection_node → [Speech]
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Vision Stub - заглушка для камеры
        Node(
            package='rob_box_perception',
            executable='vision_stub_node',
            name='vision_stub_node',
            output='screen',
            parameters=[{
                'publish_rate': 1.0,  # 1 Hz
            }],
            remappings=[
                ('/oak/rgb/image_raw/compressed', '/oak/rgb/image_raw/compressed'),
            ]
        ),
        
        # Context Aggregator - сборщик контекста (MPC lite)
        Node(
            package='rob_box_perception',
            executable='context_aggregator',
            name='context_aggregator',
            output='screen',
            parameters=[{
                'publish_rate': 2.0,  # 2 Hz - частота событий
                'memory_window': 60,  # 60 сек
            }],
        ),
        
        # Reflection Node v2.0 - внутренний диалог (event-driven)
        Node(
            package='rob_box_perception',
            executable='reflection_node',
            name='reflection_node',
            output='screen',
            parameters=[{
                'dialogue_timeout': 10.0,  # 10 сек
                'enable_speech': True,
                'system_prompt_file': 'reflection_prompt.txt',
                'urgent_response_timeout': 2.0,  # 2 сек для срочных ответов
            }],
        ),
    ])

