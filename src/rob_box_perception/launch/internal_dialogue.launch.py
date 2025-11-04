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

Параметры загружаются из config файла (если доступен)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Config file path (для локальной разработки может не существовать)
    # Path to config file (may not exist for local development)
    config_file = '/config/perception/context_aggregator.yaml'

    # Check if config file exists, otherwise use inline parameters
    # Проверяем существование config файла, иначе используем параметры по умолчанию
    use_config_file = os.path.exists(config_file)

    if use_config_file:
        # Load from config file
        context_aggregator_params = [config_file]
    else:
        # Use inline parameters (fallback for local development)
        context_aggregator_params = [{
            'publish_rate': 2.0,  # 2 Hz - частота событий
            'memory_window': 60,  # 60 сек
            'timezone': 'Europe/Moscow',  # Default: Moscow (MSK)
        }]

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
        # Параметры загружаются из config файла (если доступен)
        Node(
            package='rob_box_perception',
            executable='context_aggregator',
            name='context_aggregator',
            output='screen',
            parameters=context_aggregator_params,
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
