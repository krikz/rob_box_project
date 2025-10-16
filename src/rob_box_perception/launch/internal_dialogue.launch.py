#!/usr/bin/env python3
"""
internal_dialogue.launch.py - Запуск Internal Dialogue Agent

Запускает:
1. vision_stub_node - заглушка для vision context (пока нет AI HAT)
2. reflection_node - ядро внутреннего диалога
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
                'publish_rate': 1.0,  # 1 Hz - достаточно для размышлений
            }],
            remappings=[
                # Подписка на камеру OAK-D
                ('/oak/rgb/image_raw/compressed', '/oak/rgb/image_raw/compressed'),
            ]
        ),
        
        # Reflection Node - главная нода размышлений
        Node(
            package='rob_box_perception',
            executable='reflection_node',
            name='reflection_node',
            output='screen',
            parameters=[{
                'reflection_rate': 1.0,  # 1 Hz - размышления каждую секунду
                'dialogue_timeout': 10.0,  # 10 сек - тайм-аут диалога
                'memory_window': 60,  # 60 сек - окно короткой памяти
                'enable_speech': True,  # Включить речь робота
            }],
            # Используем переменные окружения из .env.secrets
            # (DEEPSEEK_API_KEY уже должен быть в окружении)
        ),
    ])
