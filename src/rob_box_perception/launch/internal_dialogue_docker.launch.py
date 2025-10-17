#!/usr/bin/env python3
"""
internal_dialogue_docker.launch.py - Запуск для Docker контейнера v2.0

НОВАЯ АРХИТЕКТУРА (Event-Driven):
- context_aggregator - собирает данные со всех топиков
- reflection_node v2.0 - размышляет на основе событий

Без vision_stub (будет отдельный контейнер с AI HAT)
Параметры из environment variables
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Параметры из environment или defaults
    dialogue_timeout = float(os.getenv('DIALOGUE_TIMEOUT', '10.0'))
    enable_speech = os.getenv('ENABLE_SPEECH', 'true').lower() == 'true'
    system_prompt_file = os.getenv('SYSTEM_PROMPT_FILE', 'reflection_prompt.txt')
    urgent_response_timeout = float(os.getenv('URGENT_RESPONSE_TIMEOUT', '2.0'))
    
    # Context Aggregator параметры
    context_publish_rate = float(os.getenv('CONTEXT_PUBLISH_RATE', '2.0'))
    memory_window = int(os.getenv('MEMORY_WINDOW', '60'))
    
    return LaunchDescription([
        # Health Monitor - мониторинг здоровья системы
        Node(
            package='rob_box_perception',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'check_rate': 1.0,  # Hz
                'error_window': 30,  # секунд для подсчёта ошибок
                'degraded_threshold': 5,  # ошибок для degraded
                'critical_threshold': 10,  # ошибок для critical
            }],
        ),
        
        # Context Aggregator - сборщик контекста (MPC lite)
        Node(
            package='rob_box_perception',
            executable='context_aggregator',
            name='context_aggregator',
            output='screen',
            parameters=[{
                'publish_rate': context_publish_rate,
                'memory_window': memory_window,
            }],
        ),
        
        # Reflection Node v2.0 - внутренний диалог (event-driven)
        Node(
            package='rob_box_perception',
            executable='reflection_node',
            name='reflection_node',
            output='screen',
            parameters=[{
                'dialogue_timeout': dialogue_timeout,
                'enable_speech': enable_speech,
                'system_prompt_file': system_prompt_file,
                'urgent_response_timeout': urgent_response_timeout,
            }],
        ),
    ])

