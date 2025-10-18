#!/usr/bin/env python3
"""
Простой launch для тестирования STT без dialogue
Запускает только audio_node + stt_node
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Audio Node - запись с ReSpeaker
        Node(
            package='rob_box_voice',
            executable='audio_node',
            name='audio_node',
            parameters=[{
                'device_index': 6,  # ReSpeaker hw:1,0
                'channels': 6,      # 6 каналов
                'sample_rate': 16000,
            }],
            output='screen'
        ),
        
        # STT Node - распознавание речи через Vosk
        Node(
            package='rob_box_voice',
            executable='stt_node',
            name='stt_node',
            parameters=[{
                'model_path': '/models/vosk-model-small-ru-0.22',
                'sample_rate': 16000,
            }],
            output='screen'
        ),
    ])
