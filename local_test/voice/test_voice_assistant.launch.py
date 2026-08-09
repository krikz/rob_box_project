#!/usr/bin/env python3
"""
Launch файл для ПОЛНОГО голосового ассистента с DeepSeek LLM
STT → Dialogue (DeepSeek) → TTS → Sound
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Проверяем что секреты загружены
    deepseek_key = os.getenv('DEEPSEEK_API_KEY')
    if not deepseek_key:
        print("❌ DEEPSEEK_API_KEY не найден!")
        print("Запустите: source /home/ros2/rob_box_project/src/rob_box_voice/.env.secrets")
        raise RuntimeError("DEEPSEEK_API_KEY required")
    
    return LaunchDescription([
        # 1. Audio Node - захват аудио с ReSpeaker
        Node(
            package='rob_box_voice',
            executable='audio_node',
            name='audio_node',
            output='screen',
            parameters=[{
                'device_index': 6,
                'channels': 6,  # RAW режим - 6 каналов (используем channel 0)
                'sample_rate': 16000,
                'chunk_size': 1024,
                'vad_threshold': 3.5,  # VAD порог в dB
                'publish_rate': 10,
                'speech_continuation': 0.5,   # 500ms после речи
                'speech_prefetch': 0.5,       # 500ms перед речью
                'speech_min_duration': 0.3,   # Минимум 300ms
                'speech_max_duration': 10.0,  # Максимум 10s
            }]
        ),
        
        # 2. STT Node - распознавание речи через Vosk
        Node(
            package='rob_box_voice',
            executable='stt_node',
            name='stt_node',
            output='screen',
            parameters=[{
                'model_path': '/models/vosk-model-small-ru-0.22',
            }]
        ),
        
        # 3. Dialogue Node - LLM диалог через DeepSeek API
        Node(
            package='rob_box_voice',
            executable='dialogue_node',
            name='dialogue_node',
            output='screen',
            parameters=[{
                'api_key': deepseek_key,
                'base_url': 'https://api.deepseek.com',
                'model': 'deepseek-chat',
                'temperature': 0.7,
                'max_tokens': 500,
            }]
        ),
        
        # 4. TTS Node - синтез речи через Silero
        Node(
            package='rob_box_voice',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'model_path': '/home/ros2/models/silero_v4_ru.pt',
                'speaker': 'aidar',
                'sample_rate': 48000,
                'chipmunk_mode': True,
                'pitch_shift': 2.0,
                'prosody_rate': 'x-slow',
                'device_index': 6,
            }]
        ),
        
        # 5. Sound Node - воспроизведение звука
        Node(
            package='rob_box_voice',
            executable='sound_node',
            name='sound_node',
            output='screen',
            parameters=[{
                'device_index': 6,  # ReSpeaker output
            }]
        ),
    ])
