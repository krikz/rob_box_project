#!/usr/bin/env python3
"""
Тест звуковой ноды с исправлениями для ReSpeaker
"""

import os
import sys

# Настраиваем окружение ROS2
os.environ.setdefault('ROS_DISTRO', 'humble')
os.environ.setdefault('ROS_PYTHON_VERSION', '3')
os.environ.setdefault('ROS_VERSION', '2')

# Добавляем пути для ROS2 и нашего кода
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
sys.path.insert(0, '/ws/install/rob_box_voice/lib/python3.10/site-packages')
sys.path.append("/ws/src/rob_box_voice")

try:
    print("Импортируем sounddevice...")
    import sounddevice as sd
    print(f"✅ sounddevice импортирован, версия: {sd.__version__}")
    
    print("Проверяем аудио устройства...")
    devices = sd.query_devices()
    print(f"Найдено устройств: {len(devices)}")
    
    # Ищем ReSpeaker
    for i, device in enumerate(devices):
        if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
            print(f"✅ ReSpeaker найден: {device['name']} (index {i})")
            print(f"   Max output channels: {device['max_output_channels']}")
    
    print("Тестируем простое воспроизведение...")
    import numpy as np
    
    # Создаем тестовый звук (1 секунда, 440Hz)
    duration = 1.0
    sample_rate = 16000
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = np.sin(2 * np.pi * 440 * t) * 0.1  # Низкая громкость
    
    # Проверяем что можем воспроизвести без ошибок каналов
    try:
        print("Тестируем воспроизведение (моно)...")
        sd.play(tone, samplerate=sample_rate, device=0)  # device 0 должен быть ReSpeaker
        sd.wait()
        print("✅ Моно воспроизведение успешно")
        
        print("Тестируем воспроизведение (стерео)...")
        stereo_tone = np.column_stack((tone, tone))  # Делаем стерео
        sd.play(stereo_tone, samplerate=sample_rate, device=0, channels=2)
        sd.wait()
        print("✅ Стерео воспроизведение успешно")
        
    except Exception as e:
        print(f"❌ Ошибка воспроизведения: {e}")
    
    print("Импортируем ROS2...")
    import rclpy
    print("✅ rclpy импортирован успешно")
    
    print("Импортируем SoundNode...")
    from rob_box_voice.sound_node import SoundNode
    print("✅ SoundNode импортирован успешно")
    
    # Попробуем создать ноду
    print("Инициализируем ROS2...")
    rclpy.init()
    print("Создаем SoundNode...")
    node = SoundNode()
    print("✅ SoundNode создан успешно")
    print(f"Device index: {node.device_index}")
    node.destroy_node()
    rclpy.shutdown()
    print("✅ SoundNode тест завершен успешно")
    
except Exception as e:
    print(f"❌ Ошибка: {e}")
    import traceback
    traceback.print_exc()