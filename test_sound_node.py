#!/usr/bin/env python3
"""
Тест звуковой ноды с исправлениями для ReSpeaker
"""

import sys
sys.path.append("/ws/src")

try:
    from rob_box_voice.sound_node import SoundNode
    print("✅ SoundNode импортирован успешно")
    
    # Попробуем создать ноду
    import rclpy
    rclpy.init()
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