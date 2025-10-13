#!/usr/bin/env python3
"""
Тестовый скрипт для проверки sound_node
Публикует триггеры в /voice/sound/trigger
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class SoundTester(Node):
    def __init__(self):
        super().__init__('sound_tester')
        self.publisher = self.create_publisher(String, '/voice/sound/trigger', 10)
        self.get_logger().info('Sound Tester готов!')
    
    def trigger(self, sound_name: str):
        msg = String()
        msg.data = sound_name
        self.publisher.publish(msg)
        self.get_logger().info(f'🔔 Триггер: {sound_name}')


def main():
    rclpy.init()
    tester = SoundTester()
    
    # Список всех звуков
    sounds = [
        'thinking',
        'surprise',
        'confused',
        'angry_1',
        'angry_2',
        'cute',
        'very_cute',
        'talk_1',
        'talk_2',
        'talk_3',
        'talk_4',
        # Группы
        'talk',   # Random из talk_1..4
        'angry',  # Random из angry_1..2
        'cute',   # Random из cute/very_cute
    ]
    
    print("\n╔════════════════════════════════════════════════════════╗")
    print("║       🎵 SOUND NODE TESTER                            ║")
    print("╚════════════════════════════════════════════════════════╝\n")
    print("Доступные звуки:")
    for i, sound in enumerate(sounds, 1):
        print(f"  {i}. {sound}")
    print("\n  0. Выход")
    print("  a. Все подряд (с паузой)")
    print("═══════════════════════════════════════════════════════\n")
    
    while True:
        try:
            choice = input("Выбор: ").strip().lower()
            
            if choice == '0':
                break
            elif choice == 'a':
                print("\n▶️ Играю все звуки подряд...\n")
                for sound in sounds[:11]:  # Только индивидуальные, без групп
                    tester.trigger(sound)
                    time.sleep(2.5)  # Пауза между звуками
                print("\n✅ Завершено!\n")
            elif choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(sounds):
                    tester.trigger(sounds[idx])
                else:
                    print("❌ Неверный номер")
            else:
                # Прямой ввод имени звука
                tester.trigger(choice)
            
            # Небольшой спин для обработки
            rclpy.spin_once(tester, timeout_sec=0.1)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ Ошибка: {e}")
    
    tester.destroy_node()
    rclpy.shutdown()
    print("\n👋 До свидания!\n")


if __name__ == '__main__':
    main()
