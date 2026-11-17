#!/usr/bin/env python3
"""
Тестовый скрипт для проверки command_node
Эмулирует STT результаты с разными командами
"""

import rclpy
from rclpy.node import Node
from rob_box_dialog_msgs.msg import Utterance
import sys


class CommandTester(Node):
    def __init__(self):
        super().__init__('command_tester')
        self.publisher = self.create_publisher(Utterance, '/voice/stt/utterance', 10)
        self.get_logger().info('Command Tester готов!')
    
    def send_command(self, text: str):
        msg = Utterance()
        msg.text = text
        msg.source = Utterance.SOURCE_RESPEAKER
        self.publisher.publish(msg)
        self.get_logger().info(f'🎤 STT: {text}')


def main():
    rclpy.init()
    tester = CommandTester()
    
    # Список тестовых команд
    commands = [
        # Навигация
        'двигайся к точке 1',
        'иди к точке 2',
        'поезжай к точке 3',
        'двигайся к кухня',
        'иди к дом',
        'поезжай к гостиная',
        
        # Остановка
        'стоп',
        'остановись',
        'стой',
        'отмени движение',
        
        # Статус
        'где ты',
        'покажи статус',
        'расскажи положение',
        
        # Карта
        'покажи карту',
        'построй карту',
        
        # Зрение
        'что видишь',
        'найди объект',
        
        # Следование
        'следуй за мной',
        'включи режим следования',
    ]
    
    print("\n╔════════════════════════════════════════════════════════╗")
    print("║       🎯 COMMAND NODE TESTER                          ║")
    print("╚════════════════════════════════════════════════════════╝\n")
    print("Доступные команды:")
    for i, cmd in enumerate(commands, 1):
        print(f"  {i:2d}. {cmd}")
    print("\n  0. Выход")
    print("  a. Все подряд (с паузой)")
    print("  c. Своя команда")
    print("═══════════════════════════════════════════════════════\n")
    
    while True:
        try:
            choice = input("Выбор: ").strip().lower()
            
            if choice == '0':
                break
            elif choice == 'a':
                print("\n▶️ Отправляю все команды подряд...\n")
                import time
                for cmd in commands:
                    tester.send_command(cmd)
                    time.sleep(2.0)
                print("\n✅ Завершено!\n")
            elif choice == 'c':
                custom = input("Введите команду: ").strip()
                if custom:
                    tester.send_command(custom)
            elif choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(commands):
                    tester.send_command(commands[idx])
                else:
                    print("❌ Неверный номер")
            else:
                # Прямой ввод команды
                if choice:
                    tester.send_command(choice)
            
            # Небольшой спин
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
