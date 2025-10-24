#!/usr/bin/env python3
"""
Примеры использования оригинального голоса ROBBOX

Демонстрирует:
1. Нормальную скорость воспроизведения
2. SSML управление pitch/rate
3. Управление громкостью через команды
4. Переключение режимов chipmunk
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class VoiceExampleNode(Node):
    """Пример использования voice system"""
    
    def __init__(self):
        super().__init__('voice_example_node')
        
        # Publisher для dialogue response (имитация dialogue_node)
        self.dialogue_pub = self.create_publisher(
            String, '/voice/dialogue/response', 10
        )
        
        # Publisher для STT result (имитация user речи)
        self.stt_pub = self.create_publisher(
            String, '/voice/stt/result', 10
        )
        
        self.get_logger().info('✅ VoiceExampleNode запущен')
    
    def send_speech(self, ssml: str, dialogue_id: str = "test123"):
        """Отправить SSML для синтеза речи"""
        chunk_data = {
            "ssml": ssml,
            "dialogue_id": dialogue_id
        }
        
        msg = String()
        msg.data = json.dumps(chunk_data, ensure_ascii=False)
        self.dialogue_pub.publish(msg)
        
        self.get_logger().info(f'📤 Отправлено: {ssml[:60]}...')
    
    def send_user_command(self, text: str):
        """Имитация пользовательской команды"""
        msg = String()
        msg.data = text
        self.stt_pub.publish(msg)
        
        self.get_logger().info(f'👤 User: {text}')
    
    def run_examples(self):
        """Запустить примеры"""
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🎤 Примеры использования ROBBOX Voice')
        self.get_logger().info('='*60)
        
        # Пример 1: Нормальная скорость (по умолчанию)
        self.get_logger().info('\n📌 Пример 1: Нормальная скорость речи')
        self.send_speech('<speak>Привет! Я говорю на нормальной скорости.</speak>')
        time.sleep(5)
        
        # Пример 2: Быстрая речь через SSML
        self.get_logger().info('\n📌 Пример 2: Быстрая речь (rate=1.5)')
        self.send_speech(
            '<speak><prosody rate="1.5">Сейчас я говорю быстрее обычного.</prosody></speak>'
        )
        time.sleep(5)
        
        # Пример 3: Медленная речь через SSML
        self.get_logger().info('\n📌 Пример 3: Медленная речь (rate=0.7)')
        self.send_speech(
            '<speak><prosody rate="0.7">А сейчас я говорю медленнее.</prosody></speak>'
        )
        time.sleep(7)
        
        # Пример 4: Использование ключевых слов
        self.get_logger().info('\n📌 Пример 4: Ключевые слова (fast, slow)')
        self.send_speech(
            '<speak><prosody rate="fast">Очень быстро!</prosody></speak>'
        )
        time.sleep(3)
        
        # Пример 5: Управление громкостью голосом
        self.get_logger().info('\n📌 Пример 5: Команда увеличения громкости')
        self.send_user_command('робот, громче')
        time.sleep(3)
        
        self.send_speech('<speak>Теперь я говорю громче!</speak>')
        time.sleep(4)
        
        # Пример 6: Уменьшение громкости
        self.get_logger().info('\n📌 Пример 6: Команда уменьшения громкости')
        self.send_user_command('робот, тише')
        time.sleep(3)
        
        self.send_speech('<speak>А теперь тише.</speak>')
        time.sleep(4)
        
        # Пример 7: Максимальная громкость
        self.get_logger().info('\n📌 Пример 7: Максимальная громкость')
        self.send_user_command('робот, говори громко')
        time.sleep(3)
        
        self.send_speech('<speak>Максимальная громкость!</speak>')
        time.sleep(4)
        
        # Пример 8: Восстановление нормальной громкости
        self.get_logger().info('\n📌 Пример 8: Нормальная громкость')
        self.send_user_command('робот, нормальная громкость')
        time.sleep(3)
        
        self.send_speech('<speak>Вернулся к нормальной громкости.</speak>')
        time.sleep(4)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✅ Все примеры выполнены!')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    node = VoiceExampleNode()
    
    # Даём время для инициализации других нод
    node.get_logger().info('⏳ Ожидание 3 секунды для инициализации...')
    time.sleep(3)
    
    # Запуск примеров
    node.run_examples()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
