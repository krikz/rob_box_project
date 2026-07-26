#!/usr/bin/env python3
"""
startup_greeting_node.py - Приветствие при запуске системы

Ждёт пока все критичные ноды запустятся и система будет готова,
затем проигрывает звук + говорит приветствие через TTS.

Критерии готовности:
1. Health Monitor видит статус 'healthy' или 'degraded'
2. Context Aggregator публикует данные
3. Voice Assistant готов (опционально)

Последовательность:
1. [Звук] thinking.mp3 (сразу при старте)
2. [Пауза 2-3 сек] - система загружается
3. [Звук] cute.mp3 или very_cute.mp3 (случайно)
4. [TTS] Случайное приветствие из набора

Приветствия (случайный выбор):
- "Я вернулся! Системы в норме, готов к работе!"
- "Привет! Все модули загружены, жду команд!"
- "Здравствуйте! Загрузка завершена, готов помогать!"
- "Системы запущены! Как я могу помочь?"
- "Всё готово! Чем займёмся?"
- "Рад вернуться! Все датчики в норме!"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rob_box_perception_msgs.msg import PerceptionEvent
import random
import time


class StartupGreetingNode(Node):
    """Нода приветствия при старте системы"""
    
    # Набор приветствий
    GREETINGS = [
        "Я вернулся! Системы в норме, готов к работе!",
        "Привет! Все модули загружены, жду команд!",
        "Здравствуйте! Загрузка завершена, готов помогать!",
        "Системы запущены! Как я могу помочь?",
        "Всё готово! Чем займёмся?",
        "Рад вернуться! Все датчики в норме!",
        "Онлайн! Диалоговая система активна!",
        "Загрузка завершена! Слушаю вас!",
    ]
    
    def __init__(self):
        super().__init__('startup_greeting_node')
        
        # Параметры
        self.declare_parameter('wait_time', 5.0)  # секунд ожидания перед проверкой
        self.declare_parameter('check_timeout', 30.0)  # максимальное время ожидания готовности
        self.declare_parameter('enable_greeting', True)
        
        self.wait_time = self.get_parameter('wait_time').value
        self.check_timeout = self.get_parameter('check_timeout').value
        self.enable_greeting = self.get_parameter('enable_greeting').value
        
        # Publishers
        self.sound_pub = self.create_publisher(String, '/voice/sound/trigger', 10)
        self.tts_pub = self.create_publisher(String, '/voice/dialogue/response', 10)
        
        # Subscribers (для проверки готовности)
        self.context_sub = self.create_subscription(
            PerceptionEvent,
            '/perception/context_update',
            self.on_context,
            10
        )
        
        # Состояние
        self.context_received = False
        self.greeting_done = False
        self.start_time = time.time()
        
        # Сразу играем "thinking" звук
        self.get_logger().info('🚀 Startup Greeting инициализирован')
        self.get_logger().info('   Проигрываю звук загрузки...')
        self.play_sound('thinking')
        
        # Таймер для проверки готовности
        self.check_timer = self.create_timer(1.0, self.check_readiness)
    
    def on_context(self, msg: PerceptionEvent):
        """Получен контекст - система работает"""
        if not self.context_received:
            self.get_logger().info('✅ Context Aggregator готов!')
            self.context_received = True
    
    def check_readiness(self):
        """Проверка готовности системы"""
        if self.greeting_done:
            self.check_timer.cancel()
            return
        
        elapsed = time.time() - self.start_time
        
        # Таймаут - говорим приветствие даже если не всё готово
        if elapsed > self.check_timeout:
            self.get_logger().warn(f'⚠️  Таймаут готовности ({self.check_timeout}s), говорю приветствие...')
            self.say_greeting()
            return
        
        # Минимальное время ожидания прошло?
        if elapsed < self.wait_time:
            return
        
        # Контекст получен - система готова!
        if self.context_received:
            self.get_logger().info('✅ Система готова! Говорю приветствие...')
            self.say_greeting()
    
    def say_greeting(self):
        """Произнести приветствие"""
        if self.greeting_done or not self.enable_greeting:
            return
        
        self.greeting_done = True
        
        # 1. Звук радости
        sound = random.choice(['cute', 'very_cute'])
        self.get_logger().info(f'🎵 Проигрываю звук: {sound}')
        self.play_sound(sound)
        
        # 2. Пауза перед речью
        time.sleep(1.5)
        
        # 3. TTS приветствие
        greeting = random.choice(self.GREETINGS)
        self.get_logger().info(f'🗣️  Говорю: "{greeting}"')
        
        # Формируем JSON chunk (как dialogue_node)
        import json
        chunk_json = {
            "ssml": f"<speak>{greeting}</speak>"
        }
        
        msg = String()
        msg.data = json.dumps(chunk_json, ensure_ascii=False)
        self.tts_pub.publish(msg)
        
        # 4. Завершаем работу ноды через 3 секунды (без once=True для kilted)
        self.shutdown_timer = self.create_timer(3.0, self.shutdown_node)
    
    def shutdown_node(self):
        """Завершить работу ноды (больше не нужна)"""
        self.get_logger().info('👋 Приветствие завершено, завершаю работу ноды')
        
        # Отменяем таймер
        if hasattr(self, 'shutdown_timer'):
            self.shutdown_timer.cancel()
        
        self.destroy_node()
        rclpy.shutdown()
    
    def play_sound(self, sound_name: str):
        """Триггер звука"""
        msg = String()
        msg.data = sound_name
        self.sound_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StartupGreetingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
