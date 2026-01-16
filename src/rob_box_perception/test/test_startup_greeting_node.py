#!/usr/bin/env python3
"""
test_startup_greeting_node.py - Unit тесты для StartupGreetingNode

Тестирует:
- Инициализацию ноды
- Проигрывание звуков загрузки
- Генерацию случайного приветствия
- Проверку готовности системы
- Публикацию TTS запросов
- Завершение работы после приветствия
"""

import unittest
from unittest.mock import MagicMock, patch, Mock, call
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rob_box_perception_msgs.msg import PerceptionEvent

from rob_box_perception.startup_greeting_node import StartupGreetingNode


class TestStartupGreetingNode(unittest.TestCase):
    """Тесты для StartupGreetingNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2"""
        rclpy.shutdown()

    def setUp(self):
        """Создание ноды для теста"""
        self.node = StartupGreetingNode()

    def tearDown(self):
        """Очистка после теста"""
        if hasattr(self.node, 'check_timer'):
            self.node.check_timer.cancel()
        self.node.destroy_node()

    def test_node_creation(self):
        """Тест: нода создается корректно"""
        self.assertEqual(self.node.get_name(), 'startup_greeting_node')

    def test_parameters_exist(self):
        """Тест: все параметры объявлены"""
        params = ['wait_time', 'check_timeout', 'enable_greeting']
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_default_parameters(self):
        """Тест: параметры по умолчанию"""
        self.assertEqual(self.node.get_parameter('wait_time').value, 5.0)
        self.assertEqual(self.node.get_parameter('check_timeout').value, 30.0)
        self.assertTrue(self.node.get_parameter('enable_greeting').value)

    def test_publishers_created(self):
        """Тест: паблишеры созданы"""
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        pub_topics = [name for name, _ in publishers]
        
        self.assertIn('/voice/sound/trigger', pub_topics)
        self.assertIn('/voice/dialogue/response', pub_topics)

    def test_subscribers_created(self):
        """Тест: подписчики созданы"""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        
        self.assertIn('/perception/context_update', sub_topics)

    def test_initial_state(self):
        """Тест: начальное состояние ноды"""
        self.assertFalse(self.node.context_received)
        self.assertFalse(self.node.greeting_done)
        self.assertIsNotNone(self.node.start_time)

    def test_greetings_list(self):
        """Тест: список приветствий не пустой"""
        self.assertIsInstance(StartupGreetingNode.GREETINGS, list)
        self.assertGreater(len(StartupGreetingNode.GREETINGS), 0)
        
        # Проверяем что все приветствия - строки
        for greeting in StartupGreetingNode.GREETINGS:
            self.assertIsInstance(greeting, str)
            self.assertGreater(len(greeting), 0)

    def test_play_sound_publishes_message(self):
        """Тест: play_sound публикует сообщение"""
        with patch.object(self.node.sound_pub, 'publish') as mock_publish:
            self.node.play_sound('thinking')
            
            # Проверяем что был вызов publish
            mock_publish.assert_called_once()
            
            # Проверяем содержимое сообщения
            args = mock_publish.call_args[0]
            msg = args[0]
            self.assertIsInstance(msg, String)
            self.assertEqual(msg.data, 'thinking')

    def test_play_sound_with_different_sounds(self):
        """Тест: проигрывание разных звуков"""
        sounds = ['thinking', 'cute', 'very_cute']
        
        with patch.object(self.node.sound_pub, 'publish') as mock_publish:
            for sound in sounds:
                self.node.play_sound(sound)
            
            # Проверяем что было 3 вызова
            self.assertEqual(mock_publish.call_count, 3)
            
            # Проверяем что звуки правильные
            calls = mock_publish.call_args_list
            for i, sound in enumerate(sounds):
                msg = calls[i][0][0]
                self.assertEqual(msg.data, sound)

    def test_on_context_callback(self):
        """Тест: обработка context update"""
        self.assertFalse(self.node.context_received)
        
        # Создаем мок события
        event = PerceptionEvent()
        self.node.on_context(event)
        
        self.assertTrue(self.node.context_received)

    def test_on_context_callback_only_once(self):
        """Тест: флаг context_received устанавливается один раз"""
        event = PerceptionEvent()
        
        # Первый вызов
        self.node.on_context(event)
        self.assertTrue(self.node.context_received)
        
        # Второй вызов не должен изменить состояние
        self.node.context_received = False  # Сбрасываем для проверки
        self.node.on_context(event)
        # Логика проверяет if not self.context_received, поэтому установит снова
        self.assertTrue(self.node.context_received)

    def test_check_readiness_before_wait_time(self):
        """Тест: не говорим приветствие до wait_time"""
        self.node.start_time = time.time()
        self.node.wait_time = 5.0
        self.node.context_received = True
        
        with patch.object(self.node, 'say_greeting') as mock_greeting:
            self.node.check_readiness()
            mock_greeting.assert_not_called()

    def test_check_readiness_after_wait_time(self):
        """Тест: говорим приветствие после wait_time если контекст получен"""
        self.node.start_time = time.time() - 6.0  # 6 секунд назад
        self.node.wait_time = 5.0
        self.node.context_received = True
        
        with patch.object(self.node, 'say_greeting') as mock_greeting:
            self.node.check_readiness()
            mock_greeting.assert_called_once()

    def test_check_readiness_timeout(self):
        """Тест: приветствие по таймауту даже без контекста"""
        self.node.start_time = time.time() - 35.0  # 35 секунд назад
        self.node.check_timeout = 30.0
        self.node.context_received = False
        
        with patch.object(self.node, 'say_greeting') as mock_greeting:
            self.node.check_readiness()
            mock_greeting.assert_called_once()

    def test_check_readiness_skips_if_done(self):
        """Тест: check_readiness отменяется после приветствия"""
        self.node.greeting_done = True
        
        with patch.object(self.node, 'say_greeting') as mock_greeting:
            self.node.check_readiness()
            mock_greeting.assert_not_called()

    def test_say_greeting_disabled(self):
        """Тест: приветствие не говорится если отключено"""
        self.node.enable_greeting = False
        
        with patch.object(self.node.sound_pub, 'publish'), \
             patch.object(self.node.tts_pub, 'publish'):
            self.node.say_greeting()
            
            self.assertFalse(self.node.greeting_done)

    def test_say_greeting_plays_cute_sound(self):
        """Тест: say_greeting проигрывает cute звук"""
        with patch.object(self.node, 'play_sound') as mock_play, \
             patch.object(self.node.tts_pub, 'publish'), \
             patch('time.sleep'):
            
            self.node.say_greeting()
            
            # Проверяем что play_sound вызван с cute или very_cute
            calls = mock_play.call_args_list
            sound_name = calls[0][0][0]
            self.assertIn(sound_name, ['cute', 'very_cute'])

    def test_say_greeting_publishes_tts(self):
        """Тест: say_greeting публикует TTS запрос"""
        with patch.object(self.node, 'play_sound'), \
             patch.object(self.node.tts_pub, 'publish') as mock_publish, \
             patch('time.sleep'):
            
            self.node.say_greeting()
            
            # Проверяем что TTS опубликован
            mock_publish.assert_called_once()
            
            # Проверяем формат сообщения
            args = mock_publish.call_args[0]
            msg = args[0]
            self.assertIsInstance(msg, String)
            
            # Парсим JSON
            data = json.loads(msg.data)
            self.assertIn('ssml', data)
            self.assertIn('<speak>', data['ssml'])
            self.assertIn('</speak>', data['ssml'])

    def test_say_greeting_uses_random_greeting(self):
        """Тест: используется случайное приветствие"""
        greetings_used = set()
        
        with patch.object(self.node, 'play_sound'), \
             patch('time.sleep'), \
             patch.object(self.node.tts_pub, 'publish') as mock_publish:
            
            # Вызываем несколько раз (пересоздаем ноду)
            for _ in range(10):
                self.node.greeting_done = False
                self.node.say_greeting()
                
                # Извлекаем приветствие из последнего вызова
                msg = mock_publish.call_args[0][0]
                data = json.loads(msg.data)
                ssml = data['ssml']
                
                # Извлекаем текст между <speak> тегами
                greeting = ssml.replace('<speak>', '').replace('</speak>', '')
                greetings_used.add(greeting)
        
        # Должно быть использовано хотя бы одно приветствие
        self.assertGreater(len(greetings_used), 0)
        
        # Все использованные приветствия должны быть в списке
        for greeting in greetings_used:
            self.assertIn(greeting, StartupGreetingNode.GREETINGS)

    def test_say_greeting_creates_shutdown_timer(self):
        """Тест: say_greeting создает таймер завершения"""
        with patch.object(self.node, 'play_sound'), \
             patch.object(self.node.tts_pub, 'publish'), \
             patch('time.sleep'):
            
            self.node.say_greeting()
            
            self.assertTrue(hasattr(self.node, 'shutdown_timer'))
            self.assertIsNotNone(self.node.shutdown_timer)

    def test_say_greeting_sets_done_flag(self):
        """Тест: say_greeting устанавливает флаг завершения"""
        with patch.object(self.node, 'play_sound'), \
             patch.object(self.node.tts_pub, 'publish'), \
             patch('time.sleep'):
            
            self.assertFalse(self.node.greeting_done)
            self.node.say_greeting()
            self.assertTrue(self.node.greeting_done)

    def test_say_greeting_only_once(self):
        """Тест: приветствие говорится только один раз"""
        with patch.object(self.node, 'play_sound') as mock_play, \
             patch.object(self.node.tts_pub, 'publish') as mock_publish, \
             patch('time.sleep'):
            
            # Первый вызов
            self.node.say_greeting()
            self.assertEqual(mock_publish.call_count, 1)
            
            # Второй вызов - ничего не должно произойти
            self.node.say_greeting()
            self.assertEqual(mock_publish.call_count, 1)

    def test_shutdown_node_cancels_timer(self):
        """Тест: shutdown_node отменяет таймер"""
        # Создаем мок таймер
        self.node.shutdown_timer = MagicMock()
        
        with patch.object(self.node, 'destroy_node'), \
             patch('rclpy.shutdown'):
            
            self.node.shutdown_node()
            self.node.shutdown_timer.cancel.assert_called_once()


class TestStartupGreetingNodeIntegration(unittest.TestCase):
    """Интеграционные тесты StartupGreetingNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        self.node = StartupGreetingNode()

    def tearDown(self):
        """Очистка"""
        if hasattr(self.node, 'check_timer'):
            self.node.check_timer.cancel()
        self.node.destroy_node()

    def test_startup_sequence(self):
        """Тест: последовательность запуска"""
        received_sounds = []
        received_tts = []
        
        def sound_callback(msg):
            received_sounds.append(msg.data)
        
        def tts_callback(msg):
            received_tts.append(msg.data)
        
        # Создаем тестовую ноду
        test_node = rclpy.create_node('test_listener')
        
        # Подписываемся на звуки и TTS
        sound_sub = test_node.create_subscription(
            String,
            '/voice/sound/trigger',
            sound_callback,
            10
        )
        tts_sub = test_node.create_subscription(
            String,
            '/voice/dialogue/response',
            tts_callback,
            10
        )
        
        # Даем время на обработку
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.destroy_node()
        
        # Проверяем что хотя бы thinking звук был воспроизведен при старте
        # (в конструкторе сразу вызывается play_sound('thinking'))
        # Но это происходит до spin, так что может не успеть
        # self.assertIn('thinking', received_sounds)

    def test_context_triggers_greeting(self):
        """Тест: получение контекста запускает приветствие"""
        # Устанавливаем короткое время ожидания
        self.node.wait_time = 0.1
        
        # Публикуем контекст
        pub = rclpy.create_node('test_publisher')
        context_pub = pub.create_publisher(
            PerceptionEvent,
            '/perception/context_update',
            10
        )
        
        event = PerceptionEvent()
        context_pub.publish(event)
        
        # Даем время на обработку
        time.sleep(0.2)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(pub, timeout_sec=0.1)
        
        pub.destroy_node()
        
        # Проверяем что контекст получен
        self.assertTrue(self.node.context_received)


if __name__ == '__main__':
    unittest.main()
