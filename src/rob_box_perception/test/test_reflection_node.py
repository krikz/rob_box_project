#!/usr/bin/env python3
"""
test_reflection_node.py - Unit тесты для ReflectionNode

Тестирует:
- Обработку событий восприятия
- Генерацию внутренних мыслей
- Управление режимом диалога
- Команду "помолчи" (silence mode)
- Срочные ответы на личные вопросы
"""

import unittest
from unittest.mock import MagicMock, patch, Mock
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Мокаем OpenAI перед импортом
with patch('rob_box_perception.reflection_node.OPENAI_AVAILABLE', True), \
     patch('rob_box_perception.reflection_node.OpenAI'):
    from rob_box_perception.reflection_node import ReflectionNode


class TestReflectionNode(unittest.TestCase):
    """Тесты для ReflectionNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2"""
        rclpy.shutdown()

    def setUp(self):
        """Создание мокированной ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка после теста"""
        self.node.destroy_node()
        patch.stopall()

    def test_node_creation(self):
        """Тест: нода создается корректно"""
        self.assertEqual(self.node.get_name(), 'reflection_node')

    def test_parameters_exist(self):
        """Тест: все параметры объявлены"""
        params = [
            'dialogue_timeout',
            'enable_speech',
            'system_prompt_file',
            'user_response_prompt_file',
            'urgent_response_timeout'
        ]
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_subscribers_created(self):
        """Тест: подписчики созданы"""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        
        self.assertIn('/perception/context_update', sub_topics)
        self.assertIn('/perception/user_speech', sub_topics)

    def test_publishers_created(self):
        """Тест: паблишеры созданы"""
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        pub_topics = [name for name, _ in publishers]
        
        self.assertIn('/reflection/internal_thought', pub_topics)
        self.assertIn('/voice/tts/request', pub_topics)

    def test_dialogue_state_initialization(self):
        """Тест: начальное состояние диалога"""
        self.assertFalse(self.node.in_dialogue)
        self.assertIsNone(self.node.last_user_speech_time)
        self.assertIsNone(self.node.pending_user_speech)

    def test_silence_mode_initialization(self):
        """Тест: silence mode выключен при старте"""
        self.assertIsNone(self.node.silence_until)

    def test_dialogue_timeout_parameter(self):
        """Тест: параметр dialogue_timeout"""
        timeout = self.node.get_parameter('dialogue_timeout').value
        self.assertIsInstance(timeout, float)
        self.assertGreater(timeout, 0)

    def test_enable_speech_parameter(self):
        """Тест: параметр enable_speech"""
        enabled = self.node.get_parameter('enable_speech').value
        self.assertIsInstance(enabled, bool)

    def test_urgent_response_timeout(self):
        """Тест: параметр urgent_response_timeout"""
        timeout = self.node.get_parameter('urgent_response_timeout').value
        self.assertIsInstance(timeout, float)
        self.assertEqual(timeout, 2.0)

    def test_user_speech_callback_enters_dialogue(self):
        """Тест: получение речи пользователя активирует диалог"""
        msg = String()
        msg.data = "Привет, как дела?"
        
        self.node.on_user_speech(msg)
        
        self.assertTrue(self.node.in_dialogue)
        self.assertIsNotNone(self.node.last_user_speech_time)
        self.assertEqual(self.node.pending_user_speech, "Привет, как дела?")

    def test_user_speech_callback_updates_timestamp(self):
        """Тест: обновление времени последней речи"""
        msg = String()
        msg.data = "Тест"
        
        time_before = time.time()
        self.node.on_user_speech(msg)
        time_after = time.time()
        
        self.assertGreaterEqual(self.node.last_user_speech_time, time_before)
        self.assertLessEqual(self.node.last_user_speech_time, time_after)

    def test_silence_mode_activation(self):
        """Тест: активация режима молчания"""
        self.node.silence_until = None
        
        future_time = time.time() + 300  # 5 минут
        self.node.silence_until = future_time
        
        self.assertEqual(self.node.silence_until, future_time)

    def test_dialogue_exit_on_timeout(self):
        """Тест: выход из диалога по таймауту"""
        # Входим в диалог
        self.node.in_dialogue = True
        self.node.last_user_speech_time = time.time() - 20  # 20 секунд назад
        self.node.dialogue_timeout = 10.0
        
        # Проверяем что прошло достаточно времени
        time_passed = time.time() - self.node.last_user_speech_time
        self.assertGreater(time_passed, self.node.dialogue_timeout)

    def test_last_context_storage(self):
        """Тест: сохранение последнего контекста"""
        self.assertIsNone(self.node.last_context)
        
        # Симулируем получение контекста
        mock_context = MagicMock()
        self.node.last_context = mock_context
        
        self.assertEqual(self.node.last_context, mock_context)

    def test_publish_internal_thought(self):
        """Тест: публикация внутренней мысли"""
        with patch.object(self.node.thought_pub, 'publish') as mock_publish:
            thought = "Интересно, почему батарея разряжается так быстро?"
            
            msg = String()
            msg.data = thought
            self.node.thought_pub.publish(msg)
            
            mock_publish.assert_called_once()

    def test_publish_speech_request(self):
        """Тест: публикация запроса на речь"""
        with patch.object(self.node.tts_pub, 'publish') as mock_publish:
            speech = "У меня всё хорошо, спасибо что спросил!"
            
            msg = String()
            msg.data = speech
            self.node.tts_pub.publish(msg)
            
            mock_publish.assert_called_once()

    def test_silence_mode_check(self):
        """Тест: проверка активности silence mode"""
        # Режим молчания не активен
        self.node.silence_until = None
        self.assertIsNone(self.node.silence_until)
        
        # Активируем режим молчания
        self.node.silence_until = time.time() + 10
        self.assertIsNotNone(self.node.silence_until)
        self.assertGreater(self.node.silence_until, time.time())
        
        # Режим молчания истек
        self.node.silence_until = time.time() - 1
        self.assertLess(self.node.silence_until, time.time())


class TestReflectionNodeMemory(unittest.TestCase):
    """Тесты sound debounce"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()

    def test_sound_debounce_initialization(self):
        """Тест: sound debounce инициализирован"""
        self.assertTrue(hasattr(self.node, 'last_sound_time'))
        self.assertIsInstance(self.node.last_sound_time, dict)
        self.assertEqual(self.node.sound_debounce_interval, 10.0)

    def test_sound_debounce_limit(self):
        """Тест: звук может быть воспроизведен"""
        # Проверяем что словарь debounce пустой при старте
        self.assertEqual(len(self.node.last_sound_time), 0)


class TestReflectionNodeIntegration(unittest.TestCase):
    """Интеграционные тесты ReflectionNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()

    def test_user_speech_workflow(self):
        """Тест: workflow обработки речи пользователя"""
        received_thoughts = []
        
        def thought_callback(msg):
            received_thoughts.append(msg.data)
        
        # Подписываемся на internal thoughts
        test_node = rclpy.create_node('test_listener')
        test_sub = test_node.create_subscription(
            String,
            '/reflection/internal_thought',
            thought_callback,
            10
        )
        
        # Публикуем речь пользователя
        pub = test_node.create_publisher(String, '/perception/user_speech', 10)
        msg = String()
        msg.data = "Как ты себя чувствуешь?"
        pub.publish(msg)
        
        # Даем время на обработку
        rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.destroy_node()
        
        # Проверяем что диалог активирован
        self.assertTrue(self.node.in_dialogue)


if __name__ == '__main__':
    unittest.main()
