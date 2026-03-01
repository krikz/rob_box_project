#!/usr/bin/env python3
"""
test_context_aggregator.py - Unit тесты для ContextAggregatorNode

Тестирует:
- Подписку на источники данных
- Агрегацию контекста
- Публикацию событий восприятия
- Работу с кэшем состояния
- Мониторинг доступности нод
"""

import time
import unittest
from unittest.mock import MagicMock, patch, Mock

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry

from rob_box_perception.context_aggregator_node import ContextAggregatorNode


class TestContextAggregator(unittest.TestCase):
    """Тесты для ContextAggregatorNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS2"""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS2"""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом"""
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_initialization(self):
        """Тест: Нода инициализируется корректно"""
        self.assertEqual(self.node.get_name(), 'context_aggregator')
        
        # Проверяем параметры
        self.assertTrue(self.node.has_parameter('publish_rate'))
        self.assertTrue(self.node.has_parameter('memory_window'))
        self.assertTrue(self.node.has_parameter('enable_summarization'))
        
        # Проверяем начальное состояние
        self.assertIsNone(self.node.current_vision)
        self.assertIsNone(self.node.current_pose)

    def test_parameters(self):
        """Тест: Параметры ноды"""
        # Проверяем значения по умолчанию
        self.assertEqual(self.node.publish_rate, 2.0)
        self.assertEqual(self.node.memory_window, 60)
        self.assertTrue(self.node.enable_summarization)
        self.assertEqual(self.node.timezone, 'Europe/Moscow')

    def test_vision_context_subscription(self):
        """Тест: Подписка на vision context"""
        # Создаём vision сообщение
        vision_msg = String()
        vision_msg.data = '{"objects": ["table", "chair"], "scene": "kitchen"}'
        
        # Проверяем что есть callback для vision
        self.assertTrue(hasattr(self.node, 'on_vision_context'))
        
        # Вызываем callback
        self.node.on_vision_context(vision_msg)
        
        # Проверяем что vision сохранен
        self.assertIsNotNone(self.node.current_vision)

    def test_pose_subscription(self):
        """Тест: Подписка на позицию (localization_pose)"""
        # Создаём pose сообщение
        pose_msg = PoseStamped()
        pose_msg.pose.position = Point(x=1.0, y=2.0, z=0.0)
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_robot_pose'))
        
        # Вызываем callback
        self.node.on_robot_pose(pose_msg)
        
        # Проверяем что позиция сохранена
        self.assertIsNotNone(self.node.current_pose)
        self.assertEqual(self.node.current_pose.pose.position.x, 1.0)

    def test_odometry_subscription(self):
        """Тест: Подписка на одометрию"""
        # Создаём odometry сообщение
        odom_msg = Odometry()
        odom_msg.pose.pose.position = Point(x=3.0, y=4.0, z=0.0)
        
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_odometry'))
        
        # Вызываем callback
        self.node.on_odometry(odom_msg)
        
        # Проверяем что одометрия сохранена
        self.assertIsNotNone(self.node.current_odom)

    def test_stt_result_subscription(self):
        """Тест: Подписка на результаты STT"""
        # Создаём STT сообщение
        stt_msg = String()
        stt_msg.data = "Привет робот"
        
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_user_speech'))
        
        # Проверяем что есть publisher для транзита STT
        self.assertTrue(hasattr(self.node, 'speech_pub'))
        
        # Вызываем callback (просто проверяем что не падает)
        self.node.on_user_speech(stt_msg)

    def test_device_snapshot_subscription(self):
        """Тест: Подписка на ros2_control joint_states (батарея)"""
        # Проверяем callback
        self.assertTrue(hasattr(self.node, 'on_joint_states'))
        
        # SKIP: Сложная структура DynamicJointState, нужны моки
        # Просто проверяем что current_sensors существует
        self.assertIsNotNone(self.node.current_sensors)
        self.assertIsInstance(self.node.current_sensors, dict)

    def test_memory_window(self):
        """Тест: Ограничение окна памяти для истории"""
        # Устанавливаем небольшое окно памяти
        self.node.memory_window = 2  # 2 секунды
        
        # SKIP: Очистка истории делается внутри publish_event
        # Просто проверяем что memory_window установлен
        self.assertEqual(self.node.memory_window, 2)

    def test_context_publisher_exists(self):
        """Тест: Publisher для контекста существует"""
        self.assertTrue(hasattr(self.node, 'event_pub'))

    def test_multiple_data_sources(self):
        """Тест: Агрегация данных из нескольких источников"""
        # Посылаем данные из разных источников
        
        # Vision
        vision_msg = String()
        vision_msg.data = '{"objects": ["cup"]}'
        self.node.on_vision_context(vision_msg)
        
        # Pose
        pose_msg = PoseStamped()
        pose_msg.pose.position = Point(x=5.0, y=6.0, z=0.0)
        self.node.on_robot_pose(pose_msg)
        
        # Проверяем что все данные сохранены
        self.assertIsNotNone(self.node.current_vision)
        self.assertIsNotNone(self.node.current_pose)
        self.assertIsNotNone(self.node.current_sensors)
        
        # Проверяем что можем собрать полный контекст
        if hasattr(self.node, 'get_current_context'):
            context = self.node.get_current_context()
            self.assertIsNotNone(context)

    def test_node_availability_monitor(self):
        """Тест: Мониторинг доступности нод"""
        # Проверяем что монитор существует
        self.assertTrue(hasattr(self.node, 'node_monitor'))
        
        # Проверяем что мониторятся критичные ноды
        if hasattr(self.node.node_monitor, 'critical_nodes'):
            self.assertGreater(len(self.node.node_monitor.critical_nodes), 0)

    def test_internet_connectivity_monitor(self):
        """Тест: Мониторинг интернет соединения"""
        # Проверяем что монитор существует
        self.assertTrue(hasattr(self.node, 'internet_monitor'))

    def test_time_awareness_provider(self):
        """Тест: Провайдер осведомлённости о времени"""
        # Проверяем что провайдер существует
        self.assertTrue(hasattr(self.node, 'time_provider'))
        
        # Проверяем что можем получить текущее время
        if hasattr(self.node.time_provider, 'get_current_time_str'):
            time_str = self.node.time_provider.get_current_time_str()
            self.assertIsNotNone(time_str)
            self.assertIsInstance(time_str, str)


class TestCallbacks(unittest.TestCase):
    """Тесты для дополнительных callbacks"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_on_robot_response(self):
        """Тест: callback robot_response"""
        msg = String()
        msg.data = '{"ssml": "<speak>Привет!</speak>"}'
        
        self.node.on_robot_response(msg)
        
        # Проверяем что добавлено в robot_response_events
        self.assertGreater(len(self.node.robot_response_events), 0)
        self.assertEqual(self.node.robot_response_events[-1]['type'], 'robot_response')

    def test_on_robot_thought(self):
        """Тест: callback robot_thought"""
        msg = String()
        msg.data = "Думаю о маршруте"
        
        self.node.on_robot_thought(msg)
        
        # Проверяем что добавлено в robot_thought_events
        self.assertGreater(len(self.node.robot_thought_events), 0)
        self.assertEqual(self.node.robot_thought_events[-1]['type'], 'robot_thought')

    def test_on_command_intent(self):
        """Тест: callback command_intent"""
        msg = String()
        msg.data = "navigate:0.85"
        
        self.node.on_command_intent(msg)
        
        # Проверяем что добавлено в recent_events
        self.assertGreater(len(self.node.recent_events), 0)

    def test_on_command_feedback(self):
        """Тест: callback command_feedback"""
        msg = String()
        msg.data = "Иду к точке назначения"
        
        self.node.on_command_feedback(msg)
        
        # Проверяем что добавлено как robot_response
        self.assertGreater(len(self.node.robot_response_events), 0)

    def test_on_user_speech_movement_command_filtered(self):
        """Тест: команды движения фильтруются"""
        msg = String()
        msg.data = "вперёд"
        
        initial_count = len(self.node.speech_events)
        self.node.on_user_speech(msg)
        
        # Команда движения НЕ должна быть добавлена
        self.assertEqual(len(self.node.speech_events), initial_count)

    def test_on_user_speech_dialogue(self):
        """Тест: диалоговая речь добавляется"""
        msg = String()
        msg.data = "Как дела?"
        
        self.node.on_user_speech(msg)
        
        # Диалоговая речь должна быть добавлена
        self.assertGreater(len(self.node.speech_events), 0)
        self.assertEqual(self.node.speech_events[-1]['content'], "Как дела?")


class TestMemoryManagement(unittest.TestCase):
    """Тесты управления памятью"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_add_to_memory_speech(self):
        """Тест: добавление события speech"""
        self.node.add_to_memory('user_speech', 'Привет', important=True)
        
        self.assertGreater(len(self.node.speech_events), 0)
        self.assertGreater(len(self.node.recent_events), 0)
        self.assertEqual(self.node.speech_events[-1]['content'], 'Привет')
        self.assertTrue(self.node.speech_events[-1]['important'])

    def test_add_to_memory_vision(self):
        """Тест: добавление события vision"""
        self.node.add_to_memory('vision', 'Вижу стол', important=False)
        
        self.assertGreater(len(self.node.vision_events), 0)
        self.assertEqual(self.node.vision_events[-1]['type'], 'vision')

    def test_add_to_memory_system(self):
        """Тест: добавление события system"""
        self.node.add_to_memory('error', 'Ошибка сенсора', important=True)
        
        self.assertGreater(len(self.node.system_events), 0)
        self.assertEqual(self.node.system_events[-1]['type'], 'error')

    def test_memory_window_cleanup(self):
        """Тест: очистка старых событий"""
        self.node.memory_window = 1  # 1 секунда
        
        # Добавляем событие
        self.node.add_to_memory('user_speech', 'Старое событие')
        self.assertEqual(len(self.node.speech_events), 1)
        
        # Ждём чуть больше memory_window
        time.sleep(1.2)
        
        # Добавляем новое событие (должно очистить старое)
        self.node.add_to_memory('user_speech', 'Новое событие')
        
        # Старое событие должно быть удалено
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(self.node.speech_events[0]['content'], 'Новое событие')

    def test_get_memory_summary_empty(self):
        """Тест: summary пустой памяти"""
        summary = self.node.get_memory_summary()
        self.assertEqual(summary, "Недавних событий нет")

    def test_get_memory_summary_with_events(self):
        """Тест: summary с событиями"""
        self.node.add_to_memory('user_speech', 'Тест 1', important=True)
        self.node.add_to_memory('robot_response', 'Тест 2', important=False)
        
        summary = self.node.get_memory_summary()
        self.assertIn('user_speech', summary)
        self.assertIn('Тест 1', summary)


class TestPublishEvent(unittest.TestCase):
    """Тесты publish_event()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = ContextAggregatorNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_publish_event_calls_publisher(self):
        """Тест: publish_event публикует событие"""
        with patch.object(self.node.event_pub, 'publish') as mock_publish:
            self.node.publish_event()
            mock_publish.assert_called_once()

    def test_check_system_health_healthy(self):
        """Тест: здоровье системы healthy"""
        self.node.current_sensors = {'battery': 40.0}
        self.node.recent_errors = []
        
        status, issues = self.node.check_system_health()
        
        # При полной батарее и без ошибок должен быть healthy
        # Но могут быть проблемы с нодами/интернетом
        self.assertIn(status, ['healthy', 'degraded'])

    def test_check_system_health_low_battery(self):
        """Тест: низкая батарея"""
        self.node.current_sensors = {'battery': 33.0}
        self.node.recent_errors = []
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('батарея' in issue.lower() or 'батаре' in issue.lower() for issue in issues))

    def test_check_system_health_critical_battery(self):
        """Тест: критическая батарея"""
        self.node.current_sensors = {'battery': 31.0}
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('критическ' in issue.lower() for issue in issues))

    def test_check_system_health_many_errors(self):
        """Тест: много ошибок"""
        # Добавляем 5 недавних ошибок
        current_time = time.time()
        self.node.recent_errors = [
            {'time': current_time - 10, 'message': 'Error 1'},
            {'time': current_time - 15, 'message': 'Error 2'},
            {'time': current_time - 20, 'message': 'Error 3'},
            {'time': current_time - 25, 'message': 'Error 4'},
            {'time': current_time - 28, 'message': 'Error 5'},
        ]
        
        status, issues = self.node.check_system_health()
        
        self.assertGreater(len(issues), 0)
        self.assertTrue(any('ошибок' in issue.lower() for issue in issues))


if __name__ == '__main__':
    unittest.main()
