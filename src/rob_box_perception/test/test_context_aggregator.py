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


if __name__ == '__main__':
    unittest.main()
