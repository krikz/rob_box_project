#!/usr/bin/env python3
"""
test_health_monitor.py - Unit тесты для HealthMonitor

Тестирует:
- Обработку логов ERROR/WARN
- Фильтрацию и ограничение истории
- Публикацию звуковых уведомлений
- Отчёты о здоровье системы
"""

import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from rob_box_perception.health_monitor import HealthMonitor


class TestHealthMonitor(unittest.TestCase):
    """Тесты для HealthMonitor"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS2 один раз для всех тестов"""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS2"""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом"""
        self.node = HealthMonitor()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_initialization(self):
        """Тест: Нода инициализируется корректно"""
        self.assertEqual(self.node.get_name(), 'health_monitor')
        self.assertIsNotNone(self.node.rosout_sub)
        self.assertIsNotNone(self.node.sound_pub)
        self.assertEqual(len(self.node.errors), 0)
        self.assertEqual(len(self.node.warnings), 0)

    def test_error_log_handling(self):
        """Тест: Обработка ERROR логов"""
        # Создаём ERROR лог
        log_msg = Log()
        log_msg.level = 40  # ERROR
        log_msg.name = 'test_node'
        log_msg.msg = 'Test error message'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        
        # Отправляем лог
        self.node.on_log(log_msg)
        
        # Проверяем что ошибка сохранена
        self.assertEqual(len(self.node.errors), 1)
        self.assertEqual(self.node.errors[0]['node'], 'test_node')
        self.assertEqual(self.node.errors[0]['level'], 'ERROR')
        self.assertEqual(self.node.errors[0]['msg'], 'Test error message')

    def test_fatal_log_handling(self):
        """Тест: Обработка FATAL логов"""
        log_msg = Log()
        log_msg.level = 50  # FATAL
        log_msg.name = 'critical_node'
        log_msg.msg = 'Fatal error!'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        
        self.node.on_log(log_msg)
        
        self.assertEqual(len(self.node.errors), 1)
        self.assertEqual(self.node.errors[0]['level'], 'FATAL')

    def test_warning_log_handling(self):
        """Тест: Обработка WARN логов"""
        log_msg = Log()
        log_msg.level = 30  # WARN
        log_msg.name = 'test_node'
        log_msg.msg = 'Warning message'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        
        self.node.on_log(log_msg)
        
        self.assertEqual(len(self.node.warnings), 1)
        self.assertEqual(self.node.warnings[0]['node'], 'test_node')

    def test_info_log_ignored(self):
        """Тест: INFO логи игнорируются"""
        log_msg = Log()
        log_msg.level = 20  # INFO
        log_msg.name = 'test_node'
        log_msg.msg = 'Info message'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        
        self.node.on_log(log_msg)
        
        # INFO не должны попадать в errors или warnings
        self.assertEqual(len(self.node.errors), 0)
        self.assertEqual(len(self.node.warnings), 0)

    def test_error_history_limit(self):
        """Тест: Ограничение истории ошибок (макс 20)"""
        # Генерируем 25 ошибок
        for i in range(25):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
        
        # Должно остаться только 20 последних
        self.assertEqual(len(self.node.errors), 20)
        # Проверяем что это последние (с индексами 5-24)
        self.assertEqual(self.node.errors[0]['node'], 'node_5')
        self.assertEqual(self.node.errors[-1]['node'], 'node_24')

    def test_warning_history_limit(self):
        """Тест: Ограничение истории предупреждений (макс 10)"""
        # Генерируем 15 предупреждений
        for i in range(15):
            log_msg = Log()
            log_msg.level = 30
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Warning {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
        
        # Должно остаться только 10 последних
        self.assertEqual(len(self.node.warnings), 10)
        self.assertEqual(self.node.warnings[0]['node'], 'node_5')
        self.assertEqual(self.node.warnings[-1]['node'], 'node_14')

    def test_multiple_log_types(self):
        """Тест: Обработка смешанных типов логов"""
        logs = [
            (20, 'INFO'),    # Игнорируется
            (30, 'WARN'),    # В warnings
            (40, 'ERROR'),   # В errors
            (20, 'INFO'),    # Игнорируется
            (50, 'FATAL'),   # В errors
            (30, 'WARN'),    # В warnings
        ]
        
        for level, level_name in logs:
            log_msg = Log()
            log_msg.level = level
            log_msg.name = 'test_node'
            log_msg.msg = f'{level_name} message'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
        
        # 2 ошибки (ERROR + FATAL)
        self.assertEqual(len(self.node.errors), 2)
        # 2 предупреждения
        self.assertEqual(len(self.node.warnings), 2)

    @patch('builtins.print')
    def test_print_report(self, mock_print):
        """Тест: Печать отчёта"""
        # Добавляем несколько ошибок
        for i in range(3):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
        
        # Вызываем отчёт
        self.node.print_report()
        
        # Проверяем что print был вызван (отчёт напечатан)
        self.assertTrue(mock_print.called)
        # Проверяем что в выводе есть заголовок отчёта
        calls = [str(call) for call in mock_print.call_args_list]
        self.assertTrue(any('HEALTH REPORT' in str(call) for call in calls))

    def test_parameters(self):
        """Тест: Параметры ноды"""
        # Проверяем что параметр enable_sounds существует
        self.assertTrue(self.node.has_parameter('enable_sounds'))
        # По умолчанию должен быть True
        self.assertTrue(self.node.enable_sounds)


if __name__ == '__main__':
    unittest.main()
