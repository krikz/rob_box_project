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

    @patch('builtins.print')
    def test_status_healthy(self, mock_print):
        """Тест: Статус HEALTHY (нет критичных ошибок, <5 за минуту)"""
        # Добавляем 2 некритичных ошибки
        for i in range(2):
            log_msg = Log()
            log_msg.level = 40  # ERROR (не FATAL)
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
        
        # Вызываем отчёт
        self.node.print_report()
        
        # Проверяем что статус HEALTHY
        calls = ' '.join([str(call) for call in mock_print.call_args_list])
        self.assertIn('HEALTHY', calls)

    @patch('builtins.print')
    def test_status_degraded(self, mock_print):
        """Тест: Статус DEGRADED (5+ ошибок за последнюю минуту)"""
        # Добавляем 6 ошибок с текущим временем
        current_time = time.time()
        for i in range(6):
            log_msg = Log()
            log_msg.level = 40  # ERROR
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Recent error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
            # Убеждаемся что время меньше 60 сек назад
            self.node.errors[-1]['time'] = current_time
        
        # Вызываем отчёт
        self.node.print_report()
        
        # Проверяем статус DEGRADED
        calls = ' '.join([str(call) for call in mock_print.call_args_list])
        self.assertIn('DEGRADED', calls)

    @patch('builtins.print')
    def test_status_critical(self, mock_print):
        """Тест: Статус CRITICAL (есть FATAL ошибка)"""
        # Добавляем FATAL ошибку
        log_msg = Log()
        log_msg.level = 50  # FATAL
        log_msg.name = 'critical_node'
        log_msg.msg = 'Fatal crash!'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        self.node.on_log(log_msg)
        
        # Вызываем отчёт
        self.node.print_report()
        
        # Проверяем статус CRITICAL
        calls = ' '.join([str(call) for call in mock_print.call_args_list])
        self.assertIn('CRITICAL', calls)

    @patch('builtins.print')
    def test_sound_trigger_on_status_change_to_critical(self, mock_print):
        """Тест: Звуковой сигнал при переходе в CRITICAL"""
        # Mock sound publisher
        self.node.sound_pub.publish = MagicMock()
        
        # Сначала здоровая система
        self.node.print_report()
        self.assertEqual(self.node.last_status, '✅ HEALTHY')
        
        # Теперь добавляем FATAL ошибку
        log_msg = Log()
        log_msg.level = 50
        log_msg.name = 'crash_node'
        log_msg.msg = 'System crash'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        self.node.on_log(log_msg)
        
        # Триггерим новый отчёт
        self.node.print_report()
        
        # Проверяем что статус изменился
        self.assertEqual(self.node.last_status, '🚨 CRITICAL')
        
        # Проверяем что был вызван звук 'angry_2'
        self.assertTrue(self.node.sound_pub.publish.called)
        published_msg = self.node.sound_pub.publish.call_args[0][0]
        self.assertEqual(published_msg.data, 'angry_2')

    @patch('builtins.print')
    def test_sound_trigger_on_status_change_to_degraded(self, mock_print):
        """Тест: Звуковой сигнал при переходе в DEGRADED"""
        self.node.sound_pub.publish = MagicMock()
        
        # Сначала здоровая система
        self.node.print_report()
        
        # Добавляем 5+ ошибок за последнюю минуту
        current_time = time.time()
        for i in range(6):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
            self.node.errors[-1]['time'] = current_time
        
        # Триггерим отчёт
        self.node.print_report()
        
        # Проверяем звук 'confused'
        self.assertEqual(self.node.last_status, '⚠️  DEGRADED')
        published_msg = self.node.sound_pub.publish.call_args[0][0]
        self.assertEqual(published_msg.data, 'confused')

    @patch('builtins.print')
    def test_sound_trigger_on_recovery(self, mock_print):
        """Тест: Звуковой сигнал при восстановлении (DEGRADED → HEALTHY)"""
        self.node.sound_pub.publish = MagicMock()
        
        # Сначала добавляем 5+ ошибок (DEGRADED)
        current_time = time.time()
        for i in range(6):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'node_{i}'
            log_msg.msg = f'Error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
            self.node.errors[-1]['time'] = current_time
        
        self.node.print_report()  # Устанавливаем DEGRADED
        self.assertEqual(self.node.last_status, '⚠️  DEGRADED')
        
        # Теперь "стареем" ошибки (делаем их старше 60 сек)
        old_time = current_time - 120  # 2 минуты назад
        for error in self.node.errors:
            error['time'] = old_time
        
        # Сбрасываем mock
        self.node.sound_pub.publish.reset_mock()
        
        # Триггерим новый отчёт - должен показать HEALTHY
        self.node.print_report()
        
        # Проверяем восстановление и звук 'cute'
        self.assertEqual(self.node.last_status, '✅ HEALTHY')
        self.assertTrue(self.node.sound_pub.publish.called)
        published_msg = self.node.sound_pub.publish.call_args[0][0]
        self.assertEqual(published_msg.data, 'cute')

    def test_sound_disabled(self):
        """Тест: Звуки не проигрываются когда enable_sounds=False"""
        # Создаём ноду с отключенными звуками
        self.node.destroy_node()
        self.node = HealthMonitor()
        self.node.enable_sounds = False
        self.node.sound_pub.publish = MagicMock()
        
        # Добавляем FATAL ошибку
        log_msg = Log()
        log_msg.level = 50
        log_msg.name = 'crash_node'
        log_msg.msg = 'Fatal error'
        log_msg.stamp = self.node.get_clock().now().to_msg()
        self.node.on_log(log_msg)
        
        # Триггерим отчёт
        with patch('builtins.print'):
            self.node.print_report()
        
        # Звук НЕ должен быть вызван
        self.assertFalse(self.node.sound_pub.publish.called)

    def test_play_sound_exception_handling(self):
        """Тест: Обработка ошибок в _play_sound()"""
        # Мокаем publisher чтобы выбросить исключение
        self.node.sound_pub.publish = MagicMock(side_effect=Exception('Publish failed'))
        
        # Вызываем _play_sound - не должен упасть
        try:
            self.node._play_sound('test_sound')
        except Exception:
            self.fail('_play_sound() should not raise exception')

    @patch('builtins.print')
    def test_recent_errors_calculation(self, mock_print):
        """Тест: Подсчёт ошибок за последние 60 секунд"""
        current_time = time.time()
        
        # Добавляем 3 свежих ошибки (меньше минуты назад)
        for i in range(3):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'recent_node_{i}'
            log_msg.msg = f'Recent error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
            self.node.errors[-1]['time'] = current_time - 30  # 30 сек назад
        
        # Добавляем 2 старые ошибки (больше минуты назад)
        for i in range(2):
            log_msg = Log()
            log_msg.level = 40
            log_msg.name = f'old_node_{i}'
            log_msg.msg = f'Old error {i}'
            log_msg.stamp = self.node.get_clock().now().to_msg()
            self.node.on_log(log_msg)
            self.node.errors[-1]['time'] = current_time - 120  # 2 минуты назад
        
        # Вызываем отчёт
        self.node.print_report()
        
        # Проверяем что в выводе указано только 3 свежих ошибки
        calls = ' '.join([str(call) for call in mock_print.call_args_list])
        # Должно быть: "Total Errors: 5 (последние 3 за минуту)"
        self.assertIn('3', calls)  # 3 свежих ошибки


if __name__ == '__main__':
    unittest.main()
