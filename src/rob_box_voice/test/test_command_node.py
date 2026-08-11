#!/usr/bin/env python3
"""
test_command_node.py - Тесты для Command Node

Тестирует:
- classify_intent() - распознавание намерений
- Парсинг команд движения (вперёд, назад, повороты)
- Извлечение waypoints и направлений
- Обработку команд stop, status, map, vision
- Публикацию intent и feedback
"""

import unittest
from unittest.mock import MagicMock, patch, Mock
import math

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rob_box_voice.command_node import CommandNode, IntentType


class TestCommandNode(unittest.TestCase):
    """Тесты для Command Node."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом."""
        self.node = CommandNode()

    def tearDown(self):
        """Очистка после каждого теста."""
        self.node.destroy_node()

    def test_node_creation(self):
        """Тест: Command нода создаётся корректно."""
        self.assertEqual(self.node.get_name(), 'command_node')

    def test_stt_subscriber_exists(self):
        """Тест: Подписка на STT результаты существует."""
        self.assertTrue(hasattr(self.node, 'stt_sub'))

    def test_intent_publisher_exists(self):
        """Тест: Publisher интентов существует."""
        self.assertTrue(hasattr(self.node, 'intent_pub'))

    def test_feedback_publisher_exists(self):
        """Тест: Publisher feedback существует."""
        self.assertTrue(hasattr(self.node, 'feedback_pub'))

    def test_classify_intent_method_exists(self):
        """Тест: Метод classify_intent существует."""
        self.assertTrue(hasattr(self.node, 'classify_intent'))

    def test_patterns_defined(self):
        """Тест: Паттерны команд определены."""
        self.assertTrue(hasattr(self.node, 'patterns'))
        self.assertGreater(len(self.node.patterns), 0)


class TestClassifyIntent(unittest.TestCase):
    """Тесты classify_intent() - распознавание намерений."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = CommandNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_classify_forward_simple(self):
        """Тест: распознавание 'вперёд'."""
        cmd = self.node.classify_intent("вперёд")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('direction'), 'вперёд')

    def test_classify_forward_with_verb(self):
        """Тест: 'поезжай вперёд'."""
        cmd = self.node.classify_intent("поезжай вперёд")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('direction'), 'вперёд')

    def test_classify_backward(self):
        """Тест: 'назад'."""
        cmd = self.node.classify_intent("назад")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('direction'), 'назад')

    def test_classify_turn_left_simple(self):
        """Тест: 'налево' (без глагола)."""
        cmd = self.node.classify_intent("налево")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('direction'), 'налево')

    def test_classify_turn_right_with_verb(self):
        """Тест: 'поверни направо'."""
        cmd = self.node.classify_intent("поверни направо")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('direction'), 'направо')

    def test_classify_stop_simple(self):
        """Тест: 'стоп'."""
        cmd = self.node.classify_intent("стоп")
        self.assertEqual(cmd.intent, IntentType.STOP)

    def test_classify_stop_halt(self):
        """Тест: 'остановись'."""
        cmd = self.node.classify_intent("остановись")
        self.assertEqual(cmd.intent, IntentType.STOP)

    def test_classify_waypoint_number(self):
        """Тест: 'иди к точке 2'."""
        cmd = self.node.classify_intent("иди к точке 2")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('waypoint'), 'точка 2')

    def test_classify_waypoint_name(self):
        """Тест: 'поезжай к кухня'."""
        cmd = self.node.classify_intent("поезжай к кухня")
        self.assertEqual(cmd.intent, IntentType.NAVIGATE)
        self.assertEqual(cmd.entities.get('waypoint'), 'кухня')

    def test_classify_unknown(self):
        """Тест: неизвестная команда."""
        cmd = self.node.classify_intent("расскажи анекдот")
        self.assertEqual(cmd.intent, IntentType.UNKNOWN)

    def test_classify_status(self):
        """Тест: 'где ты'."""
        cmd = self.node.classify_intent("где ты")
        self.assertEqual(cmd.intent, IntentType.STATUS)


class TestCommandExecution(unittest.TestCase):
    """Тесты execute_command() - выполнение команд."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = CommandNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_execute_stop_command(self):
        """Тест: выполнение команды stop."""
        with patch.object(self.node, 'handle_stop') as mock_stop:
            cmd = self.node.classify_intent("стоп")
            self.node.execute_command(cmd)
            mock_stop.assert_called_once()

    def test_execute_navigate_command(self):
        """Тест: выполнение команды navigate."""
        with patch.object(self.node, 'handle_navigate') as mock_nav:
            cmd = self.node.classify_intent("вперёд")
            self.node.execute_command(cmd)
            mock_nav.assert_called_once()

    def test_execute_status_command(self):
        """Тест: выполнение команды status."""
        with patch.object(self.node, 'handle_status') as mock_status:
            cmd = self.node.classify_intent("где ты")
            self.node.execute_command(cmd)
            mock_status.assert_called_once()


class TestHandleDirection(unittest.TestCase):
    """Тесты handle_direction() - обработка направлений."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = CommandNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_handle_direction_forward(self):
        """Тест: направление 'вперёд'."""
        with patch.object(self.node, 'send_relative_nav2_goal') as mock_nav:
            self.node.handle_direction('вперёд')
            mock_nav.assert_called_once()
            args = mock_nav.call_args[0]
            self.assertEqual(args[0], 1.0)  # x=1.0м вперёд
            self.assertEqual(args[1], 0.0)  # y=0
            self.assertEqual(args[2], 0.0)  # theta=0

    def test_handle_direction_backward(self):
        """Тест: направление 'назад'."""
        with patch.object(self.node, 'send_relative_nav2_goal') as mock_nav:
            self.node.handle_direction('назад')
            mock_nav.assert_called_once()
            args = mock_nav.call_args[0]
            self.assertEqual(args[0], -1.0)  # x=-1.0м назад

    def test_handle_direction_turn_left(self):
        """Тест: поворот 'налево'."""
        with patch.object(self.node, 'send_relative_nav2_goal') as mock_nav:
            self.node.handle_direction('налево')
            mock_nav.assert_called_once()
            args = mock_nav.call_args[0]
            self.assertAlmostEqual(args[2], math.pi/2, places=5)  # theta=90°

    def test_handle_direction_turn_right(self):
        """Тест: поворот 'направо'."""
        with patch.object(self.node, 'send_relative_nav2_goal') as mock_nav:
            self.node.handle_direction('направо')
            mock_nav.assert_called_once()
            args = mock_nav.call_args[0]
            self.assertAlmostEqual(args[2], -math.pi/2, places=5)  # theta=-90°

    def test_handle_direction_unknown(self):
        """Тест: неизвестное направление."""
        with patch.object(self.node, 'publish_feedback') as mock_feedback:
            self.node.handle_direction('куда-то')
            mock_feedback.assert_called_once()


class TestPublishMethods(unittest.TestCase):
    """Тесты publish_intent() и publish_feedback()."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = CommandNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_publish_intent(self):
        """Тест: публикация intent."""
        with patch.object(self.node.intent_pub, 'publish') as mock_pub:
            cmd = self.node.classify_intent("стоп")
            self.node.publish_intent(cmd)
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            self.assertIn('stop', msg.data)

    def test_publish_feedback(self):
        """Тест: публикация feedback."""
        with patch.object(self.node.feedback_pub, 'publish') as mock_pub:
            self.node.publish_feedback("Тестовый feedback")
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            self.assertEqual(msg.data, "Тестовый feedback")


class TestSttCallback(unittest.TestCase):
    """Тесты stt_callback() - обработка STT результатов."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        self.node = CommandNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_stt_callback_strips_wake_word(self):
        """Тест: удаление wake word из команды."""
        with patch.object(self.node, 'classify_intent', wraps=self.node.classify_intent) as mock_classify:
            msg = String()
            msg.data = "робот вперёд"
            self.node.stt_callback(msg)
            # Должен передать "вперёд" без "робот"
            mock_classify.assert_called_once_with("вперёд")

    def test_stt_callback_empty_string(self):
        """Тест: пустая строка игнорируется."""
        with patch.object(self.node, 'classify_intent') as mock_classify:
            msg = String()
            msg.data = ""
            self.node.stt_callback(msg)
            mock_classify.assert_not_called()

    def test_stt_callback_unknown_intent(self):
        """Тест: UNKNOWN intent публикуется но не выполняется."""
        with patch.object(self.node, 'execute_command') as mock_execute:
            with patch.object(self.node, 'publish_intent') as mock_pub_intent:
                msg = String()
                msg.data = "расскажи анекдот"
                self.node.stt_callback(msg)
                # Intent публикуется
                mock_pub_intent.assert_called_once()
                # Но команда НЕ выполняется
                mock_execute.assert_not_called()

    def test_stt_callback_normal_confidence(self):
        """Тест: нормальная уверенность → выполнение команды."""
        with patch.object(self.node, 'execute_command') as mock_execute:
            msg = String()
            msg.data = "стоп"
            self.node.stt_callback(msg)
            # Команда выполняется
            mock_execute.assert_called_once()


if __name__ == '__main__':
    unittest.main()
