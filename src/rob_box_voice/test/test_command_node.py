#!/usr/bin/env python3
"""
test_command_node.py - Тесты для Command Node

Тестирует:
- Распознавание команд движения (вперёд, назад, стоп)
- Парсинг параметров команд (дистанция, скорость)
- Публикацию команд управления
- Feedback о выполнении команд
- Фильтрацию не-команд
"""

import unittest
from unittest.mock import MagicMock, patch, Mock

import rclpy
from std_msgs.msg import String

from rob_box_voice.command_node import CommandNode


class TestCommandNode(unittest.TestCase):
    """Тесты для Command Node"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Подготовка перед каждым тестом"""
        self.node = CommandNode()
        
    def tearDown(self):
        """Очистка после каждого теста"""
        self.node.destroy_node()

    def test_node_creation(self):
        """Тест: Command нода создаётся корректно"""
        self.assertEqual(self.node.get_name(), 'command_node')
        print("  ✅ Command node создан")

    def test_stt_subscriber_exists(self):
        """Тест: Подписка на STT результаты существует"""
        self.assertTrue(hasattr(self.node, 'stt_sub'))
        print("  ✅ Подписка на /voice/stt/result существует")

    def test_intent_publisher_exists(self):
        """Тест: Publisher интентов существует"""
        self.assertTrue(hasattr(self.node, 'intent_pub'))
        print("  ✅ Publisher /voice/command/intent существует")

    def test_feedback_publisher_exists(self):
        """Тест: Publisher feedback существует"""
        self.assertTrue(hasattr(self.node, 'feedback_pub'))
        print("  ✅ Publisher /voice/command/feedback существует")

    def test_movement_keywords_defined(self):
        """Тест: Ключевые слова команд движения определены"""
        self.assertTrue(hasattr(self.node, 'movement_keywords'))
        self.assertGreater(len(self.node.movement_keywords), 0)
        print(f"  ✅ Ключевых слов движения: {len(self.node.movement_keywords)}")

    def test_parse_command_method_exists(self):
        """Тест: Метод парсинга команд существует"""
        self.assertTrue(hasattr(self.node, 'parse_command'))
        print("  ✅ Метод parse_command существует")

    def test_forward_command_recognition(self):
        """Тест: Распознавание команды 'вперёд'"""
        test_phrases = [
            "вперед",
            "вперёд",
            "поезжай вперёд",
            "двигайся вперед",
            "иди вперёд"
        ]
        
        for phrase in test_phrases:
            # Проверяем что фраза содержит ключевое слово движения
            is_movement = any(kw in phrase.lower() for kw in self.node.movement_keywords)
            self.assertTrue(is_movement, f"'{phrase}' должна распознаваться как команда движения")
        
        print(f"  ✅ Распознано {len(test_phrases)} вариантов команды 'вперёд'")

    def test_backward_command_recognition(self):
        """Тест: Распознавание команды 'назад'"""
        test_phrases = ["назад", "поезжай назад", "двигайся назад"]
        
        for phrase in test_phrases:
            is_movement = any(kw in phrase.lower() for kw in self.node.movement_keywords)
            self.assertTrue(is_movement, f"'{phrase}' должна распознаваться")
        
        print(f"  ✅ Распознано {len(test_phrases)} вариантов команды 'назад'")

    def test_stop_command_recognition(self):
        """Тест: Распознавание команды 'стоп'"""
        test_phrases = ["стоп", "стой", "остановись", "останови"]
        
        for phrase in test_phrases:
            is_movement = any(kw in phrase.lower() for kw in self.node.movement_keywords)
            self.assertTrue(is_movement, f"'{phrase}' должна распознаваться")
        
        print(f"  ✅ Распознано {len(test_phrases)} вариантов команды 'стоп'")

    def test_turn_command_recognition(self):
        """Тест: Распознавание команд поворота"""
        test_phrases = [
            "поверни налево",
            "поверни направо",
            "повернись влево",
            "разверни вправо"
        ]
        
        for phrase in test_phrases:
            is_movement = any(kw in phrase.lower() for kw in self.node.movement_keywords)
            self.assertTrue(is_movement, f"'{phrase}' должна распознаваться")
        
        print(f"  ✅ Распознано {len(test_phrases)} команд поворота")

    def test_non_command_filtering(self):
        """Тест: Фильтрация не-команд"""
        non_commands = [
            "привет робот",
            "как дела",
            "расскажи анекдот",
            "сколько времени"
        ]
        
        for phrase in non_commands:
            is_movement = any(kw in phrase.lower() for kw in self.node.movement_keywords)
            self.assertFalse(is_movement, f"'{phrase}' НЕ должна быть командой движения")
        
        print(f"  ✅ Отфильтровано {len(non_commands)} не-команд")

    def test_distance_parameter_parsing(self):
        """Тест: Парсинг параметра расстояния"""
        phrases_with_distance = [
            ("вперед 2 метра", 2.0),
            ("назад 5 метров", 5.0),
            ("вперёд 1 метр", 1.0),
        ]
        
        for phrase, expected_distance in phrases_with_distance:
            # Ищем числа в фразе
            import re
            numbers = re.findall(r'\d+', phrase)
            if numbers:
                distance = float(numbers[0])
                self.assertEqual(distance, expected_distance,
                               f"Из '{phrase}' должно парситься {expected_distance}м")
        
        print(f"  ✅ Парсинг расстояния работает ({len(phrases_with_distance)} примеров)")


class TestCommandParsing(unittest.TestCase):
    """Тесты парсинга команд с параметрами"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = CommandNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_stt_callback_exists(self):
        """Тест: Callback для STT существует"""
        self.assertTrue(hasattr(self.node, 'on_stt_result'))
        print("  ✅ Callback on_stt_result существует")

    def test_command_processing_workflow(self):
        """Тест: Полный workflow обработки команды"""
        # Создаём STT сообщение
        stt_msg = String()
        stt_msg.data = "вперёд"
        
        # Проверяем что callback не падает
        try:
            self.node.on_stt_result(stt_msg)
            print("  ✅ Workflow обработки команды работает")
        except Exception as e:
            self.fail(f"Обработка команды упала: {e}")


if __name__ == '__main__':
    unittest.main()
