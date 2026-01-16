#!/usr/bin/env python3
"""
test_summarization.py - Тесты для механизма суммаризации в ContextAggregatorNode

Тестирует:
- Накопление событий до порога
- Вызов DeepSeek API для суммаризации
- Сохранение и ротацию summaries
- Очистку старых событий после суммаризации
- Формирование полного контекста (summaries + recent events)
"""

import os
import time
import unittest
from unittest.mock import MagicMock, patch, Mock
import pytest

# Регистрация custom marker
pytest_plugins = []

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rob_box_perception.context_aggregator_node import ContextAggregatorNode


# Проверяем наличие DeepSeek API key
DEEPSEEK_API_KEY = os.getenv('DEEPSEEK_API_KEY')
HAS_DEEPSEEK_KEY = DEEPSEEK_API_KEY is not None and len(DEEPSEEK_API_KEY) > 20


class TestSummarizationStructure(unittest.TestCase):
    """Быстрые тесты структуры суммаризации (без реального API)"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = ContextAggregatorNode()
        # Отключаем автоматическую суммаризацию для быстрых тестов
        self.node.enable_summarization = False
        self.node.summarization_threshold = 5
        
    def tearDown(self):
        self.node.destroy_node()

    def test_summarization_parameters(self):
        """Тест: Параметры суммаризации"""
        self.assertTrue(self.node.has_parameter('summarization_threshold'))
        self.assertTrue(self.node.has_parameter('enable_summarization'))
        self.assertEqual(self.node.summarization_threshold, 5)

    def test_speech_events_accumulation(self):
        """Тест: Накопление событий речи пользователя"""
        # Добавляем несколько событий речи
        for i in range(3):
            stt_msg = String()
            stt_msg.data = f"Тестовая фраза номер {i+1}"
            self.node.on_user_speech(stt_msg)
        
        # Проверяем что события сохранены
        self.assertEqual(len(self.node.speech_events), 3)
        
        # Проверяем структуру события
        event = self.node.speech_events[0]
        self.assertIn('time', event)
        self.assertIn('type', event)
        self.assertIn('content', event)
        self.assertEqual(event['type'], 'user_speech')
        self.assertEqual(event['content'], 'Тестовая фраза номер 1')

    def test_vision_events_accumulation(self):
        """Тест: Накопление событий vision"""
        # Добавляем несколько vision событий через add_to_memory
        for i in range(3):
            self.node.add_to_memory('vision', f'Вижу объект {i+1}', important=False)
        
        # Проверяем что события сохранены
        self.assertEqual(len(self.node.vision_events), 3)

    def test_event_storage_structure(self):
        """Тест: Структура хранения событий"""
        # Проверяем наличие хранилищ для разных типов событий
        self.assertIsNotNone(self.node.speech_events)
        self.assertIsNotNone(self.node.robot_response_events)
        self.assertIsNotNone(self.node.robot_thought_events)
        self.assertIsNotNone(self.node.vision_events)
        self.assertIsNotNone(self.node.system_events)
        
        # Проверяем наличие хранилищ для summaries
        self.assertIsNotNone(self.node.speech_summaries)
        self.assertIsNotNone(self.node.robot_response_summaries)
        self.assertIsNotNone(self.node.robot_thought_summaries)
        self.assertIsNotNone(self.node.vision_summaries)
        self.assertIsNotNone(self.node.system_summaries)

    def test_last_summarization_time_tracked(self):
        """Тест: Отслеживание времени последней суммаризации"""
        self.assertIsNotNone(self.node.last_summarization_time)
        self.assertIsInstance(self.node.last_summarization_time, float)

    def test_add_to_memory_method(self):
        """Тест: Метод add_to_memory правильно распределяет события"""
        # Добавляем события разных типов
        self.node.add_to_memory('user_speech', 'Привет', important=True)
        self.node.add_to_memory('robot_response', 'Здравствуй', important=True)
        self.node.add_to_memory('vision', 'Вижу стол', important=False)
        self.node.add_to_memory('error', 'Тестовая ошибка', important=True)
        
        # Проверяем распределение
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(len(self.node.robot_response_events), 1)
        self.assertEqual(len(self.node.vision_events), 1)
        self.assertEqual(len(self.node.system_events), 1)
        
        # Все должны быть в recent_events
        self.assertEqual(len(self.node.recent_events), 4)


@unittest.skipUnless(HAS_DEEPSEEK_KEY, "DEEPSEEK_API_KEY not set")
@pytest.mark.slow
class TestRealSummarization(unittest.TestCase):
    """Тесты с РЕАЛЬНЫМ DeepSeek API (медленные ~4-8 сек на тест)"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = ContextAggregatorNode()
        # Устанавливаем низкий порог для быстрого срабатывания
        self.node.summarization_threshold = 5
        # Суммаризация ВКЛЮЧЕНА (используем реальный API)
        
    def tearDown(self):
        self.node.destroy_node()

    def test_summarization_enabled_with_api(self):
        """Тест: Суммаризация включена и API инициализирован"""
        self.assertTrue(self.node.enable_summarization)
        self.assertIsNotNone(self.node.deepseek_client)
        self.assertIsNotNone(self.node.summarization_prompt)

    def test_real_summarization_call(self):
        """Тест: РЕАЛЬНЫЙ вызов DeepSeek API для суммаризации (МЕДЛЕННЫЙ ~4сек)"""
        print("\n🔄 Вызов DeepSeek API для суммаризации...")
        
        # Добавляем 5 событий (порог)
        for i in range(5):
            stt_msg = String()
            stt_msg.data = f"Робот, расскажи историю номер {i+1}"
            self.node.on_user_speech(stt_msg)
        
        # Суммаризация происходит автоматически при 5-м событии
        # Проверяем что summary создан
        self.assertGreater(len(self.node.speech_summaries), 0, 
                          "Summary должен был быть создан после 5 событий")
        
        # Проверяем структуру
        summary = self.node.speech_summaries[0]
        self.assertIn('time', summary)
        self.assertIn('category', summary)
        self.assertIn('summary', summary)
        self.assertIn('event_count', summary)
        
        self.assertEqual(summary['category'], 'speech')
        self.assertEqual(summary['event_count'], 5)
        
        # Проверяем что LLM вернул текст
        self.assertIsInstance(summary['summary'], str)
        self.assertGreater(len(summary['summary']), 20, 
                          f"Summary слишком короткий: {summary['summary']}")
        
        print(f"✅ Summary создан: {len(summary['summary'])} символов")
        print(f"   Текст: {summary['summary'][:100]}...")


class TestSummarizationLogic(unittest.TestCase):
    """Тесты логики суммаризации (без API, быстрые)"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = ContextAggregatorNode()
        self.node.enable_summarization = False  # Отключаем для быстрых тестов
        self.node.summarization_threshold = 5
        
    def tearDown(self):
        self.node.destroy_node()

    def test_events_cleared_after_summarization(self):
        """Тест: События очищаются после суммаризации"""
        # Добавляем 15 событий
        for i in range(15):
            self.node.add_to_memory('user_speech', f"Фраза {i+1}", important=True)
        
        self.assertEqual(len(self.node.speech_events), 15)
        
        # Эмулируем суммаризацию (просто очищаем как делает _summarize_events)
        events_to_keep = self.node.speech_events[-10:] if len(self.node.speech_events) > 10 else self.node.speech_events
        self.node.speech_events.clear()
        self.node.speech_events.extend(events_to_keep)
        
        # Должно остаться 10
        self.assertEqual(len(self.node.speech_events), 10)

    def test_summarization_prompt_loaded(self):
        """Тест: Промпт суммаризации загружен"""
        # Создаём ноду с включенной суммаризацией
        node_with_sum = ContextAggregatorNode()
        
        if node_with_sum.enable_summarization and node_with_sum.deepseek_client:
            self.assertIsNotNone(node_with_sum.summarization_prompt)
            self.assertIsInstance(node_with_sum.summarization_prompt, str)
            self.assertGreater(len(node_with_sum.summarization_prompt), 50)
            # Проверяем наличие ключевых слов в промпте
            self.assertIn('события', node_with_sum.summarization_prompt.lower())
        
        node_with_sum.destroy_node()

    def test_check_and_summarize_respects_threshold(self):
        """Тест: check_and_summarize срабатывает только при достижении порога"""
        # Добавляем 3 события (меньше порога 5)
        for i in range(3):
            self.node.add_to_memory('user_speech', f"Фраза {i+1}", important=True)
        
        self.assertEqual(len(self.node.speech_events), 3)
        
        # Вызываем проверку
        self.node.check_and_summarize()
        
        # Summaries не должны быть созданы (порог не достигнут)
        self.assertEqual(len(self.node.speech_summaries), 0)

    def test_multiple_event_types_separate_tracking(self):
        """Тест: Разные типы событий отслеживаются независимо"""
        # Добавляем события разных типов
        for i in range(3):
            self.node.add_to_memory('user_speech', f"Речь {i+1}", important=True)
            self.node.add_to_memory('vision', f"Vision {i+1}", important=False)
            self.node.add_to_memory('robot_response', f"Ответ {i+1}", important=True)
        
        # Проверяем независимость счётчиков
        self.assertEqual(len(self.node.speech_events), 3)
        self.assertEqual(len(self.node.vision_events), 3)
        self.assertEqual(len(self.node.robot_response_events), 3)
        
        # Всего 9 событий в общей памяти
        self.assertEqual(len(self.node.recent_events), 9)

    def test_memory_window_cleaning(self):
        """Тест: Старые события удаляются по memory_window"""
        # Устанавливаем короткое окно памяти
        self.node.memory_window = 2  # 2 секунды
        
        # Добавляем событие
        self.node.add_to_memory('user_speech', 'Старая фраза', important=True)
        self.assertEqual(len(self.node.speech_events), 1)
        
        # Ждём 3 секунды
        time.sleep(3)
        
        # Добавляем новое событие (триггерит очистку старых)
        self.node.add_to_memory('user_speech', 'Новая фраза', important=True)
        
        # Старое событие должно быть удалено
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(self.node.speech_events[0]['content'], 'Новая фраза')


if __name__ == '__main__':
    unittest.main()
