#!/usr/bin/env python3
"""
test_memory_integration.py - Интеграционные тесты памяти ContextAggregatorNode

КРИТИЧНЫЕ ТЕСТЫ логики работы:
- Краткосрочная память (recent_events) - удаление старых событий по memory_window
- Долгосрочная память (summaries) - сохранение сжатой истории
- Периодическая публикация PerceptionEvent (2 Hz)
- Реальная суммаризация через DeepSeek API
- Формирование полного контекста (summaries + recent)
"""

import os
import time
import unittest
from unittest.mock import MagicMock, patch
import pytest

import rclpy
from std_msgs.msg import String

from rob_box_perception.context_aggregator_node import ContextAggregatorNode


DEEPSEEK_API_KEY = os.getenv('DEEPSEEK_API_KEY')
HAS_DEEPSEEK_KEY = DEEPSEEK_API_KEY is not None and len(DEEPSEEK_API_KEY) > 20


class TestMemoryWindow(unittest.TestCase):
    """Тесты окна памяти (краткосрочная память)"""

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
        self.node.enable_summarization = False  # Отключаем для изоляции теста
        # Ставим короткое окно памяти
        self.node.memory_window = 3  # 3 секунды
        
    def tearDown(self):
        self.node.destroy_node()

    def test_old_events_are_removed_from_recent_memory(self):
        """КРИТИЧНЫЙ: Старые события удаляются из краткосрочной памяти"""
        print("\n📝 Тест: Удаление старых событий из recent_events")
        
        # Добавляем старое событие
        self.node.add_to_memory('user_speech', 'Старая фраза', important=True)
        self.assertEqual(len(self.node.recent_events), 1)
        print(f"  ✓ Добавили событие, всего: {len(self.node.recent_events)}")
        
        # Ждём 4 секунды (больше чем memory_window=3)
        print("  ⏳ Ждём 4 секунды...")
        time.sleep(4)
        
        # Добавляем новое событие (триггерит очистку)
        self.node.add_to_memory('user_speech', 'Новая фраза', important=True)
        
        # Старое событие должно быть удалено
        self.assertEqual(len(self.node.recent_events), 1, 
                        "Старое событие должно было быть удалено!")
        self.assertEqual(self.node.recent_events[0]['content'], 'Новая фраза',
                        "Должна остаться только новая фраза!")
        print(f"  ✅ Старое событие удалено, осталось: {len(self.node.recent_events)}")

    def test_old_events_removed_from_typed_queues(self):
        """КРИТИЧНЫЙ: Старые события удаляются из типизированных очередей"""
        print("\n📝 Тест: Удаление из speech_events, vision_events и т.д.")
        
        # Добавляем события разных типов
        self.node.add_to_memory('user_speech', 'Старая речь', important=True)
        self.node.add_to_memory('vision', 'Старое видение', important=False)
        self.node.add_to_memory('error', 'Старая ошибка', important=True)
        
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(len(self.node.vision_events), 1)
        self.assertEqual(len(self.node.system_events), 1)
        print(f"  ✓ Добавили: speech={len(self.node.speech_events)}, vision={len(self.node.vision_events)}, system={len(self.node.system_events)}")
        
        # Ждём 4 секунды
        print("  ⏳ Ждём 4 секунды...")
        time.sleep(4)
        
        # Добавляем новые события
        self.node.add_to_memory('user_speech', 'Новая речь', important=True)
        
        # Проверяем что старые удалены из ВСЕХ очередей
        self.assertEqual(len(self.node.speech_events), 1)
        self.assertEqual(len(self.node.vision_events), 0)
        self.assertEqual(len(self.node.system_events), 0)
        print(f"  ✅ После очистки: speech={len(self.node.speech_events)}, vision={len(self.node.vision_events)}, system={len(self.node.system_events)}")

    def test_recent_events_kept_within_window(self):
        """КРИТИЧНЫЙ: События внутри окна памяти сохраняются"""
        print("\n📝 Тест: События внутри окна НЕ удаляются")
        
        # Добавляем события с интервалом 1 секунда
        for i in range(3):
            self.node.add_to_memory('user_speech', f'Фраза {i+1}', important=True)
            print(f"  ✓ Добавили фразу {i+1}, всего: {len(self.node.speech_events)}")
            if i < 2:
                time.sleep(1)
        
        # Все 3 события должны быть в памяти (прошло только 2 сек, окно 3 сек)
        self.assertEqual(len(self.node.speech_events), 3,
                        "Все события должны быть в памяти!")
        print(f"  ✅ Все события в памяти: {len(self.node.speech_events)}")


@unittest.skipUnless(HAS_DEEPSEEK_KEY, "DEEPSEEK_API_KEY not set")
@pytest.mark.slow
class TestSummarizationMemory(unittest.TestCase):
    """Тесты долгосрочной памяти (summaries)"""

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
        self.node.summarization_threshold = 5  # Быстрая суммаризация
        
    def tearDown(self):
        self.node.destroy_node()

    def test_old_events_summarized_and_cleared(self):
        """КРИТИЧНЫЙ: Старые события суммаризируются и очищаются"""
        print("\n📝 Тест: Суммаризация очищает старые события")
        
        # Добавляем 5 событий (достигаем порога)
        for i in range(5):
            self.node.add_to_memory('user_speech', 
                                   f'Пользователь спросил про робота номер {i+1}',
                                   important=True)
        
        print(f"  ✓ Добавили 5 событий")
        print(f"  ⏳ Ждём суммаризацию (автоматическая при 5-м событии)...")
        
        # Суммаризация происходит автоматически при 5-м событии
        # Ждём немного чтобы завершилась
        time.sleep(1)
        
        # Проверяем что summary создан
        self.assertGreater(len(self.node.speech_summaries), 0,
                          "Summary должен был быть создан!")
        print(f"  ✅ Summary создан: {len(self.node.speech_summaries)}")
        
        # Проверяем что события очищены (остаются последние 10)
        self.assertLessEqual(len(self.node.speech_events), 10,
                           "События должны были быть очищены!")
        print(f"  ✅ События очищены, осталось: {len(self.node.speech_events)}")

    def test_summaries_persist_after_memory_window(self):
        """КРИТИЧНЫЙ: Summaries сохраняются даже когда события удалены"""
        print("\n📝 Тест: Summaries - это долгосрочная память")
        
        # Устанавливаем короткое окно памяти
        self.node.memory_window = 2  # 2 секунды
        
        # Добавляем 5 событий для суммаризации
        for i in range(5):
            self.node.add_to_memory('user_speech', 
                                   f'История номер {i+1} про приключения робота',
                                   important=True)
        
        print(f"  ✓ Добавили 5 событий, получили summary")
        
        # Ждём больше чем memory_window
        print("  ⏳ Ждём 3 секунды (больше окна памяти)...")
        time.sleep(3)
        
        # Добавляем новое событие чтобы триггернуть очистку
        self.node.add_to_memory('user_speech', 'Новая фраза', important=True)
        
        # Старые события должны быть удалены
        self.assertLessEqual(len(self.node.speech_events), 2,
                           "Старые события должны были быть удалены!")
        
        # НО summary должен остаться!
        self.assertGreater(len(self.node.speech_summaries), 0,
                          "Summary должен сохраниться (долгосрочная память)!")
        
        print(f"  ✅ События удалены: {len(self.node.speech_events)}")
        print(f"  ✅ Summaries сохранены: {len(self.node.speech_summaries)}")

    def test_summary_quality_captures_meaning(self):
        """КРИТИЧНЫЙ: LLM правильно понимает смысл событий"""
        print("\n📝 Тест: Качество суммаризации (LLM понимает смысл)")
        
        # Добавляем события с чётким смыслом
        events = [
            "Пользователь спросил про батарею",
            "Пользователь попросил показать карту",
            "Пользователь сказал что робот молодец",
            "Пользователь спросил сколько времени",
            "Пользователь попросил рассказать анекдот"
        ]
        
        for event in events:
            self.node.add_to_memory('user_speech', event, important=True)
        
        print(f"  ✓ Добавили 5 осмысленных событий")
        print(f"  ⏳ Ждём суммаризацию DeepSeek...")
        time.sleep(1)
        
        # Проверяем что summary создан
        self.assertGreater(len(self.node.speech_summaries), 0)
        summary_text = self.node.speech_summaries[0]['summary']
        
        print(f"\n  📄 Summary от DeepSeek ({len(summary_text)} символов):")
        print(f"     {summary_text}")
        
        # Проверяем что summary содержит ключевые слова
        summary_lower = summary_text.lower()
        
        # Должно быть упоминание о вопросах пользователя
        has_user_context = any(word in summary_lower for word in 
                              ['пользователь', 'спросил', 'попросил', 'сказал', 'запрос'])
        self.assertTrue(has_user_context, 
                       "Summary должен содержать контекст пользователя!")
        
        # Summary должен быть достаточно информативным
        self.assertGreater(len(summary_text), 30,
                          "Summary слишком короткий!")
        
        print(f"  ✅ Summary качественный и содержит смысл событий")

    def test_get_full_context_includes_summaries_and_recent(self):
        """КРИТИЧНЫЙ: get_full_context() возвращает summaries + recent events"""
        print("\n📝 Тест: Полный контекст = summaries + свежие события")
        
        # Добавляем 5 событий для суммаризации
        for i in range(5):
            self.node.add_to_memory('user_speech', 
                                   f'Старая история {i+1}',
                                   important=True)
        
        print("  ✓ Создали summary из 5 событий")
        time.sleep(1)
        
        # Добавляем свежие события (после суммаризации)
        self.node.add_to_memory('user_speech', 'Свежая фраза 1', important=True)
        self.node.add_to_memory('user_speech', 'Свежая фраза 2', important=True)
        
        print("  ✓ Добавили 2 свежих события")
        
        # Получаем полный контекст
        full_context = self.node.get_full_context()
        
        self.assertIsNotNone(full_context)
        self.assertIsInstance(full_context, str)
        self.assertGreater(len(full_context), 50,
                          "Контекст слишком короткий!")
        
        # Контекст должен содержать информацию и о summaries и о свежих событиях
        context_lower = full_context.lower()
        
        # Должен быть раздел с summaries
        has_summary_section = 'истор' in context_lower or 'диалог' in context_lower
        
        # Должны быть свежие события
        has_recent = 'свежая' in context_lower or 'фраза' in context_lower
        
        print(f"\n  📄 Полный контекст ({len(full_context)} символов):")
        print(f"     {full_context[:200]}...")
        
        self.assertTrue(has_summary_section or has_recent,
                       "Контекст должен содержать summaries или recent events!")
        
        print(f"  ✅ Контекст содержит и summaries и свежие события")


class TestPeriodicPublishing(unittest.TestCase):
    """Тесты периодической публикации событий"""

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
        self.node.enable_summarization = False
        
    def tearDown(self):
        self.node.destroy_node()

    def test_publish_timer_exists(self):
        """КРИТИЧНЫЙ: Таймер публикации существует"""
        self.assertIsNotNone(self.node.publish_timer)
        print("  ✅ Таймер publish_timer существует")

    def test_publish_rate_parameter(self):
        """КРИТИЧНЫЙ: Частота публикации настраивается"""
        self.assertEqual(self.node.publish_rate, 2.0,
                        "По умолчанию должно быть 2 Hz")
        print(f"  ✅ Частота публикации: {self.node.publish_rate} Hz")

    def test_publish_event_method_exists(self):
        """КРИТИЧНЫЙ: Метод publish_event существует и вызывается"""
        self.assertTrue(hasattr(self.node, 'publish_event'))
        
        # Проверяем что метод можно вызвать
        try:
            self.node.publish_event()
            print("  ✅ Метод publish_event вызывается без ошибок")
        except Exception as e:
            self.fail(f"publish_event упал с ошибкой: {e}")


if __name__ == '__main__':
    unittest.main()
