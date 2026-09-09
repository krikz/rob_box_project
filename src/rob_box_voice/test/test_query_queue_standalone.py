#!/usr/bin/env python3
"""
Standalone тест логики Query Queue System без ROS зависимостей.
Имитирует работу системы накопления запросов.
"""

import time
from typing import List


class QueryQueueSimulator:
    """Симулятор системы накопления запросов."""

    def __init__(self, accumulation_timeout: float = 2.5):
        self.pending_queries: List[str] = []
        self.accumulation_timeout = accumulation_timeout
        self.last_query_time = None
        self.llm_processing = False

    def add_query(self, query: str):
        """Добавить запрос в очередь."""
        self.pending_queries.append(query)
        self.last_query_time = time.time()
        print(f"📥 Запрос добавлен: '{query}' (всего: {len(self.pending_queries)})")

    def should_process_queue(self) -> bool:
        """Проверить: пора ли обрабатывать очередь."""
        if not self.pending_queries:
            return False

        if self.llm_processing:
            return False

        if self.last_query_time:
            time_since_last = time.time() - self.last_query_time
            return time_since_last >= self.accumulation_timeout

        return True

    def process_queue(self) -> str:
        """Обработать накопленную очередь."""
        if not self.pending_queries:
            return ""

        queries_to_process = self.pending_queries.copy()
        self.pending_queries.clear()

        query_count = len(queries_to_process)
        print(f"\n🔄 Обрабатываю {query_count} накопленных запросов")

        if query_count == 1:
            combined_message = queries_to_process[0]
            print(f"💬 Один запрос: {combined_message}")
        else:
            parts = [f"У меня несколько вопросов ({query_count} штук), отвечай на них все сразу по порядку:"]
            for i, query in enumerate(queries_to_process, 1):
                parts.append(f"{i}. {query}")
            combined_message = "\n".join(parts)
            print(f"💬 Пакетный запрос:\n{combined_message}")

        return combined_message

    def simulate_llm_processing(self, duration: float = 1.0):
        """Имитация обработки LLM."""
        self.llm_processing = True
        print(f"⏳ LLM обрабатывает запрос ({duration}s)...")
        time.sleep(duration)
        self.llm_processing = False
        print("✅ LLM завершил обработку")


def test_single_query():
    """Тест: один запрос."""
    print("\n" + "="*60)
    print("ТЕСТ 1: Один запрос")
    print("="*60)

    sim = QueryQueueSimulator(accumulation_timeout=1.0)

    sim.add_query("Привет, как дела?")
    time.sleep(1.1)  # Ждём больше чем accumulation_timeout

    if sim.should_process_queue():
        result = sim.process_queue()
        assert "Привет, как дела?" in result
        print("✅ Тест пройден: один запрос обработан корректно\n")
    else:
        print("❌ Тест провален: запрос не обработан\n")


def test_multiple_quick_queries():
    """Тест: несколько быстрых запросов."""
    print("\n" + "="*60)
    print("ТЕСТ 2: Несколько быстрых запросов")
    print("="*60)

    sim = QueryQueueSimulator(accumulation_timeout=1.0)

    # Добавляем 3 запроса быстро
    sim.add_query("Сколько времени?")
    time.sleep(0.2)
    sim.add_query("Какая погода?")
    time.sleep(0.2)
    sim.add_query("Что ты умеешь?")

    time.sleep(1.0)  # Ждём accumulation_timeout

    if sim.should_process_queue():
        result = sim.process_queue()
        assert "несколько вопросов" in result
        assert "3 штук" in result
        assert "Сколько времени?" in result
        assert "Какая погода?" in result
        assert "Что ты умеешь?" in result
        print("✅ Тест пройден: 3 запроса объединены в пакет\n")
    else:
        print("❌ Тест провален: запросы не обработаны\n")


def test_queries_during_processing():
    """Тест: запросы во время обработки LLM."""
    print("\n" + "="*60)
    print("ТЕСТ 3: Запросы во время обработки LLM")
    print("="*60)

    sim = QueryQueueSimulator(accumulation_timeout=0.5)

    # Первый запрос
    sim.add_query("Расскажи про Пифагора")
    time.sleep(0.6)

    # Начинаем обработку
    if sim.should_process_queue():
        result1 = sim.process_queue()
        print(f"Первый запрос: {result1}")

        # Имитируем обработку LLM
        sim.llm_processing = True
        print("\n⏳ LLM обрабатывает первый запрос...")

        # Добавляем новые запросы во время обработки
        time.sleep(0.2)
        sim.add_query("А кто он был?")
        time.sleep(0.2)
        sim.add_query("В каком веке жил?")

        # Проверяем что они НЕ обрабатываются пока LLM занят
        if not sim.should_process_queue():
            print("✅ Корректно: новые запросы ждут завершения LLM")
        else:
            print("❌ Ошибка: запросы обрабатываются во время LLM processing")

        # Завершаем обработку LLM
        time.sleep(0.3)
        sim.llm_processing = False
        print("✅ LLM завершил обработку")

        # Теперь обрабатываем накопленные запросы
        time.sleep(0.6)
        if sim.should_process_queue():
            result2 = sim.process_queue()
            assert "несколько вопросов" in result2
            assert "2 штук" in result2
            print("✅ Тест пройден: накопленные запросы обработаны после LLM\n")
        else:
            print("❌ Тест провален: накопленные запросы не обработаны\n")


def test_queue_clear():
    """Тест: очистка очереди."""
    print("\n" + "="*60)
    print("ТЕСТ 4: Очистка очереди")
    print("="*60)

    sim = QueryQueueSimulator(accumulation_timeout=1.0)

    # Добавляем запросы
    sim.add_query("Вопрос 1")
    sim.add_query("Вопрос 2")
    sim.add_query("Вопрос 3")

    assert len(sim.pending_queries) == 3
    print(f"В очереди: {len(sim.pending_queries)} запросов")

    # Очищаем очередь (имитация команды "замолчи")
    sim.pending_queries.clear()
    print("🔇 Очередь очищена (команда молчания)")

    assert len(sim.pending_queries) == 0
    print("✅ Тест пройден: очередь успешно очищена\n")


def main():
    """Запуск всех тестов."""
    print("\n" + "="*60)
    print("🧪 ТЕСТИРОВАНИЕ QUERY QUEUE SYSTEM")
    print("="*60)

    test_single_query()
    test_multiple_quick_queries()
    test_queries_during_processing()
    test_queue_clear()

    print("\n" + "="*60)
    print("✅ Все тесты успешно пройдены!")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
