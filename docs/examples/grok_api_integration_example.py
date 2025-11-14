#!/usr/bin/env python3
"""
Пример интеграции Grok API в Rob Box Voice Assistant

Демонстрирует:
1. Базовое подключение к Grok API
2. Streaming ответов
3. Function calling для управления роботом
4. Сравнение с DeepSeek API

ВАЖНО: Это только пример кода для тестирования!
Реальная интеграция требует модификации dialogue_node.py
"""

import os
import time
from typing import List, Dict, Any
from openai import OpenAI


class GrokAPIClient:
    """Клиент для работы с xAI Grok API"""
    
    def __init__(self, api_key: str = None):
        """
        Инициализация клиента Grok API
        
        Args:
            api_key: API ключ xAI. Если не указан, берётся из XAI_API_KEY
        """
        self.api_key = api_key or os.getenv("XAI_API_KEY")
        if not self.api_key:
            raise ValueError("XAI_API_KEY не найден! Установите через переменную окружения или параметр.")
        
        # Grok API совместим с OpenAI SDK
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://api.x.ai/v1"
        )
        
        self.model = "grok-4"  # или "grok-4-fast-reasoning"
        self.conversation_history: List[Dict[str, str]] = []
    
    def set_system_prompt(self, prompt: str):
        """
        Установить системный промпт
        
        Args:
            prompt: Системный промпт для Grok
        """
        self.conversation_history = [
            {"role": "system", "content": prompt}
        ]
    
    def chat(self, user_message: str, stream: bool = False) -> str:
        """
        Отправить сообщение и получить ответ
        
        Args:
            user_message: Сообщение пользователя
            stream: Использовать ли streaming (рекомендуется)
        
        Returns:
            Ответ от Grok
        """
        # Добавить сообщение в историю
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        if stream:
            return self._chat_streaming()
        else:
            return self._chat_blocking()
    
    def _chat_blocking(self) -> str:
        """Обычный запрос (блокирующий)"""
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            temperature=0.7,
            max_tokens=500
        )
        
        assistant_message = response.choices[0].message.content
        
        # Добавить ответ в историю
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })
        
        return assistant_message
    
    def _chat_streaming(self) -> str:
        """Streaming запрос (рекомендуется для real-time)"""
        print("🤖 Grok: ", end="", flush=True)
        
        full_response = ""
        
        stream = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            temperature=0.7,
            max_tokens=500,
            stream=True  # Включить streaming
        )
        
        for chunk in stream:
            if chunk.choices[0].delta.content is not None:
                content = chunk.choices[0].delta.content
                print(content, end="", flush=True)
                full_response += content
        
        print()  # Новая строка после ответа
        
        # Добавить ответ в историю
        self.conversation_history.append({
            "role": "assistant",
            "content": full_response
        })
        
        return full_response
    
    def chat_with_functions(self, user_message: str, functions: List[Dict]) -> Dict[str, Any]:
        """
        Чат с возможностью вызова функций (для управления роботом)
        
        Args:
            user_message: Сообщение пользователя
            functions: Список доступных функций
        
        Returns:
            Dict с ответом и вызванными функциями
        """
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            functions=functions,
            function_call="auto",  # Автоматический выбор функций
            temperature=0.7
        )
        
        message = response.choices[0].message
        
        result = {
            "content": message.content,
            "function_call": None
        }
        
        # Проверить вызов функции
        if message.function_call:
            result["function_call"] = {
                "name": message.function_call.name,
                "arguments": message.function_call.arguments
            }
        
        # Добавить в историю
        self.conversation_history.append({
            "role": "assistant",
            "content": message.content or "",
            "function_call": message.function_call
        })
        
        return result


class DeepSeekAPIClient:
    """Клиент для работы с DeepSeek API (для сравнения)"""
    
    def __init__(self, api_key: str = None):
        """Инициализация клиента DeepSeek API"""
        self.api_key = api_key or os.getenv("DEEPSEEK_API_KEY")
        if not self.api_key:
            raise ValueError("DEEPSEEK_API_KEY не найден!")
        
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://api.deepseek.com"
        )
        
        self.model = "deepseek-chat"
        self.conversation_history: List[Dict[str, str]] = []
    
    def set_system_prompt(self, prompt: str):
        """Установить системный промпт"""
        self.conversation_history = [
            {"role": "system", "content": prompt}
        ]
    
    def chat(self, user_message: str, stream: bool = False) -> str:
        """Отправить сообщение и получить ответ"""
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        if stream:
            return self._chat_streaming()
        else:
            return self._chat_blocking()
    
    def _chat_blocking(self) -> str:
        """Обычный запрос"""
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            temperature=0.7,
            max_tokens=500
        )
        
        assistant_message = response.choices[0].message.content
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })
        
        return assistant_message
    
    def _chat_streaming(self) -> str:
        """Streaming запрос"""
        print("🤖 DeepSeek: ", end="", flush=True)
        
        full_response = ""
        
        stream = self.client.chat.completions.create(
            model=self.model,
            messages=self.conversation_history,
            temperature=0.7,
            max_tokens=500,
            stream=True
        )
        
        for chunk in stream:
            if chunk.choices[0].delta.content is not None:
                content = chunk.choices[0].delta.content
                print(content, end="", flush=True)
                full_response += content
        
        print()
        
        self.conversation_history.append({
            "role": "assistant",
            "content": full_response
        })
        
        return full_response


# ============================================================================
# ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
# ============================================================================

def example_basic_chat():
    """Пример 1: Базовый чат с Grok"""
    print("\n" + "="*60)
    print("ПРИМЕР 1: Базовый чат с Grok API")
    print("="*60 + "\n")
    
    # Инициализация
    grok = GrokAPIClient()
    
    # Установить системный промпт
    grok.set_system_prompt(
        "Ты — РОББОКС, автономный робот-помощник. "
        "Отвечай коротко и по делу. Используй дружелюбный тон."
    )
    
    # Диалог
    questions = [
        "Привет! Как дела?",
        "Что ты умеешь делать?",
        "Сколько будет 2 плюс 2?"
    ]
    
    for question in questions:
        print(f"👤 Пользователь: {question}")
        response = grok.chat(question, stream=True)
        print()


def example_streaming_comparison():
    """Пример 2: Сравнение Grok vs DeepSeek (streaming)"""
    print("\n" + "="*60)
    print("ПРИМЕР 2: Сравнение Grok vs DeepSeek (streaming)")
    print("="*60 + "\n")
    
    system_prompt = (
        "Ты — РОББОКС, автономный робот для доставки в помещениях. "
        "Используешь ROS 2, SLAM, голосовое управление."
    )
    
    question = "Объясни мне что такое SLAM простыми словами"
    
    # DeepSeek
    print("\n--- DeepSeek ---")
    deepseek = DeepSeekAPIClient()
    deepseek.set_system_prompt(system_prompt)
    print(f"👤 Пользователь: {question}")
    deepseek_start = time.time()
    deepseek.chat(question, stream=True)
    deepseek_time = time.time() - deepseek_start
    
    # Grok
    print("\n--- Grok ---")
    grok = GrokAPIClient()
    grok.set_system_prompt(system_prompt)
    print(f"👤 Пользователь: {question}")
    grok_start = time.time()
    grok.chat(question, stream=True)
    grok_time = time.time() - grok_start
    
    # Сравнение
    print("\n" + "-"*60)
    print(f"⏱️  Время ответа DeepSeek: {deepseek_time:.2f} сек")
    print(f"⏱️  Время ответа Grok: {grok_time:.2f} сек")
    print(f"📊 Разница: {abs(grok_time - deepseek_time):.2f} сек")


def example_function_calling():
    """Пример 3: Function calling для управления роботом"""
    print("\n" + "="*60)
    print("ПРИМЕР 3: Function calling (управление роботом)")
    print("="*60 + "\n")
    
    # Определить доступные функции робота
    robot_functions = [
        {
            "name": "move_forward",
            "description": "Двигаться вперёд на указанное расстояние",
            "parameters": {
                "type": "object",
                "properties": {
                    "distance": {
                        "type": "number",
                        "description": "Расстояние в метрах"
                    },
                    "speed": {
                        "type": "number",
                        "description": "Скорость в м/с (опционально)",
                        "default": 0.5
                    }
                },
                "required": ["distance"]
            }
        },
        {
            "name": "turn",
            "description": "Повернуться на указанный угол",
            "parameters": {
                "type": "object",
                "properties": {
                    "angle": {
                        "type": "number",
                        "description": "Угол поворота в градусах (+ по часовой, - против)"
                    }
                },
                "required": ["angle"]
            }
        },
        {
            "name": "stop",
            "description": "Немедленная остановка робота",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        }
    ]
    
    grok = GrokAPIClient()
    grok.set_system_prompt(
        "Ты — РОББОКС, робот с навигацией. "
        "Когда пользователь просит движение — вызывай соответствующие функции. "
        "Подтверждай действия пользователю."
    )
    
    # Тестовые команды
    commands = [
        "Поезжай вперёд на 2 метра",
        "Поверни налево на 90 градусов",
        "Остановись!"
    ]
    
    for command in commands:
        print(f"👤 Пользователь: {command}")
        result = grok.chat_with_functions(command, robot_functions)
        
        if result["function_call"]:
            print(f"🔧 Функция: {result['function_call']['name']}")
            print(f"📋 Параметры: {result['function_call']['arguments']}")
        
        if result["content"]:
            print(f"🤖 Grok: {result['content']}")
        
        print()


def example_long_context():
    """Пример 4: Использование длинного контекста (1M токенов)"""
    print("\n" + "="*60)
    print("ПРИМЕР 4: Длинный контекст (преимущество Grok)")
    print("="*60 + "\n")
    
    grok = GrokAPIClient()
    
    # Системный промпт с большим контекстом
    long_context = """
    Ты — РОББОКС, автономный робот для доставки в помещениях.
    
    ТЕХНИЧЕСКИЕ ХАРАКТЕРИСТИКИ:
    - Платформа: 4-колёсный дифференциальный привод
    - Размеры: 50x40x60 см
    - Вес: 15 кг
    - Грузоподъёмность: 5 кг
    - Аккумулятор: LiPo 4S 5000mAh (16.8V)
    - Время работы: 4-6 часов
    
    СЕНСОРЫ:
    - OAK-D Stereo Camera (RGB + Depth)
    - LSLIDAR N10 (2D LiDAR, 10m range)
    - ReSpeaker Mic Array v2.0 (4 микрофона, VAD, DoA)
    - IMU (MPU6050)
    - Одометрия (энкодеры моторов)
    
    ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ:
    - OS: Ubuntu 22.04 + ROS 2 Humble
    - SLAM: RTAB-Map (RGB-D + 2D LiDAR fusion)
    - Навигация: Nav2 (локальное планирование пути)
    - Голосовой ассистент: Vosk STT + DeepSeek LLM + Yandex TTS
    - DDS: Zenoh (оптимизированное промежуточное ПО)
    
    ВОЗМОЖНОСТИ:
    1. Автономная навигация по помещениям
    2. Построение карт (SLAM)
    3. Голосовое управление и диалог
    4. Обнаружение и обход препятствий
    5. AprilTag локализация
    
    ... (ещё много текста — Grok может обработать до 1M токенов!)
    """
    
    grok.set_system_prompt(long_context)
    
    # Вопросы, требующие знания контекста
    questions = [
        "Какой у меня LiDAR?",
        "Сколько весит робот без груза?",
        "Какую навигационную систему я использую?"
    ]
    
    for question in questions:
        print(f"👤 Пользователь: {question}")
        response = grok.chat(question, stream=False)
        print(f"🤖 Grok: {response}\n")


# ============================================================================
# ГЛАВНАЯ ФУНКЦИЯ
# ============================================================================

def main():
    """Запуск всех примеров"""
    print("\n" + "="*60)
    print(" Примеры интеграции Grok API в Rob Box Voice Assistant")
    print("="*60)
    
    # Проверить наличие API ключей
    if not os.getenv("XAI_API_KEY"):
        print("\n❌ ОШИБКА: XAI_API_KEY не найден!")
        print("\nУстановите API ключ:")
        print("  export XAI_API_KEY='xai-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'")
        print("\nПолучить ключ: https://x.ai/api")
        return
    
    if not os.getenv("DEEPSEEK_API_KEY"):
        print("\n⚠️  ПРЕДУПРЕЖДЕНИЕ: DEEPSEEK_API_KEY не найден!")
        print("   Пример 2 (сравнение) не будет работать.")
        print("\nУстановите API ключ:")
        print("  export DEEPSEEK_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'")
        print()
    
    try:
        # Запустить примеры
        example_basic_chat()
        
        if os.getenv("DEEPSEEK_API_KEY"):
            example_streaming_comparison()
        
        example_function_calling()
        example_long_context()
        
        print("\n" + "="*60)
        print("✅ Все примеры выполнены успешно!")
        print("="*60 + "\n")
        
        print("📝 СЛЕДУЮЩИЕ ШАГИ:")
        print("1. Оценить качество ответов Grok vs DeepSeek")
        print("2. Измерить стоимость API запросов")
        print("3. Интегрировать в dialogue_node.py (если целесообразно)")
        print("4. A/B тестирование на реальных диалогах")
        
    except Exception as e:
        print(f"\n❌ ОШИБКА: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
