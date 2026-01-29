#!/usr/bin/env python3
"""
llm_adapter.py - Адаптер для интеграции MCP tools с LLM API

Этот модуль обрабатывает tool_calls из OpenAI-совместимых LLM API (DeepSeek, Qwen, и др.)
и вызывает соответствующие MCP инструменты через ROS 2 топики.

Поддерживает любые LLM с OpenAI-совместимым API, включая:
- DeepSeek
- Qwen
- OpenAI GPT
- Другие провайдеры с tool_calls support

Версия 2.0: Добавлена поддержка async execution с прерыванием
"""

import json
import uuid
import asyncio
from typing import Dict, Any, List, Optional, Callable
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from .async_executor import AsyncToolExecutor, ToolCallAccumulator
from .base import ToolExecutionType


class LLMToolCallAdapter:
    """
    Адаптер для обработки tool calls из OpenAI-совместимых LLM API

    Преобразует tool_calls в запросы к MCP серверу и ожидает результаты.
    Работает с любым LLM, поддерживающим OpenAI tool_calls формат.
    
    Версия 2.0: Поддержка async execution, параллельное выполнение, прерывания
    """

    def __init__(self, node: Node):
        """
        Инициализация адаптера

        Args:
            node: ROS 2 Node для доступа к publishers/subscribers
        """
        self.node = node

        # QoS для минимизации задержек в Zenoh
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher для запросов выполнения инструментов
        self.execute_pub = node.create_publisher(String, "/mcp/execute", qos_profile)

        # Subscriber для результатов
        self.result_sub = node.create_subscription(String, "/mcp/result", self.on_result, qos_profile)

        # Кэш ожидающих результатов: request_id -> callback
        self.pending_requests: Dict[str, Callable] = {}

        # Кэш результатов: request_id -> result
        self.results_cache: Dict[str, Dict[str, Any]] = {}
        
        # Event'ы для синхронного ожидания: request_id -> Event
        self.result_events: Dict[str, threading.Event] = {}

        # Timeout для ожидания результата (секунды)
        self.timeout = 5.0
        
        # ============ Async Execution Engine ============
        self.async_executor = AsyncToolExecutor(
            execute_pub=self.execute_pub,
            result_callback=self._on_async_result,
            logger=node.get_logger()
        )
        
        # Tool Call Accumulator для streaming
        self.tool_call_accumulator = ToolCallAccumulator()

        self.node.get_logger().info("✅ LLM Tool Call Adapter v2.0 инициализирован (async + interrupts)")

    def on_result(self, msg: String):
        """Обработка результата выполнения инструмента"""
        try:
            result = json.loads(msg.data)
            request_id = result.get("request_id", "")
            
            self.node.get_logger().info(f"📩 on_result вызван для request_id: {request_id[:8] if request_id else 'empty'}")

            if not request_id:
                self.node.get_logger().warn("⚠️ Получен результат без request_id")
                return

            # Сохраняем результат в кэш
            self.results_cache[request_id] = result
            self.node.get_logger().info(f"💾 Результат сохранён в кэш для {request_id[:8]}")
            
            # Уведомляем ждущий поток через Event
            if request_id in self.result_events:
                self.result_events[request_id].set()
                self.node.get_logger().info(f"✅ Event установлен для {request_id[:8]}")
            
            # Уведомляем async executor
            self.async_executor.on_result_received(request_id, result)

            # Вызываем callback если он есть
            if request_id in self.pending_requests:
                callback = self.pending_requests[request_id]
                callback(result)
                del self.pending_requests[request_id]

        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"❌ Ошибка парсинга результата: {e}")
        except Exception as e:
            self.node.get_logger().error(f"❌ Ошибка обработки результата: {e}")
    
    def _on_async_result(self, request_id: str, result: Dict[str, Any]) -> None:
        """
        Callback для async executor когда получен результат
        
        Args:
            request_id: ID запроса
            result: Результат выполнения
        """
        # Уже обработано в on_result, это просто дополнительный callback
        pass

    def execute_tool_call(
        self, tool_name: str, parameters: Dict[str, Any], callback: Optional[Callable] = None, timeout: Optional[float] = None
    ) -> str:
        """
        Выполнить tool call асинхронно

        Args:
            tool_name: Имя инструмента
            parameters: Параметры инструмента
            callback: Функция для вызова при получении результата
            timeout: Таймаут ожидания результата (по умолчанию self.timeout)

        Returns:
            request_id: Уникальный ID запроса для отслеживания результата
        """
        request_id = str(uuid.uuid4())

        # Формируем запрос
        request = {"tool_name": tool_name, "parameters": parameters, "request_id": request_id}

        # Регистрируем callback если передан
        if callback:
            self.pending_requests[request_id] = callback

        # Отправляем запрос
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.execute_pub.publish(msg)

        self.node.get_logger().info(f"📤 Отправлен запрос {request_id[:8]}: {tool_name}")

        return request_id

    def execute_tool_call_sync(self, tool_name: str, parameters: Dict[str, Any], timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Выполнить tool call синхронно (блокируется до получения результата)

        Args:
            tool_name: Имя инструмента
            parameters: Параметры инструмента
            timeout: Таймаут ожидания результата

        Returns:
            Dict с результатом выполнения
        """
        if timeout is None:
            timeout = self.timeout

        request_id = str(uuid.uuid4())

        # Создаём Event для ожидания результата ПЕРЕД публикацией запроса
        # Это предотвращает race condition, когда результат приходит до регистрации Event
        result_event = threading.Event()
        self.result_events[request_id] = result_event

        # Формируем запрос
        request = {"tool_name": tool_name, "parameters": parameters, "request_id": request_id}

        # Публикуем запрос
        request_msg = String()
        request_msg.data = json.dumps(request, ensure_ascii=False)
        self.execute_pub.publish(request_msg)

        self.node.get_logger().info(f"📤 Отправлен запрос {request_id[:8]}: {tool_name}")

        # Ожидаем результат с таймаутом
        # Используем простой wait() вместо spin_once, т.к. node уже управляется MultiThreadedExecutor
        # который автоматически вызовет on_result() callback в фоновом потоке
        result_received = result_event.wait(timeout=timeout)
        
        # Проверяем финальный статус
        if not result_received:
            # Timeout - очищаем Event
            self.result_events.pop(request_id, None)
            self.node.get_logger().error(f"⏱️ Timeout ожидания результата для {tool_name} (request_id: {request_id[:8]})")
            return {"success": False, "error": "Timeout ожидания результата инструмента"}

        # Очищаем Event
        self.result_events.pop(request_id, None)

        # Получаем результат из кэша
        if request_id not in self.results_cache:
            self.node.get_logger().error(f"❌ Результат для {request_id[:8]} не найден в кэше после Event.set()")
            return {"success": False, "error": "Результат не найден в кэше"}
            
        result_data = self.results_cache.pop(request_id)
        tool_result = result_data.get("result", {"success": False, "error": "Пустой результат"})
        
        # Проверка: если инструмент async (возвращает сразу), то не ждем завершения
        if tool_result.get("data", {}).get("async"):
            self.node.get_logger().info(f"⚡ Инструмент {tool_name} асинхронный - возвращаем результат сразу")
        
        return tool_result

    def process_tool_calls_from_message(self, message: Any) -> List[Dict[str, Any]]:
        """
        Обработать tool_calls из ответа LLM API (OpenAI-совместимый формат)

        Args:
            message: OpenAI message object с tool_calls

        Returns:
            Список результатов выполнения инструментов
        """
        if not hasattr(message, "tool_calls") or not message.tool_calls:
            return []

        results = []

        for tool_call in message.tool_calls:
            tool_name = tool_call.function.name
            # Парсим JSON аргументы
            try:
                parameters = json.loads(tool_call.function.arguments)
            except json.JSONDecodeError:
                self.node.get_logger().error(f"❌ Не удалось распарсить аргументы для {tool_name}")
                results.append(
                    {"tool_call_id": tool_call.id, "tool_name": tool_name, "success": False, "error": "Неверный формат аргументов"}
                )
                continue

            # Выполняем инструмент синхронно
            result = self.execute_tool_call_sync(tool_name, parameters)
            result["tool_call_id"] = tool_call.id
            result["tool_name"] = tool_name
            results.append(result)

        return results

    def format_tool_results_for_llm(self, results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Форматировать результаты инструментов для отправки обратно в LLM

        Args:
            results: Список результатов выполнения

        Returns:
            Список сообщений в формате OpenAI tool result
        """
        messages = []

        for result in results:
            tool_call_id = result.get("tool_call_id", "")
            tool_name = result.get("tool_name", "unknown")

            # Формируем content для LLM
            if result.get("success"):
                content = result.get("message", "Выполнено успешно")
                if result.get("data"):
                    content += f"\nДанные: {json.dumps(result['data'], ensure_ascii=False)}"
            else:
                content = f"Ошибка: {result.get('error', 'Неизвестная ошибка')}"

            messages.append({"role": "tool", "tool_call_id": tool_call_id, "name": tool_name, "content": content})

        return messages
    
    # ============================================================
    # Async Execution Methods (v2.0)
    # ============================================================
    
    async def new_user_request(self) -> int:
        """
        Отметить начало нового пользовательского запроса
        
        Увеличивает sequence_id, тем самым помечая все предыдущие
        tool_calls как устаревшие. Используется при прерывании.
        
        Returns:
            Новый sequence ID
        """
        sequence_id = await self.async_executor.new_sequence()
        
        # Также прерываем все активные LONG задачи
        cancelled = await self.async_executor.interrupt_all_long_tasks()
        if cancelled > 0:
            self.node.get_logger().info(
                f"🔄 Новый запрос (sequence #{sequence_id}): прервано {cancelled} задач"
            )
        
        return sequence_id
    
    def get_current_sequence_id(self) -> int:
        """Получить текущий sequence ID"""
        return self.async_executor.get_current_sequence_id()
    
    async def execute_tools_parallel_async(
        self,
        tool_calls: List[Dict[str, Any]],
        tool_registry: Optional[Dict[str, Any]] = None,
        sequence_id: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Асинхронное параллельное выполнение tool_calls
        
        Использует AsyncToolExecutor для оптимального выполнения:
        - INSTANT: Fire-and-forget
        - FAST/MEDIUM: Parallel await
        - LONG: Background tasks
        
        Args:
            tool_calls: Список tool_calls
            tool_registry: Опциональный реестр для определения execution_type
            sequence_id: Sequence ID для проверки актуальности
        
        Returns:
            Список результатов выполнения
        """
        return await self.async_executor.execute_tools_parallel(
            tool_calls, tool_registry, sequence_id
        )
    
    async def interrupt_all_long_tasks(self) -> int:
        """
        Прервать все активные LONG задачи
        
        Returns:
            Количество прерванных задач
        """
        return await self.async_executor.interrupt_all_long_tasks()
    
    async def interrupt_task_by_name(self, tool_name: str) -> int:
        """
        Прервать все LONG задачи с указанным именем
        
        Args:
            tool_name: Имя инструмента
        
        Returns:
            Количество прерванных задач
        """
        return await self.async_executor.interrupt_task_by_name(tool_name)
    
    def accumulate_tool_call_chunk(self, delta_tool_calls: List[Any]) -> None:
        """
        Добавить chunk tool_calls из streaming response
        
        Args:
            delta_tool_calls: Список tool_call delta objects из OpenAI API
        """
        self.tool_call_accumulator.add_chunk(delta_tool_calls)
    
    def get_accumulated_tool_calls(self) -> List[Dict[str, Any]]:
        """
        Получить полный список накопленных tool_calls
        
        Returns:
            Список tool_calls готовых к выполнению
        """
        return self.tool_call_accumulator.get_complete_tool_calls()
    
    def clear_tool_call_accumulator(self) -> None:
        """Очистить accumulator для нового streaming запроса"""
        self.tool_call_accumulator.clear()
    
    async def process_tool_calls_from_message_async(
        self,
        message: Any,
        tool_registry: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Обработать tool_calls из ответа LLM API асинхронно с параллельным выполнением
        
        Args:
            message: OpenAI message object с tool_calls
            tool_registry: Опциональный реестр инструментов
        
        Returns:
            Список результатов выполнения инструментов
        """
        if not hasattr(message, "tool_calls") or not message.tool_calls:
            return []
        
        # Конвертируем tool_calls в нужный формат
        tool_calls = []
        for tool_call in message.tool_calls:
            try:
                parameters = json.loads(tool_call.function.arguments)
            except json.JSONDecodeError:
                self.node.get_logger().error(f"❌ Не удалось распарсить аргументы для {tool_call.function.name}")
                continue
            
            tool_calls.append({
                "id": tool_call.id,
                "type": "function",
                "function": {
                    "name": tool_call.function.name,
                    "arguments": parameters
                }
            })
        
        # Параллельное выполнение
        results = await self.execute_tools_parallel_async(tool_calls, tool_registry)
        
        # Добавляем tool_call_id и tool_name к результатам
        for i, result in enumerate(results):
            if i < len(tool_calls):
                result["tool_call_id"] = tool_calls[i]["id"]
                result["tool_name"] = tool_calls[i]["function"]["name"]
        
        return results
