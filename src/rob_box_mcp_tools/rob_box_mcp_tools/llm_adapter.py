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
"""

import json
import uuid
from typing import Dict, Any, List, Optional, Callable
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LLMToolCallAdapter:
    """
    Адаптер для обработки tool calls из OpenAI-совместимых LLM API

    Преобразует tool_calls в запросы к MCP серверу и ожидает результаты.
    Работает с любым LLM, поддерживающим OpenAI tool_calls формат.
    """

    def __init__(self, node: Node):
        """
        Инициализация адаптера

        Args:
            node: ROS 2 Node для доступа к publishers/subscribers
        """
        self.node = node

        # Publisher для запросов выполнения инструментов
        self.execute_pub = node.create_publisher(String, "/mcp/execute", 10)

        # Subscriber для результатов
        self.result_sub = node.create_subscription(String, "/mcp/result", self.on_result, 10)

        # Кэш ожидающих результатов: request_id -> callback
        self.pending_requests: Dict[str, Callable] = {}

        # Кэш результатов: request_id -> result
        self.results_cache: Dict[str, Dict[str, Any]] = {}

        # Timeout для ожидания результата (секунды)
        self.timeout = 5.0

        self.node.get_logger().info("✅ LLM Tool Call Adapter инициализирован")

    def on_result(self, msg: String):
        """Обработка результата выполнения инструмента"""
        try:
            result = json.loads(msg.data)
            request_id = result.get("request_id", "")

            if not request_id:
                return

            # Сохраняем результат в кэш
            self.results_cache[request_id] = result

            # Вызываем callback если он есть
            if request_id in self.pending_requests:
                callback = self.pending_requests[request_id]
                callback(result)
                del self.pending_requests[request_id]

        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"❌ Ошибка парсинга результата: {e}")
        except Exception as e:
            self.node.get_logger().error(f"❌ Ошибка обработки результата: {e}")

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

        # Формируем запрос
        request = {"tool_name": tool_name, "parameters": parameters, "request_id": request_id}

        # Публикуем запрос
        request_msg = String()
        request_msg.data = json.dumps(request, ensure_ascii=False)
        self.execute_pub.publish(request_msg)

        self.node.get_logger().info(f"📤 Отправлен запрос {request_id[:8]}: {tool_name}")

        # Ожидаем результат с таймаутом используя polling результатов в кэше
        # ВАЖНО: не используем spin_once т.к. находимся внутри callback'а dialogue_node
        # Результаты приходят в кэш через on_result callback который вызывается асинхронно
        start_time = time.time()
        while request_id not in self.results_cache:
            if time.time() - start_time > timeout:
                self.node.get_logger().error(f"⏱️ Timeout ожидания результата для {tool_name} (request_id: {request_id[:8]})")
                return {"success": False, "error": "Timeout ожидания результата инструмента"}

            # Короткая пауза для снижения CPU load
            time.sleep(0.05)

        # Получаем результат из кэша
        result_data = self.results_cache.pop(request_id)
        return result_data.get("result", {"success": False, "error": "Пустой результат"})

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
