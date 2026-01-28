#!/usr/bin/env python3
"""
dialogue_node_mcp_integration.py - Пример интеграции MCP tools с dialogue_node

Этот файл показывает как интегрировать MCP инструменты в dialogue_node.
Основные изменения:
1. Добавить LLMToolCallAdapter для обработки tool_calls
2. Получать список инструментов из MCP сервера
3. Передавать tools в LLM API (DeepSeek, Qwen, или любой OpenAI-совместимый)
4. Обрабатывать tool_calls из streaming ответов
"""

import json
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Импорт MCP адаптера
sys.path.insert(0, str(Path(__file__).parent.parent))
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

try:
    from openai import OpenAI
except ImportError:
    print("❌ Ошибка: установите openai library (pip install openai)")
    sys.exit(1)


class DialogueNodeWithMCP(Node):
    """
    Пример dialogue_node с интеграцией MCP tools

    ОСНОВНЫЕ ИЗМЕНЕНИЯ от оригинального dialogue_node:
    1. Добавлен LLMToolCallAdapter для обработки tool calls
    2. Добавлена подписка на /mcp/tools для получения списка инструментов
    3. Tools передаются в LLM API при каждом запросе (работает с DeepSeek, Qwen, и др.)
    4. Tool calls обрабатываются в streaming режиме
    """

    def __init__(self):
        super().__init__("dialogue_node_mcp")

        # ========== НОВОЕ: MCP Integration ==========
        # Адаптер для обработки tool calls
        self.mcp_adapter = LLMToolCallAdapter(self)

        # Подписка на список доступных инструментов
        self.tools_sub = self.create_subscription(String, "/mcp/tools", self.on_tools_update, 10)

        # Кэш доступных инструментов (OpenAI Tool Calls format)
        self.available_tools = []

        # ========== Оригинальная конфигурация dialogue_node ==========
        # Пример с DeepSeek, но можно использовать любой OpenAI-совместимый API
        self.declare_parameter("provider", "deepseek")
        self.declare_parameter("api_key", "")
        self.declare_parameter("base_url", "https://api.deepseek.com")  # Или Qwen, OpenAI, и др.
        self.declare_parameter("model", "deepseek-chat")  # Или qwen-max, gpt-4, и др.
        self.declare_parameter("temperature", 0.7)
        self.declare_parameter("max_tokens", 500)
        self.declare_parameter("system_prompt_file", "master_prompt.txt")

        # LLM клиент (универсальный для любого OpenAI-совместимого API)
        api_key = self.get_parameter("api_key").value or os.getenv("LLM_API_KEY") or os.getenv("DEEPSEEK_API_KEY")
        base_url = self.get_parameter("base_url").value
        self.model = self.get_parameter("model").value
        
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.temperature = self.get_parameter("temperature").value
        self.max_tokens = self.get_parameter("max_tokens").value

        # Подписки и публикации
        self.stt_sub = self.create_subscription(String, "/voice/stt/result", self.stt_callback, 10)
        self.response_pub = self.create_publisher(String, "/voice/dialogue/response", 10)

        # История диалога
        self.conversation_history = []
        self.system_prompt = self._load_system_prompt()

        self.get_logger().info("✅ DialogueNodeWithMCP инициализирован")
        self.get_logger().info("   🛠️ MCP Tools integration: ENABLED")

    def _load_system_prompt(self) -> str:
        """Загрузить system prompt"""
        # Упрощённый промпт для примера
        return """Ты ROBBOX - мобильный робот-ассистент.

У тебя есть доступ к различным инструментам для управления твоими функциями.
Используй инструменты когда пользователь просит выполнить действие.

Примеры:
- "Иди к кухне" → используй navigate_to_waypoint
- "Говори громче" → используй set_volume
- "Покажи анимацию радости" → используй play_animation

Отвечай в формате JSON:
{"ssml": "<speak>твой ответ</speak>"}
"""

    def on_tools_update(self, msg: String):
        """
        НОВОЕ: Обработка обновления списка инструментов из MCP сервера

        Получаем список инструментов в OpenAI Tool Calls формате
        """
        try:
            self.available_tools = json.loads(msg.data)
            self.get_logger().info(f"🛠️ Получено {len(self.available_tools)} инструментов из MCP сервера")
            for tool in self.available_tools:
                tool_name = tool.get("function", {}).get("name", "unknown")
                self.get_logger().debug(f"   - {tool_name}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга списка инструментов: {e}")

    def stt_callback(self, msg: String):
        """Обработка распознанной речи"""
        user_message = msg.data.strip()
        if not user_message:
            return

        self.get_logger().info(f"👤 User: {user_message}")

        # Добавляем в историю
        self.conversation_history.append({"role": "user", "content": user_message})

        # Запрос к LLM с tool calls
        self._ask_llm_with_tools()

    def _ask_llm_with_tools(self):
        """
        НОВОЕ: Запрос к LLM с поддержкой tool calls

        Основные отличия от оригинального _ask_llm_streaming:
        1. Передаём tools в API запрос
        2. Обрабатываем tool_calls в streaming ответе
        3. Отправляем результаты tool calls обратно в LLM
        """
        messages = [{"role": "system", "content": self.system_prompt}, *self.conversation_history]

        self.get_logger().info(f"🤖 Запрос к LLM с {len(self.available_tools)} доступными инструментами")

        try:
            # ========== НОВОЕ: Передаём tools в API ==========
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                tools=self.available_tools if self.available_tools else None,  # ← НОВОЕ!
                stream=True,
            )

            full_response = ""
            tool_calls_buffer = []  # Буфер для накопления tool calls

            for chunk in stream:
                delta = chunk.choices[0].delta

                # Обработка обычного контента
                if delta.content:
                    full_response += delta.content
                    # ... публикация chunks как в оригинальном коде ...

                # ========== НОВОЕ: Обработка tool calls ==========
                if hasattr(delta, "tool_calls") and delta.tool_calls:
                    for tool_call_delta in delta.tool_calls:
                        # Накапливаем tool calls (они могут приходить по частям в streaming)
                        # Здесь упрощённая обработка для примера
                        tool_calls_buffer.append(tool_call_delta)
                        self.get_logger().info(f"🔧 Tool call обнаружен: {tool_call_delta}")

                # Проверка finish_reason
                if chunk.choices[0].finish_reason:
                    if chunk.choices[0].finish_reason == "tool_calls":
                        self.get_logger().info("🔧 LLM запросил выполнение инструментов")
                        # Обрабатываем tool calls
                        self._handle_tool_calls(messages, tool_calls_buffer)
                        return  # Рекурсивно вернёмся к LLM после выполнения tools
                    break

            # Сохраняем ответ в историю
            self.conversation_history.append({"role": "assistant", "content": full_response})
            self.get_logger().info(f"✅ DeepSeek ответил")

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка LLM запроса: {e}")

    def _handle_tool_calls(self, messages: list, tool_calls_buffer: list):
        """
        НОВОЕ: Обработка tool calls от LLM

        Args:
            messages: Текущая история сообщений
            tool_calls_buffer: Список tool calls для выполнения
        """
        self.get_logger().info(f"🔧 Обработка {len(tool_calls_buffer)} tool calls")

        # Реконструируем полные tool calls из delta chunks
        # В реальной реализации нужна более сложная логика агрегации
        # Здесь упрощённая версия для демонстрации

        tool_results = []
        for tool_call_delta in tool_calls_buffer:
            if hasattr(tool_call_delta, "function"):
                tool_name = tool_call_delta.function.name
                # В streaming tool_call.function.arguments может приходить по частям
                # Нужно накопить и распарсить полный JSON
                tool_args = json.loads(tool_call_delta.function.arguments)

                self.get_logger().info(f"   → Выполнение: {tool_name}({tool_args})")

                # Выполняем инструмент через MCP адаптер
                result = self.mcp_adapter.execute_tool_call_sync(tool_name, tool_args)

                tool_results.append(
                    {
                        "tool_call_id": tool_call_delta.id,
                        "tool_name": tool_name,
                        "success": result.get("success", False),
                        "message": result.get("message", ""),
                        "data": result.get("data", {}),
                    }
                )

        # Форматируем результаты для отправки обратно в LLM
        tool_messages = self.mcp_adapter.format_tool_results_for_llm(tool_results)

        # Добавляем tool call и результаты в историю
        # NOTE: Нужно добавить assistant message с tool_calls перед tool results
        # Упрощённая версия для примера:
        messages.extend(tool_messages)

        self.get_logger().info(f"✅ Все инструменты выполнены, возвращаемся к LLM")

        # Делаем новый запрос к LLM с результатами tool calls
        self._continue_after_tools(messages)

    def _continue_after_tools(self, messages: list):
        """
        НОВОЕ: Продолжение диалога после выполнения инструментов

        LLM получает результаты выполнения и генерирует финальный ответ пользователю
        """
        self.get_logger().info("🤖 Запрос к LLM с результатами инструментов")

        try:
            # Запрос к LLM с результатами tools
            stream = self.client.chat.completions.create(
                model=self.model, messages=messages, temperature=self.temperature, max_tokens=self.max_tokens, stream=True
            )

            full_response = ""
            for chunk in stream:
                if chunk.choices[0].delta.content:
                    full_response += chunk.choices[0].delta.content
                    # Публикация chunks...

                if chunk.choices[0].finish_reason:
                    break

            # Сохраняем финальный ответ
            self.conversation_history.append({"role": "assistant", "content": full_response})
            self.get_logger().info("✅ Финальный ответ сгенерирован")

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка продолжения после tools: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DialogueNodeWithMCP()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
