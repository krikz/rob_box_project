#!/usr/bin/env python3
"""
base.py - Базовые классы для MCP-подобной системы инструментов

MCPTool - базовый класс для всех инструментов, которые LLM может вызывать.
Каждый инструмент описывает свои параметры в формате JSON Schema (OpenAI Tool Calls format).
Совместимо с DeepSeek, Qwen, OpenAI и другими LLM провайдерами.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from enum import Enum
import json


class ToolExecutionType(Enum):
    """
    Тип выполнения инструмента для оптимизации async execution

    INSTANT: Мгновенные операции < 100ms (fire-and-forget)
        - Установка анимаций, эмоций
        - Не требует ожидания результата
        - Выполняется параллельно с другими операциями

    FAST: Быстрые операции < 2s (await completion)
        - Воспроизведение коротких звуков
        - Установка системных параметров
        - Требует ожидания завершения перед продолжением

    MEDIUM: Средние операции 2-10s (await with progress)
        - Запросы данных о состоянии
        - Получение контекста восприятия
        - Показ прогресса пользователю опционален

    LONG: Длительные операции > 10s (background task)
        - Навигация к точкам
        - Картографирование
        - Выполняется в фоне с возможностью прерывания
    """
    INSTANT = "instant"  # < 100ms, fire-and-forget
    FAST = "fast"        # < 2s, await completion
    MEDIUM = "medium"    # 2-10s, await with optional progress
    LONG = "long"        # > 10s, background task with callbacks


@dataclass
class MCPToolParameter:
    """Параметр инструмента (JSON Schema style)."""

    name: str
    type: str  # "string", "number", "integer", "boolean", "object", "array"
    description: str
    required: bool = True
    enum: Optional[List[str]] = None
    properties: Optional[Dict[str, "MCPToolParameter"]] = None  # Для type="object"
    items: Optional["MCPToolParameter"] = None  # Для type="array"
    default: Optional[Any] = None

    def to_json_schema(self) -> Dict[str, Any]:
        """Конвертировать в JSON Schema для OpenAI-совместимого Tool Calls формата."""
        schema: Dict[str, Any] = {
            "type": self.type,
            "description": self.description,
        }

        if self.enum is not None:
            schema["enum"] = self.enum

        if self.default is not None:
            schema["default"] = self.default

        if self.type == "object" and self.properties:
            schema["properties"] = {name: param.to_json_schema() for name, param in self.properties.items()}
            # В strict mode все свойства объекта должны быть required
            schema["required"] = [name for name, param in self.properties.items() if param.required]
            schema["additionalProperties"] = False

        if self.type == "array" and self.items:
            schema["items"] = self.items.to_json_schema()

        return schema


@dataclass
class MCPToolResult:
    """Результат выполнения инструмента."""

    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    message: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Конвертировать в словарь для передачи в LLM."""
        result = {"success": self.success}
        if self.data is not None:
            result["data"] = self.data
        if self.error is not None:
            result["error"] = self.error
        if self.message is not None:
            result["message"] = self.message
        return result

    def to_json(self) -> str:
        """Конвертировать в JSON строку."""
        return json.dumps(self.to_dict(), ensure_ascii=False)


class MCPTool(ABC):
    """
    Базовый класс для MCP инструмента.

    Каждый инструмент должен:
    1. Определить имя (name)
    2. Определить описание (description)
    3. Определить параметры (parameters)
    4. Определить тип выполнения (execution_type)
    5. Реализовать метод execute() для выполнения
    """

    def __init__(self, node: Optional[Any] = None):
        """
        Инициализация инструмента

        Args:
            node: ROS 2 Node для доступа к logger, publishers, services и т.д.
        """
        self.node = node

    @property
    @abstractmethod
    def name(self) -> str:
        """Уникальное имя инструмента (snake_case)."""
        pass

    @property
    @abstractmethod
    def description(self) -> str:
        """Описание инструмента для LLM (на русском)."""
        pass

    @property
    @abstractmethod
    def parameters(self) -> List[MCPToolParameter]:
        """Список параметров инструмента."""
        pass

    @property
    def execution_type(self) -> ToolExecutionType:
        """
        Тип выполнения инструмента (по умолчанию MEDIUM)

        Переопределите для оптимизации async execution:
        - INSTANT: анимации, эмоции (fire-and-forget)
        - FAST: звуки, системные параметры (< 2s)
        - MEDIUM: запросы данных (2-10s) [DEFAULT]
        - LONG: навигация, mapping (> 10s, background)
        """
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        """
        Инструмент не меняет состояние системы (readOnlyHint)

        - True: только чтение данных (напр., get_battery_level)
        - False: модифицирует состояние [DEFAULT]
        """
        return False

    @property
    def destructive(self) -> bool:
        """
        Инструмент выполняет разрушительные операции (destructiveHint)

        - True: может изменить/удалить данные [DEFAULT]
        - False: не деструктивен (напр., чтение, проигрывание звуков)
        """
        return True

    @property
    def idempotent(self) -> bool:
        """
        Повторный вызов с теми же аргументами не даёт побочного эффекта (idempotentHint)

        - True: каждый вызов даёт одинаковый результат
        - False: повторные вызовы могут добавлять эффекты [DEFAULT]
        """
        return False

    @property
    def blocking(self) -> bool:
        """
        Требуется ли ждать результата перед продолжением диалога

        - True: LLM получает результат перед генерацией ответа (DEFAULT)
        - False: fire-and-forget, результат не нужен для продолжения

        Для INSTANT обычно False, для остальных обычно True.
        """
        return self.execution_type != ToolExecutionType.INSTANT

    @property
    def interruptible(self) -> bool:
        """
        Можно ли прервать выполнение новым запросом от пользователя

        - True: операция прерывается при новом запросе (навигация, mapping)
        - False: операция должна завершиться (звуки, системные команды)

        Актуально только для LONG execution_type.
        """
        return self.execution_type == ToolExecutionType.LONG

    @abstractmethod
    def execute(self, **kwargs) -> MCPToolResult:
        """
        Выполнить инструмент с заданными параметрами

        Args:
            **kwargs: Параметры инструмента (имена соответствуют определенным в parameters)

        Returns:
            MCPToolResult: Результат выполнения
        """
        pass

    def to_openai_tool_format(self) -> Dict[str, Any]:
        """
        Конвертировать инструмент в OpenAI Tool Calls формат

        Совместимо с DeepSeek, Qwen, OpenAI и другими провайдерами.

        Returns:
            Dict в формате:
            {
                "type": "function",
                "function": {
                    "name": "tool_name",
                    "description": "Tool description",
                    "parameters": {
                        "type": "object",
                        "properties": {...},
                        "required": [...]
                    }
                }
            }
        """
        # Собираем properties из параметров
        properties = {}
        required = []

        for param in self.parameters:
            properties[param.name] = param.to_json_schema()
            if param.required:
                required.append(param.name)

        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": properties,
                    "required": required,
                    "additionalProperties": False,
                },
            },
            "annotations": {
                "readOnlyHint": self.read_only,
                "destructiveHint": self.destructive,
                "idempotentHint": self.idempotent,
            },
        }

    # Backward compatibility alias
    def to_deepseek_function(self) -> Dict[str, Any]:
        """Устаревший метод. Используйте to_openai_tool_format()."""
        return self.to_openai_tool_format()

    def validate_parameters(self, **kwargs) -> tuple[bool, Optional[str]]:
        """
        Валидация параметров перед выполнением

        Returns:
            (success, error_message)
        """
        # Проверяем наличие обязательных параметров
        for param in self.parameters:
            if param.required and param.name not in kwargs:
                return False, f"Отсутствует обязательный параметр: {param.name}"

        # Проверяем enum значения
        for param in self.parameters:
            if param.name in kwargs and param.enum is not None:
                if kwargs[param.name] not in param.enum:
                    return (
                        False,
                        f"Недопустимое значение для {param.name}: {kwargs[param.name]}. "
                        f"Допустимые: {', '.join(param.enum)}",
                    )

        return True, None

    def log_info(self, message: str):
        """Логирование info сообщения."""
        if self.node:
            self.node.get_logger().info(f"[{self.name}] {message}")
        else:
            print(f"[{self.name}] INFO: {message}")

    def log_warning(self, message: str):
        """Логирование warning сообщения."""
        if self.node:
            self.node.get_logger().warning(f"[{self.name}] {message}")
        else:
            print(f"[{self.name}] WARNING: {message}")

    def log_error(self, message: str):
        """Логирование error сообщения."""
        if self.node:
            self.node.get_logger().error(f"[{self.name}] {message}")
        else:
            print(f"[{self.name}] ERROR: {message}")
