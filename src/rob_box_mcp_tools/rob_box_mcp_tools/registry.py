#!/usr/bin/env python3
"""
registry.py - Реестр MCP инструментов

MCPToolRegistry управляет коллекцией инструментов:
- Регистрация инструментов
- Поиск инструментов по имени
- Получение списка всех инструментов для LLM
- Выполнение инструментов по имени
"""

from typing import Dict, List, Optional, Any
from .base import MCPTool, MCPToolResult
import json


class MCPToolRegistry:
    """Реестр MCP инструментов"""

    def __init__(self):
        self._tools: Dict[str, MCPTool] = {}

    def register(self, tool: MCPTool) -> None:
        """
        Зарегистрировать инструмент в реестре

        Args:
            tool: Экземпляр MCPTool для регистрации

        Raises:
            ValueError: Если инструмент с таким именем уже зарегистрирован
        """
        if tool.name in self._tools:
            raise ValueError(f"Инструмент '{tool.name}' уже зарегистрирован")

        self._tools[tool.name] = tool

    def unregister(self, name: str) -> bool:
        """
        Удалить инструмент из реестра

        Args:
            name: Имя инструмента

        Returns:
            True если инструмент был удален, False если не найден
        """
        if name in self._tools:
            del self._tools[name]
            return True
        return False

    def get_tool(self, name: str) -> Optional[MCPTool]:
        """
        Получить инструмент по имени

        Args:
            name: Имя инструмента

        Returns:
            MCPTool или None если не найден
        """
        return self._tools.get(name)

    def list_tools(self) -> List[str]:
        """
        Получить список имен всех зарегистрированных инструментов

        Returns:
            Список имен инструментов
        """
        return list(self._tools.keys())

    def get_deepseek_tools(self) -> List[Dict[str, Any]]:
        """
        Получить список всех инструментов в формате DeepSeek Tool Calls

        Returns:
            Список словарей в формате DeepSeek functions
        """
        return [tool.to_deepseek_function() for tool in self._tools.values()]

    def execute(self, name: str, **kwargs) -> MCPToolResult:
        """
        Выполнить инструмент по имени с заданными параметрами

        Args:
            name: Имя инструмента
            **kwargs: Параметры для инструмента

        Returns:
            MCPToolResult: Результат выполнения
        """
        tool = self.get_tool(name)
        if tool is None:
            return MCPToolResult(success=False, error=f"Инструмент '{name}' не найден")

        # Валидация параметров
        valid, error_msg = tool.validate_parameters(**kwargs)
        if not valid:
            return MCPToolResult(success=False, error=f"Ошибка валидации параметров: {error_msg}")

        # Выполнение инструмента
        try:
            return tool.execute(**kwargs)
        except Exception as e:
            error_msg = f"Ошибка выполнения инструмента '{name}': {str(e)}"
            if tool.node:
                tool.node.get_logger().error(error_msg)
            return MCPToolResult(success=False, error=error_msg)

    def __len__(self) -> int:
        """Количество зарегистрированных инструментов"""
        return len(self._tools)

    def __contains__(self, name: str) -> bool:
        """Проверка наличия инструмента"""
        return name in self._tools

    def __repr__(self) -> str:
        """Строковое представление реестра"""
        return f"MCPToolRegistry({len(self._tools)} tools: {', '.join(self.list_tools())})"
