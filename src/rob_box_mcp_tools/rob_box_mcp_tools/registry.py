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
    """Реестр инструментов MCP-сервера.

    Помимо регистрации и поиска, нормализует устаревшие/альтернативные
    имена параметров (``aliases``) перед валидацией. Например, для
    инструмента ``set_vibe_preset`` принимается и ``preset_name``, и
    ``preset`` (issue #935 — LLM иногда путал их, и сервер падал с
    "Отсутствует обязательный параметр: preset_name"). Алиасы объявляются
    через ``MCPTool.aliases`` (по умолчанию пустой dict).
    """

    def __init__(self) -> None:
        self._tools: Dict[str, MCPTool] = {}

    # ------------------------------------------------------------------
    # Parameter alias normalization — issue #935
    # ------------------------------------------------------------------

    #: Алиасы, которые применяются к **любому** инструменту. Полезно как
    #: «escape hatch», когда старая LLM-промпт использует устаревшее
    #: имя параметра, а менять schema нельзя (рассылаем LLM-агенту).
    GLOBAL_PARAM_ALIASES: Dict[str, Dict[str, str]] = {
        # tool_name: {alias_param: canonical_param}
        "set_vibe_preset": {"preset": "preset_name"},
    }

    def normalize_params(self, tool_name: str, **kwargs) -> Dict[str, Any]:
        """Перевести альтернативные имена параметров в канонические.

        Делается **до** ``validate_parameters`` чтобы сервер не отвергал
        корректный запрос только потому, что LLM использовал старое имя.
        Канонические имена имеют приоритет; алиас применяется только когда
        канонический параметр ещё не передан.
        """
        tool = self._tools.get(tool_name)
        alias_map: Dict[str, str] = {}
        if tool is not None:
            aliases = getattr(tool, "aliases", None)
            if isinstance(aliases, dict):
                alias_map.update(aliases)
        alias_map.update(self.GLOBAL_PARAM_ALIASES.get(tool_name, {}))
        if not alias_map:
            return kwargs
        normalized = dict(kwargs)
        for alias_name, canonical_name in alias_map.items():
            if (
                alias_name in normalized
                and canonical_name not in normalized
            ):
                normalized[canonical_name] = normalized.pop(alias_name)
            elif alias_name in normalized and canonical_name in normalized:
                # Both present — drop the alias to keep canonical value
                # authoritative (avoids 'multiple values' surprises).
                normalized.pop(alias_name, None)
        return normalized

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

    def get_openai_tools(self) -> List[Dict[str, Any]]:
        """
        Получить список всех инструментов в OpenAI Tool Calls формате

        Совместимо с DeepSeek, Qwen, OpenAI и другими провайдерами.

        Returns:
            Список словарей в формате OpenAI functions
        """
        return [tool.to_openai_tool_format() for tool in self._tools.values()]

    # Backward compatibility alias
    def get_deepseek_tools(self) -> List[Dict[str, Any]]:
        """Устаревший метод. Используйте get_openai_tools()."""
        return self.get_openai_tools()

    def execute(self, tool_name: str, **kwargs) -> MCPToolResult:
        """
        Выполнить инструмент по имени с заданными параметрами

        Args:
            tool_name: Имя инструмента
            **kwargs: Параметры для инструмента

        Returns:
            MCPToolResult: Результат выполнения
        """
        tool = self.get_tool(tool_name)
        if tool is None:
            return MCPToolResult(success=False, error=f"Инструмент '{tool_name}' не найден")

        # Normalize parameter aliases (issue #935) so the legacy ``preset``
        # name still routes to ``preset_name`` instead of failing validation.
        normalized_kwargs = self.normalize_params(tool_name, **kwargs)

        # Валидация параметров
        valid, error_msg = tool.validate_parameters(**normalized_kwargs)
        if not valid:
            return MCPToolResult(success=False, error=f"Ошибка валидации параметров: {error_msg}")

        # Выполнение инструмента
        try:
            return tool.execute(**normalized_kwargs)
        except Exception as e:
            error_msg = f"Ошибка выполнения инструмента '{tool_name}': {str(e)}"
            if tool.node:
                tool.node.get_logger().error(error_msg)
            return MCPToolResult(success=False, error=error_msg)

    def __len__(self) -> int:
        """Количество зарегистрированных инструментов."""
        return len(self._tools)

    def __contains__(self, name: str) -> bool:
        """Проверка наличия инструмента."""
        return name in self._tools

    def __repr__(self) -> str:
        """Строковое представление реестра."""
        return f"MCPToolRegistry({len(self._tools)} tools: {', '.join(self.list_tools())})"
