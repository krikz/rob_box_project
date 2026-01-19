#!/usr/bin/env python3
"""
perception.py - Инструменты для запроса данных восприятия

Инструменты:
- GetPerceptionContextTool: Получить текущий контекст восприятия
- GetBatteryLevelTool: Получить уровень заряда батареи
"""

from typing import List, Optional
import json

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class GetPerceptionContextTool(MCPTool):
    """Инструмент для получения контекста восприятия"""

    def __init__(self, node):
        super().__init__(node)
        # Кэш последнего контекста (будет обновляться из подписки)
        self.last_context: Optional[dict] = None

    @property
    def name(self) -> str:
        return "get_perception_context"

    @property
    def description(self) -> str:
        return "Получить текущий контекст восприятия робота (vision, sensors, environment)."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []
    
    @property
    def execution_type(self) -> ToolExecutionType:
        """Get perception context - MEDIUM операция (кэшированные данные, но может потребовать запросов)"""
        return ToolExecutionType.MEDIUM

    def execute(self) -> MCPToolResult:
        """Получить контекст восприятия"""
        self.log_info("Запрос контекста восприятия")

        if self.last_context is None:
            return MCPToolResult(
                success=False, error="Контекст восприятия недоступен", message="Данные восприятия ещё не получены"
            )

        return MCPToolResult(success=True, data=self.last_context, message="Контекст восприятия получен")

    def update_context(self, context: dict):
        """Обновить кэш контекста (вызывается из MCP сервера при получении данных)"""
        self.last_context = context


class GetBatteryLevelTool(MCPTool):
    """Инструмент для получения уровня заряда батареи"""

    def __init__(self, node):
        super().__init__(node)
        self.battery_level: Optional[float] = None

    @property
    def name(self) -> str:
        return "get_battery_level"

    @property
    def description(self) -> str:
        return "Получить текущий уровень заряда батареи робота в процентах."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []
    
    @property
    def execution_type(self) -> ToolExecutionType:
        """Get battery level - FAST операция (кэшированные данные)"""
        return ToolExecutionType.FAST

    def execute(self) -> MCPToolResult:
        """Получить уровень батареи"""
        self.log_info("Запрос уровня батареи")

        if self.battery_level is None:
            return MCPToolResult(success=False, error="Данные о батарее недоступны")

        # Проверка уровня для предупреждения
        if self.battery_level < 20.0:
            message = f"⚠️ Низкий заряд батареи: {self.battery_level:.1f}%"
        elif self.battery_level < 50.0:
            message = f"Заряд батареи: {self.battery_level:.1f}%"
        else:
            message = f"Заряд батареи нормальный: {self.battery_level:.1f}%"

        return MCPToolResult(
            success=True, data={"battery_level": self.battery_level, "warning": self.battery_level < 20.0}, message=message
        )

    def update_battery(self, level: float):
        """Обновить уровень батареи (вызывается из MCP сервера)"""
        self.battery_level = level
