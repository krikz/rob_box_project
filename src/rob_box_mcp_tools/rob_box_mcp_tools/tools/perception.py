#!/usr/bin/env python3
"""
perception.py - Инструменты для запроса данных восприятия

Инструменты:
- GetPerceptionContextTool: Получить текущий контекст восприятия
- GetBatteryLevelTool: Получить уровень заряда батареи
"""

from typing import Callable, List, Optional
import time

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType

#: Агрегатор шлёт контекст 2 раза в секунду. Снимок старше этого - восприятие молчит (issue #4352).
CONTEXT_STALE_S = 5.0


class GetPerceptionContextTool(MCPTool):
    """Инструмент для получения контекста восприятия."""

    def __init__(self, node, clock: Callable[[], float] = time.monotonic):
        super().__init__(node)
        # Кэш последнего контекста (будет обновляться из подписки)
        self.last_context: Optional[dict] = None
        self._clock = clock
        self._received_at: Optional[float] = None

    @property
    def name(self) -> str:
        return "get_perception_context"

    @property
    def description(self) -> str:
        # Честное описание (issue #2532): что реально лежит в контексте,
        # задаётся PROJECTED_FIELDS в perception_projection.py. Зрительные
        # события (vision_events_json) входят туда с PR #2583 — stub-детекции
        # отсекает продюсер. Про камеру сказано первым: без этого на «что ты
        # видишь» модель не находила тул и отвечала из головы (16.09.2026).
        return (
            "Что робот сейчас видит камерой и чувствует. vision_context — готовая фраза о кадре "
            "(лица с именем, если узнаны, силуэты, метки AprilTag; «нет данных», если данных нет) — "
            "говори о кадре только по ней. faces — лица в кадре (name=null — не узнан). Ещё: "
            "время суток, заряд батареи, температура, движение (is_moving), здоровье системы, "
            "режим картографирования, сводка недавних событий. Вызывай на «что ты видишь», "
            "«кто перед тобой», «видишь меня»."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        """Get perception context - MEDIUM операция (кэшированные данные, но может потребовать запросов)."""
        return ToolExecutionType.MEDIUM

    def execute(self) -> MCPToolResult:
        """Получить контекст восприятия."""
        self.log_info("Запрос контекста восприятия")

        if self.last_context is None:
            return MCPToolResult(
                success=False, error="Контекст восприятия недоступен", message="Данные восприятия ещё не получены"
            )

        age = self._clock() - self._received_at
        if age > CONTEXT_STALE_S:
            # Старый снимок не выдаём за «сейчас»: кадр и метки обнуляются честной фразой.
            data = dict(self.last_context, faces=[], apriltag_ids=[], perception_age_s=round(age, 1))
            data["vision_context"] = f"камера: нет данных (восприятие молчит {age:.0f} с)"
            return MCPToolResult(success=True, data=data, message="Контекст восприятия устарел")

        return MCPToolResult(success=True, data=self.last_context, message="Контекст восприятия получен")

    def update_context(self, context: dict):
        """Обновить кэш контекста (вызывается из MCP сервера при получении данных)."""
        self.last_context = context
        self._received_at = self._clock()


class GetBatteryLevelTool(MCPTool):
    """Инструмент для получения уровня заряда батареи."""

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
        """Get battery level - FAST операция (кэшированные данные)."""
        return ToolExecutionType.FAST

    def execute(self) -> MCPToolResult:
        """Получить уровень батареи."""
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
        """Обновить уровень батареи (вызывается из MCP сервера)."""
        self.battery_level = level
