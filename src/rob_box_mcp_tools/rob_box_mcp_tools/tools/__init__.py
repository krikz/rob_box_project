"""
tools - Коллекция MCP инструментов для Rob Box

Категории инструментов:
- navigation: Навигация и движение робота
- system: Управление системой (громкость, TTS и т.д.)
- perception: Запрос данных восприятия
- animation: Управление LED анимациями
- sound: Управление звуковыми эффектами
"""

from .navigation import *
from .system import *
from .perception import *

__all__ = [
    # Navigation tools
    "NavigateToWaypointTool",
    "MoveDirectionTool",
    "StopNavigationTool",
    "ListWaypointsTool",
    # System tools
    "SetVolumeTool",
    "SetPitchTool",
    "SetSpeedTool",
    "GetRobotStatusTool",
    # Perception tools
    "GetPerceptionContextTool",
    "GetBatteryLevelTool",
]
