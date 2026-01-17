"""
tools - Коллекция MCP инструментов для Rob Box

Категории инструментов:
- navigation: Навигация и движение робота
- system: Управление системой (громкость, TTS и т.д.)
- perception: Запрос данных восприятия
- mapping: Управление картографированием (RTABMap)
- animation: Управление LED анимациями
- sound: Управление звуковыми эффектами
- dialogue: Управление диалогом (TTS, STT)
"""

from .navigation import *
from .system import *
from .perception import *
from .mapping import *
from .animation import *
from .sound import *
from .dialogue import *

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
    # Mapping tools
    "StartMappingTool",
    "ContinueMappingTool",
    "FinishMappingTool",
    # Animation tools
    "PlayAnimationTool",
    "SetEmotionTool",
    # Sound tools
    "PlaySoundTool",
    # Dialogue tools
    "SpeakTextTool",
    "ListenForResponseTool",
]
