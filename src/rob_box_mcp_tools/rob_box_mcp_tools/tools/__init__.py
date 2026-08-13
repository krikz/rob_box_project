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
- memory: Долгосрочная память (VoiceMemory + Ollama embeddings)
- music: Управление музыкой в реальном времени через Renardo
"""

from .navigation import *
from .system import *  # includes GetCurrentTimeTool, SetVolumeTool, etc.
from .perception import *
from .mapping import *
from .animation import *
from .sound import *
from .dialogue import *
from .memory import *
from .music import *
from .web_search import *

__all__ = [
    # Navigation tools
    "NavigateToWaypointTool",
    "NavigateToCoordinatesTool",
    "MoveDirectionTool",
    "StopNavigationTool",
    "ListWaypointsTool",
    "SaveWaypointTool",
    "DeleteWaypointTool",
    "ClearWaypointsTool",
    "GetCurrentPoseTool",
    # System tools
    "SetVolumeTool",
    "SetPitchTool",
    "SetSpeedTool",
    "GetRobotStatusTool",
    "GetCurrentTimeTool",
    # Perception tools
    "GetPerceptionContextTool",
    "GetBatteryLevelTool",
    # Mapping tools
    "StartMappingTool",
    "ContinueMappingTool",
    "FinishMappingTool",
    "OptimizeMapTool",
    "LoadMapTool",
    # Animation tools
    "PlayAnimationTool",
    # Sound tools
    "PlaySoundTool",
    "GetSoundInfoTool",
    # Dialogue tools
    "SpeakTextTool",
    "ListenForResponseTool",
    "SetVoiceTool",
    # Memory tools
    "MemorySaveTool",
    "MemorySearchTool",
    "MemoryContextTool",
    # Music tools
    "MusicManager",
    "ExecuteMusicCodeTool",
    "StopMusicTool",
    "SetVibePresetTool",
    "GetMusicStateTool",
    "SetDjModeTool",
    "SearchSamplesTool",
    # Track library tools
    "SaveTrackTool",
    "ListTracksTool",
    "LoadTrackTool",
    "DeleteTrackTool",
    # FAQ / Event tools
    "FaqSearchTool",
    # Web search tools (issue #1101)
    "SearchWebTool",
]
