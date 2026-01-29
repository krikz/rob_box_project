#!/usr/bin/env python3
"""
sound.py - Инструменты для управления звуковыми эффектами

Инструменты:
- PlaySoundTool: Воспроизвести звуковой эффект
"""

from typing import List, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class PlaySoundTool(MCPTool):
    """Инструмент для воспроизведения звуковых эффектов"""

    # Доступные звуковые эффекты (должны соответствовать файлам в sound_pack/)
    AVAILABLE_SOUNDS = [
        "thinking",
        "cute",
        "very_cute",
        "confused",
        "angry_1",
        "angry_2",
        "surprise",
        "talk_1",
        "talk_2",
        "talk_3",
        "talk_4",
        "error",  # Alias для angry_2 (звучит как ошибка)
    ]

    # Алиасы для звуков (сопоставление логического имени с физическим файлом)
    SOUND_ALIASES = {
        "error": "angry_2",  # error звучит как angry_2.mp3
    }

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String
        
        # Publisher для триггеров звуков
        self.sound_pub = node.create_publisher(String, "/voice/sound/trigger", 10)

    @property
    def name(self) -> str:
        return "play_sound"

    @property
    def description(self) -> str:
        return "Воспроизвести звуковой эффект. ИСПОЛЬЗУЙ АВТОМАТИЧЕСКИ для звукового сопровождения эмоций во время разговора."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="sound",
                type="string",
                description="Название звукового эффекта",
                required=True,
                enum=self.AVAILABLE_SOUNDS,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Звуки - мгновенные операции (fire-and-forget), sound_node не возвращает результат"""
        return ToolExecutionType.INSTANT

    def execute(self, sound: str) -> MCPToolResult:
        """Воспроизвести звук"""
        self.log_info(f"Воспроизведение звука: {sound}")

        if sound not in self.AVAILABLE_SOUNDS:
            return MCPToolResult(
                success=False, error=f"Неизвестный звук: {sound}", message=f"Доступные: {', '.join(self.AVAILABLE_SOUNDS)}"
            )

        # Преобразовать алиас в реальное имя файла (если есть)
        actual_sound = self.SOUND_ALIASES.get(sound, sound)
        
        # Публикуем триггер звука
        from std_msgs.msg import String
        msg = String()
        msg.data = actual_sound
        self.sound_pub.publish(msg)

        self.log_info(f"Звук '{sound}' (физический: '{actual_sound}') отправлен")

        return MCPToolResult(success=True, data={"sound": sound, "actual_sound": actual_sound}, message=f"Воспроизвожу звук: {sound}")
