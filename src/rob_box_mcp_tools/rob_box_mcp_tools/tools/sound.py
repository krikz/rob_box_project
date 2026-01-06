#!/usr/bin/env python3
"""
sound.py - Инструменты для управления звуковыми эффектами

Инструменты:
- PlaySoundTool: Воспроизвести звуковой эффект
"""

from typing import List
from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class PlaySoundTool(MCPTool):
    """Инструмент для воспроизведения звуковых эффектов"""

    # Доступные звуковые эффекты
    AVAILABLE_SOUNDS = [
        "thinking",
        "cute",
        "confused",
        "angry_1",
        "angry_2",
        "happy",
        "sad",
        "surprised",
        "beep",
        "success",
        "error",
        "notification",
    ]

    def __init__(self, node):
        super().__init__(node)
        # Publisher для триггеров звуков
        self.sound_pub = node.create_publisher(String, "/voice/sound/trigger", 10)

    @property
    def name(self) -> str:
        return "play_sound"

    @property
    def description(self) -> str:
        return "Воспроизвести звуковой эффект. Используй для звукового сопровождения действий робота."

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

    def execute(self, sound: str) -> MCPToolResult:
        """Воспроизвести звук"""
        self.log_info(f"Воспроизведение звука: {sound}")

        if sound not in self.AVAILABLE_SOUNDS:
            return MCPToolResult(
                success=False, error=f"Неизвестный звук: {sound}", message=f"Доступные: {', '.join(self.AVAILABLE_SOUNDS)}"
            )

        # Публикуем триггер звука
        msg = String()
        msg.data = sound
        self.sound_pub.publish(msg)

        self.log_info(f"Звук '{sound}' отправлен")

        return MCPToolResult(success=True, data={"sound": sound}, message=f"Воспроизвожу звук: {sound}")
