#!/usr/bin/env python3
"""
animation.py - Инструменты для управления LED анимациями

Инструменты:
- PlayAnimationTool: Запустить анимацию по имени
- SetEmotionTool: Установить эмоцию через LED
"""

from typing import List, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class PlayAnimationTool(MCPTool):
    """Инструмент для запуска LED анимации"""

    # Доступные анимации
    AVAILABLE_ANIMATIONS = [
        "idle",
        "happy",
        "sad",
        "angry",
        "surprised",
        "thinking",
        "victory",
        "wakeup",
        "sleep",
        "talking",
        "error",
        "low_battery",
        "charging",
        # Дорожные
        "police_lights",
        "ambulance",
        "fire_truck",
        "road_service",
        "turn_left",
        "turn_right",
        "accelerating",
        "braking",
    ]

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String
        
        # Publisher для запроса анимаций
        self.animation_pub = node.create_publisher(String, "/voice/animation/request", 10)

    @property
    def name(self) -> str:
        return "play_animation"

    @property
    def description(self) -> str:
        return (
            "Запустить LED анимацию на матрице робота (381 LED). "
            "ИСПОЛЬЗУЙ АВТОМАТИЧЕСКИ для визуального выражения эмоций во время разговора. "
            "Когда показываешь анимации по запросу пользователя - НЕ описывай что происходит в анимации, "
            "просто говори: 'Показываю анимацию <название>' или 'Есть анимация <название>'. "
            "Примеры названий: 'полиция' → police_lights, 'пожарная' → fire_truck, 'скорая' → ambulance, "
            "'поворот налево' → turn_left, 'поворот направо' → turn_right."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="animation",
                type="string",
                description="Название анимации для воспроизведения",
                required=True,
                enum=self.AVAILABLE_ANIMATIONS,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Анимации - мгновенные (fire-and-forget)"""
        return ToolExecutionType.INSTANT

    @property
    def blocking(self) -> bool:
        """Анимации не блокируют диалог"""
        return False

    def execute(self, animation: str) -> MCPToolResult:
        """Запустить анимацию"""
        self.log_info(f"Запуск анимации: {animation}")

        if animation not in self.AVAILABLE_ANIMATIONS:
            return MCPToolResult(
                success=False,
                error=f"Неизвестная анимация: {animation}",
                message=f"Доступные: {', '.join(self.AVAILABLE_ANIMATIONS)}",
            )

        # Публикуем запрос анимации
        from std_msgs.msg import String
        msg = String()
        msg.data = animation
        self.animation_pub.publish(msg)

        self.log_info(f"Анимация '{animation}' отправлена")

        return MCPToolResult(success=True, data={"animation": animation}, message=f"Показываю анимацию: {animation}")


class SetEmotionTool(MCPTool):
    """Инструмент для установки эмоции через LED"""

    # Маппинг эмоций на анимации
    EMOTION_TO_ANIMATION = {
        "радость": "happy",
        "грусть": "sad",
        "злость": "angry",
        "удивление": "surprised",
        "думаю": "thinking",
        "победа": "victory",
        "привет": "wave",
        "нейтрально": "idle",
    }

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String
        
        self.animation_pub = node.create_publisher(String, "/voice/animation/request", 10)

    @property
    def name(self) -> str:
        return "set_emotion"

    @property
    def description(self) -> str:
        return "Установить эмоцию робота через LED анимацию. ИСПОЛЬЗУЙ АВТОМАТИЧЕСКИ во время каждого ответа для выражения соответствующей эмоции."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="emotion",
                type="string",
                description="Эмоция для выражения",
                required=True,
                enum=list(self.EMOTION_TO_ANIMATION.keys()),
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Эмоции - мгновенные (fire-and-forget)"""
        return ToolExecutionType.INSTANT

    @property
    def blocking(self) -> bool:
        """Эмоции не блокируют диалог"""
        return False

    def execute(self, emotion: str) -> MCPToolResult:
        """Установить эмоцию"""
        self.log_info(f"Установка эмоции: {emotion}")

        animation = self.EMOTION_TO_ANIMATION.get(emotion)
        if not animation:
            return MCPToolResult(success=False, error=f"Неизвестная эмоция: {emotion}")

        # Публикуем анимацию
        from std_msgs.msg import String
        msg = String()
        msg.data = animation
        self.animation_pub.publish(msg)

        self.log_info(f"Эмоция '{emotion}' → анимация '{animation}'")

        return MCPToolResult(success=True, data={"emotion": emotion, "animation": animation}, message=f"Выражаю эмоцию: {emotion}")
