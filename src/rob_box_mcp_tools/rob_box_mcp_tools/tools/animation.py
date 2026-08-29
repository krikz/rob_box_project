#!/usr/bin/env python3
"""
animation.py - Инструменты для управления LED анимациями

Инструменты:
- PlayAnimationTool: Запустить анимацию по имени с указанной длительностью
"""

from typing import List, Optional, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..animations import KNOWN_ANIMATIONS, normalize_animation


class PlayAnimationTool(MCPTool):
    """Инструмент для запуска LED анимации."""

    #: Kept as a class attribute for backwards compatibility; the list
    #: itself lives in :mod:`rob_box_mcp_tools.animations` so ``speak_text``
    #: and ``play_animation`` cannot disagree about what the matrix can show.
    AVAILABLE_ANIMATIONS = list(KNOWN_ANIMATIONS)

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
            "Запустить LED анимацию на матрице робота (381 LED) на указанное время. "
            "ОБЯЗАТЕЛЬНО используй анимации во время разговора для визуального выражения эмоций и действий. "
            "При рассказе историй, объяснениях, описаниях - ВСЕГДА добавляй подходящие анимации параллельно с текстом. "
            "Доступны анимации с эмоциями (happy, sad, angry, surprised, thinking, victory) "
            "и другие анимации (police_lights, fire_truck, ambulance, turn_left, turn_right). "
            "Когда показываешь анимации по запросу - НЕ описывай что происходит в анимации, "
            "просто говори: 'Показываю анимацию <название>' или 'Есть анимация <название>'."
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
            ),
            MCPToolParameter(
                name="duration",
                type="number",
                description="Длительность анимации в секундах (рекомендуется от 2 до 30, значения вне диапазона будут установлены в 2)",
                required=False,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Анимации - мгновенные (fire-and-forget)."""
        return ToolExecutionType.INSTANT

    @property
    def blocking(self) -> bool:
        """Анимации не блокируют диалог."""
        return False

    def execute(self, animation: str, duration: Optional[float] = None) -> MCPToolResult:
        """Запустить анимацию."""
        self.log_info(f"Запуск анимации: {animation}, длительность: {duration}s")

        # Псевдонимы (``excited`` от MiniMax M3, русские названия) приводятся
        # к реальному имени манифеста общей картой — раньше здесь был
        # одиночный ``if animation == "excited"``, из-за чего каждое новое
        # слово из эмоционального словаря модели приходилось добавлять сюда
        # руками, отдельно от такой же карты в ``speak_text``.
        resolved = normalize_animation(animation, fallback="")
        if not resolved:
            return MCPToolResult(
                success=False,
                error=f"Неизвестная анимация: {animation}",
                message=f"Доступные: {', '.join(KNOWN_ANIMATIONS)}",
            )
        animation = resolved

        # Валидация длительности - если вне диапазона, устанавливаем минимальную
        if duration is not None:
            if duration < 2 or duration > 30:
                original_duration = duration
                duration = 2.0
                self.log_info(f"Длительность {original_duration}s вне диапазона, установлена минимальная: {duration}s")

        # Публикуем запрос анимации
        from std_msgs.msg import String
        msg = String()
        # Если указана длительность, добавляем её в формате animation:duration
        if duration is not None:
            msg.data = f"{animation}:{duration}"
        else:
            msg.data = animation
        self.animation_pub.publish(msg)

        self.log_info(f"Анимация '{animation}' отправлена")

        result_data = {"animation": animation}
        if duration is not None:
            result_data["duration"] = duration

        return MCPToolResult(success=True, data=result_data, message=f"Показываю анимацию: {animation}")
