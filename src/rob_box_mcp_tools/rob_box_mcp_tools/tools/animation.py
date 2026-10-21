#!/usr/bin/env python3
"""
animation.py - Инструменты для управления LED анимациями

Инструменты:
- PlayAnimationTool: Запустить анимацию по имени с указанной длительностью
"""

from typing import Dict, List, Optional

from rob_box_dialog import face_scene
from rob_box_dialog import knowledge as kn

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..animations import KNOWN_ANIMATIONS, clamp_show_seconds, normalize_animation


def _face_properties() -> Dict[str, MCPToolParameter]:
    """Слоты параметра ``face``: enum из ``knowledge.FACE_SCENE_SLOTS`` - та же схема, что у ``Command.expression.face``.

    Каталог тулов собирает эту схему сам (``tools/gen_tool_catalog.py::_face_param``) из той же таблицы.
    """
    schema = face_scene.schema()
    slots = {
        name: MCPToolParameter(name=name, type="string", description=f"Слот лица «{name}»", required=False,
                               enum=list(prop["enum"]))
        for name, prop in schema["properties"].items() if "enum" in prop
    }
    slots["duration_s"] = MCPToolParameter(
        name="duration_s", type="number", description="Сколько держать сцену, секунды", required=False,
        minimum=0, maximum=kn.FACE_SCENE_MAX_S)
    return slots


class PlayAnimationTool(MCPTool):
    """Инструмент для запуска LED анимации."""

    #: Kept as a class attribute for backwards compatibility; the list
    #: itself lives in :mod:`rob_box_mcp_tools.animations` so ``speak_text``
    #: and ``play_animation`` cannot disagree about what the matrix can show.
    AVAILABLE_ANIMATIONS = list(KNOWN_ANIMATIONS)

    # ADR-0150 PR-3b (#3690): тул лицо не трогает. Сцену и время показывает диалог: он видит вызов
    # (стадия executing хода) и шлёт ExpressionRequest через expression_client, лицо отвечает ExpressionEvent.

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
                description="Название анимации для воспроизведения (можно вместе с face)",
                required=False,
                enum=self.AVAILABLE_ANIMATIONS,
            ),
            MCPToolParameter(
                name="duration",
                type="number",
                description="Длительность анимации в секундах (рекомендуется от 2 до 30, значения вне диапазона будут установлены в 2)",
                required=False,
            ),
            MCPToolParameter(
                name="face",
                type="object",
                description=face_scene.schema()["description"],
                required=False,
                properties=_face_properties(),
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Анимации - мгновенные (fire-and-forget)."""
        return ToolExecutionType.INSTANT

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: LED-анимации. personality.
        return "personality"

    @property
    def blocking(self) -> bool:
        """Анимации не блокируют диалог."""
        return False

    def execute(self, animation: Optional[str] = None, duration: Optional[float] = None,
                face: Optional[dict] = None) -> MCPToolResult:
        """Запустить анимацию и/или сцену лица из слотов (нужно хотя бы одно)."""
        self.log_info(f"Запуск анимации: {animation}, face={face}, длительность: {duration}s")

        scene, face_why = (None, "") if face is None else face_scene.validate(face)
        if face is not None and scene is None:
            self.log_info(f"face отброшен: {face_why}")
        if not animation and scene is None:
            return MCPToolResult(
                success=False,
                error=f"Нужна анимация или корректный face ({face_why or 'ничего не задано'})",
                message=f"Слоты face: {', '.join(kn.FACE_SCENE_SLOTS)}",
            )

        data = {}
        if animation:
            # Псевдонимы (``excited`` от MiniMax M3, русские названия) приводятся к реальному имени манифеста общей
            # картой - той же, что у ``speak_text``.
            resolved = normalize_animation(animation, fallback="")
            if not resolved:
                return MCPToolResult(
                    success=False,
                    error=f"Неизвестная анимация: {animation}",
                    message=f"Доступные: {', '.join(KNOWN_ANIMATIONS)}",
                )
            data["animation"] = resolved

        # Валидация длительности - вне диапазона ставится минимальная (то же правило применяет диалог)
        if duration is not None:
            original_duration, duration = duration, clamp_show_seconds(duration)
            if duration != original_duration:
                self.log_info(f"Длительность {original_duration}s вне диапазона, установлена минимальная: {duration}s")
            data["duration"] = duration
        if scene is not None:
            data["face"] = scene.to_dict()

        self.log_info(f"Принято {data} (показывает диалог по ExpressionRequest)")
        dropped = f" (face отброшен: {face_why})" if face is not None and scene is None else ""
        what = data.get("animation") or "сцена лица"
        return MCPToolResult(success=True, data=data, message=f"Показываю анимацию: {what}{dropped}")
