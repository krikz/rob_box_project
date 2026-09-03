#!/usr/bin/env python3
"""
say.py - Инструмент «say» для супервизор-агента (AV-21).

Минимальный TTS-инструмент для команд оператора:
- принимает только ``text`` (никаких voice/animation overrides);
- публикует запрос в существующий TTS-канал (как ``speak_text``);
- backend с voice-floor-логикой — в карточке AV-27 (Phase 2). Здесь —
  fire-and-forget заглушка: текст уходит в топик, executor возвращает
  ``success=True`` немедленно.

Зачем отдельный инструмент, когда есть ``speak_text``:
- ``speak_text`` завязан на dialogue_node (voice_held_by, emotion, анимация
  по умолчанию) — это контракт личности робота, а не оператора.
- ``say`` — контракт оператора: минимальный, «проговори мой текст»,
  без эмоций. Личность в этот момент МОЛЧИТ (ADR-0028 S5, voice-mode
  swap «off»), и ``say`` отвечает за «голос в тишине».

Это каркас (AV-21), не финальная интеграция. Полный TTS через
voice-floor — карточка AV-27; в этом PR ``say`` публикует в
``/voice/tts/request`` (тот же топик, что и ``speak_text``), и
``tts_node`` синтезирует независимо от dialogue_node state.
"""

from typing import List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from std_msgs.msg import String


from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class SayTool(MCPTool):
    """Произнести текст голосом оператора через TTS-канал (AV-21)."""

    #: Канонический ROS-топик, который уже слушает ``tts_node``.
    #: Совпадает с ``speak_text`` — это умышленно: ``tts_node`` не знает,
    #: от кого пришёл запрос, и ему не нужно (пол отправителя определяется
    #: на уровне супервизора через voice_floor).
    TTS_REQUEST_TOPIC: str = "/voice/tts/request"

    def __init__(self, node) -> None:
        super().__init__(node)
        # Динамический импорт, чтобы unit-тесты могли работать без ROS 2.
        from std_msgs.msg import String

        self._tts_pub = node.create_publisher(String, self.TTS_REQUEST_TOPIC, 10)

    @property
    def name(self) -> str:
        return "say"

    @property
    def description(self) -> str:
        return (
            "Произнести текст голосом от имени оператора робота. "
            "Используй, когда оператор дал голосовую или текстовую команду "
            "произнести что-то вслух (например, мотивирующая фраза, "
            "обращение к людям рядом, объявление). НЕ используй для "
            "эмоциональной реплики от лица робота — это делает speak_text. "
            "Пока say выполняется, личность робота молчит (ADR-0028 S5)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="text",
                type="string",
                description=(
                    "Текст для произнесения. Без ударений и SSML — операторский "
                    "voice-floor сам выберет провайдера и голос. "
                    "Можно использовать русский язык."
                ),
                required=True,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Fire-and-forget: текст ушёл в топик — LLM может продолжать."""
        return ToolExecutionType.INSTANT

    def execute(self, text: str) -> MCPToolResult:
        """Опубликовать текст в TTS-канал.

        Каркас (AV-21): не ждём подтверждения от ``tts_node``. Это
        согласовано с INSTANT-семантикой выше. Полная интеграция с
        voice-floor и barge-in — карточка AV-27.
        """
        if not text or not text.strip():
            return MCPToolResult(
                success=False,
                error="empty_text",
                message="Текст для произнесения пустой",
            )

        from std_msgs.msg import String

        msg = String()
        # Полезная нагрузка — JSON с маркером источника. tts_node читает
        # тот же формат, что и от speak_text; дополнительное поле
        # ``source`` нужно для метрик (avatar_agent_tool_calls_total).
        import json

        msg.data = json.dumps(
            {
                "text": text,
                "source": "operator",
            },
            ensure_ascii=False,
        )
        self._tts_pub.publish(msg)

        return MCPToolResult(
            success=True,
            data={"text": text, "source": "operator"},
            message=f"say: опубликовано {len(text)} символов в TTS-канал",
        )


__all__ = ["SayTool"]
