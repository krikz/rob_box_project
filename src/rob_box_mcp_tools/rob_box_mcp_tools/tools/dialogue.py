#!/usr/bin/env python3
"""
dialogue.py - Инструменты для управления диалогом с пользователем

Инструменты:
- SpeakTextTool: Произнести текст голосом (TTS)
- ListenForResponseTool: Ждать ответ пользователя (STT)
"""

from typing import List
from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class SpeakTextTool(MCPTool):
    """Инструмент для произнесения текста голосом"""

    def __init__(self, node):
        super().__init__(node)
        # Publisher для TTS запросов
        self.tts_pub = node.create_publisher(String, "/voice/tts/request", 10)

    @property
    def name(self) -> str:
        return "speak_text"

    @property
    def description(self) -> str:
        return (
            "Произнести текст голосом через TTS. "
            "ИСПОЛЬЗУЙ ЭТО вместо возврата JSON с SSML. "
            "Можешь вызвать несколько раз для разных фраз, делать паузы между ними через play_sound или set_emotion."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="text",
                type="string",
                description="Текст для произнесения. Можно использовать русские ударения (+ после гласной).",
                required=True,
            ),
            MCPToolParameter(
                name="emotion",
                type="string",
                description="Эмоция для выражения (happy, sad, angry, neutral). Опционально.",
                required=False,
                enum=["happy", "sad", "angry", "neutral", "excited"],
            ),
        ]

    def execute(self, text: str, emotion: str = "neutral") -> MCPToolResult:
        """Произнести текст"""
        self.log_info(f"Произношение текста: {text[:50]}... (emotion: {emotion})")

        if not text:
            return MCPToolResult(success=False, error="Пустой текст", message="Текст не может быть пустым")

        # Формируем SSML с эмоцией
        if emotion and emotion != "neutral":
            # Добавляем pitch для эмоций
            pitch_map = {"happy": "high", "sad": "low", "angry": "high", "excited": "x-high"}
            pitch = pitch_map.get(emotion, "medium")
            ssml_text = f"<speak><prosody pitch='{pitch}'>{text}</prosody></speak>"
        else:
            ssml_text = f"<speak>{text}</speak>"

        # Публикуем запрос TTS
        msg = String()
        msg.data = ssml_text
        self.tts_pub.publish(msg)

        self.log_info(f"TTS запрос отправлен: {text[:30]}...")

        return MCPToolResult(
            success=True, data={"text": text, "emotion": emotion, "ssml": ssml_text}, message=f"Произношу: {text[:50]}..."
        )


class ListenForResponseTool(MCPTool):
    """Инструмент для ожидания ответа пользователя"""

    def __init__(self, node):
        super().__init__(node)
        # Publisher для запроса активации STT
        self.stt_request_pub = node.create_publisher(String, "/voice/stt/request", 10)

    @property
    def name(self) -> str:
        return "listen_for_response"

    @property
    def description(self) -> str:
        return (
            "Ждать ответ пользователя через речь. "
            "Робот активирует микрофон и будет ждать ответа (таймаут 30 секунд). "
            "ИСПОЛЬЗУЙ когда задал вопрос пользователю или ждёшь продолжения диалога."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="timeout_seconds",
                type="integer",
                description="Таймаут ожидания в секундах (по умолчанию 30)",
                required=False,
            ),
            MCPToolParameter(
                name="prompt_text",
                type="string",
                description="Текст-подсказка что ждёшь от пользователя (для логирования)",
                required=False,
            ),
        ]

    def execute(self, timeout_seconds: int = 30, prompt_text: str = "") -> MCPToolResult:
        """Активировать режим ожидания ответа"""
        self.log_info(f"Ожидание ответа пользователя (таймаут: {timeout_seconds}s)")

        if prompt_text:
            self.log_info(f"Подсказка: {prompt_text}")

        # Публикуем запрос на активацию STT
        msg = String()
        msg.data = f"listen:{timeout_seconds}"
        self.stt_request_pub.publish(msg)

        self.log_info("STT активирован для ожидания ответа")

        return MCPToolResult(
            success=True,
            data={"timeout_seconds": timeout_seconds, "prompt_text": prompt_text},
            message=f"Жду ответ пользователя ({timeout_seconds}s таймаут)",
        )
