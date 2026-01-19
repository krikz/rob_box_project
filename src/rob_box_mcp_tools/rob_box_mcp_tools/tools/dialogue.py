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
        # Subscriber для получения завершения произношения
        self.finished_sub = node.create_subscription(String, "/voice/tts/finished", self._on_tts_finished, 10)
        # Кэш ожидающих произношений: speech_id -> result
        self.pending_speeches = {}
        import threading
        self.pending_speeches_lock = threading.Lock()

    def _on_tts_finished(self, msg: String):
        """Обработка завершения произношения"""
        import json
        try:
            result = json.loads(msg.data)
            speech_id = result.get("speech_id")
            if speech_id and speech_id in self.pending_speeches:
                with self.pending_speeches_lock:
                    self.pending_speeches[speech_id] = result
        except json.JSONDecodeError:
            pass

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
        import json
        import uuid
        import time
        import rclpy
        
        self.log_info(f"Произношение текста: {text[:50]}... (emotion: {emotion})")

        if not text:
            return MCPToolResult(success=False, error="Пустой текст", message="Текст не может быть пустым")

        # Генерируем speech_id
        speech_id = str(uuid.uuid4())

        # Формируем SSML с эмоцией
        if emotion and emotion != "neutral":
            pitch_map = {"happy": "high", "sad": "low", "angry": "high", "excited": "x-high"}
            pitch = pitch_map.get(emotion, "medium")
            ssml_text = f"<speak><prosody pitch='{pitch}'>{text}</prosody></speak>"
        else:
            ssml_text = f"<speak>{text}</speak>"

        # Регистрируем ожидание
        with self.pending_speeches_lock:
            self.pending_speeches[speech_id] = None

        # Публикуем запрос TTS в JSON формате
        tts_request = {"ssml": ssml_text, "speech_id": speech_id}
        msg = String()
        msg.data = json.dumps(tts_request, ensure_ascii=False)
        self.tts_pub.publish(msg)

        self.log_info(f"TTS запрос отправлен: {text[:30]}... (speech_id: {speech_id[:8]})")

        # Ждём завершения с таймаутом 20 секунд
        timeout = 20.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.pending_speeches_lock:
                result = self.pending_speeches.get(speech_id)
                if result is not None:
                    # Получили результат!
                    del self.pending_speeches[speech_id]
                    if result.get("success"):
                        self.log_info(f"✅ Произношение завершено: {text[:30]}...")
                        return MCPToolResult(
                            success=True, 
                            data={"text": text, "emotion": emotion, "speech_id": speech_id}, 
                            message=f"Произнесено: {text[:50]}..."
                        )
                    else:
                        error = result.get("error", "Unknown error")
                        self.log_warn(f"⚠️ Ошибка произношения: {error}")
                        return MCPToolResult(success=False, error=error, message=f"Ошибка TTS: {error}")
            
            # Спим немного и спиним ноду
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Таймаут
        with self.pending_speeches_lock:
            if speech_id in self.pending_speeches:
                del self.pending_speeches[speech_id]
        
        self.log_error(f"⏱️ Timeout ожидания произношения (20с): {text[:30]}...")
        return MCPToolResult(success=False, error="Timeout ожидания произношения", message="TTS не ответил в течение 20 секунд")


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
