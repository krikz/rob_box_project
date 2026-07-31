#!/usr/bin/env python3
"""
dialogue.py - Инструменты для управления диалогом с пользователем

Инструменты:
- SpeakTextTool: Произнести текст голосом (TTS)
- ListenForResponseTool: Ждать ответ пользователя (STT)
"""

from __future__ import annotations
from typing import List, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class SpeakTextTool(MCPTool):
    """Инструмент для произнесения текста голосом."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String

        # Publisher для TTS запросов
        self.tts_pub = node.create_publisher(String, "/voice/tts/request", 10)
        # Publisher для анимаций (инициализируем сразу, чтобы не создавать дубли в execute())
        self.animation_pub = node.create_publisher(String, "/voice/animation/request", 10)
        # Subscriber для получения завершения произношения
        self.finished_sub = node.create_subscription(String, "/voice/tts/finished", self._on_tts_finished, 10)
        # Subscriber для получения текущего dialogue_id от dialogue_node
        # (чтобы включать его в TTS запросы и tts_node мог отбрасывать старые)
        self._current_dialogue_id: str | None = None
        self._dialogue_id_sub = node.create_subscription(
            String, "/voice/current_dialogue_id", self._on_current_dialogue_id, 1
        )
        # Трекер активных произношений: speech_id -> None; очищается в _on_tts_finished
        self.pending_speeches: dict = {}
        import threading
        self.pending_speeches_lock = threading.Lock()

    def _on_current_dialogue_id(self, msg: "String"):
        """Обновление текущего dialogue_id от dialogue_node."""
        self._current_dialogue_id = msg.data

    def _on_tts_finished(self, msg: "String"):
        """Обработка завершения произношения — очищает запись из pending_speeches."""
        import json
        try:
            result = json.loads(msg.data)
            speech_id = result.get("speech_id")
            self.log_info(f"🔔 TTS finished: speech_id={speech_id[:8] if speech_id else 'None'}..., success={result.get('success')}")
            if speech_id:
                with self.pending_speeches_lock:
                    if speech_id in self.pending_speeches:
                        del self.pending_speeches[speech_id]  # Очищаем, чтобы не росло в памяти
                        self.log_info(f"✅ Speech {speech_id[:8]}... удалён из pending_speeches")
                    else:
                        self.log_warning(f"⚠️ Speech {speech_id[:8]}... не найден в pending_speeches (возможно уже удалён)")
        except json.JSONDecodeError as e:
            self.log_error(f"❌ Ошибка парсинга TTS finished: {e}")

    @property
    def name(self) -> str:
        return "speak_text"

    @property
    def description(self) -> str:
        return (
            "Произнести текст голосом через TTS. "
            "ИСПОЛЬЗУЙ ЭТО вместо возврата JSON с SSML. "
            "ОБЯЗАТЕЛЬНО указывай animation - это покажет соответствующую анимацию на LED матрице робота (happy, sad, police_lights, и т.д.). "
            "Можешь вызвать несколько раз для разных фраз, делать паузы между ними через play_sound или play_animation."
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
                name="animation",
                type="string",
                description=(
                    "Анимация для отображения на LED матрице во время речи. "
                    "Выбирай подходящую анимацию для контекста (эмоциональные: happy, sad, angry, surprised; "
                    "специальные: police_lights, fire_truck, thinking, и т.д.). "
                    "Псевдонимы нормализуются: neutral→idle, excited→happy, confused→thinking, talk→talking. "
                    "Если указано неизвестное значение — будет warning в лог и анимация останется без изменений."
                ),
                required=False,
            ),
        ]

    # Максимальная длина чанка для Yandex TTS (лимит ~250 символов SSML)
    _MAX_CHUNK_CHARS = 200

    @staticmethod
    def _split_sentences(text: str, max_len: int = 200) -> list:
        """Разбивает текст на предложения не длиннее max_len символов.

        Сначала режет по знакам конца предложения (. ! ? ;), потом при
        необходимости разбивает слишком длинные куски по запятым/пробелам.
        """
        import re
        # Разбиваем по концу предложений, сохраняя разделители
        raw = re.split(r'(?<=[.!?;])\s+', text.strip())
        chunks: list = []
        buf = ""
        for part in raw:
            part = part.strip()
            if not part:
                continue
            candidate = (buf + " " + part).strip() if buf else part
            if len(candidate) <= max_len:
                buf = candidate
            else:
                if buf:
                    chunks.append(buf)
                # Если одно предложение само по себе длиннее лимита — бьём по запятым
                if len(part) > max_len:
                    sub_parts = re.split(r'(?<=,)\s+', part)
                    sub_buf = ""
                    for sp in sub_parts:
                        sub_candidate = (sub_buf + " " + sp).strip() if sub_buf else sp
                        if len(sub_candidate) <= max_len:
                            sub_buf = sub_candidate
                        else:
                            if sub_buf:
                                chunks.append(sub_buf)
                            # Последний резорт — режем по словам
                            words = sp.split()
                            word_buf = ""
                            for w in words:
                                wc = (word_buf + " " + w).strip() if word_buf else w
                                if len(wc) <= max_len:
                                    word_buf = wc
                                else:
                                    if word_buf:
                                        chunks.append(word_buf)
                                    word_buf = w
                            sub_buf = word_buf
                    buf = sub_buf
                else:
                    buf = part
        if buf:
            chunks.append(buf)
        return [c for c in chunks if c.strip()]

    def execute(self, text: str, animation: str = "neutral") -> MCPToolResult:
        """Произнести текст."""
        import json
        import uuid

        self.log_info(f"Произношение текста: {text[:50]}... (animation: {animation})")

        if not text:
            return MCPToolResult(success=False, error="Пустой текст", message="Текст не может быть пустым")

        # Нормализация анимаций (для обратной совместимости и маппинга несуществующих)
        animation_map = {
            # Русские названия
            "нейтрально": "idle",
            "нейтральная": "idle",
            "нейтральный": "idle",
            "радость": "happy",
            "радостный": "happy",
            "счастливый": "happy",
            "грустный": "sad",
            "грусть": "sad",
            "печаль": "sad",
            "злой": "angry",
            "злость": "angry",
            "возбужденный": "happy",
            "возбуждение": "happy",
            "смущенный": "thinking",
            "смущение": "thinking",
            "растерянный": "thinking",
            # Несуществующие анимации → замена на похожие
            "neutral": "idle",
            "excited": "happy",
            "confused": "thinking",
            "laughing": "happy",
            "smiling": "happy",
            "dancing": "excited",
            "singing": "happy",
            # LLM часто пишет "talk" вместо "talking"
            "talk": "talking",
        }
        # Множество реально существующих анимаций (без алиасов)
        _KNOWN_ANIMATIONS = {
            "idle", "talking", "wakeup", "sleep",
            "happy", "sad", "angry", "surprised", "thinking", "victory",
            "error", "low_battery", "charging",
            "police_lights", "ambulance", "fire_truck", "road_service",
            "turn_left", "turn_right", "accelerating", "braking",
        }
        animation = animation_map.get(animation.lower() if animation else "idle", animation) if animation else "idle"
        if animation not in _KNOWN_ANIMATIONS:
            self.log_warning(
                f"⚠️ Неизвестная анимация '{animation}' — "
                f"использую 'talking' (робот же говорит), текст будет произнесён"
            )
            animation = "talking"

        # Определяем pitch для голоса на основе анимации (только для эмоциональных)
        pitch_map = {
            "happy": "high",
            "sad": "low",
            "angry": "high",
            "excited": "x-high",
            "surprised": "high"
        }
        pitch = pitch_map.get(animation, None)

        def _make_ssml(chunk: str) -> str:
            if pitch:
                return f"<speak><prosody pitch='{pitch}'>{chunk}</prosody></speak>"
            return f"<speak>{chunk}</speak>"

        # Разбиваем текст на чанки ≤ _MAX_CHUNK_CHARS (лимит Yandex TTS)
        chunks = self._split_sentences(text, self._MAX_CHUNK_CHARS)
        if len(chunks) > 1:
            self.log_info(f"✂️ Текст разбит на {len(chunks)} чанков (длина: {len(text)} символов)")

        from std_msgs.msg import String
        speech_ids = []
        for i, chunk in enumerate(chunks):
            speech_id = str(uuid.uuid4())
            speech_ids.append(speech_id)

            # Анимацию ставим только для первого чанка (чтобы не мигало)
            if i == 0 and animation and animation != "idle":
                try:
                    from std_msgs.msg import String as StringMsg
                    anim_msg = StringMsg()
                    anim_msg.data = animation
                    self.animation_pub.publish(anim_msg)
                    self.log_info(f"🎨 Установлена анимация '{animation}'")
                except Exception as e:
                    self.log_warning(f"⚠️ Не удалось установить анимацию: {e}")

            # Регистрируем ожидание
            with self.pending_speeches_lock:
                self.pending_speeches[speech_id] = None

            tts_request = {"ssml": _make_ssml(chunk), "speech_id": speech_id}
            if self._current_dialogue_id:
                tts_request["dialogue_id"] = self._current_dialogue_id
            msg = String()
            msg.data = json.dumps(tts_request, ensure_ascii=False)
            self.tts_pub.publish(msg)
            self.log_info(
                f"📤 TTS чанк {i+1}/{len(chunks)}: {chunk[:40]!r}... "
                f"(speech_id: {speech_id[:8]}, dialogue_id: {self._current_dialogue_id[:8] if self._current_dialogue_id else 'None'})"
            )

        self.log_info(f"✅ TTS запрос(ов) принято: {len(chunks)} (асинхронный режим)")
        return MCPToolResult(
            success=True,
            data={"text": text, "animation": animation, "chunks": len(chunks), "speech_ids": speech_ids, "async": True},
            message=f"TTS запрос отправлен ({len(chunks)} чанк(ов)): {text[:50]}..."
        )

class ListenForResponseTool(MCPTool):
    """Инструмент для ожидания ответа пользователя."""

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String

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
        """Активировать режим ожидания ответа."""
        self.log_info(f"Ожидание ответа пользователя (таймаут: {timeout_seconds}s)")

        if prompt_text:
            self.log_info(f"Подсказка: {prompt_text}")

        # Публикуем запрос на активацию STT
        from std_msgs.msg import String
        msg = String()
        msg.data = f"listen:{timeout_seconds}"
        self.stt_request_pub.publish(msg)

        self.log_info("STT активирован для ожидания ответа")

        return MCPToolResult(
            success=True,
            data={"timeout_seconds": timeout_seconds, "prompt_text": prompt_text},
            message=f"Жду ответ пользователя ({timeout_seconds}s таймаут)",
        )
