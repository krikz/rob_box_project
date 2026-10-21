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

ADR-0150 PR-3 (#3688): реплика уходит единственным клиентом речи
``rob_box_core.speech_client.say`` (``SpeechRequest``, агент ``operator``);
``tts_node`` синтезирует независимо от состояния dialogue_node. Подтверждение
по ``SpeechEvent`` — PR-13 (ТАРС: say как action.speak_aloud).
"""

from typing import List

from rob_box_core import speech_client
from rob_box_core.utterance import Sink, Utterance

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class SayTool(MCPTool):
    """Произнести текст голосом оператора через TTS-канал (AV-21)."""

    event_source = "/voice/speech/event"

    def __init__(self, node) -> None:
        super().__init__(node)
        speech_client.prepare(node, "request")

    @property
    def name(self) -> str:
        return "say"

    @property
    def llm_visible(self) -> bool:
        # ТАРС-только (operator.speech): личность использует speak_text.
        return False

    @property
    def operator_visible(self) -> bool:
        # Оператор (ТАРС) её зовёт, личность — нет (issue #3305).
        return True

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

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: ``say`` — это голосовой канал
        # оператора (ТАРС), LLM-личность его НЕ зовёт. Срез фиксируется
        # тут, в slice_policy.yaml — список для ``operator.speech``
        # (один из четырёх operator-тулов, которые dialogue_node не может
        # звать, даже если бы LLM попросил).
        return "operator.speech"

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

        speech_id = speech_client.say(
            self.node, Utterance(text=text, sink=Sink.SPEAKERS), agent="operator"
        )
        return MCPToolResult(
            success=True,
            data={"text": text, "source": "operator", "speech_id": speech_id},
            message=f"say: опубликовано {len(text)} символов в TTS-канал",
        )


__all__ = ["SayTool"]
