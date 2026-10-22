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

import uuid
from typing import List, Optional

from rob_box_core import speech_client
from rob_box_core.utterance import Sink, Utterance

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..owner_event import OwnerEvents, owner_events

#: Сколько тул ждёт начала реплики. Дольше - реплика стоит в очереди владельца речи за чужой: итог придёт позже
#: событием по ``speech_id`` (``pending``), а не «не озвучено».
START_WAIT_S = 8.0


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


class SayAloudTool(MCPTool):
    """Озвучить текст из чата Telegram динамиками робота (ADR-0150 §5.1, §9.2.3; PR-12b #3707).

    ``SpeechRequest(agent=telegram, sink=speakers, interruptible=false)``: реплики разных агентов очередь ``tts_node``
    не перебивает, тишина Личности (решение О2) и её «хватит» чата не касаются. Успех - не публикация запроса, а
    событие владельца речи: началась - ``pending`` с ``event_ref=speech_id`` (итог «Озвучил (12 с)» придёт позже
    событием ``finished``), отказ - ``rejected{reason}``. Отказ ``operator_hold`` решает хост чата до вызова.
    """

    event_source = "/voice/speech/event"

    def __init__(self, node) -> None:
        super().__init__(node)
        speech_client.prepare(node, "request")

    @property
    def name(self) -> str:
        return "say_aloud"

    @property
    def llm_visible(self) -> bool:
        return False  # агент чата видит его через telegram_visible; Личность и оператор - нет

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Озвучить текст динамиками робота, когда человек в чате просит рассказать, прочитать или сказать "
            "вслух (например, сочинённую историю). Текст чата ты пишешь в ответе, а сюда передай ту же "
            "реплику целиком. Итог «Озвучил» в чат пишет код по событию динамиков, не ты."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="text", type="string", description="Что произнести, русским текстом без SSML.",
                             required=True),
            MCPToolParameter(name="voice", type="string", required=False,
                             description="Голос разовой реплики; пусто - текущий голос робота."),
        ]

    @property
    def slice(self) -> str:
        return "telegram"

    def execute(self, text: str, voice: Optional[str] = None) -> MCPToolResult:
        """Отдать реплику владельцу речи и дождаться начала (или отказа)."""
        if not text or not text.strip():
            return MCPToolResult(success=False, error="empty_text", message="Текст для озвучки пустой",
                                 data={"status": "rejected", "reason": "empty_text"})
        speech_id = str(uuid.uuid4())

        def publish(rid: str) -> None:
            utterance = Utterance(text=text, sink=Sink.SPEAKERS, voice=voice or None, speech_id=rid)
            speech_client.say(self.node, utterance, agent="telegram", interruptible=False)

        _rid, event = self._events().ask(publish, START_WAIT_S, request_id=speech_id)
        status, reason, seconds = speech_client.event_outcome(event) if event is not None else ("pending", None, 0)
        data = {"status": status, "reason": reason, "seconds": seconds, "speech_id": speech_id}
        if status == "pending":
            data["event_ref"] = speech_id
        self.log_info(f"say_aloud {speech_id[:8]}: {status} {reason or ''}".rstrip())
        ok = status in ("done", "pending")
        return MCPToolResult(success=ok, data=data, error=None if ok else f"Не озвучено: {reason or status}",
                             message=f"say_aloud: {status}")

    def _events(self) -> OwnerEvents:
        from rob_box_dialog_msgs.msg import SpeechEvent

        wanted = (speech_client.KIND_STARTED, *speech_client.KIND_STATUS)
        return owner_events(self.node, "say_aloud", lambda: OwnerEvents(
            self.node, SpeechEvent, speech_client.SPEECH_EVENT_TOPIC, key_of=lambda m: m.speech_id,
            accept=lambda m: m.agent == "telegram" and int(m.kind) in wanted))


__all__ = ["SayAloudTool", "SayTool"]
