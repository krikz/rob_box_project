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
``tts_node`` синтезирует независимо от состояния dialogue_node.

ADR-0150 PR-13 (#3708, §9.3, решение В7 «да, а»): ``say`` — действие класса ``action.speak_aloud``, как ``say_aloud``
чата. Успех — событие владельца речи, а не публикация: началась — ``pending`` с ``event_ref=speech_id`` (итог
«Озвучил (N с)» приходит позже событием ``finished``), отказ — ``rejected{reason}``.
"""

import uuid
from typing import List, Optional

from rob_box_core import speech_client
from rob_box_core.utterance import Sink, Utterance

from ..base import MCPTool, MCPToolParameter, MCPToolResult
from ..owner_event import OwnerEvents, owner_events

#: Сколько тул ждёт начала реплики. Дольше - реплика стоит в очереди владельца речи за чужой: итог придёт позже
#: событием по ``speech_id`` (``pending``), а не «не озвучено».
START_WAIT_S = 8.0


def speak_and_wait(tool: MCPTool, text: str, voice: Optional[str], agent: str, **request: object) -> MCPToolResult:
    """Отдать реплику агента ``agent`` владельцу речи и дождаться её начала (или отказа): исход — по ``SpeechEvent``.

    ``pending`` (звучит) — успех с ``event_ref``: итог хост агента получит позже событием с тем же ``speech_id``."""
    if not text or not text.strip():
        return MCPToolResult(success=False, error="empty_text", message="Текст для озвучки пустой",
                             data={"status": "rejected", "reason": "empty_text"})
    speech_id = str(uuid.uuid4())

    def publish(rid: str) -> None:
        utterance = Utterance(text=text, sink=Sink.SPEAKERS, voice=voice or None, speech_id=rid)
        speech_client.say(tool.node, utterance, agent=agent, **request)

    _rid, event = _events(tool, agent).ask(publish, START_WAIT_S, request_id=speech_id)
    status, reason, seconds = speech_client.event_outcome(event) if event is not None else ("pending", None, 0)
    data = {"status": status, "reason": reason, "seconds": seconds, "speech_id": speech_id}
    if status == "pending":
        data["event_ref"] = speech_id
    tool.log_info(f"{tool.name} {speech_id[:8]}: {status} {reason or ''}".rstrip())
    ok = status in ("done", "pending")
    return MCPToolResult(success=ok, data=data, error=None if ok else f"Не озвучено: {reason or status}",
                         message=f"{tool.name}: {status}")


def _events(tool: MCPTool, agent: str) -> OwnerEvents:
    from rob_box_dialog_msgs.msg import SpeechEvent

    wanted = (speech_client.KIND_STARTED, *speech_client.KIND_STATUS)
    return owner_events(tool.node, tool.name, lambda: OwnerEvents(
        tool.node, SpeechEvent, speech_client.SPEECH_EVENT_TOPIC, key_of=lambda m: m.speech_id,
        accept=lambda m: m.agent == agent and int(m.kind) in wanted))


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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: ``say`` — голосовой канал оператора (ТАРС), LLM-личность его НЕ зовёт.
        return "operator.speech"

    def execute(self, text: str) -> MCPToolResult:
        """Реплика оператора динамиками; исход — событие владельца речи (``action.speak_aloud``, PR-13)."""
        return speak_and_wait(self, text, None, "operator")


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
        return speak_and_wait(self, text, voice, "telegram", interruptible=False)


__all__ = ["SayAloudTool", "SayTool", "speak_and_wait"]
