"""voice_request - запрос смены голоса TTS и данные его исхода (ADR-0150 §8, PR-4 ч.3, #3693).

Заменяет ``voice_state.VoiceStateStore``: голос хранит один владелец - ``tts_node`` (``VoiceOwner``), здесь только
просьба ``/voice/tts/voice_request`` и ожидание ``SpeechEvent voice_changed{requested, applied, provider}`` с тем же
``request_id`` в ``turn_id``. Нет события за ``WAIT_S`` - честное «не знаю, применился ли». Тул фразу не говорит: он
отдаёт исход данными (:func:`outcome_data`), а фразу «Говорю голосом X» / «Голос X недоступен, остаюсь на Y» строит
диалог по результату (``rob_box_dialog.knowledge.VOICE_DONE``, ADR-0148): одна фраза на ход, а не тул и код рядом.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from .owner_event import OwnerEvents

REQUEST_TOPIC = "/voice/tts/voice_request"
EVENT_TOPIC = "/voice/speech/event"
WAIT_S = 4.0

try:
    from rob_box_voice.tts_voice_registry import (PROVIDER_LABELS, resolve_voice, robot_default_voice,
                                                  voice_display_name)
except ImportError:  # pragma: no cover - минимальные окружения без rob_box_voice
    PROVIDER_LABELS: Dict[str, str] = {}

    def resolve_voice(provider: str, requested: Optional[str]) -> Tuple[str, bool]:
        return (requested or ""), False

    def robot_default_voice() -> Tuple[str, str]:
        return "", ""

    def voice_display_name(provider: str, voice_id: str) -> str:
        return voice_id


@dataclass(frozen=True)
class VoiceOutcome:
    """Исход запроса: ``applied`` - голос применён как просили, ``fallback`` - применён другой, ``timeout`` - тишина."""

    status: str
    requested: str
    applied: str
    provider: str
    reason: str = ""


def resolve_speak_voice(provider: str, requested: Optional[str]) -> Tuple[str, bool]:
    """Голос разовой реплики ``speak_text(voice=...)``: известный - как есть, неизвестный - дефолт провайдера.

    Без ``voice`` - пусто: реплика звучит текущим голосом ``tts_node`` (его выбор и хранит владелец голоса).
    """
    if not requested:
        return "", False
    return resolve_voice(provider, requested)


class VoiceRequests:
    """Публикатор запросов голоса; ожидание ``voice_changed`` - общий механизм ``owner_event`` (один на ноду)."""

    def __init__(self, node: Any) -> None:
        from rob_box_dialog_msgs.msg import SpeechEvent  # noqa: PLC0415
        from std_msgs.msg import String  # noqa: PLC0415

        self._String = String
        self._pub = node.create_publisher(String, REQUEST_TOPIC, 10)
        kind = SpeechEvent.KIND_VOICE_CHANGED
        self._events = OwnerEvents(node, SpeechEvent, EVENT_TOPIC, key_of=lambda m: m.turn_id,
                                   accept=lambda m: m.kind == kind)

    def request(self, voice: str, provider: str = "", wait_s: float = WAIT_S) -> VoiceOutcome:
        """Попросить голос ``voice`` (пусто - дефолт провайдера) и дождаться исхода от владельца."""
        def publish(rid: str) -> None:
            body = self._String()
            body.data = json.dumps({"request_id": rid, "voice": voice, "provider": provider}, ensure_ascii=False)
            self._pub.publish(body)

        _rid, ev = self._events.ask(publish, wait_s)
        if ev is None:
            return VoiceOutcome("timeout", voice, "", provider)
        ok = ev.voice_applied and not ev.reason
        return VoiceOutcome("applied" if ok else "fallback", ev.voice_requested, ev.voice_applied, ev.provider, ev.reason)


def voice_requests(node: Any) -> VoiceRequests:
    """Один ``VoiceRequests`` на ноду (подписка и публикатор создаются при первом запросе)."""
    if "_voice_requests" not in node.__dict__:
        node.__dict__["_voice_requests"] = VoiceRequests(node)
    return node.__dict__["_voice_requests"]


def outcome_data(outcome: VoiceOutcome) -> Dict[str, Any]:
    """Данные исхода для фразы диалога (``knowledge.VOICE_DONE`` / ``VOICE_FAILED``): исход владельца и слоты фраз.

    ``status`` - ``applied`` / ``fallback`` / ``timeout`` (``already_default`` - голос и так обычный, владельцу ничего
    не просили); ``voice_name`` / ``requested_name`` / ``provider_label`` - названия для озвучки из каталога голосов
    (``tts_voice_registry.voice_display_name``: «Весельчак», а не ``English_Jovialman``; чужое имя - как сказали);
    ``voice_default`` - применённый голос и есть обычный голос робота (нет применённого - ключа нет); у неудачи -
    ``reason`` (диалог берёт её из данных результата, а не из текста ошибки)."""
    data: Dict[str, Any] = {
        "status": outcome.status, "voice_requested": outcome.requested, "voice_applied": outcome.applied,
        "provider": outcome.provider, "voice_name": voice_display_name(outcome.provider, outcome.applied),
        "requested_name": voice_display_name(outcome.provider, outcome.requested),
        "provider_label": PROVIDER_LABELS.get(outcome.provider, outcome.provider),
    }
    if outcome.applied:
        data["voice_default"] = (outcome.provider, outcome.applied) == robot_default_voice()
    if outcome.status in ("fallback", "timeout"):
        data["reason"] = outcome.reason or outcome.status
    return data
