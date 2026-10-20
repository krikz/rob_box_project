"""voice_request - запрос смены голоса TTS и фраза по событию (ADR-0150 §8, PR-4 ч.3, #3693).

Заменяет ``voice_state.VoiceStateStore``: голос хранит один владелец - ``tts_node`` (``VoiceOwner``), здесь только
просьба ``/voice/tts/voice_request`` и ожидание ``SpeechEvent voice_changed{requested, applied, provider}`` с тем же
``request_id`` в ``turn_id``. Фраза «Говорю голосом X» / «Голос X недоступен, остаюсь на Y» - из ``phrases/ru.yaml``
по исходу, а не слова LLM (ADR-0148). Нет события за ``WAIT_S`` - честное «не знаю, применился ли».
"""

from __future__ import annotations

import json
import threading
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

REQUEST_TOPIC = "/voice/tts/voice_request"
EVENT_TOPIC = "/voice/speech/event"
WAIT_S = 4.0

try:
    from rob_box_voice.tts_voice_registry import resolve_voice
except ImportError:  # pragma: no cover - минимальные окружения без rob_box_voice
    def resolve_voice(provider: str, requested: Optional[str]) -> Tuple[str, bool]:
        return (requested or ""), False


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
    """Публикатор запросов и ожидание ответа; один на ноду (``voice_requests(node)``)."""

    def __init__(self, node: Any) -> None:
        from rob_box_dialog_msgs.msg import SpeechEvent  # noqa: PLC0415
        from std_msgs.msg import String  # noqa: PLC0415

        self._String = String
        self._kind = SpeechEvent.KIND_VOICE_CHANGED
        self._pub = node.create_publisher(String, REQUEST_TOPIC, 10)
        self._lock = threading.Lock()
        self._waiters: Dict[str, Tuple[threading.Event, list]] = {}
        node.create_subscription(SpeechEvent, EVENT_TOPIC, self._on_event, 20)

    def _on_event(self, msg: Any) -> None:
        if msg.kind != self._kind:
            return
        with self._lock:
            waiter = self._waiters.get(msg.turn_id)
        if waiter is not None:
            waiter[1].append(msg)
            waiter[0].set()

    def request(self, voice: str, provider: str = "", wait_s: float = WAIT_S) -> VoiceOutcome:
        """Попросить голос ``voice`` (пусто - дефолт провайдера) и дождаться исхода от владельца."""
        rid = uuid.uuid4().hex
        done: Tuple[threading.Event, list] = (threading.Event(), [])
        with self._lock:
            self._waiters[rid] = done
        try:
            body = self._String()
            body.data = json.dumps({"request_id": rid, "voice": voice, "provider": provider}, ensure_ascii=False)
            self._pub.publish(body)
            if not done[0].wait(wait_s):
                return VoiceOutcome("timeout", voice, "", provider)
        finally:
            with self._lock:
                self._waiters.pop(rid, None)
        ev = done[1][0]
        ok = ev.voice_applied and not ev.reason
        return VoiceOutcome("applied" if ok else "fallback", ev.voice_requested, ev.voice_applied, ev.provider, ev.reason)


def voice_requests(node: Any) -> VoiceRequests:
    """Один ``VoiceRequests`` на ноду (подписка и публикатор создаются при первом запросе)."""
    if "_voice_requests" not in node.__dict__:
        node.__dict__["_voice_requests"] = VoiceRequests(node)
    return node.__dict__["_voice_requests"]


def outcome_phrase(outcome: VoiceOutcome) -> str:
    """Фраза исхода из ``phrases/ru.yaml`` (код, не LLM)."""
    from rob_box_dialog import phrases  # noqa: PLC0415

    if outcome.status == "timeout":
        return phrases.text("voice.timeout")
    if outcome.status == "applied":
        return phrases.text("voice.applied", voice=outcome.applied)
    if not outcome.applied:
        return phrases.text("voice.rejected", provider=outcome.provider or "?")
    return phrases.text("voice.unavailable", requested=outcome.requested or outcome.applied, applied=outcome.applied)
