"""Сессия агента с одним владельцем (ADR-0150 §7.1–7.2, §7.5 п.1, решение В11).

Пишет только владелец — через owner-методы :class:`Session`; все остальные (LLM, ноды, тесты) видят
неизменяемый :class:`SessionSnapshot`. Сцены — стек кадров с явным ``pop`` и TTL. Каждое изменение —
событие :class:`SessionChanged` подписчикам. Рестарт процесса = новая сессия: новый ``session_id``, эпоха 0.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Callable, List, Mapping, Optional, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Ask

#: Состояния имени собеседника (ADR-0130 §2.6): по имени обращаться можно только при ``confirmed``.
NAME_STATES: Tuple[str, ...] = ("unknown", "tentative", "confirmed")


@dataclass(frozen=True)
class SceneFrame:
    """Кадр стека сцен: ``Silence``, ``Persona``, ``DJ``, ``OperatorHold``, ``Identity``."""

    kind: str
    set_by: str
    ttl_until: Optional[float] = None
    payload: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))


@dataclass(frozen=True)
class VoiceState:
    """Голос: что просили, что применил владелец речи, у какого провайдера (одно место вместо трёх)."""

    requested: Optional[str] = None
    applied: Optional[str] = None
    provider: Optional[str] = None


@dataclass(frozen=True)
class SpeakerRef:
    """Собеседник — контракт шва с восприятием (§7.5 п.1); источник — адаптер, не диалог."""

    person_ref: Optional[str] = None
    track_ref: Optional[str] = None
    name: Optional[str] = None
    name_state: str = "unknown"
    azimuth_deg: Optional[float] = None
    since: float = 0.0
    channels: Tuple[str, ...] = ("voice",)


@dataclass(frozen=True)
class MusicStateRef:
    """Копия latched-снимка владельца плеера (``/voice/music/state``)."""

    state: str = "idle"
    track_id: Optional[str] = None
    bpm: Optional[float] = None
    title: Optional[str] = None


@dataclass(frozen=True)
class ProviderHealth:
    """Здоровье провайдера одного вида (llm/stt/tts) из ``ProviderState`` (§8)."""

    health: str = "ok"
    since: float = 0.0
    last_error_code: Optional[str] = None


@dataclass(frozen=True)
class SessionSnapshot:
    """Неизменяемый снимок сессии — то, что видят LLM (структурным блоком) и все, кроме владельца."""

    agent: str
    session_id: str
    epoch: int
    started_at: float
    scenes: Tuple[SceneFrame, ...] = ()
    voice: VoiceState = VoiceState()
    speaker: Optional[SpeakerRef] = None
    pending_ask: Optional[Ask] = None
    music: MusicStateRef = MusicStateRef()
    providers: Mapping[str, ProviderHealth] = field(default_factory=lambda: MappingProxyType({}))
    #: Звучит ли речь агента сейчас (``SpeechEvent started`` без ``finished/cancelled``) — канал громкости (§4.2).
    speaking: bool = False
    #: Последний канал громкости, который трогали (``knowledge.VOLUME_CHANNELS``): «громче» в тишине — он.
    volume_channel: str = "tts"

    def top(self, kind: str) -> Optional[SceneFrame]:
        """Верхний кадр этого вида или ``None``."""
        return next((f for f in reversed(self.scenes) if f.kind == kind), None)


@dataclass(frozen=True)
class SessionChanged:
    """Событие владельца: что поменялось (имя owner-метода) и снимок после изменения."""

    change: str
    snapshot: SessionSnapshot


Listener = Callable[[SessionChanged], None]

#: TTL кадров по умолчанию: пользовательская тишина — 10 мин, вопрос «X, это ты?» — 15 с.
_DEFAULT_TTL_S: Mapping[Tuple[str, str], float] = MappingProxyType({
    ("Silence", "user"): float(kn.SILENCE_TTL_S),
    ("Identity", "code"): float(kn.IDENTITY_TTL_S),
})


class Session:
    """Владелец сессии одного агента. Писать в неё может только код через эти методы."""

    def __init__(self, agent: str, clock: Callable[[], float] = time.monotonic) -> None:
        if agent not in kn.AGENTS:
            raise ValueError(f"Session: agent must be one of {kn.AGENTS}, got {agent!r}")
        self._clock = clock
        self._listeners: List[Listener] = []
        self._snap = SessionSnapshot(agent=agent, session_id=uuid.uuid4().hex, epoch=0, started_at=clock())

    @property
    def snapshot(self) -> SessionSnapshot:
        return self._snap

    def subscribe(self, listener: Listener) -> None:
        self._listeners.append(listener)

    def push_scene(self, kind: str, set_by: str, ttl_s: Optional[float] = None,
                   payload: Optional[Mapping[str, Any]] = None) -> SceneFrame:
        """Положить кадр; TTL по умолчанию — из :mod:`.knowledge` (тишина пользователя, вопрос личности)."""
        if kind not in kn.SCENE_KINDS or set_by not in kn.SCENE_SETTERS:
            raise ValueError(f"push_scene: unknown kind/set_by {kind!r}/{set_by!r}")
        ttl = _DEFAULT_TTL_S.get((kind, set_by)) if ttl_s is None else ttl_s
        frame = SceneFrame(kind, set_by, None if ttl is None else self._clock() + ttl,
                           MappingProxyType(dict(payload or {})))
        self._commit("push_scene", scenes=self._snap.scenes + (frame,))
        return frame

    def pop_scene(self, kind: str, set_by: Optional[str] = None) -> Optional[SceneFrame]:
        """Снять верхний кадр вида ``kind``; ``set_by`` — только кадр этого автора (оператор не снимает тишину
        пользователя, и наоборот). Нечего снимать — ``None``, событие не шлётся."""
        scenes = list(self._snap.scenes)
        for i in range(len(scenes) - 1, -1, -1):
            if scenes[i].kind == kind and set_by in (None, scenes[i].set_by):
                frame = scenes.pop(i)
                self._commit("pop_scene", scenes=tuple(scenes))
                return frame
        return None

    def expire(self) -> Tuple[SceneFrame, ...]:
        """Снять кадры с истёкшим TTL (и вопрос, если истёк его кадр ``Identity``)."""
        now = self._clock()
        gone = tuple(f for f in self._snap.scenes if f.ttl_until is not None and f.ttl_until <= now)
        if gone:
            asked = {"pending_ask": None} if any(f.kind == "Identity" for f in gone) else {}
            self._commit("expire", scenes=tuple(f for f in self._snap.scenes if f not in gone), **asked)
        return gone

    def reset(self) -> None:
        """Новая сессия разговора: эпоха + 1, стек и вопрос очищены; голос, собеседник, музыка и провайдеры —
        не трогаются («забудь всё» ≠ «выключи музыку»)."""
        self._commit("reset", epoch=self._snap.epoch + 1, scenes=(), pending_ask=None)

    def set_voice(self, requested: Optional[str] = None, applied: Optional[str] = None,
                  provider: Optional[str] = None) -> None:
        """Обновить голос: переданные поля заменяются, ``None`` — оставить как было."""
        old = self._snap.voice
        voice = VoiceState(requested if requested is not None else old.requested,
                           applied if applied is not None else old.applied,
                           provider if provider is not None else old.provider)
        self._commit("set_voice", voice=voice)

    def set_speaker(self, speaker: Optional[SpeakerRef]) -> None:
        if speaker is not None and speaker.name_state not in NAME_STATES:
            raise ValueError(f"set_speaker: name_state must be one of {NAME_STATES}")
        self._commit("set_speaker", speaker=speaker)

    def set_pending_ask(self, ask: Optional[Ask]) -> None:
        self._commit("set_pending_ask", pending_ask=ask)

    def _commit(self, change: str, **fields: Any) -> None:
        self._snap = replace(self._snap, **fields)
        event = SessionChanged(change, self._snap)
        for listener in list(self._listeners):
            listener(event)
