"""Voice floor — серверный mutex на «голосовой поток» quest-WS.

Контекст (docs/plans/2026-08-27-quest-voice-passthrough-design.md §5):

  Голосовой поток квеста (voice_ptt_start/stop, VOICE_AUDIO фреймы →
  /avatar/voice_in) сейчас один на всех WS-клиентов. Когда AV-23
  приземлится, два клиента смогут одновременно слать голос — звук
  начнёт рвать/микшироваться. Этот модуль — серверный gate:

    - ровно один держатель (state ∈ {"idle","listening","speaking"});
    - holder_id = "<session_id>|<client_id>" (client_id берётся из HELLO
      payload или генерируется сервером);
    - try_acquire: если свободен → ok; иначе → denied + busy_holder;
    - release: только держатель может освободить (идемпотентно);
    - force_release_for: при отвале сессии без явного stop.

Состояния `listening` / `speaking` соответствуют wire-формату
voice_state в docs/architecture/meta-quest-api.md §4 (`state: "idle"|"listening"|...`).
Доп. `denied` — это локальный для quest-сервера код, отправляется в том же
JSON_EVENT{type:"voice_state"} фрейме чтобы UI мог показать «у робота
говорит другой» без нового типа.

Чистая логика, без зависимостей от aiohttp/ROS/Zenoh. Потокобезопасности
НЕ требуется — мутации только из event-loop aiohttp (как ClientSession).
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class FloorState(str, Enum):
    """Состояния voice floor.

    Значения совпадают со схемой ``voice_state`` в meta-quest-api.md §4,
    кроме ``DENIED`` (локальное расширение для UI)."""

    IDLE = "idle"
    LISTENING = "listening"
    SPEAKING = "speaking"
    DENIED = "denied"


@dataclass(frozen=True)
class FloorHolder:
    """Идентификатор держателя floor."""

    session_id: str
    client_id: str  # из HELLO.payload.client_id, либо серверный fallback

    def label(self) -> str:
        """Короткая метка для логов и voice_state.detail (≤ ~32 символа)."""
        # Берём первые 8 символов session_id — визуально достаточно и
        # уникально в пределах одной WS-сессии.
        sid = self.session_id[:8] if len(self.session_id) >= 8 else self.session_id
        return f"{self.client_id}:{sid}"


class VoiceFloor:
    """Серверный mutex на голосовой поток. См. module docstring."""

    def __init__(self) -> None:
        self._state: FloorState = FloorState.IDLE
        self._holder: Optional[FloorHolder] = None
        self._acquired_monotonic: Optional[float] = None

    @property
    def state(self) -> FloorState:
        return self._state

    @property
    def holder(self) -> Optional[FloorHolder]:
        """Текущий держатель (для подписки на voice_state: начальный snapshot)."""
        return self._holder

    def try_acquire(
        self, session_id: str, client_id: Optional[str] = None
    ) -> Tuple[bool, Optional[FloorHolder], FloorState]:
        """Захватить floor от имени (session_id, client_id).

        Returns:
            (acquired, busy_holder, new_state)
            - acquired=True  → захватили; busy_holder=None; new_state=LISTENING
              (или SPEAKING, если передан mode="speaking" — параметр mode
               сейчас не принимается, оставлен на Phase 2 с левым grip).
            - acquired=False → отказ; busy_holder=текущий держатель;
              new_state=DENIED.

        Args:
            session_id: id WS-сессии (ClientSession.session_id).
            client_id: id клиента (опционально, из HELLO payload).
        """
        if self._state == FloorState.IDLE or self._holder is None:
            cid = client_id or f"anon-{uuid.uuid4().hex[:6]}"
            self._holder = FloorHolder(session_id=session_id, client_id=cid)
            self._state = FloorState.LISTENING
            self._acquired_monotonic = time.monotonic()
            return True, None, FloorState.LISTENING
        return False, self._holder, FloorState.DENIED

    def release(self, session_id: str) -> Tuple[bool, FloorState]:
        """Освободить floor от имени session_id.

        Только текущий держатель может освободить — идемпотентно.
        Returns:
            (released, new_state)
            - released=True  → floor освобождён, new_state=IDLE
            - released=False → не держатель; no-op, new_state=текущее
        """
        if self._holder is None:
            return True, FloorState.IDLE
        if self._holder.session_id != session_id:
            return False, self._state
        self._holder = None
        self._state = FloorState.IDLE
        self._acquired_monotonic = None
        return True, FloorState.IDLE

    def force_release_for(self, session_id: str) -> bool:
        """Принудительно снять holder, если это данная session_id.

        Вызывается при отвале сессии (GOODBYE / watchdog / disconnect).
        Идемпотентно: если session_id не держит — False, иначе True.
        """
        released, _ = self.release(session_id)
        return released

    def snapshot(self) -> dict:
        """Сериализация текущего состояния для логов / телеметрии."""
        return {
            "state": self._state.value,
            "holder": (
                {
                    "session_id": self._holder.session_id,
                    "client_id": self._holder.client_id,
                    "held_for_s": (
                        time.monotonic() - self._acquired_monotonic
                        if self._acquired_monotonic is not None
                        else 0.0
                    ),
                }
                if self._holder is not None
                else None
            ),
        }


__all__ = ["FloorState", "FloorHolder", "VoiceFloor"]
