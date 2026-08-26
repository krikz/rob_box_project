"""Per-session voice state для rob_box_quest.

Хранит:
- active_voice_id / active_preset — что выбрал клиент (per-client, не broadcast).
- preview rate limiter — sliding 10-sec window, max 3 preview_voice.

Чистая логика — без зависимостей от aiohttp / ROS. Тестируется в pytest
без event-loop'а.

Источник истины: задача t_7eba64d9 §"Решения (приняты родителем)":
> Rate limit: in-memory counter per session_id, reset каждые 10sec
> voice_state публикуется per-client (не broadcast)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional


# Rate limit: 3 preview per 10 seconds (per session).
PREVIEW_RATE_LIMIT = 3
PREVIEW_WINDOW_S = 10.0


@dataclass
class SessionVoiceState:
    """Состояние голоса одной WS-сессии.

    Создаётся на connect (в ClientSession или WSSServer), мутируется
    из event-loop aiohttp. Потокобезопасность НЕ требуется.
    """

    session_id: str
    # Defaults: первый голос каталога + standard preset.
    active_voice_id: Optional[str] = None  # None до первого set_voice
    active_preset: str = "standard"
    # Rate limiter: timestamps последних preview в монотонных секундах.
    _preview_timestamps: list[float] = field(default_factory=list)
    # Последняя ошибка (для voice_state payload).
    last_error: Optional[str] = None
    # listening flag — устанавливается из dialogue_node FSM (Phase 2 ext.).
    # Пока hardcoded False (сервер не слушает голос Quest — только меняет).
    listening: bool = False

    def apply_voice(self, voice_id: str, preset: str) -> None:
        """Применить выбор. Валидация голоса — на уровне каталога в handler'е."""
        self.active_voice_id = voice_id
        self.active_preset = preset
        self.last_error = None

    def set_error(self, message: str) -> None:
        """Записать last_error (например, "voice-pipeline offline")."""
        self.last_error = message

    def to_state_payload(self, ts_ms: int) -> dict:
        """Payload для voice_state event (msgpack → encode_voice_state)."""
        return {
            "active_voice_id": self.active_voice_id or "",
            "active_preset": self.active_preset,
            "listening": self.listening,
            "last_error": self.last_error,
            "ts_ms": ts_ms,
        }

    # --- Rate limiter -----------------------------------------------------

    def check_preview_quota(
        self, now_monotonic: Optional[float] = None
    ) -> bool:
        """True если можно отправить preview (под окном), False если rate-limited.

        Скользящее окно: timestamps старше PREVIEW_WINDOW_S удаляются.
        Если в окне меньше PREVIEW_RATE_LIMIT timestamps — пропускаем
        и записываем новый. Если равно или больше — блокируем.
        """
        now = (
            now_monotonic
            if now_monotonic is not None
            else time.monotonic()
        )
        cutoff = now - PREVIEW_WINDOW_S
        # Оставляем только timestamps внутри окна.
        self._preview_timestamps = [
            ts for ts in self._preview_timestamps if ts >= cutoff
        ]
        if len(self._preview_timestamps) >= PREVIEW_RATE_LIMIT:
            return False
        self._preview_timestamps.append(now)
        return True


class VoiceStateRegistry:
    """Реестр SessionVoiceState — session_id → state.

    Не thread-safe (только event-loop aiohttp). Один инстанс на WSSServer.
    """

    def __init__(self) -> None:
        self._by_session: Dict[str, SessionVoiceState] = {}

    def get_or_create(self, session_id: str) -> SessionVoiceState:
        st = self._by_session.get(session_id)
        if st is None:
            st = SessionVoiceState(session_id=session_id)
            self._by_session[session_id] = st
        return st

    def remove(self, session_id: str) -> None:
        self._by_session.pop(session_id, None)

    def __contains__(self, session_id: object) -> bool:
        return session_id in self._by_session

    def __len__(self) -> int:
        return len(self._by_session)


__all__ = [
    "PREVIEW_RATE_LIMIT",
    "PREVIEW_WINDOW_S",
    "SessionVoiceState",
    "VoiceStateRegistry",
]