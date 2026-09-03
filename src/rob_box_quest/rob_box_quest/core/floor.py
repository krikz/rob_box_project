"""SupervisorFloorTracker — локальный учёт teleop-floor-ов для ws_server.

Источник истины: docs/adr/0028 §4.2 (floor-ы), §4.4 (heartbeat), §6 Q4 (dead-man 500мс).

Контекст (AV-19, issue #1911):
- В Phase 1 ``avatar_supervisor`` задеплоен в ``monitor``-режиме и НЕ
  рулит twist_mux/dialogue_node параметрами (ADR-0028 §4.5).
- Кастомный IDL для ``AcquireFloor``/``ReleaseFloor`` (AV-5, AV-12)
  ещё не готов, поэтому полноценный service-call на
  ``/avatar_supervisor/acquire_floor`` через ROS 2 из ws_server
  невозможен без ломки существующих unit-тестов (которые бегут без
  rclpy).
- Поэтому ws_server ведёт **локальный** floor-tracker: кто из
  client_id сейчас держит ``teleop``. Это согласуется с мотивом
  ADR-0028 §1.1 — «пока один источник голоса — floor всегда свободен»:
  в Phase 1 (только Quest) tracker-а достаточно, чтобы гейтить
  ``teleop_twist`` без разрешения супервизора.
- Когда supervisor-сервисы станут доступны (AV-12), tracker
  превращается в тонкий proxy: тот же публичный API, но ``acquire``
  дёргает ``/avatar_supervisor/acquire_floor`` через
  ``Bridge.acquire_teleop_floor``. В PR это явно отмечено в
  ``meta-quest-api.md`` и design.md.

Контракт:
- Floor-ы — исключительно ``teleop`` (AV-19 фокус). ``voice`` живёт в
  supervisor-клиенте (VoiceFloor), см. rob_box_telegram/.../supervisor_client.py.
- ``acquire(client_id)`` — попытка взять floor. Возвращает
  ``AcquireResult(granted, held_by, reason)``.
- ``release(client_id)`` — отпустить; идемпотентно, ошибки нет если
  клиент не держал.
- ``holder()`` — текущий держатель (client_id) или ``None``.
- ``is_held_by(client_id)`` — True если client_id == holder().
- ``FLOOR_HELD_RATE_LIMIT_S`` — минимальный интервал между двумя
  ошибками ``FLOOR_HELD`` для одной сессии (anti-spam: Quest шлёт
  ``teleop_twist`` 30 Гц; если floor держит Telegram, ws_server не
  должен заливать сокет ошибками).

Используется в ws_server._on_json_cmd (gate teleop_twist +
rate-limited FLOOR_HELD) и WSSServer._unregister_session (release
на закрытии).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


# Anti-spam для ERROR{FLOOR_HELD}: на 30 Гц teleop_twist при свободном
# holder-сессии не должны получать по 30 ошибок в секунду. Раз в 1 с — норма.
FLOOR_HELD_RATE_LIMIT_S: float = 1.0


@dataclass
class AcquireResult:
    """Ответ на ``acquire(client_id)``."""

    granted: bool
    held_by: Optional[str] = None
    reason: str = ""


class SupervisorFloorTracker:
    """Локальный учёт teleop-floor-ов для ws_server (AV-19).

    Single-thread: все вызовы из aiohttp event-loop. Никаких блокировок.
    """

    __slots__ = (
        "_holder",
        "_last_error_monotonic",
        "_now_fn",
    )

    def __init__(self, now_fn=None) -> None:
        self._holder: Optional[str] = None
        self._last_error_monotonic: dict[str, float] = {}
        # Injectable clock для тестов с fake-clock.
        self._now_fn = now_fn if now_fn is not None else time.monotonic

    @property
    def holder(self) -> Optional[str]:
        """client_id текущего держателя teleop_floor или None."""
        return self._holder

    def is_held_by(self, client_id: Optional[str]) -> bool:
        return client_id is not None and self._holder == client_id

    def acquire(self, client_id: str) -> AcquireResult:
        """Попытка взять teleop_floor от имени ``client_id``.

        Идемпотентно: если уже держит тот же client_id — granted=True.
        Иначе: granted=False, ``held_by`` — текущий держатель.
        """
        if not client_id:
            return AcquireResult(granted=False, held_by=self._holder, reason="invalid_client_id")
        if self._holder == client_id:
            return AcquireResult(granted=True, held_by=client_id, reason="already_held")
        if self._holder is None:
            self._holder = client_id
            return AcquireResult(granted=True, held_by=client_id, reason="granted")
        return AcquireResult(granted=False, held_by=self._holder, reason="held_by_other")

    def release(self, client_id: str) -> bool:
        """Отпустить floor. True если клиент реально держал его.

        Идемпотентно: если client_id не держит — False, но без raise.
        Используется в _unregister_session — клиент мог и не взять floor.
        """
        if self._holder == client_id:
            self._holder = None
            return True
        return False

    def force_release(self) -> Optional[str]:
        """Снять floor (используется при shutdown/reset). Возвращает бывшего holder."""
        prev = self._holder
        self._holder = None
        return prev

    def should_send_floor_held_error(self, client_id: str) -> bool:
        """Rate-limit для ERROR{FLOOR_HELD} на одного клиента.

        True если можно слать ошибку (прошло >= FLOOR_HELD_RATE_LIMIT_S
        с последней ошибки для этого client_id). Сбрасывает таймер после
        True (вызывающий код СРАЗУ шлёт ошибку и больше не повторяет до
        истечения окна).
        """
        now = self._now_fn()
        last = self._last_error_monotonic.get(client_id, 0.0)
        if (now - last) >= FLOOR_HELD_RATE_LIMIT_S:
            self._last_error_monotonic[client_id] = now
            return True
        return False

    def reset_rate_limit(self, client_id: Optional[str]) -> None:
        """Сбросить rate-limit окно для client_id (вызывается на release)."""
        if client_id is not None:
            self._last_error_monotonic.pop(client_id, None)


__all__ = [
    "SupervisorFloorTracker",
    "AcquireResult",
    "FLOOR_HELD_RATE_LIMIT_S",
]
