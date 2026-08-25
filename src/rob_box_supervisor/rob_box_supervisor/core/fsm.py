"""ModeManager — FSM режимов аватара (AV-3, ADR-0028 §4.1).

Это **чистая Python-логика** без ROS, asyncio и threading, чтобы её было
легко покрыть TDD (см. ``test/unit/core/test_fsm.py``). FSM владеет
своим представлением о том, кто держит ``voice_floor`` /
``teleop_floor`` — этого достаточно, чтобы принимать решения о
переходах. Это **не** замена AV-4 ``LockManager``: LockManager
отвечает за heartbeat/dead-man и ownership для ``twist_mux`` /
TTS-канала, а FSM — про режимы и переходы между ними.

Таблица переходов — точная копия ADR-0028 §4.1 (mermaid-stateDiagram).
Конфликты (``ConflictError``) возникают, когда целевой переход требует
``floor``, который держит другой клиент. ``* → off`` всегда разрешён
(escape hatch): никакое состояние не может заблокировать выключение.

Часы — опциональный инжектируемый ``clock`` (callable → int ms).
Используется для ``IDLE_TIMEOUT_S`` (см. ADR-0028 §4.1: «timeout 30 s
no activity»). Если ``clock`` не передан, берётся ``time.monotonic() *
1000``.
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Callable, Optional


# === Константы / доменные типы =======================================

# ADR-0028 §4.1: «telegram_release (timeout 30s no activity)» /
# «quest_release (timeout 30s no activity)». По истечении — переход в ``off``.
IDLE_TIMEOUT_S: float = 30.0
IDLE_TIMEOUT_MS: int = int(IDLE_TIMEOUT_S * 1000)

FLOOR_VOICE: str = "voice_floor"
FLOOR_TELEOP: str = "teleop_floor"


class Mode(str, Enum):
    """Режимы аватара из ADR-0028 §4.1.

    Значения — строки, чтобы удобно сериализовать в ``/avatar/state``
    (msgpack) без отдельного IDL.
    """

    OFF = "off"
    TELEGRAM_ACTIVE = "telegram_active"
    AVATAR_PRESENT = "avatar_present"
    MIXED = "mixed"

    @classmethod
    def values(cls) -> tuple:
        """Все валидные режимы."""
        return (cls.OFF, cls.TELEGRAM_ACTIVE, cls.AVATAR_PRESENT, cls.MIXED)


class Floor(str, Enum):
    """Floor-ы, которыми FSM интересуется для принятия решений."""

    VOICE = FLOOR_VOICE
    TELEOP = FLOOR_TELEOP

    @classmethod
    def values(cls) -> tuple:
        return (cls.VOICE, cls.TELEOP)


# Имя клиента — это ``str`` (telegram/quest/...). Типизируем для
# читаемости сигнатур.
ClientId = str


# === Исключения ======================================================


class ConflictError(Exception):
    """Запрошенный переход невозможен: целевой ``floor`` занят.

    Атрибуты:
      current_mode — в каком режиме находились;
      target_mode — куда пытались перейти;
      floor — какой ``floor`` заблокировал переход;
      held_by — кто держит этот ``floor``;
      requested_by — кто пытался перейти.
    """

    def __init__(
        self,
        current_mode: Mode,
        target_mode: Mode,
        floor: str,
        held_by: ClientId,
        requested_by: ClientId,
    ):
        self.current_mode = current_mode
        self.target_mode = target_mode
        self.floor = floor
        self.held_by = held_by
        self.requested_by = requested_by
        super().__init__(
            f"cannot transition {current_mode.value} → {target_mode.value}: "
            f"floor {floor!r} is held by {held_by!r}; "
            f"requested by {requested_by!r}"
        )


# === Имена событий ===================================================

EVENT_TELEGRAM_ACQUIRE_FLOOR: str = "telegram_acquire_floor"
EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR: str = "telegram_acquire_voice_floor"
EVENT_TELEGRAM_RELEASE: str = "telegram_release"
EVENT_QUEST_ACQUIRE_FLOOR: str = "quest_acquire_floor"
EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY: str = "quest_acquire_floor_teleop_only"
EVENT_QUEST_RELEASE: str = "quest_release"
EVENT_BOTH_RELEASE: str = "both_release"

# Escape hatch: «* → off» (см. ADR-0028 §4.1 + §6 Q1 fail-safe).
EVENT_FORCE_OFF: str = "force_off"


_ALL_EVENTS: tuple = (
    EVENT_TELEGRAM_ACQUIRE_FLOOR,
    EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
    EVENT_TELEGRAM_RELEASE,
    EVENT_QUEST_ACQUIRE_FLOOR,
    EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
    EVENT_QUEST_RELEASE,
    EVENT_BOTH_RELEASE,
    EVENT_FORCE_OFF,
)


# === ModeManager =====================================================


class ModeManager:
    """FSM режимов аватара.

    Контракт:
      - ``mode`` — текущий режим (изначально ``OFF``).
      - ``transition(event, client_id=None) -> Mode`` — переход по событию.
        Возвращает новый ``Mode``. Бросает:
          - ``ConflictError`` если переход требует ``floor``, который
            держит другой клиент;
          - ``ValueError`` если ``event`` не известен.
        Повторный acquire от того же клиента — **no-op** (идемпотентность).
      - ``* → off`` через ``EVENT_FORCE_OFF`` — всегда разрешён.
      - ``IDLE_TIMEOUT_S`` простоя (без ``acquire``/``release``) в
        ``telegram_active`` или ``avatar_present`` → следующий
        ``*_release`` переводит FSM в ``off``.
      - FSM владеет ``voice_held_by`` / ``teleop_held_by`` для принятия
        решений; LockManager (AV-4) — отдельный компонент.
    """

    def __init__(self, clock: Optional[Callable[[], int]] = None):
        self._clock = clock or self._default_clock
        self._mode: Mode = Mode.OFF
        # Кто держит какой floor (для решений FSM). None = свободен.
        self._voice_held_by: Optional[ClientId] = None
        self._teleop_held_by: Optional[ClientId] = None
        # ``last_activity_ms`` — последний раз, когда FSM менял mode или
        # holder-ов (для IDLE_TIMEOUT_S).
        self._last_activity_ms: int = self._clock()

    # ---------- публичный API ----------

    @property
    def mode(self) -> Mode:
        """Текущий режим."""
        return self._mode

    def voice_held_by(self) -> Optional[ClientId]:
        """Кто держит ``voice_floor`` (для отладки/тестов)."""
        return self._voice_held_by

    def teleop_held_by(self) -> Optional[ClientId]:
        """Кто держит ``teleop_floor`` (для отладки/тестов)."""
        return self._teleop_held_by

    def transition(self, event: str, client_id: Optional[ClientId] = None) -> Mode:
        """Применить событие и вернуть новый ``Mode``.

        Параметры:
          event — имя события (``EVENT_*`` или ``force_off``);
          client_id — идентификатор клиента-инициатора. Обязателен для
            событий ``acquire``/``release``, опционален для
            ``both_release`` / ``force_off``.
        """
        if event not in _ALL_EVENTS:
            raise ValueError(f"unknown event {event!r}")

        self._check_idle_timeout()

        if event == EVENT_FORCE_OFF:
            return self._force_off()

        if event == EVENT_BOTH_RELEASE:
            return self._both_release()

        # Acquire-события: требуют client_id и проверяют идемпотентность.
        if event in (
            EVENT_TELEGRAM_ACQUIRE_FLOOR,
            EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
            EVENT_QUEST_ACQUIRE_FLOOR,
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
        ):
            return self._handle_acquire(event, client_id)

        # Release-события.
        if event == EVENT_TELEGRAM_RELEASE:
            return self._handle_release_telegram(client_id)
        if event == EVENT_QUEST_RELEASE:
            return self._handle_release_quest(client_id)

        # На случай, если забыли ветку — ValueError.
        raise ValueError(f"event {event!r} is declared but not handled")

    def tick(self) -> Mode:
        """Проверить idle-timeout и вернуть текущий ``Mode``.

        Чистая операция «проверки состояния»: никакого side-effect на
        ``voice_held_by`` / ``teleop_held_by``, кроме возможного
        перехода в ``off`` при превышении ``IDLE_TIMEOUT_S``.

        Используется ROS 2 таймером супервизора (см. AV-6) и в тестах.
        """
        self._check_idle_timeout()
        return self._mode

    # ---------- обработчики ----------

    def _handle_acquire(self, event: str, client_id: Optional[ClientId]) -> Mode:
        if not client_id:
            raise ValueError(f"event {event!r} requires client_id")

        # Идемпотентность: тот же клиент повторно acquire-ит — no-op.
        if event in (
            EVENT_TELEGRAM_ACQUIRE_FLOOR,
            EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR,
        ):
            if self._voice_held_by == client_id and self._mode != Mode.OFF:
                # Телеграм уже в игре — повторный acquire без смены режима.
                return self._mode
        if event in (
            EVENT_QUEST_ACQUIRE_FLOOR,
            EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY,
        ):
            if self._teleop_held_by == client_id and self._mode != Mode.OFF:
                return self._mode

        if event == EVENT_TELEGRAM_ACQUIRE_FLOOR:
            return self._acquire_telegram_full(client_id)
        if event == EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR:
            return self._acquire_telegram_voice(client_id)
        if event == EVENT_QUEST_ACQUIRE_FLOOR:
            return self._acquire_quest_full(client_id)
        if event == EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY:
            return self._acquire_quest_teleop(client_id)

        # Сюда не должны попадать, но на всякий случай:
        raise ValueError(f"unhandled acquire event {event!r}")

    def _acquire_telegram_full(self, client_id: ClientId) -> Mode:
        """``off → telegram_active`` (или уже там)."""
        if self._mode == Mode.OFF:
            # ADR-0028 §4.1: «off → telegram_active : telegram_acquire_floor».
            # Никаких конфликтов — никто ничего не держит.
            self._voice_held_by = client_id
            self._teleop_held_by = client_id
            self._set_mode(Mode.TELEGRAM_ACTIVE)
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # Уже там, ничего не меняем.
            return self._mode

        # В avatar_present/mixed — нельзя «повторно» захватить full floor.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.TELEGRAM_ACTIVE,
            floor=FLOOR_VOICE,
            held_by=self._voice_held_by or "?",
            requested_by=client_id,
        )

    def _acquire_telegram_voice(self, client_id: ClientId) -> Mode:
        """``avatar_present → mixed`` (telegram держит voice)."""
        if self._mode == Mode.AVATAR_PRESENT:
            # ADR-0028 §4.1: «avatar_present → mixed : telegram_acquire_voice_floor».
            self._voice_held_by = client_id
            self._set_mode(Mode.MIXED)
            return self._mode

        # Из других режимов acquire_voice_floor не имеет смысла.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.MIXED,
            floor=FLOOR_VOICE,
            held_by=self._voice_held_by or "<none>",
            requested_by=client_id,
        )

    def _acquire_quest_full(self, client_id: ClientId) -> Mode:
        """``off → avatar_present`` (или ``telegram_active → avatar_present``)."""
        # Конфликт, если voice_floor уже занят другим (ADR-0028 §4.1).
        if self._voice_held_by is not None and self._voice_held_by != client_id:
            raise ConflictError(
                current_mode=self._mode,
                target_mode=Mode.AVATAR_PRESENT,
                floor=FLOOR_VOICE,
                held_by=self._voice_held_by,
                requested_by=client_id,
            )

        if self._mode == Mode.OFF:
            # «off → avatar_present : quest_acquire_floor».
            self._voice_held_by = client_id
            self._teleop_held_by = client_id
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # «telegram_active → avatar_present : quest_acquire_full_floor».
            self._voice_held_by = client_id
            self._teleop_held_by = client_id
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        if self._mode == Mode.AVATAR_PRESENT:
            return self._mode

        # В mixed — full acquire не имеет смысла (telegram держит voice).
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.AVATAR_PRESENT,
            floor=FLOOR_VOICE,
            held_by=self._voice_held_by or "<none>",
            requested_by=client_id,
        )

    def _acquire_quest_teleop(self, client_id: ClientId) -> Mode:
        """``telegram_active → mixed`` (quest держит только teleop)."""
        if self._mode == Mode.TELEGRAM_ACTIVE:
            # ADR-0028 §4.1: «telegram_active → mixed : quest_acquire_floor (teleop only)».
            self._teleop_held_by = client_id
            self._set_mode(Mode.MIXED)
            return self._mode

        # Из других режимов «acquire teleop only» не имеет смысла.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.MIXED,
            floor=FLOOR_TELEOP,
            held_by=self._teleop_held_by or "<none>",
            requested_by=client_id,
        )

    def _handle_release_telegram(self, client_id: Optional[ClientId]) -> Mode:
        """Telegram отпускает (полностью или voice — нашае решение по FSM-state)."""
        # ``telegram_release`` означает: «я вышел».
        # Если FSM не в режиме с Telegram — это no-op (идемпотентность).
        if self._mode == Mode.OFF:
            return self._mode
        if self._mode == Mode.AVATAR_PRESENT:
            # Telegram не было в игре — no-op.
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # ADR-0028 §4.1: «telegram_active → off : telegram_release (timeout 30s)».
            self._clear_holders()
            self._set_mode(Mode.OFF)
            return self._mode

        # mixed: «mixed → avatar_present : telegram_release_voice».
        # Quest держит teleop, telegram держал voice — после release
        # остаётся только quest.
        if self._mode == Mode.MIXED:
            self._voice_held_by = None
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        return self._mode

    def _handle_release_quest(self, client_id: Optional[ClientId]) -> Mode:
        """Quest отпускает."""
        if self._mode == Mode.OFF:
            return self._mode
        if self._mode == Mode.TELEGRAM_ACTIVE:
            return self._mode

        if self._mode == Mode.AVATAR_PRESENT:
            # ADR-0028 §4.1: «avatar_present → off : quest_release (timeout 30s)».
            self._clear_holders()
            self._set_mode(Mode.OFF)
            return self._mode

        if self._mode == Mode.MIXED:
            # ADR-0028 §4.1: «mixed → telegram_active : quest_release_teleop».
            self._teleop_held_by = None
            self._set_mode(Mode.TELEGRAM_ACTIVE)
            return self._mode

        return self._mode

    def _both_release(self) -> Mode:
        """``mixed → off : both_release``."""
        if self._mode == Mode.MIXED:
            self._clear_holders()
            self._set_mode(Mode.OFF)
            return self._mode
        # Из других режимов — no-op.
        return self._mode

    def _force_off(self) -> Mode:
        """``* → off`` (escape hatch, ADR-0028 §4.1 + §6 Q1 fail-safe)."""
        self._clear_holders()
        self._set_mode(Mode.OFF)
        return self._mode

    # ---------- idle timeout ----------

    def _check_idle_timeout(self) -> None:
        """Если с последней активности прошло > IDLE_TIMEOUT_MS — снять holder-ов."""
        if self._mode in (Mode.OFF, Mode.MIXED):
            # В mixed оба активны — timeout неуместен. В off — нечего снимать.
            return
        now_ms = self._clock()
        if (now_ms - self._last_activity_ms) <= IDLE_TIMEOUT_MS:
            return
        # Idle > timeout — снимаем всё.
        self._clear_holders()
        self._mode = Mode.OFF
        self._last_activity_ms = now_ms

    # ---------- внутреннее ----------

    def _set_mode(self, new_mode: Mode) -> None:
        self._mode = new_mode
        self._last_activity_ms = self._clock()

    def _clear_holders(self) -> None:
        self._voice_held_by = None
        self._teleop_held_by = None

    @staticmethod
    def _default_clock() -> int:
        """Реальное время в миллисекундах от произвольной точки."""
        return int(time.monotonic() * 1000)


__all__ = (
    "ConflictError",
    "ClientId",
    "EVENT_BOTH_RELEASE",
    "EVENT_FORCE_OFF",
    "EVENT_QUEST_ACQUIRE_FLOOR",
    "EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY",
    "EVENT_QUEST_RELEASE",
    "EVENT_TELEGRAM_ACQUIRE_FLOOR",
    "EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR",
    "EVENT_TELEGRAM_RELEASE",
    "FLOOR_TELEOP",
    "FLOOR_VOICE",
    "Floor",
    "IDLE_TIMEOUT_MS",
    "IDLE_TIMEOUT_S",
    "Mode",
    "ModeManager",
)
