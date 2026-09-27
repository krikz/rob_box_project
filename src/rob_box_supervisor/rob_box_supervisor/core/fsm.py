"""ModeManager — FSM режимов аватара (AV-3, ADR-0028 §4.1, ADR-0051 §2.2).

Это **чистая Python-логика** без ROS, asyncio и threading, чтобы её было
легко покрыть TDD (см. ``test/unit/core/test_fsm.py``). FSM **хранит
только режим**: кто держит ``voice_floor`` / ``teleop_floor`` — знает
исключительно :class:`LockManager` (AV-4). Это второй инвариант
ADR-0051 («один владелец floor»), следующий шаг после ADR-0028 §4.2:
раньше FSM вело свою копию ``voice_held_by`` / ``teleop_held_by``, и в
тестах на гонку два клиента получали два разных вердикта (FSM говорил
«conflict», LockManager говорил «granted» — оба правы по своим
контрактам, и это самый частый регресс оператора). Теперь FSM читает
только ``mode`` + ``last_activity_ms`` для ``IDLE_TIMEOUT_S``.

Таблица переходов — точная копия ADR-0028 §4.1 (mermaid-stateDiagram).
Конфликты (``ConflictError``) теперь чисто-режимные: «из текущего
режима запрошенный переход недопустим». Всё, что требует знания
«кто держит voice», — зона ответственности арбитра
(``avatar_arbiter._set_avatar_mode_logic``): он запрашивает
``LockManager.holder()`` ДО ``transition()`` и отдаёт ``reason=conflict``
в клиентский ответ.

``* → off`` всегда разрешён (escape hatch): никакое состояние не может
заблокировать выключение.

Часы — опциональный инжектируемый ``clock`` (callable → int ms).
Используется для ``IDLE_TIMEOUT_S`` (см. ADR-0028 §4.1: «timeout 30 s
|no activity»). Если ``clock`` не передан, берётся ``time.monotonic() *
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


# Имя клиента — это ``str`` (telegram/quest/...). Типизируем для
# читаемости сигнатур.
ClientId = str


# === Исключения ======================================================


class ConflictError(Exception):
    """Запрошенный переход невозможен из текущего режима.

    Атрибуты:
      current_mode — в каком режиме находились;
      target_mode — куда пытались перейти.

    До ADR-0051 §2.2 здесь были ещё ``floor`` / ``held_by`` /
    ``requested_by`` — FSM знал, кто держит voice/teleop. С тех пор
    единственный владелец этих фактов — :class:`LockManager`, и
    клиентский «кто сейчас держит floor» нужно читать оттуда (или из
    ``/avatar/state`` через кэш). FSM-уровень conflict теперь чисто
    про режимы: «из mixed нельзя взять ``quest_acquire_floor`` full —
    это семантически переход mixed → avatar_present, требующий
    сначала ``telegram_release``».
    """

    def __init__(
        self,
        current_mode: Mode,
        target_mode: Mode,
    ):
        self.current_mode = current_mode
        self.target_mode = target_mode
        super().__init__(
            f"cannot transition {current_mode.value} → {target_mode.value}"
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

    Контракт (ADR-0051 §2.2, после удаления holder-полей):
      - ``mode`` — текущий режим (изначально ``OFF``).
      - ``transition(event, client_id=None) -> Mode`` — переход по событию.
        Возвращает новый ``Mode``. Бросает:
          - ``ConflictError`` если переход семантически недопустим из
            текущего режима (например, ``quest_acquire_floor`` из
            ``mixed`` — это требовало бы сначала ``telegram_release``);
          - ``ValueError`` если ``event`` не известен.
        Идемпотентность: повторный acquire в том же режиме — **no-op**
        (без raise). Раньше idempotency явно проверялась по
        ``voice_held_by == client_id``; теперь она автоматически
        вытекает из того, что FSM не меняет режим, если он уже в
        целевом (mode-based idempotency).
      - ``* → off`` через ``EVENT_FORCE_OFF`` — всегда разрешён.
      - ``IDLE_TIMEOUT_S`` простоя (без ``acquire``/``release``) в
        ``telegram_active`` или ``avatar_present`` → следующий
        ``*_release`` переводит FSM в ``off``.

    FSM **не знает**, кто держит ``voice_floor`` / ``teleop_floor``.
    Это знает :class:`LockManager` (AV-4) — единый владелец floor-ов.
    Арбитр (``avatar_arbiter``) координирует их: до ``transition()``
    смотрит ``LockManager.holder()``, чтобы вернуть клиенту
    ``reason=conflict`` если целевой floor занят другим; после —
    зеркально освобождает floor в LockManager, если режим сменился
    с активного на пассивный.
    """

    def __init__(self, clock: Optional[Callable[[], int]] = None):
        self._clock = clock or self._default_clock
        self._mode: Mode = Mode.OFF
        # ``last_activity_ms`` — последний раз, когда FSM менял mode
        # (для IDLE_TIMEOUT_S). Раньше сюда же «обновлялась»
        # holder-активность; теперь — только смена mode.
        self._last_activity_ms: int = self._clock()

    # ---------- публичный API ----------

    @property
    def mode(self) -> Mode:
        """Текущий режим."""
        return self._mode

    def transition(self, event: str, client_id: Optional[ClientId] = None) -> Mode:
        """Применить событие и вернуть новый ``Mode``.

        Параметры:
          event — имя события (``EVENT_*`` или ``force_off``);
          client_id — идентификатор клиента-инициатора. Обязателен для
            событий ``acquire``/``release``, опционален для
            ``both_release`` / ``force_off``.

        ``client_id`` сейчас НЕ используется внутри FSM — owner-floor
        знает :class:`LockManager`. Параметр сохранён в сигнатуре
        ради backward compat и на случай, если FSM снова понадобится
        учитывать client_id (например, для метрик или аудита).
        """
        if event not in _ALL_EVENTS:
            raise ValueError(f"unknown event {event!r}")

        self._check_idle_timeout()

        if event == EVENT_FORCE_OFF:
            return self._force_off()

        if event == EVENT_BOTH_RELEASE:
            return self._both_release()

        # Acquire-события: idempotency обеспечивается тем, что mode
        # уже целевой → возвращаемся без изменений.
        if event == EVENT_TELEGRAM_ACQUIRE_FLOOR:
            return self._acquire_telegram_full(client_id)
        if event == EVENT_TELEGRAM_ACQUIRE_VOICE_FLOOR:
            return self._acquire_telegram_voice(client_id)
        if event == EVENT_QUEST_ACQUIRE_FLOOR:
            return self._acquire_quest_full(client_id)
        if event == EVENT_QUEST_ACQUIRE_FLOOR_TELEOP_ONLY:
            return self._acquire_quest_teleop(client_id)

        # Release-события.
        if event == EVENT_TELEGRAM_RELEASE:
            return self._handle_release_telegram(client_id)
        if event == EVENT_QUEST_RELEASE:
            return self._handle_release_quest(client_id)

        # На случай, если забыли ветку — ValueError.
        raise ValueError(f"event {event!r} is declared but not handled")

    def tick(self) -> Mode:
        """Проверить idle-timeout и вернуть текущий ``Mode``.

        Чистая операция «проверки состояния»: без побочных эффектов
        на holder-полях (их больше нет), кроме возможного перехода в
        ``off`` при превышении ``IDLE_TIMEOUT_S``.

        Используется ROS 2 таймером супервизора (см. AV-6) и в тестах.
        """
        self._check_idle_timeout()
        return self._mode

    # ---------- обработчики ----------

    def _acquire_telegram_full(self, client_id: Optional[ClientId]) -> Mode:
        """``off → telegram_active`` (или уже там).

        Идемпотентность: если уже ``TELEGRAM_ACTIVE`` — no-op без raise.
        Если mode ``AVATAR_PRESENT`` / ``MIXED`` — это конфликт
        режимов: телеграм не может «повторно» встать в TELEGRAM_ACTIVE,
        пока активен quest (для этого нужен ``quest_release``). С
        точки зрения арбитра voice_floor при этом занят другим
        клиентом — он отдаст ``reason=conflict`` ещё до этого вызова,
        но на уровне FSM остаётся семантическая проверка «можно ли из
        текущего mode перейти в TELEGRAM_ACTIVE».
        """
        del client_id  # FSM больше не использует client_id

        if self._mode == Mode.OFF:
            # ADR-0028 §4.1: «off → telegram_active : telegram_acquire_floor».
            self._set_mode(Mode.TELEGRAM_ACTIVE)
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # Уже там, ничего не меняем.
            return self._mode

        # В avatar_present/mixed — нельзя «повторно» захватить full floor.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.TELEGRAM_ACTIVE,
        )

    def _acquire_telegram_voice(self, client_id: Optional[ClientId]) -> Mode:
        """``avatar_present → mixed`` (telegram держит voice).

        Из других режимов ``acquire_voice_floor`` не имеет смысла —
        семантический конфликт режимов.
        """
        del client_id

        if self._mode == Mode.AVATAR_PRESENT:
            # ADR-0028 §4.1: «avatar_present → mixed : telegram_acquire_voice_floor».
            self._set_mode(Mode.MIXED)
            return self._mode

        # Из других режимов acquire_voice_floor не имеет смысла.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.MIXED,
        )

    def _acquire_quest_full(self, client_id: Optional[ClientId]) -> Mode:
        """``off → avatar_present`` (или ``telegram_active → avatar_present``).

        Арбитр до этого вызова уже проверил ``LockManager.holder()`` —
        если voice_floor держит другой client, вернётся
        ``reason=conflict`` ещё до FSM. Здесь остаётся чисто
        режимная проверка: full-quest-acquire из ``mixed`` /
        ``avatar_present`` не имеет смысла без предварительного release.
        """
        del client_id

        if self._mode == Mode.OFF:
            # «off → avatar_present : quest_acquire_floor».
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # «telegram_active → avatar_present : quest_acquire_full_floor».
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        if self._mode == Mode.AVATAR_PRESENT:
            # Уже там — idempotency no-op.
            return self._mode

        # В mixed — full acquire не имеет смысла: телеграм всё ещё
        # держит voice. Переход mixed → avatar_present требует
        # сначала ``telegram_release``.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.AVATAR_PRESENT,
        )

    def _acquire_quest_teleop(self, client_id: Optional[ClientId]) -> Mode:
        """``telegram_active → mixed`` (quest держит только teleop).

        Из других режимов «acquire teleop only» не имеет смысла —
        семантический конфликт режимов.
        """
        del client_id

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # ADR-0028 §4.1: «telegram_active → mixed : quest_acquire_floor (teleop only)».
            self._set_mode(Mode.MIXED)
            return self._mode

        # Из других режимов «acquire teleop only» не имеет смысла.
        raise ConflictError(
            current_mode=self._mode,
            target_mode=Mode.MIXED,
        )

    def _handle_release_telegram(self, client_id: Optional[ClientId]) -> Mode:
        """Telegram отпускает (полностью или voice — наше решение по FSM-state)."""
        del client_id

        # ``telegram_release`` означает: «я вышел».
        # Если FSM не в режиме с Telegram — это no-op (идемпотентность).
        if self._mode == Mode.OFF:
            return self._mode
        if self._mode == Mode.AVATAR_PRESENT:
            # Telegram не было в игре — no-op.
            return self._mode

        if self._mode == Mode.TELEGRAM_ACTIVE:
            # ADR-0028 §4.1: «telegram_active → off : telegram_release (timeout 30s)».
            self._set_mode(Mode.OFF)
            return self._mode

        # mixed: «mixed → avatar_present : telegram_release_voice».
        # Quest держит teleop, telegram держал voice — после release
        # остаётся только quest.
        if self._mode == Mode.MIXED:
            self._set_mode(Mode.AVATAR_PRESENT)
            return self._mode

        return self._mode

    def _handle_release_quest(self, client_id: Optional[ClientId]) -> Mode:
        """Quest отпускает."""
        del client_id

        if self._mode == Mode.OFF:
            return self._mode
        if self._mode == Mode.TELEGRAM_ACTIVE:
            return self._mode

        if self._mode == Mode.AVATAR_PRESENT:
            # ADR-0028 §4.1: «avatar_present → off : quest_release (timeout 30s)».
            self._set_mode(Mode.OFF)
            return self._mode

        if self._mode == Mode.MIXED:
            # ADR-0028 §4.1: «mixed → telegram_active : quest_release_teleop».
            self._set_mode(Mode.TELEGRAM_ACTIVE)
            return self._mode

        return self._mode

    def _both_release(self) -> Mode:
        """``mixed → off : both_release``."""
        if self._mode == Mode.MIXED:
            self._set_mode(Mode.OFF)
            return self._mode
        # Из других режимов — no-op.
        return self._mode

    def _force_off(self) -> Mode:
        """``* → off`` (escape hatch, ADR-0028 §4.1 + §6 Q1 fail-safe)."""
        self._set_mode(Mode.OFF)
        return self._mode

    # ---------- idle timeout ----------

    def _check_idle_timeout(self) -> None:
        """Если с последней активности прошло > IDLE_TIMEOUT_MS — снять режим.

        В mixed оба клиента активны — timeout неуместен. В off — нечего
        снимать.
        """
        if self._mode in (Mode.OFF, Mode.MIXED):
            return
        now_ms = self._clock()
        if (now_ms - self._last_activity_ms) <= IDLE_TIMEOUT_MS:
            return
        # Idle > timeout — переходим в off.
        self._mode = Mode.OFF
        self._last_activity_ms = now_ms

    # ---------- внутреннее ----------

    def _set_mode(self, new_mode: Mode) -> None:
        self._mode = new_mode
        self._last_activity_ms = self._clock()

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
    "IDLE_TIMEOUT_MS",
    "IDLE_TIMEOUT_S",
    "Mode",
    "ModeManager",
)
