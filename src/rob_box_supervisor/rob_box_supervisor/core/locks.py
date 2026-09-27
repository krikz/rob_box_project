"""LockManager — два независимых floor-а (teleop/voice) + dead-man timeout.

ADR-0028 §4.2 (LockManager) + §6 Q4 (dead-man 500 мс). Это чистая
логика без ROS, asyncio и threading — чтобы было легко покрыть TDD
(см. `test/unit/core/test_locks.py`).

Контракт:
  - acquire(client_id, floor) → None если OK, ConflictError если другой
    держит; идемпотентно для того же клиента (повторный acquire — no-op).
  - release(client_id, floor) → None; PermissionError если client_id
    не совпадает с holder; no-op если floor уже released / никем не
    занят (идемпотентность FSM-переходов).
  - heartbeat(client_id, floor, now_ms) → None; PermissionError если
    client_id не совпадает с holder; обновляет last_heartbeat_ms.
  - holder(floor) → client_id | None; если прошло > DEAD_MAN_TIMEOUT_MS
    с последнего heartbeat — возвращает None и чистит состояние.

Часы: опциональный инжектируемый `clock` (callable → int ms). Если не
передан, используется `time.monotonic() * 1000` от `time` stdlib.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional


# === Константы / доменные типы =======================================

DEAD_MAN_TIMEOUT_MS = 500  # ADR-0028 §6 Q4

FLOOR_TELEOP = "teleop_floor"
FLOOR_VOICE = "voice_floor"


class Floor:
    """Доменные значения floor-ов. Строки, чтобы удобно сериализовать."""

    TELEOP = FLOOR_TELEOP
    VOICE = FLOOR_VOICE

    _ALL = (TELEOP, VOICE)

    @classmethod
    def values(cls) -> tuple:
        """Возвращает все валидные значения floor-ов."""
        return cls._ALL


def _all_floors() -> tuple:
    return Floor.values()


# === Исключения =======================================================


class ConflictError(Exception):
    """Запрошенный floor уже занят другим клиентом.

    Атрибуты:
      floor — какой floor был запрошен;
      held_by — кто сейчас держит;
      requested_by — кто пытался acquire.
    """

    def __init__(self, floor: str, held_by: str, requested_by: str):
        self.floor = floor
        self.held_by = held_by
        self.requested_by = requested_by
        super().__init__(
            f"floor {floor!r} already held by {held_by!r}; "
            f"requested by {requested_by!r}"
        )


# === Состояние одного floor-а =========================================


class _FloorState:
    """holder + last_heartbeat_ms; живёт в LockManager._floors[floor]."""

    __slots__ = ("holder", "last_heartbeat_ms")

    def __init__(self, holder: str, last_heartbeat_ms: int):
        self.holder = holder
        self.last_heartbeat_ms = last_heartbeat_ms


# === LockManager =====================================================


class LockManager:
    """Менеджер floor-ов: `teleop_floor` + `voice_floor` с dead-man timeout.

    Потокобезопасность: НЕ потокобезопасен (по контракту используется из
    однопоточного FSM-обработчика ROS 2 callback-group, см. ADR-0028 §4).
    """

    def __init__(
        self,
        clock: Optional[Callable[[], int]] = None,
        timeout_ms: Optional[int] = None,
    ):
        """Создать LockManager.

        :param clock: опциональный инжектируемый источник времени (ms).
            ``None`` → ``time.monotonic()``.
        :param timeout_ms: опциональный override dead-man timeout. ``None`` →
            module-level :data:`DEAD_MAN_TIMEOUT_MS`. AV-13: супервизор
            пробрасывает сюда значение ROS-параметра
            ``dead_man_timeout_ms`` (ADR-0028 §6 Q4 — настраивается на
            железе без пересборки).
        """
        self._clock = clock or self._default_clock
        self._timeout_ms = timeout_ms if timeout_ms is not None else DEAD_MAN_TIMEOUT_MS
        # floor → _FloorState (None если свободен)
        self._floors: Dict[str, Optional[_FloorState]] = {
            FLOOR_TELEOP: None,
            FLOOR_VOICE: None,
        }

    # ---------- основной API ----------

    def acquire(self, client_id: str, floor: str, now_ms: Optional[int] = None) -> None:
        """Захватить floor. ConflictError если другой держит; идемпотентно для owner."""
        self._validate_client_id(client_id)
        self._validate_floor(floor)

        state = self._floors[floor]
        if state is not None:
            # Сначала проверяем, не истёк ли floor (dead-man мог сработать).
            if self._is_alive(state, now_ms):
                if state.holder == client_id:
                    # Идемпотентность: тот же клиент acquire-ит тот же floor — no-op.
                    return
                raise ConflictError(
                    floor=floor, held_by=state.holder, requested_by=client_id
                )
            # Истёк — авто-release перед новым acquire.
            self._floors[floor] = None

        ts = self._resolve_now(now_ms)
        self._floors[floor] = _FloorState(holder=client_id, last_heartbeat_ms=ts)

    def release(self, client_id: str, floor: str) -> None:
        """Отпустить floor. PermissionError если чужой; no-op если уже released."""
        self._validate_client_id(client_id)
        self._validate_floor(floor)

        state = self._floors[floor]
        if state is None:
            # Идемпотентность: уже released / никем не занят — no-op.
            return
        if state.holder != client_id:
            raise PermissionError(
                f"client {client_id!r} cannot release floor {floor!r}; "
                f"held by {state.holder!r}"
            )
        self._floors[floor] = None

    def heartbeat(
        self, client_id: str, floor: str, now_ms: Optional[int] = None
    ) -> None:
        """Продлить жизнь floor-а. PermissionError если чужой."""
        self._validate_client_id(client_id)
        self._validate_floor(floor)

        state = self._floors[floor]
        if state is None:
            # Heartbeat без активного holder-а — это либо race с release,
            # либо acquire ещё не было. Контракт: PermissionError, чтобы
            # клиент явно видел, что heartbeat пришёл не туда.
            raise PermissionError(
                f"no client holds floor {floor!r}; cannot heartbeat as {client_id!r}"
            )
        if state.holder != client_id:
            raise PermissionError(
                f"client {client_id!r} cannot heartbeat floor {floor!r}; "
                f"held by {state.holder!r}"
            )
        state.last_heartbeat_ms = self._resolve_now(now_ms)

    def holder(self, floor: str) -> Optional[str]:
        """Кто сейчас держит floor; None если свободен или expired."""
        # Различаем: None (явно освобождён) vs "не передан" — тест
        # `holder(None) → ValueError` должен ловить именно None как
        # невалидный floor.
        if floor is None:
            raise ValueError("floor must be a non-None string")
        if floor not in self._floors:
            raise ValueError(f"unknown floor {floor!r}")

        state = self._floors[floor]
        if state is None:
            return None
        if not self._is_alive(state, now_ms=None):
            # Dead-man: авто-release при следующем чтении.
            self._floors[floor] = None
            return None
        return state.holder

    def force_expire(self, floor: str, now_ms: Optional[int] = None) -> Optional[str]:
        """Активно снять floor по истечении dead-man timeout (AV-13).

        Возвращает ``client_id`` который держал floor, или ``None`` если
        floor уже свободен / не истёк / неизвестен. Не валит на
        не-валидном floor (это watcher, он обходит все floor-ы из
        :py:meth:`Floor.values`).

        Контракт: в отличие от :py:meth:`holder` (ленивый auto-release
        при чтении), :py:meth:`force_expire` активно переводит expired
        floor в ``None`` — это нужно watcher-у, чтобы засечь trip и
        опубликовать ``/avatar/state`` ВНЕОЧЕРЕДНО (а не ждать следующего
        ``holder()``). Идемпотентно: повторный вызов на уже-released
        floor → ``None``.
        """
        state = self._floors.get(floor)
        if state is None:
            return None
        if self._is_alive(state, now_ms):
            return None  # ещё не истёк
        expired_holder = state.holder
        self._floors[floor] = None
        return expired_holder

    # ---------- внутреннее ----------

    @staticmethod
    def _default_clock() -> int:
        """Реальное время в миллисекундах от произвольной точки."""
        return int(time.monotonic() * 1000)

    def _resolve_now(self, now_ms: Optional[int]) -> int:
        """Если передан явный now_ms — он (тестируемо); иначе clock()."""
        return now_ms if now_ms is not None else self._clock()

    def _is_alive(self, state: _FloorState, now_ms: Optional[int]) -> bool:
        """True если с последнего heartbeat прошло <= self._timeout_ms.

        AV-13: раньше был жёстко зашит :data:`DEAD_MAN_TIMEOUT_MS` — теперь
        берётся из instance, чтобы супервизор мог тюнить порог через
        ROS-параметр ``dead_man_timeout_ms`` без пересборки.
        """
        ts = self._resolve_now(now_ms)
        return (ts - state.last_heartbeat_ms) <= self._timeout_ms

    @staticmethod
    def _validate_client_id(client_id: str) -> None:
        if not isinstance(client_id, str) or not client_id:
            raise ValueError("client_id must be a non-empty string")

    @staticmethod
    def _validate_floor(floor: str) -> None:
        if floor not in _all_floors():
            raise ValueError(f"unknown floor {floor!r}")
