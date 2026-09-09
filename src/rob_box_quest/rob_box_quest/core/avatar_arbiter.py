"""LocalAvatarArbiterClient — локальный stub avatar_arbiter service для Quest-WS.

ADR-0051 §2.2 (issue #1999) — единственный владелец floor-ов это
:class:`rob_box_supervisor.core.locks.LockManager`. Quest-WS в
production должен звать avatar_arbiter через ROS 2 (планируется
отдельной карточкой), но до тех пор ws_server нуждается в
test-friendly замене — этой заменой и является
:class:`LocalAvatarArbiterClient`.

Назначение:

  - **Для тестов**: реализует локальный mutex на teleop_floor /
    voice_floor (раньше жил в ``SupervisorFloorTracker`` и
    ``VoiceFloor``). Тесты инжектят клиент через DI и не нуждаются
    в rclpy;
  - **Для runtime в Phase 1** (до ROS 2 service call): клиент
    выполняет ту же роль на уровне одного процесса. Поведение
    тождественно «локальному avatar_arbiter»: один держатель,
    идемпотентность, rate-limit FLOOR_HELD;
  - **Когда ROS 2 client будет подключён**: вся эта логика
    переедет в ``RosAvatarArbiterClient``, клиент удалится. Quest
    код не поменяется — поменяется только DI-объект.

Контракт клиента (сигнатура совпадает с тем, что avatar_arbiter
service будет отдавать):

  - ``try_acquire_floor(session_id, client_id) -> AcquireResult``:
    попытка занять teleop_floor. Идемпотентно для того же session_id;
  - ``release_floor(session_id) -> bool``: отпустить, True если
    session_id реально держал;
  - ``try_acquire_voice(session_id, client_id) -> AcquireResult``:
    то же для voice_floor;
  - ``release_voice(session_id) -> bool``: voice release;
  - ``force_release_for(session_id) -> bool``: watchdog-trip /
    disconnect — освободить ОБА floor-а если их держала эта сессия;
  - ``floor_holder`` / ``voice_holder``: read-only свойства —
    mirror текущего состояния (после каждой мутации кэш
    синхронизируется);
  - ``should_send_floor_held_error(session_id) -> bool`` /
    ``reset_floor_held_rate_limit(session_id)``: rate-limit для
    FLOOR_HELD на стороне сервиса.

Все мутации клиента проталкивают результат в
:class:`AvatarStateFloorCache` (для read-only UI / WS-подписчиков).
Сам кэш по-прежнему read-only и не имеет acquire/release — это
гарантирует ADR-0051 §2.2 в том смысле, что Quest не имеет
stateful floor-логики вне клиента.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

from rob_box_quest.core.floor import (
    AvatarFloorSnapshot,
    AvatarStateFloorCache,
    FloorHolder,
    make_server_client_id,
)


# Анти-spam для ERROR{FLOOR_HELD}: на 30 Гц teleop_twist при свободном
# holder-сессии не должны получать по 30 ошибок в секунду. Раз в 1 с — норма.
FLOOR_HELD_RATE_LIMIT_S: float = 1.0


@dataclass(frozen=True)
class AcquireResult:
    """Ответ на ``try_acquire_*``.

    Зеркалит ``AcquireResult`` из старого ``SupervisorFloorTracker``
    для backward compat с уже написанным ws_server кодом.

    ``busy_holder`` — :class:`FloorHolder` текущего держителя
    voice_floor (None для teleop_floor, там достаточно session_id).
    """

    granted: bool
    held_by: Optional[str] = None
    reason: str = ""
    busy_holder: Optional[FloorHolder] = None


class LocalAvatarArbiterClient:
    """Локальный stub avatar_arbiter service для Quest-WS.

    До того, как Quest подключится к реальному ``/avatar_arbiter``
    service через ROS 2, этот клиент играет его роль в одном
    процессе. Когда сервис будет подключён, класс заменится на
    ``RosAvatarArbiterClient`` (DI-объект), Quest-код не поменяется.

    После каждой мутации (acquire/release/force_release) клиент
    пушит свежий :class:`AvatarFloorSnapshot` в связанный кэш —
    чтобы read-only подписчики видели актуальное состояние без
    прямого доступа к mutex-логике клиента.
    """

    __slots__ = (
        "_floor_holder",
        "_voice_holder",
        "_voice_holder_client_id",
        "_last_error_monotonic",
        "_now_fn",
        "_cache",
    )

    def __init__(
        self,
        cache: AvatarStateFloorCache,
        now_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        # issue #2190 (voice-vr 05): теперь храним server_client_id
        # (формат ``"quest:<session_uuid>"``) — единый идентификатор
        # сессии во ВСЕХ точках соприкосновения (ws_server gate,
        # heartbeat, supervisor_* API, STATE_UPDATE клиенту).
        # Раньше здесь лежал session_id, что ломало сопоставление
        # client_id из /avatar/state и client_id из supervisor_*
        # вызовов (там всегда "quest:<uuid>").
        self._floor_holder: Optional[str] = None
        # voice floor: session_id + client_id (последний — для UI label).
        # ADR-0051 §2.2: avatar_arbiter service в Phase 2 будет
        # возвращать FloorHolder целиком; пока мы его собираем сами.
        # Для voice floor session_id остаётся «как было» — это
        # внутренний ключ, который видит только Quest-WS
        # (release/force_release идут по session_id, голосовой floor
        # не уходит в supervisor_* API напрямую).
        self._voice_holder: Optional[str] = None
        self._voice_holder_client_id: Optional[str] = None
        self._last_error_monotonic: dict[str, float] = {}
        self._now_fn = now_fn if now_fn is not None else time.monotonic
        self._cache: AvatarStateFloorCache = cache

    # ---------- teleop_floor ----------

    def try_acquire_floor(
        self, session_id: str, client_id: str = ""
    ) -> AcquireResult:
        """Попытка занять teleop_floor от имени ``session_id``.

        Идемпотентно для того же session_id. ``client_id`` —
        дополнительный идентификатор для логов (``AcquireResult.held_by``
        вернёт именно его, если передан). Для хранения holder-а в
        ``_floor_holder`` ВСЕГДА используется
        :py:func:`make_server_client_id` формат (``"quest:<session_uuid>"``),
        см. issue #2190 §«единый client_id». Это обеспечивает
        совпадение с ``/avatar/state.teleop_floor.client_id``, с
        WELCOME.teleop_floor_held_by и с ``Bridge.supervisor_*``
        client_id-аргументами — единая идентичность во всех точках.

        Контракт ADR-0051 §2.2: фактический holder — server_client_id,
        session_id остаётся внутренним ключом release-логики
        (``release_floor(session_id)``).
        """
        if not session_id:
            return AcquireResult(granted=False, reason="invalid_session_id")
        holder_key = make_server_client_id(session_id)
        if not holder_key:
            return AcquireResult(granted=False, reason="invalid_session_id")
        # ``held_by`` = server_client_id holder-а (issue #2190 §«единый
        # client_id»). UI/логи/JSON-ответ — везде один формат
        # ``"quest:<uuid>"``, без отдельного ``client_id``.
        if self._floor_holder == holder_key:
            return AcquireResult(
                granted=True, held_by=holder_key, reason="already_held"
            )
        if self._floor_holder is None:
            self._floor_holder = holder_key
            self._push_to_cache()
            return AcquireResult(
                granted=True, held_by=holder_key, reason="granted"
            )
        return AcquireResult(
            granted=False,
            held_by=self._floor_holder,
            reason="held_by_other",
        )

    def release_floor(self, session_id: str) -> bool:
        """Отпустить teleop_floor. True если session_id реально держал.

        Принимает session_id (для совместимости с _unregister_session),
        но ищет по ``server_client_id(session_id)`` — потому что
        фактический holder хранится именно в этом формате.
        """
        if not session_id:
            return False
        holder_key = make_server_client_id(session_id)
        if self._floor_holder == holder_key:
            self._floor_holder = None
            self._push_to_cache()
            return True
        return False

    def force_release_floor(self) -> Optional[str]:
        """Сбросить teleop_floor (reset/shutdown). Возвращает prev holder."""
        prev = self._floor_holder
        self._floor_holder = None
        self._push_to_cache()
        return prev

    # ---------- voice_floor ----------

    def try_acquire_voice(
        self, session_id: str, client_id: str = ""
    ) -> AcquireResult:
        """Попытка занять voice_floor от имени ``session_id``.

        Один держатель на все WS-сессии — два квеста (оператор +
        telegram-bridge) не должны микшировать голос в /avatar/voice_in.

        ``client_id`` сохраняется в BusyHolder для UI label
        (как делал старый VoiceFloor).
        """
        if not session_id:
            return AcquireResult(granted=False, reason="invalid_session_id")
        if self._voice_holder == session_id:
            return AcquireResult(
                granted=True,
                held_by=session_id,
                reason="already_held",
                busy_holder=self._voice_holder_obj(),
            )
        if self._voice_holder is None:
            self._voice_holder = session_id
            self._voice_holder_client_id = client_id or None
            self._push_to_cache()
            return AcquireResult(
                granted=True,
                held_by=session_id,
                reason="granted",
                busy_holder=self._voice_holder_obj(),
            )
        return AcquireResult(
            granted=False,
            held_by=self._voice_holder,
            reason="held_by_other",
            busy_holder=self._voice_holder_obj(),
        )

    def release_voice(self, session_id: str) -> bool:
        """Отпустить voice_floor. True если session_id реально держал."""
        if self._voice_holder == session_id:
            self._voice_holder = None
            self._voice_holder_client_id = None
            self._push_to_cache()
            return True
        return False

    def force_release_voice(self) -> Optional[str]:
        """Сбросить voice_floor."""
        prev = self._voice_holder
        self._voice_holder = None
        self._voice_holder_client_id = None
        self._push_to_cache()
        return prev

    def _voice_holder_obj(self) -> Optional[FloorHolder]:
        """FloorHolder для текущего держателя voice_floor (или None)."""
        if self._voice_holder is None:
            return None
        return FloorHolder(
            session_id=self._voice_holder,
            client_id=self._voice_holder_client_id or "anon",
        )

    def force_release_for(self, session_id: str) -> bool:
        """Принудительное освобождение ОБОИХ floor-ов для данной сессии.

        Используется при watchdog-trip / disconnect: если эта
        сессия держала хоть один floor — освободить и вернуть True.
        """
        released_floor = self.release_floor(session_id)
        released_voice = self.release_voice(session_id)
        return released_floor or released_voice

    # ---------- rate-limit FLOOR_HELD ----------

    def should_send_floor_held_error(self, session_id: str) -> bool:
        """True если можно слать ERROR{FLOOR_HELD} (анти-spam 1 с)."""
        now = self._now_fn()
        # ``-inf`` гарантирует, что первая ошибка после старта/сброса
        # всегда проходит (раньше тут был ``0.0``, и при ``now=0.0``
        # ``now - last = 0 < 1.0`` → ошибка блокировалась).
        last = self._last_error_monotonic.get(session_id, float("-inf"))
        if (now - last) >= FLOOR_HELD_RATE_LIMIT_S:
            self._last_error_monotonic[session_id] = now
            return True
        return False

    def reset_floor_held_rate_limit(self, session_id: Optional[str]) -> None:
        """Сброс rate-limit окна (при release / re-acquire)."""
        if session_id is not None:
            self._last_error_monotonic.pop(session_id, None)

    # ---------- read-only mirror ----------

    @property
    def floor_holder(self) -> Optional[str]:
        """Текущий держатель teleop_floor (или None). Для UI/логов."""
        return self._floor_holder

    @property
    def voice_holder(self) -> Optional[str]:
        """Текущий держатель voice_floor (или None)."""
        return self._voice_holder

    # ---------- внутреннее ----------

    def _push_to_cache(self) -> None:
        """Обновить snapshot в подключённом кэше.

        Делается после каждой мутации floor-ов, чтобы read-only
        подписчики (UI / ``/avatar/state``-зеркала) видели
        актуальное состояние без прямого доступа к mutex.
        """
        self._cache.update(
            AvatarFloorSnapshot(
                teleop_holder=self._floor_holder,
                voice_holder=self._voice_holder,
            )
        )


__all__ = [
    "LocalAvatarArbiterClient",
    "AcquireResult",
    "FLOOR_HELD_RATE_LIMIT_S",
]
