"""AvatarStateFloorCache + VoiceFloorCache — read-only кэши floor-ов из /avatar/state.

ADR-0051 §2.2 (issue #1999) — единственный владелец floor-ов это
:class:`rob_box_supervisor.core.locks.LockManager`. Quest больше **не
имеет своего состояния** floor-ов (раньше ``SupervisorFloorTracker``
вел локальный учёт teleop_floor и был вторым источником истины — отсюда
росли гонки вида «FSM говорит conflict, tracker говорит granted»).

Этот модуль — **только отображение**:

  - WS-сервер получает ``/avatar/state`` (msgpack/JSON из avatar_arbiter),
    декодирует в :class:`AvatarFloorSnapshot` и пушит в
    :class:`AvatarStateFloorCache` (для teleop) и
    :class:`VoiceFloorCache` (для voice);
  - :py:meth:`AvatarStateFloorCache.is_held_by` отвечает на вопрос
    «quest-сессия сейчас держит teleop?» — ws_server гейтит
    ``teleop_twist`` на основе этого ответа;
  - :py:meth:`VoiceFloorCache.is_held_by_session` гейтит
    ``voice_ptt_start/stop`` и VOICE_AUDIO фреймы — пропускаем только
    если наша сессия — текущий holder.

Контракт (ADR-0051 §2.2, шаг 9 «один владелец floor»):

  - **никаких** ``acquire`` / ``release`` / ``force_release``: всё это
    теперь живёт в LockManager и зовётся через avatar_arbiter service.
    Кэш тут — только mirror последнего увиденного состояния;
  - ``holder`` / ``state`` — read-only свойства;
  - :py:meth:`update` — единственный мутирующий метод, вызывается из
    ws_server по приходу ``/avatar/state``.

Контекст миграции (было в ``SupervisorFloorTracker`` и ``VoiceFloor``,
удалено в #1999 / C2):

  - Раньше ``acquire(client_id)`` локально проверял, свободен ли
    floor, и выставлял ``_holder``. Теперь это целиком зона
    ответственности LockManager — ws_server не зовёт ``acquire``,
    только смотрит кэш;
  - Раньше ``should_send_floor_held_error`` rate-limit'ил FLOOR_HELD
    ошибки. Теперь FLOOR_HELD отдаёт сам avatar_arbiter (reason в
    ответе сервиса), и rate-limit переехал на сторону сервиса —
    quest-кэш не нужен;
  - ``is_held_by(client_id)`` остался — это единственное, что нужно
    ws_server-у для гейта ``teleop_twist``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


# Клиент-id, который считается «quest-сессией» по умолчанию для
# fallback-сравнений, если в snapshot явно не передан.
QUEST_DEFAULT_CLIENT_ID: str = "quest"


def make_server_client_id(session_id: Optional[str]) -> str:
    """Серверный ``client_id`` для ``session_id``.

    Единый формат по issue #2190 (voice-vr 05): все точки
    соприкосновения (ws_server gate, heartbeat, avatar_arbiter
    client, STATE_UPDATE в браузер) используют один и тот же
    ``server_client_id`` (``"quest:<session_uuid>"``) — это
    «внешнее имя» сессии, которое видит avatar_supervisor и
    клиент в /avatar/state. Раньше в LocalAvatarArbiterClient
    хранился ``session_id`` напрямую, что приводило к
    расхождению с supervisor_* API и невозможности сопоставить
    client_id из STATE_UPDATE с client_id из HELLO.

    ``session_id=None`` → возвращаем пустую строку, чтобы
    type-checker в тестах и call-sites не ругался на Optional.

    ВАЖНО: дубликат :py:func:`rob_box_quest.server.session.server_client_id`
    (там же формат строки). Дубликат нужен, чтобы не тащить
    server/session в core/avatar_arbiter.py (иначе — циркулярный
    импорт через server/__init__.py → ws_server → avatar_arbiter).
    Если формат изменится — синхронизировать обе функции.
    """
    if not session_id:
        return ""
    return f"quest:{session_id}"


@dataclass(frozen=True)
class AvatarFloorSnapshot:
    """Снимок состояния floor-ов из /avatar/state (AV-14, ADR-0051 §2.2).

    Attributes:
        teleop_holder: client_id держателя teleop_floor или None.
        voice_holder: client_id держателя voice_floor или None.
        avatar_mode: текущий режим FSM ("off" / "telegram_active" /
            "avatar_present" / "mixed"). Используется только для
            информационных сообщений в логах и UI — решения о гейтах
            принимаются по ``teleop_holder``.
        schema_version: версия схемы /avatar/state (AV-14). Если
            None — мы не знаем версию и считаем snapshot сырым.
    """

    teleop_holder: Optional[str] = None
    voice_holder: Optional[str] = None
    avatar_mode: str = "off"
    schema_version: Optional[int] = None


class AvatarStateFloorCache:
    """Read-only кэш floor-ов из /avatar/state (ADR-0051 §2.2).

    Single-thread: все вызовы из aiohttp event-loop. Никаких блокировок
    и никаких блокирующих I/O — обновление синхронное.
    """

    __slots__ = ("_snapshot",)

    def __init__(
        self,
        initial_snapshot: Optional[AvatarFloorSnapshot] = None,
    ) -> None:
        self._snapshot: AvatarFloorSnapshot = (
            initial_snapshot
            if initial_snapshot is not None
            else AvatarFloorSnapshot()
        )

    @property
    def holder(self) -> Optional[str]:
        """client_id текущего держателя teleop_floor или None."""
        return self._snapshot.teleop_holder

    @property
    def voice_holder(self) -> Optional[str]:
        """client_id текущего держателя voice_floor или None."""
        return self._snapshot.voice_holder

    @property
    def avatar_mode(self) -> str:
        """Текущий режим FSM (для логов и UI)."""
        return self._snapshot.avatar_mode

    def is_held_by(self, client_id: Optional[str]) -> bool:
        """True если ``client_id`` держит teleop_floor прямо сейчас.

        Используется ws_server-ом для гейта ``teleop_twist``: если
        ``client_id`` (наша сессия) держит floor — пропускаем твист;
        если держит кто-то другой — гейтим и шлём FLOOR_HELD.
        """
        return client_id is not None and self._snapshot.teleop_holder == client_id

    def held_by_other(self, client_id: Optional[str]) -> bool:
        """True если floor держит кто-то, но не ``client_id``.

        Разница с ``not is_held_by(client_id)``: возвращает False, если
        floor свободен (нет holder-а). Удобно для различения
        «не твой» vs «никого нет».
        """
        holder = self._snapshot.teleop_holder
        if holder is None:
            return False
        return holder != client_id

    def update(self, snapshot: AvatarFloorSnapshot) -> "FloorViewUpdate":
        """Положить новый снимок из /avatar/state.

        Единственный мутирующий метод класса. Вызывается из
        ws_server-овского подписчика на ``/avatar_state_topic``.

        Возвращает :class:`FloorViewUpdate` — diff между prev и next.
        Это позволяет QuestBridge реагировать на переходы
        «кто-то держал → нас сняли → JSON_EVENT{floor_lost}» без
        хранения prev-состояния снаружи (см. issue #2190 §
        "FloorView.update возвращает, что именно изменилось").

        Idempotent: повторный update с тем же snapshot — diff
        с пустыми флагами (вызывающая сторона его игнорирует).
        """
        prev = self._snapshot
        self._snapshot = snapshot
        return FloorViewUpdate.from_diff(prev=prev, next_snapshot=snapshot)

    def reset(self) -> "FloorViewUpdate":
        """Очистить кэш (используется при reset/shutdown).

        В AvatarStateFloorCache нет «освобождения» floor-а (это
        решает LockManager по факту release-вызова). Здесь —
        только сброс локального зеркала, чтобы в UI/логах после
        reset не висел старый holder. Возвращает diff, чтобы
        вызывающая сторона могла разослать JSON_EVENT{floor_lost}
        прежнему держателю (если он был).
        """
        prev = self._snapshot
        self._snapshot = AvatarFloorSnapshot()
        return FloorViewUpdate.from_diff(prev=prev, next_snapshot=self._snapshot)


@dataclass(frozen=True)
class FloorViewUpdate:
    """Diff, который возвращает :py:meth:`AvatarStateFloorCache.update`.

    Карточка issue #2190 (voice-vr 05): ``FloorView.update`` должен
    возвращать, **что именно** изменилось — иначе QuestBridge не
    может решить, кому слать ``JSON_EVENT{floor_lost}`` (только
    прежнему держателю) и нельзя ли сейчас успокоить UI
    (``teleopLabel: "my" → "other"``).

    Поля:
      - ``prev_teleop_holder`` / ``next_teleop_holder`` — что было и
        что стало (для UI). Оба ``Optional[str]`` (server_client_id
        вида ``"quest:<session_uuid>"`` или ``None``).
      - ``teleop_holder_changed`` — True если prev != next (включая
        None↔X). Удобно для дедупа: вызывающая сторона может
        игнорировать «пустые» обновления.
      - ``teleop_lost`` — True если раньше КТО-ТО держал (prev_holder
        is not None), а теперь НИКТО (next_holder is None). Это
        условие, при котором прежний держатель должен получить
        ``JSON_EVENT{floor_lost}`` (Avatar-arbiter отпустил floor
        целиком — робот не должен продолжать ехать).
      - ``teleop_replaced`` — True если был один holder, стал другой
        (не None). Прежний holder получит ``JSON_EVENT{floor_lost}``
        (его floor забрали).
      - ``new_teleop_holder`` — алиас ``next_teleop_holder`` для
        удобства вызывающей стороны («кто теперь держит?»).

    Используется из QuestBridge.on_avatar_state: идём по diff и
    шлём floor_lost тем WS-сессиям, чей ``server_client_id`` стал
    неактуальным holder-ом (а не «всем подряд»).
    """

    prev_teleop_holder: Optional[str]
    next_teleop_holder: Optional[str]
    teleop_holder_changed: bool
    teleop_lost: bool
    teleop_replaced: bool
    new_teleop_holder: Optional[str] = None

    @classmethod
    def from_diff(
        cls,
        prev: "AvatarFloorSnapshot",
        next_snapshot: "AvatarFloorSnapshot",
    ) -> "FloorViewUpdate":
        prev_h = prev.teleop_holder
        next_h = next_snapshot.teleop_holder
        changed = prev_h != next_h
        lost = changed and prev_h is not None and next_h is None
        replaced = changed and prev_h is not None and next_h is not None
        return cls(
            prev_teleop_holder=prev_h,
            next_teleop_holder=next_h,
            teleop_holder_changed=changed,
            teleop_lost=lost,
            teleop_replaced=replaced,
            new_teleop_holder=next_h,
        )


# === Voice floor (read-only cache) =============================================
# Раньше жил в ``server/voice_floor.py`` вместе с mutex-логикой. После
# ADR-0051 §2.2 muteх ушёл в avatar_arbiter, и ``VoiceFloorCache``
# переехал в ``core/floor.py`` — чтобы ``LocalAvatarArbiterClient``
# (тоже в core/) мог импортировать ``FloorHolder`` без циркулярной
# зависимости через ``server/__init__.py`` → ``ws_server`` →
# ``core.avatar_arbiter`` → ``server.voice_floor``.

class FloorState(str, Enum):
    """Состояния voice floor.

    Значения совпадают со схемой ``voice_state`` в meta-quest-api.md §4,
    кроме ``DENIED`` (локальное расширение для UI Quest-сервера).

    После ADR-0051 §2.2 это read-only enum — мутации состояния
    делает avatar_arbiter; здесь мы только зеркалим.
    """

    IDLE = "idle"
    LISTENING = "listening"
    SPEAKING = "speaking"
    DENIED = "denied"


@dataclass(frozen=True)
class FloorHolder:
    """Идентификатор держателя voice floor (mirror из /avatar/state).

    До ADR-0051 §2.2 holder был «локальной» сущностью Quest-WS
    (создавался на try_acquire). Теперь avatar_arbiter публикует
    фактического holder-а в /avatar_state, и мы просто зеркалим
    сюда для UI-логики Quest-клиента.

    Attributes:
        session_id: id WS-сессии (зеркало из arbiter-а).
        client_id: id клиента из HELLO payload, либо серверный fallback.
    """

    session_id: str
    client_id: str

    def label(self) -> str:
        """Короткая метка для логов и voice_state.detail (≤ ~32 символа)."""
        sid = self.session_id[:8] if len(self.session_id) >= 8 else self.session_id
        return f"{self.client_id}:{sid}"


class VoiceFloorCache:
    """Read-only кэш voice floor-а (ADR-0051 §2.2).

    Single-thread: все вызовы из aiohttp event-loop. Никаких
    блокировок — обновление синхронное, как и в
    :class:`AvatarStateFloorCache`.
    """

    __slots__ = ("_state", "_holder")

    def __init__(
        self,
        initial_state: FloorState = FloorState.IDLE,
        initial_holder: Optional[FloorHolder] = None,
    ) -> None:
        self._state: FloorState = initial_state
        self._holder: Optional[FloorHolder] = initial_holder

    @property
    def state(self) -> FloorState:
        """Текущее состояние voice floor (idle / listening / speaking / denied)."""
        return self._state

    @property
    def holder(self) -> Optional[FloorHolder]:
        """Текущий держатель voice floor (mirror из /avatar_state) или None."""
        return self._holder

    def is_held_by_session(self, session_id: Optional[str]) -> bool:
        """True если ``session_id`` держит voice floor прямо сейчас.

        Используется ws_server-ом для гейта voice_ptt_start/stop и
        VOICE_AUDIO фреймов: пропускаем только если наша сессия —
        текущий holder.
        """
        return (
            session_id is not None
            and self._holder is not None
            and self._holder.session_id == session_id
        )

    def held_by_other_session(self, session_id: Optional[str]) -> bool:
        """True если voice floor держит другая сессия (не наша)."""
        if self._holder is None or self._state == FloorState.IDLE:
            return False
        return self._holder.session_id != session_id

    def update(
        self,
        state: FloorState,
        holder: Optional[FloorHolder],
    ) -> None:
        """Положить новый снимок voice floor-а из /avatar_state.

        Единственный мутирующий метод класса. Вызывается из
        ws_server-овского подписчика на ``/avatar_state_topic``.

        Контракт: avatar_arbiter присылает ``state`` и ``holder``
        вместе; ``DENIED`` (локальное расширение) ставится самим
        ws_server-ом в момент отказа клиенту, чтобы UI мог показать
        «у робота говорит другой» — это единственный путь, где
        ``state != holder``, и обрабатывается он явно в ws_server.
        """
        self._state = state
        self._holder = holder

    def reset(self) -> None:
        """Очистить кэш (используется при reset/shutdown).

        Реальное освобождение voice floor-а — зона LockManager; здесь
        только сброс зеркала.
        """
        self._state = FloorState.IDLE
        self._holder = None


__all__ = [
    "AvatarStateFloorCache",
    "AvatarFloorSnapshot",
    "QUEST_DEFAULT_CLIENT_ID",
    "FloorState",
    "FloorHolder",
    "VoiceFloorCache",
    "FloorViewUpdate",
    "make_server_client_id",
]