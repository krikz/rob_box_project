"""Encounter seam — «Встреча» (issue #2442).

Разграничение со швом идентичности (#2440, ``rob_box_harness.identity``):
:class:`~rob_box_harness.identity.IdentitySeam` отвечает на вопрос «кто» —
сводит три несвязанных ключа биометрии в один стабильный ``Acquaintance.id``.
Этот шов отвечает на другой вопрос — «встретились ли мы **сейчас**»: сводит
независимые, ничего друг о друге не знающие каналы восприятия (голос,
лицо) в одно значение :class:`Encounter` — кто присутствует прямо сейчас,
с какой уверенностью, каким каналом подтверждено, с какого момента длится
непрерывное присутствие и когда виделись до этого (через
``IdentitySeam.since_last_seen``).

Встреча *использует* ``Acquaintance`` как поле «кто», но не переоткрывает
и не дублирует логику слияния идентичности — это работа #2440.

Контракт шва — одна операция чтения и одна операция записи:

* :meth:`EncounterSeam.current` — синхронная, без I/O. Возвращает
  последнюю известную Встречу, или ``None``, если непрерывное присутствие
  истекло по таймауту (``presence_timeout_sec``) или ещё не начиналось.
  Единственная операция, которую видят потребители (``dialogue_node``,
  ``mcp_server``) — оба читают один и тот же шов вместо того, чтобы
  независимо парсить сырой сигнал и держать собственную копию состояния.
* :meth:`EncounterSeam.observe` — асинхронная (обращается к
  ``IdentitySeam.since_last_seen``, который ходит в ``MemoryStore``).
  Вызывается адаптером канала (см. ``voice_adapter.py``) при поступлении
  сырого сигнала присутствия. Два сигнала про одного и того же ``who`` в
  пределах ``presence_timeout_sec`` сливаются в одну Встречу: множество
  каналов объединяется, уверенность — максимум, ``since`` (момент начала
  непрерывного присутствia) не сдвигается.

Шов не знает про ROS, топики или конкретный формат сырого сигнала —
это дело адаптера. Голосовой адаптер живёт в ``voice_adapter.py`` этого же
пакета; зрительный (issue #2531/#2583) — следующий шаг, вне скоупа
первого инкремента (issue #2442, PR-описание).
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import FrozenSet, Optional

from rob_box_harness.identity import Acquaintance, IdentitySeam

#: Сколько секунд без единого сигнала ни по одному каналу считается
#: разрывом непрерывного присутствия. Подобрано по аналогии с voice_floor
#: dead-man (500 мс) на порядок выше — присутствие человека в мастерской
#: не должно "мигать" между репликами диалога.
DEFAULT_PRESENCE_TIMEOUT_SEC = 30.0


class EncounterChannel(enum.Enum):
    """Канал, подтвердивший присутствие человека.

    Пустое множество каналов у :class:`Encounter` означает «есть сигнал
    присутствия (сцена/объект), но личность не подтверждена ни одним
    именованным каналом» — валидное состояние, не ошибка.
    """

    VOICE = "voice"
    FACE = "face"


@dataclass(frozen=True)
class Encounter:
    """«Встреча» — value-объект текущего присутствия человека.

    :param who: :class:`Acquaintance` из шва идентичности (#2440), или
        ``None`` — присутствие без опознания тоже валидная Встреча.
    :param confidence: уверенность в присутствии/опознании — максимум по
        всем сигналам, слитым в эту Встречу.
    :param channels: множество каналов, подтвердивших Встречу. Пустое —
        присутствие без канала опознания личности.
    :param since: unix-timestamp момента начала непрерывного присутствия
        (первый сигнал, с которого началась текущая Встреча).
    :param since_last_seen: секунды с прошлого раза, когда видели этого
        же ``who`` (через ``IdentitySeam.since_last_seen``), посчитанные
        на момент **начала** этой Встречи. ``None``, если ``who is None``
        или это первая встреча (профиля ещё нет).
    """

    who: Optional[Acquaintance]
    confidence: float
    channels: FrozenSet[EncounterChannel]
    since: float
    since_last_seen: Optional[float] = None


def _same_who(a: Optional[Acquaintance], b: Optional[Acquaintance]) -> bool:
    """Тот же человек? Сравнение только по ``id`` (см. Acquaintance).

    ``None == None`` — тоже True: два сигнала без опознанной личности
    трактуются как продолжение той же (анонимной) Встречи, а не как смена
    человека, потому что у шва нет способа отличить одного анонима от
    другого.
    """
    if a is None or b is None:
        return a is b
    return a.id == b.id


class EncounterSeam:
    """Шов «Встреча»: сливает сигналы каналов в одно значение присутствия.

    :param identity: :class:`IdentitySeam` — используется только для
        ``since_last_seen`` (поле «виделись до этого»). Шов не резолвит
        сигналы сам — адаптер обязан отдать уже разрешённого
        ``Acquaintance`` (или ``None``, если сигнал не опознан).
    :param presence_timeout_sec: см. :data:`DEFAULT_PRESENCE_TIMEOUT_SEC`.
    """

    def __init__(
        self,
        identity: IdentitySeam,
        *,
        presence_timeout_sec: float = DEFAULT_PRESENCE_TIMEOUT_SEC,
    ) -> None:
        self._identity = identity
        self._presence_timeout_sec = presence_timeout_sec
        self._encounter: Optional[Encounter] = None
        self._last_activity: Optional[float] = None

    def current(self, *, now: Optional[float] = None) -> Optional[Encounter]:
        """Текущая Встреча, или ``None`` (нет сигналов / истёк таймаут).

        Синхронная, без I/O — потребители (``dialogue_node``,
        ``mcp_server``) читают её на каждом ходу диалога без риска
        заблокироваться на памяти/сети.
        """
        ts = now if now is not None else time.time()
        return self._current_locked(ts)

    def _current_locked(self, ts: float) -> Optional[Encounter]:
        if self._encounter is None or self._last_activity is None:
            return None
        if ts - self._last_activity > self._presence_timeout_sec:
            # Истекло — присутствие прервалось. Следующий observe()
            # откроет новую Встречу, а не продолжит эту (since сдвинется).
            self._encounter = None
            self._last_activity = None
            return None
        return self._encounter

    async def observe(
        self,
        channel: EncounterChannel,
        who: Optional[Acquaintance],
        confidence: float,
        *,
        now: Optional[float] = None,
    ) -> Encounter:
        """Учесть сигнал присутствия от одного канала.

        Если сигнал про того же ``who``, что и текущая (не истёкшая)
        Встреча — сливает: множество каналов объединяется, уверенность —
        максимум входных, ``since`` не сдвигается. Иначе открывает новую
        Встречу (``since = now``) и — если ``who`` опознан — спрашивает
        шов идентичности, когда виделись до этого.

        Возвращает актуальную Встречу (то же, что вернул бы
        последующий :meth:`current`).
        """
        ts = now if now is not None else time.time()
        current = self._current_locked(ts)

        if current is not None and _same_who(current.who, who):
            encounter = Encounter(
                who=who,
                confidence=max(current.confidence, confidence),
                channels=current.channels | {channel},
                since=current.since,
                since_last_seen=current.since_last_seen,
            )
        else:
            since_last_seen = (
                await self._identity.since_last_seen(who, now=ts)
                if who is not None
                else None
            )
            encounter = Encounter(
                who=who,
                confidence=confidence,
                channels=frozenset({channel}),
                since=ts,
                since_last_seen=since_last_seen,
            )

        self._encounter = encounter
        self._last_activity = ts
        return encounter


__all__ = [
    "DEFAULT_PRESENCE_TIMEOUT_SEC",
    "Encounter",
    "EncounterChannel",
    "EncounterSeam",
]
