"""Identity seam — «Знакомый» (issue #2440).

Единый шов идентичности человека взамен трёх несвязанных ключей, которые
сейчас гуляют по стеку:

1. Биометрический UUID голоса (``speaker_id`` в ``/data/speakers.db``);
2. Yandex ``speaker_tag`` («0», «1», …) — per-session, не стабилен между
   сессиями;
3. ``voice_memory.speaker_id`` — полный UUID в слое памяти.

До этого шва каждая подсистема работала со «своим» представлением
человека, и они не совпадали: диаризация подтверждала реплику по
``speaker_tag``, а факты писались под ``speaker_scope(tag)`` — через одну
сессию этот scope уже ничего не значил (дефект C в issue #2440).

Контракт шва (три операции + merge):

* :meth:`IdentitySeam.resolve` — превратить сырой сигнал биометрии
  (голосовой/лицевой эмбеддинг) в :class:`Acquaintance` со СТАБИЛЬНЫМ
  ``id``. Каждый адаптер сам знает, что такое его «сигнал»: голосовой
  адаптер ждёт d-vector resemblyzer, лицевой (ADR-0089 Phase 2) —
  128-dim эмбеддинг arcface. Шов не знает ни про resemblyzer, ни про
  Yandex tag, ни про конкретную БД.
* :meth:`IdentitySeam.note_seen` — обновить ``last_seen``/``dialog_count``
  для знакомого. Ключ памяти — ``speaker_scope(person.id)``, то есть
  стабильный биометрический id, а не per-session tag.
* :meth:`IdentitySeam.since_last_seen` — сколько секунд прошло с прошлого
  появления (``None``, если профиля нет).
* :meth:`IdentitySeam.merge` — склеить две записи одного человека:
  адаптер биометрии переносит эмбеддинги, памятный слой — факты профиля.
  Обе операции работают в одном пространстве id, поэтому ручного
  маппинга tag↔uuid больше не нужно.

``resolve`` — синхронный (инференс биометрии), ``note_seen``/
``since_last_seen``/``merge`` — асинхронные (ходят в async ``MemoryStore``).
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass
from typing import Any

from rob_box_harness.memory import (
    MemoryStore,
    get_speaker_profile,
    merge_speaker_facts,
    touch_speaker,
)


@dataclass(frozen=True)
class Acquaintance:
    """«Знакомый» — value-объект идентичности человека.

    Единственное обязательное поле — ``id``: стабильный ключ, под которым
    в системе уже сведены биометрический UUID, имя и эпитет. Остальные
    поля — удобная метаинформация для логов и LLM-контекста; шов на неё
    не полагается при сравнении (равенство — только по ``id``).
    """

    id: str
    name: str | None = None
    epithet: str | None = None
    confidence: float | None = None


class IdentitySeam(abc.ABC):
    """Шов идентичности: память + (в подклассе) адаптер биометрии.

    :param memory: ``MemoryStore``, в котором хранятся профили и факты
        знакомых (scope ``speaker:<id>``).
    """

    def __init__(self, memory: MemoryStore) -> None:
        self._memory = memory

    @abc.abstractmethod
    def resolve(self, signal: Any) -> Acquaintance | None:
        """Разрешить сырой биометрический сигнал в знакомого.

        Возвращает ``None``, если сигнал не опознан (нет стабильного id).
        """
        raise NotImplementedError

    async def note_seen(
        self, person: Acquaintance, *, now: float | None = None
    ) -> dict:
        """Обновить ``last_seen``/``dialog_count`` знакомого (создаёт при первом).

        Возвращает актуальный профиль. Преемник ``touch_speaker``: ключ —
        ``person.id`` (стабильный биометрический id), а не Yandex tag.
        """
        return await touch_speaker(self._memory, person.id, now=now)

    async def since_last_seen(
        self, person: Acquaintance, *, now: float | None = None
    ) -> float | None:
        """Вернуть секунды с прошлого ``note_seen``, или ``None``.

        ``None`` означает «профиля нет / ещё не видели» — вызывающий код
        не должен трактовать его как «только что видел» (0 секунд).
        """
        profile = await get_speaker_profile(self._memory, person.id)
        if not profile:
            return None
        last_seen = profile.get("last_seen")
        if not last_seen:
            return None
        ts = now if now is not None else time.time()
        return ts - float(last_seen)

    async def merge(self, src_id: str, dst_id: str) -> tuple[int, int]:
        """Склеить две записи одного человека.

        Базовый шов переносит только факты памятного слоя и возвращает
        ``(embeddings_moved, facts_moved)``, где ``embeddings_moved`` у
        базового шва всегда 0 — за перенос биометрии отвечает адаптер,
        переопределяющий этот метод (см. ``VoiceIdentitySeam.merge``).
        """
        facts_moved = await merge_speaker_facts(self._memory, src_id, dst_id)
        return 0, facts_moved


class MemoryIdentitySeam(IdentitySeam):
    """Памятный шов без биометрии.

    Для узлов, которые получают уже разрешённый id извне (например,
    ``dialogue_node`` читает результат ``speaker_id_node`` из топика) и
    нуждаются только в ``note_seen``/``since_last_seen``/``merge``.
    """

    def resolve(self, signal: Any) -> Acquaintance | None:
        raise NotImplementedError(
            "MemoryIdentitySeam не разрешает биометрические сигналы — "
            "используйте адаптер биометрии (VoiceIdentitySeam / face adapter)"
        )


__all__ = [
    "Acquaintance",
    "IdentitySeam",
    "MemoryIdentitySeam",
]
