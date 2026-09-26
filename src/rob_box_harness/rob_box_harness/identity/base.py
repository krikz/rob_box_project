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
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional

from rob_box_harness.memory import (
    MemoryStore,
    get_speaker_profile,
    merge_speaker_facts,
    touch_speaker,
)

# ADR-0135 §2.4 — полосы уверенности для face hint.
# Дефолты из ADR-0123 §6 (high) и ADR-0089 §2.2 (low стаб-зеркало).
DEFAULT_FACE_HINT_HIGH = 0.78
DEFAULT_FACE_HINT_LOW = 0.65
DEFAULT_FACE_HINT_WINDOW_SEC = 30.0
DEFAULT_FACE_HINT_BUFFER_CAPACITY = 8


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


def _classify_face_band(
    similarity: float,
    *,
    high: float = DEFAULT_FACE_HINT_HIGH,
    low: float = DEFAULT_FACE_HINT_LOW,
) -> str:
    """Полоса уверенности face-hint (ADR-0135 §2.4).

    * ``high`` — отменяет голосовой переспрос #2809;
    * ``tentative`` — присутствует в логе/контексте, но НЕ отменяет;
    * ``low`` — игнорируется (стаб-фильтр зеркалит ADR-0089 §2.2).
    """
    if similarity >= high:
        return "high"
    if similarity >= low:
        return "tentative"
    return "low"


@dataclass(frozen=True)
class FaceSignal:
    """Сырой сигнал лица из vision_node → ``IdentitySeam.note_face_seen``.

    Структурно-типизирован (ADR-0135 §2.1): шов не зависит от
    ``rob_box_perception``, dataclass'ы достаточно для сериализации
    через ``/vision/hailo/events`` → ``parse_meeting_marker`` (см.
    ``dialogue_node._on_vision_event``).
    """

    person_id: str       # UUID из FaceStore
    name: Optional[str]  # имя, если FaceStore его уже знает
    similarity: float    # 0..1, score ArcFace-матча
    is_new: bool         # новая запись (created_at == сейчас) или старая
    source_camera: str
    captured_at: float   # unix-time; для freshness-окна


@dataclass(frozen=True)
class FaceObservation:
    """Наблюдение лица, прочитанное через ``IdentitySeam.recent_face_observation``.

    Не пишется в долговременный стор (ADR-0135 §2.2: «наблюдение, а не
    обновление профиля») — это in-memory подсказка для потребителей шва
    (``dialogue_node``) о том, что лицо только что видело кандидата с
    указанной уверенностью.
    """

    person_id: str
    name: Optional[str]
    similarity: float
    captured_at: float
    is_new: bool
    confidence_band: str   # "high" | "tentative" | "low" (см. ADR-0135 §2.4)

    def age_sec(self, *, now: Optional[float] = None) -> float:
        return (now if now is not None else time.time()) - self.captured_at

    def is_recent(
        self,
        *,
        window_sec: float = DEFAULT_FACE_HINT_WINDOW_SEC,
        now: Optional[float] = None,
    ) -> bool:
        """Свежесть в пределах окна ``window_sec`` (дефолт 30с, ADR-0135 §2.4)."""
        return self.age_sec(now=now) <= window_sec


class IdentitySeam(abc.ABC):
    """Шов идентичности: память + (в подклассе) адаптер биометрии.

    :param memory: ``MemoryStore``, в котором хранятся профили и факты
        знакомых (scope ``speaker:<id>``).
    """

    def __init__(self, memory: MemoryStore) -> None:
        self._memory = memory
        # ADR-0135 §2.2 — in-memory кольцевой буфер face-наблюдений per
        # person_id. Ключ = person_id (Vision), потому что человек-человек
        # ещё не сшиты (см. ADR-0123 §6 Phase 2). Буфер живёт только в
        # памяти процесса — это НАБЛЮДЕНИЕ, а не обновление Acquaintance.
        self._face_observations: Dict[str, Deque[FaceObservation]] = {}
        self._face_buffer_capacity: int = DEFAULT_FACE_HINT_BUFFER_CAPACITY
        self._face_high_threshold: float = DEFAULT_FACE_HINT_HIGH
        self._face_low_threshold: float = DEFAULT_FACE_HINT_LOW
        self._face_window_sec: float = DEFAULT_FACE_HINT_WINDOW_SEC

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

    # ------------------------------------------------------------------
    # ADR-0135 §2.2 — FaceSignal → in-memory ring buffer
    # ------------------------------------------------------------------

    def configure_face_hint(
        self,
        *,
        buffer_capacity: int = DEFAULT_FACE_HINT_BUFFER_CAPACITY,
        high_threshold: float = DEFAULT_FACE_HINT_HIGH,
        low_threshold: float = DEFAULT_FACE_HINT_LOW,
        window_sec: float = DEFAULT_FACE_HINT_WINDOW_SEC,
    ) -> None:
        """Задать параметры face-hint (вызывается из конфига ноды).

        Те же дефолты, что в ADR-0135 §2.6. Метод аддитивный — старые
        швы без face-hint продолжают работать (если его не звали,
        буфер пуст и ``recent_face_observation`` всегда возвращает
        ``None``, см. ADR-0135 §2.5 деградация).
        """
        self._face_buffer_capacity = int(buffer_capacity)
        self._face_high_threshold = float(high_threshold)
        self._face_low_threshold = float(low_threshold)
        self._face_window_sec = float(window_sec)

    def note_face_seen(
        self, signal: FaceSignal, *, now: Optional[float] = None
    ) -> FaceObservation:
        """Положить наблюдение лица в кольцевой буфер (ADR-0135 §2.2).

        Синхронная (in-memory), без записи в долговременный стор
        (это НАБЛЮДЕНИЕ, а не обновление ``Acquaintance``: лицо
        остаётся источником подсказки, а не источником ``name``
        — пока владелец не сделает полную arbitration ADR-0123 §6).
        """
        ts = now if now is not None else time.time()
        band = _classify_face_band(
            signal.similarity,
            high=self._face_high_threshold,
            low=self._face_low_threshold,
        )
        obs = FaceObservation(
            person_id=signal.person_id,
            name=signal.name,
            similarity=float(signal.similarity),
            captured_at=float(signal.captured_at or ts),
            is_new=bool(signal.is_new),
            confidence_band=band,
        )
        buf = self._face_observations.setdefault(
            signal.person_id, deque(maxlen=self._face_buffer_capacity)
        )
        buf.append(obs)
        return obs

    def recent_face_observation(
        self,
        person_id: str,
        *,
        window_sec: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Optional[FaceObservation]:
        """Самое свежее наблюдение лица для ``person_id`` в окне.

        Возвращает ``None``, если буфер пуст, ключа нет, или последнее
        наблюдение протухло (старше ``window_sec``). Дефолт окна — из
        ``configure_face_hint`` (ADR-0135 §2.6 дефолт 30с).
        """
        buf = self._face_observations.get(person_id)
        if not buf:
            return None
        win = self._face_window_sec if window_sec is None else float(window_sec)
        ts = now if now is not None else time.time()
        for obs in reversed(buf):
            if (ts - obs.captured_at) <= win:
                return obs
        return None

    def recent_face_observation_by_name(
        self,
        name: str,
        *,
        window_sec: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Optional[FaceObservation]:
        """Самое свежее наблюдение лица С ЗАДАННЫМ ИМЕНЕМ в окне.

        Нужно голосовому потребителю ``_handle_tentative_speaker``:
        voice знает имя кандидата (``tentative_name``), но НЕ знает
        ``person_id`` (Vision-пространство). Сшивка по имени — единственная
        доступная до полной arbitration ADR-0123 §6 (Phase 2). Когда
        arbitration появится, можно будет добавить
        ``recent_face_observation_by_biometric_uuid`` без удаления этого.

        Если ``name`` пустой (``face_store`` ещё не знает имени) —
        возвращает ``None``: голос не может сшить безымянный face-hint
        с именем-кандидатом от speaker_id_node.
        """
        if not name:
            return None
        win = self._face_window_sec if window_sec is None else float(window_sec)
        ts = now if now is not None else time.time()
        best: Optional[FaceObservation] = None
        for buf in self._face_observations.values():
            for obs in reversed(buf):
                if obs.name != name:
                    continue
                if (ts - obs.captured_at) > win:
                    continue
                if best is None or obs.captured_at > best.captured_at:
                    best = obs
                break  # самое свежее для этого person_id уже нашли
        return best


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
    "DEFAULT_FACE_HINT_BUFFER_CAPACITY",
    "DEFAULT_FACE_HINT_HIGH",
    "DEFAULT_FACE_HINT_LOW",
    "DEFAULT_FACE_HINT_WINDOW_SEC",
    "FaceObservation",
    "FaceSignal",
    "IdentitySeam",
    "MemoryIdentitySeam",
]
