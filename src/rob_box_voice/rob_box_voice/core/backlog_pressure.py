#!/usr/bin/env python3
"""BacklogPressure — sliding-window tracker for ``no_wake_word`` fragment rate.

Issue #1668 (live 25.08.2026) — в комнате робота 10.1.1.21 непрерывно играет
чужой голос (видео/радио: обзоры мини-кухонь, лестниц, резисторов RX24 и т.п.).
STT распознаёт ~16 фраз/мин и кладёт их в ``no_wake_word`` backlog
(:class:`rob_box_voice.core.speech_accumulator.SpeechAccumulator`). Когда e2e
проигрывает синтез-команду «Робот, ...», wake-gate на равных обрабатывает и
фоновый голос, и синтез → синтез-команда теряется.

Этот модуль даёт wake-gate давление «сколько фонового шума сейчас идёт»:

* :class:`BacklogPressure` — pure-Python counter с скользящим окном, считает
  ``events_per_minute`` (events / (window_sec/60)).
* :class:`BacklogPressureLevel` — enum ``LOW / ELEVATED / HIGH``, вычисляется
  по порогам ``low_above`` и ``high_above``.
* :meth:`BacklogPressure.should_suppress_publish` — **fairness gate**: при
  HIGH wake-gate НЕ добавляет новые ``no_wake_word`` в backlog (но всё равно
  инкрементит счётчик для телеметрии). LLM-bound backlog остаётся приоритетным
  путём для синтез-команд.

Дизайн:

* Чистый модуль без I/O/ROS2 — дёшево тестируется unit-тестами.
* ``window_sec`` по умолчанию 60с — давление считается за последнюю минуту.
* Пороги ``low_above=5.0``, ``high_above=10.0`` соответствуют наблюдаемым
  значениям (≈16/мин на зашумлённом роботе, ~0-1/мин в тишине).
* ``publish_min_interval_sec=1.0`` ограничивает частоту публикации
  диагностического топика 1 Hz (acceptance criteria).

История:

* Issue #1668 (root) — STT-регрессия 16 e2e раундов подряд FAIL.
* ADR-0027 (systemic wake-gate no_wake_word blocker) — описывает observed behavior.
* ADR-0029 (cold-start known-state) — после merge voice-pipeline первые 2-3
  раунда могут фейлить wake-gate, но это не flaky acceptance.

Что НЕ делаем (по требованию issue #1668):

* НЕ отключаем STT / wake-gate полностью.
* НЕ удаляем no_wake_word backlog полностью — только подавляем публикацию.
* НЕ меняем AEC/audio pipeline.
"""

from __future__ import annotations

import time
from collections import deque
from enum import Enum
from typing import Deque, Optional


DEFAULT_WINDOW_SEC = 60.0
DEFAULT_LOW_ABOVE_PER_MIN = 5.0
DEFAULT_HIGH_ABOVE_PER_MIN = 10.0
DEFAULT_PUBLISH_MIN_INTERVAL_SEC = 1.0


class BacklogPressureLevel(str, Enum):
    """Уровень давления на backlog no_wake_word.

    * ``LOW`` — мало фонового шума, backlog публикуется нормально.
    * ``ELEVATED`` — фоновый шум растёт, но backlog ещё публикуется.
    * ``HIGH`` — фоновый шум забивает wake-gate, fairness подавляет
      публикацию (но wake-gate остаётся открытым).
    """

    LOW = "low"
    ELEVATED = "elevated"
    HIGH = "high"


def level_from_rate(
    rate_per_min: float,
    low_above: float,
    high_above: float,
) -> BacklogPressureLevel:
    """Преобразовать ``events/min`` в :class:`BacklogPressureLevel`.

    * ``rate < low_above`` → LOW
    * ``low_above <= rate < high_above`` → ELEVATED
    * ``rate >= high_above`` → HIGH

    Параметры:

    * ``low_above`` должен быть ``<= high_above``. Иначе функция возвращает
      HIGH, если ``rate >= low_above``, и ELEVATED при ``rate < low_above``
      (защита от невалидного конфига).
    """
    if rate_per_min >= high_above:
        return BacklogPressureLevel.HIGH
    if rate_per_min >= low_above:
        return BacklogPressureLevel.ELEVATED
    return BacklogPressureLevel.LOW


class BacklogPressure:
    """Скользящее окно «сколько ``no_wake_word`` событий за последнюю минуту».

    Использование в :class:`DialogueNode`:

    .. code-block:: python

        self._backlog_pressure = BacklogPressure()
        # в _on_stt, ветка no_wake_word:
        level = self._backlog_pressure.record()
        if self._backlog_pressure.should_suppress_publish(level):
            self._llm_skipped_counter["no_wake_word"] += 1
            # ... лог
        else:
            self._speech_accumulator.add(...)
        # периодически (≤1 Hz):
        snapshot = self._backlog_pressure.snapshot(now=time.monotonic())
        self._backlog_overflow_pub.publish(snapshot.to_json())

    Thread-safety: методы ``record`` / ``snapshot`` не блокируют (deque
    append/pop — атомарны под GIL). Если DialogueNode одновременно пишет и
    читает — ок, иначе нужен внешний ``threading.Lock``.
    """

    def __init__(
        self,
        window_sec: float = DEFAULT_WINDOW_SEC,
        low_above: float = DEFAULT_LOW_ABOVE_PER_MIN,
        high_above: float = DEFAULT_HIGH_ABOVE_PER_MIN,
        publish_min_interval_sec: float = DEFAULT_PUBLISH_MIN_INTERVAL_SEC,
        clock=time.monotonic,
    ) -> None:
        if window_sec <= 0:
            raise ValueError(f"window_sec must be > 0, got {window_sec}")
        if low_above < 0 or high_above < 0:
            raise ValueError(
                f"thresholds must be >= 0, got low_above={low_above} "
                f"high_above={high_above}"
            )
        if low_above > high_above:
            raise ValueError(
                f"low_above ({low_above}) must be <= high_above ({high_above})"
            )
        if publish_min_interval_sec < 0:
            raise ValueError(
                f"publish_min_interval_sec must be >= 0, "
                f"got {publish_min_interval_sec}"
            )
        self.window_sec = window_sec
        self.low_above = low_above
        self.high_above = high_above
        self.publish_min_interval_sec = publish_min_interval_sec
        self._clock = clock
        # События хранятся как monotonic timestamps.
        self._events: Deque[float] = deque()
        # Всего событий с момента инициализации (включая отброшенные).
        self._total: int = 0
        # Сколько событий было suppressed (для телеметрии).
        self._suppressed: int = 0
        # Последний snapshot-publish (monotonic). None = ещё не публиковали.
        self._last_publish: Optional[float] = None

    # ------------------------------------------------------------------
    # Запись событий
    # ------------------------------------------------------------------

    def record(self, now: Optional[float] = None) -> BacklogPressureLevel:
        """Зарегистрировать ``no_wake_word`` событие. Возвращает текущий уровень.

        Параметры:

        * ``now`` — монотонная временная метка (``time.monotonic()``).
          По умолчанию ``self._clock()`` для тестируемости.

        Возвращает:

        Уровень давления ПОСЛЕ записи события (т.е. с учётом только что
        добавленного события).
        """
        ts = self._clock() if now is None else now
        self._events.append(ts)
        self._total += 1
        self._prune(ts)
        return self.level(now=ts)

    def should_suppress_publish(self, level: BacklogPressureLevel) -> bool:
        """Fairness gate: подавлять ли публикацию ``no_wake_word`` в backlog.

        Правила:

        * ``HIGH`` → подавляем (не добавляем в SpeechAccumulator).
        * ``ELEVATED`` / ``LOW`` → публикуем.

        Возвращает ``True``, если wake-gate должен **подавить** добавление
        в backlog (но wake-gate остаётся открытым — STT не выключается).
        """
        return level is BacklogPressureLevel.HIGH

    def mark_suppressed(self) -> None:
        """Засчитать suppressed-событие для телеметрии.

        Вызывать после ``should_suppress_publish(level) == True`` —
        инкрементирует счётчик ``_suppressed`` (в дополнение к ``_total``,
        который растёт на каждый ``record``).
        """
        self._suppressed += 1

    # ------------------------------------------------------------------
    # Чтение / публикация
    # ------------------------------------------------------------------

    def level(self, now: Optional[float] = None) -> BacklogPressureLevel:
        """Текущий уровень давления (без записи нового события)."""
        ts = self._clock() if now is None else now
        self._prune(ts)
        return level_from_rate(
            self.events_per_minute(now=ts),
            low_above=self.low_above,
            high_above=self.high_above,
        )

    def events_per_minute(self, now: Optional[float] = None) -> float:
        """Сколько событий приходилось в минуту за последнее ``window_sec``.

        Нормализация: ``count / (window_sec / 60)``. При ``window_sec=60``
        результат = количество событий (events/min). При пустом окне = 0.
        """
        ts = self._clock() if now is None else now
        self._prune(ts)
        if self.window_sec <= 0:
            return 0.0
        per_min = self.window_sec / 60.0
        if per_min <= 0:
            return 0.0
        return len(self._events) / per_min

    def should_publish(self, now: Optional[float] = None) -> bool:
        """Прошло ли ``publish_min_interval_sec`` с последнего ``mark_published``?"""
        ts = self._clock() if now is None else now
        if self.publish_min_interval_sec <= 0:
            return True
        if self._last_publish is None:
            return True
        return (ts - self._last_publish) >= self.publish_min_interval_sec

    def mark_published(self, now: Optional[float] = None) -> None:
        """Засчитать, что snapshot был опубликован сейчас."""
        self._last_publish = self._clock() if now is None else now

    def snapshot(self, now: Optional[float] = None) -> "BacklogPressureSnapshot":
        """Сериализуемый snapshot текущего состояния."""
        ts = self._clock() if now is None else now
        self._prune(ts)
        return BacklogPressureSnapshot(
            events_per_minute=self.events_per_minute(now=ts),
            window_events=len(self._events),
            total_events=self._total,
            suppressed_events=self._suppressed,
            level=self.level(now=ts).value,
            window_sec=self.window_sec,
            low_above=self.low_above,
            high_above=self.high_above,
        )

    def is_quiet(self, now: Optional[float] = None) -> bool:
        """«Робот сейчас в тишине?» — для self-test / e2e pre-flight.

        True = текущее давление LOW и за последнее окно было ≤ 1 событие.
        Жёстче, чем ``level() == LOW``: даже LOW-уровень с одним-двумя
        событиями считаем «почти тишина», но если событий >1 — мы ещё не
        «в тишине», это «почти тишина с редкими фразами».
        """
        ts = self._clock() if now is None else now
        self._prune(ts)
        return len(self._events) == 0 or (
            len(self._events) <= 1 and self.events_per_minute(now=ts) < self.low_above
        )

    # ------------------------------------------------------------------
    # Служебное
    # ------------------------------------------------------------------

    def _prune(self, now: float) -> None:
        """Удалить события старше ``now - window_sec``."""
        cutoff = now - self.window_sec
        while self._events and self._events[0] < cutoff:
            self._events.popleft()

    def reset(self) -> None:
        """Полный сброс (для тестов и админ-команд)."""
        self._events.clear()
        self._total = 0
        self._suppressed = 0
        self._last_publish = None


class BacklogPressureSnapshot:
    """Сериализуемое состояние :class:`BacklogPressure`.

    Используется для публикации в ``/diagnostics/backlog_overflow`` (≤ 1 Hz)
    и для self-test ``/voice/diagnostics/quiet`` (Bool).
    """

    __slots__ = (
        "events_per_minute",
        "window_events",
        "total_events",
        "suppressed_events",
        "level",
        "window_sec",
        "low_above",
        "high_above",
    )

    def __init__(
        self,
        events_per_minute: float,
        window_events: int,
        total_events: int,
        suppressed_events: int,
        level: str,
        window_sec: float,
        low_above: float,
        high_above: float,
    ) -> None:
        self.events_per_minute = events_per_minute
        self.window_events = window_events
        self.total_events = total_events
        self.suppressed_events = suppressed_events
        self.level = level
        self.window_sec = window_sec
        self.low_above = low_above
        self.high_above = high_above

    def to_json(self) -> str:
        """JSON-сериализация для ROS2 topic (String).

        Используется в ``/diagnostics/backlog_overflow``. Поля стабильные
        (sort_keys=True) для удобства логирования.
        """
        import json

        return json.dumps(
            {
                "level": self.level,
                "events_per_minute": round(self.events_per_minute, 3),
                "window_events": self.window_events,
                "total_events": self.total_events,
                "suppressed_events": self.suppressed_events,
                "window_sec": self.window_sec,
                "low_above": self.low_above,
                "high_above": self.high_above,
            },
            sort_keys=True,
            ensure_ascii=False,
        )

    def __repr__(self) -> str:  # pragma: no cover — debug helper
        return (
            f"BacklogPressureSnapshot(level={self.level!r}, "
            f"events_per_minute={self.events_per_minute:.2f}, "
            f"window_events={self.window_events}, "
            f"total={self.total_events}, suppressed={self.suppressed_events})"
        )
