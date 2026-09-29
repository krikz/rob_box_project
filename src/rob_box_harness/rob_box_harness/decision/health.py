"""Circuit breaker и метрики decision layer (issue #3084).

Circuit breaker — чтобы при лежащем Jev/Laya не платить дедлайн на каждом
решении: после ``failure_threshold`` подряд неудач провайдер пропускается
до ``reset_after_s``, затем пропускается одна пробная попытка (half-open).

Метрики — простые счётчики в памяти, без зависимостей. Метки исходов
совпадают с acceptance criteria issue: ``<provider>_success``,
``<provider>_timeout``, ``<provider>_error``, ``<provider>_invalid``,
плюс ``low_confidence``/``circuit_open`` и общий ``fallback_used``.
Ни ключи, ни state в метрики не пишутся.
"""

from __future__ import annotations

import threading
import time
from collections import Counter
from dataclasses import dataclass
from enum import Enum
from typing import Callable


class CircuitState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"


class CircuitBreaker:
    """Счётчик подряд идущих неудач с таймаутом восстановления."""

    def __init__(
        self,
        *,
        failure_threshold: int = 3,
        reset_after_s: float = 30.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if failure_threshold < 1:
            raise ValueError("failure_threshold must be >= 1")
        self._threshold = failure_threshold
        self._reset_after_s = reset_after_s
        self._clock = clock
        self._failures = 0
        self._opened_at: float | None = None
        self._lock = threading.Lock()

    @property
    def state(self) -> CircuitState:
        with self._lock:
            return self._state_locked()

    def _state_locked(self) -> CircuitState:
        if self._opened_at is None:
            return CircuitState.CLOSED
        if self._clock() - self._opened_at >= self._reset_after_s:
            return CircuitState.HALF_OPEN
        return CircuitState.OPEN

    def allow(self) -> bool:
        """Можно ли сейчас звать провайдера (CLOSED или пробный HALF_OPEN)."""
        with self._lock:
            return self._state_locked() is not CircuitState.OPEN

    def record_success(self) -> None:
        with self._lock:
            self._failures = 0
            self._opened_at = None

    def record_failure(self) -> None:
        with self._lock:
            self._failures += 1
            half_open = self._state_locked() is CircuitState.HALF_OPEN
            if half_open or self._failures >= self._threshold:
                self._opened_at = self._clock()


@dataclass(frozen=True)
class LatencySummary:
    count: int
    p50_ms: float | None
    p95_ms: float | None


class DecisionMetrics:
    """Потокобезопасные счётчики исходов и латентности по провайдерам."""

    #: Ограничение памяти на выборку латентностей одного провайдера.
    MAX_SAMPLES = 2048

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counters: Counter[str] = Counter()
        self._latencies: dict[str, list[float]] = {}

    def count(self, label: str) -> None:
        with self._lock:
            self._counters[label] += 1

    def observe_latency(self, provider: str, latency_ms: float) -> None:
        with self._lock:
            samples = self._latencies.setdefault(provider, [])
            samples.append(latency_ms)
            if len(samples) > self.MAX_SAMPLES:
                del samples[0]

    def counters(self) -> dict[str, int]:
        with self._lock:
            return dict(self._counters)

    def latency(self, provider: str) -> LatencySummary:
        with self._lock:
            samples = sorted(self._latencies.get(provider, ()))
        if not samples:
            return LatencySummary(0, None, None)
        return LatencySummary(len(samples), _percentile(samples, 50), _percentile(samples, 95))


def _percentile(sorted_samples: list[float], pct: int) -> float:
    """Nearest-rank перцентиль по уже отсортированной выборке."""
    rank = max(1, -(-pct * len(sorted_samples) // 100))
    return sorted_samples[rank - 1]
