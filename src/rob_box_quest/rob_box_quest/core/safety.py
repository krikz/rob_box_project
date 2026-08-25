"""Safety Watchdog — детектор «клиент пропал» для safe-stop.

Источник истины: docs/adr/0027 §3.3 (dead-man, watchdog, emergency B),
docs/architecture/meta-quest-api.md §7 (heartbeat/ping).

Чистая логика: feed(now) — клиент жив; tripped — нет. Используется в
quest_node.py для публикации cmd_vel_emergency при тишине > TIMEOUT_S.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


WATCHDOG_TIMEOUT_S: float = 0.5  # 500 мс без активности → tripped


@dataclass
class Watchdog:
    """Простой watchdog на временной интервал.

    feed(now) сбрасывает счётчик.
    tripped() True если прошло > WATCHDOG_TIMEOUT_S с последнего feed.
    reset() — вернуть в активное состояние (после emergency clear).
    """

    _last_feed_monotonic: Optional[float] = None
    timeout_s: float = WATCHDOG_TIMEOUT_S

    def feed(self, now_monotonic: float) -> None:
        self._last_feed_monotonic = now_monotonic

    def tripped(self, now_monotonic: float) -> bool:
        if self._last_feed_monotonic is None:
            return True  # ни разу не feed → считаем мёртвым
        return (now_monotonic - self._last_feed_monotonic) > self.timeout_s

    def reset(self) -> None:
        self._last_feed_monotonic = None

    @property
    def last_feed_monotonic(self) -> Optional[float]:
        return self._last_feed_monotonic
