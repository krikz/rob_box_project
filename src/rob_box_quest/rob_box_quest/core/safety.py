"""Safety Watchdog — детектор «клиент пропал» для safe-stop.

Источник истины: docs/adr/0027 §3.3 (dead-man, watchdog, emergency B),
docs/architecture/meta-quest-api.md §7 (heartbeat/ping).

Edge-triggered + ARMED/DISARMED FSM:
  - DISARMED (по умолчанию): ни одного клиента не было → trip=False.
    Убирает спам "EMERGENCY STOP" в первые 100мс после старта ноды
    и блокирует ложные срабатывания пока клиент вообще не подключался.
  - ARMED (после первого feed()): trip=True ОДИН раз при превышении
    timeout, затем False до следующего feed(). Edge через consume_trip().
  - reset() → DISARMED: клиент ушёл, до следующего HELLO не триггерим.

Используется в quest_node.py для публикации cmd_vel_emergency при
тишине > TIMEOUT_S после того как клиент реально был активен.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


WATCHDOG_TIMEOUT_S: float = 0.5  # 500 мс без активности → tripped


@dataclass
class Watchdog:
    """Edge-triggered watchdog с ARMED/DISARMED состояниями.

    DISARMED → trip = False (никогда не триггерим без активного клиента).
    ARMED    → trip = True ОДИН раз при превышении timeout, затем False
               до следующего feed(). Дедуп через _trip_consumed.
    """

    _last_feed_monotonic: Optional[float] = None
    _armed: bool = False
    _trip_consumed: bool = False
    timeout_s: float = WATCHDOG_TIMEOUT_S

    def feed(self, now_monotonic: float) -> None:
        """Клиент жив: ARMED + новый цикл + снять edge-флаг."""
        self._last_feed_monotonic = now_monotonic
        self._armed = True
        self._trip_consumed = False

    def tripped(self, now_monotonic: float) -> bool:
        """Состояние без побочных эффектами (для тестов/диагностики)."""
        if not self._armed:
            return False
        if self._last_feed_monotonic is None:
            return False
        return (now_monotonic - self._last_feed_monotonic) > self.timeout_s

    def consume_trip(self, now_monotonic: float) -> bool:
        """Edge-triggered: True только ОДИН раз на trip, потом False
        до следующего feed(). Используется timer'ом 10 Гц чтобы
        однократно триггернуть emergency flow."""
        if not self.tripped(now_monotonic):
            return False
        if self._trip_consumed:
            return False
        self._trip_consumed = True
        return True

    def reset(self) -> None:
        """DISARMED: до следующего feed() никаких trip'ов."""
        self._last_feed_monotonic = None
        self._armed = False
        self._trip_consumed = False

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def last_feed_monotonic(self) -> Optional[float]:
        return self._last_feed_monotonic
