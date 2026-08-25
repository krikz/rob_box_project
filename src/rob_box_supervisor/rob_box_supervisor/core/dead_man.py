"""DeadManCounter — счётчик ``dead_man_trips_total{client_id}`` (AV-6, ADR-0028 §6 Q4).

Phase 1 (AV-6) только собирает метрику. Полный heartbeat-watcher с
500 ms порогом и снятием teleop_floor (ADR-0028 §4.4 S10) появится в
Phase 2 вместе с FSM/LockManager. Здесь — переносимый счётчик,
которым удобно тестировать policy без ROS 2.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Dict, Optional


class DeadManCounter:
    """Простой счётчик трипов dead-man per-client_id (Phase 1).

    API вынесен в отдельный класс чтобы :class:`StateAggregator` мог
    делегировать ему хранение и чтобы Phase 2 легко добавил policy
    (окно наблюдения, reset, экспорт в Prometheus) без правки aggregator.
    """

    def __init__(self) -> None:
        self._counts: Dict[str, int] = defaultdict(int)

    def trip(self, client_id: str) -> int:
        """Зарегистрировать трип и вернуть новое значение."""
        cid = str(client_id)
        self._counts[cid] += 1
        return self._counts[cid]

    def count(self, client_id: str) -> int:
        """Текущее значение (0 если ещё не было)."""
        return self._counts.get(str(client_id), 0)

    def snapshot(self) -> Dict[str, int]:
        """Копия всего словаря (для публикации в ``/avatar/state``).

        Имя ``snapshot`` вместо ``all`` чтобы не затенять built-in
        ``all`` (flake8 A003 — критично для unit-тестов в Phase 2,
        когда счётчик будут экспортировать в Prometheus/msgpack).
        """
        return dict(self._counts)

    def reset(self, client_id: Optional[str] = None) -> None:
        """Сбросить счётчик (полезно для тестов; не вызывается в проде Phase 1)."""
        if client_id is None:
            self._counts.clear()
        else:
            self._counts.pop(str(client_id), None)
