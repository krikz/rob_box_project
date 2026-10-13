"""Лимиты кредитов как данные (ADR-0158 §2.6).

Значения — дефолты; решение по В12 за товарищем Шифу (ADR §8): §2.6 говорит
120/120, §8 предлагает 240/120. Заложено консервативное; менять только здесь.
Позже лимиты переедут в ``/data/music_source_state.json`` (PR-5).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Optional

from .library import GeneratedMusicLibrary
from .provider import GenUnavailable

DAILY_CREDITS_DEFAULT = 120.0   # ≈ 10 задач в сутки
SET_CREDITS_DEFAULT = 120.0     # 10 задач сета (В5)


@dataclass(frozen=True)
class BudgetPolicy:
    daily_credits: float = DAILY_CREDITS_DEFAULT
    set_credits: float = SET_CREDITS_DEFAULT


class Budget:
    """Проверка «хватит ли кредитов» по факту из ``gen_ledger``, а не по вере."""

    def __init__(
        self,
        library: GeneratedMusicLibrary,
        policy: BudgetPolicy = BudgetPolicy(),
        *,
        now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    ) -> None:
        self._lib = library
        self.policy = policy
        self._now = now

    def _day_start_iso(self) -> str:
        n = self._now()
        return n.replace(hour=0, minute=0, second=0, microsecond=0).isoformat()

    def spent_today(self) -> float:
        return self._lib.ledger.spent_since(self._day_start_iso())

    def remaining_today(self) -> float:
        return max(0.0, self.policy.daily_credits - self.spent_today())

    def check(self, cost: float, *, set_spent: Optional[float] = None) -> None:
        """``GenUnavailable(budget_exhausted)``, если задача не влезает в день/сет."""
        if cost > self.remaining_today():
            raise GenUnavailable(
                "budget_exhausted",
                f"дневной лимит: нужно {cost:g}, осталось {self.remaining_today():g}",
            )
        if set_spent is not None and set_spent + cost > self.policy.set_credits:
            raise GenUnavailable(
                "budget_exhausted",
                f"лимит сета: потрачено {set_spent:g} + {cost:g} > {self.policy.set_credits:g}",
            )
