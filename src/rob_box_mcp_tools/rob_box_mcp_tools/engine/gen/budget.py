"""Лимиты кредитов как данные (ADR-0158 §2.6).

Значения — дефолты; В12 (ADR §8): 14.10 Шифу на время тестов поднял до 600/день (UTC), 180/сет.
Лимит не снят совсем — это защита от зацикливания трат. Менять только здесь.
Единственный источник лимитов; их читает ``engine.source_route.day_left`` (остаток дня — из ``gen_ledger``).
Переключателя режима нет (решение Шифу 13.10): путь выбирает чойсер по кандидатам кода.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Optional

from .library import GeneratedMusicLibrary
from .provider import GenUnavailable

DAILY_CREDITS_DEFAULT = 600.0   # ≈ 50 задач в сутки (на время тестов, В12)
SET_CREDITS_DEFAULT = 180.0     # ≈ 15 задач сета (сет 10 минут ≈ 5 задач)


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
