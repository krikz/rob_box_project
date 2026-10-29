"""Последний сочинённый трек - единственное место знания «какая песня - эта» (#4216, ADR-0148).

Заказ генерации даёт два варианта в кэш, играет один (песня, инструментал). Вчерашние версии с тем же названием лежат
рядом, и модель, выбирая по названию из списка, брала их (n707/n708 марафона-5 29.10). Генерация отмечает здесь тот
вариант, что сыграла; ``music_library`` читает отсюда, а не угадывает.
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Tuple

#: Сколько секунд только что сочинённый трек остаётся «этой песней»: «расскажи про неё», «включи с начала» через минуты
#: после ``[song] готова`` - про него, а не про старую версию с тем же названием.
GENERATED_FRESH_S = 900.0


class LastGenerated:
    """Id последнего сочинённого трека и когда; ``exists`` - есть ли трек ещё в кэше (стёртый «этой песней» не бывает)."""

    def __init__(self, exists: Callable[[str], bool], clock: Callable[[], float] = time.monotonic) -> None:
        self._exists, self._clock = exists, clock
        self._noted: Optional[Tuple[str, float]] = None

    def note(self, track_id: str) -> None:
        """Запомнить трек, который генерация только что сыграла."""
        if track_id:
            self._noted = (str(track_id), self._clock())

    def fresh(self, max_age_s: float = GENERATED_FRESH_S) -> Optional[str]:
        """Id последнего сочинённого трека, если он не старше ``max_age_s`` и ещё в кэше, иначе ``None``."""
        noted = self._noted
        if noted is None or self._clock() - noted[1] > max_age_s:
            return None
        return noted[0] if self._exists(noted[0]) else None


__all__ = ["GENERATED_FRESH_S", "LastGenerated"]
