"""Срок давности окна диалога: код решает, когда прошлые ходы уже не относятся к разговору (ADR-0148).

Окно ``TurnLog`` режется по числу ходов и токенам, но не по времени: ходы вчерашней e2e-роли доживали до утра, и робот
называл человека её именем. Окно сбрасывается кодом в двух случаях: долгая пауза между адресованными ходами
(:class:`WindowGuard`) и смена режима e2e в любую сторону (``reason`` ``e2e_on``/``e2e_off``, зовёт хост).
Сброс тот же, что у «новой сессии»: ``session.reset()`` + ``log.reset()``; журнал смены и память фактов не трогаются.
"""
from __future__ import annotations

import logging
from typing import Any, Optional

from . import knowledge as kn

REASON_IDLE = "idle"
REASON_E2E_ON = "e2e_on"
REASON_E2E_OFF = "e2e_off"


class WindowGuard:
    """Когда был последний адресованный ход; ``idle_s`` секунд без них - окно устарело."""

    def __init__(self, idle_s: float = kn.DIALOG_WINDOW_IDLE_RESET_S) -> None:
        self._idle_s = float(idle_s)
        self._last: Optional[float] = None

    def gap(self, now: float) -> Optional[float]:
        """Секунд с последнего хода; ходов не было - ``None``."""
        return None if self._last is None else max(0.0, now - self._last)

    def expired(self, now: float) -> bool:
        gap = self.gap(now)
        return gap is not None and gap > self._idle_s

    def touch(self, now: float) -> None:
        self._last = now

    def clear(self) -> None:
        self._last = None


def reset_window(session: Any, log: Any, guard: WindowGuard, reason: str, now: float,
                 logger: logging.Logger) -> None:
    """Сбросить окно: сессия + журнал ходов; пауза отсчитывается заново."""
    gap = guard.gap(now)
    logger.info(f"🧹 [window-reset] reason={reason} gap_s={'-' if gap is None else f'{gap:.0f}'}")
    session.reset()
    log.reset()
    guard.clear()


def reset_if_idle(session: Any, log: Any, guard: WindowGuard, now: float, logger: logging.Logger) -> bool:
    """Начало адресованного хода: пауза длиннее порога - окно сброшено. ``True`` - сброс был."""
    if not guard.expired(now):
        return False
    reset_window(session, log, guard, REASON_IDLE, now, logger)
    return True
