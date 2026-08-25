"""Avatar Supervisor core helpers (FSM + aggregator + dead-man counter).

Pure-Python координация (FSM, lock manager, aggregator, dead-man counter):
эти модули живут **без ROS/asyncio/threading**, чтобы их легко было
покрыть TDD (см. ``test/unit/core/``). Связь с ROS 2 — через
``supervisor_node.py`` (Phase 1 monitor, AV-6).

Phase 1 (monitor, AV-6) добавил:
- :class:`StateAggregator` — собирает ``/avatar/state`` из входов
  (``/odom``, ``/device/snapshot``, ``/voice/dialogue/state``).
- :class:`DeadManCounter` — ``dead_man_trips_total{client_id}`` счётчик
  (ADR-0028 §6 Q4, Phase 1 metric).

AV-3 (FSM ModeManager) добавил:
- :class:`ModeManager` + :class:`Mode` — конечный автомат режимов
  ``off / telegram_active / avatar_present / mixed``.
"""

from rob_box_supervisor.core.fsm import (
    ConflictError as FSMConflictError,
    Mode,
    ModeManager,
)
from .aggregator import StateAggregator
from .dead_man import DeadManCounter

__all__ = (
    "FSMConflictError",
    "Mode",
    "ModeManager",
    "StateAggregator",
    "DeadManCounter",
)