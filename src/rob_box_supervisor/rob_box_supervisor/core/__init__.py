"""Avatar Supervisor core helpers.

Pure-Python building blocks that do NOT import rclpy:

AV-3 (issue #1597) — state-machine of avatar modes
(:class:`ModeManager`, :class:`Mode`, :class:`FSMConflictError`).

AV-6 (issue #1600) — Phase 1 monitor helpers used by
``supervisor_node.py``:

- :class:`StateAggregator` — assembles ``/avatar/state`` from inputs
  (``/odom``, ``/device/snapshot``, ``/voice/dialogue/state``).
- :class:`DeadManCounter` — ``dead_man_trips_total{client_id}`` counter
  (ADR-0028 §6 Q4, Phase 1 metric).

W3-2 (issue #968 wave2 gap G2/G3) — резолвит дубликат floor-логики
между :mod:`core.fsm` и :mod:`core.locks` (ADR-0028 §4.2): реальная
выдача ``voice_floor``/``teleop_floor`` идёт через :class:`LockManager`
(dead-man 500 мс, независимые floor-ы). :class:`ModeManager` остаётся
только за режимами аватара (``off``/``telegram_active``/... ) и
использует свои ``voice_held_by``/``teleop_held_by`` лишь как вход для
решений о переходах — это НЕ источник истины по floor-ам для сервисов
``acquire_floor``/``release_floor``.

Living them in a dedicated sub-package keeps the ROS-dependent
``supervisor_node`` module thin and lets us unit-test the aggregator,
the dead-man counter and the lock manager without spinning up ``rclpy``.
"""

from rob_box_supervisor.core.fsm import (
    ConflictError as FSMConflictError,
    Mode,
    ModeManager,
)
from rob_box_supervisor.core.locks import (
    ConflictError as LockConflictError,
    Floor,
    LockManager,
)
from .aggregator import StateAggregator
from .dead_man import DeadManCounter

__all__ = (
    "FSMConflictError",
    "Mode",
    "ModeManager",
    "LockConflictError",
    "Floor",
    "LockManager",
    "StateAggregator",
    "DeadManCounter",
)
