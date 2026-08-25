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

Living them in a dedicated sub-package keeps the ROS-dependent
``supervisor_node`` module thin and lets us unit-test the aggregator
and the dead-man counter without spinning up ``rclpy``.
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
