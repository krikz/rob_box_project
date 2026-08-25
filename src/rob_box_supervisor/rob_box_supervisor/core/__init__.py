"""Avatar Supervisor core helpers (Phase 1 monitor, AV-6).

Pure-Python building blocks that do NOT import rclpy:

- :class:`StateAggregator` — assembles ``/avatar/state`` from inputs
  (``/odom``, ``/device/snapshot``, ``/voice/dialogue/state``).
- :class:`DeadManCounter` — ``dead_man_trips_total{client_id}`` counter
  (ADR-0028 §6 Q4, Phase 1 metric).

Living them in a dedicated sub-package keeps the ROS-dependent
``supervisor_node`` module thin and lets us unit-test the aggregator
and the dead-man counter without spinning up ``rclpy``.
"""

from .aggregator import StateAggregator
from .dead_man import DeadManCounter

__all__ = ["StateAggregator", "DeadManCounter"]
