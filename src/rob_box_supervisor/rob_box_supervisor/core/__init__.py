"""Core modules for Avatar Supervisor (FSM, LockManager, dispatcher, aggregator).

Pure-Python координация (FSM, lock manager): эти модули живут
**без ROS/asyncio/threading**, чтобы их легко было покрыть TDD
(см. ``test/unit/core/``). Связь с ROS 2 — через ``supervisor_node.py``,
который будет добавлен в AV-6.
"""

from rob_box_supervisor.core.fsm import (
    ConflictError as FSMConflictError,
    Mode,
    ModeManager,
)

__all__ = ("FSMConflictError", "Mode", "ModeManager")
