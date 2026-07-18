"""rob_box_core — cross-cutting abstractions shared by every rob_box harness.

This package is deliberately ROS-free at its core so the same code can be
unit-tested without bringing up rclpy. ROS-specific helpers (logger adapter,
param guard, lifecycle) live in submodules and stay optional.

Public surface:
    Clock / SystemClock / MockClock — time injection for tests
    MemoryStore / InMemoryStore / Turn / Fact / MemoryHit — persistence port
    DialogueState / DialogueStateMachine / IllegalTransitionError — state
        machine port (P0.3 — additive wrapper around the existing
        ``DialogueManager``; see ``docs/refactoring-plan.md``)
"""

from rob_box_core.clock import Clock, SystemClock, MockClock
from rob_box_core.memory import (
    Fact,
    InMemoryStore,
    MemoryHit,
    MemoryStore,
    Turn,
)
from rob_box_core.dialogue_state import (
    DialogueState,
    DialogueStateMachine,
    IllegalTransitionError,
)

__all__ = [
    "Clock",
    "SystemClock",
    "MockClock",
    "MemoryStore",
    "InMemoryStore",
    "Turn",
    "Fact",
    "MemoryHit",
    "DialogueState",
    "DialogueStateMachine",
    "IllegalTransitionError",
]

__version__ = "0.1.0"
