"""``rob_box_harness.core`` — pure building blocks for harnesses.

This package contains infrastructure that harnesses depend on but
that has NO I/O, NO ROS2, and NO LLM dependencies — making every
component here immediately testable with plain ``pytest``.
"""

from __future__ import annotations

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)

__all__ = [
    "DialogueEvent",
    "DialogueStateKind",
    "DialogueStateMachine",
]
