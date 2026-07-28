"""rob_box_core — ROS-free contracts shared by the harness packages."""

from rob_box_core.clock import Clock, MockClock, SystemClock
from rob_box_core.dialogue_state import (
    DialogueState,
    DialogueStateMachine,
    IllegalTransitionError,
)
from rob_box_core.memory import Fact, InMemoryStore, MemoryHit, MemoryStore, Turn
from rob_box_core.ports import (
    ToolContext,
    ToolDescriptor,
    ToolNotFound,
    ToolNotFoundError,
    ToolProvider,
    ToolProviderError,
    ToolResult,
    ToolTimeout,
    ToolTimeoutError,
    ToolValidationError,
    ValidationResult,
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
    "ToolProvider",
    "ToolDescriptor",
    "ToolResult",
    "ToolContext",
    "ValidationResult",
    "ToolProviderError",
    "ToolNotFound",
    "ToolNotFoundError",
    "ToolTimeout",
    "ToolTimeoutError",
    "ToolValidationError",
]

__version__ = "0.1.0"
