"""rob_box_core — ROS-free contracts shared by the harness packages."""

from rob_box_core.avatar_command import (
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
    SOURCES,
    build_command,
    build_command_result,
    decode_command,
    decode_command_result,
    encode_command,
    encode_command_result,
    make_quest_client_id,
    make_telegram_client_id,
    new_request_id,
    now_ts_ms,
)
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
    # avatar_command (AV-22, freeze §3.3 worker-brief)
    "AVATAR_COMMAND_TOPIC",
    "AVATAR_COMMAND_RESULT_TOPIC",
    "SOURCES",
    "build_command",
    "build_command_result",
    "decode_command",
    "decode_command_result",
    "encode_command",
    "encode_command_result",
    "make_quest_client_id",
    "make_telegram_client_id",
    "new_request_id",
    "now_ts_ms",
]

__version__ = "0.1.0"
