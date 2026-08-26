"""Phase 2.2 perf logger public API.

QuestPerfLoggerNode экспортируется только если rclpy установлен (в ROS2 env).
На dev-машинах rclpy отсутствует — unit-тесты используют только JSON/CLI логику.
"""

from .logger_node import (
    DEFAULT_LOG_DIR,
    DEFAULT_MAX_BYTES_PER_FILE,
    DEFAULT_RETENTION_DAYS,
    JsonlWriter,
    PerfDedup,
    main,
    validate_telemetry_payload,
)

try:
    from .logger_node import QuestPerfLoggerNode  # noqa: F401
    _HAS_ROS = True
except ImportError:
    QuestPerfLoggerNode = None  # type: ignore[assignment,misc]
    _HAS_ROS = False

__all__ = [
    "DEFAULT_LOG_DIR",
    "DEFAULT_MAX_BYTES_PER_FILE",
    "DEFAULT_RETENTION_DAYS",
    "JsonlWriter",
    "PerfDedup",
    "QuestPerfLoggerNode",
    "main",
    "validate_telemetry_payload",
]