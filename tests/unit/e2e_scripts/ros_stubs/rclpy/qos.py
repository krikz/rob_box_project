"""Фейковые QoS-типы (см. rclpy/__init__.py)."""

from enum import Enum


class ReliabilityPolicy(Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy(Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class HistoryPolicy(Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
