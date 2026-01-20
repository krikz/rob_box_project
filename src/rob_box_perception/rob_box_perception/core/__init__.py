"""Core modules for rob_box_perception package.

Pure Python modules with no ROS dependencies for event detection,
context building, health analysis, and memory management.
"""

from rob_box_perception.core.event_detector import EventDetector, EventChange

__all__ = [
    'EventDetector',
    'EventChange',
]
