"""Transport package — real and fake transport implementations.

* :mod:`rob_box_harness.transport.base` — протокол ``Transport``,
  ``BaseTransport`` и ``FakeTransport`` (фейк для тестов).
* :mod:`rob_box_harness.transport.ros2_transport` — живой ``ROS2Transport``.

Раньше ``base.py`` лежал рядом с пакетом как ``transport.py`` и был
недостижим обычным импортом: при конфликте «модуль и пакет с одним именем»
Питон выбирает пакет. Этот ``__init__`` подгружал файл вручную через
``importlib.util.spec_from_file_location`` — см. пояснение в
:mod:`rob_box_harness.memory`. Файл переехал внутрь пакета, хак снят.
"""

from __future__ import annotations

from rob_box_harness.transport.base import (
    BaseTransport,
    EventHandler,
    FakeTransport,
    KeyEvent,
    TelegramUpdate,
    Transport,
    VadEvent,
)

# ── Lazy-load ROS2Transport (avoids importing rclpy at package-init) ──

_LAZY_NAMES = {"ROS2Transport"}


def __getattr__(name: str):
    if name in _LAZY_NAMES:
        from rob_box_harness.transport.ros2_transport import ROS2Transport as _rt

        globals()[name] = _rt
        return _rt
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "VadEvent",
    "KeyEvent",
    "TelegramUpdate",
    "Transport",
    "BaseTransport",
    "FakeTransport",
    "EventHandler",
    "ROS2Transport",
]
