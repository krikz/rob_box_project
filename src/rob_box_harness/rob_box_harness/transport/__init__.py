"""Transport package — real and fake transport implementations.

When both ``transport.py`` (module) and ``transport/`` (package) exist,
Python resolves ``rob_box_harness.transport`` to the **package**.
This init re-exports everything from the original module file so
existing ``from rob_box_harness.transport import Transport`` imports
continue to work alongside the new ``ROS2Transport``.
"""

from __future__ import annotations

import importlib.util
import os
import sys

# ── Re-export everything from the transport.py module ───────────
# We cannot use a normal ``from rob_box_harness.transport import ...``
# because that would resolve to THIS package (circular). Instead we
# load the .py file by absolute path.
_transport_py = os.path.join(os.path.dirname(__file__), "..", "transport.py")
_spec = importlib.util.spec_from_file_location(
    "rob_box_harness._transport_module",
    os.path.abspath(_transport_py),
)
_transport_mod = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_harness._transport_module"] = _transport_mod
_spec.loader.exec_module(_transport_mod)

# Copy all public names from the module into the package namespace
_transport_all = getattr(_transport_mod, "__all__", None)
if _transport_all is not None:
    for _name in _transport_all:
        globals()[_name] = getattr(_transport_mod, _name)
else:
    for _name in dir(_transport_mod):
        if not _name.startswith("_"):
            globals()[_name] = getattr(_transport_mod, _name)

# ── Lazy-load ROS2Transport (avoids importing rclpy at package-init) ──

_LAZY_NAMES = {"ROS2Transport"}


def __getattr__(name: str):
    if name in _LAZY_NAMES:
        from rob_box_harness.transport.ros2_transport import ROS2Transport as _rt

        globals()[name] = _rt
        return _rt
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    *_transport_all,
    "ROS2Transport",
]
