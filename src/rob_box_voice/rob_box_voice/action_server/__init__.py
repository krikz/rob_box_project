"""Generic long-running action protocol used by the voice runtime.

The transport is deliberately independent from ROS.  A PASTE adapter can
submit speculative, side-effect-free work to the shadow queue and commit the
chosen action exactly once.  The server owns cancellation and lifecycle.
"""

from .server import ActionError, ActionHandle, ActionServer, ActionState, Health
from .paste import PastePlanner, ShadowAction

__all__ = [
    "ActionError", "ActionHandle", "ActionServer", "ActionState", "Health",
    "PastePlanner", "ShadowAction",
]
