"""conftest.py — ROS2 stubs tailored to ``command_node`` import surface.

The shared ``test/unit/node/conftest.py`` mocks enough rclpy bits for
DialogueNode but NOT for ``rob_box_voice.command_node`` — the command
node also pulls in ``rclpy.action``, ``geometry_msgs``,
``nav2_msgs.action``, and ``tf2_ros``. Each is set up here so a
plain ``import rob_box_voice.command_node`` succeeds without a real
ROS install.

Run only with::

    pytest src/rob_box_voice/test/unit/node/command_reflex_bridge/ -v

Why a local conftest and not extending the shared one? ``test/unit/
node/conftest.py`` is loaded first by pytest (closest first), so
extending it would override the DialogNode-shaping mocks and break
``test_dialogue_node.py``. A sibling subdirectory keeps the bridge
tests fully isolated.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

# Make ``ros_stubs`` importable — same trick as the parent conftest.
_HERE = Path(__file__).resolve().parent
_TEST_ROOT = _HERE.parents[2]
if str(_TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(_TEST_ROOT))


def _install_command_node_mocks() -> None:
    """Register stubs for every package ``command_node`` imports."""

    # --- rclpy.action (ActionClient) ---------------------------------
    mock_rclpy_action = MagicMock()
    mock_rclpy_action.ActionClient = MagicMock

    # --- geometry_msgs.msg -------------------------------------------
    mock_geometry_msgs = MagicMock()
    mock_geometry_msgs_msg = MagicMock()
    mock_geometry_msgs_msg.PoseStamped = MagicMock
    mock_geometry_msgs_msg.Twist = MagicMock

    # --- nav2_msgs.action --------------------------------------------
    mock_nav2_msgs = MagicMock()
    mock_nav2_msgs_action = MagicMock()
    mock_nav2_msgs_action.NavigateToPose = MagicMock

    # --- tf2_ros (not actually imported by command_node at top-level
    # but appears via core.command_parser; safety net) ----------------
    mock_tf2_ros = MagicMock()
    mock_tf2_ros.Buffer = MagicMock
    mock_tf2_ros.TransformListener = MagicMock

    # --- nav_msgs.msg.Odometry (used by command_parser imports) ------
    mock_nav_msgs = MagicMock()
    mock_nav_msgs_msg = MagicMock()
    mock_nav_msgs_msg.Odometry = MagicMock

    # --- rclpy.executors + rclpy.task (used by some downstream paths) -
    mock_rclpy = MagicMock()
    mock_rclpy_executors = MagicMock()
    mock_rclpy_task = MagicMock()

    targets = {
        "rclpy": mock_rclpy,
        "rclpy.action": mock_rclpy_action,
        "rclpy.executors": mock_rclpy_executors,
        "rclpy.task": mock_rclpy_task,
        "geometry_msgs": mock_geometry_msgs,
        "geometry_msgs.msg": mock_geometry_msgs_msg,
        "nav2_msgs": mock_nav2_msgs,
        "nav2_msgs.action": mock_nav2_msgs_action,
        "nav_msgs": mock_nav_msgs,
        "nav_msgs.msg": mock_nav_msgs_msg,
        "tf2_ros": mock_tf2_ros,
    }
    for name, stub in targets.items():
        sys.modules.setdefault(name, stub)


_install_command_node_mocks()
