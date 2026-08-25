"""supervisor_node entry-point (skeleton, AV-2).

The actual ROS 2 node body lands in AV-6. For Phase 1 (AV-9) the node
runs in monitor-only mode and just publishes ``/avatar/state``; this
module exists today so the ``supervisor_node`` console_script entry
point resolves cleanly under colcon/pytest.
"""

from typing import Optional

import rclpy
from rclpy.node import Node


def _build_node() -> Node:
    """Construct the (currently minimal) supervisor Node.

    Kept as a tiny helper so AV-6 can extend it without changing the
    console-script entry point (``supervisor_node = ...:main``).
    """
    return Node("avatar_supervisor")


def main(args: Optional[list] = None) -> None:
    """Console-script entry point: ``ros2 run rob_box_supervisor supervisor_node``."""
    rclpy.init(args=args)
    node = _build_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
