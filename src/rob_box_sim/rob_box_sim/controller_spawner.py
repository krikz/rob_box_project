"""Запуск контроллера после первого такта Gazebo."""

from __future__ import annotations

import argparse
import os
import sys


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Spawner контроллера после первого /clock Gazebo",
        allow_abbrev=False,
    )
    ap.add_argument("controller")
    ap.add_argument("--clock-topic", default="/clock")
    ap.add_argument("--clock-timeout", type=float, default=60.0)
    args, spawner_args = ap.parse_known_args(argv)

    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.task import Future
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("controller_spawner_clock_gate")
    ready = Future()

    def on_clock(_msg: Clock) -> None:
        if not ready.done():
            ready.set_result(None)

    node.create_subscription(
        Clock,
        args.clock_topic,
        on_clock,
        QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
    )
    try:
        rclpy.spin_until_future_complete(node, ready, timeout_sec=args.clock_timeout)
        if not ready.done():
            node.get_logger().error(
                f"Gazebo clock '{args.clock_topic}' не появился за {args.clock_timeout:g} с; "
                f"контроллер {args.controller} не запущен"
            )
            return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    os.execvp(
        "ros2",
        [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            args.controller,
            *spawner_args,
        ],
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
