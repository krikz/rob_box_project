#!/usr/bin/env python3
"""
Precise movement test for Rob Box.
Creates publisher, waits for subscriber discovery, then sends
exact velocity for exact duration.

Usage: python3 move_test.py [speed_m_s] [duration_s]
  Default: 0.1 m/s for 1.0 s = 10 cm
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class MoveTest(Node):
    def __init__(self, speed: float, duration: float):
        super().__init__("move_test")
        self.speed = speed
        self.duration = duration
        self.start_odom = None
        self.end_odom = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_web", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.get_logger().info(f"Waiting for /odom and subscriber discovery...")

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def wait_ready(self, timeout=10.0):
        """Wait for odom data and at least one subscriber on cmd_vel_web."""
        start = time.time()
        self.last_odom = None
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            subs = self.cmd_pub.get_subscription_count()
            if self.last_odom is not None and subs > 0:
                self.get_logger().info(
                    f"Ready! {subs} subscriber(s) on /cmd_vel_web, odom flowing"
                )
                return True
        self.get_logger().error("Timeout waiting for odom/subscribers")
        return False

    def run_test(self):
        # Record start
        rclpy.spin_once(self, timeout_sec=0.1)
        self.start_odom = self.last_odom
        sx = self.start_odom.pose.pose.position.x
        sy = self.start_odom.pose.pose.position.y
        self.get_logger().info(f"START: x={sx:.4f}, y={sy:.4f}")

        # Send velocity
        twist = Twist()
        twist.linear.x = self.speed
        rate_hz = 20
        interval = 1.0 / rate_hz
        count = int(self.duration * rate_hz)

        self.get_logger().info(
            f"Sending {self.speed} m/s for {self.duration}s "
            f"({count} msgs at {rate_hz}Hz)..."
        )

        t0 = time.time()
        for i in range(count):
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.001)
            elapsed = time.time() - t0
            sleep_time = (i + 1) * interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Send stop
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

        actual_duration = time.time() - t0
        self.get_logger().info(f"Stopped. Actual send duration: {actual_duration:.3f}s")

        # Wait for robot to settle
        time.sleep(1.5)

        # Record end
        rclpy.spin_once(self, timeout_sec=0.5)
        self.end_odom = self.last_odom
        ex = self.end_odom.pose.pose.position.x
        ey = self.end_odom.pose.pose.position.y
        dx = ex - sx
        dy = ey - sy
        dist = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(f"END:   x={ex:.4f}, y={ey:.4f}")
        self.get_logger().info(f"DELTA: dx={dx:.4f}, dy={dy:.4f}")
        self.get_logger().info(f"ODOM DISTANCE: {dist:.4f} m ({dist*100:.1f} cm)")
        self.get_logger().info(
            f"EXPECTED: {abs(self.speed) * self.duration:.4f} m "
            f"({abs(self.speed) * self.duration * 100:.1f} cm)"
        )
        ratio = dist / (abs(self.speed) * self.duration) if self.speed != 0 else 0
        self.get_logger().info(f"RATIO (actual/expected): {ratio:.3f}")


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    node = MoveTest(speed, duration)

    if not node.wait_ready():
        node.destroy_node()
        rclpy.shutdown()
        return

    node.run_test()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
