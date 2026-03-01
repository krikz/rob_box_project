#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomChecker(Node):
    def __init__(self):
        super().__init__('odom_checker')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback,
            10)
        self.count = 0

    def callback(self, msg):
        self.count += 1
        if self.count <= 3:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            self.get_logger().info(f'Sample {self.count}:')
            self.get_logger().info(f'  Position: x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f}')
            self.get_logger().info(f'  Orientation: w={ori.w:.3f} x={ori.x:.3f} y={ori.y:.3f} z={ori.z:.3f}')
            
            # Check if identity
            if abs(pos.x) < 0.001 and abs(pos.y) < 0.001 and abs(ori.w - 1.0) < 0.001:
                self.get_logger().warn('  ⚠️  IDENTITY POSE!')
        
        if self.count >= 3:
            raise KeyboardInterrupt()

def main():
    rclpy.init()
    node = OdomChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
