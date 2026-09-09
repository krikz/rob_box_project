#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class CompareOdom(Node):
    def __init__(self):
        super().__init__('compare_odom')
        
        self.diff_odom = None
        self.icp_odom = None
        
        self.sub_diff = self.create_subscription(
            Odometry, '/odom', self.diff_callback, 10)
        
        self.sub_icp = self.create_subscription(
            Odometry, '/rtabmap/odom', self.icp_callback, 10)
        
        self.timer = self.create_timer(2.0, self.print_comparison)
    
    def diff_callback(self, msg):
        self.diff_odom = msg
    
    def icp_callback(self, msg):
        self.icp_odom = msg
    
    def print_comparison(self):
        self.get_logger().info('=' * 60)
        
        if self.diff_odom:
            pos = self.diff_odom.pose.pose.position
            self.get_logger().info(f'DIFF DRIVE: x={pos.x:7.3f} y={pos.y:7.3f} z={pos.z:7.3f}')
        else:
            self.get_logger().warn('DIFF DRIVE: No data')
        
        if self.icp_odom:
            pos = self.icp_odom.pose.pose.position
            self.get_logger().info(f'ICP RTABMAP: x={pos.x:7.3f} y={pos.y:7.3f} z={pos.z:7.3f}')
        else:
            self.get_logger().warn('ICP RTABMAP: No data')

def main():
    rclpy.init()
    node = CompareOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
