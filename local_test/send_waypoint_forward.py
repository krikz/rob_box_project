#!/usr/bin/env python3
"""
Send a waypoint 1 meter forward to Nav2
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # QoS profile for Nav2 goal_pose topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile
        )
        
        # Wait a bit for discovery
        self.get_logger().info('Waiting for Nav2 to discover...')
        import time
        time.sleep(2)
        
        # Create waypoint 1 meter forward
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.header.stamp = self.get_clock().now().to_msg()
        
        # Position: 1 meter forward (x=1.0, y=0.0)
        waypoint.pose.position.x = 1.0
        waypoint.pose.position.y = 0.0
        waypoint.pose.position.z = 0.0
        
        # Orientation: facing forward (no rotation)
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Publishing waypoint: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}')
        
        # Publish multiple times to ensure delivery
        for i in range(5):
            waypoint.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(waypoint)
            self.get_logger().info(f'Published waypoint (attempt {i+1}/5)')
            time.sleep(0.5)
        
        self.get_logger().info('Waypoint published successfully!')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
