#!/usr/bin/env python3
"""
Test snake_start_row parameter for wheel_front_right
Publishes test pattern with pupil offset to check snake alignment
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import time


class SnakeTestPublisher(Node):
    def __init__(self):
        super().__init__('snake_test_publisher')
        
        self.publisher = self.create_publisher(Image, '/panel_image', 10)
        
        # Wait for subscriber
        time.sleep(2)
        
        self.get_logger().info('Publishing test patterns to wheel_front_right...')
        
        # Test 1: Pupil centered
        self.get_logger().info('Test 1: Pupil centered (should look correct)')
        self.publish_eye_pattern(pupil_x=4, pupil_y=4)
        time.sleep(3)
        
        # Test 2: Pupil left
        self.get_logger().info('Test 2: Pupil left (check if distorted)')
        self.publish_eye_pattern(pupil_x=3, pupil_y=4)
        time.sleep(3)
        
        # Test 3: Pupil right
        self.get_logger().info('Test 3: Pupil right (check if distorted)')
        self.publish_eye_pattern(pupil_x=5, pupil_y=4)
        time.sleep(3)
        
        # Test 4: Pupil up
        self.get_logger().info('Test 4: Pupil up (check if distorted)')
        self.publish_eye_pattern(pupil_x=4, pupil_y=3)
        time.sleep(3)
        
        # Test 5: Pupil down
        self.get_logger().info('Test 5: Pupil down (check if distorted)')
        self.publish_eye_pattern(pupil_x=4, pupil_y=5)
        time.sleep(3)
        
        self.get_logger().info('Test complete!')
    
    def publish_eye_pattern(self, pupil_x, pupil_y):
        """Create 8x8 eye pattern with pupil at specified position"""
        # Create 8x8 RGB image
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        
        # White background (eye white)
        image[:, :] = [255, 255, 255]
        
        # Draw iris (blue circle, radius 2)
        center_x, center_y = 4, 4
        for y in range(8):
            for x in range(8):
                dist = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                if dist < 2.5:
                    image[y, x] = [0, 100, 255]  # Blue iris
        
        # Draw pupil (black circle at offset position)
        for y in range(8):
            for x in range(8):
                dist = np.sqrt((x - pupil_x)**2 + (y - pupil_y)**2)
                if dist < 1.0:
                    image[y, x] = [0, 0, 0]  # Black pupil
        
        # Create ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wheel_front_right'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'rgb8'
        msg.step = 8 * 3
        msg.is_bigendian = 0
        msg.data = image.tobytes()
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SnakeTestPublisher()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
