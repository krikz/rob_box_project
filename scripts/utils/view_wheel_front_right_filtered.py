#!/usr/bin/env python3
"""
View wheel_front_right LED matrix images only
Filters /panel_image topic by frame_id='wheel_front_right'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class WheelImageViewer(Node):
    def __init__(self):
        super().__init__('wheel_front_right_viewer')
        
        self.subscription = self.create_subscription(
            Image,
            '/panel_image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Starting wheel_front_right viewer...')
        self.get_logger().info('Filtering: frame_id="wheel_front_right"')
        self.get_logger().info('Press Ctrl+C to stop')
        
        # Create window
        cv2.namedWindow('wheel_front_right (8x8)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('wheel_front_right (8x8)', 400, 400)
        
        self.frame_count = 0

    def image_callback(self, msg):
        # Filter by frame_id
        if msg.header.frame_id != 'wheel_front_right':
            return
        
        self.frame_count += 1
        
        try:
            # Convert ROS Image to numpy array (manual conversion, no cv_bridge)
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = img_array.reshape((msg.height, msg.width, 3))
            
            # Scale up for better visibility (8x8 -> 400x400)
            scale_factor = 50
            scaled_image = cv2.resize(
                cv_image, 
                (cv_image.shape[1] * scale_factor, cv_image.shape[0] * scale_factor),
                interpolation=cv2.INTER_NEAREST
            )
            
            # Convert RGB to BGR for OpenCV display
            bgr_image = cv2.cvtColor(scaled_image, cv2.COLOR_RGB2BGR)
            
            # Add frame info
            info_text = f'Frame: {self.frame_count} | Size: {msg.width}x{msg.height}'
            cv2.putText(
                bgr_image, 
                info_text, 
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
            
            # Show image
            cv2.imshow('wheel_front_right (8x8)', bgr_image)
            cv2.waitKey(1)
            
            # Log every 10 frames
            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f'Received {self.frame_count} frames from wheel_front_right'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    viewer = WheelImageViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.get_logger().info(f'Total frames received: {viewer.frame_count}')
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
