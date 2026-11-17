#!/usr/bin/env python3
"""
Симулятор 2D лидара для тестирования rtabmap
Публикует LaserScan данные с простой геометрией (квадратная комната)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import time

class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Параметры лидара (как у LSLidar LS128)
        self.angle_min = -math.pi  # -180 градусов
        self.angle_max = math.pi   # +180 градусов
        self.angle_increment = math.radians(0.2)  # 0.2 градуса разрешение
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        self.range_min = 0.1  # минимум 10 см
        self.range_max = 20.0  # максимум 20 м
        
        # Таймер для публикации
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        
        self.get_logger().info('Fake LiDAR node started')
        self.get_logger().info(f'Publishing {self.num_readings} points per scan')
        
        self.scan_count = 0
        
    def publish_scan(self):
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Создаём простую геометрию: квадратная комната 5x5 метров
        # Робот в центре
        ranges = []
        intensities = []
        
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            
            # Имитация квадратной комнаты
            # Стены расположены на расстоянии 2.5м от центра
            distance = 2.5
            
            # Добавляем небольшой шум для реалистичности
            noise = (math.sin(self.scan_count * 0.1 + i * 0.01) * 0.02)
            distance += noise
            
            # Добавляем "препятствие" (например, стул) в определённом секторе
            if -math.pi/4 < angle < math.pi/4:
                distance = min(distance, 1.5 + noise)
            
            ranges.append(float(distance))
            intensities.append(100.0)
        
        scan.ranges = ranges
        scan.intensities = intensities
        
        self.publisher.publish(scan)
        self.scan_count += 1
        
        if self.scan_count % 100 == 0:
            self.get_logger().info(f'Published {self.scan_count} scans')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
