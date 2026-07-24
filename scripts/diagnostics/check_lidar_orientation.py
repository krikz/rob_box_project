#!/usr/bin/env python3
"""
Анализ ориентации LiDAR - проверка направления сканирования
Этот скрипт подписывается на /scan и показывает в каких секторах обнаружены препятствия
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarOrientationChecker(Node):
    def __init__(self):
        super().__init__('lidar_orientation_checker')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info('✅ Subscribed to /scan')
        self.get_logger().info('📊 Waiting for LaserScan data...')
        self.msg_count = 0

    def scan_callback(self, msg):
        self.msg_count += 1
        
        if self.msg_count % 10 != 0:  # Показываем каждое 10-е сообщение
            return
            
        # Анализируем углы
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)
        
        self.get_logger().info('━' * 80)
        self.get_logger().info(f'📡 Frame: {msg.header.frame_id}')
        self.get_logger().info(f'📐 Angle range: {math.degrees(angle_min):.1f}° to {math.degrees(angle_max):.1f}°')
        self.get_logger().info(f'📊 Number of readings: {num_readings}')
        self.get_logger().info(f'🔄 Angle increment: {math.degrees(angle_increment):.3f}°')
        
        # Разбиваем на сектора (каждый по 45°)
        sectors = {
            'Front (±22.5°)': 0,
            'Front-Left (22.5-67.5°)': 0,
            'Left (67.5-112.5°)': 0,
            'Rear-Left (112.5-157.5°)': 0,
            'Rear (157.5-180° + -180 to -157.5°)': 0,
            'Rear-Right (-157.5 to -112.5°)': 0,
            'Right (-112.5 to -67.5°)': 0,
            'Front-Right (-67.5 to -22.5°)': 0,
        }
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if math.isinf(distance) or math.isnan(distance):
                continue
                
            angle = angle_min + i * angle_increment
            angle_deg = math.degrees(angle)
            
            # Определяем сектор
            if -22.5 <= angle_deg <= 22.5:
                sectors['Front (±22.5°)'] += 1
            elif 22.5 < angle_deg <= 67.5:
                sectors['Front-Left (22.5-67.5°)'] += 1
            elif 67.5 < angle_deg <= 112.5:
                sectors['Left (67.5-112.5°)'] += 1
            elif 112.5 < angle_deg <= 157.5:
                sectors['Rear-Left (112.5-157.5°)'] += 1
            elif angle_deg > 157.5 or angle_deg < -157.5:
                sectors['Rear (157.5-180° + -180 to -157.5°)'] += 1
            elif -157.5 <= angle_deg < -112.5:
                sectors['Rear-Right (-157.5 to -112.5°)'] += 1
            elif -112.5 <= angle_deg < -67.5:
                sectors['Right (-112.5 to -67.5°)'] += 1
            elif -67.5 <= angle_deg < -22.5:
                sectors['Front-Right (-67.5 to -22.5°)'] += 1
        
        # Показываем результаты
        self.get_logger().info('🎯 Obstacle detection by sector:')
        for sector, count in sectors.items():
            if count > 0:
                bar = '█' * min(count // 5, 40)  # Визуальная полоска
                self.get_logger().info(f'  {sector:40s}: {count:4d} points {bar}')
        
        self.get_logger().info('━' * 80)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = LidarOrientationChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
