#!/usr/bin/env python3
"""
Dummy Joint State Publisher
Публикует значения для всех continuous joints с симуляцией движения вперед
Используется для визуализации робота в RViz без реальных моторов
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DummyJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_state_publisher')
        
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Публикуем 50 Hz для плавного вращения
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # Текущий угол вращения колес (радианы)
        self.wheel_angle = 0.0
        
        # Скорость вращения колес (рад/с) - имитирует движение ~0.5 м/с
        # v = ω * r, где r = 0.115м (радиус колеса)
        # ω = v / r = 0.5 / 0.115 ≈ 4.35 рад/с
        self.wheel_velocity = 4.35  # рад/с
        
        self.get_logger().info('Dummy Joint State Publisher запущен - робот "едет" вперед')
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Все continuous joints (колеса)
        msg.name = [
            'rear_left_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Увеличиваем угол вращения
        dt = 0.02  # период таймера
        self.wheel_angle += self.wheel_velocity * dt
        
        # Нормализуем угол в диапазоне [0, 2π]
        if self.wheel_angle > 2 * math.pi:
            self.wheel_angle -= 2 * math.pi
        
        # Все колеса вращаются с одинаковой скоростью (движение вперед)
        # Левые колеса: положительное направление (axis xyz="1 0 0")
        # Правые колеса: отрицательное направление (axis xyz="-1 0 0")
        msg.position = [
            self.wheel_angle,   # rear_left
            self.wheel_angle,   # front_left
            -self.wheel_angle,  # front_right (обратное направление оси)
            -self.wheel_angle   # rear_right (обратное направление оси)
        ]
        
        msg.velocity = [
            self.wheel_velocity,   # rear_left
            self.wheel_velocity,   # front_left
            -self.wheel_velocity,  # front_right
            -self.wheel_velocity   # rear_right
        ]
        
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
