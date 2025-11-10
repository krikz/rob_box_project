#!/usr/bin/env python3
"""
Zenoh REST to ROS Bridge
Мост между Zenoh REST API и ROS топиками

Решает проблему: REST API публикует на простой ключ (robots/.../cmd_vel_voice),
но ROS подписывается на сложный ключ с типом сообщения и хашем.

Этот мост:
1. Подписывается на упрощенный Zenoh топик (robots/.../cmd_vel_web_bridge)
2. Конвертирует JSON в ROS сообщение Twist
3. Публикует в ROS топик cmd_vel_voice

Использование:
    python3 zenoh_rest_bridge.py
    
    # В другом терминале отправить команду:
    curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge \
      -H "Content-Type: application/json" \
      -d '{"linear":{"x":0.1},"angular":{"z":0.0}}'
"""

import json
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import zenoh
except ImportError:
    print("ERROR: zenoh-python не установлен!")
    print("Установите: pip install eclipse-zenoh")
    sys.exit(1)


class ZenohRestBridge(Node):
    """Мост между Zenoh REST API и ROS топиками"""
    
    def __init__(self):
        super().__init__('zenoh_rest_bridge')
        
        # Параметры
        self.declare_parameter('robot_id', 'RBXU100001')
        self.declare_parameter('bridge_topic', 'cmd_vel_web_bridge')
        self.declare_parameter('ros_topic', 'cmd_vel_voice')
        self.declare_parameter('zenoh_config', '')  # Путь к конфигу или пусто для auto
        
        self.robot_id = self.get_parameter('robot_id').value
        self.bridge_topic = self.get_parameter('bridge_topic').value
        self.ros_topic = self.get_parameter('ros_topic').value
        zenoh_config_path = self.get_parameter('zenoh_config').value
        
        # ROS publisher для команд управления
        self.cmd_vel_pub = self.create_publisher(Twist, self.ros_topic, 10)
        
        # Zenoh session
        if zenoh_config_path:
            conf = zenoh.Config.from_file(zenoh_config_path)
        else:
            # Автоматическая конфигурация - подключение к локальному роутеру
            conf = zenoh.Config()
        
        self.get_logger().info(f'Подключение к Zenoh...')
        self.session = zenoh.open(conf)
        
        # Zenoh ключ для подписки (без namespace, так как сессия уже имеет namespace)
        # REST API будет публиковать на: robots/{robot_id}/cmd_vel_web_bridge
        # Но так как у сессии уже есть namespace robots/{robot_id}, подписываемся просто на имя
        zenoh_key = f'{self.bridge_topic}'
        
        self.get_logger().info(f'Подписка на Zenoh топик: {zenoh_key}')
        self.get_logger().info(f'Публикация в ROS топик: {self.ros_topic}')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info('')
        self.get_logger().info('Для отправки команды используйте:')
        self.get_logger().info(f'  curl -X PUT http://zenoh.robbox.online/robots/{self.robot_id}/{self.bridge_topic} \\')
        self.get_logger().info('    -H "Content-Type: application/json" \\')
        self.get_logger().info('    -d \'{"linear":{"x":0.1,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}\'')
        self.get_logger().info('')
        
        # Подписка на Zenoh топик
        self.subscriber = self.session.declare_subscriber(
            zenoh_key,
            self._on_zenoh_message
        )
        
        self.get_logger().info('✅ Мост запущен и готов к работе!')
    
    def _on_zenoh_message(self, sample):
        """Обработчик сообщений из Zenoh"""
        try:
            # Получить данные
            payload_bytes = sample.payload.to_bytes()
            
            # Попробовать декодировать как JSON
            try:
                data = json.loads(payload_bytes.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                self.get_logger().warn(f'Не удалось декодировать JSON: {e}')
                return
            
            # Создать ROS сообщение Twist
            twist = Twist()
            
            # Заполнить из JSON
            if 'linear' in data:
                twist.linear.x = float(data['linear'].get('x', 0.0))
                twist.linear.y = float(data['linear'].get('y', 0.0))
                twist.linear.z = float(data['linear'].get('z', 0.0))
            
            if 'angular' in data:
                twist.angular.x = float(data['angular'].get('x', 0.0))
                twist.angular.y = float(data['angular'].get('y', 0.0))
                twist.angular.z = float(data['angular'].get('z', 0.0))
            
            # Опубликовать в ROS
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info(
                f'📨 Получено и отправлено: '
                f'linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}), '
                f'angular=({twist.angular.x:.2f}, {twist.angular.y:.2f}, {twist.angular.z:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки сообщения: {e}')
    
    def destroy_node(self):
        """Очистка ресурсов"""
        self.get_logger().info('Остановка моста...')
        if hasattr(self, 'subscriber'):
            self.subscriber.undeclare()
        if hasattr(self, 'session'):
            self.session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = ZenohRestBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
