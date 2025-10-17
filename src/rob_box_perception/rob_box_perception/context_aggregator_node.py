#!/usr/bin/env python3
"""
context_aggregator_node.py - Perception Context Aggregator (MPC lite)

Легковесный агрегатор данных восприятия.
Собирает данные со всех источников → публикует unified события.

Архитектура:
  [Sensors] → Context Aggregator → [PerceptionEvent] → Reflection Agent
  
НЕ думает, НЕ принимает решений - только сбор и публикация.

Подписывается на:
- /perception/vision_context (vision processing)
- /rtabmap/localization_pose (позиция)
- /odom (одометрия)
- /device/snapshot (ESP32 sensors)
- /apriltag/detections (AprilTags)
- /rosout (system logs)
- /voice/stt/result (входящая речь)

Публикует:
- /perception/context_update (PerceptionEvent) - агрегированный контекст
- /perception/user_speech (String) - транзит STT для рефлексии
"""

import json
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log

# Custom messages
try:
    from rob_box_perception.msg import PerceptionEvent
except ImportError:
    PerceptionEvent = None  # Fallback if not built yet


class ContextAggregatorNode(Node):
    """Агрегатор контекста восприятия (MPC lite)"""
    
    def __init__(self):
        super().__init__('context_aggregator')
        
        # ============ Параметры ============
        self.declare_parameter('publish_rate', 2.0)  # Hz - частота публикации событий
        self.declare_parameter('memory_window', 60)  # секунды
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.memory_window = self.get_parameter('memory_window').value
        
        # ============ Текущее состояние (кэш) ============
        self.current_vision: Optional[Dict] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_odom: Optional[Odometry] = None
        self.current_sensors: Dict = {}
        self.last_apriltags: List[int] = []
        
        # Здоровье системы
        self.recent_errors: List[Dict] = []
        self.recent_warnings: List[Dict] = []
        
        # Короткая память (для memory_summary)
        self.recent_events: List[Dict] = []
        
        # ============ Подписки ============
        
        # Vision
        self.vision_sub = self.create_subscription(
            String,
            '/perception/vision_context',
            self.on_vision_context,
            10
        )
        
        # Pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/rtabmap/localization_pose',
            self.on_robot_pose,
            10
        )
        
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odometry,
            10
        )
        
        # Sensors (ESP32)
        # self.sensor_hub_sub = self.create_subscription(
        #     DeviceSnapshot,
        #     '/device/snapshot',
        #     self.on_device_snapshot,
        #     10
        # )
        
        # AprilTags
        # self.apriltag_sub = self.create_subscription(
        #     AprilTagDetectionArray,
        #     '/apriltag/detections',
        #     self.on_apriltags,
        #     10
        # )
        
        # System logs (/rosout)
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.on_rosout,
            10
        )
        
        # STT (входящая речь пользователя)
        self.stt_sub = self.create_subscription(
            String,
            '/voice/stt/result',
            self.on_user_speech,
            10
        )
        
        # ============ Публикации ============
        
        if PerceptionEvent:
            self.event_pub = self.create_publisher(
                PerceptionEvent,
                '/perception/context_update',
                10
            )
        else:
            self.get_logger().warn('⚠️  PerceptionEvent message не найден! Соберите пакет.')
            self.event_pub = None
        
        # Транзит STT для рефлексии
        self.speech_pub = self.create_publisher(
            String,
            '/perception/user_speech',
            10
        )
        
        # ============ Таймер публикации событий ============
        timer_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(timer_period, self.publish_event)
        
        self.get_logger().info('📊 Context Aggregator запущен')
        self.get_logger().info(f'   Частота событий: {self.publish_rate} Hz')
        self.get_logger().info(f'   Окно памяти: {self.memory_window} сек')
    
    # ============================================================
    # Callbacks - Сбор данных
    # ============================================================
    
    def on_vision_context(self, msg: String):
        """Обновление vision context"""
        try:
            self.current_vision = json.loads(msg.data)
            self.get_logger().debug(f'👁️  Vision: {self.current_vision.get("description", "N/A")}')
        except json.JSONDecodeError:
            self.get_logger().error('❌ Ошибка парсинга vision_context')
    
    def on_robot_pose(self, msg: PoseStamped):
        """Обновление позиции"""
        self.current_pose = msg
        self.get_logger().debug(f'📍 Pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def on_odometry(self, msg: Odometry):
        """Обновление одометрии"""
        self.current_odom = msg
    
    def on_device_snapshot(self, msg):
        """Обновление сенсоров ESP32"""
        self.current_sensors = {
            'battery': msg.battery_voltage,
            'temperature': msg.temperature
        }
    
    def on_apriltags(self, msg):
        """Обнаружены AprilTags"""
        self.last_apriltags = [det.id for det in msg.detections]
        self.add_to_memory('apriltag', f'Обнаружены маркеры: {self.last_apriltags}')
    
    def on_rosout(self, msg: Log):
        """Мониторинг системных логов"""
        # Собираем ERROR и WARN
        if msg.level >= 40:  # ERROR или FATAL
            error_info = {
                'time': time.time(),
                'node': msg.name,
                'message': msg.msg
            }
            self.recent_errors.append(error_info)
            if len(self.recent_errors) > 10:
                self.recent_errors.pop(0)
            self.get_logger().debug(f'⚠️  [{msg.name}] ERROR: {msg.msg[:50]}...')
        
        elif msg.level == 30:  # WARN
            warn_info = {
                'time': time.time(),
                'node': msg.name,
                'message': msg.msg
            }
            self.recent_warnings.append(warn_info)
            if len(self.recent_warnings) > 5:
                self.recent_warnings.pop(0)
    
    def on_user_speech(self, msg: String):
        """Получена речь пользователя (STT)"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f'👤 Пользователь: "{text}"')
            self.add_to_memory('user_speech', text, important=True)
            
            # Транзит для рефлексии
            self.speech_pub.publish(msg)
    
    # ============================================================
    # Память событий
    # ============================================================
    
    def add_to_memory(self, event_type: str, content: str, important: bool = False):
        """Добавить событие в память"""
        event = {
            'time': time.time(),
            'type': event_type,
            'content': content,
            'important': important
        }
        self.recent_events.append(event)
        
        # Очистка старых событий
        cutoff = time.time() - self.memory_window
        self.recent_events = [e for e in self.recent_events if e['time'] > cutoff]
    
    def get_memory_summary(self) -> str:
        """Получить краткое резюме памяти"""
        if not self.recent_events:
            return "Недавних событий нет"
        
        # Последние 5 событий
        recent = self.recent_events[-5:]
        lines = []
        for event in recent:
            age = time.time() - event['time']
            emoji = "❗" if event.get('important') else "•"
            lines.append(f"{emoji} [{age:.0f}s] {event['type']}: {event['content']}")
        
        return '\n'.join(lines)
    
    # ============================================================
    # Публикация агрегированного события
    # ============================================================
    
    def publish_event(self):
        """Публикация PerceptionEvent с агрегированным контекстом"""
        if not self.event_pub:
            return
        
        # Проверка здоровья
        health_status, health_issues = self.check_system_health()
        
        # Создаём событие
        event = PerceptionEvent()
        event.stamp = self.get_clock().now().to_msg()
        
        # Vision
        if self.current_vision:
            event.vision_context = json.dumps(self.current_vision, ensure_ascii=False)
        else:
            event.vision_context = ""
        
        # Pose
        if self.current_pose:
            event.pose = self.current_pose.pose
        
        # Velocity & Moving
        if self.current_odom:
            event.velocity = self.current_odom.twist.twist
            vx = abs(self.current_odom.twist.twist.linear.x)
            wz = abs(self.current_odom.twist.twist.angular.z)
            event.is_moving = vx > 0.01 or wz > 0.01
        else:
            event.is_moving = False
        
        # Sensors
        event.battery_voltage = self.current_sensors.get('battery', 0.0)
        event.temperature = self.current_sensors.get('temperature', 0.0)
        
        # AprilTags
        event.apriltag_ids = self.last_apriltags
        
        # System health
        event.system_health_status = health_status
        event.health_issues = health_issues
        
        # Memory
        event.memory_summary = self.get_memory_summary()
        
        # Публикуем
        self.event_pub.publish(event)
        self.get_logger().debug(f'📤 Event: health={health_status}, moving={event.is_moving}')
    
    def check_system_health(self) -> tuple[str, List[str]]:
        """Проверка здоровья системы"""
        issues = []
        
        # Проверка ошибок
        recent_error_count = len([e for e in self.recent_errors if time.time() - e['time'] < 30])
        if recent_error_count >= 5:
            issues.append(f'Много ошибок: {recent_error_count} за 30 сек')
        
        # Проверка батареи
        battery = self.current_sensors.get('battery', 100.0)
        if battery < 11.0:
            issues.append(f'Низкая батарея: {battery:.1f}V')
        
        # Определяем статус
        if len(issues) == 0:
            status = "healthy"
        elif len(issues) <= 2:
            status = "degraded"
        else:
            status = "critical"
        
        return status, issues


def main(args=None):
    rclpy.init(args=args)
    node = ContextAggregatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
