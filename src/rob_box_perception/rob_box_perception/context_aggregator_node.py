#!/usr/bin/env python3
"""
context_aggregator_node.py - Perception Context Aggregator

Легковесный агрегатор данных восприятия.
Собирает данные со всех источников → публикует unified события.

Архитектура:
  [Sensors] → Context Aggregator → [PerceptionEvent] → dialogue_node

НЕ думает, НЕ принимает решений - только сбор и публикация.

Подписывается на:
- /perception/vision_context (vision processing)
- /rtabmap/localization_pose (позиция)
- /odom (одометрия)
- /device/snapshot (ESP32 sensors)
- /apriltag/detections (AprilTags)
- /rosout (system logs)
- /voice/stt/result (входящая речь)
- /voice/dialogue/response (ответы робота)
- /voice/command/intent (intent команд)
- /voice/command/feedback (feedback команд)
- /dynamic_joint_states (battery/voltage)

Публикует:
- /perception/context_update (PerceptionEvent) - агрегированный контекст
"""

import json
import time
import os
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from control_msgs.msg import DynamicJointState

# Monitoring components
from rob_box_perception.utils.node_monitor import NodeAvailabilityMonitor
from rob_box_perception.utils.internet_monitor import InternetConnectivityMonitor
from rob_box_perception.utils.time_provider import TimeAwarenessProvider

# Custom messages
try:
    from rob_box_perception_msgs.msg import PerceptionEvent
except ImportError:
    PerceptionEvent = None  # Fallback if not built yet


class ContextAggregatorNode(Node):
    """Агрегатор контекста восприятия (MPC lite)."""

    def __init__(self):
        super().__init__('context_aggregator')

        # ============ Параметры ============
        self.declare_parameter('publish_rate', 2.0)  # Hz - частота публикации событий
        self.declare_parameter('memory_window', 60)  # секунды
        self.declare_parameter('timezone', 'Europe/Moscow')  # Часовой пояс для времени

        self.publish_rate = self.get_parameter('publish_rate').value
        self.memory_window = self.get_parameter('memory_window').value
        self.timezone = self.get_parameter('timezone').value

        # ============ Текущее состояние (кэш) ============
        self.current_vision: Optional[Dict] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_odom: Optional[Odometry] = None
        self.current_sensors: Dict = {}
        self.last_apriltags: List[int] = []

        # Здоровье системы
        self.recent_errors: List[Dict] = []
        self.recent_warnings: List[Dict] = []

        # Короткая память (для memory_summary) - РАЗДЕЛЕНО ПО ТИПАМ
        self.recent_events: List[Dict] = []  # Все события (для совместимости)
        self.speech_events: List[Dict] = []  # Речь пользователя
        self.robot_response_events: List[Dict] = []  # Ответы робота
        self.robot_thought_events: List[Dict] = []  # Внутренние мысли робота
        self.vision_events: List[Dict] = []  # Визуальные события
        self.system_events: List[Dict] = []  # Ошибки, battery, warnings

        # ============ Мониторинг компоненты ============

        # Node availability monitor
        self.node_monitor = NodeAvailabilityMonitor(self)

        # Internet connectivity monitor
        self.internet_monitor = InternetConnectivityMonitor(self, check_interval=30.0)

        # Time awareness provider (using timezone parameter)
        self.time_provider = TimeAwarenessProvider(timezone=self.timezone)

        self.get_logger().info('✅ Monitoring components initialized')
        self.get_logger().info(f'   Timezone: {self.timezone}')

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

        # Battery monitoring from ros2_control
        self.joint_states_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.on_joint_states,
            10
        )

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

        # Dialogue responses (ответы робота)
        self.dialogue_response_sub = self.create_subscription(
            String,
            '/voice/dialogue/response',
            self.on_robot_response,
            10
        )

        # Command intents (от CommandNode - распознанные команды)
        self.command_intent_sub = self.create_subscription(
            String,
            '/voice/command/intent',
            self.on_command_intent,
            10
        )

        # Command feedback (от CommandNode - результаты выполнения)
        self.command_feedback_sub = self.create_subscription(
            String,
            '/voice/command/feedback',
            self.on_command_feedback,
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
            self.get_logger().warning('⚠️  PerceptionEvent message не найден! Соберите пакет.')
            self.event_pub = None

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
        """Обновление vision context."""
        try:
            self.current_vision = json.loads(msg.data)
            self.get_logger().debug(f'👁️  Vision: {self.current_vision.get("description", "N/A")}')
        except json.JSONDecodeError:
            self.get_logger().error('❌ Ошибка парсинга vision_context')

    def on_robot_pose(self, msg: PoseStamped):
        """Обновление позиции."""
        self.current_pose = msg
        self.get_logger().debug(f'📍 Pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def on_odometry(self, msg: Odometry):
        """Обновление одометрии."""
        self.current_odom = msg

    def on_device_snapshot(self, msg):
        """Обновление сенсоров ESP32."""
        self.current_sensors = {
            'battery': msg.battery_voltage,
            'temperature': msg.temperature
        }

    def on_apriltags(self, msg):
        """Обнаружены AprilTags."""
        self.last_apriltags = [det.id for det in msg.detections]
        self.add_to_memory('apriltag', f'Обнаружены маркеры: {self.last_apriltags}')

    def on_rosout(self, msg: Log):
        """Мониторинг системных логов."""
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

    def on_joint_states(self, msg: DynamicJointState):
        """Обновление данных ros2_control."""
        for interface_values in msg.interface_values:
            for name, value in zip(interface_values.interface_names,
                                   interface_values.values):
                if name == 'battery/voltage':
                    self.current_sensors['battery'] = value
                    break

    def on_user_speech(self, msg: String):
        """Получена речь пользователя (STT)."""
        text = msg.data.strip()
        if text:
            # Проверка: это команда движения или диалог?
            # Команды движения (вперед, назад, налево, направо, стой) НЕ добавляем в память
            # Они обрабатываются CommandNode и не нужны для диалога
            movement_keywords = [
                'вперед', 'вперёд', 'назад', 'влево', 'вправо', 'налево', 'направо',
                'поверни', 'повернись', 'разверн', 'двигайся', 'иди', 'поезжай', 'езжай',
                'стой', 'стоп', 'остановись', 'останови'
            ]

            is_movement_command = any(keyword in text.lower() for keyword in movement_keywords)

            if is_movement_command:
                self.get_logger().debug(f'🎮 Команда движения (пропускаем): "{text}"')
                # НЕ добавляем в память, НЕ транслируем в рефлексию
                return

            self.get_logger().info(f'👤 Пользователь: "{text}"')
            self.add_to_memory('user_speech', text, important=True)

    def on_robot_response(self, msg: String):
        """Получен ответ робота (dialogue_node)."""
        # Парсим JSON (может содержать SSML)
        try:
            import json
            data = json.loads(msg.data)
            text = data.get('ssml', '').replace('<speak>', '').replace('</speak>', '').strip()
        except:
            text = msg.data.strip()

        if text:
            self.get_logger().info(f'🤖 Робот: "{text[:50]}..."')
            self.add_to_memory('robot_response', text, important=True)

    def on_command_intent(self, msg: String):
        """Получен intent команды (от CommandNode)."""
        # Формат: "navigate:0.85" или "stop:1.0"
        parts = msg.data.split(':')
        if len(parts) == 2:
            intent, confidence = parts
            self.get_logger().debug(f'🎮 Команда: {intent} (conf={confidence})')
            # Добавляем в системные события (не важные для диалога)
            self.add_to_memory('command', f'Команда: {intent}', important=False)

    def on_command_feedback(self, msg: String):
        """Получен feedback от команды (от CommandNode)."""
        feedback = msg.data.strip()
        if feedback:
            self.get_logger().debug(f'✅ Feedback: "{feedback}"')
            # Добавляем как robot_response для истории диалога
            self.add_to_memory('robot_response', feedback, important=False)

    # ============================================================
    # Память событий
    # ============================================================

    def add_to_memory(self, event_type: str, content: str, important: bool = False):
        """Добавить событие в память (с разделением по типам)."""
        event = {
            'time': time.time(),
            'type': event_type,
            'content': content,
            'important': important
        }

        # Добавляем в общую память
        self.recent_events.append(event)

        # Добавляем в типизированные очереди
        if event_type == 'user_speech':
            self.speech_events.append(event)
        elif event_type == 'robot_response':
            self.robot_response_events.append(event)
        elif event_type == 'robot_thought':
            self.robot_thought_events.append(event)
        elif event_type in ['vision', 'apriltag']:
            self.vision_events.append(event)
        elif event_type in ['error', 'warning', 'battery', 'system']:
            self.system_events.append(event)

        # Очистка старых событий
        cutoff = time.time() - self.memory_window
        self.recent_events = [e for e in self.recent_events if e['time'] > cutoff]
        self.speech_events = [e for e in self.speech_events if e['time'] > cutoff]
        self.robot_response_events = [e for e in self.robot_response_events if e['time'] > cutoff]
        self.robot_thought_events = [e for e in self.robot_thought_events if e['time'] > cutoff]
        self.vision_events = [e for e in self.vision_events if e['time'] > cutoff]
        self.system_events = [e for e in self.system_events if e['time'] > cutoff]

    def get_memory_summary(self) -> str:
        """Получить краткое резюме памяти."""
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
        """Публикация PerceptionEvent с агрегированным контекстом."""
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

        # ============ НОВЫЕ ПОЛЯ: Мониторинг ============

        # Time context
        time_context = self.time_provider.get_current_time_context()
        event.current_time_human = time_context['human_readable']
        event.time_period = time_context['period']
        event.time_context_json = json.dumps(time_context, ensure_ascii=False)

        # Internet connectivity
        event.internet_available = self.internet_monitor.get_status()['is_online']

        # Node availability
        node_summary = self.node_monitor.get_status_summary()
        event.active_nodes = node_summary['active_list']
        event.failed_nodes = node_summary['failed_list']
        event.missing_nodes = node_summary['missing_list']

        # Equipment summary (placeholder for Stage 2)
        event.equipment_summary_json = "{}"

        # Mapping mode — читаем /maps/mapping_state.json (volume :ro)
        try:
            _state_path = "/maps/mapping_state.json"
            if os.path.exists(_state_path):
                with open(_state_path, "r") as _f:
                    _state = json.load(_f)
                event.mapping_mode = _state.get("mode", "unknown")
            else:
                event.mapping_mode = "unknown"
        except Exception:
            event.mapping_mode = "unknown"

        # Memory
        event.memory_summary = self.get_memory_summary()

        # Публикуем
        self.event_pub.publish(event)
        self.get_logger().debug(f'📤 Event: health={health_status}, moving={event.is_moving}')

    def check_system_health(self) -> tuple[str, List[str]]:
        """Проверка здоровья системы."""
        issues = []

        # Проверка ошибок
        recent_error_count = len([e for e in self.recent_errors if time.time() - e['time'] < 30])
        if recent_error_count >= 5:
            issues.append(f'Много ошибок: {recent_error_count} за 30 сек')

        # Проверка батареи
        battery = self.current_sensors.get('battery', 100.0)
        if battery > 0 and battery < 32.0:  # CRITICAL for 36V 10S
            issues.append(f'КРИТИЧЕСКАЯ БАТАРЕЯ: {battery:.1f}V - СРОЧНО НА ЗАРЯДКУ!')
        elif battery > 0 and battery < 34.0:  # LOW for 36V 10S
            issues.append(f'Низкая батарея: {battery:.1f}V')

        # Проверка нод (добавлено)
        node_summary = self.node_monitor.get_status_summary()
        if node_summary['failed']:
            issues.append(f"Упавшие ноды: {', '.join(node_summary['failed_list'])}")
        if node_summary['missing'] > 2:
            issues.append(f"Отсутствуют {node_summary['missing']} нод")

        # Проверка интернета (добавлено)
        if not self.internet_monitor.get_status()['is_online']:
            issues.append("Нет интернета")

        # Определяем статус
        if len(issues) == 0:
            status = "HEALTHY"
        elif len(issues) <= 2:
            status = "DEGRADED"
        else:
            status = "UNHEALTHY"

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
