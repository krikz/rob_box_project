#!/usr/bin/env python3
"""
reflection_node.py - Internal Dialogue Agent

Ядро системы внутреннего диалога робота.
Собирает контекст из всех источников, размышляет, решает говорить или нет.

Подписывается на:
- /perception/vision_context (семантический контекст с камер - стаб)
- /apriltag/detections (AprilTag маркеры)
- /voice/stt/result (речь пользователя)
- /voice/dialogue/response (ответы робота)
- /rtabmap/localization_pose (позиция на карте)
- /odom (одометрия)
- /device/snapshot (сенсоры ESP32: батарея, температура)
- /sensors/motor_state/* (состояние моторов)

Публикует:
- /reflection/internal_thought (внутренние мысли для логов)
- /voice/tts/request (когда решает сказать что-то вслух)
"""

import json
import time
from typing import Optional, Dict, List
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# TODO: Добавить импорты для AprilTag и DeviceSnapshot когда понадобятся
# from apriltag_msgs.msg import AprilTagDetectionArray
# from robot_sensor_hub_msg.msg import DeviceSnapshot


class ReflectionNode(Node):
    """Нода внутреннего диалога робота"""
    
    def __init__(self):
        super().__init__('reflection_node')
        
        # ============ Параметры ============
        self.declare_parameter('reflection_rate', 1.0)  # Hz - частота размышлений
        self.declare_parameter('dialogue_timeout', 10.0)  # секунд - тайм-аут диалога
        self.declare_parameter('memory_window', 60)  # секунд - окно короткой памяти
        self.declare_parameter('enable_speech', True)  # Включить речь робота
        
        self.reflection_rate = self.get_parameter('reflection_rate').value
        self.dialogue_timeout = self.get_parameter('dialogue_timeout').value
        self.memory_window = self.get_parameter('memory_window').value
        self.enable_speech = self.get_parameter('enable_speech').value
        
        # ============ Состояние ============
        self.in_dialogue = False
        self.last_user_speech_time = None
        
        # ============ Память ============
        self.short_memory: List[Dict] = []  # Последние 60 секунд
        self.long_memory: List[Dict] = []  # Важные события
        
        # ============ Текущий контекст ============
        self.current_vision = None
        self.current_pose = None
        self.current_odom = None
        self.current_sensors = None
        self.last_apriltags = []
        
        # ============ Подписки - Vision ============
        self.vision_sub = self.create_subscription(
            String,
            '/perception/vision_context',
            self.on_vision_context,
            10
        )
        
        # TODO: Раскомментировать когда AprilTag messages будут доступны
        # self.apriltag_sub = self.create_subscription(
        #     AprilTagDetectionArray,
        #     '/apriltag/detections',
        #     self.on_apriltags,
        #     10
        # )
        
        # ============ Подписки - Диалог ============
        self.stt_sub = self.create_subscription(
            String,
            '/voice/stt/result',
            self.on_user_speech,
            10
        )
        
        self.dialogue_sub = self.create_subscription(
            String,
            '/voice/dialogue/response',
            self.on_robot_response,
            10
        )
        
        # ============ Подписки - Локализация ============
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/rtabmap/localization_pose',
            self.on_robot_pose,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odometry,
            10
        )
        
        # TODO: Подписка на сенсоры ESP32
        # self.sensor_hub_sub = self.create_subscription(
        #     DeviceSnapshot,
        #     '/device/snapshot',
        #     self.on_device_snapshot,
        #     10
        # )
        
        # ============ Публикации ============
        self.thought_pub = self.create_publisher(
            String,
            '/reflection/internal_thought',
            10
        )
        
        self.tts_pub = self.create_publisher(
            String,
            '/voice/tts/request',
            10
        )
        
        # ============ Таймер размышлений ============
        timer_period = 1.0 / self.reflection_rate
        self.reflection_timer = self.create_timer(timer_period, self.reflection_loop)
        
        # ============ DeepSeek API клиент ============
        self.deepseek_api_key = os.getenv('DEEPSEEK_API_KEY')
        if not self.deepseek_api_key:
            self.get_logger().warn('⚠️  DEEPSEEK_API_KEY не найден! Используется заглушка.')
        
        self.get_logger().info('🧠 Reflection Node запущен')
        self.get_logger().info(f'   Частота размышлений: {self.reflection_rate} Hz')
        self.get_logger().info(f'   Тайм-аут диалога: {self.dialogue_timeout} сек')
        self.get_logger().info(f'   Окно памяти: {self.memory_window} сек')
    
    # ============================================================
    # Callbacks - Восприятие
    # ============================================================
    
    def on_vision_context(self, msg: String):
        """Получен семантический контекст с камер"""
        try:
            self.current_vision = json.loads(msg.data)
            self.get_logger().debug(f'👁️  Vision: {self.current_vision.get("description", "N/A")}')
        except json.JSONDecodeError:
            self.get_logger().error('❌ Ошибка парсинга vision_context JSON')
    
    def on_apriltags(self, msg):
        """Обнаружены AprilTag маркеры"""
        self.last_apriltags = msg.detections
        if len(self.last_apriltags) > 0:
            tag_ids = [d.id for d in self.last_apriltags]
            self.get_logger().info(f'🏷️  AprilTags: {tag_ids}')
            self.add_to_memory('apriltag', f'Обнаружены маркеры: {tag_ids}')
    
    def on_robot_pose(self, msg: PoseStamped):
        """Обновление позиции робота на карте"""
        self.current_pose = msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().debug(f'📍 Позиция: ({x:.2f}, {y:.2f})')
    
    def on_odometry(self, msg: Odometry):
        """Обновление одометрии"""
        self.current_odom = msg
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.get_logger().debug(f'🚗 Скорость: vx={vx:.2f}, wz={wz:.2f}')
    
    def on_device_snapshot(self, msg):
        """Данные сенсоров от ESP32"""
        self.current_sensors = {
            'battery': msg.battery_voltage,
            'temperature': msg.temperature,
            'timestamp': time.time()
        }
        self.get_logger().debug(
            f'🔋 Батарея: {msg.battery_voltage:.1f}V, '
            f'Температура: {msg.temperature:.1f}°C'
        )
    
    # ============================================================
    # Callbacks - Диалог
    # ============================================================
    
    def on_user_speech(self, msg: String):
        """Пользователь начал говорить - ставим диалог на паузу"""
        self.in_dialogue = True
        self.last_user_speech_time = time.time()
        
        self.get_logger().info(f'👤 Пользователь: "{msg.data}"')
        self.add_to_memory('user', msg.data)
    
    def on_robot_response(self, msg: String):
        """Робот ответил - продолжаем диалог"""
        self.last_user_speech_time = time.time()
        
        self.get_logger().info(f'🤖 Робот: "{msg.data}"')
        self.add_to_memory('robot', msg.data)
    
    # ============================================================
    # Память
    # ============================================================
    
    def add_to_memory(self, event_type: str, content: str, important: bool = False):
        """Добавить событие в память"""
        event = {
            'type': event_type,
            'content': content,
            'timestamp': time.time()
        }
        
        # Короткая память
        self.short_memory.append(event)
        
        # Долгая память (только важные события)
        if important:
            self.long_memory.append(event)
        
        # Очистка старых событий из короткой памяти
        cutoff_time = time.time() - self.memory_window
        self.short_memory = [
            e for e in self.short_memory 
            if e['timestamp'] > cutoff_time
        ]
    
    def get_memory_context(self) -> str:
        """Получить контекст из памяти для промпта"""
        lines = []
        for event in self.short_memory[-10:]:  # Последние 10 событий
            elapsed = time.time() - event['timestamp']
            lines.append(f"[{elapsed:.0f}s назад] {event['type']}: {event['content']}")
        return '\n'.join(lines) if lines else 'Память пуста'
    
    # ============================================================
    # Главный цикл размышлений
    # ============================================================
    
    def reflection_loop(self):
        """Главный цикл рефлексии"""
        # Проверяем не идёт ли диалог
        if self.in_dialogue:
            if self.last_user_speech_time:
                elapsed = time.time() - self.last_user_speech_time
                if elapsed > self.dialogue_timeout:
                    self.in_dialogue = False
                    self.get_logger().info('💬 Диалог завершён (тайм-аут)')
                else:
                    # Ждём пока диалог закончится
                    return
        
        # Диалога нет - можем размышлять и говорить
        self.think_and_maybe_speak()
    
    def think_and_maybe_speak(self):
        """Размышление и решение говорить или нет"""
        # Собираем текущий контекст
        context = self.build_context()
        
        if not context:
            return  # Нет данных для размышлений
        
        # Отправляем в DeepSeek для анализа
        thought, should_speak, speech_text = self.analyze_context(context)
        
        # Публикуем внутреннюю мысль
        if thought:
            thought_msg = String()
            thought_msg.data = thought
            self.thought_pub.publish(thought_msg)
            self.get_logger().debug(f'💭 Мысль: {thought}')
        
        # Говорим, если решили
        if should_speak and speech_text and self.enable_speech:
            tts_msg = String()
            tts_msg.data = speech_text
            self.tts_pub.publish(tts_msg)
            self.get_logger().info(f'🗣️  Говорю: "{speech_text}"')
            self.add_to_memory('robot_reflection', speech_text, important=True)
    
    def build_context(self) -> Optional[Dict]:
        """Собрать текущий контекст из всех источников"""
        context = {
            'timestamp': time.time(),
            'vision': self.current_vision,
            'pose': None,
            'moving': False,
            'sensors': self.current_sensors,
            'apriltags': [tag.id for tag in self.last_apriltags] if hasattr(self, 'last_apriltags') else [],
            'memory': self.get_memory_context()
        }
        
        # Позиция
        if self.current_pose:
            context['pose'] = {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y
            }
        
        # Движение
        if self.current_odom:
            vx = abs(self.current_odom.twist.twist.linear.x)
            wz = abs(self.current_odom.twist.twist.angular.z)
            context['moving'] = vx > 0.01 or wz > 0.01
        
        return context if any([
            self.current_vision,
            self.current_pose,
            self.current_sensors,
            len(self.last_apriltags) > 0
        ]) else None
    
    def analyze_context(self, context: Dict) -> tuple[Optional[str], bool, Optional[str]]:
        """
        Анализ контекста через DeepSeek API
        
        Returns:
            (internal_thought, should_speak, speech_text)
        """
        if not self.deepseek_api_key:
            # Заглушка без API
            return self._stub_analyze(context)
        
        # TODO: Реальный вызов DeepSeek API
        # Пока используем заглушку
        return self._stub_analyze(context)
    
    def _stub_analyze(self, context: Dict) -> tuple[Optional[str], bool, Optional[str]]:
        """Заглушка для анализа (без DeepSeek API)"""
        # Простая логика для тестирования
        
        thought = None
        should_speak = False
        speech_text = None
        
        # Проверяем батарею
        if context['sensors'] and context['sensors'].get('battery'):
            battery = context['sensors']['battery']
            if battery < 11.0:  # Низкий заряд
                thought = f"Батарея низкая: {battery:.1f}V"
                should_speak = True
                speech_text = f"Внимание! Батарея разряжена: {battery:.1f} вольт"
        
        # Проверяем AprilTags
        if context['apriltags'] and len(context['apriltags']) > 0:
            tags = context['apriltags']
            thought = f"Вижу маркеры: {tags}"
            # Не говорим каждый раз, только если это новое
            
        # Проверяем движение
        if context['moving']:
            thought = "Я еду"
        else:
            thought = "Я стою на месте"
        
        return thought, should_speak, speech_text


def main(args=None):
    rclpy.init(args=args)
    node = ReflectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
