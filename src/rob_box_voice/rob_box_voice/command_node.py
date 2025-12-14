#!/usr/bin/env python3
"""
CommandNode - распознавание голосовых команд для управления роботом
Подписывается: /voice/stt/result (String)
Публикует: /voice/command/intent (String), /voice/command/feedback (String)
Action Clients: NavigateToPose, FollowPath
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import re
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum


class IntentType(Enum):
    """Типы намерений команд"""
    NAVIGATE = "navigate"           # Навигация к точке
    STOP = "stop"                   # Остановка
    FOLLOW = "follow"               # Следование
    STATUS = "status"               # Запрос статуса
    MAP = "map"                     # Работа с картой
    VISION = "vision"               # Зрение/детекция
    UNKNOWN = "unknown"             # Неизвестная команда


@dataclass
class Command:
    """Распознанная команда"""
    intent: IntentType
    text: str
    entities: Dict[str, any]
    confidence: float


class CommandNode(Node):
    """Нода для распознавания и выполнения голосовых команд"""
    
    def __init__(self):
        super().__init__('command_node')
        
        # Параметры
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('enable_navigation', True)
        self.declare_parameter('enable_follow', False)  # TODO: Phase 6
        self.declare_parameter('enable_vision', False)  # TODO: Phase 6
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.enable_navigation = self.get_parameter('enable_navigation').value
        self.enable_follow = self.get_parameter('enable_follow').value
        self.enable_vision = self.get_parameter('enable_vision').value
        
        # Subscribers
        self.stt_sub = self.create_subscription(
            String,
            '/voice/stt/result',
            self.stt_callback,
            10
        )
        
        # Subscribe to dialogue state (чтобы не мешать диалогу)
        self.dialogue_state_sub = self.create_subscription(
            String,
            '/voice/dialogue/state',
            self.dialogue_state_callback,
            10
        )
        
        # Publishers
        self.intent_pub = self.create_publisher(String, '/voice/command/intent', 10)
        self.feedback_pub = self.create_publisher(String, '/voice/command/feedback', 10)
        
        # Publisher для управления движением
        # Публикуем на /cmd_vel_voice (priority: 25 в twist_mux)
        # Приоритет ниже чем у оператора (joy:100, web:50) но выше Nav2 (10)
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_voice', 10)
        
        # State tracking
        self.dialogue_state = 'IDLE'  # IDLE | LISTENING | DIALOGUE | SILENCED
        
        # Action clients
        if self.enable_navigation:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.current_goal_handle = None  # Для отмены текущего goal
        
        # Словарь команд (паттерны)
        self._build_command_patterns()
        
        # Waypoints (заранее известные точки)
        self.waypoints = {
            'дом': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'кухня': {'x': 2.0, 'y': 1.0, 'theta': 0.0},
            'гостиная': {'x': 3.0, 'y': 2.0, 'theta': 1.57},
            'точка 1': {'x': 1.0, 'y': 0.0, 'theta': 0.0},
            'точка 2': {'x': 2.0, 'y': 0.0, 'theta': 0.0},
            'точка 3': {'x': 3.0, 'y': 0.0, 'theta': 0.0},
        }
        
        self.get_logger().info('✅ CommandNode инициализирован')
        self.get_logger().info(f'  Navigation: {"✓" if self.enable_navigation else "✗"}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
    
    def _build_command_patterns(self):
        """Построить паттерны распознавания команд"""
        self.patterns = {
            # Навигация
            IntentType.NAVIGATE: [
                (r'(двигайся|иди|поезжай|езжай|направляйся)\s+к\s+точке\s+(\d+)', 'waypoint_number'),
                (r'(двигайся|иди|поезжай|езжай)\s+к\s+(дом|кухня|гостиная)', 'waypoint_name'),
                # Движение с глаголом
                (r'(двигайся|иди|поезжай|езжай|катись|двигай)\s+(вперед|вперёд|назад|влево|вправо)', 'direction'),
                # Движение без глагола (просто направление)
                (r'^(вперед|вперёд|назад)$', 'direction'),
                # Повороты с глаголом
                (r'(поверни|повернись|разверн|развернись)\s+(налево|направо|влево|вправо)', 'turn'),
                # Повороты БЕЗ глагола (после удаления "робот")
                (r'^(налево|направо|влево|вправо)$', 'turn'),
            ],
            # Остановка
            IntentType.STOP: [
                (r'(стой|стоп|остановись|останови|halt|стоять|хватит|замри)', None),
                (r'(отмени|cancel)\s+(движение|навигацию|задание)', None),
            ],
            # Следование
            IntentType.FOLLOW: [
                (r'(следуй|иди)\s+за\s+(мной|человеком)', None),
                (r'(включи|активируй)\s+режим\s+следования', None),
            ],
            # Статус
            IntentType.STATUS: [
                (r'(где|куда)\s+(ты|робот)', None),
                (r'(покажи|расскажи)\s+(статус|положение|координаты)', None),
            ],
            # Карта
            IntentType.MAP: [
                (r'(покажи|открой|загрузи)\s+карту', None),
                (r'(создай|построй|сделай)\s+карту', None),
            ],
            # Зрение
            IntentType.VISION: [
                (r'что\s+(видишь|перед\s+тобой)', None),
                (r'(найди|покажи|обнаружь)\s+(объект|человека|предмет)', None),
            ],
        }
    
    def dialogue_state_callback(self, msg: String):
        """Callback для состояния dialogue_node"""
        self.dialogue_state = msg.data
        self.get_logger().debug(f'📊 Dialogue state: {self.dialogue_state}')
    
    def stt_callback(self, msg: String):
        """Обработка распознанной речи"""
        text = msg.data.strip().lower()
        if not text:
            return
        
        self.get_logger().info(f'🎤 STT: {text}')
        
        # Удалить wake word из начала команды
        wake_words = ['робот', 'робокс', 'робобокс']
        for wake_word in wake_words:
            if text.startswith(wake_word):
                text = text[len(wake_word):].strip()
                break
        
        # Распознать команду
        command = self.classify_intent(text)
        
        # Всегда публиковать intent (даже UNKNOWN) для dialogue_node
        self.publish_intent(command)
        
        if command.intent == IntentType.UNKNOWN:
            self.get_logger().debug(f'🤷 Неизвестная команда - пере даю dialogue_node: {text}')
            # Не выполняем команду, но публикуем intent=UNKNOWN для dialogue
            return
        
        if command.confidence < self.confidence_threshold:
            self.get_logger().warn(f'⚠️ Низкая уверенность: {command.confidence:.2f}')
            self.publish_feedback('Я не уверен, что правильно понял')
            return
        
        self.get_logger().info(f'🎯 Intent: {command.intent.value} ({command.confidence:.2f})')
        self.get_logger().info(f'📦 Entities: {command.entities}')
        
        # Выполнить команду
        self.execute_command(command)
    
    def classify_intent(self, text: str) -> Command:
        """Классифицировать намерение и извлечь сущности"""
        best_match = None
        best_confidence = 0.0
        best_intent = IntentType.UNKNOWN
        best_entities = {}
        
        # Проверить все паттерны
        for intent, patterns in self.patterns.items():
            for pattern, entity_type in patterns:
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    confidence = 0.8 + (len(match.group(0)) / len(text)) * 0.2
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_intent = intent
                        best_match = match
                        
                        # Извлечь сущности
                        if entity_type == 'waypoint_number':
                            best_entities = {'waypoint': f"точка {match.group(2)}"}
                        elif entity_type == 'waypoint_name':
                            best_entities = {'waypoint': match.group(2)}
                        elif entity_type == 'direction':
                            # Направление может быть в группе 1 (только направление) или 2 (с глаголом)
                            direction = match.group(2) if match.lastindex >= 2 else match.group(1)
                            best_entities = {'direction': direction}
                        elif entity_type == 'turn':
                            # Поворот может быть в группе 1 (только направление) или 2 (с глаголом)
                            direction = match.group(2) if match.lastindex >= 2 else match.group(1)
                            best_entities = {'direction': direction}
        
        return Command(
            intent=best_intent,
            text=text,
            entities=best_entities,
            confidence=best_confidence
        )
    
    def execute_command(self, command: Command):
        """Выполнить распознанную команду"""
        if command.intent == IntentType.NAVIGATE:
            self.handle_navigate(command)
        elif command.intent == IntentType.STOP:
            self.handle_stop(command)
        elif command.intent == IntentType.STATUS:
            self.handle_status(command)
        elif command.intent == IntentType.MAP:
            self.handle_map(command)
        elif command.intent == IntentType.VISION:
            self.handle_vision(command)
        elif command.intent == IntentType.FOLLOW:
            self.handle_follow(command)
    
    def handle_navigate(self, command: Command):
        """Обработка команды навигации"""
        if not self.enable_navigation:
            self.get_logger().warn('⚠️ Навигация отключена')
            self.publish_feedback('Навигация недоступна')
            return
        
        # Проверка на команду направления (поверни налево/направо)
        direction = command.entities.get('direction')
        if direction:
            self.handle_direction(direction)
            return
        
        waypoint_name = command.entities.get('waypoint')
        if not waypoint_name:
            self.publish_feedback('Не указана точка назначения')
            return
        
        # Найти координаты waypoint
        if waypoint_name not in self.waypoints:
            self.get_logger().warn(f'⚠️ Waypoint не найден: {waypoint_name}')
            self.publish_feedback(f'Я не знаю где находится {waypoint_name}')
            return
        
        coords = self.waypoints[waypoint_name]
        
        self.get_logger().info(f'🎯 Навигация к "{waypoint_name}": x={coords["x"]}, y={coords["y"]}')
        self.publish_feedback(f'Иду к {waypoint_name}')
        
        # Отправить Nav2 goal
        self.send_nav2_goal(coords['x'], coords['y'], coords['theta'])
    
    def send_nav2_goal(self, x: float, y: float, theta: float):
        """Отправить цель в Nav2 (абсолютные координаты в map frame)"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('❌ Nav2 action server недоступен')
            self.publish_feedback('Навигация недоступна')
            return
        
        # Создать goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Ориентация из угла theta
        import math
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Отправить goal
        self.get_logger().info(f'📤 Отправка Nav2 goal (map): ({x:.2f}, {y:.2f}, {theta:.2f})')
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def send_relative_nav2_goal(self, x: float, y: float, theta: float):
        """Отправить цель в Nav2 (относительно текущей позиции в base_link frame)
        
        Для команд типа "вперёд на метр", "повернись направо"
        Nav2 автоматически преобразует base_link в map через TF
        """
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('❌ Nav2 action server недоступен')
            self.publish_feedback('Навигация недоступна')
            return
        
        # Создать goal относительно текущей позиции
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'base_link'  # ← ОТНОСИТЕЛЬНЫЕ координаты!
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Ориентация из угла theta
        import math
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Отправить goal
        self.get_logger().info(f'📤 Отправка Nav2 goal (relative): ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)')
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Callback ответа Nav2 goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Nav2 goal отклонён')
            self.publish_feedback('Не могу выполнить навигацию')
            self.current_goal_handle = None
            return
        
        self.get_logger().info('✅ Nav2 goal принят')
        self.current_goal_handle = goal_handle  # Сохраняем для отмены
        
        # Ожидать результата
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Callback feedback Nav2"""
        feedback = feedback_msg.feedback
        # Можно опубликовать прогресс
        # self.get_logger().debug(f'Nav2 feedback: {feedback}')
    
    def nav_result_callback(self, future):
        """Callback результата Nav2"""
        result = future.result().result
        self.current_goal_handle = None  # Очищаем после завершения
        self.get_logger().info(f'✅ Nav2 завершён: {result}')
        self.publish_feedback('Прибыл в точку назначения')
    
    def handle_stop(self, command: Command):
        """Обработка команды остановки"""
        self.get_logger().info('🛑 Остановка')
        self.publish_feedback('Останавливаюсь')
        
        # Отменить ВСЕ Nav2 goals через cancel service
        if self.enable_navigation:
            try:
                from action_msgs.srv import CancelGoal
                from action_msgs.msg import GoalInfo
                
                # Создаём клиент для cancel service (если ещё нет)
                if not hasattr(self, 'cancel_client'):
                    self.cancel_client = self.create_client(
                        CancelGoal,
                        '/navigate_to_pose/_action/cancel_goal'
                    )
                
                if self.cancel_client.wait_for_service(timeout_sec=0.5):
                    # Пустой GoalInfo = отменить все goals
                    request = CancelGoal.Request()
                    request.goal_info = GoalInfo()  # Пустой = все goals
                    
                    self.get_logger().info('🛑 Отменяю все Nav2 goals...')
                    future = self.cancel_client.call_async(request)
                    future.add_done_callback(self.nav_cancel_callback)
                else:
                    self.get_logger().warn('⚠️ Nav2 cancel service недоступен')
            except Exception as e:
                self.get_logger().error(f'❌ Ошибка отмены: {e}')
        
        # Очистить наш локальный handle
        self.current_goal_handle = None
    
    def nav_cancel_callback(self, future):
        """Callback отмены Nav2 goal"""
        try:
            cancel_response = future.result()
            self.get_logger().info(f'✅ Nav2 goal отменён: {cancel_response}')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка отмены Nav2 goal: {e}')
        finally:
            self.current_goal_handle = None
    
    def handle_direction(self, direction: str):
        """Обработка команды поворота/движения в направлении
        
        Используем Nav2 NavigateToPose вместо прямого cmd_vel!
        Преимущества:
        - Точное расстояние через одометрию (closed-loop)
        - Автоматическое объезжание препятствий
        - Безопасная остановка при проблемах
        - Не зависит от duty cycle калибровки VESC
        """
        import math
        
        # Маппинг направлений: (x, y, theta, description)
        # Координаты относительно текущей позиции робота (base_link frame)
        direction_map = {
            'налево': (0.0, 0.0, math.pi/2, 'поворачиваю налево'),      # 90° влево
            'влево': (0.0, 0.0, math.pi/2, 'поворачиваю влево'),
            'направо': (0.0, 0.0, -math.pi/2, 'поворачиваю направо'),   # 90° вправо
            'вправо': (0.0, 0.0, -math.pi/2, 'поворачиваю вправо'),
            'вперед': (1.0, 0.0, 0.0, 'двигаюсь вперёд'),               # 1 метр вперёд
            'вперёд': (1.0, 0.0, 0.0, 'двигаюсь вперёд'),
            'назад': (-1.0, 0.0, 0.0, 'двигаюсь назад'),                # 1 метр назад
        }
        
        if direction not in direction_map:
            self.get_logger().warn(f'⚠️ Неизвестное направление: {direction}')
            self.publish_feedback(f'Не понимаю куда {direction}')
            return
        
        x, y, theta, feedback = direction_map[direction]
        
        self.get_logger().info(f'🎯 Команда направления: {feedback}')
        self.get_logger().info(f'   Относительная цель: x={x:.2f}м, y={y:.2f}м, theta={math.degrees(theta):.1f}°')
        self.publish_feedback(feedback.capitalize())
        
        # Отправить Nav2 goal относительно текущей позиции
        self.send_relative_nav2_goal(x, y, theta)
    
    def handle_status(self, command: Command):
        """Обработка запроса статуса"""
        self.get_logger().info('📊 Запрос статуса')
        # TODO: Получить текущую позицию из /odom или /tf
        self.publish_feedback('Я нахожусь в стартовой позиции')
    
    def handle_map(self, command: Command):
        """Обработка команд с картой"""
        self.get_logger().info('🗺️ Команда карты')
        self.publish_feedback('Функция карты в разработке')
    
    def handle_vision(self, command: Command):
        """Обработка команд зрения"""
        if not self.enable_vision:
            self.get_logger().warn('⚠️ Зрение отключено')
            self.publish_feedback('Функция зрения недоступна')
            return
        
        self.get_logger().info('👁️ Команда зрения')
        # TODO: Object detection
        self.publish_feedback('Сканирую окружение')
    
    def handle_follow(self, command: Command):
        """Обработка команды следования"""
        if not self.enable_follow:
            self.get_logger().warn('⚠️ Следование отключено')
            self.publish_feedback('Функция следования недоступна')
            return
        
        self.get_logger().info('🚶 Режим следования')
        # TODO: Person following
        self.publish_feedback('Включаю режим следования')
    
    def publish_intent(self, command: Command):
        """Публикация распознанного намерения"""
        msg = String()
        msg.data = f'{command.intent.value}:{command.confidence:.2f}'
        self.intent_pub.publish(msg)
    
    def publish_feedback(self, text: str):
        """Публикация feedback пользователю"""
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'💬 Feedback: {text}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
