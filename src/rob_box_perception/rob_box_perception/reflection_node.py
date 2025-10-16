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
from rcl_interfaces.msg import Log  # /rosout логи

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

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
        self.declare_parameter('system_prompt_file', 'reflection_prompt.txt')  # Файл с system prompt
        
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
        
        # ============ Мониторинг здоровья системы ============
        self.recent_errors: List[Dict] = []  # Последние ошибки (max 10)
        self.recent_warnings: List[Dict] = []  # Последние предупреждения (max 5)
        self.health_issues: List[str] = []  # Текущие проблемы
        self.last_topic_check = time.time()
        self.topic_check_interval = 10.0  # Проверять топики каждые 10 секунд
        
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
        
        # ============ Подписки - Мониторинг системы ============
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.on_rosout,
            50  # Большая очередь для логов
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
        self.deepseek_client = None
        
        if not self.deepseek_api_key:
            self.get_logger().warn('⚠️  DEEPSEEK_API_KEY не найден! Используется заглушка.')
        elif not OPENAI_AVAILABLE:
            self.get_logger().warn('⚠️  OpenAI библиотека не установлена! pip install openai')
        else:
            try:
                self.deepseek_client = OpenAI(
                    api_key=self.deepseek_api_key,
                    base_url="https://api.deepseek.com"
                )
                self.get_logger().info('✅ DeepSeek API клиент инициализирован')
            except Exception as e:
                self.get_logger().error(f'❌ Ошибка инициализации DeepSeek: {e}')
        
        # Загрузка системного промпта
        self.system_prompt = self._load_system_prompt()
        
        self.get_logger().info('🧠 Reflection Node запущен')
        self.get_logger().info(f'   Частота размышлений: {self.reflection_rate} Hz')
        self.get_logger().info(f'   Тайм-аут диалога: {self.dialogue_timeout} сек')
        self.get_logger().info(f'   Окно памяти: {self.memory_window} сек')
    
    def _load_system_prompt(self) -> str:
        """Загрузить system prompt из файла"""
        prompt_file = self.get_parameter('system_prompt_file').value
        
        # Ищем в share/rob_box_perception/prompts/
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('rob_box_perception')
            prompt_path = os.path.join(pkg_share, 'prompts', prompt_file)
            
            with open(prompt_path, 'r', encoding='utf-8') as f:
                prompt = f.read()
            
            self.get_logger().info(f'✅ Загружен prompt: {prompt_file} ({len(prompt)} байт)')
            return prompt
        except Exception as e:
            self.get_logger().warn(f'⚠️  Не удалось загрузить prompt: {e}')
            # Fallback на встроенный промпт
            return """Ты - внутренний голос робота РобБокс. 

Твоя задача:
1. Анализировать контекст (датчики, камера, позиция, память, здоровье системы)
2. Генерировать внутренние мысли (рефлексия, гипотезы, наблюдения)
3. РЕШАТЬ: говорить вслух или молчать

Правила речи:
- Говори ТОЛЬКО если есть важная информация или вопрос
- НЕ болтай просто так
- НЕ комментируй очевидное ("я стою", "я вижу стену")
- Говори при: низкой батарее, обнаружении человека, важном событии
- ОБЯЗАТЕЛЬНО сообщай о технических проблемах (ошибки, сбои нод, отсутствие данных)
- Будь лаконичным и дружелюбным

Мониторинг системы:
- system_health.status: 'healthy' | 'degraded' | 'critical'
- system_health.issues: список текущих проблем
- system_health.recent_errors: ошибки от других нод
- Если status='critical' - ОБЯЗАТЕЛЬНО сообщи пользователю
- Если много ошибок - предложи помощь

Формат ответа JSON:
{
  "thought": "внутренняя мысль для логов",
  "should_speak": true/false,
  "speech": "текст для произнесения (если should_speak=true)"
}"""
    
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
    # Callbacks - Мониторинг системы
    # ============================================================
    
    def on_rosout(self, msg: Log):
        """Получен лог от любой ноды через /rosout"""
        # Log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
        
        # Игнорируем свои собственные логи
        if msg.name == self.get_name():
            return
        
        # Обрабатываем только ERROR и WARN
        if msg.level >= 40:  # ERROR or FATAL
            error_info = {
                'node': msg.name,
                'level': 'ERROR' if msg.level == 40 else 'FATAL',
                'message': msg.msg,
                'timestamp': time.time()
            }
            self.recent_errors.append(error_info)
            
            # Оставляем только последние 10 ошибок
            if len(self.recent_errors) > 10:
                self.recent_errors = self.recent_errors[-10:]
            
            self.get_logger().warn(f'⚠️  [{error_info["node"]}] {error_info["level"]}: {msg.msg}')
            
            # Критичные ошибки добавляем в память
            if 'fail' in msg.msg.lower() or 'crash' in msg.msg.lower():
                self.add_to_memory('system_error', f'{msg.name}: {msg.msg}', important=True)
        
        elif msg.level == 30:  # WARN
            warning_info = {
                'node': msg.name,
                'message': msg.msg,
                'timestamp': time.time()
            }
            self.recent_warnings.append(warning_info)
            
            # Оставляем только последние 5 предупреждений
            if len(self.recent_warnings) > 5:
                self.recent_warnings = self.recent_warnings[-5:]
    
    def check_system_health(self) -> Dict:
        """
        Проверка здоровья системы
        
        Возвращает:
        {
            'status': 'healthy' | 'degraded' | 'critical',
            'issues': List[str],
            'recent_errors': List[Dict],
            'recent_warnings': List[Dict]
        }
        """
        issues = []
        status = 'healthy'
        
        # Проверка критичных ошибок за последние 30 секунд
        recent_critical = [
            e for e in self.recent_errors
            if time.time() - e['timestamp'] < 30 and e['level'] == 'FATAL'
        ]
        
        if len(recent_critical) > 0:
            status = 'critical'
            issues.append(f'🚨 {len(recent_critical)} критичных ошибок за 30 сек')
        
        # Проверка обычных ошибок
        recent_errors = [
            e for e in self.recent_errors
            if time.time() - e['timestamp'] < 60
        ]
        
        if len(recent_errors) >= 5:
            status = 'degraded' if status == 'healthy' else status
            issues.append(f'⚠️  {len(recent_errors)} ошибок за последнюю минуту')
        
        # Проверка активности важных топиков (каждые 10 секунд)
        if time.time() - self.last_topic_check >= self.topic_check_interval:
            self.last_topic_check = time.time()
            
            # Проверяем, когда последний раз обновлялись важные данные
            if self.current_vision is None:
                issues.append('👁️  Нет данных с камеры')
                status = 'degraded' if status == 'healthy' else status
            
            if self.current_odom is None:
                issues.append('🚗 Нет одометрии')
                status = 'degraded' if status == 'healthy' else status
        
        return {
            'status': status,
            'issues': issues,
            'recent_errors': self.recent_errors[-3:],  # Последние 3 ошибки
            'recent_warnings': self.recent_warnings[-2:]  # Последние 2 предупреждения
        }
    
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
        # Проверка здоровья системы
        health = self.check_system_health()
        
        context = {
            'timestamp': time.time(),
            'vision': self.current_vision,
            'pose': None,
            'moving': False,
            'sensors': self.current_sensors,
            'apriltags': [tag.id for tag in self.last_apriltags] if hasattr(self, 'last_apriltags') else [],
            'memory': self.get_memory_context(),
            'system_health': health  # Добавляем информацию о здоровье системы
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
        if not self.deepseek_client:
            # Заглушка без API
            return self._stub_analyze(context)
        
        try:
            # Формируем контекст для промпта
            context_text = self._format_context_for_prompt(context)
            
            # Вызов DeepSeek API
            response = self.deepseek_client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": context_text}
                ],
                temperature=0.7,
                max_tokens=200,
                response_format={"type": "json_object"}
            )
            
            # Парсим ответ
            result = json.loads(response.choices[0].message.content)
            
            thought = result.get('thought', '')
            should_speak = result.get('should_speak', False)
            speech_text = result.get('speech', '')
            
            self.get_logger().debug(f'🤖 DeepSeek: thought="{thought}", speak={should_speak}')
            
            return thought, should_speak, speech_text
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка DeepSeek API: {e}')
            return self._stub_analyze(context)
    
    def _format_context_for_prompt(self, context: Dict) -> str:
        """Форматирование контекста для промпта"""
        lines = [
            "=== ТЕКУЩИЙ КОНТЕКСТ РОБОТА ===",
            ""
        ]
        
        # Vision
        if context.get('vision'):
            vision = context['vision']
            lines.append(f"📸 Камера: {vision.get('description', 'N/A')}")
            if vision.get('objects'):
                lines.append(f"   Объекты: {vision['objects']}")
        else:
            lines.append("📸 Камера: нет данных")
        
        # AprilTags
        if context.get('apriltags') and len(context['apriltags']) > 0:
            lines.append(f"🏷️  AprilTag маркеры: {context['apriltags']}")
        
        # Позиция
        if context.get('pose'):
            pos = context['pose']
            lines.append(f"📍 Позиция на карте: ({pos['x']:.2f}, {pos['y']:.2f})")
        
        # Движение
        if context.get('moving'):
            lines.append("🚗 Статус: Еду")
        else:
            lines.append("🚗 Статус: Стою на месте")
        
        # Сенсоры
        if context.get('sensors'):
            sensors = context['sensors']
            battery = sensors.get('battery', 'N/A')
            temp = sensors.get('temperature', 'N/A')
            lines.append(f"🔋 Батарея: {battery}V")
            lines.append(f"🌡️  Температура: {temp}°C")
        
        # Память
        lines.append("")
        lines.append("=== НЕДАВНИЕ СОБЫТИЯ ===")
        lines.append(context.get('memory', 'Память пуста'))
        
        return '\n'.join(lines)
    
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
