#!/usr/bin/env python3
"""
reflection_node.py - Internal Dialogue Agent (Event-Driven)

НОВАЯ АРХИТЕКТУРА (v2.0):
Разделение ответственности с context_aggregator_node:

context_aggregator (MPC lite) → events → reflection_node → thoughts/speech

Подписывается на:
- /perception/context_update (PerceptionEvent) - агрегированный контекст
- /perception/user_speech (String) - речь пользователя

Публикует:
- /reflection/internal_thought (String) - внутренние мысли
- /voice/tts/request (String) - речь робота

Особенности:
1. Event-driven: размышляет при получении события (не по таймеру)
2. Hook для срочных ответов: личные вопросы → быстрый ответ
3. НЕ собирает данные - только думает и решает
"""

import json
import time
import re
from typing import Optional, Dict, List
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

try:
    from rob_box_perception.msg import PerceptionEvent
except ImportError:
    PerceptionEvent = None


class ReflectionNode(Node):
    """Internal Dialogue Agent - Event-Driven"""
    
    def __init__(self):
        super().__init__('reflection_node')
        
        # ============ Параметры ============
        self.declare_parameter('dialogue_timeout', 10.0)  # секунд
        self.declare_parameter('enable_speech', True)
        self.declare_parameter('system_prompt_file', 'reflection_prompt.txt')
        self.declare_parameter('urgent_response_timeout', 2.0)  # секунд для срочного ответа
        
        self.dialogue_timeout = self.get_parameter('dialogue_timeout').value
        self.enable_speech = self.get_parameter('enable_speech').value
        self.urgent_response_timeout = self.get_parameter('urgent_response_timeout').value
        
        # ============ Состояние диалога ============
        self.in_dialogue = False
        self.last_user_speech_time = None
        self.pending_user_speech: Optional[str] = None  # Ожидающий срочный вопрос
        
        # ============ Silence Mode (команда "помолчи") ============
        self.silence_until: Optional[float] = None  # Timestamp до которого молчать
        
        # ============ Последний контекст ============
        self.last_context: Optional[PerceptionEvent] = None
        
        # ============ Память размышлений ============
        self.recent_thoughts: List[str] = []  # Последние 10 мыслей
        
        # ============ DeepSeek API ============
        self.deepseek_api_key = os.getenv('DEEPSEEK_API_KEY')
        self.deepseek_client = None
        
        if not self.deepseek_api_key:
            self.get_logger().warn('⚠️  DEEPSEEK_API_KEY не найден! Используется заглушка.')
        elif not OPENAI_AVAILABLE:
            self.get_logger().warn('⚠️  OpenAI библиотека не установлена!')
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
        
        # ============ Подписки (Event-Driven) ============
        
        if PerceptionEvent:
            self.context_sub = self.create_subscription(
                PerceptionEvent,
                '/perception/context_update',
                self.on_context_update,
                10
            )
        else:
            self.get_logger().error('❌ PerceptionEvent message не найден! Соберите пакет.')
        
        self.speech_sub = self.create_subscription(
            String,
            '/perception/user_speech',
            self.on_user_speech,
            10
        )
        
        # Подписка на ответы робота (для отслеживания диалога)
        self.dialogue_sub = self.create_subscription(
            String,
            '/voice/dialogue/response',
            self.on_robot_response,
            10
        )
        
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
        
        self.get_logger().info('🧠 Reflection Node v2.0 (Event-Driven) запущен')
        self.get_logger().info(f'   Тайм-аут диалога: {self.dialogue_timeout} сек')
        self.get_logger().info(f'   Срочный ответ: {self.urgent_response_timeout} сек')
    
    def _load_system_prompt(self) -> str:
        """Загрузить system prompt из файла"""
        prompt_file = self.get_parameter('system_prompt_file').value
        
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
            return self._get_fallback_prompt()
    
    def _get_fallback_prompt(self) -> str:
        """Fallback system prompt"""
        return """Ты - внутренний голос робота РобБокс.

Твоя задача:
1. Анализировать контекст (датчики, камера, позиция, память, здоровье)
2. Генерировать внутренние мысли (рефлексия, гипотезы, наблюдения)
3. РЕШАТЬ: говорить вслух или молчать

Правила речи:
- Говори ТОЛЬКО если есть важная информация
- НЕ болтай просто так
- Говори при: низкой батарее, важном событии, личном вопросе

Формат ответа JSON:
{
  "thought": "внутренняя мысль (всегда)",
  "should_speak": false/true,
  "speech": "текст речи (если should_speak=true)"
}
"""
    
    # ============================================================
    # Callbacks - Events
    # ============================================================
    
    def on_context_update(self, msg: PerceptionEvent):
        """Получено событие обновления контекста"""
        self.last_context = msg
        
        # Если есть ожидающий срочный вопрос - обрабатываем немедленно
        if self.pending_user_speech:
            self.get_logger().info('⚡ Срочный ответ на личный вопрос')
            self.process_urgent_question(self.pending_user_speech)
            self.pending_user_speech = None
            return
        
        # Если диалог активен - не размышляем (ждём тайм-аута)
        if self.in_dialogue:
            if self.last_user_speech_time:
                elapsed = time.time() - self.last_user_speech_time
                if elapsed > self.dialogue_timeout:
                    self.in_dialogue = False
                    self.get_logger().info('💬 Диалог завершён (тайм-аут)')
                else:
                    return
        
        # Обычное размышление
        self.think_and_maybe_speak()
    
    def on_user_speech(self, msg: String):
        """Получена речь пользователя"""
        text = msg.data.strip()
        if not text:
            return
        
        self.get_logger().info(f'👤 Пользователь: "{text}"')
        
        # ПРИОРИТЕТ: Проверка стоп-слов ("помолчи")
        if self._is_silence_command(text):
            self.get_logger().warn('🔇 КОМАНДА SILENCE: робот будет молчать 5 минут')
            self.silence_until = time.time() + 300  # 5 минут
            # НЕ публикуем в TTS - просто устанавливаем флаг
            return
        
        self.in_dialogue = True
        self.last_user_speech_time = time.time()
        
        # Проверка: личный вопрос? (hook для срочного ответа)
        if self._is_personal_question(text):
            self.get_logger().info('🎯 Обнаружен личный вопрос → срочный ответ')
            self.pending_user_speech = text
            
            # Если контекст уже есть - отвечаем сразу
            if self.last_context:
                self.process_urgent_question(text)
                self.pending_user_speech = None
        else:
            # Обычный вопрос - пусть dialogue_node обрабатывает
            self.get_logger().debug('💬 Обычный вопрос → dialogue_node')
    
    def on_robot_response(self, msg: String):
        """Робот ответил"""
        self.get_logger().debug(f'🤖 Робот: {msg.data[:50]}...')
        self.last_user_speech_time = time.time()  # Обновляем время
    
    # ============================================================
    # Личные вопросы (Hook для срочных ответов)
    # ============================================================
    
    def _is_personal_question(self, text: str) -> bool:
        """Проверка: личный вопрос о состоянии робота?"""
        text_lower = text.lower()
        
        personal_patterns = [
            r'\bкак\s+(у\s+тебя\s+)?дела\b',
            r'\bкак\s+ты\b',
            r'\bчто\s+у\s+тебя\b',
            r'\bкак\s+твоё?\s+(настроение|самочувствие)\b',
            r'\bчто\s+(делаешь|происходит)\b',
            r'\bкак\s+себя\s+чувствуешь\b',
        ]
        
        for pattern in personal_patterns:
            if re.search(pattern, text_lower):
                return True
        
        return False
    
    def _is_silence_command(self, text: str) -> bool:
        """Проверка: команда замолчать?"""
        text_lower = text.lower()
        
        silence_patterns = [
            r'\bпомолч',    # помолчи, помолчите
            r'\bзамолч',    # замолчи, замолчите
            r'\bхватит\b',
            r'\bзакрой',    # закройся
            r'\bзаткн',     # заткнись, заткнитесь
            r'\bне\s+меша',  # не мешай
        ]
        
        for pattern in silence_patterns:
            if re.search(pattern, text_lower):
                return True
        
        return False
    
    def process_urgent_question(self, question: str):
        """Обработка срочного личного вопроса"""
        if not self.last_context:
            self.get_logger().warn('⚠️  Нет контекста для срочного ответа')
            return
        
        # Формируем специальный промпт для быстрого ответа
        context_summary = self._format_context_summary(self.last_context)
        
        prompt = f"""СРОЧНЫЙ ЛИЧНЫЙ ВОПРОС: "{question}"

{context_summary}

Дай КОРОТКИЙ (1-2 предложения) естественный ответ на вопрос пользователя.
Упомяни текущее состояние если релевантно.

Формат JSON:
{{
  "thought": "внутренняя оценка ситуации",
  "speech": "короткий ответ пользователю"
}}
"""
        
        # Вызов AI
        result = self._call_deepseek(prompt, urgent=True)
        
        if result:
            thought = result.get('thought', '')
            speech = result.get('speech', '')
            
            if thought:
                self._publish_thought(thought)
            
            if speech and self.enable_speech:
                self._publish_speech(speech)
    
    # ============================================================
    # Обычное размышление
    # ============================================================
    
    def think_and_maybe_speak(self):
        """Обычный цикл размышлений"""
        if not self.last_context:
            self.get_logger().debug('⏸️  Нет контекста для размышлений')
            return
        
        context_text = self._format_context_for_prompt(self.last_context)
        
        # Вызов AI
        result = self._call_deepseek(context_text, urgent=False)
        
        if result:
            thought = result.get('thought', '')
            should_speak = result.get('should_speak', False)
            speech = result.get('speech', '')
            
            # Публикуем мысль
            if thought:
                self._publish_thought(thought)
                self.get_logger().info(f'🧠 Размышление: {thought}')
            
            # Говорим если решили
            if should_speak and speech and self.enable_speech:
                self._publish_speech(speech)
                self.get_logger().info(f'🗣️  Говорю: "{speech}"')
    
    # ============================================================
    # Форматирование контекста
    # ============================================================
    
    def _format_context_summary(self, ctx: PerceptionEvent) -> str:
        """Краткое резюме контекста (для срочных ответов)"""
        lines = ["=== ТЕКУЩЕЕ СОСТОЯНИЕ ==="]
        
        # Health
        lines.append(f"Здоровье: {ctx.system_health_status}")
        if ctx.health_issues:
            lines.append(f"Проблемы: {', '.join(ctx.health_issues)}")
        
        # Battery
        if ctx.battery_voltage > 0:
            lines.append(f"🔋 Батарея: {ctx.battery_voltage:.1f}V")
        
        # Moving
        if ctx.is_moving:
            lines.append("🚗 Статус: Еду")
        else:
            lines.append("🚗 Статус: Стою")
        
        return '\n'.join(lines)
    
    def _format_context_for_prompt(self, ctx: PerceptionEvent) -> str:
        """Полный контекст для обычного размышления"""
        lines = ["=== ТЕКУЩИЙ КОНТЕКСТ РОБОТА ===", ""]
        
        # Vision
        if ctx.vision_context:
            try:
                vision = json.loads(ctx.vision_context)
                lines.append(f"📸 Камера: {vision.get('description', 'N/A')}")
            except:
                lines.append("📸 Камера: ошибка парсинга")
        else:
            lines.append("📸 Камера: нет данных")
        
        # Pose
        if ctx.pose.position.x != 0 or ctx.pose.position.y != 0:
            lines.append(f"📍 Позиция: ({ctx.pose.position.x:.2f}, {ctx.pose.position.y:.2f})")
        
        # Movement
        if ctx.is_moving:
            vx = ctx.velocity.linear.x
            wz = ctx.velocity.angular.z
            lines.append(f"🚗 Движение: v={vx:.2f} m/s, ω={wz:.2f} rad/s")
        else:
            lines.append("🚗 Статус: Стою на месте")
        
        # Sensors
        if ctx.battery_voltage > 0:
            lines.append(f"🔋 Батарея: {ctx.battery_voltage:.1f}V")
        if ctx.temperature > 0:
            lines.append(f"🌡️  Температура: {ctx.temperature:.1f}°C")
        
        # AprilTags
        if ctx.apriltag_ids:
            lines.append(f"🏷️  AprilTags: {ctx.apriltag_ids}")
        
        # System Health
        lines.append("")
        lines.append("=== ЗДОРОВЬЕ СИСТЕМЫ ===")
        lines.append(f"Статус: {ctx.system_health_status}")
        if ctx.health_issues:
            for issue in ctx.health_issues:
                lines.append(f"⚠️  {issue}")
        
        # Memory
        if ctx.memory_summary:
            lines.append("")
            lines.append("=== НЕДАВНИЕ СОБЫТИЯ ===")
            lines.append(ctx.memory_summary)
        
        return '\n'.join(lines)
    
    # ============================================================
    # DeepSeek API
    # ============================================================
    
    def _call_deepseek(self, prompt: str, urgent: bool = False) -> Optional[Dict]:
        """Вызов DeepSeek API"""
        if not self.deepseek_client:
            return self._stub_response(urgent)
        
        try:
            response = self.deepseek_client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=150 if urgent else 200,
                response_format={"type": "json_object"}
            )
            
            result = json.loads(response.choices[0].message.content)
            
            thought = result.get('thought', '')[:80]
            self.get_logger().info(f'🤖 AI: thought="{thought}..."')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка DeepSeek API: {e}')
            return self._stub_response(urgent)
    
    def _stub_response(self, urgent: bool) -> Dict:
        """Заглушка без API"""
        if urgent:
            return {
                'thought': 'Срочный вопрос обнаружен',
                'speech': 'У меня всё отлично, готов к работе!'
            }
        else:
            return {
                'thought': 'Стою на месте, всё спокойно',
                'should_speak': False,
                'speech': ''
            }
    
    # ============================================================
    # Публикации
    # ============================================================
    
    def _publish_thought(self, thought: str):
        """Публикация внутренней мысли"""
        msg = String()
        msg.data = thought
        self.thought_pub.publish(msg)
        
        # Сохраняем в историю
        self.recent_thoughts.append(thought)
        if len(self.recent_thoughts) > 10:
            self.recent_thoughts.pop(0)
    
    def _publish_speech(self, speech: str):
        """Публикация речи в TTS"""
        # Проверка: silence mode активен?
        if self.silence_until and time.time() < self.silence_until:
            remaining = int(self.silence_until - time.time())
            self.get_logger().debug(f'🔇 Silence mode: не говорю (осталось {remaining} сек)')
            return  # НЕ публикуем речь
        
        msg = String()
        msg.data = speech
        self.tts_pub.publish(msg)


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
