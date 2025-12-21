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

ВАЖНО: Речь пользователя НЕ включается в обычное размышление!
Она обрабатывается dialogue_node. Reflection размышляет ТОЛЬКО о:
- Состоянии систем (health, battery, sensors)
- Изменениях в окружении (vision, AprilTags, движение)
- Своих мыслях и ответах робота
"""

import json
import time
import re
from typing import Optional, Dict, List, Any
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
    from rob_box_perception_msgs.msg import PerceptionEvent
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
        self.declare_parameter('user_response_prompt_file', 'reflection_user_response_prompt.txt')
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
        
        # ============ Event State Tracking (State-Based Detection) ============
        # Отслеживание текущего состояния событий для edge detection
        self.event_states: Dict[str, Any] = {}  # event_name -> current state
        self.event_last_reaction: Dict[str, float] = {}  # event_name -> timestamp
        self.periodic_event_cooldown = 60.0  # 1 минута для periodic checks
        
        # ============ Speech Debounce (Legacy, will be replaced) ============
        self.last_speech_time: Optional[float] = None  # Когда последний раз говорили
        self.speech_debounce_interval = 30.0  # Не говорить чаще чем раз в 30 секунд
        
        # ============ LLM API (Qwen/DeepSeek) ============
        # Проверяем API ключи (приоритет: специфичные → унифицированный)
        self.llm_api_key = os.getenv('QWEN_API_KEY') or os.getenv('LLM_API_KEY')
        self.llm_base_url = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
        self.llm_model = "qwen-max"
        self.llm_provider = "Qwen"
        
        # Fallback на DeepSeek если Qwen недоступен
        if not self.llm_api_key:
            self.llm_api_key = os.getenv('DEEPSEEK_API_KEY')
            if self.llm_api_key:
                self.llm_base_url = "https://api.deepseek.com"
                self.llm_model = "deepseek-chat"
                self.llm_provider = "DeepSeek"
        
        self.deepseek_client = None
        
        if not self.llm_api_key:
            self.get_logger().warn('⚠️  QWEN_API_KEY или DEEPSEEK_API_KEY не найдены! Используется заглушка.')
        elif not OPENAI_AVAILABLE:
            self.get_logger().warn('⚠️  OpenAI библиотека не установлена!')
        else:
            try:
                self.deepseek_client = OpenAI(
                    api_key=self.llm_api_key,
                    base_url=self.llm_base_url
                )
                self.get_logger().info(f'✅ {self.llm_provider} API клиент инициализирован ({self.llm_model})')
            except Exception as e:
                self.get_logger().error(f'❌ Ошибка инициализации {self.llm_provider}: {e}')
        
        # Загрузка системного промпта
        self.system_prompt = self._load_system_prompt()
        self.user_response_prompt = self._load_user_response_prompt()
        
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
        
        self.sound_pub = self.create_publisher(
            String,
            '/voice/sound/trigger',
            10
        )
        
        # ============ Sound Debounce ============
        self.last_sound_time: Dict[str, float] = {}  # sound_name -> timestamp
        self.sound_debounce_interval = 10.0  # секунд между одинаковыми звуками
        
        self.get_logger().info('🧠 Reflection Node v2.0 (Event-Driven) запущен')
        self.get_logger().info(f'   Тайм-аут диалога: {self.dialogue_timeout} сек')
        self.get_logger().info(f'   Срочный ответ: {self.urgent_response_timeout} сек')
        self.get_logger().info(f'   🔇 Sound debounce: {self.sound_debounce_interval} сек')
    
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
    
    def _load_user_response_prompt(self) -> str:
        """Загрузить user response prompt из файла"""
        prompt_file = self.get_parameter('user_response_prompt_file').value
        
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('rob_box_perception')
            prompt_path = os.path.join(pkg_share, 'prompts', prompt_file)
            
            with open(prompt_path, 'r', encoding='utf-8') as f:
                prompt = f.read()
            
            self.get_logger().info(f'✅ Загружен user response prompt: {prompt_file} ({len(prompt)} байт)')
            return prompt
        except Exception as e:
            self.get_logger().warn(f'⚠️  Не удалось загрузить user response prompt: {e}')
            return self._get_fallback_user_response_prompt()
    
    def _get_fallback_user_response_prompt(self) -> str:
        """Fallback user response prompt"""
        return """Ответь на личный вопрос пользователя кратко (1-2 предложения).

Формат ответа JSON:
{
  "speech_ssml": "<speak>Текст с SSML<break time='300ms'/></speak>"
}
"""
    
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
        
        # ============ STATE-BASED EVENT DETECTION ============
        # Проверяем изменения в health status (edge detection)
        self._check_health_status_change(msg)
        
        # Обычное размышление (только если нет недавней речи)
        # Старая логика сохранена для других типов событий (vision, apriltag, etc.)
        # self.think_and_maybe_speak()  # ❌ ВРЕМЕННО ОТКЛЮЧЕНО - только health_status
    
    def _check_health_status_change(self, ctx: PerceptionEvent):
        """Проверка изменения health status (STATE-BASED DETECTION)"""
        current_health = ctx.system_health_status
        previous_health = self.event_states.get('health_status')
        
        current_time = time.time()
        
        # EDGE DETECTION: State changed (false→true or true→false)
        if previous_health != current_health:
            self.get_logger().info(f'📊 Health status changed: {previous_health} → {current_health}')
            
            # React to state change
            if current_health == 'HEALTHY' and previous_health in ['DEGRADED', 'UNHEALTHY', None]:
                # System recovered or first start
                if previous_health:  # Not first start
                    speech = "Отлично! Системы восстановлены, всё работает нормально."
                else:  # First start
                    speech = "Всё хорошо! Батарея полная, системы в норме, готов к работе."
                
                self._publish_speech(speech)
                self.get_logger().info(f'🗣️  Говорю (health recovery): "{speech}"')
            
            elif current_health == 'DEGRADED':
                speech = "Внимание! Обнаружены проблемы с системой."
                if ctx.health_issues:
                    speech += f" {', '.join(ctx.health_issues[:2])}"  # First 2 issues
                
                self._publish_speech(speech)
                self.get_logger().info(f'🗣️  Говорю (health degraded): "{speech}"')
            
            elif current_health == 'UNHEALTHY':
                speech = "Критическое состояние! Требуется внимание."
                self._publish_speech(speech)
                self.get_logger().info(f'🗣️  Говорю (health critical): "{speech}"')
            
            # Update state
            self.event_states['health_status'] = current_health
            self.event_last_reaction['health_status'] = current_time
        
        # PERIODIC CHECK: State unchanged for 1+ minute
        elif previous_health == current_health and previous_health:
            last_reaction = self.event_last_reaction.get('health_status', 0)
            time_since_reaction = current_time - last_reaction
            
            # Only react if 1 minute passed AND status is not healthy (don't spam "all good")
            if time_since_reaction > self.periodic_event_cooldown and current_health != 'HEALTHY':
                self.get_logger().info(f'⏰ Periodic check: health still {current_health} after {time_since_reaction:.0f}s')
                
                if current_health == 'DEGRADED':
                    speech = "Проблемы с системой всё ещё не устранены."
                elif current_health == 'UNHEALTHY':
                    speech = "Критическое состояние продолжается!"
                else:
                    return  # Don't speak for other states
                
                self._publish_speech(speech)
                self.event_last_reaction['health_status'] = current_time

    
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
        
        user_prompt = f"""Вопрос пользователя: "{question}"

{context_summary}

Сформируй короткий ответ (1-2 предложения) в формате SSML."""
        
        # Вызов AI с user response prompt
        result = self._call_deepseek_user_response(user_prompt)
        
        if result:
            speech_ssml = result.get('speech_ssml', '')
            
            if speech_ssml and self.enable_speech:
                self._publish_speech_ssml(speech_ssml)
    
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
        
        # Time (NEW)
        if hasattr(ctx, 'current_time_human') and ctx.current_time_human:
            try:
                time_data = json.loads(ctx.time_context_json)
                lines.append(f"🕐 Сейчас: {time_data.get('period_ru', ctx.time_period)}, {ctx.current_time_human}")
            except:
                lines.append(f"🕐 Время: {ctx.current_time_human}")
        
        # Health
        lines.append(f"Здоровье: {ctx.system_health_status}")
        if ctx.health_issues:
            lines.append(f"Проблемы: {', '.join(ctx.health_issues)}")
        
        # Internet status (NEW)
        if hasattr(ctx, 'internet_available'):
            internet_status = "доступен" if ctx.internet_available else "недоступен"
            lines.append(f"🌐 Интернет: {internet_status}")
        
        # Node status (NEW)
        if hasattr(ctx, 'failed_nodes') and ctx.failed_nodes:
            lines.append(f"⚠️  Упавшие ноды: {', '.join(ctx.failed_nodes)}")
        
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
        
        # ============ НОВОЕ: Time Context ============
        if hasattr(ctx, 'current_time_human') and ctx.current_time_human:
            lines.append("=== ВРЕМЯ ===")
            lines.append(f"🕐 Время: {ctx.current_time_human}")
            lines.append(f"📅 Период: {ctx.time_period}")
            
            # Парсим JSON для более детальной информации
            try:
                time_data = json.loads(ctx.time_context_json)
                lines.append(f"   {time_data.get('weekday_ru', '')}, {time_data.get('period_ru', '')}")
            except:
                pass
            lines.append("")
        
        # ============ НОВОЕ: Internet and Node Status ============
        if hasattr(ctx, 'internet_available'):
            internet_emoji = "✅" if ctx.internet_available else "❌"
            internet_text = "Доступен" if ctx.internet_available else "Недоступен"
            lines.append(f"🌐 Интернет: {internet_emoji} {internet_text}")
        
        if hasattr(ctx, 'active_nodes') and (ctx.active_nodes or ctx.failed_nodes or ctx.missing_nodes):
            lines.append(f"📡 Ноды: {len(ctx.active_nodes)} активных")
            
            if ctx.failed_nodes:
                lines.append(f"   ❌ Упавшие: {', '.join(ctx.failed_nodes)}")
            
            if ctx.missing_nodes:
                lines.append(f"   ⚠️  Отсутствуют: {', '.join(ctx.missing_nodes)}")
        
        lines.append("")
        
        # ВАЖНО: Последние мысли (для избежания повторений)
        if self.recent_thoughts:
            lines.append("=== МОИ ПОСЛЕДНИЕ МЫСЛИ (для контекста) ===")
            for i, thought in enumerate(self.recent_thoughts[-5:], 1):  # Последние 5
                lines.append(f"{i}. {thought}")
            lines.append("")
        
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
        
        # Summarized History (суммаризованная история по типам)
        lines.append("")
        lines.append("=== СУММАРИЗОВАННАЯ ИСТОРИЯ ===")
        
        if ctx.speech_summaries and ctx.speech_summaries != '[]':
            try:
                speech_sums = json.loads(ctx.speech_summaries)
                if speech_sums:
                    lines.append("\n👤 ВОПРОСЫ ПОЛЬЗОВАТЕЛЯ:")
                    for s in speech_sums[-3:]:  # Последние 3
                        lines.append(f"  • {s['summary']}")
            except:
                pass
        
        if ctx.robot_response_summaries and ctx.robot_response_summaries != '[]':
            try:
                response_sums = json.loads(ctx.robot_response_summaries)
                if response_sums:
                    lines.append("\n🤖 МОИ ОТВЕТЫ:")
                    for s in response_sums[-3:]:  # Последние 3
                        lines.append(f"  • {s['summary']}")
            except:
                pass
        
        if ctx.robot_thought_summaries and ctx.robot_thought_summaries != '[]':
            try:
                thought_sums = json.loads(ctx.robot_thought_summaries)
                if thought_sums:
                    lines.append("\n🧠 МОИ РАЗМЫШЛЕНИЯ:")
                    for s in thought_sums[-3:]:  # Последние 3
                        lines.append(f"  • {s['summary']}")
            except:
                pass
        
        if ctx.vision_summaries and ctx.vision_summaries != '[]':
            try:
                vision_sums = json.loads(ctx.vision_summaries)
                if vision_sums:
                    lines.append("\n👁️  НАБЛЮДЕНИЯ:")
                    for s in vision_sums[-3:]:  # Последние 3
                        lines.append(f"  • {s['summary']}")
            except:
                pass
        
        if ctx.system_summaries and ctx.system_summaries != '[]':
            try:
                system_sums = json.loads(ctx.system_summaries)
                if system_sums:
                    lines.append("\n⚙️  СИСТЕМА:")
                    for s in system_sums[-3:]:  # Последние 3
                        lines.append(f"  • {s['summary']}")
            except:
                pass
        
        # Memory (недавние события ~10)
        if ctx.memory_summary:
            lines.append("")
            lines.append("=== НЕДАВНИЕ СОБЫТИЯ (последние ~10) ===")
            lines.append(ctx.memory_summary)
        
        return '\n'.join(lines)
    
    # ============================================================
    # DeepSeek API
    # ============================================================
    
    def _call_deepseek(self, prompt: str, urgent: bool = False) -> Optional[Dict]:
        """Вызов DeepSeek API для обычного размышления"""
        if not self.deepseek_client:
            return self._stub_response(urgent)
        
        try:
            response = self.deepseek_client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=150 if urgent else 200,
                response_format={"type": "json_object"},
                extra_body={
                    "enable_search": True  # Включаем веб-поиск для Qwen
                }
            )
            
            result = json.loads(response.choices[0].message.content)
            
            thought = result.get('thought', '')[:80]
            self.get_logger().info(f'🤖 AI: thought="{thought}..."')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка DeepSeek API: {e}')
            return self._stub_response(urgent)
    
    def _call_deepseek_user_response(self, user_prompt: str) -> Optional[Dict]:
        """Вызов DeepSeek API для ответа пользователю (с SSML)"""
        if not self.deepseek_client:
            return {
                'speech_ssml': '<speak>У мен+я вс+ё отл+ично!<break time="300ms"/>Гот+ов к раб+оте.<break time="400ms"/></speak>'
            }
        
        try:
            response = self.deepseek_client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": self.user_response_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=150,
                response_format={"type": "json_object"},
                extra_body={
                    "enable_search": True  # Включаем веб-поиск для Qwen
                }
            )
            
            result = json.loads(response.choices[0].message.content)
            
            speech_ssml = result.get('speech_ssml', '')
            self.get_logger().info(f'🤖 AI User Response: {speech_ssml[:100]}...')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка DeepSeek API (user response): {e}')
            return {
                'speech_ssml': '<speak>У мен+я вс+ё отл+ично!<break time="300ms"/>Гот+ов к раб+оте.<break time="400ms"/></speak>'
            }
    
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
        
        # Триггер звука на основе эмоции
        self._trigger_sound_for_thought(thought)
        
        # Сохраняем в историю
        self.recent_thoughts.append(thought)
        if len(self.recent_thoughts) > 10:
            self.recent_thoughts.pop(0)
    
    def _publish_speech(self, speech: str):
        """Публикация речи в TTS (в формате SSML) - для state-based events"""
        # Проверка: silence mode активен?
        if self.silence_until and time.time() < self.silence_until:
            remaining = int(self.silence_until - time.time())
            self.get_logger().debug(f'🔇 Silence mode: не говорю (осталось {remaining} сек)')
            return  # НЕ публикуем речь
        
        # ❌ NO DEBOUNCE - state-based logic controls when to speak!
        # Каждый вызов _publish_speech() уже прошёл проверку в _check_health_status_change()
        current_time = time.time()
        
        # Формируем JSON с SSML (как dialogue_node)
        import json
        response_json = {
            "ssml": f"<speak>{speech}</speak>"
        }
        
        msg = String()
        msg.data = json.dumps(response_json, ensure_ascii=False)
        self.tts_pub.publish(msg)
        
        # Обновляем время последней речи (для логирования)
        self.last_speech_time = current_time
    
    def _publish_speech_ssml(self, speech_ssml: str):
        """Публикация речи в TTS (уже в SSML формате) - для ответов пользователю"""
        # Проверка: silence mode активен?
        if self.silence_until and time.time() < self.silence_until:
            remaining = int(self.silence_until - time.time())
            self.get_logger().debug(f'🔇 Silence mode: не говорю (осталось {remaining} сек)')
            return  # НЕ публикуем речь
        
        # ❌ БЕЗ DEBOUNCE для пользовательских ответов!
        # Пользователь задал вопрос - должен получить ответ немедленно
        current_time = time.time()
        
        # Формируем JSON с готовым SSML
        import json
        response_json = {
            "ssml": speech_ssml
        }
        
        msg = String()
        msg.data = json.dumps(response_json, ensure_ascii=False)
        self.tts_pub.publish(msg)
        
        # Обновляем время последней речи
        self.last_speech_time = current_time
        
        self.get_logger().info(f'🗣️  Reflection → TTS: {speech_ssml[:80]}...')
    
    # ============================================================
    # Звуковые эффекты (эмоции)
    # ============================================================
    
    def _trigger_sound_for_thought(self, thought: str):
        """Триггер звука на основе эмоции размышления"""
        thought_lower = thought.lower()
        
        # Удивление при новой информации
        if any(word in thought_lower for word in ['удивительно', 'вау', 'неожиданно', 'странно', 'интересно']):
            self._play_sound('surprise')
        
        # Размышление при обдумывании
        elif any(word in thought_lower for word in ['думаю', 'размышляю', 'анализирую', 'рассматриваю', 'проверяю']):
            self._play_sound('thinking')
        
        # Замешательство при неопределённости
        elif any(word in thought_lower for word in ['не уверен', 'сложно', 'непонятно', 'затрудняюсь', 'не знаю']):
            self._play_sound('confused')
        
        # Злость только при КРИТИЧНЫХ проблемах (не degraded - это warning)
        elif any(word in thought_lower for word in ['критично', 'авария', 'критическая ошибка']):
            self._play_sound('angry')
        
        # Радость при успехе
        elif any(word in thought_lower for word in ['отлично', 'хорошо', 'успешно', 'готов', 'healthy', 'норма']):
            self._play_sound('cute')
    
    def _play_sound(self, sound_name: str):
        """Проиграть звуковой эффект (с debounce защитой от повторов)"""
        # Проверка debounce: не играть один и тот же звук слишком часто
        current_time = time.time()
        if sound_name in self.last_sound_time:
            time_since_last = current_time - self.last_sound_time[sound_name]
            if time_since_last < self.sound_debounce_interval:
                self.get_logger().debug(
                    f'🔇 Sound debounce: пропускаю {sound_name} '
                    f'(прошло {time_since_last:.1f}s < {self.sound_debounce_interval}s)'
                )
                return  # НЕ публикуем звук
        
        try:
            msg = String()
            msg.data = sound_name
            self.sound_pub.publish(msg)
            self.get_logger().debug(f'🎵 Звук: {sound_name}')
            
            # Обновляем время последнего воспроизведения этого звука
            self.last_sound_time[sound_name] = current_time
        except Exception as e:
            self.get_logger().warn(f'⚠️  Ошибка триггера звука: {e}')


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
