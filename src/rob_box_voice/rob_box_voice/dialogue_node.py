#!/usr/bin/env python3
"""
DialogueNode - LLM диалоговая система с DeepSeek API (streaming)

Подписывается на: /voice/stt/result (распознанная речь)
Публикует: /voice/dialogue/response (JSON chunks для TTS)
Использует: DeepSeek API streaming + accent_replacer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import json
import sys
import time
import re
from pathlib import Path

# Импортируем из scripts
scripts_path = Path(__file__).parent.parent / 'scripts'
sys.path.insert(0, str(scripts_path))

try:
    from openai import OpenAI
    from accent_replacer import AccentReplacer
except ImportError as e:
    print(f"❌ Ошибка импорта: {e}")
    print("Установите: pip install openai")
    sys.exit(1)


class DialogueNode(Node):
    """ROS2 нода для LLM диалога с DeepSeek"""
    
    def __init__(self):
        super().__init__('dialogue_node')
        
        # Параметры
        self.declare_parameter('api_key', '')
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        self.declare_parameter('model', 'deepseek-chat')
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_tokens', 500)
        self.declare_parameter('system_prompt_file', 'master_prompt_simple.txt')
        self.declare_parameter('wake_words', ['робок', 'робот', 'роббокс'])
        self.declare_parameter('silence_commands', ['помолч', 'замолч', 'хватит'])
        
        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.getenv('DEEPSEEK_API_KEY')
        
        if not api_key:
            self.get_logger().error('❌ DEEPSEEK_API_KEY не найден!')
            raise RuntimeError('DEEPSEEK_API_KEY required')
        
        base_url = self.get_parameter('base_url').value
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        
        # DeepSeek client
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        
        # Accent replacer
        self.accent_replacer = AccentReplacer()
        stats = self.accent_replacer.get_stats()
        self.get_logger().info(f'📖 Словарь ударений: {stats["total_words"]} слов')
        
        # System prompt
        self.system_prompt = self._load_system_prompt()
        
        # История диалога
        self.conversation_history = []
        
        # Подписка на распознанную речь
        self.stt_sub = self.create_subscription(
            String,
            '/voice/stt/result',
            self.stt_callback,
            10
        )
        
        # Подписка на feedback от command_node (Phase 5)
        self.command_feedback_sub = self.create_subscription(
            String,
            '/voice/command/feedback',
            self.command_feedback_callback,
            10
        )
        
        # Публикация ответов (JSON chunks)
        self.response_pub = self.create_publisher(String, '/voice/dialogue/response', 10)
        
        # Публикация в TTS для синтеза (Phase 6 - добавлено!)
        self.tts_pub = self.create_publisher(String, '/voice/tts/request', 10)
        
        # Публикация звуковых триггеров (Phase 4)
        self.sound_trigger_pub = self.create_publisher(String, '/voice/sound/trigger', 10)
        
        # Публикация control commands в TTS
        self.tts_control_pub = self.create_publisher(String, '/voice/tts/control', 10)
        
        # Публикация state для других нод (command_node)
        self.state_pub = self.create_publisher(String, '/voice/dialogue/state', 10)
        
        # Публикация срочных запросов к внутреннему диалогу (reflection)
        self.reflection_request_pub = self.create_publisher(String, '/perception/user_speech', 10)
        
        # ============ State Machine ============
        # IDLE -> LISTENING -> DIALOGUE -> SILENCED
        self.state = 'IDLE'  # IDLE | LISTENING | DIALOGUE | SILENCED
        self.silence_until = None  # Timestamp когда закончится SILENCED
        self.last_interaction_time = time.time()
        self.dialogue_timeout = 30.0  # секунд без активности -> IDLE
        
        # Wake words и silence commands из параметров
        self.wake_words = self.get_parameter('wake_words').value
        self.silence_commands = self.get_parameter('silence_commands').value
        
        # Unsilence commands - команды для выхода из SILENCED режима
        self.unsilence_commands = [
            "говори",
            "включ",     # включись, включайся
            "работ",     # работай, работайте
            "отвеч",     # отвечай, отвечайте
            "разговар",  # разговаривай
        ]
        
        # Флаг что dialogue_node обработал запрос (чтобы игнорировать command feedback)
        self.dialogue_in_progress = False
        
        # Текущий streaming запрос (для прерывания)
        self.current_stream = None
        
        # Таймер для проверки dialogue timeout
        self.timeout_timer = self.create_timer(5.0, self._check_dialogue_timeout)
        
        self.get_logger().info('✅ DialogueNode инициализирован')
        self.get_logger().info(f'  Wake words: {", ".join(self.wake_words)}')
        self.get_logger().info(f'  Silence commands: {", ".join(self.silence_commands)}')
        self.get_logger().info(f'  Model: {self.model}')
        self.get_logger().info(f'  Temperature: {self.temperature}')
        self.get_logger().info(f'  Max tokens: {self.max_tokens}')
        self.get_logger().info(f'  Dialogue timeout: {self.dialogue_timeout}s')
    
    def _load_system_prompt(self) -> str:
        """Загрузить упрощённый system prompt"""
        prompt_file = self.get_parameter('system_prompt_file').value
        
        # Ищем в share/rob_box_voice/prompts/
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('rob_box_voice')
            prompt_path = os.path.join(pkg_share, 'prompts', prompt_file)
            
            with open(prompt_path, 'r', encoding='utf-8') as f:
                prompt = f.read()
            
            self.get_logger().info(f'✅ Загружен prompt: {prompt_file} ({len(prompt)} байт)')
            return prompt
        except Exception as e:
            self.get_logger().warn(f'⚠ Не удалось загрузить prompt: {e}')
            return "Ты ROBBOX - мобильный робот-ассистент. Отвечай в JSON: {\"ssml\": \"<speak>...</speak>\"}"
    
    # ============================================================
    # Wake Word & Silence Detection
    # ============================================================
    
    def _has_wake_word(self, text: str) -> bool:
        """Проверка наличия wake word"""
        for wake_word in self.wake_words:
            if wake_word in text:
                return True
        return False
    
    def _remove_wake_word(self, text: str) -> str:
        """Убрать wake word из текста"""
        for wake_word in self.wake_words:
            text = text.replace(wake_word, '').strip()
        return text
    
    def _is_silence_command(self, text: str) -> bool:
        """Проверка: команда замолчать?"""
        # Используем silence_commands из параметров
        for command in self.silence_commands:
            if command in text:
                return True
        
        return False
    
    def _is_unsilence_command(self, text: str) -> bool:
        """Проверка: команда выхода из silence режима?"""
        for command in self.unsilence_commands:
            if command in text:
                return True
        
        return False
    
    def _handle_silence_command(self):
        """Обработка команды silence"""
        self.get_logger().warn('🔇 SILENCE: останавливаем TTS и переходим в SILENCED')
        
        # 1. Прервать текущий streaming
        if self.current_stream:
            try:
                # Не можем прервать генератор, но можем установить флаг
                self.current_stream = None
            except Exception as e:
                self.get_logger().error(f'Ошибка прерывания stream: {e}')
        
        # 2. Отправить STOP в TTS
        stop_msg = String()
        stop_msg.data = 'STOP'
        self.tts_control_pub.publish(stop_msg)
        self.get_logger().info('  → STOP отправлен в TTS')
        
        # 3. Перейти в SILENCED на 5 минут
        self.state = 'SILENCED'
        self.silence_until = time.time() + 300  # 5 минут
        self._publish_state()
        self.get_logger().info('  → State: SILENCED (5 минут)')
        
        # 4. Короткое подтверждение (через TTS напрямую)
        self._speak_simple('Хорошо, молчу')
    
    def _speak_simple(self, text: str):
        """Простая речь без LLM"""
        response_json = {
            "ssml": f"<speak>{text}</speak>"
        }
        
        response_msg = String()
        response_msg.data = json.dumps(response_json, ensure_ascii=False)
        self.response_pub.publish(response_msg)
        self.tts_pub.publish(response_msg)
    
    def _publish_state(self):
        """Публикация текущего состояния dialogue_node"""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
    
    # ============================================================
    # Main Callback
    # ============================================================
    
    def stt_callback(self, msg: String):
        """Обработка распознанной речи с State Machine"""
        user_message = msg.data.strip()
        if not user_message:
            return
        
        user_message_lower = user_message.lower()
        self.get_logger().info(f'👤 User: {user_message} [State: {self.state}]')
        
        # ============ ПРИОРИТЕТ 1: Проверка SILENCE command ============
        if self._is_silence_command(user_message_lower):
            self.get_logger().warn('🔇 SILENCE COMMAND обнаружена!')
            self._handle_silence_command()
            return
        
        # ============ ПРИОРИТЕТ 2: Проверка SILENCED state ============
        if self.state == 'SILENCED':
            # В SILENCED: проверяем unsilence команды с wake word
            if self._has_wake_word(user_message_lower):
                # Проверяем: команда выхода из silence?
                if self._is_unsilence_command(user_message_lower):
                    self.get_logger().info('🔓 Unsilence command обнаружена → IDLE')
                    self.state = 'IDLE'
                    self.silence_until = None
                    self._publish_state()
                    self._speak_simple("Хорошо, слушаю!")
                    return
                else:
                    # Обычная команда с wake word в SILENCED
                    self.get_logger().info('🔓 Wake word в SILENCED → разрешаем ТОЛЬКО команды')
                    # TODO: передать в command_node для навигации/LED
                    # Пока просто логируем
                    self.get_logger().info('  → Команда должна быть обработана command_node')
                    return
            else:
                self.get_logger().debug('🔇 SILENCED: игнорируем (нет wake word)')
                return
        
        # ============ ПРИОРИТЕТ 3: Wake Word Detection ============
        if self.state == 'IDLE':
            # В IDLE: требуется wake word
            if self._has_wake_word(user_message_lower):
                self.get_logger().info('👋 Wake word обнаружен → LISTENING')
                self.state = 'LISTENING'
                self._publish_state()
                self.last_interaction_time = time.time()
                
                # Убираем wake word из текста
                user_message_clean = self._remove_wake_word(user_message_lower)
                if not user_message_clean:
                    # Только wake word без команды/вопроса
                    self._speak_simple("Слушаю!")
                    return
                
                user_message = user_message_clean
            else:
                self.get_logger().debug('⏸️  IDLE: игнорируем (нет wake word)')
                return
        
        # ============ State: LISTENING или DIALOGUE ============
        self.state = 'DIALOGUE'
        self._publish_state()
        self.last_interaction_time = time.time()
        self.dialogue_in_progress = True
        
        # Добавляем в историю
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        # Ограничиваем историю (последние 10 сообщений)
        if len(self.conversation_history) > 10:
            self.conversation_history = self.conversation_history[-10:]
        
        # Триггер звука "thinking" (Phase 4)
        self._trigger_sound('thinking')
        
        # Запрос к DeepSeek (streaming)
        self._ask_deepseek_streaming()
    
    def _ask_deepseek_streaming(self):
        """Streaming запрос к DeepSeek с парсингом JSON chunks"""
        messages = [
            {"role": "system", "content": self.system_prompt},
            *self.conversation_history
        ]
        
        self.get_logger().info('🤔 Запрос к DeepSeek...')
        
        try:
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                stream=True
            )
            
            # Накопление response
            full_response = ""
            current_chunk = ""
            brace_count = 0
            in_json = False
            chunk_count = 0
            
            # Обработка streaming chunks
            for chunk in stream:
                if chunk.choices[0].delta.content:
                    token = chunk.choices[0].delta.content
                    full_response += token
                    current_chunk += token
                    
                    # Подсчёт скобок для определения границ JSON
                    for char in token:
                        if char == '{':
                            brace_count += 1
                            in_json = True
                        elif char == '}':
                            brace_count -= 1
                    
                    # Если скобки сбалансированы - парсим
                    if in_json and brace_count == 0:
                        json_text = current_chunk.strip()
                        
                        # Убираем markdown ```json если есть
                        if json_text.startswith('```json'):
                            json_text = json_text.replace('```json', '').replace('```', '').strip()
                        
                        # Парсим JSON
                        try:
                            chunk_data = json.loads(json_text)
                            
                            # ============ ПРОВЕРКА: ask_reflection команда ============
                            if 'action' in chunk_data and chunk_data['action'] == 'ask_reflection':
                                question = chunk_data.get('question', '')
                                self.get_logger().warn(f'🔁 DeepSeek перенаправляет к Reflection: "{question}"')
                                
                                # Публикуем в /perception/user_speech для reflection_node
                                reflection_msg = String()
                                reflection_msg.data = question
                                self.reflection_request_pub.publish(reflection_msg)
                                self.get_logger().info(f'  → Запрос отправлен к внутреннему диалогу')
                                
                                # Сброс для следующего chunk
                                current_chunk = ""
                                in_json = False
                                brace_count = 0
                                continue  # Не обрабатываем дальше как обычный chunk
                            
                            # Применяем автоударения
                            if 'ssml' in chunk_data:
                                ssml = chunk_data['ssml']
                                ssml_with_accents = self.accent_replacer.add_accents(ssml)
                                chunk_data['ssml'] = ssml_with_accents
                                
                                # Публикуем chunk
                                chunk_count += 1
                                self.get_logger().info(f'📤 Chunk {chunk_count}: {ssml[:50]}...')
                                
                                # Обновляем время взаимодействия (робот говорит)
                                self.last_interaction_time = time.time()
                                
                                response_msg = String()
                                response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                                self.response_pub.publish(response_msg)
                                
                                # Публикуем в TTS для синтеза (Phase 6)
                                self.tts_pub.publish(response_msg)
                                self.get_logger().info(f'🔊 Отправлено в TTS: chunk {chunk_count}')
                            
                        except json.JSONDecodeError:
                            pass  # Ждём больше данных
                        
                        # Сброс для следующего chunk
                        current_chunk = ""
                        in_json = False
                        brace_count = 0
            
            # Сохраняем ответ в историю
            self.conversation_history.append({
                "role": "assistant",
                "content": full_response
            })
            
            self.get_logger().info(f'✅ DeepSeek ответил ({chunk_count} chunks)')
            
            # Сбрасываем флаг после успешного ответа
            self.dialogue_in_progress = False
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка DeepSeek: {e}')
            # Сбрасываем флаг даже при ошибке
            self.dialogue_in_progress = False
    
    def _trigger_sound(self, sound_name: str):
        """Триггер звукового эффекта (Phase 4)"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_trigger_pub.publish(msg)
            self.get_logger().debug(f'🔔 Триггер звука: {sound_name}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Ошибка триггера звука: {e}')
    
    def command_feedback_callback(self, msg: String):
        """Обработка feedback от command_node (Phase 5)"""
        feedback = msg.data.strip()
        if not feedback:
            return
        
        # Игнорируем feedback если dialogue уже обработал запрос
        if self.dialogue_in_progress:
            self.get_logger().debug(f'🔇 Игнор command feedback (dialogue in progress): {feedback}')
            return
        
        self.get_logger().info(f'📢 Command feedback: {feedback}')
        
        # Отправить feedback в TTS (напрямую в response)
        response_json = {
            "ssml": f"<speak>{feedback}</speak>"
        }
        
        response_msg = String()
        response_msg.data = json.dumps(response_json, ensure_ascii=False)
        self.response_pub.publish(response_msg)
    
    def _check_dialogue_timeout(self):
        """Проверить тайм-аут диалога и вернуться в IDLE если нет активности"""
        if self.state not in ['LISTENING', 'DIALOGUE']:
            return
        
        elapsed = time.time() - self.last_interaction_time
        if elapsed > self.dialogue_timeout:
            self.get_logger().info(f'⏰ Dialogue timeout ({elapsed:.1f}s) → IDLE')
            self.state = 'IDLE'
            self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = DialogueNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
