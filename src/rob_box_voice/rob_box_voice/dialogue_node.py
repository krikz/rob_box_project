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
        
        # Флаг что dialogue_node обработал запрос (чтобы игнорировать command feedback)
        self.dialogue_in_progress = False
        
        self.get_logger().info('✅ DialogueNode инициализирован')
        self.get_logger().info(f'  Model: {self.model}')
        self.get_logger().info(f'  Temperature: {self.temperature}')
        self.get_logger().info(f'  Max tokens: {self.max_tokens}')
    
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
    
    def stt_callback(self, msg: String):
        """Обработка распознанной речи"""
        user_message = msg.data.strip()
        if not user_message:
            return
        
        self.get_logger().info(f'👤 User: {user_message}')
        
        # Устанавливаем флаг что dialogue обрабатывает запрос
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
                            
                            # Применяем автоударения
                            if 'ssml' in chunk_data:
                                ssml = chunk_data['ssml']
                                ssml_with_accents = self.accent_replacer.add_accents(ssml)
                                chunk_data['ssml'] = ssml_with_accents
                                
                                # Публикуем chunk
                                chunk_count += 1
                                self.get_logger().info(f'📤 Chunk {chunk_count}: {ssml[:50]}...')
                                
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
