#!/usr/bin/env python3
"""
Интерактивный чат с ROBBOX роботом

Использует:
- DeepSeek API для генерации ответов (JSON с SSML)
- Text Normalizer v2 для обработки ответов
- Silero TTS для озвучивания (с "бурундуком")
- Командная система для управления роботом
"""

import os
import sys
import json
import time
import re
import torch
import sounddevice as sd
from openai import OpenAI
from text_normalizer_v2 import RobboxVoiceHandler


class SileroTTS:
    """Silero TTS v4 с поддержкой "бурундука" """
    
    def __init__(self, sample_rate: int = 24000, chipmunk_mode: bool = True):
        self.sample_rate = sample_rate
        self.chipmunk_mode = chipmunk_mode
        
        print("🔄 Загрузка Silero TTS v4...")
        self.device = torch.device('cpu')
        
        # Загружаем локальную модель из /models (предзагружена в Dockerfile)
        model_path = "/models/silero_v4_ru.pt"
        if os.path.exists(model_path):
            print(f"📦 Загрузка Silero из локального файла: {model_path}")
            self.model = torch.jit.load(model_path, map_location=self.device)
            print("✅ Silero TTS загружен из локального файла\n")
        else:
            # Fallback на онлайн загрузку
            print(f"⚠️ Локальная модель не найдена: {model_path}, загружаем из GitHub")
            self.model, _ = torch.hub.load(
                repo_or_dir='snakers4/silero-models',
                model='silero_tts',
                language='ru',
                speaker='v4_ru'
            )
            self.model.to(self.device)
            print("✅ Silero TTS загружен из GitHub\n")
    
    def synthesize_and_play(self, text: str, speaker: str = 'aidar'):
        """
        Синтезирует и воспроизводит речь
        
        Args:
            text: Нормализованный текст для озвучивания
            speaker: aidar, baya, kseniya, xenia
        
        Оптимальные параметры для ROBBOX:
        - speaker: aidar
        - pitch: medium
        - rate: x-slow (компенсирует pitch shift 2.0x)
        - sample_rate: 24000 Hz → 48000 Hz с бурундуком
        """
        if not text.strip():
            return
        
        print(f"🔊 Говорю: {text}")
        
        # Оборачиваем текст в SSML с оптимальными параметрами
        # rate="x-slow" чтобы после pitch shift 2x было нормально
        # pitch="medium" для среднего тона
        ssml_text = f'<speak><prosody rate="x-slow" pitch="medium">{text}</prosody></speak>'
        
        # Синтез через SSML (поддерживает prosody теги!)
        audio = self.model.apply_tts(
            ssml_text=ssml_text,
            speaker=speaker,
            sample_rate=self.sample_rate
        )
        
        # Конвертируем в numpy
        audio_np = audio.numpy()
        
        # Воспроизведение с pitch shift (бурундук)
        if self.chipmunk_mode:
            playback_rate = self.sample_rate * 2  # 24000 → 48000 (2x pitch shift)
            print("🐿️  Режим 'Бурундук': pitch shift 2.0x (rate=x-slow + pitch=medium)")
        else:
            playback_rate = self.sample_rate
        
        # Воспроизводим и ЖДЁМ ОКОНЧАНИЯ
        sd.play(audio_np, playback_rate)
        sd.wait()  # Критично! Без этого фразы обрубаются
        print()


class RobboxChat:
    """Главный класс чата с ROBBOX"""
    
    def __init__(self):
        # Проверяем API ключ
        api_key = os.getenv('DEEPSEEK_API_KEY')
        if not api_key:
            print("❌ Ошибка: DEEPSEEK_API_KEY не найден в переменных окружения")
            print("Установите: export DEEPSEEK_API_KEY='your-key'")
            sys.exit(1)
        
        # Инициализируем компоненты
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://api.deepseek.com"
        )
        
        self.voice_handler = RobboxVoiceHandler()
        self.tts = SileroTTS(sample_rate=24000, chipmunk_mode=True)
        
        # История диалога
        self.conversation_history = []
        
        # Загружаем системный промпт
        self.system_prompt = self._load_system_prompt()
        
        print("="*60)
        print("🤖 ROBBOX ЧАТ - ИНТЕРАКТИВНЫЙ РЕЖИМ")
        print("="*60)
        print("Компоненты:")
        print("  ✅ DeepSeek API подключен")
        print("  ✅ Silero TTS загружен (бурундук режим)")
        print("  ✅ Text Normalizer v2 готов")
        print("  ✅ Система команд активна")
        print()
        print("Команды чата:")
        print("  /quit    - Выход")
        print("  /clear   - Очистить историю")
        print("  /history - Показать историю")
        print("="*60)
        print()
    
    def _load_system_prompt(self) -> str:
        """Загружает системный промпт"""
        prompt_path = os.path.join(
            os.path.dirname(__file__),
            '../prompts/master_prompt.txt'
        )
        
        try:
            with open(prompt_path, 'r', encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            print(f"⚠️  Промпт не найден: {prompt_path}")
            return "Ты ROBBOX - мобильный робот-ассистент. Отвечай в JSON формате с полями: text, ssml, commands, emotion."
    
    def ask_deepseek(self, user_message: str) -> str:
        """
        Отправляет вопрос в DeepSeek и получает JSON ответ
        
        Returns:
            JSON строка с полями: text, ssml, commands, emotion
        """
        # Добавляем сообщение в историю
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        # Формируем запрос
        messages = [
            {"role": "system", "content": self.system_prompt},
            *self.conversation_history
        ]
        
        print("🤔 ROBBOX думает...")
        
        # Запрос к DeepSeek
        try:
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=messages,
                temperature=0.7,
                max_tokens=500
            )
            
            assistant_message = response.choices[0].message.content
            
            # Убираем markdown блоки ```json ... ```
            assistant_message = re.sub(r'^```json\s*', '', assistant_message)
            assistant_message = re.sub(r'\s*```$', '', assistant_message)
            assistant_message = assistant_message.strip()
            
            # Добавляем ответ в историю
            self.conversation_history.append({
                "role": "assistant",
                "content": assistant_message
            })
            
            return assistant_message
            
        except Exception as e:
            print(f"❌ Ошибка DeepSeek API: {e}")
            return json.dumps({
                "text": "Извините, произошла ошибка при обработке запроса.",
                "emotion": "sad"
            })
    
    def process_and_speak(self, response: str):
        """
        Обрабатывает ответ DeepSeek и озвучивает
        
        Args:
            response: JSON строка от DeepSeek
        """
        # Парсим и обрабатываем ответ
        result = self.voice_handler.process_response(response, execute_commands=True)
        
        # Показываем эмоцию
        emotion_emoji = {
            'neutral': '😐',
            'happy': '😊',
            'sad': '😢',
            'thinking': '🤔',
            'alert': '⚠️',
            'angry': '😠',
            'surprised': '😮'
        }
        emoji = emotion_emoji.get(result['emotion'], '🤖')
        
        print("="*60)
        print(f"🤖 ROBBOX [{emoji} {result['emotion']}]:")
        print("="*60)
        
        # Используем оптимальный голос
        speaker = 'aidar'  # Мужской голос, лучше всего звучит с бурундуком
        
        # Если есть SSML chunks с паузами - используем ИХ (приоритет)
        if result['ssml_chunks']:
            for text, pause_ms in result['ssml_chunks']:
                self.tts.synthesize_and_play(text, speaker=speaker)
                if pause_ms:
                    print(f"⏸️  Пауза {pause_ms}ms...")
                    time.sleep(pause_ms / 1000.0)
        # Иначе озвучиваем обычные фразы
        elif result['phrases']:
            for phrase in result['phrases']:
                self.tts.synthesize_and_play(phrase, speaker=speaker)
        
        print("="*60)
        print()
    
    def run(self):
        """Запускает интерактивный чат"""
        while True:
            try:
                # Ввод пользователя
                user_input = input("👤 Вы: ").strip()
                
                if not user_input:
                    continue
                
                # Обработка команд чата
                if user_input.startswith('/'):
                    if user_input == '/quit':
                        print("\n👋 До свидания!")
                        break
                    
                    elif user_input == '/clear':
                        self.conversation_history = []
                        print("🗑️  История диалога очищена\n")
                        continue
                    
                    elif user_input == '/history':
                        print("\n📜 История диалога:")
                        for i, msg in enumerate(self.conversation_history, 1):
                            role = "👤" if msg['role'] == 'user' else "🤖"
                            content = msg['content'][:100] + "..." if len(msg['content']) > 100 else msg['content']
                            print(f"{i}. {role} {content}")
                        print()
                        continue
                    
                    else:
                        print(f"❌ Неизвестная команда: {user_input}\n")
                        continue
                
                # Отправляем вопрос в DeepSeek
                response_json = self.ask_deepseek(user_input)
                
                # Показываем сырой JSON (для отладки)
                print(f"\n📋 JSON ответ:")
                try:
                    parsed = json.loads(response_json)
                    print(json.dumps(parsed, indent=2, ensure_ascii=False))
                except:
                    print(response_json)
                print()
                
                # Обрабатываем и озвучиваем
                self.process_and_speak(response_json)
                
            except KeyboardInterrupt:
                print("\n\n👋 Прервано пользователем. До свидания!")
                break
            
            except Exception as e:
                print(f"\n❌ Ошибка: {e}")
                import traceback
                traceback.print_exc()
                print()


def main():
    """Точка входа"""
    chat = RobboxChat()
    chat.run()


if __name__ == '__main__':
    main()
