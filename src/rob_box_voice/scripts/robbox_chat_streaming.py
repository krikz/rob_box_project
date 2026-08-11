#!/usr/bin/env python3
"""
Интерактивный чат с ROBBOX роботом (STREAMING версия)

Использует:
- DeepSeek API streaming для быстрых ответов
- Text Normalizer v2 для обработки ответов
- Silero TTS для озвучивания (с "бурундуком")
- Потоковое озвучивание: начинаем говорить ДО получения полного ответа
"""

import os
import sys
import json
import time
import re
import threading
import queue
import torch
import sounddevice as sd
from openai import OpenAI
from text_normalizer_v2 import RobboxVoiceHandler
from accent_replacer import AccentReplacer


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
        """
        if not text.strip():
            return

        print(f"🔊 Говорю: {text}")

        # Оборачиваем текст в SSML с оптимальными параметрами
        ssml_text = f'<speak><prosody rate="x-slow" pitch="medium">{text}</prosody></speak>'

        # Синтез через SSML
        audio = self.model.apply_tts(
            ssml_text=ssml_text,
            speaker=speaker,
            sample_rate=self.sample_rate
        )

        # Конвертируем в numpy
        audio_np = audio.numpy()

        # Воспроизведение с pitch shift (бурундук)
        if self.chipmunk_mode:
            playback_rate = self.sample_rate * 2
            print("🐿️  Режим 'Бурундук': pitch shift 2.0x (rate=x-slow + pitch=medium)")
        else:
            playback_rate = self.sample_rate

        # Воспроизводим и ЖДЁМ ОКОНЧАНИЯ
        sd.play(audio_np, playback_rate)
        sd.wait()
        print()


class StreamingChatBot:
    """
    Потоковый чат-бот

    Особенности:
    - Получает ответ от DeepSeek по частям (streaming)
    - Накапливает предложения
    - Озвучивает каждое предложение сразу как оно готово
    - Не ждёт полного ответа!
    """

    def __init__(self):
        # API ключ
        api_key = os.getenv('DEEPSEEK_API_KEY')
        if not api_key:
            print("❌ Ошибка: DEEPSEEK_API_KEY не найден")
            sys.exit(1)

        # Инициализация
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://api.deepseek.com"
        )

        self.voice_handler = RobboxVoiceHandler()
        self.tts = SileroTTS(sample_rate=24000, chipmunk_mode=True)
        self.accent_replacer = AccentReplacer()

        # История и промпт
        self.conversation_history = []
        self.system_prompt = self._load_system_prompt()

        print("="*60)
        print("🤖 ROBBOX ЧАТ - STREAMING РЕЖИМ")
        print("="*60)
        print("Компоненты:")
        print("  ✅ DeepSeek API (streaming)")
        print("  ✅ Silero TTS (бурундук режим)")
        print("  ✅ Автоматические ударения (словарь)")
        print("  ✅ Быстрый отклик (streaming + параллельное озвучивание)")
        print()
        print("Команды:")
        print("  /quit    - Выход")
        print("  /clear   - Очистить историю")
        print("="*60)
        print()

        # Показываем статистику словаря ударений
        stats = self.accent_replacer.get_stats()
        print(f"📖 Словарь ударений: {stats['total_words']} слов")
        print()

    def _load_system_prompt(self) -> str:
        """Загружает системный промпт."""
        # Используем упрощённый промпт без ударений
        prompt_path = os.path.join(
            os.path.dirname(__file__),
            '../prompts/master_prompt_simple.txt'
        )

        try:
            with open(prompt_path, 'r', encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            return "Ты ROBBOX - мобильный робот-ассистент. Отвечай в JSON формате."

    def ask_deepseek_streaming(self, user_message: str):
        """
        Отправляет вопрос в DeepSeek и получает streaming ответ

        СТРАТЕГИЯ с JSON chunks:
        1. DeepSeek возвращает несколько маленьких JSON объектов
        2. Каждый JSON = 1-2 предложения
        3. Парсим и озвучиваем каждый chunk сразу как получили
        4. = МИНИМАЛЬНАЯ ЗАДЕРЖКА!
        """
        # Добавляем в историю
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })

        # Формируем запрос
        messages = [
            {"role": "system", "content": self.system_prompt},
            *self.conversation_history
        ]

        print("🤔 ROBBOX думает...\n")

        try:
            # Streaming запрос к DeepSeek
            stream = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=messages,
                temperature=0.7,
                max_tokens=500,
                stream=True
            )

            # Накапливаем chunks и полный ответ
            full_response = ""
            current_chunk = ""  # Накапливаем текущий JSON chunk
            brace_count = 0  # Счётчик фигурных скобок для парсинга JSON
            in_json = False
            chunk_count = 0

            print("="*60)
            print("🤖 ROBBOX:")
            print("="*60)

            # DEBUG
            debug = os.getenv('DEBUG', '').lower() == 'true'

            # Обрабатываем поток
            for chunk in stream:
                if chunk.choices[0].delta.content:
                    token = chunk.choices[0].delta.content
                    full_response += token

                    if debug:
                        print(f"[DEBUG] Token: {repr(token[:30])}")

                    # Ищем начало JSON
                    if '{' in token and not in_json:
                        in_json = True
                        # Добавляем символы после '{'
                        idx = token.index('{')
                        current_chunk = token[idx:]
                        brace_count = current_chunk.count('{') - current_chunk.count('}')

                        if debug:
                            print(f"[DEBUG] JSON начался, brace_count={brace_count}")

                    elif in_json:
                        current_chunk += token
                        brace_count += token.count('{') - token.count('}')

                        if debug and ('{' in token or '}' in token):
                            print(f"[DEBUG] brace_count={brace_count}")

                        # Когда скобки сбалансированы - у нас полный JSON
                        if brace_count == 0:
                            # Парсим JSON
                            try:
                                chunk_data = json.loads(current_chunk)
                                chunk_count += 1

                                # Показываем chunk
                                print(f"\n📦 Chunk {chunk_count}: {json.dumps(chunk_data, ensure_ascii=False)[:100]}...")

                                # Озвучиваем сразу!
                                self._speak_chunk(chunk_data)

                            except json.JSONDecodeError as e:
                                if debug:
                                    print(f"[DEBUG] JSON parse error: {e}")
                                    print(f"[DEBUG] Current chunk: {current_chunk[:100]}")

                            # Сбрасываем для следующего chunk
                            current_chunk = ""
                            in_json = False
                            brace_count = 0

            # Если остался незавершённый JSON - пробуем парсить
            if in_json and current_chunk.strip():
                try:
                    chunk_data = json.loads(current_chunk.strip())
                    chunk_count += 1
                    print(f"\n📦 Chunk {chunk_count} (final): {json.dumps(chunk_data, ensure_ascii=False)[:100]}...")
                    self._speak_chunk(chunk_data)
                except json.JSONDecodeError as e:
                    if debug:
                        print(f"[DEBUG] Final JSON parse error: {e}")

            print("\n" + "="*60 + "\n")

            # Добавляем в историю
            self.conversation_history.append({
                "role": "assistant",
                "content": full_response
            })

            return full_response

        except Exception as e:
            print(f"\n❌ Ошибка: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _speak_chunk(self, chunk_data: dict):
        """
        Озвучивает один JSON chunk

        Args:
            chunk_data: {"chunk": 1, "ssml": "...", "emotion": "..."}
        """
        # Извлекаем SSML (основное поле теперь!)
        ssml = chunk_data.get('ssml', '')

        if not ssml:
            print("⚠️  Пустой SSML в chunk, пропускаем")
            return

        # ✨ ДОБАВЛЯЕМ УДАРЕНИЯ из словаря
        ssml = self.accent_replacer.add_accents(ssml)

        # Проверяем есть ли SSML теги
        if self.voice_handler.ssml_processor.has_ssml_tags(ssml):
            # Парсим SSML chunks с паузами
            ssml_chunks = self.voice_handler.ssml_processor.parse_ssml_for_timing(ssml)
            for ssml_text, pause_ms in ssml_chunks:
                # SSML уже содержит ударения, просто озвучиваем
                # Нормализация нужна только для латинских букв
                normalized = self.voice_handler.parser.base_normalizer.normalize(ssml_text)
                self.tts.synthesize_and_play(normalized, speaker='aidar')
                if pause_ms:
                    print(f"⏸️  Пауза {pause_ms}ms...")
                    time.sleep(pause_ms / 1000.0)
        else:
            # Если нет SSML тегов - просто нормализуем и озвучиваем
            normalized = self.voice_handler.parser.base_normalizer.normalize(ssml)
            self.tts.synthesize_and_play(normalized, speaker='aidar')

        # Выполняем команды (только если есть)
        commands = chunk_data.get('commands', [])
        if commands:
            print(f"\n{'='*60}")
            print("⚙️  ВЫПОЛНЕНИЕ КОМАНД:")
            print('='*60)
            for cmd_string in commands:
                command, param = self.voice_handler.executor.parse_command(cmd_string)
                if self.voice_handler.executor.validate_command(command, param):
                    self.voice_handler.executor.execute(command, param)
                else:
                    print(f"⚠️  Неизвестная команда: {cmd_string}")

    def run(self):
        """Запускает интерактивный чат."""
        while True:
            try:
                user_input = input("👤 Вы: ").strip()

                if not user_input:
                    continue

                # Команды
                if user_input.startswith('/'):
                    if user_input == '/quit':
                        print("\n👋 До свидания!")
                        break

                    elif user_input == '/clear':
                        self.conversation_history = []
                        print("🗑️  История очищена\n")
                        continue

                    else:
                        print(f"❌ Неизвестная команда\n")
                        continue

                # Отправляем вопрос
                self.ask_deepseek_streaming(user_input)

            except KeyboardInterrupt:
                print("\n\n👋 Прервано. До свидания!")
                break

            except Exception as e:
                print(f"\n❌ Ошибка: {e}")
                import traceback
                traceback.print_exc()
                print()


def main():
    """Точка входа."""
    chat = StreamingChatBot()
    chat.run()


if __name__ == '__main__':
    main()
