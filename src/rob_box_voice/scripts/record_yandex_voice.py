#!/usr/bin/env python3
"""
Скрипт для записи голоса ROBBOX через Yandex SpeechKit

Цель: Собрать аудио данные для обучения кастомной TTS модели

Требования:
- API ключи Yandex Cloud (IAM token или API key)
- Текстовый корпус для синтеза (минимум 20-30 минут аудио)

Рекомендации:
- Для fine-tuning: 20-30 минут (200-300 фраз)
- Для полного обучения: 3-5 часов (3000-5000 фраз)

Использование:
    python3 record_yandex_voice.py --input sentences.txt --output dataset/
"""

import os
import sys
import json
import time
import argparse
from pathlib import Path
from typing import List

try:
    import requests
    import soundfile as sf
    import numpy as np
except ImportError as e:
    print(f"❌ Ошибка импорта: {e}")
    print("\nУстановите зависимости:")
    print("  pip install requests soundfile numpy")
    sys.exit(1)


class YandexTTSRecorder:
    """Записывает аудио через Yandex SpeechKit для создания датасета"""
    
    def __init__(self, api_key: str, folder_id: str, voice: str = "anton", 
                 emotion: str = "neutral", speed: float = 1.0):
        """
        Args:
            api_key: Yandex Cloud API key или IAM token
            folder_id: ID каталога Yandex Cloud
            voice: Голос (anton, alena, ermil)
            emotion: Эмоция (neutral, good, evil)
            speed: Скорость речи (0.1-3.0)
        """
        self.api_key = api_key
        self.folder_id = folder_id
        self.voice = voice
        self.emotion = emotion
        self.speed = speed
        
        # Yandex SpeechKit API endpoint
        self.url = "https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize"
    
    def synthesize(self, text: str) -> bytes:
        """
        Синтезировать текст в аудио через Yandex API
        
        Returns:
            PCM audio data (48kHz, 16-bit, mono)
        """
        headers = {
            "Authorization": f"Api-Key {self.api_key}"
        }
        
        data = {
            "text": text,
            "lang": "ru-RU",
            "voice": self.voice,
            "emotion": self.emotion,
            "speed": self.speed,
            "format": "lpcm",
            "sampleRateHertz": 48000
        }
        
        try:
            response = requests.post(self.url, headers=headers, data=data, timeout=30)
            response.raise_for_status()
            return response.content
        except requests.exceptions.RequestException as e:
            print(f"❌ Ошибка запроса к Yandex API: {e}")
            return None
    
    def save_audio(self, audio_data: bytes, output_path: Path, sample_rate: int = 48000):
        """
        Сохранить аудио в WAV файл
        
        Args:
            audio_data: PCM audio bytes
            output_path: Путь для сохранения WAV
            sample_rate: Частота дискретизации
        """
        # Конвертация bytes в numpy array (int16)
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # Нормализация в float32 [-1.0, 1.0]
        audio_float = audio_array.astype(np.float32) / 32768.0
        
        # Сохранение WAV
        sf.write(output_path, audio_float, sample_rate)
    
    def record_dataset(self, sentences: List[str], output_dir: Path, 
                      delay: float = 1.0, resume_from: int = 0):
        """
        Записать датасет из списка предложений
        
        Args:
            sentences: Список предложений для синтеза
            output_dir: Директория для сохранения аудио
            delay: Задержка между запросами (секунды)
            resume_from: С какого индекса продолжить (для восстановления)
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Metadata файл (для обучения)
        metadata_path = output_dir / "metadata.csv"
        metadata_file = open(metadata_path, "a" if resume_from > 0 else "w", encoding="utf-8")
        
        if resume_from == 0:
            # Заголовок CSV: filename|text|duration
            metadata_file.write("filename|text|duration\n")
        
        total = len(sentences)
        success_count = 0
        fail_count = 0
        
        print(f"📝 Начинаю запись датасета:")
        print(f"   Всего фраз: {total}")
        print(f"   Голос: {self.voice}")
        print(f"   Эмоция: {self.emotion}")
        print(f"   Скорость: {self.speed}")
        print(f"   Выходная папка: {output_dir}")
        
        if resume_from > 0:
            print(f"   ⏭️  Продолжаю с фразы {resume_from + 1}")
        
        print()
        
        for i, sentence in enumerate(sentences[resume_from:], start=resume_from):
            # Прогресс
            progress = (i + 1) / total * 100
            print(f"[{i+1}/{total}] ({progress:.1f}%) {sentence[:50]}...")
            
            # Синтез
            audio_data = self.synthesize(sentence)
            
            if audio_data is None:
                print(f"   ❌ Ошибка синтеза")
                fail_count += 1
                continue
            
            # Сохранение
            filename = f"robbox_{i:05d}.wav"
            output_path = output_dir / filename
            
            try:
                self.save_audio(audio_data, output_path)
                
                # Вычисление длительности
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                duration = len(audio_array) / 48000.0
                
                # Запись метаданных
                metadata_file.write(f"{filename}|{sentence}|{duration:.2f}\n")
                metadata_file.flush()
                
                print(f"   ✅ Сохранено: {filename} ({duration:.2f}s)")
                success_count += 1
                
            except Exception as e:
                print(f"   ❌ Ошибка сохранения: {e}")
                fail_count += 1
                continue
            
            # Задержка (чтобы не забанили API)
            if i < total - 1:
                time.sleep(delay)
        
        metadata_file.close()
        
        # Итоги
        print()
        print("=" * 60)
        print(f"✅ Запись завершена!")
        print(f"   Успешно: {success_count} / {total}")
        print(f"   Ошибок: {fail_count}")
        
        if success_count > 0:
            total_duration = sum([
                float(line.split('|')[2]) 
                for line in open(metadata_path, encoding='utf-8').readlines()[1:]
            ])
            print(f"   Общая длительность: {total_duration / 60:.1f} минут")
            print(f"   Метаданные: {metadata_path}")
            
            # Рекомендации
            print()
            print("📊 Рекомендации:")
            if total_duration < 20 * 60:
                print("   ⚠️  Меньше 20 минут - хватит только для voice cloning")
                print("   💡 Для fine-tuning нужно минимум 20-30 минут")
            elif total_duration < 3 * 60 * 60:
                print("   ✅ 20-30 минут - достаточно для fine-tuning")
                print("   💡 Для полного обучения нужно 3-5 часов")
            else:
                print("   ✅ Больше 3 часов - достаточно для полного обучения!")


def load_sentences(input_path: Path) -> List[str]:
    """
    Загрузить предложения из текстового файла
    
    Формат: одна фраза на строку
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        sentences = [line.strip() for line in f if line.strip()]
    return sentences


def generate_corpus_suggestions():
    """Предложить источники текста для датасета"""
    print("💡 Откуда взять текст для датасета?")
    print()
    print("1. Системные фразы ROBBOX:")
    print("   - 'Слушаю', 'Готово', 'Еду вперёд', 'Препятствие', и т.д.")
    print("   - Файл: src/rob_box_voice/config/voice_assistant.yaml (system_phrases)")
    print()
    print("2. Типичные команды:")
    print("   - 'Поверни налево', 'Остановись', 'Покажи карту', и т.д.")
    print()
    print("3. Диалоги:")
    print("   - Примеры вопросов и ответов")
    print()
    print("4. Общий корпус русского языка:")
    print("   - OpenCorpora (opencorpora.org)")
    print("   - Tatoeba (tatoeba.org/rus)")
    print("   - Wikipedia dump (сложнее обработать)")
    print()
    print("5. Генерация через LLM:")
    print("   - DeepSeek, ChatGPT: 'Сгенерируй 1000 фраз для робота'")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Запись голоса ROBBOX через Yandex SpeechKit для обучения TTS модели"
    )
    
    parser.add_argument("--input", "-i", type=str,
                       help="Текстовый файл с предложениями (одна фраза на строку)")
    parser.add_argument("--output", "-o", type=str, default="dataset/robbox_voice",
                       help="Директория для сохранения аудио (default: dataset/robbox_voice)")
    
    parser.add_argument("--api-key", type=str, 
                       help="Yandex Cloud API key (или переменная окружения YANDEX_API_KEY)")
    parser.add_argument("--folder-id", type=str,
                       help="Yandex Cloud Folder ID (или переменная окружения YANDEX_FOLDER_ID)")
    
    parser.add_argument("--voice", type=str, default="anton",
                       choices=["anton", "alena", "ermil"],
                       help="Голос Yandex (default: anton)")
    parser.add_argument("--emotion", type=str, default="neutral",
                       choices=["neutral", "good", "evil"],
                       help="Эмоция (default: neutral)")
    parser.add_argument("--speed", type=float, default=1.0,
                       help="Скорость речи 0.1-3.0 (default: 1.0)")
    
    parser.add_argument("--delay", type=float, default=1.0,
                       help="Задержка между запросами в секундах (default: 1.0)")
    parser.add_argument("--resume-from", type=int, default=0,
                       help="Продолжить с фразы N (для восстановления)")
    
    parser.add_argument("--suggest-corpus", action="store_true",
                       help="Показать предложения по источникам текста")
    
    args = parser.parse_args()
    
    # Показать предложения
    if args.suggest_corpus:
        generate_corpus_suggestions()
        return
    
    # Проверка обязательных аргументов
    if not args.input:
        print("❌ Ошибка: требуется --input с текстовым файлом")
        print()
        print("Пример использования:")
        print("  python3 record_yandex_voice.py --input sentences.txt --output dataset/")
        print()
        print("Или используйте --suggest-corpus для идей:")
        print("  python3 record_yandex_voice.py --suggest-corpus")
        sys.exit(1)
    
    # API ключи из аргументов или переменных окружения
    api_key = args.api_key or os.getenv("YANDEX_API_KEY")
    folder_id = args.folder_id or os.getenv("YANDEX_FOLDER_ID")
    
    if not api_key or not folder_id:
        print("❌ Ошибка: требуются API ключи Yandex Cloud")
        print()
        print("Укажите через аргументы:")
        print("  --api-key YOUR_API_KEY --folder-id YOUR_FOLDER_ID")
        print()
        print("Или через переменные окружения:")
        print("  export YANDEX_API_KEY='your_key'")
        print("  export YANDEX_FOLDER_ID='your_folder_id'")
        print()
        print("Как получить ключи:")
        print("  https://cloud.yandex.ru/docs/iam/operations/api-key/create")
        sys.exit(1)
    
    # Загрузка предложений
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"❌ Ошибка: файл не найден: {input_path}")
        sys.exit(1)
    
    sentences = load_sentences(input_path)
    if not sentences:
        print(f"❌ Ошибка: файл пустой: {input_path}")
        sys.exit(1)
    
    print(f"📄 Загружено {len(sentences)} предложений из {input_path}")
    print()
    
    # Создание рекордера
    recorder = YandexTTSRecorder(
        api_key=api_key,
        folder_id=folder_id,
        voice=args.voice,
        emotion=args.emotion,
        speed=args.speed
    )
    
    # Запись датасета
    output_dir = Path(args.output)
    recorder.record_dataset(
        sentences=sentences,
        output_dir=output_dir,
        delay=args.delay,
        resume_from=args.resume_from
    )


if __name__ == "__main__":
    main()
