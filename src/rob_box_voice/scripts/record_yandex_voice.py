#!/usr/bin/env python3
"""
Запись голоса ROBBOX через Yandex Cloud TTS API v3 (gRPC).

Использует ТОЧНЫЕ настройки из вашего рабочего скрипта:
- voice: anton (доступен только в API v3!)
- speed: 0.4 (медленнее обычного для чёткости)
- API: gRPC (yandex.cloud.ai.tts.v3)

ВАЖНО ПРО ИСКАЖЕНИЕ ГОЛОСА:
В вашем скрипте голос искажается ("эффект бурундука") потому что:
1. Yandex TTS возвращает WAV с частотой 22050 Hz
2. np.frombuffer(audio_data, dtype=np.int16) читает байты БЕЗ заголовка WAV
3. sd.play(samples, samplerate=44100) воспроизводит на частоте устройства WM8960
4. Результат: голос в 2x быстрее и выше (44100/22050 = 2.0x pitch shift)

Это искажение создаёт уникальный звук ROBBOX!

Для обучения сохраняем файлы БЕЗ изменений (оригинал 22050 Hz).

Требования:
- pip install grpcio grpcio-tools soundfile numpy tqdm
- pip install yandex-cloud-ml-sdk  # Для yandex.cloud.ai.tts.v3
- YANDEX_API_KEY или YANDEX_IAM_TOKEN
- YANDEX_FOLDER_ID

Установка зависимостей:
    pip install -r requirements_yandex_v3.txt
"""

import os
import sys
import argparse
import time
from pathlib import Path
from typing import Optional
import grpc
import soundfile as sf
import numpy as np
from tqdm import tqdm

# Проверка зависимостей
try:
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc
except ImportError:
    print("❌ Ошибка: yandex-cloud-ml-sdk не установлен!")
    print("\nУстановите:")
    print("  pip install yandex-cloud-ml-sdk")
    print("\nИли через requirements:")
    print("  pip install -r requirements_yandex_v3.txt")
    sys.exit(1)


class YandexTTSRecorderV3:
    """Запись датасета через Yandex TTS API v3 (gRPC)"""
    
    def __init__(self, api_key: str, folder_id: str, voice: str = "anton", speed: float = 0.4):
        """
        Args:
            api_key: Yandex API Key или IAM Token
            folder_id: Yandex Cloud Folder ID
            voice: Голос (anton, lea, madi, marina - только v3!)
            speed: Скорость речи (0.1-3.0, рекомендуется 0.4-1.0)
        """
        self.api_key = api_key
        self.folder_id = folder_id
        self.voice = voice
        self.speed = speed
        
        # Инициализация gRPC канала
        self.channel = grpc.secure_channel(
            'tts.api.cloud.yandex.net:443',
            grpc.ssl_channel_credentials()
        )
        self.stub = tts_service_pb2_grpc.SynthesizerStub(self.channel)
        
        print(f"✅ Yandex TTS v3 инициализирован")
        print(f"   Voice: {voice}")
        print(f"   Speed: {speed}")
        print(f"   Folder ID: {folder_id}")
    
    def synthesize(self, text: str) -> Optional[bytes]:
        """
        Синтез текста в аудио через gRPC API v3.
        
        Args:
            text: Текст для синтеза
            
        Returns:
            WAV audio данные или None при ошибке
        """
        # Создаём запрос как в вашем скрипте
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(
                    container_audio_type=tts_pb2.ContainerAudio.WAV
                )
            ),
            hints=[
                tts_pb2.Hints(voice=self.voice),
                tts_pb2.Hints(speed=self.speed),
            ],
            loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS
        )
        
        try:
            # Отправляем запрос с авторизацией
            responses = self.stub.UtteranceSynthesis(
                request,
                metadata=(('authorization', f'Api-Key {self.api_key}'),)
            )
            
            # Собираем аудио данные из стрима
            audio_data = b""
            for response in responses:
                audio_data += response.audio_chunk.data
            
            if audio_data:
                return audio_data
            else:
                print(f"⚠️  Пустой ответ от TTS для: {text[:50]}...")
                return None
                
        except grpc.RpcError as e:
            print(f"❌ gRPC ошибка: {e.code()} - {e.details()}")
            return None
        except Exception as e:
            print(f"❌ Ошибка синтеза: {e}")
            return None
    
    def save_audio(self, audio_data: bytes, output_path: Path) -> Optional[float]:
        """
        Сохранение WAV аудио ТОЧНО как получено от Yandex TTS v3.
        БЕЗ ресемплинга, БЕЗ нормализации - сохраняем оригинал 22050 Hz!
        
        Args:
            audio_data: WAV данные от TTS (с заголовком)
            output_path: Путь для сохранения
            
        Returns:
            Длительность в секундах или None при ошибке
        """
        try:
            # Сохраняем WAV как есть от Yandex (22050 Hz, PCM_16, mono)
            with open(output_path, 'wb') as f:
                f.write(audio_data)
            
            # Читаем только для получения длительности
            data, samplerate = sf.read(output_path)
            duration = len(data) / samplerate
            
            return duration
            
        except Exception as e:
            print(f"❌ Ошибка сохранения {output_path}: {e}")
            return None
    
    def record_dataset(
        self,
        input_file: Path,
        output_dir: Path,
        delay: float = 1.5,
        resume_from: int = 0
    ) -> bool:
        """
        Запись полного датасета из текстового файла.
        
        Args:
            input_file: Файл с текстами (одна строка = одна фраза)
            output_dir: Директория для сохранения WAV + metadata.csv
            delay: Задержка между запросами (секунды)
            resume_from: Продолжить с N-ой фразы
            
        Returns:
            True если успешно
        """
        # Читаем входной файл
        if not input_file.exists():
            print(f"❌ Файл не найден: {input_file}")
            return False
        
        with open(input_file, 'r', encoding='utf-8') as f:
            sentences = [line.strip() for line in f if line.strip()]
        
        if not sentences:
            print("❌ Файл пустой!")
            return False
        
        # Создаём выходную директорию
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Путь к metadata
        metadata_path = output_dir / "metadata.csv"
        
        # Если resume, читаем существующий metadata
        existing_count = 0
        if resume_from > 0 and metadata_path.exists():
            with open(metadata_path, 'r', encoding='utf-8') as f:
                existing_count = len(f.readlines())
            print(f"📄 Найдено {existing_count} существующих записей")
        
        # Открываем metadata для дозаписи
        mode = 'a' if resume_from > 0 else 'w'
        metadata_file = open(metadata_path, mode, encoding='utf-8')
        
        print("\n" + "=" * 60)
        print("ЗАПИСЬ ДАТАСЕТА - YANDEX TTS V3")
        print("=" * 60)
        print(f"Входной файл: {input_file}")
        print(f"Выходная директория: {output_dir}")
        print(f"Всего фраз: {len(sentences)}")
        if resume_from > 0:
            print(f"Продолжаем с фразы: {resume_from}")
        print(f"Настройки: voice={self.voice}, speed={self.speed}")
        print("=" * 60)
        print()
        
        # Обработка фраз
        success_count = 0
        fail_count = 0
        total_duration = 0.0
        
        try:
            for i, text in enumerate(tqdm(sentences, desc="Запись", unit="фраза")):
                # Пропускаем уже обработанные
                if i < resume_from:
                    continue
                
                # Генерируем имя файла
                filename = f"robbox_{i:05d}.wav"
                output_path = output_dir / filename
                
                # Синтез
                audio_data = self.synthesize(text)
                
                if audio_data is None:
                    fail_count += 1
                    tqdm.write(f"⚠️  Пропущено [{i}]: {text[:50]}...")
                    time.sleep(delay)  # Задержка даже при ошибке
                    continue
                
                # Сохранение
                duration = self.save_audio(audio_data, output_path)
                
                if duration is None:
                    fail_count += 1
                    continue
                
                # Запись в metadata
                metadata_file.write(f"{filename}|{text}|{duration:.2f}\n")
                metadata_file.flush()
                
                success_count += 1
                total_duration += duration
                
                # Задержка перед следующим запросом
                if i < len(sentences) - 1:
                    time.sleep(delay)
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Прервано пользователем")
            print(f"Для продолжения используйте: --resume-from {success_count + resume_from}")
        finally:
            metadata_file.close()
        
        # Итоговая статистика
        print("\n" + "=" * 60)
        print("РЕЗУЛЬТАТЫ")
        print("=" * 60)
        print(f"✅ Успешно: {success_count}")
        print(f"❌ Ошибок: {fail_count}")
        print(f"⏱️  Общая длительность: {total_duration / 60:.1f} минут")
        
        if success_count > 0:
            avg_duration = total_duration / success_count
            print(f"📊 Средняя длительность фразы: {avg_duration:.1f}с")
            print()
            print("💡 Рекомендации по обучению:")
            
            if total_duration < 600:  # < 10 минут
                print("   ⚠️  Мало данных для quality training")
                print("   Подход: Voice cloning (быстро, качество 3-4/5)")
            elif total_duration < 1800:  # < 30 минут
                print("   ⭐ Оптимально для fine-tuning")
                print("   Подход: Fine-tuning (1-3 дня, качество 4/5)")
            else:
                print("   🚀 Достаточно для full training")
                print("   Подход: Full training (5-14 дней, качество 4-5/5)")
        
        print("=" * 60)
        print()
        
        return success_count > 0


def main():
    parser = argparse.ArgumentParser(
        description="Запись TTS датасета для ROBBOX через Yandex TTS API v3 (gRPC)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры использования:

  # Базовая запись (с настройками из вашего скрипта)
  python3 record_yandex_voice.py \\
    --input ../dataset/robbox_sentences_example.txt \\
    --output ~/robbox_tts_training/datasets/robbox_voice/

  # С пользовательскими настройками
  python3 record_yandex_voice.py \\
    --input ../dataset/my_sentences.txt \\
    --output ~/robbox_tts_training/datasets/my_voice/ \\
    --voice anton \\
    --speed 0.4 \\
    --delay 2.0

  # Продолжить с фразы 150 (если прервалось)
  python3 record_yandex_voice.py \\
    --input ../dataset/my_sentences.txt \\
    --output ~/robbox_tts_training/datasets/my_voice/ \\
    --resume-from 150

Переменные окружения:
  YANDEX_API_KEY    - API ключ Yandex Cloud
  YANDEX_IAM_TOKEN  - IAM токен (альтернатива API key)
  YANDEX_FOLDER_ID  - Folder ID из Yandex Cloud

Получение ключей:
  API Key: https://cloud.yandex.ru/docs/iam/operations/api-key/create
  IAM Token: yc iam create-token
  Folder ID: yc config list

Важно:
  - Голос "anton" доступен ТОЛЬКО в API v3!
  - IAM токен действует 12 часов
  - API Key действует постоянно (рекомендуется)
  - Speed 0.4 = медленная чёткая речь (как в вашем скрипте)
  - Файлы сохраняются в оригинальном качестве 22050 Hz
        """
    )
    
    parser.add_argument(
        "--input",
        required=True,
        help="Входной файл с текстами (одна строка = одна фраза)"
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Выходная директория для WAV файлов и metadata.csv"
    )
    
    parser.add_argument(
        "--voice",
        default="anton",
        choices=["anton", "lea", "madi", "marina"],
        help="Голос (только v3, по умолчанию: anton как в вашем скрипте)"
    )
    
    parser.add_argument(
        "--speed",
        type=float,
        default=0.4,
        help="Скорость речи 0.1-3.0 (по умолчанию: 0.4 как в вашем скрипте)"
    )
    
    parser.add_argument(
        "--delay",
        type=float,
        default=1.5,
        help="Задержка между запросами в секундах (по умолчанию: 1.5)"
    )
    
    parser.add_argument(
        "--resume-from",
        type=int,
        default=0,
        help="Продолжить с N-ой фразы (для восстановления)"
    )
    
    args = parser.parse_args()
    
    # Получаем API ключ
    api_key = os.getenv("YANDEX_API_KEY") or os.getenv("YANDEX_IAM_TOKEN")
    folder_id = os.getenv("YANDEX_FOLDER_ID")
    
    if not api_key:
        print("❌ API ключ не найден!")
        print("\nУстановите одну из переменных:")
        print("  export YANDEX_API_KEY='ваш_api_key'")
        print("  export YANDEX_IAM_TOKEN='ваш_iam_token'")
        print("\nИли загрузите из .env.secrets:")
        print("  source ../.env.secrets")
        return 1
    
    if not folder_id:
        print("❌ YANDEX_FOLDER_ID не установлен!")
        print("\nУстановите:")
        print("  export YANDEX_FOLDER_ID='ваш_folder_id'")
        print("\nИли загрузите из .env.secrets:")
        print("  source ../.env.secrets")
        return 1
    
    # Проверка параметров
    if not 0.1 <= args.speed <= 3.0:
        print(f"❌ Speed должен быть между 0.1 и 3.0 (указано: {args.speed})")
        return 1
    
    if args.delay < 0:
        print(f"❌ Delay не может быть отрицательным (указано: {args.delay})")
        return 1
    
    # Создаём рекордер
    try:
        recorder = YandexTTSRecorderV3(
            api_key=api_key,
            folder_id=folder_id,
            voice=args.voice,
            speed=args.speed
        )
    except Exception as e:
        print(f"❌ Ошибка инициализации: {e}")
        return 1
    
    # Запись датасета
    input_path = Path(args.input)
    output_path = Path(args.output)
    
    success = recorder.record_dataset(
        input_file=input_path,
        output_dir=output_path,
        delay=args.delay,
        resume_from=args.resume_from
    )
    
    if success:
        print("\n✅ Датасет успешно записан!")
        print(f"\n📂 Файлы сохранены в: {output_path}")
        print(f"📄 Метаданные: {output_path / 'metadata.csv'}")
        print("\n🎓 Следующий шаг: Обучение модели")
        print("   cd ../training")
        print(f"   python3 train_piper.py --dataset {output_path} --output ~/robbox_tts_training/models/robbox_piper")
        return 0
    else:
        print("\n❌ Ошибка при записи датасета")
        return 1


if __name__ == "__main__":
    sys.exit(main())
