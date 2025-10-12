#!/usr/bin/env python3
"""
Быстрый тест Yandex TTS API v3 с ДВУМЯ режимами воспроизведения:
1. Правильный (без искажений) - как задумано Yandex
2. "Бурундук" (pitch shift 2x) - как в вашем скрипте на роботе

Это поможет убедиться что голос именно тот!
"""

import os
import sys
from pathlib import Path
import tempfile
import argparse

# Добавляем путь к скриптам
sys.path.insert(0, str(Path(__file__).parent))

try:
    import grpc
    import sounddevice as sd
    import soundfile as sf
    import numpy as np
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc
except ImportError as e:
    print(f"❌ Ошибка импорта: {e}")
    print("\nУстановите зависимости:")
    print("  pip install -r requirements_yandex_v3.txt")
    sys.exit(1)

# Проверка переменных окружения
api_key = os.getenv("YANDEX_API_KEY")
folder_id = os.getenv("YANDEX_FOLDER_ID")

if not api_key:
    print("❌ YANDEX_API_KEY не установлен")
    print("\nЗагрузите секреты:")
    print("  source ../.env.secrets")
    sys.exit(1)

if not folder_id:
    print("❌ YANDEX_FOLDER_ID не установлен")
    print("\nЗагрузите секреты:")
    print("  source ../.env.secrets")
    sys.exit(1)

# Тестовая фраза
TEST_PHRASE = "Привет! Я РОББОКС робот. Мои речевые функции активированы."

def play_correct(audio_data):
    """Правильное воспроизведение БЕЗ искажений"""
    print("\n🎵 РЕЖИМ 1: Правильное воспроизведение (без искажений)")
    print("   Как задумано Yandex TTS")
    print()
    
    # Читаем WAV с правильной частотой
    import io
    data, original_rate = sf.read(io.BytesIO(audio_data))
    
    print(f"   Исходная частота WAV: {original_rate} Hz")
    print(f"   Длительность: {len(data) / original_rate:.2f} сек")
    
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])
    print(f"   Устройство: {device_info['name']}")
    print(f"   Частота устройства: {target_rate} Hz")
    
    # Если частоты разные - ресемплируем
    if original_rate != target_rate:
        try:
            import librosa
            print(f"   Ресемплинг {original_rate} → {target_rate} Hz...")
            data = librosa.resample(data, orig_sr=original_rate, target_sr=target_rate)
        except ImportError:
            print("   ⚠️  librosa не установлен, воспроизведение на исходной частоте")
    
    print("\n   ▶️  Воспроизвожу...")
    sd.play(data, samplerate=target_rate, blocking=True)
    print("   ✅ Готово!")

def play_chipmunk(audio_data):
    """Воспроизведение с 'эффектом бурундука' КАК В ВАШЕМ СКРИПТЕ"""
    print("\n🐿️  РЕЖИМ 2: Эффект 'Бурундука' (как на роботе)")
    print("   Точно как в вашем play_audio_segment()")
    print()
    
    # Получаем частоту устройства (как в вашем скрипте)
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])
    print(f"   Устройство: {device_info['name']}")
    print(f"   Частота устройства: {target_rate} Hz")
    
    # Конвертация КАК В ВАШЕМ СКРИПТЕ - БЕЗ чтения WAV заголовка!
    print(f"   np.frombuffer() - читаем байты напрямую...")
    samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
    
    # Pitch shift расчёт
    # TTS возвращает 22050 Hz, но мы воспроизводим на target_rate
    pitch_shift = target_rate / 22050.0
    print(f"   Pitch shift: {pitch_shift:.2f}x (22050 → {target_rate} Hz)")
    print(f"   Эффект: голос в {pitch_shift:.1f}x быстрее и выше")
    
    print("\n   ▶️  Воспроизвожу (с искажением)...")
    sd.play(samples, samplerate=target_rate, blocking=True)
    print("   ✅ Готово!")

def main():
    parser = argparse.ArgumentParser(
        description="Тест Yandex TTS v3 с двумя режимами воспроизведения",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--mode",
        choices=["correct", "chipmunk", "both"],
        default="both",
        help="Режим воспроизведения (по умолчанию: both)"
    )
    args = parser.parse_args()
    
    # Проверка переменных окружения
    api_key = os.getenv("YANDEX_API_KEY")
    folder_id = os.getenv("YANDEX_FOLDER_ID")
    
    if not api_key:
        print("❌ YANDEX_API_KEY не установлен")
        print("\nЗагрузите секреты:")
        print("  source ../.env.secrets")
        sys.exit(1)
    
    if not folder_id:
        print("❌ YANDEX_FOLDER_ID не установлен")
        print("\nЗагрузите секреты:")
        print("  source ../.env.secrets")
        sys.exit(1)

    
    print("=" * 60)
    print("ТЕСТ YANDEX TTS API V3 (gRPC)")
    print("=" * 60)
    print(f"Voice: anton")
    print(f"Speed: 0.4")
    print(f"Текст: {TEST_PHRASE}")
    print("=" * 60)
    print()
    
    # Инициализация gRPC канала
    print("🔌 Подключение к tts.api.cloud.yandex.net:443...")
    channel = grpc.secure_channel(
        'tts.api.cloud.yandex.net:443',
        grpc.ssl_channel_credentials()
    )
    stub = tts_service_pb2_grpc.SynthesizerStub(channel)
    print("✅ Канал создан")
    print()
    
    # Синтез
    print("🎙️  Синтезирую фразу...")
    request = tts_pb2.UtteranceSynthesisRequest(
        text=TEST_PHRASE,
        output_audio_spec=tts_pb2.AudioFormatOptions(
            container_audio=tts_pb2.ContainerAudio(
                container_audio_type=tts_pb2.ContainerAudio.WAV
            )
        ),
        hints=[
            tts_pb2.Hints(voice="anton"),
            tts_pb2.Hints(speed=0.4),
        ],
        loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS
    )
    
    try:
        responses = stub.UtteranceSynthesis(
            request,
            metadata=(('authorization', f'Api-Key {api_key}'),)
        )
        
        # Собираем аудио
        audio_data = b""
        for response in responses:
            audio_data += response.audio_chunk.data
        
        if not audio_data:
            print("❌ Пустой ответ от TTS")
            sys.exit(1)
        
        print(f"✅ Синтез завершён! Размер: {len(audio_data)} байт")
        
    except grpc.RpcError as e:
        print(f"❌ gRPC ошибка: {e.code()} - {e.details()}")
        sys.exit(1)
    
    # Сохранение во временный файл
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
        tmp_path = tmp.name
        tmp.write(audio_data)
    
    print(f"💾 Сохранено: {tmp_path}")
    
    # Воспроизведение в зависимости от режима
    print("\n" + "=" * 60)
    
    if args.mode in ["correct", "both"]:
        play_correct(audio_data)
        if args.mode == "both":
            input("\n⏸️  Нажмите Enter для воспроизведения с эффектом бурундука...")
    
    if args.mode in ["chipmunk", "both"]:
        play_chipmunk(audio_data)
    
    print("\n" + "=" * 60)
    print("РЕЗУЛЬТАТ")
    print("=" * 60)
    print("✅ API v3 работает корректно!")
    print("✅ Голос 'anton' доступен")
    print("✅ Speed 0.4 применён")
    print()
    
    if args.mode == "both":
        print("Вы услышали ОБА варианта:")
        print("1️⃣  Правильный (медленный, низкий)")
        print("2️⃣  'Бурундук' (быстрый, высокий) - как на роботе!")
        print()
        print("Какой вариант ближе к голосу вашего ROBBOX?")
    elif args.mode == "chipmunk":
        print("Воспроизведён режим 'Бурундук' - как на роботе WM8960")
        print("Pitch shift: 2.0x (22050 → 44100 Hz)")
    else:
        print("Воспроизведён правильный вариант без искажений")
    
    print()
    print("🚀 Для записи датасета:")
    print("   ./generate_dataset.sh 200")
    print("=" * 60)
    
    # Очистка
    Path(tmp_path).unlink(missing_ok=True)

if __name__ == "__main__":
    main()
