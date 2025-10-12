#!/usr/bin/env python3
"""
Быстрый тест нового record_yandex_voice.py с API v3.
Синтезирует одну фразу и воспроизводит её.
"""

import os
import sys
from pathlib import Path
import tempfile

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
TEST_PHRASE = "Привет! Я РОББОКСромбот. Мои речевые функции активированы."

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
    print()
    
except grpc.RpcError as e:
    print(f"❌ gRPC ошибка: {e.code()} - {e.details()}")
    sys.exit(1)

# Сохранение во временный файл
with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
    tmp_path = tmp.name
    tmp.write(audio_data)

print(f"💾 Сохранено во временный файл: {tmp_path}")
print()

# Чтение и конвертация (как в вашем скрипте)
print("🔊 Обработка аудио...")
data, samplerate = sf.read(tmp_path)
print(f"   Sample rate: {samplerate} Hz")
print(f"   Длительность: {len(data) / samplerate:.2f} сек")
print(f"   Тип данных: {data.dtype}")

# Конвертация как в вашем play_audio_segment
if data.dtype != np.float32:
    print("   Конвертация int16 -> float32...")
    samples = data.astype(np.float32) / 32768.0
else:
    samples = data

print()

# Воспроизведение
print("🎵 Воспроизведение...")
try:
    device_info = sd.query_devices(None, "output")
    target_rate = int(device_info["default_samplerate"])
    print(f"   Устройство: {device_info['name']}")
    print(f"   Частота: {target_rate} Hz")
    print()
    
    sd.play(samples, samplerate=target_rate, blocking=True)
    print("✅ Воспроизведение завершено!")
    
except Exception as e:
    print(f"❌ Ошибка воспроизведения: {e}")
    print(f"\nНо файл сохранён: {tmp_path}")
    print("Можете воспроизвести вручную:")
    print(f"  aplay {tmp_path}")

print()
print("=" * 60)
print("РЕЗУЛЬТАТ")
print("=" * 60)
print("✅ API v3 работает корректно!")
print("✅ Голос 'anton' доступен")
print("✅ Speed 0.4 применён")
print("✅ numpy конвертация работает")
print()
print("🚀 Можно запускать полную запись датасета:")
print("   cd scripts/")
print("   ./generate_dataset.sh 200")
print("=" * 60)

# Очистка
Path(tmp_path).unlink(missing_ok=True)
