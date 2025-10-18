#!/usr/bin/env python3
"""
Простой тест Vosk STT с ReSpeaker
"""

import sounddevice as sd
import json
from vosk import Model, KaldiRecognizer

# Параметры
SAMPLE_RATE = 16000
CHANNELS = 6  # ReSpeaker имеет 6 каналов (4 микрофона + 2 playback reference)
DEVICE = 6  # ReSpeaker - hw:1,0
MODEL_PATH = "/models/vosk-model-small-ru-0.22"

print(f"🎤 Загрузка Vosk модели из {MODEL_PATH}...")
model = Model(MODEL_PATH)
rec = KaldiRecognizer(model, SAMPLE_RATE)

print(f"🎙️ Запись с устройства {DEVICE} (ReSpeaker)")
print("Говорите что-нибудь... (Ctrl+C для остановки)")
print("-" * 50)

def audio_callback(indata, frames, time, status):
    """Callback для обработки аудио"""
    if status:
        print(f"⚠️ Status: {status}")
    
    # Берем только первый канал (микрофон 0) и конвертируем в моно
    mono_audio = indata[:, 0]  # Первый канал
    # Конвертируем float32 в int16 для Vosk
    audio_int16 = (mono_audio * 32767).astype('int16')
    audio_bytes = audio_int16.tobytes()
    
    if rec.AcceptWaveform(audio_bytes):
        # Финальный результат
        result = json.loads(rec.Result())
        text = result.get('text', '')
        if text:
            print(f"✅ Распознано: {text}")
    else:
        # Промежуточный результат
        partial = json.loads(rec.PartialResult())
        text = partial.get('partial', '')
        if text:
            print(f"⏳ Промежуточно: {text}", end='\r')

try:
    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        blocksize=8000,  # 0.5 секунды
        device=DEVICE,
        channels=CHANNELS,
        callback=audio_callback
    ):
        print("✅ Запись запущена!")
        sd.sleep(int(1e9))  # Бесконечный sleep
        
except KeyboardInterrupt:
    print("\n🛑 Остановлено пользователем")
except Exception as e:
    print(f"❌ Ошибка: {e}")
    
    # Покажем доступные устройства
    print("\n📋 Доступные аудио устройства:")
    print(sd.query_devices())
