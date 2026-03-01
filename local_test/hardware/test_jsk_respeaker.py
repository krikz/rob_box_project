#!/usr/bin/env python3
"""
Тест JSK ReSpeaker код - проверяем работает ли их подход
"""

import sys
import os
sys.path.insert(0, '/tmp/jsk_3rdparty/respeaker_ros/src')

from respeaker_ros import RespeakerInterface, RespeakerAudio, ignore_stderr
import time

print("=" * 60)
print("ТЕСТ JSK-ROS-PKG RESPEAKER")
print("=" * 60)

# 1. Тест RespeakerInterface (USB)
print("\n1. Тест RespeakerInterface (USB параметры)...")
try:
    respeaker = RespeakerInterface()
    print(f"   ✓ RespeakerInterface создан")
    print(f"   ✓ Version: {respeaker.version}")
    
    # Читаем VAD
    print("\n2. Читаем VAD в цикле (5 секунд)...")
    for i in range(50):
        try:
            vad = respeaker.is_voice()
            direction = respeaker.direction
            if vad:
                print(f"   🎙️  VAD: 1 - РЕЧЬ! Direction: {direction}°")
            else:
                if i % 10 == 0:
                    print(f"   VAD: 0 - тишина, Direction: {direction}°")
        except Exception as e:
            print(f"   ⚠️  Ошибка чтения: {e}")
        time.sleep(0.1)
    
    print("\n✅ USB интерфейс работает!")
    
except Exception as e:
    print(f"\n❌ Ошибка USB: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)

# 2. Тест RespeakerAudio (PyAudio)
print("\n3. Тест RespeakerAudio (PyAudio stream)...")

audio_data_count = 0

def on_audio(data, rate, bitdepth, channels):
    global audio_data_count
    audio_data_count += 1
    if audio_data_count % 100 == 0:
        print(f"   Audio chunk #{audio_data_count}: rate={rate}, channels={channels}, len={len(data)}")

try:
    print("   Создаем RespeakerAudio с suppress_error=True...")
    audio = RespeakerAudio(on_audio, channel=0, suppress_error=True)
    print(f"   ✓ Channels: {audio.channels}")
    print(f"   ✓ Rate: {audio.rate}")
    print(f"   ✓ Device index: {audio.device_index}")
    
    print("\n   Запускаем stream на 5 секунд...")
    audio.start()
    time.sleep(5)
    audio.stop()
    
    print(f"\n✅ PyAudio работает! Получено {audio_data_count} аудио чанков")
    
except Exception as e:
    print(f"\n❌ Ошибка PyAudio: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("4. Тест ОДНОВРЕМЕННОЙ работы USB VAD + PyAudio...")
print("=" * 60)

# 3. Тест одновременной работы
try:
    # Создаем заново
    respeaker2 = RespeakerInterface()
    audio2 = RespeakerAudio(on_audio, channel=0, suppress_error=True)
    
    print("\n   Запускаем PyAudio stream...")
    audio2.start()
    
    print("   Пробуем читать VAD ПОКА PyAudio работает...")
    vad_success = 0
    vad_errors = 0
    
    for i in range(50):
        try:
            vad = respeaker2.is_voice()
            direction = respeaker2.direction
            vad_success += 1
            if vad:
                print(f"   🎙️  VAD: 1 - РЕЧЬ! Direction: {direction}° [успех #{vad_success}]")
            else:
                if i % 10 == 0:
                    print(f"   VAD: 0 - тишина, Direction: {direction}° [успех #{vad_success}]")
        except Exception as e:
            vad_errors += 1
            if vad_errors <= 5:  # Показываем только первые 5 ошибок
                print(f"   ⚠️  Ошибка чтения VAD: {e}")
        
        time.sleep(0.1)
    
    audio2.stop()
    
    print(f"\n   Результат: успешных чтений VAD: {vad_success}, ошибок: {vad_errors}")
    
    if vad_success > 0:
        print("\n✅ ОДНОВРЕМЕННАЯ работа USB VAD + PyAudio ВОЗМОЖНА!")
    else:
        print("\n❌ ОДНОВРЕМЕННАЯ работа НЕ работает - все VAD чтения провалились")
    
except Exception as e:
    print(f"\n❌ Ошибка одновременной работы: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("ТЕСТ ЗАВЕРШЕН")
print("=" * 60)
