#!/usr/bin/env python3
"""
Упрощенный тест JSK подхода - БЕЗ ROS зависимостей
Проверяем может ли USB и PyAudio работать одновременно
"""

import usb.core
import usb.util
import pyaudio
import struct
import time
import os
import sys
from contextlib import contextmanager

# ignore_stderr из их кода
@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield

print("=" * 70)
print("ТЕСТ: Может ли USB VAD читаться ПОКА PyAudio стримит?")
print("=" * 70)

# 1. Найти устройство
print("\n1. Поиск ReSpeaker...")
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
if dev is None:
    print("❌ ReSpeaker не найден!")
    sys.exit(1)

print(f"✓ ReSpeaker найден: Bus {dev.bus} Device {dev.address}")

# 2. БЕЗ reset!!! (как мы теперь делаем)
print("\n2. НЕ делаем dev.reset() (он убивает audio interface)")

# 3. Создать PyAudio
print("\n3. Создаем PyAudio (с suppress_error)...")
with ignore_stderr(enable=True):
    pa = pyaudio.PyAudio()

# 4. Найти ReSpeaker audio device
print("\n4. Ищем ReSpeaker audio device...")
device_index = None
for i in range(pa.get_device_count()):
    info = pa.get_device_info_by_index(i)
    if 'ReSpeaker' in str(info['name']):
        device_index = i
        print(f"   ✓ Найден: index={i}, name={info['name']}, channels={info['maxInputChannels']}")
        break

if device_index is None:
    print("❌ ReSpeaker audio device не найден!")
    sys.exit(1)

# 5. Открыть audio stream
print("\n5. Открываем PyAudio stream (6 каналов, без callback)...")
try:
    stream = pa.open(
        format=pyaudio.paInt16,
        channels=6,  # RAW mode
        rate=16000,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=1024,
        start=False  # Не запускаем пока
    )
    print("   ✓ Stream создан")
except Exception as e:
    print(f"   ❌ Ошибка создания stream: {e}")
    sys.exit(1)

# 6. Тест USB VAD ДО запуска stream
print("\n6. Тест USB VAD ДО запуска PyAudio stream...")
vad_before = 0
for i in range(10):
    try:
        response = dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80 | 32, 19, 8, 100000)
        vad = struct.unpack(b'ii', response.tobytes())[0]
        vad_before += 1
        if i == 0:
            print(f"   ✓ VAD читается: {vad}")
    except Exception as e:
        print(f"   ⚠️  Ошибка: {e}")
    time.sleep(0.1)

print(f"   Результат: {vad_before}/10 успешных чтений")

# 7. ЗАПУСКАЕМ stream
print("\n7. ЗАПУСКАЕМ PyAudio stream...")
stream.start_stream()
print("   ✓ Stream запущен")

# 8. Читаем немного аудио
print("\n8. Читаем 5 аудио чанков...")
for i in range(5):
    data = stream.read(1024, exception_on_overflow=False)
    print(f"   Audio chunk {i+1}: {len(data)} bytes")
    time.sleep(0.1)

# 9. Тест USB VAD ВО ВРЕМЯ stream
print("\n9. Тест USB VAD ВО ВРЕМЯ работы PyAudio stream...")
vad_during = 0
vad_errors = 0

for i in range(20):
    try:
        response = dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80 | 32, 19, 8, 100000)
        vad = struct.unpack(b'ii', response.tobytes())[0]
        vad_during += 1
        if vad:
            print(f"   🎙️  VAD: {vad} - РЕЧЬ!")
        else:
            if i % 5 == 0:
                print(f"   VAD: {vad} - тишина [успех #{vad_during}]")
    except Exception as e:
        vad_errors += 1
        if vad_errors <= 3:
            print(f"   ⚠️  Ошибка #{vad_errors}: {e}")
    
    time.sleep(0.1)

print(f"\n   Результат: {vad_during}/20 успешных чтений, {vad_errors} ошибок")

# 10. Останавливаем stream
print("\n10. Останавливаем PyAudio stream...")
stream.stop_stream()
stream.close()
pa.terminate()
print("   ✓ Stream закрыт")

# 11. Тест USB VAD ПОСЛЕ остановки stream
print("\n11. Тест USB VAD ПОСЛЕ остановки PyAudio stream...")
vad_after = 0
for i in range(10):
    try:
        response = dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80 | 32, 19, 8, 100000)
        vad = struct.unpack(b'ii', response.tobytes())[0]
        vad_after += 1
        if i == 0:
            print(f"   ✓ VAD читается: {vad}")
    except Exception as e:
        print(f"   ⚠️  Ошибка: {e}")
    time.sleep(0.1)

print(f"   Результат: {vad_after}/10 успешных чтений")

# РЕЗУЛЬТАТЫ
print("\n" + "=" * 70)
print("РЕЗУЛЬТАТЫ:")
print("=" * 70)
print(f"USB VAD ДО stream:      {vad_before}/10  успешных")
print(f"USB VAD ВО ВРЕМЯ stream: {vad_during}/20  успешных, {vad_errors} ошибок")
print(f"USB VAD ПОСЛЕ stream:    {vad_after}/10  успешных")

if vad_during > 0:
    print("\n✅ ВЫВОД: USB VAD РАБОТАЕТ во время PyAudio stream!")
    print(f"   Процент успеха: {vad_during * 100 / 20:.0f}%")
else:
    print("\n❌ ВЫВОД: USB VAD НЕ РАБОТАЕТ во время PyAudio stream!")
    print("   Это подтверждает что одновременный доступ невозможен")

print("=" * 70)
