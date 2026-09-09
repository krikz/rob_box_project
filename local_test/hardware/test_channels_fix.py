#!/usr/bin/env python3
"""
Test channels parameter fix - проверка исправления конфликта каналов
"""

import numpy as np
import sounddevice as sd
import time

def find_respeaker_device():
    """Найти ReSpeaker устройство"""
    devices = sd.query_devices()
    for i, device in enumerate(devices):
        if 'ReSpeaker' in device['name'] or 'UAC1.0' in device['name']:
            print(f"🎤 Найден ReSpeaker: {device['name']} (index: {i})")
            return i
    return None

def test_mono_stereo_fixed():
    """Тест mono и stereo воспроизведения после исправления"""
    device_index = find_respeaker_device()
    if device_index is None:
        print("❌ ReSpeaker не найден")
        return
    
    print(f"🔊 Тестирование устройства {device_index}")
    
    # Создаем тоны
    duration = 0.5  # секунды
    sample_rate = 16000
    frequency = 440  # A4 note
    
    # Временной массив
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    
    # 1. Тест MONO (1D array)
    print("\n1️⃣ Тест MONO (без channels parameter)")
    mono_tone = 0.3 * np.sin(2 * np.pi * frequency * t).astype(np.float32)
    print(f"   Shape: {mono_tone.shape}, dtype: {mono_tone.dtype}")
    
    try:
        sd.play(mono_tone, samplerate=sample_rate, device=device_index)
        sd.wait()
        print("   ✅ MONO успешно")
        time.sleep(0.2)
    except Exception as e:
        print(f"   ❌ MONO ошибка: {e}")
    
    # Cleanup
    sd.stop()
    time.sleep(0.1)
    
    # 2. Тест STEREO (2D array)
    print("\n2️⃣ Тест STEREO (без channels parameter)")
    stereo_tone = np.column_stack((mono_tone, mono_tone))  # [samples, 2]
    print(f"   Shape: {stereo_tone.shape}, dtype: {stereo_tone.dtype}")
    
    try:
        sd.play(stereo_tone, samplerate=sample_rate, device=device_index)
        sd.wait()
        print("   ✅ STEREO успешно")
        time.sleep(0.2)
    except Exception as e:
        print(f"   ❌ STEREO ошибка: {e}")
    
    # Cleanup
    sd.stop()
    time.sleep(0.1)
    
    # 3. Тест с EXPLICIT channels (должен давать ошибку)
    print("\n3️⃣ Тест с explicit channels=2 (должен быть ошибка)")
    try:
        sd.play(stereo_tone, samplerate=sample_rate, device=device_index, channels=2)
        sd.wait()
        print("   ⚠️ Неожиданно: explicit channels работает")
    except Exception as e:
        print(f"   ✅ Ожидаемая ошибка с explicit channels: {e}")
    
    # Final cleanup
    sd.stop()
    time.sleep(0.1)
    print("\n🧹 Cleanup завершен")

if __name__ == "__main__":
    print("🔧 Test channels parameter fix")
    test_mono_stereo_fixed()