#!/usr/bin/env python3
"""
Диагностический скрипт для измерения белого шума ReSpeaker.

Измеряет уровень шума до и после воспроизведения звука,
чтобы количественно оценить проблему белого шума.

Usage:
    python3 diagnose_white_noise.py

Requirements:
    - ReSpeaker подключен как аудио устройство
    - sounddevice, numpy
"""

import sys
import time
import numpy as np
import sounddevice as sd


def find_respeaker_device():
    """Найти ReSpeaker аудио устройство."""
    devices = sd.query_devices()

    for idx, device in enumerate(devices):
        if "ReSpeaker" in device["name"] or "ArrayUAC10" in device["name"]:
            print(f"✓ Найден ReSpeaker: device {idx} - {device['name']}")
            return idx

    print("❌ ReSpeaker не найден!")
    print("\nДоступные устройства:")
    print(sd.query_devices())
    return None


def measure_noise_level(device_idx, duration=2.0, sample_rate=16000):
    """
    Измерить уровень шума (RMS).

    Args:
        device_idx: Index аудио устройства
        duration: Длительность записи в секундах
        sample_rate: Частота дискретизации

    Returns:
        Tuple (rms, db): RMS уровень и уровень в dB
    """
    print(f"📊 Запись {duration}s аудио для измерения шума...")

    # Запись аудио
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype=np.float32, device=device_idx)
    sd.wait()

    # Расчет RMS (Root Mean Square)
    rms = np.sqrt(np.mean(audio**2))

    # Конвертация в dB (относительно максимального уровня)
    # Reference: 1.0 = full scale
    db = 20 * np.log10(rms + 1e-10)  # +epsilon для избежания log(0)

    return rms, db


def play_test_tone(device_idx, duration=0.5, frequency=440, sample_rate=16000):
    """
    Воспроизвести тестовый тон (как триггер воспроизведения).

    Args:
        device_idx: Index аудио устройства
        duration: Длительность тона в секундах
        frequency: Частота тона в Hz (440 = нота A)
        sample_rate: Частота дискретизации
    """
    print(f"🔊 Воспроизведение тестового тона {frequency}Hz ({duration}s)...")

    # Генерация синусоиды
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = np.sin(frequency * 2 * np.pi * t)

    # Стерео (ReSpeaker требует 2 канала)
    tone_stereo = np.column_stack((tone, tone)).astype(np.float32)

    # Воспроизведение
    sd.play(tone_stereo, sample_rate, device=device_idx)
    sd.wait()

    print("✅ Воспроизведение завершено")


def main():
    """Основная функция диагностики."""
    print("=" * 60)
    print("🔊 ReSpeaker White Noise Diagnostic Tool")
    print("=" * 60)
    print()

    # 1. Найти ReSpeaker
    device_idx = find_respeaker_device()
    if device_idx is None:
        sys.exit(1)

    print()

    # 2. Измерить базовый уровень шума (до воспроизведения)
    print("📊 BASELINE: Измерение уровня шума ДО воспроизведения...")
    baseline_rms, baseline_db = measure_noise_level(device_idx, duration=2.0)
    print(f"   RMS: {baseline_rms:.6f}")
    print(f"   dB:  {baseline_db:.2f} dBFS")
    print()

    # 3. Подождать немного
    time.sleep(0.5)

    # 4. Воспроизвести тестовый звук (триггер белого шума)
    print("🎵 TRIGGER: Воспроизведение тестового тона...")
    play_test_tone(device_idx, duration=0.5)
    print()

    # 5. Подождать стабилизации (как в реальном сценарии)
    print("⏳ Ожидание 0.5s для стабилизации...")
    time.sleep(0.5)
    print()

    # 6. Измерить уровень шума после воспроизведения
    print("📊 AFTER: Измерение уровня шума ПОСЛЕ воспроизведения...")
    after_rms, after_db = measure_noise_level(device_idx, duration=2.0)
    print(f"   RMS: {after_rms:.6f}")
    print(f"   dB:  {after_db:.2f} dBFS")
    print()

    # 7. Анализ результатов
    print("=" * 60)
    print("📈 РЕЗУЛЬТАТЫ АНАЛИЗА")
    print("=" * 60)

    rms_diff = after_rms - baseline_rms
    rms_increase_pct = (rms_diff / baseline_rms) * 100 if baseline_rms > 0 else 0
    db_diff = after_db - baseline_db

    print(f"Baseline noise:     {baseline_rms:.6f} RMS ({baseline_db:.2f} dBFS)")
    print(f"After playback:     {after_rms:.6f} RMS ({after_db:.2f} dBFS)")
    print(f"Difference:         {rms_diff:+.6f} RMS ({db_diff:+.2f} dB)")
    print(f"Increase:           {rms_increase_pct:+.1f}%")
    print()

    # 8. Вердикт
    if rms_increase_pct > 50:
        print("❌ ПРОБЛЕМА: Значительный рост шума после воспроизведения!")
        print("   Рекомендуется применить программное решение для cleanup.")
    elif rms_increase_pct > 20:
        print("⚠️  ВНИМАНИЕ: Умеренный рост шума после воспроизведения")
        print("   Возможно, cleanup улучшит ситуацию.")
    else:
        print("✅ OK: Уровень шума в норме")
        print("   Значительного увеличения шума не обнаружено.")

    print()
    print("=" * 60)
    print("💡 РЕКОМЕНДАЦИИ")
    print("=" * 60)
    print("1. Запустите скрипт несколько раз для статистики")
    print("2. Сравните результаты до и после применения noise cleanup")
    print("3. Используйте этот скрипт для A/B тестирования решений")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Прервано пользователем")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
