#!/usr/bin/env python3
"""
Тестовый скрипт для проверки ReSpeaker без ROS2
Проверяет: USB подключение, VAD, DoA, LED
"""

import sys
import time
from rob_box_voice.utils.respeaker_interface import ReSpeakerInterface
from rob_box_voice.utils.audio_utils import find_respeaker_device, list_audio_devices
import pyaudio


def test_usb_connection():
    """Тест 1: Проверка USB подключения"""
    print("\n=== Тест 1: USB подключение ===")
    
    respeaker = ReSpeakerInterface()
    if respeaker.connect():
        print("✓ ReSpeaker найден!")
        
        info = respeaker.get_device_info()
        if info:
            print(f"  Vendor ID: {info['vendor_id']}")
            print(f"  Product ID: {info['product_id']}")
            print(f"  Manufacturer: {info['manufacturer']}")
            print(f"  Product: {info['product']}")
        
        respeaker.disconnect()
        return True
    else:
        print("✗ ReSpeaker НЕ найден!")
        return False


def test_audio_device():
    """Тест 2: Проверка аудио устройства"""
    print("\n=== Тест 2: Аудио устройства ===")
    
    p = pyaudio.PyAudio()
    
    print("\nДоступные аудио устройства:")
    devices = list_audio_devices(p)
    for dev in devices:
        marker = "👉" if "ReSpeaker" in dev['name'] else "  "
        print(f"{marker} [{dev['index']}] {dev['name']}")
        print(f"      {dev['channels']}ch, {dev['sample_rate']}Hz")
    
    device_index = find_respeaker_device(p)
    if device_index is not None:
        print(f"\n✓ ReSpeaker аудио найден: index={device_index}")
        p.terminate()
        return True
    else:
        print("\n✗ ReSpeaker аудио НЕ найден!")
        p.terminate()
        return False


def test_vad_doa():
    """Тест 3: Проверка VAD и DoA"""
    print("\n=== Тест 3: VAD и DoA ===")
    print("Говорите в микрофон. Ctrl+C для остановки.\n")
    
    respeaker = ReSpeakerInterface()
    if not respeaker.connect():
        print("✗ Не удалось подключиться")
        return False
    
    # Настройка параметров
    respeaker.configure_audio_processing(agc=True, noise_suppression=True)
    respeaker.set_vad_threshold(3.5)
    
    try:
        for i in range(50):  # 5 секунд
            vad = respeaker.get_vad()
            doa = respeaker.get_doa()
            
            vad_status = "🎤 РЕЧЬ" if vad else "🔇 тишина"
            doa_str = f"{doa:3d}°" if doa is not None else "---"
            
            print(f"\r{vad_status} | DoA: {doa_str}", end='', flush=True)
            time.sleep(0.1)
        
        print("\n\n✓ VAD и DoA работают!")
        return True
        
    except KeyboardInterrupt:
        print("\n\n✓ Тест прерван пользователем")
        return True
    finally:
        respeaker.disconnect()


def test_led():
    """Тест 4: Проверка LED"""
    print("\n=== Тест 4: LED индикация ===")
    
    try:
        from rob_box_voice.led_node import PixelRingLite
    except ImportError:
        print("✗ Не удалось импортировать PixelRingLite")
        return False
    
    pixel_ring = PixelRingLite()
    if not pixel_ring.connect():
        print("✗ Не удалось подключиться к LED")
        return False
    
    print("Тестирование LED режимов...")
    
    try:
        # Установить яркость
        pixel_ring.set_brightness(16)
        print("  Яркость: 16/31")
        
        # Красный
        print("  Красный...", end='', flush=True)
        pixel_ring.mono(255, 0, 0)
        time.sleep(1)
        print(" ✓")
        
        # Зелёный
        print("  Зелёный...", end='', flush=True)
        pixel_ring.mono(0, 255, 0)
        time.sleep(1)
        print(" ✓")
        
        # Синий
        print("  Синий...", end='', flush=True)
        pixel_ring.mono(0, 0, 255)
        time.sleep(1)
        print(" ✓")
        
        # Think (пульсация)
        print("  Think (пульсация)...", end='', flush=True)
        pixel_ring.think()
        time.sleep(2)
        print(" ✓")
        
        # Speak (вращение)
        print("  Speak (вращение)...", end='', flush=True)
        pixel_ring.speak()
        time.sleep(2)
        print(" ✓")
        
        # Выключить
        print("  Выключение...", end='', flush=True)
        pixel_ring.off()
        time.sleep(0.5)
        print(" ✓")
        
        print("\n✓ LED работает!")
        return True
        
    except KeyboardInterrupt:
        print("\n✗ Тест прерван")
        pixel_ring.off()
        return False
    except Exception as e:
        print(f"\n✗ Ошибка: {e}")
        pixel_ring.off()
        return False


def main():
    print("=" * 60)
    print("  ReSpeaker Mic Array v2.0 - Тест подключения")
    print("=" * 60)
    
    tests = [
        ("USB подключение", test_usb_connection),
        ("Аудио устройство", test_audio_device),
        ("VAD и DoA", test_vad_doa),
        ("LED индикация", test_led),
    ]
    
    results = []
    
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n✗ Ошибка в тесте '{name}': {e}")
            results.append((name, False))
    
    # Итоги
    print("\n" + "=" * 60)
    print("  Результаты тестов")
    print("=" * 60)
    
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}  {name}")
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    
    print(f"\nИтого: {passed}/{total} тестов пройдено")
    
    if passed == total:
        print("\n🎉 Все тесты пройдены! ReSpeaker готов к работе.")
        return 0
    else:
        print("\n⚠️  Некоторые тесты не прошли. Проверьте подключение.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
