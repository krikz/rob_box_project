#!/usr/bin/env python3
"""
Скрипт для настройки Acoustic Echo Cancellation (AEC) на ReSpeaker Mic Array v2.0
Запустить ПЕРЕД запуском audio_node для оптимального качества аудио.

Настраиваемые параметры:
- AECFREEZEONOFF: 0 = адаптивный AEC (рекомендуется)
- ECHOONOFF: 1 = включить эхоподавление
- NLATTENONOFF: 1 = включить нелинейное подавление
- Channel 0 для STT (обработанный сигнал с AEC)

Usage:
    python3 configure_respeaker_aec.py
    # или через ROS2:
    ros2 run rob_box_voice configure_respeaker_aec.py
"""

import sys
import os
import time

# Add path для импорта utils
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from rob_box_voice.utils.respeaker_interface import ReSpeakerInterface


def main():
    print("=" * 60)
    print("ReSpeaker Mic Array v2.0 - AEC Configuration")
    print("=" * 60)
    
    # Подключиться к ReSpeaker
    respeaker = ReSpeakerInterface()
    
    print("\n[1/4] Подключение к ReSpeaker...")
    if not respeaker.connect():
        print("❌ Не удалось подключиться к ReSpeaker!")
        print("   Проверьте:")
        print("   - Подключен ли ReSpeaker USB?")
        print("   - Права доступа: sudo usermod -a -G plugdev $USER")
        return 1
    
    print("✅ ReSpeaker подключен")
    
    # Показать информацию об устройстве
    device_info = respeaker.get_device_info()
    if device_info:
        print(f"   Производитель: {device_info['manufacturer']}")
        print(f"   Продукт: {device_info['product']}")
        print(f"   Vendor ID: {device_info['vendor_id']}")
        print(f"   Product ID: {device_info['product_id']}")
    
    # Настроить AEC
    print("\n[2/4] Настройка AEC параметров...")
    print("   AECFREEZEONOFF = 0 (адаптивный)")
    print("   ECHOONOFF = 1 (включено)")
    print("   NLATTENONOFF = 1 (включено)")
    
    success = respeaker.configure_aec(
        aec_on=True,        # Включить эхоподавление
        aec_freeze=False,   # Адаптивный режим (не замораживать)
        nlp_on=True         # Нелинейное подавление
    )
    
    if success:
        print("✅ AEC параметры установлены успешно")
    else:
        print("⚠️  Не все параметры удалось установить")
    
    # Проверить текущие значения
    print("\n[3/4] Проверка установленных значений...")
    aec_freeze = respeaker.read_parameter('AECFREEZEONOFF')
    echo_on = respeaker.read_parameter('ECHOONOFF')
    nlp_on = respeaker.read_parameter('NLATTENONOFF')
    
    print(f"   AECFREEZEONOFF: {aec_freeze} {'✓' if aec_freeze == 0 else '✗'}")
    print(f"   ECHOONOFF: {echo_on} {'✓' if echo_on == 1 else '✗'}")
    print(f"   NLATTENONOFF: {nlp_on} {'✓' if nlp_on == 1 else '✗'}")
    
    # Рекомендации
    print("\n[4/4] Рекомендации для STT:")
    print("   ✓ Используйте Channel 0 для STT (обработанный сигнал)")
    print("   ✓ В audio_node установите: channels=1, device_name='ReSpeaker 4 Mic Array'")
    print("   ✓ AEC настройки сохраняются до перезагрузки устройства")
    
    print("\n" + "=" * 60)
    print("Готово! Теперь можно запускать audio_node")
    print("=" * 60)
    
    respeaker.disconnect()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nПрервано пользователем")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
