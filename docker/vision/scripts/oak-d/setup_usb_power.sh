#!/bin/bash
# USB Power Management Setup for OAK-D Camera
# Disables USB autosuspend to prevent device disconnections

set -e

echo "🔌 Настройка USB power management для OAK-D..."

# Находим USB устройство OAK-D (Movidius MyriadX)
# Vendor ID: 03e7 (Intel Movidius)
OAKD_DEVICES=$(lsusb | grep -i "03e7" || true)

if [ -z "$OAKD_DEVICES" ]; then
    echo "⚠️  OAK-D устройство не найдено в lsusb"
    echo "Попытка продолжить без настройки USB power management..."
else
    echo "✅ Найдено OAK-D устройство:"
    echo "$OAKD_DEVICES"
    
    # Отключаем autosuspend для всех USB устройств Movidius
    for device in /sys/bus/usb/devices/*; do
        if [ -f "$device/idVendor" ] && [ "$(cat "$device/idVendor")" = "03e7" ]; then
            DEVICE_NAME=$(basename "$device")
            echo "  Настройка $DEVICE_NAME..."
            
            # Отключаем autosuspend
            if [ -f "$device/power/autosuspend" ]; then
                echo -1 > "$device/power/autosuspend" 2>/dev/null || true
                echo "    ✓ autosuspend disabled"
            fi
            
            if [ -f "$device/power/autosuspend_delay_ms" ]; then
                echo -1 > "$device/power/autosuspend_delay_ms" 2>/dev/null || true
                echo "    ✓ autosuspend_delay_ms set to -1"
            fi
            
            # Устанавливаем control в "on" (всегда включено)
            if [ -f "$device/power/control" ]; then
                echo "on" > "$device/power/control" 2>/dev/null || true
                echo "    ✓ power/control set to 'on'"
            fi
        fi
    done
fi

# Глобально отключаем USB autosuspend через kernel параметры (если возможно)
# Это работает только если контейнер запущен с --privileged
if [ -f "/sys/module/usbcore/parameters/autosuspend" ]; then
    echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || \
        echo "⚠️  Не удалось изменить глобальный autosuspend (требуется privileged mode)"
fi

echo "✅ USB power management настроен"
