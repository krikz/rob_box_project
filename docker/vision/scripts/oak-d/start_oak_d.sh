#!/bin/bash
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# Настройка USB power management для предотвращения disconnects
if [ -f "/scripts/setup_usb_power.sh" ]; then
    echo "🔌 Настройка USB power management..."
    bash /scripts/setup_usb_power.sh
else
    echo "⚠️  setup_usb_power.sh не найден, пропускаем настройку USB"
fi

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D с интегрированной детекцией AprilTag
# Это объединяет камеру и AprilTag детектор в одном контейнере
# для снижения нагрузки на Vision Pi
cd /oak-d/launch || exit 1
exec ros2 launch oakd_with_apriltag.launch.py
