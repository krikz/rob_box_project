#!/bin/bash
# Startup script для Voice Assistant контейнера

set -e

echo "=========================================="
echo "  Voice Assistant System Starting"
echo "=========================================="

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash

# Проверка подключения ReSpeaker
echo "Проверка ReSpeaker USB device..."
if lsusb | grep -q "2886:0018"; then
    echo "✓ ReSpeaker найден"
else
    echo "⚠ ReSpeaker НЕ найден!"
fi

# Проверка аудио устройства
echo "Проверка аудио устройств..."
arecord -l | grep -i "respeaker" || echo "⚠ ReSpeaker audio не найден"

# Создать директорию для TTS кэша
mkdir -p ${TTS_CACHE_DIR}

# Настройка ALSA (если нужно)
if [ -f /config/voice/asoundrc ]; then
    echo "Копирование ALSA конфигурации..."
    cp /config/voice/asoundrc /root/.asoundrc
fi

# Ожидание Zenoh router
echo "Ожидание Zenoh router..."
RETRY_COUNT=0
MAX_RETRIES=30

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if wget -qO- http://localhost:8000/@/local/router > /dev/null 2>&1; then
        echo "✓ Zenoh router доступен"
        break
    fi
    echo "Попытка $((RETRY_COUNT + 1))/$MAX_RETRIES..."
    sleep 2
    RETRY_COUNT=$((RETRY_COUNT + 1))
done

if [ $RETRY_COUNT -eq $MAX_RETRIES ]; then
    echo "✗ Zenoh router недоступен после $MAX_RETRIES попыток"
    echo "Продолжаем без Zenoh (local mode)"
fi

echo ""
echo "=========================================="
echo "  Запуск Voice Assistant Nodes"
echo "=========================================="

# Запуск через launch file
if [ -f /config/voice/voice_assistant.yaml ]; then
    echo "Используется конфигурация: /config/voice/voice_assistant.yaml"
    exec ros2 launch rob_box_voice voice_assistant.launch.py \
        config_file:=/config/voice/voice_assistant.yaml
else
    echo "Используется конфигурация по умолчанию"
    exec ros2 launch rob_box_voice voice_assistant.launch.py
fi
