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

# Инициализация shared volume renardo семплов (один раз, при первом запуске после пересборки)
# Без этого scsynth не может читать WAV файлы из voice-assistant и выдаёт "Buffer UGen: no buffer data"
SAMPLES_VOLUME=/root/.config/renardo/samples
SAMPLES_BUILTIN=/renardo_samples_builtin
if [ ! -f "${SAMPLES_VOLUME}/.initialized" ]; then
    if [ -d "${SAMPLES_BUILTIN}" ] && [ -n "$(ls -A ${SAMPLES_BUILTIN} 2>/dev/null)" ]; then
        echo "Initializing renardo samples shared volume from builtin..."
        mkdir -p "${SAMPLES_VOLUME}"
        cp -rp "${SAMPLES_BUILTIN}/." "${SAMPLES_VOLUME}/"
        touch "${SAMPLES_VOLUME}/.initialized"
        echo "✓ Renardo samples initialized ($(find ${SAMPLES_VOLUME} -name '*.wav' | wc -l) WAV files)"
    else
        echo "⚠ Renardo builtin samples not found at ${SAMPLES_BUILTIN} — synth-only mode"
    fi
else
    echo "✓ Renardo samples volume already initialized"
fi

# Создать директорию для ТТС кэша
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
echo "  Запуск sclang (FoxDot SynthDef compiler)"
echo "=========================================="

# Запуск sclang для компиляции SynthDef-ов (Renardo/FoxDot pipeline)
# sclang слушает OSC /foxdot на порту 57120
# Renardo посылает пути к .scd файлам → sclang компилирует → /d_recv → scsynth
if command -v sclang > /dev/null 2>&1; then
    echo "Запуск sclang с FoxDot OSCdef..."
    QT_QPA_PLATFORM=offscreen QTWEBENGINE_CHROMIUM_FLAGS=--no-sandbox \
        sclang -i none /ws/foxdot_init.sc > /tmp/sclang.log 2>&1 &
    SCLANG_PID=$!
    echo "sclang запущен (PID: ${SCLANG_PID})"
    # Ждём 5с чтобы sclang подключился к scsynth и зарегистрировал OSCdef
    sleep 5
    echo "sclang готов"
else
    echo "⚠ sclang не найден — музыкальный синтез недоступен"
fi

echo ""
echo "=========================================="
echo "  Запуск Voice Assistant Nodes (Headless)"
echo "=========================================="

# Запуск через headless launch file (без animation_player_node)
# Animation player запускается отдельно на Main Pi
if [ -f /config/voice/voice_assistant.yaml ]; then
    echo "Используется конфигурация: /config/voice/voice_assistant.yaml"
    echo "Используется headless launch (без animation_player_node)"
    exec ros2 launch /config/voice/voice_assistant_headless.launch.py \
        config_file:=/config/voice/voice_assistant.yaml
else
    echo "Используется конфигурация по умолчанию"
    exec ros2 launch /config/voice/voice_assistant_headless.launch.py
fi
