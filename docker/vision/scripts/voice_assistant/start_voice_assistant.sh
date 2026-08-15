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

# Проверка shared volume renardo семплов.
# Инициализация теперь выполняется отдельным one-shot контейнером voice-resources-init.
SAMPLES_VOLUME=/root/.config/renardo/samples
if [ -f "${SAMPLES_VOLUME}/.initialized" ]; then
    echo "✓ Renardo samples volume initialized ($(find "${SAMPLES_VOLUME}" -name '*.wav' | wc -l) WAV files)"
elif [ -d "${SAMPLES_VOLUME}" ] && [ -n "$(find "${SAMPLES_VOLUME}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
    echo "✓ Renardo samples volume present without marker ($(find "${SAMPLES_VOLUME}" -name '*.wav' | wc -l) WAV files)"
else
    echo "⚠ Renardo samples volume is empty — synth-only mode"
fi

# Создать директорию для ТТС кэша
mkdir -p ${TTS_CACHE_DIR}

# Настройка ALSA (если нужно)
if [ -f /config/voice_assistant/asoundrc ]; then
    echo "Копирование ALSA конфигурации..."
    cp /config/voice_assistant/asoundrc /root/.asoundrc
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
    if RENARDO_SCLANG_DIR=$(python3 -c 'import renardo_lib; from pathlib import Path; print(Path(renardo_lib.__file__).resolve().parent / "SynthDefManagement" / "sclang_code" / "scsynth")' 2>/dev/null); then
        echo "Renardo SynthDef dir: ${RENARDO_SCLANG_DIR}"
        sed "s|__RENARDO_SCLANG_DIR__|${RENARDO_SCLANG_DIR}|g" /ws/foxdot_init.sc > /tmp/foxdot_init_resolved.sc
    else
        echo "⚠ Не удалось определить путь к Renardo SynthDefs"
        cp /ws/foxdot_init.sc /tmp/foxdot_init_resolved.sc
    fi
    echo "Запуск sclang с FoxDot OSCdef..."
    QT_QPA_PLATFORM=offscreen QTWEBENGINE_CHROMIUM_FLAGS=--no-sandbox \
        sclang -i none /tmp/foxdot_init_resolved.sc > /tmp/sclang.log 2>&1 &
    SCLANG_PID=$!
    echo "sclang запущен (PID: ${SCLANG_PID})"

    # Критические SynthDef-ы — тот же список, что валидирует
    # validate_music_stack.py (должен совпадать с foxdot_init.sc).
    CRITICAL_SYNTHS=(strings wobblebass pianovel warmpad retrobass supersawlead imperialbrass marchstrings strangerpulsepad strangerarp strangerbrass)

    # Ждём пока sclang зарегистрирует OSCdef И прелоадит все критические
    # SynthDef-ы (максимум 90с). Раньше был фиксированный sleep 5: на
    # медленной загрузке /tmp/sclang.log ещё пуст → validate_music_stack.py
    # считает ВСЕ критические SynthDef-ы отсутствующими → деплой-воркфлоу
    # заводит ложный critical_log "Missing critical SynthDefs: strings, ..."
    # (issue #1276, ретро 15.08 t_124aa954; тот же паттерн, что ADR-0016
    # для nav2 odom race). Маркер готовности — "SynthDef preload ok: <name>"
    # из foxdot_init.sc, печатается ПОСЛЕ завершения path.load.
    MUSIC_READY_TIMEOUT=90
    MUSIC_READY_ELAPSED=0
    MUSIC_READY=0
    while [ $MUSIC_READY_ELAPSED -lt $MUSIC_READY_TIMEOUT ]; do
        MUSIC_READY=1
        for synth in "${CRITICAL_SYNTHS[@]}"; do
            if ! grep -q "SynthDef preload ok: ${synth}" /tmp/sclang.log 2>/dev/null; then
                MUSIC_READY=0
                break
            fi
        done
        if [ $MUSIC_READY -eq 1 ]; then
            break
        fi
        sleep 2
        MUSIC_READY_ELAPSED=$((MUSIC_READY_ELAPSED + 2))
    done
    if [ $MUSIC_READY -eq 1 ]; then
        echo "✓ sclang OSCdef + критические SynthDef-ы готовы (${MUSIC_READY_ELAPSED}s)"
    else
        echo "⚠ Критические SynthDef-ы не прелоажены за ${MUSIC_READY_TIMEOUT}s — продолжаем с валидацией"
    fi
    if [ -f /ws/src/rob_box_voice/scripts/validate_music_stack.py ]; then
        echo "Проверка music stack readiness..."
        MUSIC_STACK_RC=0
        CRITICAL_ARGS=()
        for synth in "${CRITICAL_SYNTHS[@]}"; do
            CRITICAL_ARGS+=(--critical-synth "$synth")
        done
        python3 /ws/src/rob_box_voice/scripts/validate_music_stack.py \
            /tmp/sclang.log \
            "${CRITICAL_ARGS[@]}" || MUSIC_STACK_RC=$?
        if [ "${MUSIC_STACK_RC}" -eq 0 ]; then
            echo "✓ Music stack validation passed"
        else
            echo "⚠ Music stack validation found non-critical errors (degraded but usable)"
            echo "  └─ Подробности: /tmp/sclang.log"
        fi
    fi
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
# Issue #1004 fix (ADR-0004): launch грузит per-node YAML
# (audio_node.yaml, tts_node.yaml, ...) из config_dir. Операторские
# конфиги лежат в /config/voice_assistant (монтируется из
# docker/vision/config/voice_assistant/) — правки применяются при
# рестарте контейнера. Монолитного voice_assistant.yaml больше нет:
# вложенные <node>: секции создавали dotted-имена, которые ноды не читали.
if [ -f /config/voice_assistant/tts_node.yaml ]; then
    echo "Используется конфигурация: /config/voice_assistant/<node>.yaml (per-node, issue #1004)"
    echo "Используется headless launch (без animation_player_node)"
    exec ros2 launch /config/voice_assistant/voice_assistant_headless.launch.py \
        config_dir:=/config/voice_assistant
else
    echo "Используется конфигурация по умолчанию (per-node YAML из образа)"
    exec ros2 launch /config/voice_assistant/voice_assistant_headless.launch.py
fi
