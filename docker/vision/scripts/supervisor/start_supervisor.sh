#!/bin/bash
# Startup script для Avatar Supervisor контейнера (AV-9, ADR-0028)
#
# Phase 1: запускает supervisor_node в monitor-режиме. Нода публикует
# /avatar/state (latched, transient_local, 1 Гц) и НЕ трогает twist_mux /
# dialogue_node (ADR-0028 §4.5). Эволюция до mode=active — отдельная
# задача, требующая закрытия расхождения #1 из плана Quest (voice_input_mode).
#
# Источники истины:
#   - docs/adr/0028-avatar-supervisor.md §4.6
#   - docker/vision/voice_assistant/start_voice_assistant.sh (паттерн)

set -e

echo "=========================================="
echo "  Avatar Supervisor Starting (Phase 1 monitor)"
echo "  ADR-0028 §4.6"
echo "=========================================="

# Source ROS2 + workspace
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ws/install/setup.bash

# Ждём Zenoh router (имя сервиса zenoh-router, container_name zenoh-router-vision)
# supervisor НЕ публикует топики до того, как Zenoh доступен — иначе
# /avatar/state теряется.
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
    echo "⚠ Zenoh router недоступен после $MAX_RETRIES попыток"
    echo "  Supervisor запустится, но /avatar/state будет публиковаться только"
    echo "  когда Zenoh поднимется (Zenoh session переподключится автоматически)."
fi

# AVATAR_SUPERVISOR_MODE приходит из compose environment (default=monitor).
# Mode=active — это Phase 2 и требует, чтобы dialogue_node принял
# voice_input_mode (расхождение #1 плана Quest).
echo ""
echo "=========================================="
echo "  Запуск supervisor_node (mode=${AVATAR_SUPERVISOR_MODE:-monitor})"
echo "=========================================="

# ros2 run или прямой бинарь — entry point зарегистрирован через
# setup.py: supervisor_node = rob_box_supervisor.supervisor_node:main
exec ros2 run rob_box_supervisor supervisor_node \
    --ros-args \
    -p mode:=${AVATAR_SUPERVISOR_MODE:-monitor}
