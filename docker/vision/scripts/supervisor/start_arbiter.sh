#!/bin/bash
# Startup script для Avatar Arbiter контейнера (ADR-0051 §2.2, issue #1987)
#
# Нода avatar_arbiter — арбитр floor/FSM без LLM: LockManager, ModeManager,
# агрегатор, сервисы /avatar_arbiter/{acquire_floor,release_floor,
# set_avatar_mode} и публикация /avatar/state. Арбитраж вынесен из
# supervisor_node (там остались голос + супервизор-агент оператора).
#
# Источники истины:
#   - docs/adr/0051-operator-agent.md §2.2 (или target-архитектура)
#   - docs/architecture/target-operator-agent-and-dialogue.md §4, §9.7
#   - issue #1987 (operator-agent 04: avatar_arbiter)
#   - docker/vision/scripts/supervisor/start_supervisor.sh (паттерн)

set -e

echo "=========================================="
echo "  Avatar Arbiter Starting (no LLM, #1987)"
echo "  ADR-0051 §2.2"
echo "=========================================="

# Source ROS2 + workspace
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ws/install/setup.bash

# Ждём Zenoh router (имя сервиса zenoh-router, container_name zenoh-router-vision)
# арбитр не публикует /avatar/state до того, как Zenoh доступен — иначе
# late-join snapshot теряется (transient_local /avatar/state).
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
    echo "  Арбитр запустится, но /avatar/state будет публиковаться только"
    echo "  когда Zenoh поднимется (Zenoh session переподключится автоматически)."
fi

# AVATAR_ARBITER_MODE приходит из compose environment (default=active).
# Арбитр по умолчанию работает в active-режиме (это его единственная роль —
# он НЕ монитор-нода, в отличие от старого supervisor-monitor).
echo ""
echo "=========================================="
echo "  Запуск avatar_arbiter (mode=${AVATAR_ARBITER_MODE:-active})"
echo "=========================================="

# entry point зарегистрирован через setup.py:
# avatar_arbiter = rob_box_supervisor.arbiter_node:main
exec ros2 run rob_box_supervisor avatar_arbiter \
    --ros-args \
    -p mode:=${AVATAR_ARBITER_MODE:-active}
