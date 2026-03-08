#!/bin/bash
# Запуск Zenoh local router для подключения к роботу
# Использование: ./local_test/start_zenoh.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROUTER_CONFIG="$SCRIPT_DIR/zenoh_local_router.json5"

echo "=== Zenoh Local Router ==="

# Убиваем старый роутер если есть
if pgrep -fa zenohd > /dev/null 2>&1; then
    echo "Остановка старого роутера..."
    pkill -f zenohd 2>/dev/null || true
    sleep 1
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Стартуем роутер через ZENOH_ROUTER_CONFIG_URI (не --config !)
echo "Запуск роутера с $ROUTER_CONFIG..."
ZENOH_ROUTER_CONFIG_URI="$ROUTER_CONFIG" \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
ROUTER_PID=$!
echo "Роутер PID=$ROUTER_PID"

sleep 2

# Проверяем что запustился
if ! kill -0 $ROUTER_PID 2>/dev/null; then
    echo "❌ Роутер упал! Лог:"
    cat /tmp/zenoh_router.log
    exit 1
fi

# Проверяем порт
if nc -z -w2 10.1.1.249 7447 2>/dev/null; then
    echo "✅ Порт 7447 открыт"
else
    echo "⚠️  Порт не слушает ещё, подождём..."
    sleep 2
fi

# Стопим ros2 daemon чтобы сбросить кеш дискавери
echo "Сброс ros2 daemon..."
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$SCRIPT_DIR/zenoh_local_session.json5"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 daemon stop 2>/dev/null || true
sleep 2

# Проверяем топики
echo "Проверка топиков..."
TOPICS=$(ros2 topic list 2>/dev/null | grep -v parameter_events | grep -v rosout | wc -l)
echo "Видно топиков: $TOPICS"

if [ "$TOPICS" -gt 5 ]; then
    echo "✅ Подключение OK! Топики:"
    ros2 topic list 2>/dev/null | grep -E "scan|rtabmap|odom|tf|map" | head -10
else
    echo "⚠️  Топиков мало. Лог роутера:"
    cat /tmp/zenoh_router.log | tail -20
fi

echo ""
echo "=== Для работы в новом терминале выполни ==="
echo "source /opt/ros/humble/setup.bash"
echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp"
echo "export ZENOH_SESSION_CONFIG_URI=$SCRIPT_DIR/zenoh_local_session.json5"
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
