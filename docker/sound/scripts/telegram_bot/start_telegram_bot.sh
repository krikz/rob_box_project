#!/bin/bash
# Startup script для Telegram Bot контейнера
set -e

echo "=========================================="
echo "  Telegram Bot Starting"
echo "=========================================="

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash

# Проверка токена
if [ -z "${TELEGRAM_BOT_TOKEN}" ]; then
    echo "❌ TELEGRAM_BOT_TOKEN не задан! Бот не может запуститься."
    exit 1
fi

echo "✓ TELEGRAM_BOT_TOKEN задан"

# Проверка whitelist
if [ -z "${TELEGRAM_ALLOWED_USERS}" ]; then
    echo "⚠ TELEGRAM_ALLOWED_USERS пустой — бот будет отклонять всех кроме /myid"
else
    echo "✓ TELEGRAM_ALLOWED_USERS: ${TELEGRAM_ALLOWED_USERS}"
fi

# Проверка API ключей для STT
if [ -n "${YANDEX_API_KEY}" ]; then
    echo "✓ YANDEX_API_KEY задан (для голосовых сообщений)"
else
    echo "⚠ YANDEX_API_KEY не задан — голосовые сообщения могут не работать"
fi

# Проверка LLM API
if [ -n "${DEEPSEEK_API_KEY}" ]; then
    echo "✓ DEEPSEEK_API_KEY задан (LLM chat)"
elif [ -n "${MIMO_API_KEY}" ]; then
    echo "✓ MIMO_API_KEY задан (LLM chat fallback)"
else
    echo "⚠ LLM API ключ не задан — чат будет недоступен"
fi

echo "=========================================="
echo "  Запуск Telegram бота..."
echo "=========================================="

# Запуск ROS 2 ноды
exec ros2 run rob_box_telegram telegram_node \
    --ros-args \
    --params-file /config/telegram_bot/telegram_bot.yaml \
    -r __ns:=/ \
    "$@"
