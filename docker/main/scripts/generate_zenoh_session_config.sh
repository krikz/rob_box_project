#!/bin/bash
set -e

# Генерация zenoh_session_config.json5 с namespace для ROS нод
# Использует ROBOT_ID из переменной окружения

echo "=================================================="
echo "  Генерация Zenoh Session Config с namespace"
echo "=================================================="

if [ -z "$ROBOT_ID" ]; then
    echo "❌ ОШИБКА: ROBOT_ID не задан!"
    exit 1
fi

echo "🤖 Robot ID: $ROBOT_ID"
echo "📡 Namespace: robots/$ROBOT_ID"

# Копируем базовый конфиг
cp /config/shared/zenoh_session_config.json5 /tmp/zenoh_session_config.json5

# Раскомментируем и заменяем namespace
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" /tmp/zenoh_session_config.json5

echo "✅ Session config сгенерирован с namespace: robots/$ROBOT_ID"
echo "📁 Конфиг сохранён в /tmp/zenoh_session_config.json5"
