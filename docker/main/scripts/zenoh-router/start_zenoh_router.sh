#!/bin/bash
set -e

echo "=========================================="
echo "  Zenoh Router с namespace для облака"
echo "=========================================="

# Проверка переменной ROBOT_ID
if [ -z "$ROBOT_ID" ]; then
    echo "❌ ОШИБКА: ROBOT_ID не задан!"
    echo "   Установите переменную ROBOT_ID в .env файле"
    exit 1
fi

echo "🤖 Robot ID: $ROBOT_ID"
echo "📡 Namespace для ROS нод: robots/$ROBOT_ID"
echo "⚠️  Namespace применяется в session config, НЕ в router config!"

# Копируем router конфиг без изменений (без namespace!)
CONFIG_FILE="/tmp/zenoh_router_config.json5"
cp /config/zenoh_router_config.json5 "$CONFIG_FILE"

echo "✅ Router конфиг скопирован БЕЗ namespace (правильно!)"
echo ""
echo "Запуск Zenoh Router..."
echo "=========================================="

# Запускаем zenoh router с модифицированным конфигом
exec /zenohd -c "$CONFIG_FILE"
