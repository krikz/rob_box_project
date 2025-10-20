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
echo "📡 Namespace: robots/$ROBOT_ID"

# Генерируем конфиг с namespace
CONFIG_FILE="/tmp/zenoh_router_config.json5"
cp /config/zenoh_router_config.json5 "$CONFIG_FILE"

# Раскомментируем и заменяем namespace в конфиге
# Используем sed для замены закомментированной строки
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$CONFIG_FILE"

echo "✅ Конфигурация сгенерирована с namespace"
echo ""
echo "Запуск Zenoh Router..."
echo "=========================================="

# Запускаем zenoh router с модифицированным конфигом
exec zenohd -c "$CONFIG_FILE"
