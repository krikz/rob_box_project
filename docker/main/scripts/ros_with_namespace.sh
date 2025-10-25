#!/bin/bash
set -e

# Универсальный wrapper для ROS нод с Zenoh namespace
# Генерирует session config с ROBOT_ID и запускает команду

echo "=================================================="
echo "  ROS Node с Zenoh namespace"
echo "=================================================="

if [ -z "$ROBOT_ID" ]; then
    echo "⚠️  ВНИМАНИЕ: ROBOT_ID не задан, используется значение по умолчанию 'default'"
    ROBOT_ID="default"
fi

echo "🤖 Robot ID: $ROBOT_ID"
echo "📡 Namespace: robots/$ROBOT_ID"

# Генерируем session config с namespace
GENERATED_CONFIG="/tmp/zenoh_session_config.json5"
cp /config/zenoh_session_config.json5 "$GENERATED_CONFIG"

# Раскомментируем и заменяем namespace (учитываем запятую в конце)
sed -i "s|// namespace: \"my/namespace\",|namespace: \"robots/$ROBOT_ID\",|g" "$GENERATED_CONFIG"

echo "✅ Session config сгенерирован: $GENERATED_CONFIG"

# Обновляем ZENOH_SESSION_CONFIG_URI на сгенерированный файл
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"

echo "🚀 Запуск: $@"
echo "=================================================="

# Запускаем переданную команду
exec "$@"
