#!/bin/bash
set -e

# Универсальный wrapper для ROS нод с Zenoh namespace
# Генерирует session config с ROBOT_ID и запускает команду
#
# Общий для всех Pi (ADR-0159 В4, PR-2a): лежит в docker/common/scripts,
# compose узла монтирует ../common/scripts:/ros_scripts:ro и
# ../common/config:/common_config:ro. Шаблон session config тоже общий;
# узловое различие — адрес локального роутера ZENOH_LISTEN_IP (дефолт задаёт
# docker-compose.yaml узла), он подставляется в connect.endpoints.
#
# ZENOH_SESSION_TEMPLATE / ZENOH_SESSION_CONFIG_OUT переопределяются только
# для проверки рендера вне контейнера.

echo "=================================================="
echo "  ROS Node с Zenoh namespace"
echo "=================================================="

if [ -z "$ROBOT_ID" ]; then
    echo "⚠️  ВНИМАНИЕ: ROBOT_ID не задан, используется значение по умолчанию 'default'"
    ROBOT_ID="default"
fi

echo "🤖 Robot ID: $ROBOT_ID"
echo "📡 Namespace: robots/$ROBOT_ID"

# Адрес локального роутера: без него нода подключилась бы к чужому Pi или
# никуда — падаем громко, а не молча.
case "$ZENOH_LISTEN_IP" in
    "" | *[!0-9.]*)
        echo "❌ ОШИБКА: ZENOH_LISTEN_IP='$ZENOH_LISTEN_IP' — нужен IPv4 eth0 этого Pi"
        echo "   (дефолт задаёт docker-compose.yaml узла, переопределение — .env)"
        exit 1
        ;;
esac
echo "🔗 Router: tcp/$ZENOH_LISTEN_IP:7447"

SESSION_TEMPLATE="${ZENOH_SESSION_TEMPLATE:-/common_config/zenoh_session_config.json5}"

# Генерируем session config с namespace
GENERATED_CONFIG="${ZENOH_SESSION_CONFIG_OUT:-/tmp/zenoh_session_config.json5}"

# Debug: проверяем существование source файла
if [ ! -f "$SESSION_TEMPLATE" ]; then
    echo "❌ ОШИБКА: $SESSION_TEMPLATE не найден!"
    echo "📁 Содержимое $(dirname "$SESSION_TEMPLATE"):"
    ls -la "$(dirname "$SESSION_TEMPLATE")" || echo "Директория $(dirname "$SESSION_TEMPLATE") не существует"
    exit 1
fi

# Debug: проверяем доступность /tmp для записи
if [ ! -w "$(dirname "$GENERATED_CONFIG")" ]; then
    echo "❌ ОШИБКА: Директория $(dirname "$GENERATED_CONFIG") недоступна для записи!"
    exit 1
fi

# Копируем template конфиг
echo "📋 Копируем $SESSION_TEMPLATE -> $GENERATED_CONFIG"
cp "$SESSION_TEMPLATE" "$GENERATED_CONFIG"

# Проверяем что файл скопирован
if [ ! -f "$GENERATED_CONFIG" ]; then
    echo "❌ ОШИБКА: Не удалось создать $GENERATED_CONFIG"
    exit 1
fi

# Раскомментируем и заменяем namespace
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"

# Адрес локального роутера
sed -i "s|\${ZENOH_LISTEN_IP}|$ZENOH_LISTEN_IP|g" "$GENERATED_CONFIG"

if grep -q '\${' "$GENERATED_CONFIG"; then
    echo "❌ ОШИБКА: в $GENERATED_CONFIG остался неподставленный плейсхолдер:"
    grep -n '\${' "$GENERATED_CONFIG"
    exit 1
fi

echo "✅ Session config сгенерирован: $GENERATED_CONFIG"

# Обновляем ZENOH_SESSION_CONFIG_URI на сгенерированный файл
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"

echo "🚀 Запуск: $@"
echo "=================================================="

# Запускаем переданную команду
exec "$@"
