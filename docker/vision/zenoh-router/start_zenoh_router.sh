#!/bin/bash
set -e

echo "=========================================="
echo "  Zenoh Router Vision Pi (без namespace)"
echo "=========================================="

# Vision Pi router НЕ использует namespace, т.к. подключается к Main Pi локально
# Namespace добавляется только на Main Pi router при передаче в облако

echo "📡 Режим: локальный роутер"
echo "🔗 Подключение к: Main Pi Router (10.1.1.10)"
echo ""
echo "Запуск Zenoh Router..."
echo "=========================================="

# Запускаем zenoh router с обычным конфигом (без namespace)
exec zenoh -c /config/zenoh_router_config.json5
