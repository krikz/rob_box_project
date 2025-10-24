#!/bin/bash
# Скрипт для выключения агентов мониторинга на Main Pi
# Использование: ./disable_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Выключение агентов мониторинга на Main Pi"
echo "================================================"
echo ""

# Останавливаем контейнеры мониторинга
echo "Остановка агентов мониторинга..."
docker-compose stop cadvisor promtail 2>/dev/null || true

echo ""
echo "Удаление контейнеров..."
docker-compose rm -f cadvisor promtail 2>/dev/null || true

echo ""
echo "================================================"
echo "  Агенты мониторинга успешно остановлены!"
echo "================================================"
echo ""
echo "Для повторного включения: ./scripts/enable_monitoring.sh"
echo ""
