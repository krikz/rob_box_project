#!/bin/bash
# Скрипт для остановки мониторинга на отдельной машине
# Использование: ./stop_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Остановка стека мониторинга Rob Box"
echo "================================================"
echo ""

# Останавливаем контейнеры
echo "Остановка контейнеров..."
docker-compose down

echo ""
echo "================================================"
echo "  Мониторинг успешно остановлен!"
echo "================================================"
echo ""
echo "Данные мониторинга сохранены в volumes:"
echo "  • prometheus-data"
echo "  • loki-data"
echo "  • grafana-data"
echo ""
echo "Для полного удаления данных выполните:"
echo "  docker-compose down -v"
echo ""
echo "Для повторного запуска: ./scripts/start_monitoring.sh"
echo ""
