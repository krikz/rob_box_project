#!/bin/bash
# Скрипт для выключения мониторинга на Main Pi
# Использование: ./disable_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Выключение мониторинга на Main Pi"
echo "================================================"
echo ""

# Останавливаем контейнеры мониторинга
echo "Остановка сервисов мониторинга..."
docker-compose stop cadvisor prometheus loki promtail grafana 2>/dev/null || true

echo ""
echo "Удаление контейнеров..."
docker-compose rm -f cadvisor prometheus loki promtail grafana 2>/dev/null || true

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
echo "  docker volume rm main_prometheus-data main_loki-data main_grafana-data"
echo ""
echo "Для повторного включения: ./scripts/enable_monitoring.sh"
echo ""
