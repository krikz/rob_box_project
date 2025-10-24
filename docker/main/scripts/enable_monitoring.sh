#!/bin/bash
# Скрипт для включения мониторинга на Main Pi
# Использование: ./enable_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================"
echo "  Включение мониторинга на Main Pi"
echo "================================================"
echo ""

# Проверяем наличие конфигураций
if [ ! -f "config/monitoring/prometheus.yml" ]; then
    echo "❌ Ошибка: файлы конфигурации мониторинга не найдены!"
    echo "   Проверьте наличие config/monitoring/"
    exit 1
fi

echo "✓ Конфигурационные файлы найдены"
echo ""

# Запускаем мониторинг с профилем
echo "Запуск сервисов мониторинга..."
docker-compose --profile monitoring up -d cadvisor prometheus loki promtail grafana

echo ""
echo "================================================"
echo "  Мониторинг успешно запущен!"
echo "================================================"
echo ""
echo "Доступ к сервисам:"
echo "  • Grafana:    http://localhost:3000 (admin/robbox)"
echo "  • Prometheus: http://localhost:9090"
echo "  • cAdvisor:   http://localhost:8080"
echo "  • Loki:       http://localhost:3100"
echo ""
echo "Для остановки: ./disable_monitoring.sh"
echo ""
