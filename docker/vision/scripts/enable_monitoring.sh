#!/bin/bash
# Скрипт для включения мониторинга на Vision Pi
# Использование: ./enable_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Включение мониторинга на Vision Pi"
echo "================================================"
echo ""

# Проверяем наличие конфигураций
if [ ! -f "config/monitoring/promtail-config.yaml" ]; then
    echo "❌ Ошибка: файлы конфигурации мониторинга не найдены!"
    echo "   Проверьте наличие config/monitoring/"
    exit 1
fi

echo "✓ Конфигурационные файлы найдены"
echo ""

# Запускаем мониторинг с профилем
echo "Запуск сервисов мониторинга..."
docker-compose --profile monitoring up -d cadvisor promtail

echo ""
echo "================================================"
echo "  Мониторинг успешно запущен!"
echo "================================================"
echo ""
echo "Доступ к сервисам:"
echo "  • cAdvisor:   http://localhost:8080"
echo ""
echo "Логи отправляются на Main Pi (10.1.1.10:3100)"
echo "Метрики доступны в Grafana на Main Pi (http://10.1.1.10:3000)"
echo ""
echo "Для остановки: ./scripts/disable_monitoring.sh"
echo ""
