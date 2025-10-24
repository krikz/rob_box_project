#!/bin/bash
# Скрипт для включения агентов мониторинга на Main Pi
# Отправляют метрики и логи на отдельную машину мониторинга
# Использование: ./enable_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Включение агентов мониторинга на Main Pi"
echo "================================================"
echo ""

# Проверяем наличие конфигураций
if [ ! -f "config/monitoring/promtail-config.yaml" ]; then
    echo "❌ Ошибка: файлы конфигурации мониторинга не найдены!"
    echo "   Проверьте наличие config/monitoring/"
    exit 1
fi

# Проверяем наличие переменной LOKI_HOST
if [ -z "${LOKI_HOST}" ]; then
    echo "⚠️  Внимание: переменная LOKI_HOST не установлена"
    echo "   Используется значение по умолчанию: monitoring-machine"
    echo ""
    echo "   Для настройки добавьте в .env файл:"
    echo "   LOKI_HOST=<IP адрес машины мониторинга>"
    echo ""
fi

echo "✓ Конфигурационные файлы найдены"
echo ""

# Запускаем агенты мониторинга с профилем
echo "Запуск агентов мониторинга..."
docker-compose --profile monitoring up -d cadvisor promtail

echo ""
echo "================================================"
echo "  Агенты мониторинга успешно запущены!"
echo "================================================"
echo ""
echo "Локальный доступ:"
echo "  • cAdvisor:   http://localhost:8080"
echo ""
echo "Данные отправляются на машину мониторинга (${LOKI_HOST:-monitoring-machine})"
echo "Для просмотра логов и метрик откройте Grafana на машине мониторинга"
echo ""
echo "Для остановки: ./scripts/disable_monitoring.sh"
echo ""
