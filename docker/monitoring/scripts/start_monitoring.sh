#!/bin/bash
# Скрипт для запуска мониторинга на отдельной машине
# Использование: ./start_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "================================================"
echo "  Запуск стека мониторинга Rob Box"
echo "================================================"
echo ""

# Проверяем наличие .env файла
if [ ! -f ".env" ]; then
    echo "⚠️  Файл .env не найден. Создаём из .env.example..."
    cp .env.example .env
    echo ""
    echo "📝 Пожалуйста, отредактируйте .env файл:"
    echo "   - Укажите IP адреса Raspberry Pi (MAIN_PI_IP, VISION_PI_IP)"
    echo "   - Измените пароль Grafana (GRAFANA_PASSWORD)"
    echo ""
    echo "Затем запустите скрипт снова."
    exit 1
fi

# Загружаем переменные окружения
source .env

echo "Конфигурация:"
echo "  Main Pi:    ${MAIN_PI_IP}"
echo "  Vision Pi:  ${VISION_PI_IP}"
echo "  Robot ID:   ${ROBOT_ID:-rob_box_01}"
echo ""

# Запускаем стек мониторинга
echo "Запуск контейнеров мониторинга..."
docker-compose up -d

echo ""
echo "================================================"
echo "  Мониторинг успешно запущен!"
echo "================================================"
echo ""
echo "Доступ к сервисам:"
echo "  • Grafana:    http://localhost:3000 (admin/${GRAFANA_PASSWORD})"
echo "  • Prometheus: http://localhost:9090"
echo "  • Loki:       http://localhost:3100"
echo ""
echo "Для остановки: ./scripts/stop_monitoring.sh"
echo ""
echo "Проверка статуса:"
docker-compose ps
echo ""
