#!/bin/bash

# Скрипт автоматического обновления и перезапуска Vision Pi (OAK-D камера)
# Использование: ./update_and_restart.sh

set -e  # Останавливаем при ошибке

echo "=========================================="
echo "Vision Pi - Обновление и перезапуск"
echo "=========================================="
echo ""

# Переходим в корень проекта
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"

echo "📂 Переход в корень проекта: $PROJECT_ROOT"
cd "$PROJECT_ROOT"
echo ""

# Останавливаем контейнеры с удалением сирот
echo "🛑 Останавливаем Docker контейнеры..."
cd docker/vision
docker-compose down --remove-orphans
echo "✅ Контейнеры остановлены"
echo ""

# Обновляем код из GitHub
echo "📥 Получаем обновления из GitHub..."
cd "$PROJECT_ROOT"
git pull origin main
echo "✅ Код обновлен"
echo ""

# Запускаем контейнеры
echo "🚀 Запускаем Docker контейнеры..."
cd docker/vision
docker-compose up -d
echo "✅ Контейнеры запущены"
echo ""

# Показываем статус
echo "📊 Статус контейнеров:"
docker-compose ps
echo ""

# Ждем немного для инициализации
echo "⏳ Ждем 5 секунд для инициализации..."
sleep 5

# Показываем логи
echo ""
echo "📝 Последние логи OAK-D камеры:"
echo "=========================================="
docker logs --tail 30 oak-d
echo "=========================================="
echo ""

echo "✅ Обновление завершено!"
echo ""
echo "💡 Полезные команды:"
echo "   docker logs -f oak-d          - следить за логами"
echo "   docker-compose ps              - статус контейнеров"
echo "   docker-compose restart         - перезапустить"
echo "   docker-compose down            - остановить"
echo ""
