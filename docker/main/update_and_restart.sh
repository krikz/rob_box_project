#!/bin/bash

# Скрипт автоматического обновления и перезапуска Main Pi (RTAB-Map)
# Использование: ./update_and_restart.sh

set -e  # Останавливаем при ошибке

echo "=========================================="
echo "Main Pi - Обновление и перезапуск"
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
cd docker/main
docker-compose down --remove-orphans
echo "✅ Контейнеры остановлены"
echo ""

# Обновляем код из GitHub
echo "📥 Получаем обновления из GitHub..."
cd "$PROJECT_ROOT"
CURRENT_BRANCH=$(git branch --show-current)
echo "   Текущая ветка: $CURRENT_BRANCH"
git fetch origin
git reset --hard origin/$CURRENT_BRANCH
echo "✅ Код обновлен"
echo ""

# Настраиваем Docker теги на основе текущей ветки
echo "🔧 Настройка Docker тегов..."
cd "$PROJECT_ROOT"
bash scripts/set-docker-tags.sh
echo ""

# Скачиваем новые образы из registry
echo "📦 Скачиваем обновленные Docker образы из registry..."
cd docker/main
docker-compose pull
echo "✅ Образы обновлены"
echo ""

# Запускаем контейнеры
echo "🚀 Запускаем Docker контейнеры..."
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
echo "📝 Последние логи RTAB-Map:"
echo "=========================================="
docker logs --tail 30 rtabmap
echo "=========================================="
echo ""

echo "✅ Обновление завершено!"
echo ""
echo "💡 Полезные команды:"
echo "   docker logs -f rtabmap         - следить за логами"
echo "   docker-compose ps              - статус контейнеров"
echo "   docker-compose restart         - перезапустить"
echo "   docker-compose down            - остановить"
echo ""
