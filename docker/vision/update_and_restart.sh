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
CURRENT_BRANCH=$(git branch --show-current)
echo "   Текущая ветка: $CURRENT_BRANCH"
git pull origin $CURRENT_BRANCH
echo "✅ Код обновлен"
echo ""

# Настраиваем Docker теги на основе текущей ветки
echo "🔧 Настройка Docker тегов..."
cd "$PROJECT_ROOT"
bash scripts/set-docker-tags.sh
echo ""

# Скачиваем новые образы из registry
echo "📦 Скачиваем обновленные Docker образы из registry..."
cd docker/vision
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

# КРИТИЧНО: Отключаем lazy publisher
echo "🔧 Отключаем lazy publisher для принудительной публикации..."
docker exec oak-d /ros_entrypoint.sh ros2 param set /camera/camera color.i_enable_lazy_publisher false 2>/dev/null || echo "  (color уже настроен)"
docker exec oak-d /ros_entrypoint.sh ros2 param set /camera/camera depth.i_enable_lazy_publisher false 2>/dev/null || echo "  (depth уже настроен)"
sleep 2

# Показываем логи
echo ""
echo "📝 Последние логи OAK-D камеры:"
echo "=========================================="
docker logs --tail 30 oak-d
echo "=========================================="
echo ""

echo "📊 Проверка топиков:"
docker exec oak-d /ros_entrypoint.sh ros2 topic info /oak/rgb/image_raw/compressed 2>/dev/null | grep "Publisher count" || echo "Топик еще не готов"
echo ""

echo "✅ Обновление завершено!"
echo ""
echo "💡 Полезные команды:"
echo "   docker logs -f oak-d          - следить за логами"
echo "   docker-compose ps              - статус контейнеров"
echo "   docker-compose restart         - перезапустить"
echo "   docker-compose down            - остановить"
echo ""
