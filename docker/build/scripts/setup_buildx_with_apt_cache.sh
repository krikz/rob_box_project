#!/bin/bash
# Скрипт для настройки Docker buildx с APT кэшем
set -e

echo "🔧 Настройка Docker buildx с APT кэшем..."

# Проверяем что APT cache запущен
if ! curl -f http://localhost:3142/acng-report.html > /dev/null 2>&1; then
    echo "❌ APT cache не запущен. Запустите сначала:"
    echo "docker compose up -d apt-cacher-ng"
    exit 1
fi

# Создаем buildx builder с конфигурацией APT кэша
if docker buildx ls | grep -q rob-box-builder; then
    echo "🗑️ Удаляем существующий builder..."
    docker buildx rm rob-box-builder || true
fi

echo "🔨 Создаем новый buildx builder с APT кэшем..."
docker buildx create \
    --name rob-box-builder \
    --driver docker-container \
    --config ./buildkitd.toml \
    --buildkitd-flags '--allow-insecure-entitlement network.host' \
    --use

echo "📋 Запускаем builder..."
docker buildx inspect --bootstrap

echo "✅ Buildx настроен с APT кэшем!"
echo ""
echo "Теперь сборки будут использовать APT кэш на http://localhost:3142"
echo ""
echo "Для проверки запустите тестовую сборку:"
echo "docker buildx build --build-arg APT_PROXY=http://host.docker.internal:3142 -t test ."