#!/bin/bash
# Тестовая сборка базового образа с APT кэшем
set -e

echo "🧪 Тестовая сборка базового образа с APT кэшем..."

# Проверяем что APT cache запущен
if ! curl -f http://localhost:3142/acng-report.html > /dev/null 2>&1; then
    echo "❌ APT cache не запущен. Запускаем..."
    cd /home/ros2/rob_box_project/docker/build
    docker compose up -d apt-cacher-ng
    sleep 5
fi

# IP адрес для Docker контейнеров (host.docker.internal не всегда работает в Linux)
APT_PROXY_URL="http://10.1.1.5:3142"

echo "🔧 Используем APT прокси: $APT_PROXY_URL"

# Тестовая сборка базового образа ros2-zenoh
cd /home/ros2/rob_box_project
docker buildx build \
    --build-arg APT_PROXY="$APT_PROXY_URL" \
    --platform linux/arm64 \
    --file docker/base/Dockerfile.ros2-zenoh \
    --tag localhost:5000/rob_box_base:ros2-zenoh-test \
    --push \
    docker/base/

echo "✅ Тестовая сборка завершена!"
echo ""
echo "Проверим кэш APT:"
echo "http://10.1.1.5:3142/acng-report.html"