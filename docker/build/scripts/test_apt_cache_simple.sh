#!/bin/bash
# Упрощенный тест APT кэша с исправлением проблемы пакетов
set -e

echo "🧪 Упрощенный тест APT кэша..."

# Проверяем что APT cache запущен
if ! curl -f http://localhost:3142/acng-report.html > /dev/null 2>&1; then
    echo "❌ APT cache не запущен. Запускаем..."
    cd /home/ros2/rob_box_project/docker/build
    docker compose up -d apt-cacher-ng
    sleep 5
fi

# IP адрес для Docker контейнеров
APT_PROXY_URL="http://10.1.1.5:3142"

echo "🔧 Используем APT прокси: $APT_PROXY_URL"

# Создаем упрощенный Dockerfile для теста
mkdir -p /home/ros2/rob_box_project/docker/build/test
cat > /home/ros2/rob_box_project/docker/build/test/Dockerfile << 'EOF'
FROM ubuntu:22.04

# Build argument для APT кэша
ARG APT_PROXY=""

# Настройка APT прокси если указан
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
        echo "🔧 Using APT proxy: $APT_PROXY"; \
    fi

# Только основные пакеты которые точно есть
RUN apt-get update && \
    apt-get install -y \
        git \
        curl \
        && rm -rf /var/lib/apt/lists/*

# Удаляем APT прокси конфигурацию
RUN rm -f /etc/apt/apt.conf.d/02proxy

CMD ["echo", "Test successful!"]
EOF

echo "🔨 Запускаем тестовую сборку..."
cd /home/ros2/rob_box_project/docker/build
docker buildx build \
    --build-arg APT_PROXY="$APT_PROXY_URL" \
    --platform linux/amd64 \
    --file test/Dockerfile \
    --tag apt-cache-test:latest \
    --load \
    test/

echo "✅ Тестовая сборка завершена!"
echo ""
echo "📊 Проверяем статистику APT кэша:"
echo "http://10.1.1.5:3142/acng-report.html"

# Проверяем размер кэша
echo ""
echo "📁 Размер кэша APT:"
sudo du -sh /home/ros2/rob_box_project/docker/build/data/apt-cache/