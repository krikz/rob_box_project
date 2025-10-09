#!/bin/bash
# Скрипт для сборки базовых Docker образов для Main Pi
# Собирает образы с предустановленными зависимостями для ускорения разработки

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$SCRIPT_DIR/../.."

echo "=== Сборка базовых образов для Main Pi ==="
echo "Директория: $BASE_DIR"

# Сборка базового образа ROS 2 + Zenoh (общая база)
echo ""
echo "1/2: Сборка rob_box_base:ros2-zenoh..."
docker build -f "$BASE_DIR/base/Dockerfile.ros2-zenoh" -t rob_box_base:ros2-zenoh "$BASE_DIR/base"

# Сборка базового образа RTAB-Map (специфичный для SLAM)
echo ""
echo "2/2: Сборка rob_box_base:rtabmap..."
docker build -f "$BASE_DIR/base/Dockerfile.rtabmap" -t rob_box_base:rtabmap "$BASE_DIR/base"

echo ""
echo "=== Сборка базовых образов завершена! ==="
echo ""
echo "Доступные образы:"
docker images | grep "rob_box_base"

echo ""
echo "Теперь можно собирать сервисы:"
echo "  cd ~/rob_box_project/docker/main"
echo "  docker compose build && docker compose up -d"

