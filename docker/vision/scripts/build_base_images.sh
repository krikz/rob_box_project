#!/bin/bash
# Скрипт для сборки базовых Docker образов для Vision Pi
# Собирает образы с предустановленными зависимостями для ускорения разработки

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$SCRIPT_DIR/../.."

echo "=== Сборка базовых образов для Vision Pi ==="
echo "Директория: $BASE_DIR"

# Сборка базового образа ROS 2 + Zenoh (общая база)
echo ""
echo "1/3: Сборка rob_box_base:ros2-zenoh..."
docker build -f "$BASE_DIR/base/Dockerfile.ros2-zenoh" -t rob_box_base:ros2-zenoh "$BASE_DIR/base"

# Сборка базового образа DepthAI (для OAK-D камеры)
echo ""
echo "2/3: Сборка rob_box_base:depthai..."
docker build -f "$BASE_DIR/base/Dockerfile.depthai" -t rob_box_base:depthai "$BASE_DIR/base"

# Сборка базового образа PCL (для лидаров)
echo ""
echo "3/3: Сборка rob_box_base:pcl..."
docker build -f "$BASE_DIR/base/Dockerfile.pcl" \
  --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh \
  -t rob_box_base:pcl "$BASE_DIR/base"

echo ""
echo "=== Сборка базовых образов завершена! ==="
echo ""
echo "Доступные образы:"
docker images | grep "rob_box_base"

echo ""
echo "Теперь можно собирать сервисы:"
echo "  cd ~/rob_box_project/docker/vision"
echo "  docker compose build && docker compose up -d"

