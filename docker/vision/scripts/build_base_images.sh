#!/bin/bash
# Скрипт для локальной сборки базовых Docker образов для Vision Pi
# Собирает образы с предустановленными зависимостями для ускорения разработки
#
# Использование:
#   ./build_base_images.sh         - собрать все базовые образы локально
#   ./build_base_images.sh --tag   - собрать и тегировать для ghcr.io (для CI/CD)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$SCRIPT_DIR/../.."

# Определяем префикс для образов
if [ "$1" == "--tag" ]; then
    IMAGE_PREFIX="ghcr.io/krikz/rob_box_base"
    echo "=== Сборка для публикации в ghcr.io ==="
else
    IMAGE_PREFIX="rob_box_base"
    echo "=== Локальная сборка базовых образов для Vision Pi ==="
fi

echo "Директория: $BASE_DIR"
echo "Префикс образов: $IMAGE_PREFIX"

# Сборка базового образа ROS 2 + Zenoh (общая база)
echo ""
echo "1/3: Сборка ${IMAGE_PREFIX}:ros2-zenoh..."
docker build -f "$BASE_DIR/base/Dockerfile.ros2-zenoh" \
    -t ${IMAGE_PREFIX}:ros2-zenoh \
    "$BASE_DIR/base"

# Сборка базового образа DepthAI (для OAK-D камеры)
echo ""
echo "2/3: Сборка ${IMAGE_PREFIX}:depthai..."
docker build -f "$BASE_DIR/base/Dockerfile.depthai" \
    -t ${IMAGE_PREFIX}:depthai \
    "$BASE_DIR/base"

# Сборка базового образа PCL (для лидаров)
echo ""
echo "3/3: Сборка ${IMAGE_PREFIX}:pcl..."
docker build -f "$BASE_DIR/base/Dockerfile.pcl" \
    --build-arg BASE_IMAGE=${IMAGE_PREFIX}:ros2-zenoh \
    -t ${IMAGE_PREFIX}:pcl \
    "$BASE_DIR/base"

echo ""
echo "=== Сборка базовых образов завершена! ==="
echo ""
echo "Доступные образы:"
docker images | grep "${IMAGE_PREFIX##*/}"

echo ""
echo "Теперь можно собирать сервисы с локальными образами:"
echo "  cd ~/rob_box_project/docker/vision"
echo "  docker compose build --build-arg BASE_IMAGE=${IMAGE_PREFIX}:depthai oak-d"
echo "  docker compose build --build-arg BASE_IMAGE=${IMAGE_PREFIX}:pcl lslidar"
echo ""
echo "Или использовать предсобранные образы из ghcr.io (по умолчанию в Dockerfile)"


