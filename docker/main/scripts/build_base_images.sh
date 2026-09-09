#!/bin/bash
# Скрипт для локальной сборки базовых Docker образов для Main Pi
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
    echo "=== Локальная сборка базовых образов для Main Pi ==="
fi

echo "Директория: $BASE_DIR"
echo "Префикс образов: $IMAGE_PREFIX"

# Сборка базового образа ROS 2 + Zenoh (общая база)
echo ""
echo "1/2: Сборка ${IMAGE_PREFIX}:ros2-zenoh..."
docker build -f "$BASE_DIR/base/Dockerfile.ros2-zenoh" \
    -t ${IMAGE_PREFIX}:ros2-zenoh \
    "$BASE_DIR/base"

# Сборка базового образа RTAB-Map (специфичный для SLAM)
echo ""
echo "2/2: Сборка ${IMAGE_PREFIX}:rtabmap..."
docker build -f "$BASE_DIR/base/Dockerfile.rtabmap" \
    -t ${IMAGE_PREFIX}:rtabmap \
    "$BASE_DIR/base"

echo ""
echo "=== Сборка базовых образов завершена! ==="
echo ""
echo "Доступные образы:"
docker images | grep "${IMAGE_PREFIX##*/}"

echo ""
echo "Теперь можно собирать сервисы с локальными образами:"
echo "  cd ~/rob_box_project/docker/main"
echo "  docker compose build --build-arg BASE_IMAGE=${IMAGE_PREFIX}:ros2-zenoh robot-state-publisher"
echo "  docker compose build --build-arg BASE_IMAGE=${IMAGE_PREFIX}:rtabmap rtabmap"
echo ""
echo "Или использовать предсобранные образы из ghcr.io (по умолчанию в Dockerfile)"


