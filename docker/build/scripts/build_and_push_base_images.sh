#!/bin/bash

# Сборка и push всех базовых образов в локальный registry
# Используется для GitHub Actions runners, которым нужны базовые образы

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"
BASE_DIR="$PROJECT_ROOT/docker/base"

LOCAL_REGISTRY="192.168.1.125:5000"
APT_PROXY="http://192.168.1.125:3142"
PLATFORM="linux/arm64"

echo "🏗️  Сборка базовых образов для локального registry..."
echo "   Registry: $LOCAL_REGISTRY"
echo "   APT Proxy: $APT_PROXY"
echo "   Platform: $PLATFORM"
echo ""

# Функция для сборки и push образа
build_and_push() {
    local dockerfile=$1
    local image_name=$2
    local build_args=$3
    
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🔨 Building: $image_name"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    docker buildx build \
        --platform "$PLATFORM" \
        --file "$BASE_DIR/$dockerfile" \
        --tag "$LOCAL_REGISTRY/krikz/rob_box_base:$image_name" \
        --build-arg "APT_PROXY=$APT_PROXY" \
        $build_args \
        --push \
        "$BASE_DIR"
    
    echo "✅ Successfully built and pushed: $image_name"
    echo ""
}

# Сборка в правильном порядке (с учетом зависимостей)

# 1. ros2-zenoh (базовый образ для всех остальных)
build_and_push "Dockerfile.ros2-zenoh" "ros2-zenoh"

# 2. rtabmap (зависит от ros2-zenoh)
build_and_push "Dockerfile.rtabmap" "rtabmap"

# 3. depthai (независимый, использует luxonis/depthai-ros)
build_and_push "Dockerfile.depthai" "depthai"

# 4. pcl (зависит от ros2-zenoh)
build_and_push "Dockerfile.pcl" "pcl" "--build-arg BASE_IMAGE=$LOCAL_REGISTRY/krikz/rob_box_base:ros2-zenoh"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 Все базовые образы собраны и запушены в локальный registry!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 Список образов в registry:"
curl -s http://$LOCAL_REGISTRY/v2/krikz/rob_box_base/tags/list | jq
