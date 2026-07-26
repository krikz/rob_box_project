#!/bin/bash
# Быстрый тест сборки проблемных образов (только AMD64 для скорости)

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_DISTRO="kilted"
PLATFORM="linux/amd64"  # Быстрее для теста

echo -e "${BLUE}╔═══════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Quick Docker Build Test (AMD64 only)    ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════╝${NC}"
echo ""

# Проверяем buildx
if ! docker buildx version &>/dev/null; then
    echo -e "${RED}❌ Docker buildx not available${NC}"
    exit 1
fi

# Функция быстрой проверки
quick_build() {
    local name=$1
    local dockerfile=$2
    
    echo -e "${YELLOW}Testing: ${name}${NC}"
    
    if docker buildx build \
        --platform "${PLATFORM}" \
        --file "${dockerfile}" \
        --build-arg ROS_DISTRO="${ROS_DISTRO}" \
        --build-arg IMAGE_TAG=latest \
        --build-arg BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-kilted-latest \
        --load \
        --tag "test:${name}" \
        --progress=plain \
        . 2>&1 | tail -50; then
        
        echo -e "${GREEN}✅ ${name} - OK${NC}\n"
        return 0
    else
        echo -e "${RED}❌ ${name} - FAILED${NC}\n"
        return 1
    fi
}

# Тестируем только исправленные образы
echo -e "${BLUE}1/4 Testing micro_ros_agent (COPY path fix)${NC}"
quick_build "micro_ros_agent" "docker/main/micro_ros_agent/Dockerfile"

echo -e "${BLUE}2/4 Testing vesc_nexus (rosidl + rosdep fix)${NC}"
quick_build "vesc_nexus" "docker/main/vesc_nexus/Dockerfile"

echo -e "${BLUE}3/4 Testing nav2 (removed nav2-recoveries)${NC}"
quick_build "nav2" "docker/main/nav2/Dockerfile"

echo -e "${BLUE}4/4 Testing led_matrix (pip upgrade fix)${NC}"
quick_build "led_matrix" "docker/vision/led_matrix/Dockerfile"

echo -e "${GREEN}╔═══════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     All quick tests passed! 🎉            ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════╝${NC}"
