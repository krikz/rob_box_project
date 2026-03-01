#!/bin/bash
# Локальное тестирование сборки Docker образов
# Имитирует GitHub Actions workflow

set -e  # Выход при ошибке

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Настройки
ROS_DISTRO="humble"
BUILD_PLATFORM="linux/arm64"  # Или linux/amd64 для быстрого теста
BASE_IMAGE_TAG="ros2-zenoh-${ROS_DISTRO}-latest"

# Лог файл
LOG_DIR="${PROJECT_ROOT}/local_build_logs"
mkdir -p "${LOG_DIR}"
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
BUILD_LOG="${LOG_DIR}/build-${TIMESTAMP}.log"

echo "=== Local Docker Build Test ===" | tee -a "${BUILD_LOG}"
echo "Time: $(date)" | tee -a "${BUILD_LOG}"
echo "Platform: ${BUILD_PLATFORM}" | tee -a "${BUILD_LOG}"
echo "ROS Distro: ${ROS_DISTRO}" | tee -a "${BUILD_LOG}"
echo "Log: ${BUILD_LOG}" | tee -a "${BUILD_LOG}"
echo "" | tee -a "${BUILD_LOG}"

# Функция для сборки образа
build_image() {
    local name=$1
    local dockerfile=$2
    local context=$3
    local build_args=$4
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Building: ${name}${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    local start_time=$(date +%s)
    
    # Сборка
    if docker buildx build \
        --platform "${BUILD_PLATFORM}" \
        --file "${dockerfile}" \
        --tag "rob_box_test:${name}" \
        ${build_args} \
        --progress=plain \
        "${context}" 2>&1 | tee -a "${BUILD_LOG}"; then
        
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        echo -e "${GREEN}✅ SUCCESS: ${name} (${duration}s)${NC}" | tee -a "${BUILD_LOG}"
        echo "${name}: SUCCESS" >> "${LOG_DIR}/summary-${TIMESTAMP}.txt"
        return 0
    else
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        echo -e "${RED}❌ FAILED: ${name} (${duration}s)${NC}" | tee -a "${BUILD_LOG}"
        echo "${name}: FAILED" >> "${LOG_DIR}/summary-${TIMESTAMP}.txt"
        
        # Сохраняем ошибки отдельно
        tail -100 "${BUILD_LOG}" > "${LOG_DIR}/error-${name}-${TIMESTAMP}.log"
        
        return 1
    fi
}

# Массив для отслеживания результатов
declare -a RESULTS

echo -e "${YELLOW}Starting local build tests...${NC}\n"

# Проверяем наличие базового образа
echo -e "${BLUE}Checking base image...${NC}"
if ! docker image inspect "ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG}" &>/dev/null; then
    echo -e "${YELLOW}⚠️  Base image not found locally. Options:${NC}"
    echo "   1. Pull from registry: docker pull ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG}"
    echo "   2. Build locally: docker buildx build -f docker/base/Dockerfile.ros2-zenoh -t ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG} docker/base"
    echo ""
    read -p "Do you want to build base image now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}Building base image...${NC}"
        build_image "base-ros2-zenoh" "docker/base/Dockerfile.ros2-zenoh" "docker/base" \
            "--build-arg ROS_DISTRO=${ROS_DISTRO} --tag ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG}"
    else
        echo -e "${YELLOW}Skipping base image build. Some images may fail.${NC}"
    fi
fi

echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}Testing Main Pi Services${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 1. micro_ros_agent
if build_image "micro_ros_agent" \
    "docker/main/micro_ros_agent/Dockerfile" \
    "." \
    "--build-arg ROS_DISTRO=${ROS_DISTRO} --build-arg IMAGE_TAG=latest"; then
    RESULTS+=("✅ micro_ros_agent")
else
    RESULTS+=("❌ micro_ros_agent")
fi
echo ""

# 2. vesc_nexus
if build_image "vesc_nexus" \
    "docker/main/vesc_nexus/Dockerfile" \
    "." \
    "--build-arg ROS_DISTRO=${ROS_DISTRO} --build-arg IMAGE_TAG=latest"; then
    RESULTS+=("✅ vesc_nexus")
else
    RESULTS+=("❌ vesc_nexus")
fi
echo ""

# 3. nav2
if build_image "nav2" \
    "docker/main/nav2/Dockerfile" \
    "." \
    "--build-arg BASE_IMAGE=ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG}"; then
    RESULTS+=("✅ nav2")
else
    RESULTS+=("❌ nav2")
fi
echo ""

# 4. robot_state_publisher
if build_image "robot_state_publisher" \
    "docker/main/robot_state_publisher/Dockerfile" \
    "." \
    "--build-arg BASE_IMAGE=ghcr.io/krikz/rob_box_base:${BASE_IMAGE_TAG}"; then
    RESULTS+=("✅ robot_state_publisher")
else
    RESULTS+=("❌ robot_state_publisher")
fi
echo ""

echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}Testing Vision Pi Services${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 5. led_matrix
if build_image "led_matrix" \
    "docker/vision/led_matrix/Dockerfile" \
    "." \
    "--build-arg ROS_DISTRO=${ROS_DISTRO} --build-arg IMAGE_TAG=latest"; then
    RESULTS+=("✅ led_matrix")
else
    RESULTS+=("❌ led_matrix")
fi
echo ""

# Итоговый отчет
echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}Build Results Summary${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

SUCCESS_COUNT=0
FAIL_COUNT=0

for result in "${RESULTS[@]}"; do
    echo "  ${result}"
    if [[ $result == ✅* ]]; then
        ((SUCCESS_COUNT++))
    else
        ((FAIL_COUNT++))
    fi
done

echo ""
echo "Total: ${SUCCESS_COUNT} succeeded, ${FAIL_COUNT} failed"
echo "Logs: ${LOG_DIR}/"
echo "Full log: ${BUILD_LOG}"

if [ -f "${LOG_DIR}/summary-${TIMESTAMP}.txt" ]; then
    echo ""
    echo "Summary file: ${LOG_DIR}/summary-${TIMESTAMP}.txt"
fi

if [ ${FAIL_COUNT} -gt 0 ]; then
    echo ""
    echo -e "${RED}❌ Some builds failed. Check logs for details.${NC}"
    exit 1
else
    echo ""
    echo -e "${GREEN}✅ All builds succeeded!${NC}"
    exit 0
fi
