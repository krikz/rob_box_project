#!/bin/bash
# Скрипт для локальной сборки Docker образов
# Позволяет ускорить разработку, не дожидаясь GitHub Actions
#
# Использование:
#   ./scripts/local-build.sh [service] [platform]
#
# Примеры:
#   ./scripts/local-build.sh voice-assistant        # Собрать voice-assistant для текущей платформы
#   ./scripts/local-build.sh all                    # Собрать все сервисы
#   ./scripts/local-build.sh oak-d arm64            # Собрать oak-d для Raspberry Pi

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Конфигурация
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$PROJECT_ROOT/docker"
ROS_DISTRO=${ROS_DISTRO:-humble}
IMAGE_TAG=${IMAGE_TAG:-local}
REGISTRY=${REGISTRY:-ghcr.io}
REPOSITORY_OWNER=${REPOSITORY_OWNER:-krikz}
SERVICE_IMAGE_PREFIX="${REGISTRY}/${REPOSITORY_OWNER}/rob_box"

# Определяем текущую ветку
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

# Платформа (по умолчанию текущая, или arm64 для Raspberry Pi)
PLATFORM=${2:-linux/arm64}

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}🔨 Локальная сборка Docker образов РОББОКС${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Ветка: $CURRENT_BRANCH"
echo "Платформа: $PLATFORM"
echo "Тег: $IMAGE_TAG"
echo "ROS дистрибутив: $ROS_DISTRO"
echo ""

# Функция для сборки сервиса
build_service() {
    local service=$1
    local context=$2
    local dockerfile=$3
    local full_tag="${SERVICE_IMAGE_PREFIX}:${service}-${ROS_DISTRO}-${IMAGE_TAG}"
    
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}📦 Сборка сервиса: $service${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo "Контекст: $context"
    echo "Dockerfile: $dockerfile"
    echo "Образ: $full_tag"
    echo ""
    
    # Проверяем существование Dockerfile
    if [ ! -f "$dockerfile" ]; then
        echo -e "${RED}❌ Ошибка: Dockerfile не найден: $dockerfile${NC}"
        return 1
    fi
    
    # Сборка образа
    if docker buildx build \
        --platform "$PLATFORM" \
        --file "$dockerfile" \
        --tag "$full_tag" \
        --load \
        "$context"; then
        echo -e "${GREEN}✅ Успешно собран: $full_tag${NC}"
        echo ""
        return 0
    else
        echo -e "${RED}❌ Ошибка сборки: $service${NC}"
        echo ""
        return 1
    fi
}

# Функция для определения сервисов Vision Pi
build_vision_services() {
    echo -e "${BLUE}🔷 Сборка Vision Pi сервисов${NC}"
    echo ""
    
    local success=0
    local failed=0
    
    # OAK-D Camera
    if build_service "oak-d" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/oak-d/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # LSLIDAR N10
    if build_service "lslidar" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/lslidar/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # AprilTag
    if build_service "apriltag" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/apriltag/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # LED Matrix
    if build_service "led-matrix" "$PROJECT_ROOT" "$DOCKER_DIR/vision/led_matrix/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # Voice Assistant
    if build_service "voice-assistant" "$PROJECT_ROOT" "$DOCKER_DIR/vision/voice_assistant/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # Perception
    if build_service "perception" "$PROJECT_ROOT" "$DOCKER_DIR/vision/perception/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}📊 Результат сборки Vision Pi${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "Успешно: ${GREEN}$success${NC}"
    echo -e "Ошибок: ${RED}$failed${NC}"
    echo ""
    
    return $failed
}

# Функция для определения сервисов Main Pi
build_main_services() {
    echo -e "${BLUE}🔶 Сборка Main Pi сервисов${NC}"
    echo ""
    
    local success=0
    local failed=0
    
    # Robot State Publisher
    if build_service "robot-state-publisher" "$PROJECT_ROOT" "$DOCKER_DIR/main/robot_state_publisher/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # RTAB-Map
    if build_service "rtabmap" "$DOCKER_DIR/main/rtabmap" "$DOCKER_DIR/main/rtabmap/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # Twist Mux
    if build_service "twist-mux" "$DOCKER_DIR/main/twist_mux" "$DOCKER_DIR/main/twist_mux/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # Micro-ROS Agent
    if build_service "micro-ros-agent" "$PROJECT_ROOT" "$DOCKER_DIR/main/micro_ros_agent/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # ROS2 Control
    if build_service "ros2-control" "$PROJECT_ROOT" "$DOCKER_DIR/main/ros2_control/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    # Nav2
    if build_service "nav2" "$DOCKER_DIR/main/nav2" "$DOCKER_DIR/main/nav2/Dockerfile"; then
        ((success++))
    else
        ((failed++))
    fi
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}📊 Результат сборки Main Pi${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "Успешно: ${GREEN}$success${NC}"
    echo -e "Ошибок: ${RED}$failed${NC}"
    echo ""
    
    return $failed
}

# Главная логика
SERVICE=${1:-help}

case "$SERVICE" in
    "all")
        echo "Сборка всех сервисов..."
        vision_failed=0
        main_failed=0
        
        build_vision_services || vision_failed=$?
        build_main_services || main_failed=$?
        
        total_failed=$((vision_failed + main_failed))
        
        if [ $total_failed -eq 0 ]; then
            echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}✅ Все сервисы успешно собраны!${NC}"
            echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            exit 0
        else
            echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${RED}❌ Сборка завершена с ошибками: $total_failed${NC}"
            echo -e "${RED}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            exit 1
        fi
        ;;
    
    "vision")
        build_vision_services
        exit $?
        ;;
    
    "main")
        build_main_services
        exit $?
        ;;
    
    "oak-d")
        build_service "oak-d" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/oak-d/Dockerfile"
        ;;
    
    "lslidar")
        build_service "lslidar" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/lslidar/Dockerfile"
        ;;
    
    "apriltag")
        build_service "apriltag" "$DOCKER_DIR/vision" "$DOCKER_DIR/vision/apriltag/Dockerfile"
        ;;
    
    "led-matrix")
        build_service "led-matrix" "$PROJECT_ROOT" "$DOCKER_DIR/vision/led_matrix/Dockerfile"
        ;;
    
    "voice-assistant")
        build_service "voice-assistant" "$PROJECT_ROOT" "$DOCKER_DIR/vision/voice_assistant/Dockerfile"
        ;;
    
    "perception")
        build_service "perception" "$PROJECT_ROOT" "$DOCKER_DIR/vision/perception/Dockerfile"
        ;;
    
    "robot-state-publisher")
        build_service "robot-state-publisher" "$PROJECT_ROOT" "$DOCKER_DIR/main/robot_state_publisher/Dockerfile"
        ;;
    
    "rtabmap")
        build_service "rtabmap" "$DOCKER_DIR/main/rtabmap" "$DOCKER_DIR/main/rtabmap/Dockerfile"
        ;;
    
    "twist-mux")
        build_service "twist-mux" "$DOCKER_DIR/main/twist_mux" "$DOCKER_DIR/main/twist_mux/Dockerfile"
        ;;
    
    "micro-ros-agent")
        build_service "micro-ros-agent" "$PROJECT_ROOT" "$DOCKER_DIR/main/micro_ros_agent/Dockerfile"
        ;;
    
    "ros2-control")
        build_service "ros2-control" "$PROJECT_ROOT" "$DOCKER_DIR/main/ros2_control/Dockerfile"
        ;;
    
    "nav2")
        build_service "nav2" "$DOCKER_DIR/main/nav2" "$DOCKER_DIR/main/nav2/Dockerfile"
        ;;
    
    "help"|*)
        echo "Использование: $0 [service] [platform]"
        echo ""
        echo "Сервисы Vision Pi:"
        echo "  oak-d             - OAK-D camera driver"
        echo "  lslidar           - LSLIDAR N10 driver"
        echo "  apriltag          - AprilTag detector"
        echo "  led-matrix        - LED matrix driver"
        echo "  voice-assistant   - Voice assistant + animations"
        echo "  perception        - Perception & dialogue system"
        echo ""
        echo "Сервисы Main Pi:"
        echo "  robot-state-publisher  - Robot state publisher"
        echo "  rtabmap                - RTAB-Map SLAM"
        echo "  twist-mux              - Twist multiplexer"
        echo "  micro-ros-agent        - Micro-ROS agent"
        echo "  ros2-control           - ROS2 Control + VESC"
        echo "  nav2                   - Nav2 navigation stack"
        echo ""
        echo "Группы:"
        echo "  all               - Собрать все сервисы"
        echo "  vision            - Собрать все Vision Pi сервисы"
        echo "  main              - Собрать все Main Pi сервисы"
        echo ""
        echo "Платформы:"
        echo "  linux/arm64       - Raspberry Pi (по умолчанию)"
        echo "  linux/amd64       - x86_64 системы"
        echo ""
        echo "Примеры:"
        echo "  $0 voice-assistant"
        echo "  $0 all"
        echo "  $0 oak-d linux/amd64"
        echo ""
        echo "Переменные окружения:"
        echo "  IMAGE_TAG=$IMAGE_TAG"
        echo "  ROS_DISTRO=$ROS_DISTRO"
        echo "  PLATFORM=$PLATFORM"
        exit 0
        ;;
esac
