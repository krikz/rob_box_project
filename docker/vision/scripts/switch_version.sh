#!/bin/bash

# Скрипт для переключения версии ROS 2 и тега образов
# Использование: ./switch_version.sh [humble|jazzy|kilted] [latest|dev|rc-X.Y.Z]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Использование: $0 <ros_distro> [image_tag]"
    echo ""
    echo "Параметры:"
    echo "  ros_distro  - Версия ROS 2: humble, jazzy, kilted"
    echo "  image_tag   - Тег образа (опционально): latest, dev, rc-X.Y.Z, test"
    echo ""
    echo "Примеры:"
    echo "  $0 humble latest       # Production на Humble"
    echo "  $0 jazzy dev           # Development на Jazzy"
    echo "  $0 humble rc-1.0.0     # Release candidate 1.0.0 на Humble"
    echo "  $0 kilted              # Production на Kilted (IMAGE_TAG=latest)"
}

if [ $# -lt 1 ]; then
    print_usage
    exit 1
fi

ROS_DISTRO=$1
IMAGE_TAG=${2:-latest}

# Валидация ROS_DISTRO
case "$ROS_DISTRO" in
    humble|jazzy|kilted)
        ;;
    *)
        echo -e "${RED}Ошибка: Неподдерживаемая версия ROS 2: $ROS_DISTRO${NC}"
        echo "Поддерживаемые версии: humble, jazzy, kilted"
        exit 1
        ;;
esac

echo -e "${GREEN}=== Переключение конфигурации ===${NC}"
echo -e "ROS Distro: ${YELLOW}$ROS_DISTRO${NC}"
echo -e "Image Tag:  ${YELLOW}$IMAGE_TAG${NC}"
echo ""

# Обновляем .env файл
cat > "$ENV_FILE" << EOF
# Конфигурация версий Docker образов
# Автоматически сгенерировано $(date)

ROS_DISTRO=$ROS_DISTRO
IMAGE_TAG=$IMAGE_TAG
REGISTRY=ghcr.io
REPOSITORY_OWNER=krikz
BASE_IMAGE_PREFIX=\${REGISTRY}/\${REPOSITORY_OWNER}/rob_box_base
SERVICE_IMAGE_PREFIX=\${REGISTRY}/\${REPOSITORY_OWNER}/rob_box
EOF

echo -e "${GREEN}✓ Файл .env обновлён${NC}"
echo ""

# Показываем какие образы будут использоваться
echo -e "${GREEN}=== Образы которые будут использоваться ===${NC}"
source "$ENV_FILE"

echo "Базовые образы:"
echo "  - ros2-zenoh: ${BASE_IMAGE_PREFIX}:ros2-zenoh-${ROS_DISTRO}-${IMAGE_TAG}"
echo "  - rtabmap:    ${BASE_IMAGE_PREFIX}:rtabmap-${ROS_DISTRO}-${IMAGE_TAG}"
echo ""
echo "Сервисы:"
echo "  - robot-state-publisher: ${SERVICE_IMAGE_PREFIX}:robot-state-publisher-${ROS_DISTRO}-${IMAGE_TAG}"
echo "  - rtabmap:               ${SERVICE_IMAGE_PREFIX}:rtabmap-${ROS_DISTRO}-${IMAGE_TAG}"
echo ""

# Спрашиваем нужно ли загрузить образы
read -p "Загрузить образы сейчас? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${GREEN}Загружаем образы...${NC}"
    docker-compose pull
    echo -e "${GREEN}✓ Образы загружены${NC}"
fi

echo ""
echo -e "${GREEN}=== Готово! ===${NC}"
echo "Для запуска контейнеров выполните:"
echo "  docker-compose up -d"
echo ""
echo "Для сборки локальных образов выполните:"
echo "  docker-compose build"
