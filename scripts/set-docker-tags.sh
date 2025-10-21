#!/bin/bash
# Скрипт для автоматического определения Docker тегов на основе текущей ветки
# Используется для деплоя правильных образов в зависимости от ветки разработки
#
# Использование:
#   source scripts/set-docker-tags.sh
#   docker-compose pull && docker-compose up -d

set -e

# Определяем текущую ветку
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

# Базовая директория проекта
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$PROJECT_ROOT/docker"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 Настройка Docker тегов для РОББОКС"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Текущая ветка: $CURRENT_BRANCH"

# Определяем какой .env файл использовать
if [ "$CURRENT_BRANCH" = "main" ]; then
    ENV_FILE="$DOCKER_DIR/.env.main"
    IMAGE_TAG="latest"
    DESCRIPTION="Production (main branch)"
elif [ "$CURRENT_BRANCH" = "develop" ]; then
    ENV_FILE="$DOCKER_DIR/.env.develop"
    IMAGE_TAG="dev"
    DESCRIPTION="Development (develop branch)"
elif [[ "$CURRENT_BRANCH" == feature/* ]] || [[ "$CURRENT_BRANCH" == feat/* ]]; then
    ENV_FILE="$DOCKER_DIR/.env.feature"
    IMAGE_TAG="test"
    DESCRIPTION="Feature testing (feature/* branch)"
elif [[ "$CURRENT_BRANCH" == release/* ]]; then
    # Release branches use release candidate tags
    VERSION="${CURRENT_BRANCH#release/}"
    IMAGE_TAG="rc-${VERSION}"
    ENV_FILE="$DOCKER_DIR/.env.develop"  # Use develop as base
    DESCRIPTION="Release candidate (release/$VERSION)"
elif [[ "$CURRENT_BRANCH" == hotfix/* ]]; then
    # Hotfix branches use hotfix tags
    VERSION="${CURRENT_BRANCH#hotfix/}"
    IMAGE_TAG="hotfix-${VERSION}"
    ENV_FILE="$DOCKER_DIR/.env.main"  # Use main as base
    DESCRIPTION="Hotfix (hotfix/$VERSION)"
else
    echo "⚠️  Неизвестная ветка: $CURRENT_BRANCH"
    echo "    Используется конфигурация по умолчанию (test)"
    ENV_FILE="$DOCKER_DIR/.env.feature"
    IMAGE_TAG="test"
    DESCRIPTION="Unknown branch (using test tags)"
fi

echo "Конфигурация: $DESCRIPTION"
echo "Env файл: $(basename $ENV_FILE)"
echo "Image tag: $IMAGE_TAG"
echo ""

# Копируем нужный .env файл в docker/vision и docker/main
if [ -f "$ENV_FILE" ]; then
    # Vision Pi
    if [ -d "$DOCKER_DIR/vision" ]; then
        cp "$ENV_FILE" "$DOCKER_DIR/vision/.env"
        echo "✅ Скопирован $ENV_FILE → docker/vision/.env"
    fi
    
    # Main Pi
    if [ -d "$DOCKER_DIR/main" ]; then
        cp "$ENV_FILE" "$DOCKER_DIR/main/.env"
        echo "✅ Скопирован $ENV_FILE → docker/main/.env"
    fi
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✅ Docker теги настроены успешно!"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Будут использоваться образы с тегом: -humble-${IMAGE_TAG}"
    echo ""
    echo "Примеры:"
    echo "  - ghcr.io/krikz/rob_box:voice-assistant-humble-${IMAGE_TAG}"
    echo "  - ghcr.io/krikz/rob_box:oak-d-humble-${IMAGE_TAG}"
    echo "  - ghcr.io/krikz/rob_box:rtabmap-humble-${IMAGE_TAG}"
    echo ""
    echo "Для деплоя выполните:"
    echo "  cd docker/vision && docker-compose pull && docker-compose up -d"
    echo "  cd docker/main && docker-compose pull && docker-compose up -d"
    echo ""
else
    echo "❌ Ошибка: Файл $ENV_FILE не найден!"
    exit 1
fi

# Экспортируем переменные для использования в текущей сессии
export IMAGE_TAG
export ROS_DISTRO=humble
export REGISTRY=ghcr.io
export REPOSITORY_OWNER=krikz
export BASE_IMAGE_PREFIX="${REGISTRY}/${REPOSITORY_OWNER}/rob_box_base"
export SERVICE_IMAGE_PREFIX="${REGISTRY}/${REPOSITORY_OWNER}/rob_box"

echo "Переменные окружения установлены:"
echo "  IMAGE_TAG=$IMAGE_TAG"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  SERVICE_IMAGE_PREFIX=$SERVICE_IMAGE_PREFIX"
