#!/bin/bash
# Quick build script для Voice Assistant контейнера
# ВАЖНО: Основная сборка идёт через GitHub Actions workflow
# Этот скрипт только для локального тестирования

set -e

echo "=========================================="
echo "  Building Voice Assistant Container"
echo "=========================================="
echo "⚠ NOTE: Production builds are done via GitHub Actions"
echo "   This is for local testing only"
echo ""

# Параметры
IMAGE_NAME_LOCAL="rob_box:voice-assistant-humble-test"
IMAGE_NAME_PROD="ghcr.io/krikz/rob_box:voice-assistant-humble-latest"
DOCKERFILE="docker/vision/voice_assistant/Dockerfile"
CONTEXT="."

# Переход в корень проекта
cd "$(dirname "$0")/../.."

# Проверка существования Dockerfile
if [ ! -f "$DOCKERFILE" ]; then
    echo "ERROR: Dockerfile not found: $DOCKERFILE"
    exit 1
fi

# Сборка
echo "Building image: $IMAGE_NAME_LOCAL"
echo "Dockerfile: $DOCKERFILE"
echo "Context: $CONTEXT"
echo ""

docker build \
    --platform linux/arm64 \
    -t "$IMAGE_NAME_LOCAL" \
    -f "$DOCKERFILE" \
    "$CONTEXT"

echo ""
echo "=========================================="
echo "  Build Complete!"
echo "=========================================="
echo "Local image: $IMAGE_NAME_LOCAL"
echo ""
echo "Next steps:"
echo "  1. Test locally:"
echo "     docker run -it --rm --network host --privileged \\"
echo "       --device /dev/snd --device /dev/bus/usb \\"
echo "       $IMAGE_NAME_LOCAL /bin/bash"
echo ""
echo "  2. For production deployment:"
echo "     - Push to feature branch"
echo "     - Merge to develop → builds $IMAGE_NAME_PROD with 'dev' tag"
echo "     - Merge to main → builds $IMAGE_NAME_PROD with 'latest' tag"
echo ""
echo "  3. Deploy on Vision Pi from GitHub registry:"
echo "     cd docker/vision"
echo "     docker-compose pull voice-assistant"
echo "     docker-compose up -d voice-assistant"
echo ""
