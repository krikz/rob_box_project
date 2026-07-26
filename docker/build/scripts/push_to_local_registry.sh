#!/bin/bash
# Push Docker image to local registry
# Usage: ./push_to_local_registry.sh <source-image> [<target-tag>]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

# Load environment
if [ -f "$BUILD_DIR/.env" ]; then
    source "$BUILD_DIR/.env"
fi

BUILD_MACHINE_IP="${BUILD_MACHINE_IP:-localhost}"
REGISTRY_PORT="${REGISTRY_PORT:-5000}"

# Parse arguments
SOURCE_IMAGE="$1"
TARGET_TAG="$2"

if [ -z "$SOURCE_IMAGE" ]; then
    echo "Usage: $0 <source-image> [<target-tag>]"
    echo ""
    echo "Examples:"
    echo "  $0 ghcr.io/krikz/rob_box:oak-d-kilted-latest"
    echo "  $0 ghcr.io/krikz/rob_box:rtabmap-kilted-latest rtabmap-latest"
    echo ""
    exit 1
fi

# Determine target tag
if [ -z "$TARGET_TAG" ]; then
    # Extract tag from source image
    TARGET_TAG="${SOURCE_IMAGE##*/}"
fi

LOCAL_IMAGE="$BUILD_MACHINE_IP:$REGISTRY_PORT/$TARGET_TAG"

echo "🐳 Pushing image to local registry"
echo "===================================="
echo ""
echo "Source: $SOURCE_IMAGE"
echo "Target: $LOCAL_IMAGE"
echo ""

# Pull source image if not already present
if ! docker image inspect "$SOURCE_IMAGE" > /dev/null 2>&1; then
    echo "📥 Pulling source image..."
    docker pull "$SOURCE_IMAGE"
fi

# Tag for local registry
echo "🏷️  Tagging image..."
docker tag "$SOURCE_IMAGE" "$LOCAL_IMAGE"

# Push to local registry
echo "📤 Pushing to local registry..."
docker push "$LOCAL_IMAGE"

echo ""
echo "✅ Image pushed successfully!"
echo ""
echo "📍 Image available at: $LOCAL_IMAGE"
echo "🔍 View in Registry UI: http://$BUILD_MACHINE_IP:8080"
