#!/bin/bash
# Delete specific image from local registry
# Usage: ./delete_image_from_registry.sh <image-name>
# Example: ./delete_image_from_registry.sh robot-state-publisher

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

# Load environment
if [ -f "$BUILD_DIR/.env" ]; then
    source "$BUILD_DIR/.env"
fi

REGISTRY_URL="http://localhost:${REGISTRY_PORT:-5000}"
REGISTRY_CONTAINER="build-registry"

if [ -z "$1" ]; then
    echo "❌ Error: Image name required"
    echo ""
    echo "Usage: $0 <image-name>"
    echo ""
    echo "Available images:"
    curl -s "$REGISTRY_URL/v2/krikz/rob_box/tags/list" | jq -r '.tags[]' | grep -o '^[^-]*' | sort -u | sed 's/^/  - /'
    exit 1
fi

IMAGE_NAME="$1"

echo "🗑️  Deleting all versions of: $IMAGE_NAME"
echo "=========================================="
echo ""

# 1. Delete tags from registry API
echo "📦 Step 1: Removing tags from registry..."
TAGS=$(curl -s "$REGISTRY_URL/v2/krikz/rob_box/tags/list" | jq -r ".tags[] | select(contains(\"$IMAGE_NAME\"))")

if [ ! -z "$TAGS" ]; then
    echo "Found tags:"
    echo "$TAGS" | sed 's/^/  - /'
    
    for tag in $TAGS; do
        echo "  Deleting: $tag"
        
        # Try multiple manifest types
        for accept_header in \
            "application/vnd.docker.distribution.manifest.v2+json" \
            "application/vnd.oci.image.manifest.v1+json"; do
            
            DIGEST=$(curl -s -I -H "Accept: $accept_header" \
                     "$REGISTRY_URL/v2/krikz/rob_box/manifests/$tag" 2>/dev/null | \
                     grep -i "Docker-Content-Digest:" | awk '{print $2}' | tr -d '\r')
            
            if [ ! -z "$DIGEST" ]; then
                curl -s -X DELETE "$REGISTRY_URL/v2/krikz/rob_box/manifests/$DIGEST" > /dev/null 2>&1
            fi
        done
    done
fi

# 2. Stop registry for safe file operations
echo ""
echo "📦 Step 2: Stopping registry..."
cd "$BUILD_DIR"
docker compose down $REGISTRY_CONTAINER > /dev/null 2>&1

# 3. Remove tag directories
echo "📦 Step 3: Removing tag directories..."
TAG_DIRS=$(find ./data/registry/docker/registry/v2/repositories/krikz/rob_box/_manifests/tags/ -type d -name "*$IMAGE_NAME*" 2>/dev/null || true)

if [ ! -z "$TAG_DIRS" ]; then
    echo "Found directories:"
    echo "$TAG_DIRS" | sed 's/^/  - /'
    sudo rm -rf ./data/registry/docker/registry/v2/repositories/krikz/rob_box/_manifests/tags/*$IMAGE_NAME* 2>/dev/null || true
fi

# 4. Restart registry
echo "📦 Step 4: Restarting registry..."
docker compose up -d $REGISTRY_CONTAINER > /dev/null 2>&1
sleep 2

# 5. Run garbage collection
echo "📦 Step 5: Running garbage collection..."
docker exec $REGISTRY_CONTAINER bin/registry garbage-collect /etc/docker/registry/config.yml > /dev/null 2>&1

# 6. Remove local Docker images
echo "📦 Step 6: Removing local Docker images..."
docker images --format "{{.Repository}}:{{.Tag}}" | grep "$IMAGE_NAME" | xargs -r docker rmi -f > /dev/null 2>&1 || true

# 7. Clean buildx cache
echo "📦 Step 7: Cleaning buildx cache..."
docker builder prune -f > /dev/null 2>&1

echo ""
echo "✅ Successfully deleted all versions of $IMAGE_NAME"
echo ""
echo "Verification:"
REMAINING=$(curl -s "$REGISTRY_URL/v2/krikz/rob_box/tags/list" | jq -r ".tags[] | select(contains(\"$IMAGE_NAME\"))" || true)
if [ -z "$REMAINING" ]; then
    echo "  ✅ No tags remaining in registry"
else
    echo "  ⚠️  Some tags still present:"
    echo "$REMAINING" | sed 's/^/     - /'
fi

LOCAL_IMAGES=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "$IMAGE_NAME" || true)
if [ -z "$LOCAL_IMAGES" ]; then
    echo "  ✅ No local Docker images"
else
    echo "  ⚠️  Some local images still present:"
    echo "$LOCAL_IMAGES" | sed 's/^/     - /'
fi
