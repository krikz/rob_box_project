#!/bin/bash
# Clean up old images from local registry
# Usage: ./cleanup_registry.sh [--all] [--dry-run]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

# Load environment
if [ -f "$BUILD_DIR/.env" ]; then
    source "$BUILD_DIR/.env"
fi

REGISTRY_URL="http://localhost:${REGISTRY_PORT:-5000}"
DRY_RUN=false
DELETE_ALL=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --all)
            DELETE_ALL=true
            shift
            ;;
        *)
            ;;
    esac
done

echo "🧹 Registry Cleanup"
echo "==================="
echo ""

if [ "$DRY_RUN" = true ]; then
    echo "🔍 DRY RUN MODE - no images will be deleted"
    echo ""
fi

# Check if registry is accessible
if ! curl -s "$REGISTRY_URL/v2/" > /dev/null 2>&1; then
    echo "❌ Registry is not accessible at $REGISTRY_URL"
    echo "   Make sure the registry service is running."
    exit 1
fi

# Get list of repositories
echo "📦 Repositories in registry:"
REPOS=$(curl -s "$REGISTRY_URL/v2/_catalog" | jq -r '.repositories[]' 2>/dev/null || echo "")

if [ -z "$REPOS" ]; then
    echo "   No repositories found"
    exit 0
fi

echo "$REPOS" | sed 's/^/   - /'
echo ""

# For each repository, show tags
for repo in $REPOS; do
    echo "📂 Repository: $repo"
    TAGS=$(curl -s "$REGISTRY_URL/v2/$repo/tags/list" | jq -r '.tags[]' 2>/dev/null || echo "")
    
    if [ -z "$TAGS" ]; then
        echo "   No tags found"
        continue
    fi
    
    for tag in $TAGS; do
        # Get manifest digest
        DIGEST=$(curl -s -I -H "Accept: application/vnd.docker.distribution.manifest.v2+json" \
                 "$REGISTRY_URL/v2/$repo/manifests/$tag" | \
                 grep -i "Docker-Content-Digest:" | awk '{print $2}' | tr -d '\r')
        
        if [ "$DELETE_ALL" = true ]; then
            if [ "$DRY_RUN" = true ]; then
                echo "   [DRY RUN] Would delete: $repo:$tag (digest: $DIGEST)"
            else
                echo "   🗑️  Deleting: $repo:$tag"
                curl -X DELETE "$REGISTRY_URL/v2/$repo/manifests/$DIGEST" 2>/dev/null || echo "      Failed to delete"
            fi
        else
            echo "   - $tag (digest: ${DIGEST:0:20}...)"
        fi
    done
    echo ""
done

if [ "$DELETE_ALL" = false ] && [ "$DRY_RUN" = false ]; then
    echo "ℹ️  No images were deleted."
    echo "   Use --all flag to delete all images"
    echo "   Use --dry-run flag to see what would be deleted"
fi

if [ "$DELETE_ALL" = true ] && [ "$DRY_RUN" = false ]; then
    echo "🔄 Running garbage collection..."
    docker exec build-registry bin/registry garbage-collect /etc/docker/registry/config.yml
    echo "✅ Cleanup complete!"
fi
