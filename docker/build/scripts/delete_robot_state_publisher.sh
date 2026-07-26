#!/bin/bash
# Delete robot-state-publisher images from registry

set -e

REGISTRY_URL="http://localhost:5000"
REPO="krikz/rob_box"

echo "🗑️  Deleting robot-state-publisher images from registry"
echo "========================================================"
echo ""

# Tags to delete
TAGS_TO_DELETE=(
    "robot-state-publisher-kilted-dev"
    "robot-state-publisher-kilted-latest"
    "robot-state-publisher-kilted-local"
    "robot_state_publisher-kilted-dev"
)

for tag in "${TAGS_TO_DELETE[@]}"; do
    echo "🔍 Checking tag: $tag"
    
    # Get manifest digest
    DIGEST=$(curl -s -I -H "Accept: application/vnd.docker.distribution.manifest.v2+json" \
             "$REGISTRY_URL/v2/$REPO/manifests/$tag" 2>/dev/null | \
             grep -i "Docker-Content-Digest:" | awk '{print $2}' | tr -d '\r')
    
    if [ -z "$DIGEST" ]; then
        echo "   ⚠️  Tag not found, skipping"
        continue
    fi
    
    echo "   📝 Digest: ${DIGEST:0:30}..."
    
    # Delete manifest
    RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" -X DELETE \
               "$REGISTRY_URL/v2/$REPO/manifests/$DIGEST" 2>/dev/null)
    
    if [ "$RESPONSE" == "202" ] || [ "$RESPONSE" == "200" ]; then
        echo "   ✅ Deleted successfully"
    else
        echo "   ❌ Failed (HTTP $RESPONSE)"
    fi
    echo ""
done

echo "🔄 Running garbage collection..."
docker exec build-registry bin/registry garbage-collect /etc/docker/registry/config.yml

echo ""
echo "✅ Cleanup complete!"
echo ""
echo "Verify deletion:"
echo "  curl -s http://localhost:5000/v2/krikz/rob_box/tags/list | jq '.tags[] | select(contains(\"robot\"))'"
