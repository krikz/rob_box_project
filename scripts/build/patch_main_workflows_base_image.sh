#!/bin/bash
# Автоматическое добавление BASE_IMAGE build-arg в оставшиеся job-ы build-main-services.yml

set -e

WORKFLOW_FILE=".github/workflows/build-main-services.yml"

echo "🔧 Patching $WORKFLOW_FILE with BASE_IMAGE build-args..."

# micro-ros-agent
echo "  - Patching micro-ros-agent..."
sed -i '/name: Build and push micro-ros-agent/,/cache-to: type=gha,mode=max/ {
  /cache-to: type=gha,mode=max/a\          build-args: |\n            BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || '\''ghcr.io/krikz/rob_box_base'\'' }}:ros2-zenoh
}' "$WORKFLOW_FILE"

# ros2-control
echo "  - Patching ros2-control..."
sed -i '/name: Build and push ros2-control/,/cache-to: type=gha,mode=max/ {
  /cache-to: type=gha,mode=max/a\          build-args: |\n            BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || '\''ghcr.io/krikz/rob_box_base'\'' }}:ros2-zenoh
}' "$WORKFLOW_FILE"

# nav2
echo "  - Patching nav2..."
sed -i '/name: Build and push nav2/,/cache-to: type=gha,mode=max/ {
  /cache-to: type=gha,mode=max/a\          build-args: |\n            BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || '\''ghcr.io/krikz/rob_box_base'\'' }}:ros2-zenoh
}' "$WORKFLOW_FILE"

# lslidar
echo "  - Patching lslidar..."
sed -i '/name: Build and push lslidar/,/cache-to: type=gha,mode=max/ {
  /cache-to: type=gha,mode=max/a\          build-args: |\n            BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || '\''ghcr.io/krikz/rob_box_base'\'' }}:pcl
}' "$WORKFLOW_FILE"

# perception
echo "  - Patching perception..."
sed -i '/name: Build and push perception/,/cache-to: type=gha,mode=max/ {
  /cache-to: type=gha,mode=max/a\          build-args: |\n            BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || '\''ghcr.io/krikz/rob_box_base'\'' }}:ros2-zenoh
}' "$WORKFLOW_FILE"

echo "✅ All main services patched successfully!"
echo "📝 Please review the changes with: git diff $WORKFLOW_FILE"
