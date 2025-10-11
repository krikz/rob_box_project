#!/bin/bash
# Быстрая проверка сборки проблемных образов под ARM64

set -e
cd "$(dirname "$0")/.."

echo "🚀 Testing Docker Builds (ARM64)"
echo ""

# Setup QEMU
echo "📋 Setting up QEMU..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 2>/dev/null || true

# Setup buildx
echo "📋 Setting up buildx..."
docker buildx create --name arm64-builder --use 2>/dev/null || docker buildx use arm64-builder

echo ""
echo "🏗️  Building images..."
echo ""

# micro-ros-agent
echo "1️⃣  Building micro-ros-agent..."
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/micro_ros_agent/Dockerfile \
  --tag rob_box_test:micro-ros-agent \
  --load \
  . || echo "❌ FAILED"

# nav2
echo "2️⃣  Building nav2..."
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/nav2/Dockerfile \
  --tag rob_box_test:nav2 \
  --load \
  . || echo "❌ FAILED"

# vesc-nexus
echo "3️⃣  Building vesc-nexus..."
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/vesc_nexus/Dockerfile \
  --tag rob_box_test:vesc-nexus \
  --load \
  . || echo "❌ FAILED"

# led-matrix
echo "4️⃣  Building led-matrix..."
docker buildx build \
  --platform linux/arm64 \
  --file docker/vision/led_matrix/Dockerfile \
  --tag rob_box_test:led-matrix \
  --load \
  . || echo "❌ FAILED"

echo ""
echo "✅ Done! Check images:"
docker images | grep rob_box_test || echo "No images built"
