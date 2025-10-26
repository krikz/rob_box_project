#!/bin/bash
# Restart build machine services

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

echo "🔄 Restarting Build Machine Services"
echo "====================================="
echo ""

# Stop services
echo "⏹️  Stopping services..."
docker compose -f "$BUILD_DIR/docker-compose.yaml" down
echo "   ✅ Services stopped"
echo ""

# Start services
echo "🚀 Starting services..."
docker compose -f "$BUILD_DIR/docker-compose.yaml" up -d
echo "   ✅ Services started"
echo ""

# Wait for services
echo "⏳ Waiting for services to be ready..."
sleep 5

# Check status
"$SCRIPT_DIR/check_status.sh"
