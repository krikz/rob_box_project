#!/bin/bash
# Update and restart build machine services
# Similar to update_and_restart.sh on Main/Vision Pi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$(dirname "$BUILD_DIR")")"

echo "🔄 Build Machine Update and Restart"
echo "===================================="
echo ""

# Navigate to project root
cd "$PROJECT_ROOT"

echo "📥 Pulling latest changes from git..."
git pull origin $(git rev-parse --abbrev-ref HEAD)
echo "   ✅ Git pull complete"
echo ""

echo "🐳 Pulling latest Docker images..."
cd "$BUILD_DIR"
docker compose pull
echo "   ✅ Docker images updated"
echo ""

echo "🔄 Restarting services..."
docker compose down
docker compose up -d
echo "   ✅ Services restarted"
echo ""

echo "⏳ Waiting for services to be ready..."
sleep 5

# Check status
"$SCRIPT_DIR/check_status.sh"

echo ""
echo "✅ Update and restart complete!"
