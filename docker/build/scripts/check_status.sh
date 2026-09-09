#!/bin/bash
# Check status of build machine services

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

echo "📊 Rob Box Build Machine Status"
echo "================================"
echo ""

# Load environment
if [ -f "$BUILD_DIR/.env" ]; then
    source "$BUILD_DIR/.env"
fi

# Check if services are running
echo "🐳 Docker Containers:"
docker compose -f "$BUILD_DIR/docker-compose.yaml" ps
echo ""

# Check registry
echo "🔍 Docker Registry:"
if curl -s http://localhost:5000/v2/_catalog > /dev/null 2>&1; then
    echo "   ✅ Registry is accessible"
    IMAGES=$(curl -s http://localhost:5000/v2/_catalog | jq -r '.repositories[]' 2>/dev/null || echo "")
    if [ -z "$IMAGES" ]; then
        echo "   📦 No images stored yet"
    else
        echo "   📦 Stored images:"
        echo "$IMAGES" | sed 's/^/      - /'
    fi
else
    echo "   ❌ Registry is not accessible"
fi
echo ""

# Check APT cache
echo "🗄️  APT Cache:"
if curl -s http://localhost:3142/acng-report.html > /dev/null 2>&1; then
    echo "   ✅ APT Cache is accessible"
    echo "   📊 Report: http://localhost:3142/acng-report.html"
else
    echo "   ❌ APT Cache is not accessible"
fi
echo ""

# Check GitHub runner
echo "🤖 GitHub Actions Runner:"
RUNNER_STATUS=$(docker inspect build-github-runner --format='{{.State.Status}}' 2>/dev/null || echo "not_found")
if [ "$RUNNER_STATUS" = "running" ]; then
    echo "   ✅ Runner container is running"
    echo "   📝 Recent logs:"
    docker logs build-github-runner --tail 5 2>&1 | sed 's/^/      /'
else
    echo "   ❌ Runner container is not running: $RUNNER_STATUS"
fi
echo ""

# Check disk usage
echo "💾 Disk Usage:"
echo "   Registry: $(du -sh "$BUILD_DIR/data/registry" 2>/dev/null | cut -f1 || echo "N/A")"
echo "   APT Cache: $(du -sh "$BUILD_DIR/data/apt-cache" 2>/dev/null | cut -f1 || echo "N/A")"
echo "   Runner: $(du -sh "$BUILD_DIR/data/runner" 2>/dev/null | cut -f1 || echo "N/A")"
echo ""

# Summary
echo "📍 Service URLs:"
echo "   - Registry:         http://${BUILD_MACHINE_IP:-localhost}:5000"
echo "   - Registry UI:      http://${BUILD_MACHINE_IP:-localhost}:8080"
echo "   - APT Cache:        http://${BUILD_MACHINE_IP:-localhost}:3142"
echo "   - APT Cache Report: http://${BUILD_MACHINE_IP:-localhost}:3142/acng-report.html"
echo ""

# Check if all critical services are healthy
ALL_HEALTHY=true
for service in build-registry build-apt-cache; do
    HEALTH=$(docker inspect $service --format='{{.State.Health.Status}}' 2>/dev/null || echo "unknown")
    if [ "$HEALTH" != "healthy" ]; then
        ALL_HEALTHY=false
    fi
done

if [ "$ALL_HEALTHY" = true ]; then
    echo "✅ All services are healthy"
else
    echo "⚠️  Some services are not healthy. Check logs with:"
    echo "   docker compose -f $BUILD_DIR/docker-compose.yaml logs"
fi
