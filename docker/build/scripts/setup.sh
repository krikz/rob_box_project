#!/bin/bash
# Setup script for Build Machine
# Настройка инфраструктуры для локальной сборки Docker образов

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"

echo "🔧 Rob Box Build Machine Setup"
echo "==============================="
echo ""

# Check if running on correct architecture
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]; then
    echo "⚠️  Warning: Build machine is expected to be ARM64, but detected: $ARCH"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if .env exists
if [ ! -f "$BUILD_DIR/.env" ]; then
    echo "📝 Creating .env file from template..."
    cp "$BUILD_DIR/.env.example" "$BUILD_DIR/.env"
    
    # Try to detect IP address
    IP=$(hostname -I | awk '{print $1}')
    if [ ! -z "$IP" ]; then
        sed -i "s/BUILD_MACHINE_IP=.*/BUILD_MACHINE_IP=$IP/" "$BUILD_DIR/.env"
        echo "   Detected IP: $IP"
    fi
    
    echo "   ✅ Created .env file"
    echo "   📝 Please review and update $BUILD_DIR/.env if needed"
fi

# Check if .env.secrets exists
if [ ! -f "$BUILD_DIR/.env.secrets" ]; then
    echo ""
    echo "🔑 GitHub Actions Runner Configuration"
    echo "   To use the GitHub self-hosted runner, you need:"
    echo "   1. GitHub Personal Access Token (PAT)"
    echo "   2. Repository owner and name"
    echo ""
    
    cp "$BUILD_DIR/.env.secrets.example" "$BUILD_DIR/.env.secrets"
    
    echo "   📝 Created .env.secrets template"
    echo "   ⚠️  IMPORTANT: Edit $BUILD_DIR/.env.secrets with your GitHub credentials"
    echo "   Create token at: https://github.com/settings/tokens/new"
    echo "   Required scopes: repo, workflow"
    echo ""
    
    SETUP_RUNNER=false
else
    echo ""
    echo "✅ Found existing .env.secrets file"
    SETUP_RUNNER=true
fi

# Create data directories with proper permissions
echo ""
echo "📁 Creating data directories..."
mkdir -p "$BUILD_DIR/data/registry"
mkdir -p "$BUILD_DIR/data/apt-cache"
mkdir -p "$BUILD_DIR/data/runner"
chmod -R 755 "$BUILD_DIR/data"
echo "   ✅ Data directories created"

# Pull required Docker images
echo ""
echo "🐳 Pulling Docker images..."
docker compose -f "$BUILD_DIR/docker-compose.yaml" pull
echo "   ✅ Docker images pulled"

# Start services
echo ""
echo "🚀 Starting build machine services..."
docker compose -f "$BUILD_DIR/docker-compose.yaml" up -d
echo "   ✅ Services started"

# Wait for services to be healthy
echo ""
echo "⏳ Waiting for services to be ready..."
sleep 5

# Check service status
echo ""
echo "📊 Service Status:"
docker compose -f "$BUILD_DIR/docker-compose.yaml" ps

# Load .env for IP address
source "$BUILD_DIR/.env"

echo ""
echo "✅ Build Machine Setup Complete!"
echo ""
echo "📍 Service URLs:"
echo "   - Docker Registry:     http://${BUILD_MACHINE_IP}:5000"
echo "   - Registry UI:         http://${BUILD_MACHINE_IP}:8080"
echo "   - APT Cache:           http://${BUILD_MACHINE_IP}:3142"
echo "   - APT Cache Report:    http://${BUILD_MACHINE_IP}:3142/acng-report.html"
echo ""

if [ "$SETUP_RUNNER" = true ]; then
    echo "🤖 GitHub Actions Runner:"
    echo "   Status: Starting..."
    echo "   Check logs: docker logs -f build-github-runner"
    echo ""
    echo "   If runner doesn't start, verify .env.secrets configuration"
else
    echo "⚠️  GitHub Actions Runner NOT configured"
    echo "   Edit .env.secrets and restart: ./scripts/restart.sh"
fi

echo ""
echo "📚 Next Steps:"
echo "   1. Verify services: ./scripts/check_status.sh"
echo "   2. Configure Raspberry Pis to use local registry and cache"
echo "   3. Update GitHub workflows to use self-hosted runner"
echo ""
echo "📖 Documentation: docker/build/README.md"
