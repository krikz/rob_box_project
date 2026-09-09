#!/bin/bash
# Configure Raspberry Pi to use build machine services
# Run this script ON EACH Raspberry Pi (Main and Vision)

set -e

# Build machine IP (update this to match your build machine)
BUILD_MACHINE_IP="${BUILD_MACHINE_IP:-10.1.1.249}"
REGISTRY_PORT="${REGISTRY_PORT:-5000}"
APT_CACHE_PORT="${APT_CACHE_PORT:-3142}"

echo "🔧 Configuring Raspberry Pi for Build Machine"
echo "=============================================="
echo ""
echo "Build Machine IP: $BUILD_MACHINE_IP"
echo "Registry Port: $REGISTRY_PORT"
echo "APT Cache Port: $APT_CACHE_PORT"
echo ""

# 1. Configure Docker to use local registry as mirror
echo "1️⃣  Configuring Docker registry mirror..."

DOCKER_DAEMON_JSON="/etc/docker/daemon.json"
if [ ! -f "$DOCKER_DAEMON_JSON" ]; then
    echo '{}' | sudo tee "$DOCKER_DAEMON_JSON" > /dev/null
fi

# Add insecure registry and registry mirror
sudo python3 << EOF
import json

daemon_config_file = "$DOCKER_DAEMON_JSON"
build_registry = "http://$BUILD_MACHINE_IP:$REGISTRY_PORT"

# Read existing config
with open(daemon_config_file, 'r') as f:
    config = json.load(f)

# Add insecure registries
if 'insecure-registries' not in config:
    config['insecure-registries'] = []
if "$BUILD_MACHINE_IP:$REGISTRY_PORT" not in config['insecure-registries']:
    config['insecure-registries'].append("$BUILD_MACHINE_IP:$REGISTRY_PORT")

# Add registry mirrors
if 'registry-mirrors' not in config:
    config['registry-mirrors'] = []
if build_registry not in config['registry-mirrors']:
    config['registry-mirrors'].insert(0, build_registry)

# Write updated config
with open(daemon_config_file, 'w') as f:
    json.dump(config, f, indent=2)

print(f"✅ Added registry mirror: {build_registry}")
EOF

# Restart Docker daemon
echo "   Restarting Docker daemon..."
sudo systemctl restart docker
echo "   ✅ Docker configured"
echo ""

# 2. Configure APT to use cache
echo "2️⃣  Configuring APT cache..."

APT_PROXY_CONF="/etc/apt/apt.conf.d/02proxy"
echo "Acquire::http::Proxy \"http://$BUILD_MACHINE_IP:$APT_CACHE_PORT\";" | sudo tee "$APT_PROXY_CONF" > /dev/null
echo "   ✅ APT cache configured: $APT_PROXY_CONF"
echo ""

# 3. Test connections
echo "3️⃣  Testing connections..."

# Test registry
if curl -s "http://$BUILD_MACHINE_IP:$REGISTRY_PORT/v2/" > /dev/null 2>&1; then
    echo "   ✅ Docker Registry is accessible"
else
    echo "   ❌ Docker Registry is NOT accessible"
    echo "      Check that build machine is running and accessible"
fi

# Test APT cache
if curl -s "http://$BUILD_MACHINE_IP:$APT_CACHE_PORT/acng-report.html" > /dev/null 2>&1; then
    echo "   ✅ APT Cache is accessible"
else
    echo "   ❌ APT Cache is NOT accessible"
    echo "      Check that build machine is running and accessible"
fi

echo ""
echo "✅ Configuration Complete!"
echo ""
echo "📝 Next Steps:"
echo "   1. Update docker-compose.yaml files to pull from local registry"
echo "   2. Images will be automatically pulled from local registry if available"
echo "   3. APT packages will be cached automatically during builds"
echo ""
echo "🧪 Test APT cache:"
echo "   sudo apt-get update"
echo "   (packages will be cached on first download)"
echo ""
echo "🧪 Test Docker registry:"
echo "   docker pull $BUILD_MACHINE_IP:$REGISTRY_PORT/hello-world"
