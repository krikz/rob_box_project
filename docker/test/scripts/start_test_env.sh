#!/bin/bash
# Start test environment on x86_64 with QEMU emulation
# Usage: ./start_test_env.sh [service1 service2 ...]

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}🚀 Starting Rob Box Test Environment on x86_64${NC}"
echo "================================================"
echo ""

# Check if QEMU is installed
echo -n "Checking QEMU installation... "
if ! command -v qemu-aarch64-static &> /dev/null; then
    echo -e "${RED}✗ FAIL${NC}"
    echo ""
    echo "QEMU is not installed. Please install it:"
    echo "  sudo apt-get install -y qemu-user-static binfmt-support"
    echo "  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    exit 1
fi
echo -e "${GREEN}✓ OK${NC}"

# Check if QEMU is registered
echo -n "Checking QEMU registration... "
if [ ! -f /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo ""
    echo "QEMU is not registered in kernel. Registering now..."
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    echo -e "${GREEN}✓ Registered${NC}"
else
    echo -e "${GREEN}✓ OK${NC}"
fi

# Check if .env exists
if [ ! -f .env ]; then
    echo ""
    echo -e "${YELLOW}⚠ .env file not found. Creating from .env.example...${NC}"
    cp .env.example .env
    echo -e "${GREEN}✓ Created .env${NC}"
fi

echo ""
echo "Starting services..."
echo ""

# Start services
if [ $# -eq 0 ]; then
    # Start only working services by default (no hardware dependencies)
    echo "Starting core services (without hardware dependencies)..."
    docker compose -f docker-compose-x86-test.yaml up -d \
        zenoh-router \
        robot-state-publisher \
        twist-mux \
        perception
else
    # Start specified services
    echo "Starting specified services: $@"
    docker compose -f docker-compose-x86-test.yaml up -d "$@"
fi

echo ""
echo -e "${GREEN}✓ Services started${NC}"
echo ""
echo "Waiting for services to initialize..."
sleep 5

# Show status
echo ""
echo "Service status:"
docker compose -f docker-compose-x86-test.yaml ps

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Test environment is ready!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Check logs:       docker compose -f docker-compose-x86-test.yaml logs -f"
echo "  2. Run smoke test:   ./scripts/smoke_test.sh"
echo "  3. Monitor topics:   ./scripts/monitor_topics.sh"
echo "  4. Check health:     ./scripts/check_health.sh"
echo ""
echo "To start ALL services (including hardware-dependent, will fail):"
echo "  docker compose -f docker-compose-x86-test.yaml up -d"
echo ""
echo "To stop:"
echo "  docker compose -f docker-compose-x86-test.yaml down"
echo ""
