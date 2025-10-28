#!/bin/bash
# Stop test environment
# Usage: ./stop_test_env.sh

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}🛑 Stopping Rob Box Test Environment${NC}"
echo "======================================"
echo ""

# Stop all services
docker compose -f docker-compose-x86-test.yaml down

echo ""
echo -e "${GREEN}✓ All services stopped${NC}"
echo ""
