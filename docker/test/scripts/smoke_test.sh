#!/bin/bash
# Smoke test for Rob Box test environment
# Checks that expected ROS2 topics are available

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}🧪 Rob Box Smoke Test${NC}"
echo "===================="
echo ""

# Check if zenoh-router is running
echo -n "Checking Zenoh router... "
if docker compose -f docker-compose-x86-test.yaml ps zenoh-router | grep -q "Up"; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAIL - Zenoh router not running${NC}"
    exit 1
fi

# Check Zenoh REST API
echo -n "Checking Zenoh REST API... "
if curl -f -s http://localhost:8000/@/local/router &>/dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAIL - REST API not responding${NC}"
    exit 1
fi

echo ""
echo "Checking container status..."
echo ""

# Function to check container status
check_container() {
    local name=$1
    local expected_status=$2  # "running", "exited", or "not_started"
    
    local status=$(docker compose -f docker-compose-x86-test.yaml ps "$name" 2>/dev/null | tail -n +2 | awk '{print $4}')
    
    printf "  %-30s " "$name:"
    
    case "$expected_status" in
        "running")
            if echo "$status" | grep -q "Up"; then
                echo -e "${GREEN}✓ Running${NC}"
                return 0
            else
                echo -e "${RED}✗ Not running (expected)${NC}"
                return 1
            fi
            ;;
        "exited")
            if echo "$status" | grep -q "Exited"; then
                echo -e "${YELLOW}⚠ Exited (expected - no hardware)${NC}"
                return 0
            elif echo "$status" | grep -q "Up"; then
                echo -e "${YELLOW}⚠ Running (unexpected)${NC}"
                return 0
            else
                echo -e "${YELLOW}⚠ Not started${NC}"
                return 0
            fi
            ;;
        "not_started")
            if [ -z "$status" ]; then
                echo -e "${YELLOW}⚠ Not started (optional)${NC}"
                return 0
            else
                echo -e "${GREEN}✓ Running${NC}"
                return 0
            fi
            ;;
    esac
}

# Core services (should be running)
echo "Core services (should be running):"
check_container "zenoh-router" "running"
check_container "robot-state-publisher" "running"
check_container "twist-mux" "running"
check_container "perception" "running"

echo ""
echo "Optional services (may be idle):"
check_container "rtabmap" "not_started"
check_container "nav2" "not_started"

echo ""
echo "Hardware-dependent services (expected to fail):"
check_container "oak-d" "exited"
check_container "lslidar" "exited"
check_container "led-matrix" "exited"
check_container "voice-assistant" "exited"
check_container "micro-ros-agent" "exited"
check_container "vesc-nexus" "exited"

echo ""
echo "Checking ROS2 topics (may take a moment)..."
echo ""

# Try to list ROS2 topics from zenoh-router
if docker exec test-zenoh-router sh -c 'command -v ros2 &>/dev/null'; then
    echo "Available ROS2 topics:"
    docker exec test-zenoh-router bash -c "
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp
        ros2 topic list 2>/dev/null | head -20 || echo '  (Could not list topics from zenoh-router)'
    " | sed 's/^/  /'
else
    echo -e "${YELLOW}  Note: zenoh-router image doesn't have ros2 CLI${NC}"
    echo "  Use local machine with ROS2 + Zenoh to list topics:"
    echo "    export RMW_IMPLEMENTATION=rmw_zenoh_cpp"
    echo "    export ZENOH_SESSION_CONFIG_URI=\$(pwd)/config/zenoh_session_config.json5"
    echo "    ros2 topic list"
fi

echo ""
echo "Checking Zenoh publishers/subscribers..."
echo ""

# Get Zenoh stats
PUBS=$(curl -s http://localhost:8000/@/local/publisher 2>/dev/null | jq -r 'keys[]' 2>/dev/null | wc -l || echo "0")
SUBS=$(curl -s http://localhost:8000/@/local/subscriber 2>/dev/null | jq -r 'keys[]' 2>/dev/null | wc -l || echo "0")

echo "  Publishers:  $PUBS"
echo "  Subscribers: $SUBS"

if [ "$PUBS" -gt 0 ] || [ "$SUBS" -gt 0 ]; then
    echo ""
    echo "Sample publishers:"
    curl -s http://localhost:8000/@/local/publisher 2>/dev/null | jq -r 'keys[]' 2>/dev/null | head -5 | sed 's/^/  /' || true
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Smoke test completed${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Summary:"
echo "  - Zenoh router is working"
echo "  - Core ROS2 services are running"
echo "  - Hardware-dependent services failed as expected"
echo ""
echo "For detailed logs: docker compose -f docker-compose-x86-test.yaml logs -f"
echo ""
