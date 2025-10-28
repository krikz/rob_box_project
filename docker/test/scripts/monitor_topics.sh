#!/bin/bash
# Monitor ROS2 topics in test environment
# Usage: ./monitor_topics.sh [continuous|once]

MODE=${1:-once}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}📊 ROS2 Topic Monitor${NC}"
echo "===================="
echo ""

# Check if any ROS2 container is running
if ! docker compose -f docker-compose-x86-test.yaml ps --filter "status=running" | grep -q "test-"; then
    echo -e "${RED}No containers are running. Start the environment first:${NC}"
    echo "  ./scripts/start_test_env.sh"
    exit 1
fi

monitor_once() {
    echo "Fetching topic list from Zenoh REST API..."
    echo ""
    
    # Get publishers from Zenoh
    echo -e "${BLUE}Publishers:${NC}"
    curl -s http://localhost:8000/@/local/publisher 2>/dev/null | \
        jq -r 'keys[]' 2>/dev/null | \
        grep -E '^rt/' | \
        sed 's|^rt/||' | \
        sort | \
        sed 's/^/  /' || echo "  (none)"
    
    echo ""
    
    # Get subscribers from Zenoh
    echo -e "${BLUE}Subscribers:${NC}"
    curl -s http://localhost:8000/@/local/subscriber 2>/dev/null | \
        jq -r 'keys[]' 2>/dev/null | \
        grep -E '^rt/' | \
        sed 's|^rt/||' | \
        sort | \
        sed 's/^/  /' || echo "  (none)"
    
    echo ""
    
    # Summary
    local pub_count=$(curl -s http://localhost:8000/@/local/publisher 2>/dev/null | jq -r 'keys[]' 2>/dev/null | grep -E '^rt/' | wc -l)
    local sub_count=$(curl -s http://localhost:8000/@/local/subscriber 2>/dev/null | jq -r 'keys[]' 2>/dev/null | grep -E '^rt/' | wc -l)
    
    echo -e "${GREEN}Total:${NC} $pub_count publishers, $sub_count subscribers"
    echo ""
    
    # Try to get topic list if ros2 is available locally
    if command -v ros2 &> /dev/null; then
        echo ""
        echo "Trying to get topic list with ros2 CLI..."
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp
        export ZENOH_SESSION_CONFIG_URI="$(pwd)/config/zenoh_session_config.json5"
        
        if timeout 5 ros2 topic list &>/dev/null; then
            echo ""
            echo -e "${BLUE}ROS2 Topics:${NC}"
            ros2 topic list | sed 's/^/  /'
        else
            echo -e "${YELLOW}Timeout waiting for ros2 topic list${NC}"
        fi
    else
        echo -e "${YELLOW}Note: Install ROS2 locally to see detailed topic info${NC}"
        echo "  sudo apt install ros-humble-ros-base ros-humble-rmw-zenoh-cpp"
    fi
}

monitor_continuous() {
    while true; do
        clear
        echo -e "${BLUE}📊 ROS2 Topic Monitor (Continuous)${NC}"
        echo "===================================="
        echo "Press Ctrl+C to stop"
        echo ""
        
        monitor_once
        
        sleep 2
    done
}

case "$MODE" in
    continuous|c|watch)
        monitor_continuous
        ;;
    once|o|*)
        monitor_once
        ;;
esac
