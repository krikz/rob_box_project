#!/bin/bash
# Docker Smoke Tests - quick sanity checks for all containers
# Usage: ./test_docker_smoke.sh [service_name]

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

FAILED_TESTS=()
PASSED_TESTS=()

# Test counter
TOTAL_TESTS=0
PASSED=0
FAILED=0

echo "🧪 Docker Smoke Tests"
echo "===================="
echo ""

# Helper functions
test_start() {
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    echo -n "Testing $1... "
}

test_pass() {
    PASSED=$((PASSED + 1))
    PASSED_TESTS+=("$1")
    echo -e "${GREEN}✓ PASS${NC}"
}

test_fail() {
    FAILED=$((FAILED + 1))
    FAILED_TESTS+=("$1: $2")
    echo -e "${RED}✗ FAIL${NC} - $2"
}

# Test if image exists
test_image_exists() {
    local image=$1
    test_start "Image exists: $image"
    
    if docker image inspect "$image" &>/dev/null; then
        test_pass "Image: $image"
    else
        test_fail "Image: $image" "Image not found"
    fi
}

# Test if container starts
test_container_starts() {
    local image=$1
    local container_name=$2
    local timeout=${3:-10}
    
    test_start "Container starts: $container_name"
    
    # Stop existing container
    docker rm -f "$container_name" &>/dev/null || true
    
    # Start container in detached mode
    if docker run -d --name "$container_name" "$image" &>/dev/null; then
        sleep 2
        
        # Check if still running
        if docker ps --filter "name=$container_name" --filter "status=running" | grep -q "$container_name"; then
            test_pass "Container: $container_name"
            docker rm -f "$container_name" &>/dev/null
        else
            local logs=$(docker logs "$container_name" 2>&1 | tail -5)
            test_fail "Container: $container_name" "Container exited. Logs: $logs"
            docker rm -f "$container_name" &>/dev/null
        fi
    else
        test_fail "Container: $container_name" "Failed to start"
    fi
}

# Test ROS2 node list
test_ros2_nodes() {
    local container_name=$1
    local expected_node=$2
    
    test_start "ROS2 nodes in: $container_name"
    
    if docker exec "$container_name" bash -c "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | grep -q "$expected_node"; then
        test_pass "ROS2 nodes: $container_name"
    else
        test_fail "ROS2 nodes: $container_name" "Expected node '$expected_node' not found"
    fi
}

# Test if service responds
test_http_endpoint() {
    local container_name=$1
    local port=$2
    local path=${3:-"/"}
    
    test_start "HTTP endpoint: $container_name:$port$path"
    
    local container_ip=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$container_name")
    
    if curl -f -s "http://$container_ip:$port$path" &>/dev/null; then
        test_pass "HTTP: $container_name:$port"
    else
        test_fail "HTTP: $container_name:$port" "Endpoint not responding"
    fi
}

# Test Zenoh router
test_zenoh_router() {
    echo ""
    echo "📡 Testing Zenoh Router"
    echo "----------------------"
    
    local image="ghcr.io/krikz/rob_box:zenoh-router-humble-latest"
    local container="test-zenoh-router"
    
    test_image_exists "$image"
    test_container_starts "$image" "$container"
    
    # Test REST API
    if docker ps --filter "name=$container" --filter "status=running" | grep -q "$container"; then
        sleep 3
        test_http_endpoint "$container" 8000 "/@/local"
        docker rm -f "$container" &>/dev/null
    fi
}

# Test OAK-D camera
test_oak_d() {
    echo ""
    echo "📹 Testing OAK-D Camera"
    echo "----------------------"
    
    local image="ghcr.io/krikz/rob_box:oak-d-humble-latest"
    
    test_image_exists "$image"
    # Skip container start (requires hardware)
    echo "⚠️  Skipping container test (requires OAK-D hardware)"
}

# Test RTAB-Map
test_rtabmap() {
    echo ""
    echo "🗺️  Testing RTAB-Map"
    echo "-------------------"
    
    local image="ghcr.io/krikz/rob_box:rtabmap-humble-latest"
    
    test_image_exists "$image"
    # Skip container start (requires camera/lidar topics)
    echo "⚠️  Skipping container test (requires sensor topics)"
}

# Test Voice Assistant
test_voice_assistant() {
    echo ""
    echo "🎤 Testing Voice Assistant"
    echo "-------------------------"
    
    local image="ghcr.io/krikz/rob_box:voice-assistant-humble-test"
    
    test_image_exists "$image"
    # Skip container start (requires API keys + hardware)
    echo "⚠️  Skipping container test (requires API keys + ReSpeaker)"
}

# Test Animation Player
test_animation_player() {
    echo ""
    echo "🎨 Testing Animation Player"
    echo "--------------------------"
    
    local image="ghcr.io/krikz/rob_box:animation-player-humble-latest"
    
    test_image_exists "$image"
    # Skip container start (requires LED hardware)
    echo "⚠️  Skipping container test (requires LED matrices)"
}

# Main test suite
main() {
    local service=$1
    
    if [ -z "$service" ]; then
        # Test all services
        test_zenoh_router
        test_oak_d
        test_rtabmap
        test_voice_assistant
        test_animation_player
    else
        # Test specific service
        case $service in
            zenoh|zenoh-router)
                test_zenoh_router
                ;;
            oak|oak-d|camera)
                test_oak_d
                ;;
            rtabmap|slam)
                test_rtabmap
                ;;
            voice|voice-assistant)
                test_voice_assistant
                ;;
            animation|animation-player)
                test_animation_player
                ;;
            *)
                echo "Unknown service: $service"
                echo "Available: zenoh, oak-d, rtabmap, voice-assistant, animation-player"
                exit 1
                ;;
        esac
    fi
    
    # Summary
    echo ""
    echo "📊 Test Summary"
    echo "==============="
    echo "Total tests: $TOTAL_TESTS"
    echo -e "Passed: ${GREEN}$PASSED${NC}"
    echo -e "Failed: ${RED}$FAILED${NC}"
    echo ""
    
    if [ $FAILED -gt 0 ]; then
        echo -e "${RED}Failed tests:${NC}"
        for test in "${FAILED_TESTS[@]}"; do
            echo "  - $test"
        done
        echo ""
        exit 1
    else
        echo -e "${GREEN}✓ All tests passed!${NC}"
        exit 0
    fi
}

# Cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up test containers..."
    docker rm -f test-zenoh-router test-oak-d test-rtabmap test-voice-assistant test-animation-player &>/dev/null || true
}

trap cleanup EXIT

# Run tests
main "$@"
