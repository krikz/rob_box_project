#!/bin/bash
# Validate Zenoh Namespace Configuration
# This script checks if Zenoh namespace is properly configured for cloud connectivity

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Zenoh Namespace Configuration Validator                    ║${NC}"
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo ""

# Check if running on correct host
HOST_TYPE=""
if [ -f "/etc/hostname" ]; then
    HOSTNAME=$(cat /etc/hostname)
    if [[ "$HOSTNAME" == *"vision"* ]]; then
        HOST_TYPE="vision"
    elif [[ "$HOSTNAME" == *"main"* ]]; then
        HOST_TYPE="main"
    fi
fi

if [ -z "$HOST_TYPE" ]; then
    echo -e "${YELLOW}⚠️  Cannot determine if this is Vision Pi or Main Pi${NC}"
    echo "Please run this script on Vision Pi or Main Pi"
    exit 1
fi

echo -e "${GREEN}✅ Detected: ${HOST_TYPE^^} Pi${NC}"
echo ""

# Function to check command existence
check_command() {
    if ! command -v $1 &> /dev/null; then
        echo -e "${RED}✗ $1 not found${NC}"
        return 1
    fi
    echo -e "${GREEN}✓ $1 found${NC}"
    return 0
}

# Function to check Docker container
check_container() {
    local container=$1
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        echo -e "${GREEN}✓ Container ${container} is running${NC}"
        return 0
    else
        echo -e "${RED}✗ Container ${container} is not running${NC}"
        return 1
    fi
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. Checking Prerequisites"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_command docker || exit 1
check_command curl || exit 1
check_command grep || exit 1

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. Checking ROBOT_ID Configuration"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check .env file
ENV_FILE="$HOME/rob_box_project/docker/${HOST_TYPE}/.env"
if [ ! -f "$ENV_FILE" ]; then
    echo -e "${RED}✗ Environment file not found: $ENV_FILE${NC}"
    exit 1
fi

ROBOT_ID=$(grep "^ROBOT_ID=" "$ENV_FILE" | cut -d'=' -f2)
if [ -z "$ROBOT_ID" ]; then
    echo -e "${RED}✗ ROBOT_ID not set in $ENV_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROBOT_ID found in .env: ${ROBOT_ID}${NC}"

# Validate ROBOT_ID format
if [[ ! "$ROBOT_ID" =~ ^[A-Za-z0-9_]+$ ]]; then
    echo -e "${RED}✗ ROBOT_ID contains invalid characters${NC}"
    echo -e "  Only alphanumeric and underscore allowed"
    exit 1
fi

echo -e "${GREEN}✓ ROBOT_ID format is valid${NC}"
echo -e "${BLUE}  Expected namespace: robots/${ROBOT_ID}${NC}"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. Checking Docker Containers"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ "$HOST_TYPE" == "vision" ]; then
    CONTAINERS=("zenoh-router-vision" "oak-d" "lslidar" "voice-assistant")
else
    CONTAINERS=("zenoh-router" "rtabmap" "nav2")
fi

ALL_RUNNING=true
for container in "${CONTAINERS[@]}"; do
    check_container "$container" || ALL_RUNNING=false
done

if [ "$ALL_RUNNING" = false ]; then
    echo -e "${YELLOW}⚠️  Some containers are not running${NC}"
    echo "Start them with: cd ~/rob_box_project/docker/${HOST_TYPE} && docker-compose up -d"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. Checking Namespace in Containers"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check containers that should have namespace
if [ "$HOST_TYPE" == "vision" ]; then
    NAMESPACE_CONTAINERS=("oak-d" "lslidar" "voice-assistant")
else
    NAMESPACE_CONTAINERS=()  # Main Pi nodes may or may not use namespace
fi

for container in "${NAMESPACE_CONTAINERS[@]}"; do
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        echo ""
        echo "Checking $container..."
        
        # Check environment variable
        CONTAINER_ROBOT_ID=$(docker exec "$container" env 2>/dev/null | grep "^ROBOT_ID=" | cut -d'=' -f2 || echo "")
        if [ "$CONTAINER_ROBOT_ID" == "$ROBOT_ID" ]; then
            echo -e "${GREEN}  ✓ ROBOT_ID environment variable: $CONTAINER_ROBOT_ID${NC}"
        else
            echo -e "${RED}  ✗ ROBOT_ID mismatch: expected $ROBOT_ID, got $CONTAINER_ROBOT_ID${NC}"
        fi
        
        # Check generated config
        NAMESPACE_IN_CONFIG=$(docker exec "$container" cat /tmp/zenoh_session_config.json5 2>/dev/null | grep -E '^\s*namespace:' | sed 's/.*"\(.*\)".*/\1/' || echo "")
        if [ "$NAMESPACE_IN_CONFIG" == "robots/$ROBOT_ID" ]; then
            echo -e "${GREEN}  ✓ Namespace in config: $NAMESPACE_IN_CONFIG${NC}"
        else
            echo -e "${RED}  ✗ Namespace not found or incorrect in /tmp/zenoh_session_config.json5${NC}"
            echo -e "    Expected: robots/$ROBOT_ID"
            echo -e "    Got: $NAMESPACE_IN_CONFIG"
        fi
        
        # Check container logs for namespace confirmation
        if docker logs "$container" 2>&1 | tail -100 | grep -q "Namespace: robots/$ROBOT_ID"; then
            echo -e "${GREEN}  ✓ Namespace confirmed in container logs${NC}"
        else
            echo -e "${YELLOW}  ⚠️  Namespace not found in recent logs (may be older)${NC}"
        fi
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. Checking Zenoh Router Connectivity"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check local Zenoh router REST API
if curl -s http://localhost:8000/@/router/status >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Local Zenoh router REST API is accessible${NC}"
    
    # Check for topics with namespace
    echo ""
    echo "Checking for topics with namespace prefix..."
    TOPICS=$(curl -s http://localhost:8000/@/local/subscriber 2>/dev/null | grep -o "robots/$ROBOT_ID/[^\"]*" | head -5)
    
    if [ -n "$TOPICS" ]; then
        echo -e "${GREEN}✓ Found topics with namespace robots/$ROBOT_ID:${NC}"
        echo "$TOPICS" | while read topic; do
            echo -e "  ${BLUE}→ $topic${NC}"
        done
    else
        echo -e "${YELLOW}⚠️  No topics with namespace robots/$ROBOT_ID found yet${NC}"
        echo "This may be normal if nodes haven't published anything yet"
    fi
else
    echo -e "${RED}✗ Cannot access Zenoh router REST API${NC}"
    echo "Check if zenoh-router container is running"
fi

# Check cloud connectivity (Main Pi only)
if [ "$HOST_TYPE" == "main" ]; then
    echo ""
    echo "Checking cloud connectivity..."
    
    # Check if Main Pi router config has cloud endpoint
    ROUTER_CONFIG="$HOME/rob_box_project/docker/main/config/zenoh_router_config.json5"
    if grep -q "zenoh.robbox.online" "$ROUTER_CONFIG"; then
        echo -e "${GREEN}✓ Cloud endpoint configured in router config${NC}"
        
        # Test network connectivity to cloud
        if timeout 3 bash -c "cat < /dev/null > /dev/tcp/zenoh.robbox.online/7447" 2>/dev/null; then
            echo -e "${GREEN}✓ Can connect to zenoh.robbox.online:7447${NC}"
        else
            echo -e "${RED}✗ Cannot connect to zenoh.robbox.online:7447${NC}"
            echo "Check network connectivity and firewall"
        fi
    else
        echo -e "${YELLOW}⚠️  Cloud endpoint not configured in router config${NC}"
    fi
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. Summary"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo ""
echo -e "${BLUE}Namespace Configuration:${NC}"
echo -e "  Robot ID:  ${GREEN}${ROBOT_ID}${NC}"
echo -e "  Namespace: ${GREEN}robots/${ROBOT_ID}${NC}"
echo -e "  Host:      ${GREEN}${HOST_TYPE^^} Pi${NC}"
echo ""

echo -e "${BLUE}Expected Topic Format in Cloud:${NC}"
echo -e "  ${GREEN}robots/${ROBOT_ID}/camera/rgb/image_raw${NC}"
echo -e "  ${GREEN}robots/${ROBOT_ID}/cmd_vel${NC}"
echo -e "  ${GREEN}robots/${ROBOT_ID}/odom${NC}"
echo ""

echo -e "${BLUE}Cloud Subscription Examples:${NC}"
echo -e "  All robots: ${YELLOW}robots/**${NC}"
echo -e "  This robot: ${YELLOW}robots/${ROBOT_ID}/**${NC}"
echo -e "  Specific:   ${YELLOW}robots/${ROBOT_ID}/camera/**${NC}"
echo ""

echo -e "${GREEN}✅ Validation Complete${NC}"
echo ""
