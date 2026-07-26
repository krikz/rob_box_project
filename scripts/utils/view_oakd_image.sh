#!/bin/bash
# View OAK-D Lite compressed image with Zenoh RMW
# This script connects to Vision Pi (10.1.1.21) via Zenoh to view camera feed

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  OAK-D Lite Camera Viewer${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 is not sourced!${NC}"
    echo -e "${YELLOW}Run: source /opt/ros/kilted/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS 2 Distro: ${ROS_DISTRO}"

# Source workspace if available
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo -e "${GREEN}✓${NC} Workspace sourced"
fi

# Check if image_view is installed
if ! ros2 pkg list 2>/dev/null | grep -q image_tools; then
    echo -e "${YELLOW}⚠️  image_tools not found, trying to install...${NC}"
    echo -e "${YELLOW}Run: sudo apt install ros-kilted-image-tools${NC}"
fi

# Check if rmw_zenoh_cpp is installed
if ros2 pkg list 2>/dev/null | grep -q rmw_zenoh_cpp; then
    echo -e "${GREEN}✓${NC} rmw_zenoh_cpp found"
else
    echo -e "${YELLOW}⚠️  Cannot verify rmw_zenoh_cpp (continuing anyway)${NC}"
fi

# Robot ID from environment or default
ROBOT_ID="${ROBOT_ID:-RBXU100001}"

echo ""
echo -e "${BLUE}🤖 Robot ID: ${ROBOT_ID}${NC}"
echo -e "${YELLOW}   Namespace: robots/${ROBOT_ID}${NC}"

# Generate Zenoh configuration with namespace
ZENOH_CONFIG_TEMPLATE="$PROJECT_ROOT/local_test/zenoh_client_config.json5"
ZENOH_CONFIG="/tmp/zenoh_oakd_viewer_${ROBOT_ID}.json5"

if [ ! -f "$ZENOH_CONFIG_TEMPLATE" ]; then
    echo -e "${RED}❌ Zenoh config template not found: $ZENOH_CONFIG_TEMPLATE${NC}"
    exit 1
fi

# Create config with namespace
cp "$ZENOH_CONFIG_TEMPLATE" "$ZENOH_CONFIG"

# Add namespace to the config
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
else
  sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
fi

echo -e "${GREEN}✓${NC} Zenoh config generated: $ZENOH_CONFIG"

# Check connectivity to Vision Pi
VISION_PI_IP="10.1.1.21"
VISION_PI_PORT="7447"

echo ""
echo -e "${YELLOW}🔍 Checking connectivity to Vision Pi (${VISION_PI_IP}:${VISION_PI_PORT})...${NC}"

if timeout 3 bash -c "echo > /dev/tcp/$VISION_PI_IP/$VISION_PI_PORT" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Vision Pi Zenoh router is reachable"
else
    echo -e "${RED}⚠️  Warning: Cannot reach Vision Pi Zenoh router${NC}"
    echo -e "${YELLOW}   Make sure Vision Pi is powered on and Zenoh router is running${NC}"
    echo -e "${YELLOW}   Continuing anyway...${NC}"
fi

# Set environment variables
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export RUST_LOG=zenoh=warn

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}🚀 Starting OAK-D Camera Viewer...${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}Environment:${NC}"
echo -e "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo -e "  ZENOH_CONFIG: ${ZENOH_CONFIG}"
echo -e "  ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE}"
echo ""
echo -e "${YELLOW}Camera topic:${NC}"
echo -e "  /oakd/rgb/image_raw/compressed (sensor_msgs/CompressedImage)"
echo ""
echo -e "${YELLOW}Connected to:${NC}"
echo -e "  Vision Pi: tcp/${VISION_PI_IP}:${VISION_PI_PORT}"
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Wait a moment for Zenoh to establish connection
echo -e "${YELLOW}⏳ Waiting for Zenoh connection...${NC}"
sleep 2

# Check if topic is available
echo -e "${YELLOW}🔍 Checking for OAK-D camera topic...${NC}"
if timeout 5 ros2 topic list 2>/dev/null | grep -q "/oakd/rgb/image_raw/compressed"; then
    echo -e "${GREEN}✓${NC} Camera topic found!"
else
    echo -e "${YELLOW}⚠️  Camera topic not found yet (might still be starting)${NC}"
    echo -e "${YELLOW}   Continuing anyway...${NC}"
fi

echo ""
echo -e "${GREEN}📹 Opening camera viewer window...${NC}"
echo -e "${YELLOW}   Press Ctrl+C to stop${NC}"
echo ""

# Launch rqt_image_view for compressed images
# Using rqt_image_view because it handles compressed images better than image_tools
exec ros2 run rqt_image_view rqt_image_view /oakd/rgb/image_raw/compressed
