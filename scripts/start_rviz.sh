#!/bin/bash
# Start RViz with Zenoh RMW to visualize rob_box robot
# This script connects to Main Pi (10.1.1.10) via Zenoh

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
echo -e "${BLUE}  RViz Launcher for Rob Box Robot${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 is not sourced!${NC}"
    echo -e "${YELLOW}Run: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS 2 Distro: ${ROS_DISTRO}"

# Source workspace if available (for URDF meshes and custom packages)
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo -e "${GREEN}✓${NC} Workspace sourced (rob_box packages available)"
else
    echo -e "${YELLOW}⚠️  Workspace not built (meshes may not load)${NC}"
    echo -e "${YELLOW}   Build with: cd $PROJECT_ROOT && colcon build${NC}"
fi

# Check if RViz is installed
if ! command -v rviz2 &> /dev/null; then
    echo -e "${RED}❌ RViz2 is not installed!${NC}"
    echo -e "${YELLOW}Install: sudo apt install ros-humble-rviz2${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} RViz2 found"

# Check if rmw_zenoh_cpp is installed (skip if ros2 pkg list fails)
if command -v ros2 &> /dev/null; then
    if ros2 pkg list 2>/dev/null | grep -q rmw_zenoh_cpp; then
        echo -e "${GREEN}✓${NC} rmw_zenoh_cpp found"
    else
        echo -e "${YELLOW}⚠️  Cannot verify rmw_zenoh_cpp (continuing anyway)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  ros2 command not available${NC}"
fi

echo -e "${GREEN}✓${NC} ROS Distro: ${ROS_DISTRO} (Humble recommended)"

# Robot ID from environment or default
ROBOT_ID="${ROBOT_ID:-RBXU100001}"

echo ""
echo -e "${BLUE}🤖 Robot ID: ${ROBOT_ID}${NC}"
echo -e "${YELLOW}   Namespace: robots/${ROBOT_ID}${NC}"

# Generate Zenoh configuration with namespace
ZENOH_CONFIG_TEMPLATE="$PROJECT_ROOT/local_test/zenoh_client_config.json5"
ZENOH_CONFIG="/tmp/zenoh_rviz_config_${ROBOT_ID}.json5"

if [ ! -f "$ZENOH_CONFIG_TEMPLATE" ]; then
    echo -e "${RED}❌ Zenoh config template not found: $ZENOH_CONFIG_TEMPLATE${NC}"
    exit 1
fi

# Create config with namespace
cp "$ZENOH_CONFIG_TEMPLATE" "$ZENOH_CONFIG"

# Add namespace to the config (insert after mode line)
# Support both "client" and "peer" modes
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
else
  sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
fi

echo -e "${GREEN}✓${NC} Zenoh config generated: $ZENOH_CONFIG"

# Check connectivity to Main Pi
MAIN_PI_IP="10.1.1.10"
MAIN_PI_PORT="7447"

echo ""
echo -e "${YELLOW}🔍 Checking connectivity to Main Pi (${MAIN_PI_IP}:${MAIN_PI_PORT})...${NC}"

if timeout 3 bash -c "echo > /dev/tcp/$MAIN_PI_IP/$MAIN_PI_PORT" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Main Pi Zenoh router is reachable"
else
    echo -e "${RED}⚠️  Warning: Cannot reach Main Pi Zenoh router${NC}"
    echo -e "${YELLOW}   Make sure Main Pi is powered on and Zenoh router is running${NC}"
    echo -e "${YELLOW}   Continuing anyway...${NC}"
fi

# Set environment variables
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG="$ZENOH_CONFIG"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export RUST_LOG=zenoh=warn
export LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

# Optional: Set ROS_DOMAIN_ID if needed
# export ROS_DOMAIN_ID=0

# CRITICAL FIX: Prioritize ROS Ogre vendor libraries over system libraries
# RViz requires Ogre 1.12.1 which is bundled with ROS, not system 1.12.10
export LD_LIBRARY_PATH="/opt/ros/humble/opt/rviz_ogre_vendor/lib:${LD_LIBRARY_PATH}"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}🚀 Starting RViz2...${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}Environment:${NC}"
echo -e "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo -e "  ZENOH_CONFIG: ${ZENOH_CONFIG}"
echo -e "  ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE}"
echo ""
echo -e "${YELLOW}Connected to:${NC}"
echo -e "  Main Pi: tcp/${MAIN_PI_IP}:${MAIN_PI_PORT}"
echo -e "  Vision Pi: tcp/10.1.1.21:7447 (fallback)"
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Check if custom RViz config exists
RVIZ_CONFIG="$PROJECT_ROOT/src/rob_box_bringup/config/rviz_config.rviz"

if [ -f "$RVIZ_CONFIG" ]; then
    echo -e "${GREEN}✓${NC} Using custom RViz config: $RVIZ_CONFIG"
    exec rviz2 -d "$RVIZ_CONFIG"
else
    echo -e "${YELLOW}⚠️  Custom RViz config not found, using default${NC}"
    exec rviz2
fi
