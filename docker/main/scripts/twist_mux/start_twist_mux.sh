#!/bin/bash
# twist_mux startup script
# Multiplexes velocity commands with prioritization

set -e

echo "═══════════════════════════════════════════════════════════"
echo "  🎮 Starting twist_mux (Velocity Command Multiplexer)"
echo "═══════════════════════════════════════════════════════════"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ERROR: ROS_DISTRO not set!${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS2 Distro: $ROS_DISTRO"

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
echo -e "${GREEN}✓${NC} ROS2 sourced"

# Check config file
CONFIG_FILE="/config/twist_mux/twist_mux.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ ERROR: Config file not found: $CONFIG_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Config file: $CONFIG_FILE"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🚀 Launching twist_mux"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Input topics (in priority order):"
echo "  1. /cmd_vel_emergency (priority: 255) - Emergency stop"
echo "  2. /cmd_vel_joy       (priority: 100) - Joystick/gamepad"
echo "  3. /cmd_vel_web       (priority:  50) - Web UI"
echo "  4. /cmd_vel           (priority:  10) - Nav2 navigation"
echo ""
echo "Output topic:"
echo "  → /diff_drive_controller/cmd_vel_unstamped (to diff_drive_controller)"
echo ""
echo "Logic:"
echo "  • Joystick active → blocks Nav2 (safety first!)"
echo "  • Joystick inactive → Nav2 can control"
echo "  • Emergency stop → blocks everything"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

# Launch twist_mux
exec ros2 run twist_mux twist_mux \
    --ros-args \
    --params-file $CONFIG_FILE \
    -r cmd_vel_out:=/diff_drive_controller/cmd_vel_unstamped \
    -p use_sim_time:=false
