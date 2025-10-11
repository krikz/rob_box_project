#!/bin/bash
# Nav2 Navigation Stack Startup Script
# Запускает автономную навигацию rob_box

set -e

echo "═══════════════════════════════════════════════════════════"
echo "  🧭 Starting Nav2 Navigation Stack"
echo "═══════════════════════════════════════════════════════════"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Проверка ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ERROR: ROS_DISTRO not set!${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS2 Distro: $ROS_DISTRO"

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
echo -e "${GREEN}✓${NC} ROS2 sourced"

# Проверка конфигурации
MAIN_CONFIG="/config/nav2/nav2_params.yaml"
LOCAL_COSTMAP="/config/nav2/local_costmap_params.yaml"
GLOBAL_COSTMAP="/config/nav2/global_costmap_params.yaml"

if [ ! -f "$MAIN_CONFIG" ]; then
    echo -e "${RED}❌ ERROR: Main config not found: $MAIN_CONFIG${NC}"
    exit 1
fi

if [ ! -f "$LOCAL_COSTMAP" ]; then
    echo -e "${RED}❌ ERROR: Local costmap config not found: $LOCAL_COSTMAP${NC}"
    exit 1
fi

if [ ! -f "$GLOBAL_COSTMAP" ]; then
    echo -e "${RED}❌ ERROR: Global costmap config not found: $GLOBAL_COSTMAP${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Configuration files validated"

# Проверка behavior trees
BT_DIR="/config/nav2/behavior_trees"
if [ ! -d "$BT_DIR" ]; then
    echo -e "${YELLOW}⚠${NC}  Warning: Behavior trees directory not found: $BT_DIR"
    echo "   Using default Nav2 behavior trees"
else
    echo -e "${GREEN}✓${NC} Behavior trees directory: $BT_DIR"
fi

# Ожидание robot_state_publisher
echo ""
echo "Waiting for /robot_description topic..."
TIMEOUT=30
ELAPSED=0
while ! ros2 topic list | grep -q "/robot_description"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${RED}❌ ERROR: /robot_description topic not found after ${TIMEOUT}s${NC}"
        echo "   Make sure robot_state_publisher is running"
        exit 1
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done
echo ""
echo -e "${GREEN}✓${NC} /robot_description topic found"

# Ожидание /odom топика (от ros2_control)
echo ""
echo "Waiting for /odom topic..."
ELAPSED=0
while ! ros2 topic list | grep -q "/odom"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}⚠${NC}  Warning: /odom topic not found after ${TIMEOUT}s"
        echo "   Nav2 will wait for odometry at runtime"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done

if ros2 topic list | grep -q "/odom"; then
    echo ""
    echo -e "${GREEN}✓${NC} /odom topic found"
fi

# Ожидание /scan топика (от lslidar)
echo ""
echo "Waiting for /scan topic..."
ELAPSED=0
while ! ros2 topic list | grep -q "/scan"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}⚠${NC}  Warning: /scan topic not found after ${TIMEOUT}s"
        echo "   Nav2 will wait for laser scan at runtime"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done

if ros2 topic list | grep -q "/scan"; then
    echo ""
    echo -e "${GREEN}✓${NC} /scan topic found"
fi

# Ожидание /map топика (от rtabmap или map_server)
echo ""
echo "Waiting for /map topic..."
ELAPSED=0
while ! ros2 topic list | grep -q "/map"; do
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo -e "${YELLOW}⚠${NC}  Warning: /map topic not found after ${TIMEOUT}s"
        echo "   Nav2 will use local costmap only (no global planning)"
        echo "   Make sure RTAB-Map or map_server is running to provide /map"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done

if ros2 topic list | grep -q "/map"; then
    echo ""
    echo -e "${GREEN}✓${NC} /map topic found"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🚀 Launching Nav2 Navigation Stack"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Configuration:"
echo "  • Main config: $MAIN_CONFIG"
echo "  • Local costmap: $LOCAL_COSTMAP"
echo "  • Global costmap: $GLOBAL_COSTMAP"
echo "  • Behavior trees: $BT_DIR"
echo ""
echo "Expected topics:"
echo "  • /robot_description (robot model)"
echo "  • /odom (odometry from ros2_control)"
echo "  • /scan (laser scan from lslidar)"
echo "  • /map (SLAM map from rtabmap)"
echo "  • /cmd_vel (output - velocity commands)"
echo ""
echo "Nav2 nodes that will start:"
echo "  • controller_server (path following)"
echo "  • planner_server (path planning)"
echo "  • behavior_server (recovery behaviors)"
echo "  • bt_navigator (behavior tree execution)"
echo "  • waypoint_follower (waypoint navigation)"
echo "  • velocity_smoother (velocity smoothing)"
echo "  • lifecycle_manager (node management)"
echo ""
echo "To send navigation goals:"
echo "  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped ..."
echo ""
echo "To monitor navigation:"
echo "  ros2 topic echo /cmd_vel"
echo "  ros2 node list"
echo "  ros2 topic list"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

# Запуск Nav2 через bringup launch
exec ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$MAIN_CONFIG \
    use_sim_time:=False \
    autostart:=True \
    use_lifecycle_mgr:=True \
    use_respawn:=True
