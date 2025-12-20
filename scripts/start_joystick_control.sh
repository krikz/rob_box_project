#!/bin/bash

set -e

echo "🎮 Starting joystick control..."

# Stop existing instances
echo "   Stopping existing processes..."
pkill -f "joy_node" 2>/dev/null || true
pkill -f "teleop_node" 2>/dev/null || true
sleep 1

# Check if joystick device exists
if [ ! -e "/dev/input/js0" ]; then
    echo "❌ Error: Joystick device /dev/input/js0 not found"
    echo "   Please connect ExpressLRS Joystick via Bluetooth"
    exit 1
fi

# Setup Zenoh config
ROBOT_ID="${ROBOT_ID:-RBXU100001}"
ZENOH_CONFIG="/tmp/zenoh_rviz_config_${ROBOT_ID}.json5"

if [ ! -f "$ZENOH_CONFIG" ]; then
    echo "❌ Error: Zenoh config not found: $ZENOH_CONFIG"
    echo "   Please run start_rviz.sh first to generate config"
    exit 1
fi

echo "   Starting joy_node..."
(
    source /opt/ros/humble/setup.bash
    [ -f install/setup.bash ] && source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 run joy joy_node --ros-args -p device_id:=0 2>&1 | sed 's/^/[joy] /'
) &
JOY_PID=$!
sleep 2

echo "   Starting teleop_node..."
(
    source /opt/ros/humble/setup.bash
    [ -f install/setup.bash ] && source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 run teleop_twist_joy teleop_node --ros-args \
        -r cmd_vel:=cmd_vel_joy \
        -p axis_linear.x:=1 \
        -p axis_angular.yaw:=4 \
        -p scale_linear.x:=-1.5 \
        -p scale_angular.yaw:=4.0 \
        -p require_enable_button:=false \
        2>&1 | sed 's/^/[teleop] /'
) &
TELEOP_PID=$!
sleep 2

echo ""
echo "✅ Joystick control started successfully!"
echo ""
echo "📋 Control mapping:"
echo "   • Axis 1 (Left stick vertical)    → Linear speed (forward/backward)"
echo "   • Axis 3 (Right stick horizontal) → Angular speed (rotation)"
echo ""
echo "⚙️  Settings:"
echo "   • Max linear speed:  1.5 m/s"
echo "   • Max angular speed: 4.0 rad/s"
echo "   • Enable button: DISABLED (for testing)"
echo ""
echo "🛑 To stop: run './scripts/stop_joystick_control.sh'"
echo ""
echo "Processes running:"
echo "   joy_node:    PID $JOY_PID"
echo "   teleop_node: PID $TELEOP_PID"
