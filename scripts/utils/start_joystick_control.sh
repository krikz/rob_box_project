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
ZENOH_CONFIG="/tmp/zenoh_joystick_config_${ROBOT_ID}.json5"

# Detect Main or Vision Pi
if ip addr show | grep -q "10.1.1.10\|10.1.1.20"; then
    # Main Pi - connect to local peer
    ZENOH_ENDPOINT="tcp/10.1.1.10:7447"
    echo "   Detected Main Pi (peer mode)"
elif ip addr show | grep -q "10.1.1.11\|10.1.1.21"; then
    # Vision Pi - connect to local peer
    ZENOH_ENDPOINT="tcp/10.1.1.11:7447"
    echo "   Detected Vision Pi (peer mode)"
else
    # Development machine - connect to LOCAL Zenoh router
    ZENOH_ENDPOINT="tcp/10.1.1.249:7447"
    echo "   Dev machine mode (connecting to local Zenoh router at 10.1.1.249:7447)"
    
    # Check if local router is running
    if ! docker ps | grep -q zenoh-router-local; then
        echo "❌ Error: Local Zenoh router is not running"
        echo "   Start it with: docker start zenoh-router-local"
        exit 1
    fi
fi

# Create Zenoh config with namespace
echo "   Creating Zenoh config: $ZENOH_CONFIG"
cat > "$ZENOH_CONFIG" << EOF
{
  mode: "peer",
  namespace: "robots/${ROBOT_ID}",
  connect: {
    endpoints: ["${ZENOH_ENDPOINT}"]
  },
  scouting: {
    multicast: {
      enabled: false
    }
  }
}
EOF

echo "   Starting joy_node..."
(
    source /opt/ros/kilted/setup.bash
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
    source /opt/ros/kilted/setup.bash
    [ -f install/setup.bash ] && source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 run teleop_twist_joy teleop_node --ros-args \
        -r cmd_vel:=cmd_vel_joy \
        -p axis_linear.x:=1 \
        -p axis_angular.yaw:=4 \
        -p scale_linear.x:=-1.0 \
        -p scale_angular.yaw:=15.0 \
        -p require_enable_button:=true \
        -p enable_button:=0 \
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
