#!/bin/bash
set -e

echo "🎮 Starting Teleoperation Service..."

# Source ROS 2 и workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspace/install/setup.bash

# Wait for joystick device (with timeout)
echo "   Waiting for joystick device /dev/input/js0..."
TIMEOUT=30
ELAPSED=0
while [ ! -e /dev/input/js0 ] && [ $ELAPSED -lt $TIMEOUT ]; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ ! -e /dev/input/js0 ]; then
    echo "⚠️  Warning: Joystick not found after ${TIMEOUT}s, starting anyway..."
else
    echo "✅ Joystick device found: /dev/input/js0"
    # Show device info
    jstest --normal /dev/input/js0 > /dev/null 2>&1 || echo "   (jstest check skipped)"
fi

# Load parameters
PARAMS_FILE="/config/teleop/joystick_params.yaml"
if [ -f "$PARAMS_FILE" ]; then
    echo "   Loading parameters from: $PARAMS_FILE"
else
    echo "⚠️  Warning: Config file not found: $PARAMS_FILE"
    PARAMS_FILE=""
fi

echo "🚀 Starting joy_node and joystick_control_node..."

# Start joy_node (reads /dev/input/event* and publishes to /joy topic)
# Using device_name to auto-detect ExpressLRS Joystick
ros2 run joy joy_linux_node --ros-args \
    --log-level info \
    -p device_name:="ExpressLRS Joystick" \
    -p deadzone:=0.05 \
    -p autorepeat_rate:=20.0 &

JOY_PID=$!
echo "   joy_node started (PID: $JOY_PID)"

# Small delay to let joy_node initialize
sleep 2

# Start joystick_control_node
if [ -n "$PARAMS_FILE" ]; then
    ros2 run rob_box_teleop joystick_control_node --ros-args \
        --params-file "$PARAMS_FILE" \
        --log-level info &
else
    ros2 run rob_box_teleop joystick_control_node --ros-args \
        --log-level info &
fi

CONTROL_PID=$!
echo "   joystick_control_node started (PID: $CONTROL_PID)"

# Wait for any process to exit
wait -n

# If one exits, kill the other
echo "❌ One process exited, stopping all..."
kill $JOY_PID $CONTROL_PID 2>/dev/null || true
exit 1
