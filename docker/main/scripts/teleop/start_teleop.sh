#!/bin/bash
set -e

echo "🎮 Starting Teleoperation Service..."

# Source ROS 2 и workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspace/install/setup.bash

# Load parameters
PARAMS_FILE="/config/teleop/joystick_params.yaml"
if [ -f "$PARAMS_FILE" ]; then
    echo "   Loading parameters from: $PARAMS_FILE"
    
    # Check if BLE mode is enabled
    USE_BLE=$(grep -A 1 "use_ble:" "$PARAMS_FILE" | grep "true" || echo "")
    
    if [ -n "$USE_BLE" ]; then
        echo "📡 BLE Direct mode enabled - joystick_control_node will connect via Bluetooth"
        echo "   No joy_node needed (bypassing HID subsystem)"
    else
        echo "🎮 HID mode - will use joy_linux_node with /dev/input/event* devices"
    fi
else
    echo "⚠️  Warning: Config file not found: $PARAMS_FILE"
    PARAMS_FILE=""
    USE_BLE=""
fi

# Start joy_node ONLY if NOT using BLE mode
if [ -z "$USE_BLE" ]; then
    echo "🚀 Starting joy_linux_node..."
    
    # Wait for joystick device (with timeout)
    echo "   Waiting for joystick device /dev/input/js0..."
    TIMEOUT=30
    ELAPSED=0
    while [ ! -e /dev/input/js0 ] && [ $ELAPSED -lt $TIMEOUT ]; do
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done

    if [ ! -e /dev/input/js0 ]; then
        echo "⚠️  Warning: Joystick not found after ${TIMEOUT}s"
    else
        echo "✅ Joystick device found: /dev/input/js0"
    fi

    # Start joy_node (reads /dev/input/event* and publishes to /joy topic)
    ros2 run joy joy_linux_node --ros-args \
        --log-level info \
        -p device_name:="ExpressLRS Joystick" \
        -p deadzone:=0.05 \
        -p autorepeat_rate:=20.0 &

    JOY_PID=$!
    echo "   joy_linux_node started (PID: $JOY_PID)"
    sleep 2
fi

echo "🚀 Starting joystick_control_node..."

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

# Wait for process to exit
if [ -n "$USE_BLE" ]; then
    # BLE mode - only wait for control node
    wait $CONTROL_PID
else
    # HID mode - wait for either process
    wait -n
    # If one exits, kill the other
    echo "❌ One process exited, stopping all..."
    kill $JOY_PID $CONTROL_PID 2>/dev/null || true
fi

exit 1
