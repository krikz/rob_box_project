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
    
    # Check if SBUS mode is enabled
    USE_SBUS=$(grep -A 1 "use_sbus:" "$PARAMS_FILE" | grep "true" || echo "")
    SERIAL_PORT=$(grep "serial_port:" "$PARAMS_FILE" | awk '{print $2}' | tr -d '"' || echo "")
    
    if [ -n "$USE_SBUS" ] && [ -n "$SERIAL_PORT" ]; then
        echo "📡 SBUS Serial mode enabled - configured port: $SERIAL_PORT"
        echo "   No joy_node needed"
        
        # Auto-detect serial port if configured one not found
        # Look for any ttyACM device (CH340G USB-UART adapter)
        if [ ! -e "$SERIAL_PORT" ]; then
            echo "⚠️  Configured port $SERIAL_PORT not found, auto-detecting..."
            # Try all ttyACM devices
            FOUND_PORT=""
            for port in /dev/ttyACM*; do
                if [ -e "$port" ]; then
                    echo "   Found serial port: $port"
                    FOUND_PORT="$port"
                fi
            done
            
            if [ -n "$FOUND_PORT" ]; then
                echo "✅ Using auto-detected port: $FOUND_PORT"
                SERIAL_PORT="$FOUND_PORT"
                # Override in params via command line (will be passed to node)
                SERIAL_PORT_OVERRIDE="$FOUND_PORT"
            else
                echo "❌ No ttyACM devices found! Waiting 30s..."
                TIMEOUT=30
                ELAPSED=0
                while [ $ELAPSED -lt $TIMEOUT ]; do
                    for port in /dev/ttyACM*; do
                        if [ -e "$port" ]; then
                            SERIAL_PORT="$port"
                            SERIAL_PORT_OVERRIDE="$port"
                            echo "✅ Found port after ${ELAPSED}s: $port"
                            break 2
                        fi
                    done
                    sleep 1
                    ELAPSED=$((ELAPSED + 1))
                done
            fi
        else
            echo "✅ Serial port found: $SERIAL_PORT"
        fi
    else
        echo "🎮 HID mode - will use joy_linux_node with /dev/input/event* devices"
    fi
else
    echo "⚠️  Warning: Config file not found: $PARAMS_FILE"
    PARAMS_FILE=""
    USE_SBUS=""
fi

# Start joy_node ONLY if NOT using SBUS mode
if [ -z "$USE_SBUS" ]; then
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

# Start joystick_control_node.
# Entry point name (from src/rob_box_teleop/setup.py) is `joystick_control_node`
# — `ros2 run` resolves it via the ament index, NOT the source filename.
# Adding `.py` here (as a previous fix attempted for `--symlink-install`) breaks
# lookup with "No executable found" and the container exits into a restart loop.
if [ -n "$PARAMS_FILE" ]; then
    # Add serial port override if port was auto-detected
    EXTRA_ARGS=""
    if [ -n "$SERIAL_PORT_OVERRIDE" ]; then
        EXTRA_ARGS="-p serial_port:=$SERIAL_PORT_OVERRIDE"
        echo "   Overriding serial_port to: $SERIAL_PORT_OVERRIDE"
    fi
    ros2 run rob_box_teleop joystick_control_node --ros-args \
        --params-file "$PARAMS_FILE" \
        $EXTRA_ARGS \
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
