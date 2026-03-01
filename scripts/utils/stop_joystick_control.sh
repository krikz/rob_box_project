#!/bin/bash
# Stop joystick control for Rob Box robot

echo "🛑 Stopping joystick control..."

# Kill joy_node
JOY_PIDS=$(pgrep -f "joy_node" 2>/dev/null || true)
if [ -n "$JOY_PIDS" ]; then
    echo "   Stopping joy_node (PIDs: $JOY_PIDS)..."
    pkill -f "joy_node"
    sleep 1
else
    echo "   joy_node not running"
fi

# Kill teleop_node
TELEOP_PIDS=$(pgrep -f "teleop_node" 2>/dev/null || true)
if [ -n "$TELEOP_PIDS" ]; then
    echo "   Stopping teleop_node (PIDs: $TELEOP_PIDS)..."
    pkill -f "teleop_node"
    sleep 1
else
    echo "   teleop_node not running"
fi

# Verify all processes stopped
if pgrep -f "joy_node\|teleop_node" > /dev/null 2>&1; then
    echo "⚠️  Warning: Some processes still running, forcing kill..."
    pkill -9 -f "joy_node"
    pkill -9 -f "teleop_node"
    sleep 1
fi

# Final check
if pgrep -f "joy_node\|teleop_node" > /dev/null 2>&1; then
    echo "❌ Error: Failed to stop all processes"
    echo "   Remaining processes:"
    ps aux | grep -E "joy_node|teleop_node" | grep -v grep
    exit 1
fi

echo "✅ Joystick control stopped successfully!"
