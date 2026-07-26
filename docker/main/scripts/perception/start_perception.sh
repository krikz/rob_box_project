#!/bin/bash
# Start script for Internal Dialogue Agent in Docker

set -e

echo "=========================================="
echo "🧠 Internal Dialogue Agent"
echo "=========================================="
echo ""

# Source ROS2 environment
source /opt/ros/kilted/setup.bash

# Source workspace если есть
if [ -f /ws/install/setup.bash ]; then
    echo "✓ Sourcing workspace /ws/install/setup.bash"
    source /ws/install/setup.bash
else
    echo "⚠️  Workspace not found at /ws/install/setup.bash"
fi

echo ""
echo "Environment:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  REFLECTION_RATE: ${REFLECTION_RATE:-1.0} Hz"
echo "  DIALOGUE_TIMEOUT: ${DIALOGUE_TIMEOUT:-10.0}s"
echo "  ENABLE_SPEECH: ${ENABLE_SPEECH:-true}"
echo ""

# Проверка DeepSeek API Key
if [ -z "$DEEPSEEK_API_KEY" ]; then
    echo "⚠️  WARNING: DEEPSEEK_API_KEY not set!"
    echo "   Using stub mode (no real AI)"
    echo ""
else
    echo "✓ DEEPSEEK_API_KEY configured"
    echo ""
fi

echo "Starting Internal Dialogue Agent..."
echo "=========================================="
echo ""

# Запуск launch файла
exec ros2 launch rob_box_perception internal_dialogue_docker.launch.py
