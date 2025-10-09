#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Генерируем URDF из xacro
echo "Generating URDF from xacro..."
xacro /workspace/src/rob_box_description/urdf/rob_box.xacro > /tmp/rob_box.urdf

# Проверяем, что URDF создан
if [ ! -s /tmp/rob_box.urdf ]; then
    echo "ERROR: Failed to generate URDF!"
    exit 1
fi

echo "URDF generated successfully ($(wc -l < /tmp/rob_box.urdf) lines)"

# Запускаем robot_state_publisher с URDF из файла
echo "Starting robot_state_publisher..."
exec ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p use_sim_time:=false \
    -p robot_description:="$(cat /tmp/rob_box.urdf)"
