#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Генерируем URDF из xacro
echo "Generating URDF from xacro..."
xacro /workspace/src/rob_box_description/urdf/rob_box.xacro > /tmp/rob_box.urdf

# Запускаем robot_state_publisher
echo "Starting robot_state_publisher..."
exec ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat /tmp/rob_box.urdf)" \
    -p use_sim_time:=false \
    -p frame_prefix:=""
