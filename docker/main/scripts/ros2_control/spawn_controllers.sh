#!/bin/bash
# Spawn Controllers for ros2_control
# Загружает и активирует контроллеры для дифференциального привода

set -e

echo "Waiting for controller_manager to be ready..."
sleep 5

# Source ROS2
source /opt/ros/humble/setup.bash

echo "Loading and activating controllers..."

# 1. Joint State Broadcaster - публикует /joint_states
echo "Spawning joint_state_broadcaster..."
ros2 run controller_manager spawner joint_state_broadcaster

# 2. Differential Drive Controller - управляет колесами
echo "Spawning diff_drive_controller..."
ros2 run controller_manager spawner diff_drive_controller

echo "✅ All controllers spawned and activated!"

# Проверяем статус
echo ""
echo "Controller status:"
ros2 control list_controllers

echo ""
echo "Controllers are ready to receive commands on /cmd_vel"
