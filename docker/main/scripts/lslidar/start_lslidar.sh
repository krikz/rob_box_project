#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/lyrical/setup.bash
source /ws/install/setup.bash

# Zenoh настраивается через ZENOH_CONFIG_OVERRIDE в docker-compose;
# RMW сам ждёт router через ZENOH_ROUTER_CHECK_ATTEMPTS.

echo "Starting LSLIDAR N10 driver..."

# Запуск LSLIDAR драйвера (headless режим, без RViz2)
# LiDAR подключен к Vision Pi через USB/Serial
# Используем кастомный headless launch файл без RViz2
exec ros2 launch /config/lslidar/lslidar_headless_launch.py
