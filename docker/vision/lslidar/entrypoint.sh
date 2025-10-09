#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Настройка Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG=/config/zenoh_session_config.json5
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Ожидание запуска Zenoh router
echo "Waiting for Zenoh router to be ready..."
ATTEMPTS=${ZENOH_ROUTER_CHECK_ATTEMPTS:-10}
for i in $(seq 1 $ATTEMPTS); do
    if wget -qO- http://localhost:8000/@/local/router > /dev/null 2>&1; then
        echo "Zenoh router is ready!"
        break
    fi
    if [ $i -eq $ATTEMPTS ]; then
        echo "WARNING: Zenoh router not responding after $ATTEMPTS attempts, starting anyway..."
    fi
    echo "Attempt $i/$ATTEMPTS: Zenoh router not ready, waiting..."
    sleep 2
done

echo "Starting LSLIDAR N10 driver..."

# Запуск LSLIDAR драйвера
# LiDAR должен быть в той же подсети 10.1.1.x
exec ros2 launch lslidar_driver lslidar_n10.launch.py \
    frame_id:=lslidar_n10 \
    scan_topic:=/scan \
    device_ip:=10.1.1.200 \
    lidar_udp_port:=2368
