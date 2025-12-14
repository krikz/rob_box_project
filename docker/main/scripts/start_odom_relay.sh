#!/bin/bash
# Relay для одометрии: /rtabmap/odom -> /odom
# Nav2 ожидает /odom, а RTABmap публикует в /rtabmap/odom

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

echo "==================================="
echo "Odometry Relay: /rtabmap/odom -> /odom"
echo "==================================="

# Ждём появления топика /rtabmap/odom
echo "Waiting for /rtabmap/odom topic..."
for i in {1..30}; do
    if ros2 topic list | grep -q "^/rtabmap/odom$"; then
        echo "✓ /rtabmap/odom topic found"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "ERROR: /rtabmap/odom topic not found after 30 seconds"
        exit 1
    fi
    sleep 1
done

echo "Starting odometry relay..."
exec ros2 run topic_tools relay /rtabmap/odom /odom
