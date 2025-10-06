#!/bin/bash
source /opt/ros/kilted/setup.bash

# Запускаем AprilTag с использованием Zenoh
exec ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_rect \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file /config/apriltag_config.yaml