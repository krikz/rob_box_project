#!/bin/bash
source /opt/ros/humble/setup.bash

# Запускаем AprilTag с использованием Zenoh
# Используем /camera/rgb/image_raw для детекции маркеров
exec ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/rgb/image_raw \
  -r camera_info:=/camera/rgb/camera_info \
  --params-file /config/apriltag/apriltag_config.yaml