#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D с оптимизацией для Raspberry Pi
exec ros2 launch depthai_ros_driver camera.launch.py \
  params_file:=/config/oak_d_config.yaml \
  rs_compat:=true