#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D с интегрированной детекцией AprilTag
# Это объединяет камеру и AprilTag детектор в одном контейнере
# для снижения нагрузки на Vision Pi
cd /oak-d/launch
exec ros2 launch oakd_with_apriltag.launch.py
