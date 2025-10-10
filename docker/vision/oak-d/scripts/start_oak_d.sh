#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D только для публикации изображений для AprilTag
# Используем python3 напрямую, т.к. ros2 launch не поддерживает произвольные пути
exec python3 /oak-d/launch/oakd_apriltag_only.launch.py config_file:=/config/oak-d/oak_d_config.yamlash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3

# Запускаем OAK-D только для публикации изображений для AprilTag
exec ros2 launch oak-d/launch/oakd_apriltag_only.launch.py