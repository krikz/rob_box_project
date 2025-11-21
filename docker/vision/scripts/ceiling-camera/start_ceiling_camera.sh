#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
# Качество 75 для баланса между размером и качеством (было 85)
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=75

# Ждем пока устройство станет доступным
# NOTE: /dev/video0 монтируется в docker-compose.yaml и указан в camera_params.yaml
VIDEO_DEVICE="/dev/video0"
echo "Waiting for $VIDEO_DEVICE..."
for i in {1..10}; do
  if [ -e "$VIDEO_DEVICE" ]; then
    echo "Found $VIDEO_DEVICE"
    break
  fi
  echo "Attempt $i: $VIDEO_DEVICE not found, waiting..."
  sleep 1
done

if [ ! -e "$VIDEO_DEVICE" ]; then
  echo "ERROR: $VIDEO_DEVICE not found after 10 attempts"
  exit 1
fi

# Проверяем поддерживаемые форматы камеры (информационно, для диагностики)
# usb_cam автоматически fallback на YUYV если MJPEG не поддерживается
echo "Checking camera supported formats..."
if command -v v4l2-ctl &> /dev/null; then
  v4l2-ctl --list-formats-ext --device "$VIDEO_DEVICE" | grep -i mjpeg && echo "MJPEG supported" || echo "MJPEG not found, will use YUYV"
fi

# Запускаем USB Camera driver
# При pixel_format: mjpeg камера публикует только /ceiling_camera/image_raw/compressed
# без декодирования, что экономит CPU Vision Pi
exec ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  --params-file /config/ceiling-camera/camera_params.yaml \
  -r __ns:=/ceiling_camera
