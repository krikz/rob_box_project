#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
# Качество 75 для баланса между размером и качеством (было 85)
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=75

# Ждем пока устройство станет доступным
echo "Waiting for /dev/video0..."
for i in {1..10}; do
  if [ -e /dev/video0 ]; then
    echo "Found /dev/video0"
    break
  fi
  echo "Attempt $i: /dev/video0 not found, waiting..."
  sleep 1
done

if [ ! -e /dev/video0 ]; then
  echo "ERROR: /dev/video0 not found after 10 attempts"
  exit 1
fi

# Проверяем поддерживаемые форматы камеры
echo "Checking camera supported formats..."
if command -v v4l2-ctl &> /dev/null; then
  v4l2-ctl --list-formats-ext --device /dev/video0 | grep -i mjpeg && echo "MJPEG supported" || echo "MJPEG not found, will use YUYV"
fi

# Запускаем USB Camera driver
# При pixel_format: mjpeg камера публикует только /ceiling_camera/image_raw/compressed
# без декодирования, что экономит CPU Vision Pi
exec ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  --params-file /config/ceiling-camera/camera_params.yaml \
  -r __ns:=/ceiling_camera
