#!/bin/bash
source /opt/ros/humble/setup.bash

# Устанавливаем переменные для оптимизации сжатия изображений
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=85

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

# AprilTag на потолочной камере — только по флагу (compose: CEILING_APRILTAG_ENABLED,
# по умолчанию false). Детектор идёт фоном: камера — главный сервис контейнера,
# падение детектора её не роняет (healthcheck смотрит на usb_cam).
if [ "${CEILING_APRILTAG_ENABLED:-false}" = "true" ]; then
  echo "CEILING_APRILTAG_ENABLED=true: запускаю ceiling_apriltag.launch.py"
  ros2 launch /config/ceiling-camera/launch/ceiling_apriltag.launch.py &
else
  echo "CEILING_APRILTAG_ENABLED=${CEILING_APRILTAG_ENABLED:-false}: AprilTag потолка выключен"
fi

# Запускаем USB Camera driver
exec ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  --params-file /config/ceiling-camera/camera_params.yaml \
  -r __ns:=/ceiling_camera
