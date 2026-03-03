#!/bin/bash
# Wrapper для запуска rtabmap с публикацией недостающих TF алиасов.
#
# Проблема: depthai_ros_driver v2 с i_rs_compat:true публикует изображения
# с frame_id=camera_color_optical_frame (RealSense naming), но URDF использует
# camera_rgb_camera_optical_frame. rtabmap не может найти трансформацию.
#
# Решение: публикуем identity TF camera_rgb_camera_optical_frame → camera_color_optical_frame
# прежде чем запустить rtabmap.

source /opt/ros/humble/setup.bash

echo "[start_rtabmap.sh] Starting static TF publisher: camera_rgb_camera_optical_frame → camera_color_optical_frame"

# ROS 2 Humble positional format: x y z qx qy qz qw parent_frame child_frame
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 \
    camera_rgb_camera_optical_frame \
    camera_color_optical_frame &

TF_PID=$!
echo "[start_rtabmap.sh] Static TF publisher PID: $TF_PID"

# Trap to kill TF publisher when main process exits
trap "kill $TF_PID 2>/dev/null" EXIT

echo "[start_rtabmap.sh] Launching: $@"
exec "$@"
