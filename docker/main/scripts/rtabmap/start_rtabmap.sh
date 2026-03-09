#!/bin/bash
# Wrapper для запуска rtabmap с публикацией недостающих TF алиасов.
#
# Проблема: depthai_ros_driver v2 с i_rs_compat:true публикует изображения
# с frame_id=camera_color_optical_frame (RealSense naming), но URDF использует
# camera_rgb_camera_optical_frame. rtabmap не может найти трансформацию.
#
# Решение: публикуем identity TF алиасы для legacy RealSense-style frame names
# прежде чем запустить rtabmap и nav2.

source /opt/ros/humble/setup.bash

# ── Mapping Lifecycle: always start in localization mode ───────────────────
# rtabmap ВСЕГДА стартует в localization:=true.
#
# Причина удаления state file: mapping_state.json на Main Pi — отдельный volume
# от Vision Pi. MCP tools пишут файл на Vision Pi, а start_rtabmap.sh читал
# другой файл на Main Pi — стейт расходился и вызывал краши.
#
# Переключение в mapping mode происходит через ROS2 сервис set_mode_mapping,
# который вызывает StartMappingTool после старта rtabmap. Это надёжнее чем
# читать stale файл при старте контейнера.
echo "[start_rtabmap.sh] Starting in localization mode (always safe default)"

# Patch rtabmap.launch.py to inject Grid/Sensor=2 as explicit ROS2 parameter.
# Required because rtabmap auto-resets Grid/Sensor=0 when subscribe_scan=true,
# unless the parameter is explicitly present in the Node's parameters dict.
python3 /ros_scripts/patch_rtabmap_launch.py || echo "[start_rtabmap.sh] WARN: patch_rtabmap_launch.py failed, Grid/Sensor may be 0"

echo "[start_rtabmap.sh] Starting static TF publisher: camera_rgb_camera_optical_frame → camera_color_optical_frame"

# ROS 2 Humble positional format: x y z qx qy qz qw parent_frame child_frame
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 \
    camera_rgb_camera_optical_frame \
    camera_color_optical_frame &

TF_PID=$!
echo "[start_rtabmap.sh] Static TF publisher PID: $TF_PID"

echo "[start_rtabmap.sh] Starting static TF publisher: camera_stereo_camera_frame → camera_depth_frame"

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 \
    camera_stereo_camera_frame \
    camera_depth_frame &

DEPTH_TF_PID=$!
echo "[start_rtabmap.sh] Depth TF publisher PID: $DEPTH_TF_PID"

# Trap to kill TF publisher when main process exits
trap "kill $TF_PID $DEPTH_TF_PID 2>/dev/null" EXIT

echo "[start_rtabmap.sh] Launching with localization:=true: $@"
exec "$@" "localization:=true"