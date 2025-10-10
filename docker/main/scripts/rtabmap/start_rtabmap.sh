#!/bin/bash
set -e

echo "Starting RTAB-Map in LaserScan-only mode..."

# Source ROS 2
source /opt/ros/humble/setup.bash

# Ждем Zenoh router
echo "Waiting for Zenoh router..."
sleep 5

# Запускаем RTAB-Map с явными параметрами (БЕЗ launch файла)
echo "Launching rtabmap node with LaserScan only..."
exec ros2 run rtabmap_slam rtabmap \
  --ros-args \
  -r odom:=odom \
  -r scan:=/scan \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p subscribe_depth:=false \
  -p subscribe_rgb:=false \
  -p subscribe_rgbd:=false \
  -p subscribe_stereo:=false \
  -p subscribe_scan:=true \
  -p queue_size:=10 \
  -p Reg/Strategy:=1 \
  -p Reg/Force3DoF:=true \
  -p RGBD/Enabled:=false \
  -p Grid/FromDepth:=false \
  -p database_path:=/maps/rtabmap.db
