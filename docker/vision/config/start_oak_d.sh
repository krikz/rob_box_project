#!/bin/bash
source /opt/ros/jazzy/setup.bash
# Запускаем OAK-D с использованием Zenoh
exec ros2 launch depthai_ros_driver camera.launch.py \
  rs_compat:=true \
  rgb_camera.color_profile:=1280,720,10 \
  depth_module.depth_profile:=1280,720,10 \
  depth_module.infra_profile:=1280,720,10