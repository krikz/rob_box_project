#!/bin/bash
source /opt/ros/jazzy/setup.bash
# Запускаем OAK-D с использованием Zenoh
exec ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=/config/oak_d_config.yaml \
  rs_compat:=true