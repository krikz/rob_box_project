#!/bin/bash
# Nav2 Direct Launch - запуск нодов напрямую без navigation_launch.py

set -e

echo "Starting Nav2 nodes directly..."

# Source ROS2
source /opt/ros/humble/setup.bash

PARAMS="/config/nav2/nav2_params.yaml"

# Запуск lifecycle manager в фоне
ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=lifecycle_manager_navigation &

sleep 2

# Запуск остальных нод
ros2 run nav2_controller controller_server \
    --ros-args \
    --params-file $PARAMS \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_planner planner_server \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=planner_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_bt_navigator bt_navigator \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=bt_navigator \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_behaviors behavior_server \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=behavior_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_waypoint_follower waypoint_follower \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=waypoint_follower \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_velocity_smoother velocity_smoother \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=velocity_smoother \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

ros2 run nav2_smoother smoother_server \
    --ros-args \
    --params-file $PARAMS \
    -r __node:=smoother_server \
    -r /tf:=tf \
    -r /tf_static:=tf_static &

# Держим скрипт живым
wait
