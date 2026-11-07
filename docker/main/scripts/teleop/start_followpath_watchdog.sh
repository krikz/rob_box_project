#!/bin/bash
# Сторож «сирот» FollowPath (#4458): гасит цель controller_server, если NavigateToPose уже завершён.
# Логика — rob_box_teleop.followpath_orphan. Сервис followpath-watchdog всегда в стеке main.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspace/install/setup.bash
# USE_SIM_TIME=true только в HIL/sim; на роботе пусто.
source /ros_scripts/use_sim_time.sh

echo "Starting followpath_watchdog_node (orphan_timeout_s=${ORPHAN_TIMEOUT_S:-1.5})..."
# Без `ros2 run`: нода сама PID 1 и выходит по SIGTERM (как proximity_guard_node).
exec python3 -m rob_box_teleop.followpath_watchdog_node --ros-args \
    "${SIM_TIME_ROS_ARGS[@]}" \
    -p orphan_timeout_s:="${ORPHAN_TIMEOUT_S:-1.5}" \
    --log-level info
