#!/bin/bash
# Зона безопасности вокруг корпуса (#4365): мигалки и сирена в движении, аварийный стоп
# через cmd_vel_emergency (twist_mux priority 255). Логика — rob_box_teleop.proximity.
# Сервис proximity-guard всегда в стеке main (без профиля, решение Шифу 05.11).
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspace/install/setup.bash
# USE_SIM_TIME=true только в профиле docker/sim (Gazebo); на роботе пусто.
source /ros_scripts/use_sim_time.sh

echo "🛡️ Starting proximity_guard_node (footprint: /config/nav2/nav2_params.yaml)..."
# Без `ros2 run`: он становится PID 1 и SIGTERM от `docker stop` ноде не пересылает — контейнер добивался
# SIGKILL через 10 с (Exited 137, HIL 05.11, #4365). Теперь нода сама PID 1 и выходит по SIGTERM.
exec python3 -m rob_box_teleop.proximity_guard_node --ros-args \
    "${SIM_TIME_ROS_ARGS[@]}" \
    --log-level info
