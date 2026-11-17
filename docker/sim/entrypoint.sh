#!/bin/bash
# Точка входа контейнера симуляции: окружение ROS 2 + собранный /ws.
set -e
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash
exec "$@"
