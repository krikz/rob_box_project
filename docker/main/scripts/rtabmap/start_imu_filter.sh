#!/bin/bash
# imu_filter_madgwick перед rtabmap (сервис imu-filter, решение Шифу 04.11).
#
#   IMU_FILTER_INPUT   сырой IMU (по умолчанию в compose — /camera/camera/imu, OAK-D)
#   RTABMAP_IMU_TOPIC  выход фильтра = вход rtabmap (по умолчанию /imu/data);
#                      пусто → IMU выключен: узел не стартует, контейнер спит,
#                      чтобы restart: unless-stopped не крутил его по кругу.

source /opt/ros/humble/setup.bash
# USE_SIM_TIME=true только в профиле docker/sim (Gazebo); на роботе false.
source /ros_scripts/use_sim_time.sh

PARAMS=/config/rtabmap/imu_filter_madgwick.yaml

if [ -z "${RTABMAP_IMU_TOPIC:-}" ]; then
    echo "[start_imu_filter.sh] RTABMAP_IMU_TOPIC пуст — IMU выключен, фильтр не запускается"
    exec sleep infinity
fi
if [ -z "${IMU_FILTER_INPUT:-}" ]; then
    echo "[start_imu_filter.sh] ❌ IMU_FILTER_INPUT пуст — нечего фильтровать" >&2
    exit 1
fi

echo "[start_imu_filter.sh] $IMU_FILTER_INPUT → imu_filter_madgwick → $RTABMAP_IMU_TOPIC"
exec ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    --params-file "$PARAMS" \
    -r imu/data_raw:="$IMU_FILTER_INPUT" \
    -r imu/data:="$RTABMAP_IMU_TOPIC" \
    "${SIM_TIME_ROS_ARGS[@]}"
