#!/bin/bash
# Wrapper для запуска LiDAR-only rtabmap на Main Pi.

source /opt/ros/humble/setup.bash

# USE_SIM_TIME=true только в профиле docker/sim (Gazebo): use_sim_time:=true
# дописывается ПОСЛЕ аргументов compose и перекрывает их use_sim_time:=false.
# На роботе массив пустой.
source /ros_scripts/use_sim_time.sh

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

# Patch rtabmap.launch.py to inject explicit LiDAR-only ROS2 parameters.
python3 /scripts/patch_rtabmap_launch.py || echo "[start_rtabmap.sh] WARN: patch_rtabmap_launch.py failed"

# ── IMU (решение Шифу 04.11) ────────────────────────────────────────────────
# RTABMAP_IMU_TOPIC задаёт docker-compose.yaml: по умолчанию /imu/data — выход
# imu_filter_madgwick (сервис imu-filter). Пусто → IMU выключен, как до 04.11:
# мёртвое имя /camera/imu/data (его никто не публикует). Пустой imu_topic:=
# в launch не передаём — пустой remap роняет узел.
IMU_TOPIC="${RTABMAP_IMU_TOPIC:-/camera/imu/data}"
if [ -n "${RTABMAP_IMU_TOPIC:-}" ]; then
    echo "[start_rtabmap.sh] IMU on: imu_topic:=$IMU_TOPIC"
else
    echo "[start_rtabmap.sh] IMU off (RTABMAP_IMU_TOPIC пуст): imu_topic:=$IMU_TOPIC"
fi

echo "[start_rtabmap.sh] Launching with localization:=true: $@"
exec "$@" "imu_topic:=$IMU_TOPIC" "localization:=true" "${SIM_TIME_LAUNCH_ARGS[@]}"