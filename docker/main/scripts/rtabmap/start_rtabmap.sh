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
# RTABMAP_IMU_TOPIC задаёт docker-compose.yaml: по умолчанию пусто (IMU выключен);
# включение — RTABMAP_IMU_TOPIC=/imu/data (выход imu_filter_madgwick, сервис imu-filter).
#   * включено: wait_imu_to_init:=true — без него icp_odometry на IMU даже не
#     подписывается (замер в симе 04.11: подписчик /imu/data только /rtabmap/rtabmap,
#     а там IMU нужен лишь для гравитации, и та выключена Optimizer/GravitySigma=0).
#     Цена: пока нет ни одного сообщения IMU, ICP-одометрия не стартует.
#   * пусто (по умолчанию): IMU выключен, как до 04.11 — мёртвое имя /camera/imu/data и
#     wait_imu_to_init:=false. Пустой imu_topic:= не передаём: пустой remap роняет узел.
if [ -n "${RTABMAP_IMU_TOPIC:-}" ]; then
    IMU_ARGS=("imu_topic:=$RTABMAP_IMU_TOPIC" "wait_imu_to_init:=true")
    echo "[start_rtabmap.sh] IMU on: ${IMU_ARGS[*]}"
else
    IMU_ARGS=("imu_topic:=/camera/imu/data" "wait_imu_to_init:=false")
    echo "[start_rtabmap.sh] IMU off (RTABMAP_IMU_TOPIC пуст): ${IMU_ARGS[*]}"
fi

echo "[start_rtabmap.sh] Launching with localization:=true: $@"
exec "$@" "${IMU_ARGS[@]}" "localization:=true" "${SIM_TIME_LAUNCH_ARGS[@]}"
