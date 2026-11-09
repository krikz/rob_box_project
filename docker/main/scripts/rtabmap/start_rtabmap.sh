#!/bin/bash
# Wrapper для запуска LiDAR-only rtabmap на Main Pi.

source /opt/ros/humble/setup.bash

# USE_SIM_TIME=true только в профиле docker/sim (Gazebo): use_sim_time:=true
# дописывается ПОСЛЕ аргументов compose и перекрывает их use_sim_time:=false.
# На роботе массив пустой.
source /ros_scripts/use_sim_time.sh

# ── Режим старта: localization (робот) | mapping (только профиль docker/sim) ──
# RTABMAP_START_MODE ставит только docker/sim/compose.yaml (SIM_RTABMAP_MODE).
# На роботе переменной нет → localization, команда та же, что раньше
# (tests/unit/docker/test_main_scripts_use_sim_time.py). mapping — как StartMapping
# с new_location=true (rob_box_mcp_tools/tools/mapping.py): localization:=false
# (Mem/IncrementalMemory=true) и пустая база (--delete_db_on_start).
RTABMAP_START_MODE="${RTABMAP_START_MODE:-localization}"
case "$RTABMAP_START_MODE" in
    localization) MODE_LAUNCH_ARGS=("localization:=true") ;;
    mapping)
        MODE_LAUNCH_ARGS=("localization:=false" "args:=--delete_db_on_start")
        if [ "$USE_SIM_TIME" != "true" ]; then
            # Стереть боевую карту стартом контейнера нельзя: mapping — только в симе.
            echo "❌ ОШИБКА: RTABMAP_START_MODE=mapping допустим только с USE_SIM_TIME=true (сим)" >&2
            exit 1
        fi
        ;;
    *)
        echo "❌ ОШИБКА: RTABMAP_START_MODE='$RTABMAP_START_MODE' — допустимо localization или mapping" >&2
        exit 1
        ;;
esac

# ── Mapping Lifecycle: always start in localization mode ───────────────────
# На роботе rtabmap ВСЕГДА стартует в localization:=true.
#
# Причина удаления state file: mapping_state.json на Main Pi — отдельный volume
# от Vision Pi. MCP tools пишут файл на Vision Pi, а start_rtabmap.sh читал
# другой файл на Main Pi — стейт расходился и вызывал краши.
#
# Переключение в mapping mode происходит через ROS2 сервис set_mode_mapping,
# который вызывает StartMappingTool после старта rtabmap. Это надёжнее чем
# читать stale файл при старте контейнера.
echo "[start_rtabmap.sh] Starting in $RTABMAP_START_MODE mode (robot: always localization)"

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

# ── Вес тега-ориентира в графе (#4346, стенд локализации): только сим ────────
# RTABMAP_SIM_TAG_LINEAR_VARIANCE ставит только docker/sim (SIM_TAG_LINEAR_VARIANCE):
# launch-аргумент tag_linear_variance → landmark_linear_variance узла rtabmap
# (по умолчанию rtabmap.launch.py — 0.0001, σ = 1 см). На роботе переменной нет, команда прежняя.
TAG_ARGS=()
if [ -n "${RTABMAP_SIM_TAG_LINEAR_VARIANCE:-}" ]; then
    if [ "$USE_SIM_TIME" != "true" ]; then
        echo "❌ ОШИБКА: RTABMAP_SIM_TAG_LINEAR_VARIANCE допустим только с USE_SIM_TIME=true (сим)" >&2
        exit 1
    fi
    if ! [[ "$RTABMAP_SIM_TAG_LINEAR_VARIANCE" =~ ^[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]; then
        echo "❌ ОШИБКА: RTABMAP_SIM_TAG_LINEAR_VARIANCE='$RTABMAP_SIM_TAG_LINEAR_VARIANCE' — нужно число" >&2
        exit 1
    fi
    TAG_ARGS=("tag_linear_variance:=$RTABMAP_SIM_TAG_LINEAR_VARIANCE")
    echo "[start_rtabmap.sh] Sim tag variance: ${TAG_ARGS[*]}"
fi

echo "[start_rtabmap.sh] Launching with ${MODE_LAUNCH_ARGS[*]}: $*"
exec "$@" "${IMU_ARGS[@]}" "${MODE_LAUNCH_ARGS[@]}" "${TAG_ARGS[@]}" "${SIM_TIME_LAUNCH_ARGS[@]}"
