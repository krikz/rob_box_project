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

echo "[start_rtabmap.sh] Launching with ${MODE_LAUNCH_ARGS[*]}: $*"
exec "$@" "${MODE_LAUNCH_ARGS[@]}" "${SIM_TIME_LAUNCH_ARGS[@]}"