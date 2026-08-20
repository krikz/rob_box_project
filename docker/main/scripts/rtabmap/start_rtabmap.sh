#!/bin/bash
# Wrapper для запуска LiDAR-only rtabmap на Main Pi.

source /opt/ros/humble/setup.bash

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
python3 /ros_scripts/patch_rtabmap_launch.py || echo "[start_rtabmap.sh] WARN: patch_rtabmap_launch.py failed"

echo "[start_rtabmap.sh] Launching with localization:=true: $@"
exec "$@" "localization:=true"