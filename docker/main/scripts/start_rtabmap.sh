#!/bin/bash
# Wrapper для запуска rtabmap с публикацией недостающих TF алиасов.
#
# Проблема: depthai_ros_driver v2 с i_rs_compat:true публикует изображения
# с frame_id=camera_color_optical_frame (RealSense naming), но URDF использует
# camera_rgb_camera_optical_frame. rtabmap не может найти трансформацию.
#
# Решение: публикуем identity TF camera_rgb_camera_optical_frame → camera_color_optical_frame
# прежде чем запустить rtabmap.

source /opt/ros/humble/setup.bash

# ── Mapping Lifecycle: determine localization mode ─────────────────────────
# Default: always start in localization. Only switch to mapping mode if
# the state file explicitly says mode=mapping (robot restarted mid-mapping).
#
# State file: /maps/mapping_state.json (same volume as rtabmap.db)
# Written by MCP tools (StartMappingTool / FinishMappingTool).
STATE_FILE="/maps/mapping_state.json"
LOCALIZATION_MODE="true"  # safe default

if [ -f "$STATE_FILE" ]; then
    MODE=$(python3 -c "import json,sys; d=json.load(open('$STATE_FILE')); print(d.get('mode','localization'))" 2>/dev/null || echo "localization")
    MAP_NAME=$(python3 -c "import json,sys; d=json.load(open('$STATE_FILE')); print(d.get('map_name','') or '')" 2>/dev/null || echo "")
    if [ "$MODE" = "mapping" ]; then
        LOCALIZATION_MODE="false"
        echo "[start_rtabmap.sh] ⚠️  State file: mode=mapping — starting in SLAM mode (map: '${MAP_NAME}')"
    else
        echo "[start_rtabmap.sh] ✅ State file: mode=localization — starting in localization mode (map: '${MAP_NAME}')"
    fi
else
    echo "[start_rtabmap.sh] ℹ️  No state file found, defaulting to localization mode"
    # Write default state so tools can read it
    mkdir -p /maps
    python3 -c "
import json, time, pathlib
p = pathlib.Path('$STATE_FILE')
if not p.exists():
    p.write_text(json.dumps({'mode':'localization','map_name':None,'map_id':None,'updated_at':time.time()}, indent=2))
" 2>/dev/null || true
fi

# Patch rtabmap.launch.py to inject Grid/Sensor=2 as explicit ROS2 parameter.
# Required because rtabmap auto-resets Grid/Sensor=0 when subscribe_scan=true,
# unless the parameter is explicitly present in the Node's parameters dict.
python3 /ros_scripts/patch_rtabmap_launch.py || echo "[start_rtabmap.sh] WARN: patch_rtabmap_launch.py failed, Grid/Sensor may be 0"

echo "[start_rtabmap.sh] Starting static TF publisher: camera_rgb_camera_optical_frame → camera_color_optical_frame"

# ROS 2 Humble positional format: x y z qx qy qz qw parent_frame child_frame
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 \
    camera_rgb_camera_optical_frame \
    camera_color_optical_frame &

TF_PID=$!
echo "[start_rtabmap.sh] Static TF publisher PID: $TF_PID"

# Trap to kill TF publisher when main process exits
trap "kill $TF_PID 2>/dev/null" EXIT

echo "[start_rtabmap.sh] Launching with localization:=${LOCALIZATION_MODE}: $@"
exec "$@" "localization:=${LOCALIZATION_MODE}"
