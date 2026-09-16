#!/usr/bin/env bash
# start_vision_face.sh — entrypoint для vision-face сервиса (ADR-0089 Phase 2).
#
# Запускает vision_face_node (RetinaFace) в ROS 2 окружении. По образцу
# start_vision_hailo.sh: параметры читаются из /config/hailo_models.yaml
# (секция vision_face_node, SSoT), ENV — override.
#
# Exit codes:
#   0 — clean shutdown по SIGTERM/SIGINT.
#   1 — setup error (config not found / ROS не sourced).

set -eo pipefail

# Порядок: непустой ENV → YAML → дефолты ниже (ADR-0120 §8). Дефолты
# ставятся ПОСЛЕ чтения YAML: compose передаёт HAILO_ENABLED/HEF_PATH
# пустыми, и ранний `${HAILO_ENABLED:-false}` превращал пустое значение в
# экспортированное "false", которое затем перебивало YAML.
HAILO_MODELS_YAML="${HAILO_MODELS_YAML:-/config/hailo_models.yaml}"

# ---------- source ROS workspace ----------
if [ ! -f /ws/install/setup.bash ]; then
    echo "[start_vision_face] ERROR: /ws/install/setup.bash не найден" >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

# ---------- если есть YAML — применяем его как defaults ----------
if [ -f "${HAILO_MODELS_YAML}" ] && command -v python3 >/dev/null 2>&1; then
    echo "[start_vision_face] loading SSoT config: ${HAILO_MODELS_YAML}"
    eval "$(python3 - "${HAILO_MODELS_YAML}" <<'PY'
import os, sys, yaml
try:
    with open(sys.argv[1]) as f:
        cfg = yaml.safe_load(f) or {}
except Exception as exc:
    print(f'echo "[start_vision_face] WARN: yaml parse failed: {exc}" >&2')
    sys.exit(0)
node = (cfg.get('vision_face_node') or {})
def emit(k, v):
    if isinstance(v, bool):
        v = 'true' if v else 'false'
    print(f'export {k.upper()}="{v}"')
for key in ('hailo_enabled', 'hef_path', 'stub_period_sec',
            'confidence_threshold', 'nms_iou_threshold', 'gaze_source',
            'first_frame_timeout_sec', 'output_topic'):
    if key not in node:
        continue
    env_name = key.upper()
    env_val = os.environ.get(env_name, '')
    if env_val:
        print(
            f'[start_vision_face] ENV override wins: {env_name} '
            f'(env="{env_val}", yaml="{node[key]}")',
            file=sys.stderr,
        )
        continue
    emit(key, node[key])
PY
    )"
fi

# ---------- defaults (если ни ENV, ни YAML не задали) ----------
HAILO_ENABLED="${HAILO_ENABLED:-false}"
HEF_PATH="${HEF_PATH:-}"
STUB_PERIOD_SEC="${STUB_PERIOD_SEC:-2.0}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.6}"
NMS_IOU_THRESHOLD="${NMS_IOU_THRESHOLD:-0.45}"
GAZE_SOURCE="${GAZE_SOURCE:-oak_d}"
FIRST_FRAME_TIMEOUT_SEC="${FIRST_FRAME_TIMEOUT_SEC:-10.0}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/vision/hailo/events}"

# ---------- summary ----------
echo "[start_vision_face] config: HAILO_ENABLED=${HAILO_ENABLED} HEF_PATH=${HEF_PATH:-<none>}"
echo "[start_vision_face] topics: output=${OUTPUT_TOPIC}"
echo "[start_vision_face] confidence_threshold=${CONFIDENCE_THRESHOLD} nms_iou=${NMS_IOU_THRESHOLD}"

# ---------- capability-honest mode check (ADR-0018) ----------
if [ "${HAILO_ENABLED}" = "true" ]; then
    DEGRADE_REASON=""
    if [ -z "${HEF_PATH}" ]; then
        DEGRADE_REASON="${DEGRADE_REASON}HEF_PATH пуст; "
    fi
    if ! python3 -c "import numpy" 2>/dev/null; then
        DEGRADE_REASON="${DEGRADE_REASON}numpy не установлен; "
    fi
    if ! python3 -c "import cv2" 2>/dev/null; then
        DEGRADE_REASON="${DEGRADE_REASON}opencv-python не установлен; "
    fi
    if ! python3 -c "import hailo_platform" 2>/dev/null; then
        DEGRADE_REASON="${DEGRADE_REASON}hailo_platform не установлен; "
    fi
    if [ -n "${DEGRADE_REASON}" ]; then
        echo "[start_vision_face] WARN: HAILO_ENABLED=true, но degraded mode: ${DEGRADE_REASON}" >&2
        echo "[start_vision_face] WARN: нода продолжит работу в stub-режиме (ADR-0018 capability-honest)" >&2
    fi
fi

# ---------- launch ROS 2 node ----------
# hef_path передаём только когда непустой (см. issue #2527 — пустой
# ``hef_path:=`` ломает ros2 launch).
LAUNCH_ARGS=(
    rob_box_perception vision_face.launch.py
    hailo_enabled:=${HAILO_ENABLED}
    stub_period_sec:=${STUB_PERIOD_SEC}
    confidence_threshold:=${CONFIDENCE_THRESHOLD}
    nms_iou_threshold:=${NMS_IOU_THRESHOLD}
    gaze_source:=${GAZE_SOURCE}
    first_frame_timeout_sec:=${FIRST_FRAME_TIMEOUT_SEC}
    output_topic:=${OUTPUT_TOPIC}
    publish_when_no_input:=true
)
if [ -n "${HEF_PATH}" ]; then
    LAUNCH_ARGS+=( hef_path:=${HEF_PATH} )
fi
exec ros2 launch "${LAUNCH_ARGS[@]}"
