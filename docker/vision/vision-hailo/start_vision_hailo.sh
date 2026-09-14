#!/usr/bin/env bash
# start_vision_hailo.sh — entrypoint для vision-hailo сервиса (ADR-0089 Phase 1).
#
# Запускает vision_hailo_node в ROS 2 окружении rob_box workspace.
# Поддерживает два режима (через ENV):
#   - HAILO_ENABLED=true + HEF_PATH=...: real inference.
#   - HAILO_ENABLED=false (default): stub для smoke-теста.
#
# Параметры читаются из /config/hailo_models.yaml (SSoT), либо из ENV
# (override для CI). YAML-парсинг — через python3 -c.
#
# Exit codes:
#   0 — clean shutdown по SIGTERM/SIGINT.
#   1 — setup error (config not found / ROS не sourced).
#   2 — node crashed (ros2 launch fallback).

set -euo pipefail

# ---------- defaults (override через ENV или YAML) ----------
HAILO_ENABLED="${HAILO_ENABLED:-false}"
HEF_PATH="${HEF_PATH:-}"
STUB_PERIOD_SEC="${STUB_PERIOD_SEC:-2.0}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.5}"
INPUT_TOPIC="${INPUT_TOPIC:-/oak/rgb/image_raw/compressed}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/vision/hailo/events}"
HAILO_MODELS_YAML="${HAILO_MODELS_YAML:-/config/hailo_models.yaml}"

# ---------- source ROS workspace ----------
# rob_box workspace собирается в /opt/rob_box/install через colcon.
# Это контракт Phase 1 — Dockerfile проверяет наличие /opt/rob_box/install.
if [ ! -f /opt/rob_box/install/setup.bash ]; then
    echo "[start_vision_hailo] ERROR: /opt/rob_box/install/setup.bash не найден" >&2
    echo "[start_vision_hailo] Соберите rob_box_perception через colcon перед сборкой образа." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/rob_box/install/setup.bash

# ---------- если есть YAML — применяем его как defaults ----------
if [ -f "${HAILO_MODELS_YAML}" ] && command -v python3 >/dev/null 2>&1; then
    echo "[start_vision_hailo] loading SSoT config: ${HAILO_MODELS_YAML}"
    eval "$(python3 - "${HAILO_MODELS_YAML}" <<'PY'
import sys, yaml
try:
    with open(sys.argv[1]) as f:
        cfg = yaml.safe_load(f) or {}
except Exception as exc:
    print(f'echo "[start_vision_hailo] WARN: yaml parse failed: {exc}" >&2')
    sys.exit(0)
node = (cfg.get('vision_hailo_node') or {})
def emit(k, v):
    # Quote value for shell; bool -> true/false (ROS2 совместимо).
    if isinstance(v, bool):
        v = 'true' if v else 'false'
    print(f'export {k.upper()}="{v}"')
for key in ('hailo_enabled', 'hef_path', 'stub_period_sec',
            'confidence_threshold', 'input_topic', 'output_topic'):
    if key in node:
        emit(key, node[key])
PY
    )"
fi

# ---------- summary ----------
echo "[start_vision_hailo] config: HAILO_ENABLED=${HAILO_ENABLED} HEF_PATH=${HEF_PATH:-<none>}"
echo "[start_vision_hailo] topics: input=${INPUT_TOPIC} output=${OUTPUT_TOPIC}"
echo "[start_vision_hailo] confidence_threshold=${CONFIDENCE_THRESHOLD} stub_period=${STUB_PERIOD_SEC}"

# ---------- smoke check (hailortcli) ----------
if [ "${HAILO_ENABLED}" = "true" ]; then
    if command -v hailortcli >/dev/null 2>&1; then
        echo "[start_vision_hailo] running hailortcli scan (smoke)..."
        hailortcli scan || {
            echo "[start_vision_hailo] ERROR: hailortcli scan failed — HAT не виден?" >&2
            exit 1
        }
    else
        echo "[start_vision_hailo] WARN: hailortcli не установлен, но HAILO_ENABLED=true" >&2
    fi
fi

# ---------- launch ROS 2 node ----------
exec ros2 run rob_box_perception vision_hailo \
    --ros-args \
    -p hailo_enabled:=${HAILO_ENABLED} \
    -p hef_path:="${HEF_PATH}" \
    -p stub_period_sec:=${STUB_PERIOD_SEC} \
    -p confidence_threshold:=${CONFIDENCE_THRESHOLD} \
    -p input_topic:="${INPUT_TOPIC}" \
    -p output_topic:="${OUTPUT_TOPIC}" \
    -p publish_when_no_input:=true
