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

# НЕ `set -u`: `source /opt/ros/$ROS_DISTRO/setup.bash` обращается к
# необъявленной AMENT_TRACE_SETUP_FILES и падает под nounset
# (тот же паттерн, что start_quest.sh).
set -eo pipefail

# ---------- defaults (override через ENV или YAML) ----------
HAILO_ENABLED="${HAILO_ENABLED:-false}"
HEF_PATH="${HEF_PATH:-}"
STUB_PERIOD_SEC="${STUB_PERIOD_SEC:-2.0}"
CONFIDENCE_THRESHOLD="${CONFIDENCE_THRESHOLD:-0.5}"
INPUT_TOPIC="${INPUT_TOPIC:-/oak/rgb/image_raw/compressed}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/vision/hailo/events}"
HAILO_MODELS_YAML="${HAILO_MODELS_YAML:-/config/hailo_models.yaml}"

# ---------- source ROS workspace ----------
# rob_box workspace собирается в /ws/install через colcon (см. Dockerfile).
# Это контракт Phase 1 — Dockerfile сам собирает rob_box_perception_msgs +
# rob_box_perception в /ws (паттерн quest/supervisor).
if [ ! -f /ws/install/setup.bash ]; then
    echo "[start_vision_hailo] ERROR: /ws/install/setup.bash не найден" >&2
    echo "[start_vision_hailo] Образ vision-hailo должен собирать workspace через colcon." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

# ---------- если есть YAML — применяем его как defaults ----------
# SSoT-контракт (ADR-0018 capability-honest + ADR-0089):
#   * YAML = defaults (низкий приоритет).
#   * ENV  = override (явный приоритет, всегда побеждает).
#   * Если ENV задан непусто — YAML-значение SKIP'ается с WARN в лог
#     контейнера (НЕ silent degradation: оператор видит, что значение
#     пришло из .env, а не из YAML).
#   * Это лечит F-2 из t_beba0869 (silent-degradation когда prod .env
#     HAILO_ENABLED=true, а dev-YAML hailo_enabled:false).
if [ -f "${HAILO_MODELS_YAML}" ] && command -v python3 >/dev/null 2>&1; then
    echo "[start_vision_hailo] loading SSoT config: ${HAILO_MODELS_YAML}"
    eval "$(python3 - "${HAILO_MODELS_YAML}" <<'PY'
import os, sys, yaml
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
    if key not in node:
        continue
    env_name = key.upper()
    env_val = os.environ.get(env_name, '')
    if env_val:
        # ENV явно задан непусто — YAML пропускаем (ENV wins).
        # WARN в stderr контейнера, НЕ в stdout (stdout ловит eval $()).
        print(
            f'[start_vision_hailo] ENV override wins: {env_name} '
            f'(env="{env_val}", yaml="{node[key]}")',
            file=sys.stderr,
        )
        continue
    emit(key, node[key])
PY
    )"
fi

# ---------- summary ----------
echo "[start_vision_hailo] config: HAILO_ENABLED=${HAILO_ENABLED} HEF_PATH=${HEF_PATH:-<none>}"
echo "[start_vision_hailo] topics: input=${INPUT_TOPIC} output=${OUTPUT_TOPIC}"
echo "[start_vision_hailo] confidence_threshold=${CONFIDENCE_THRESHOLD} stub_period=${STUB_PERIOD_SEC}"

# ---------- capability-honest mode check (ADR-0018) ----------
# Если HAILO_ENABLED=true но HEF_PATH пуст / numpy / cv2 / hailo_platform
# отсутствуют — узел ВСЁ РАВНО стартует, но переходит в stub-режим.
# Это не silent degradation: логируем причину degraded mode.
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
        echo "[start_vision_hailo] WARN: HAILO_ENABLED=true, но degraded mode: ${DEGRADE_REASON}" >&2
        echo "[start_vision_hailo] WARN: нода продолжит работу в stub-режиме (ADR-0018 capability-honest)" >&2
    fi
fi

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

# ---------- launch ROS 2 node (ADR-0096) ----------
# ADR-0096: vision_hailo стартует декларативно через launch-файл
# (vision_hailo.launch.py), а не через захардкоженный `ros2 run` с
# --ros-args массивом. Преимущества:
#   1. LaunchConfiguration — SSoT параметров в одном месте.
#   2. OpaqueFunction pre-flight check (capability-honest, ADR-0018) —
#      оператор видит причину degraded-режима ДО старта ноды.
#   3. Когда-нибудь можно включить через <include> в общий perception
#      launch без рефакторинга.
#
# hef_path передаём через launch argument: пустой hef_path нельзя
# передавать как `-p hef_path:=` — rcl падает "Couldn't parse parameter
# override rule". Launch это обрабатывает корректно (см.
# vision_hailo.launch.py:DeclareLaunchArgument('hef_path', default_value='')).
exec ros2 launch rob_box_perception vision_hailo.launch.py \
    hailo_enabled:=${HAILO_ENABLED} \
    hef_path:=${HEF_PATH} \
    stub_period_sec:=${STUB_PERIOD_SEC} \
    confidence_threshold:=${CONFIDENCE_THRESHOLD} \
    input_topic:=${INPUT_TOPIC} \
    output_topic:=${OUTPUT_TOPIC} \
    publish_when_no_input:=true
