#!/usr/bin/env bash
# healthcheck_face_frame.sh — capability-honest healthcheck vision-face (ADR-0089 Phase 2).
#
# Зеркалит healthcheck_frame.sh под лицевую ноду vision_face:
#   1) процесс vision_face жив (pgrep),
#   2) /vision/hailo/events имеет Publisher count > 0.
#
# Примечание: топик /vision/hailo/events общий для vision_hailo и
# vision_face (issue #2599 PR-A), поэтому publisher count > 0 сам по себе
# не атрибутируется к лицевой ноде. Основной сигнал — pgrep vision_face.

set -eo pipefail

if ! pgrep -f vision_face > /dev/null; then
    echo "[healthcheck_face_frame] FAIL: процесс vision_face не найден" >&2
    exit 1
fi

if ! command -v ros2 > /dev/null 2>&1; then
    echo "[healthcheck_face_frame] WARN: ros2 CLI не найден, fallback на pgrep-only" >&2
    exit 0
fi

# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source /ws/install/setup.bash 2>/dev/null || true

TOPIC_INFO=$(timeout 5 ros2 topic info /vision/hailo/events 2>&1) || {
    echo "[healthcheck_face_frame] FAIL: ros2 topic info не ответил" >&2
    exit 1
}

if echo "${TOPIC_INFO}" | grep -qE '^Publisher count: [1-9]'; then
    exit 0
fi

echo "[healthcheck_face_frame] FAIL: /vision/hailo/events — Publisher count: 0" >&2
echo "[healthcheck_face_frame] — лицо-нода ни разу не опубликовала event" >&2
exit 1
