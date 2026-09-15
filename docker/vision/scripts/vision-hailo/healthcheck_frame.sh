#!/usr/bin/env bash
# healthcheck_frame.sh — capability-honest healthcheck vision-hailo (ADR-0101).
#
# Старый healthcheck (``pgrep -f vision_hailo``) отвечал ``Up (healthy)``
# даже когда нода никогда не получала кадров от источника. Это маскировало
# реальный режим работы (issue #2531 acceptance #6): контейнер мог быть
# "здоровым" вечно, не получая ни одного кадра с OAK-D.
#
# Этот скрипт делает два независимых чека:
#
# 1) Процесс жив: ``pgrep -f vision_hailo`` — старое поведение, fast-fail.
# 2) Топик живой: ``ros2 topic info /vision/hailo/events`` показывает
#    publisher count > 0 — нода опубликовала хотя бы один event.
#
# Если оба true — exit 0 (healthy).
# Иначе — exit 1 (unhealthy, docker перезапустит по policy).
#
# Использование в docker-compose.yaml:
#   healthcheck:
#     test: ["/scripts/healthcheck_frame.sh"]
#     interval: 30s
#     timeout: 10s
#     start_period: 30s
#     retries: 3

set -eo pipefail

# ---------- 1) Процесс жив ----------
if ! pgrep -f vision_hailo > /dev/null; then
    echo "[healthcheck_frame] FAIL: процесс vision_hailo не найден" >&2
    exit 1
fi

# ---------- 2) Топик живой ----------
# ros2 topic info показывает publisher_count. Если 0 — нода не
# опубликовала ни одного event с момента старта.
if ! command -v ros2 > /dev/null 2>&1; then
    # Если ros2 CLI недоступен (например, на минимальном образе) —
    # fallback на pgrep-only чтобы не сломать CI smoke-тесты.
    echo "[healthcheck_frame] WARN: ros2 CLI не найден, fallback на pgrep-only" >&2
    exit 0
fi

# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true

# shellcheck disable=SC1091
source /ws/install/setup.bash 2>/dev/null || true

# Topic info возвращает строку вида:
#   Publisher count: 1
# Используем timeout на случай, если ros2 daemon завис.
TOPIC_INFO=$(timeout 5 ros2 topic info /vision/hailo/events 2>&1) || {
    echo "[healthcheck_frame] FAIL: ros2 topic info не ответил" >&2
    exit 1
}

if echo "${TOPIC_INFO}" | grep -qE '^Publisher count: [1-9]'; then
    exit 0
fi

echo "[healthcheck_frame] FAIL: /vision/hailo/events — Publisher count: 0" >&2
echo "[healthcheck_frame] (топик жив, но нода ни разу не опубликовала event)" >&2
echo "[healthcheck_frame] — это означает, что источник кадра (gaze_source) недоступен" >&2
echo "[healthcheck_frame] — или нода застряла в stub-режиме без реальных кадров" >&2
exit 1
