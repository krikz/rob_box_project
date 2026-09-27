#!/usr/bin/env bash
# healthcheck_frame.sh — capability-honest healthcheck vision-hailo
# (ADR-0104, issue #2703).
#
# История: старый healthcheck (``pgrep -f vision_hailo``) отвечал
# ``Up (healthy)`` даже когда нода никогда не получала кадров от источника
# (issue #2531 acceptance #6). Фикс #2608/ADR-0104 перешёл на
# ``ros2 topic echo /vision/hailo/events --once`` — но это оказалось ДВОЙНОЙ
# ложью в другую сторону (issue #2703, raw-лог 17.09.2026):
#
#   1) демон ros2cli на Vision Pi падает под rmw_zenoh
#      (``xmlrpc.client.Fault: !rclpy.ok()``) — ``ros2 topic echo`` без
#      ``--no-daemon`` фейлится НЕЗАВИСИМО от состояния ноды;
#   2) в real-режиме тишина в топике — норма: нода публикует только
#      события выше confidence_threshold (``filter_by_confidence``,
#      vision_hailo_node.py::_tick), heartbeat по stub_period_sec есть
#      только у stub-loader'а. Пустая сцена (нода жива, infer выполняется,
#      0 детекций) красила контейнер unhealthy — подтверждено деплоем
#      17.09.2026 (issue #2707).
#
# Новый чек:
#   1) процесс жив (``pgrep -f vision_hailo``) — fast-fail.
#   2) heartbeat-файл (issue #2703/#2704): нода обновляет его ПОСЛЕ
#      каждого успешного infer() — НЕ по факту публикации события
#      (rob_box_perception.utils.heartbeat.FileHeartbeat). Возраст файла
#      < HEARTBEAT_STALE_SEC → healthy. Пустая сцена не отличается от
#      "детекция была" — обе обновляют heartbeat одинаково, обе healthy.
#
# ros2cli (если где-то всё же нужен, например для ручной диагностики) —
# ТОЛЬКО с ``--no-daemon`` (issue #2703 п.1: демон падает под rmw_zenoh).
# Сам healthcheck больше НЕ зовёт ros2 CLI вообще — heartbeat-файл читается
# через ``stat``, без ROS/DDS зависимостей, поэтому быстрее и не подвержен
# падению демона.
#
# ENV:
#   HEARTBEAT_PATH      — путь к heartbeat-файлу
#                         (default /tmp/vision_hailo_heartbeat, должен
#                         совпадать с heartbeat_path launch-параметром
#                         ноды — см. utils/heartbeat.py::default_heartbeat_path).
#   HEARTBEAT_STALE_SEC — порог "протухания" в секундах (default 30 —
#                         совпадает с DEFAULT_FRAME_STALE_SEC в
#                         vision_hailo_node.py; при stub_period_sec=2.0
#                         тик идёт каждые ~0.5s, так что 30s — большой
#                         запас на единичный медленный infer).
#
# Использование в docker-compose.yaml:
#   healthcheck:
#     test: ["/scripts/healthcheck_frame.sh"]
#     interval: 30s
#     timeout: 10s
#     start_period: 30s
#     retries: 3

set -eo pipefail

HEARTBEAT_PATH="${HEARTBEAT_PATH:-/tmp/vision_hailo_heartbeat}"
HEARTBEAT_STALE_SEC="${HEARTBEAT_STALE_SEC:-30}"

# ---------- 1) Процесс жив ----------
if ! pgrep -f vision_hailo > /dev/null; then
    echo "[healthcheck_frame] FAIL: процесс vision_hailo не найден" >&2
    exit 1
fi

# ---------- 2) Heartbeat свежий ----------
# issue #2703: живость по факту успешного infer(), НЕ по наличию событий
# в топике — пустая сцена не должна читаться как смерть ноды.
if [ ! -f "${HEARTBEAT_PATH}" ]; then
    echo "[healthcheck_frame] FAIL: heartbeat-файл ${HEARTBEAT_PATH} не найден" >&2
    echo "[healthcheck_frame] — нода ещё ни разу не выполнила успешный infer()," >&2
    echo "[healthcheck_frame]   либо gaze_source недоступен (see issue #2703)" >&2
    exit 1
fi

NOW_EPOCH=$(date +%s)
HEARTBEAT_EPOCH=$(stat -c %Y "${HEARTBEAT_PATH}" 2>/dev/null || stat -f %m "${HEARTBEAT_PATH}" 2>/dev/null) || {
    echo "[healthcheck_frame] FAIL: не удалось прочитать mtime ${HEARTBEAT_PATH}" >&2
    exit 1
}
AGE=$(( NOW_EPOCH - HEARTBEAT_EPOCH ))

if [ "${AGE}" -lt 0 ] || [ "${AGE}" -gt "${HEARTBEAT_STALE_SEC}" ]; then
    echo "[healthcheck_frame] FAIL: heartbeat устарел (${AGE}s > ${HEARTBEAT_STALE_SEC}s)" >&2
    echo "[healthcheck_frame] — _tick/infer не выполнялся дольше порога:" >&2
    echo "[healthcheck_frame]   либо нода зависла, либо HEF loader в degraded" >&2
    echo "[healthcheck_frame]   state (см. vision_hailo_node.py::_tick except)" >&2
    exit 1
fi

exit 0
