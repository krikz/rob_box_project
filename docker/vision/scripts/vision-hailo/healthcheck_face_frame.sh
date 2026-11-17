#!/usr/bin/env bash
# healthcheck_face_frame.sh — capability-honest healthcheck vision-face
# (ADR-0089 Phase 2, issue #2704).
#
# История бага (issue #2704): ``command -v ros2`` стоял ДО
# ``source /opt/ros/.../setup.bash`` — ros2 CLI не в базовом PATH образа,
# поэтому скрипт ВСЕГДА уходил в fallback ``exit 0`` ("pgrep-only"),
# независимо от того, публикует ли нода хоть что-то. Тот же баг у
# vision-hailo был исправлен в 51fbcbd98 (#2608), в vision-face фикс не
# перенесли. Даже если поправить только порядок source/command-v — чек
# остаётся слабым по двум независимым причинам (issue #2704 + #2703):
#   1) ``ros2 topic info`` ходит через демон ros2cli, который на Vision Pi
#      падает под rmw_zenoh (``xmlrpc.client.Fault: !rclpy.ok()``,
#      issue #2703 п.1 — корень БАГА вне scope этого фикса, демон нужно
#      чинить отдельно) — без ``--no-daemon`` healthcheck красится по
#      чужой причине НЕЗАВИСИМО от состояния ноды;
#   2) ``Publisher count > 0`` на ``/vision/hailo/events`` — топик ОБЩИЙ с
#      vision-hailo (issue #2599 PR-A), поэтому publisher count может
#      быть ненулевым за счёт соседа, даже если vision-face немая
#      (признак разобран и признан непригодным в issue #2602).
#
# Новый чек (по образцу healthcheck_frame.sh, issue #2703/#2704):
#   1) процесс vision_face жив (``pgrep``) — fast-fail.
#   2) heartbeat-файл СВОЕЙ ноды (НЕ общий топик): vision_face_node
#      обновляет его после каждого успешного infer() — см.
#      rob_box_perception.utils.heartbeat.FileHeartbeat, тот же механизм,
#      что у vision-hailo, но отдельный файл per NODE_NAME.
#
# Решение больше НЕ зовёт ros2 CLI вообще (heartbeat читается через
# ``stat``): устраняет issue #2704 источник (source/command-v порядок
# становится не важен — ros2 не вызывается) И issue #2703 п.1
# (сломанный ros2cli-демон) одновременно, без починки самого демона.
# Если кому-то в будущем понадобится диагностика через ros2 CLI внутри
# контейнера вручную — обязательно source ПЕРЕД command -v ros2, и
# ``--no-daemon`` на каждый вызов (см. healthcheck_frame.sh комментарий).
#
# ENV:
#   HEARTBEAT_PATH      — путь к heartbeat-файлу
#                         (default /tmp/vision_face_heartbeat).
#   HEARTBEAT_STALE_SEC — порог "протухания" в секундах (default 30).
#
# Использование в docker-compose.yaml:
#   healthcheck:
#     test: ["/scripts/healthcheck_face_frame.sh"]
#     interval: 30s
#     timeout: 10s
#     start_period: 30s
#     retries: 3

set -eo pipefail

HEARTBEAT_PATH="${HEARTBEAT_PATH:-/tmp/vision_face_heartbeat}"
HEARTBEAT_STALE_SEC="${HEARTBEAT_STALE_SEC:-30}"

# ---------- 1) Процесс жив ----------
if ! pgrep -f vision_face > /dev/null; then
    echo "[healthcheck_face_frame] FAIL: процесс vision_face не найден" >&2
    exit 1
fi

# ---------- 2) Heartbeat своей ноды свежий ----------
# issue #2704: живость СВОЕЙ ноды, не Publisher count общего топика
# (issue #2602 — этот признак непригоден: publisher создаётся и у немой
# ноды, а топик /vision/hailo/events общий с vision-hailo).
if [ ! -f "${HEARTBEAT_PATH}" ]; then
    echo "[healthcheck_face_frame] FAIL: heartbeat-файл ${HEARTBEAT_PATH} не найден" >&2
    echo "[healthcheck_face_frame] — нода ещё ни разу не выполнила успешный infer()" >&2
    exit 1
fi

NOW_EPOCH=$(date +%s)
HEARTBEAT_EPOCH=$(stat -c %Y "${HEARTBEAT_PATH}" 2>/dev/null || stat -f %m "${HEARTBEAT_PATH}" 2>/dev/null) || {
    echo "[healthcheck_face_frame] FAIL: не удалось прочитать mtime ${HEARTBEAT_PATH}" >&2
    exit 1
}
AGE=$(( NOW_EPOCH - HEARTBEAT_EPOCH ))

if [ "${AGE}" -lt 0 ] || [ "${AGE}" -gt "${HEARTBEAT_STALE_SEC}" ]; then
    echo "[healthcheck_face_frame] FAIL: heartbeat устарел (${AGE}s > ${HEARTBEAT_STALE_SEC}s)" >&2
    echo "[healthcheck_face_frame] — лицо-нода не выполняла infer() дольше порога" >&2
    exit 1
fi

exit 0
