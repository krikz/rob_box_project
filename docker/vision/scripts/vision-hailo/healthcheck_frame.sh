#!/usr/bin/env bash
# healthcheck_frame.sh — capability-honest healthcheck vision-hailo (ADR-0104).
#
# Старый healthcheck (``pgrep -f vision_hailo``) отвечал ``Up (healthy)``
# даже когда нода никогда не получала кадров от источника. Это маскировало
# реальный режим работы (issue #2531 acceptance #6): контейнер мог быть
# "здоровым" вечно, не получая ни одного кадра с OAK-D.
#
# Этот скрипт делает два независимых чека:
#
# 1) Процесс жив: ``pgrep -f vision_hailo`` — старое поведение, fast-fail.
# 2) Топик публикует: ``ros2 topic echo ... --once`` получает хотя бы
#    одно событие. Publisher count > 0 НЕ подходит (issue #2602): publisher
#    создаётся на старте даже у полностью немой ноды, поэтому старый чек
#    держал контейнер healthy при нулевом выходе событий.
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

# ---------- 2) Топик публикует события ----------
# issue #2602: publisher count > 0 НЕ означает, что нода публикует —
# publisher создаётся на старте даже у немой ноды (рекурсивный spin_once
# блокировал executor). Проверяем ФАКТ доставки: --once выходит, как
# только приходит первое событие (stub_period_sec=2.0 → за ~0.5-2.5s).

# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true

# shellcheck disable=SC1091
source /ws/install/setup.bash 2>/dev/null || true

# source ДО проверки ros2: иначе command -v ros2 всегда fail на образе
# без ros2 в базовом PATH → вечный fallback на pgrep-only (тот самый
# «blind healthcheck», который issue #2602 запрещает).
if ! command -v ros2 > /dev/null 2>&1; then
    # Если ros2 CLI недоступен даже после source (минимальный образ без
    # /opt/ros и /ws/install) — fallback на pgrep-only, чтобы не сломать
    # CI smoke-тесты.
    echo "[healthcheck_frame] WARN: ros2 CLI не найден, fallback на pgrep-only" >&2
    exit 0
fi

# timeout 8 < docker-compose healthcheck timeout: 10s. Нода публикует
# каждые stub_period_sec (2.0s), поэтому первого события ждём с запасом.
if timeout 8 ros2 topic echo /vision/hailo/events rob_box_perception_msgs/msg/VisionEvent --once > /dev/null 2>&1; then
    exit 0
fi

echo "[healthcheck_frame] FAIL: нет событий в /vision/hailo/events за 8s" >&2
echo "[healthcheck_frame] — publisher создан, но нода молчит (issue #2602:" >&2
echo "[healthcheck_frame]   рекурсивный spin_once блокировал executor)," >&2
echo "[healthcheck_frame]   либо источник кадра (gaze_source) недоступен" >&2
exit 1
