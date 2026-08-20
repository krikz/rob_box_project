#!/bin/bash
# ============================================================================
# watchdog-tts.sh — мониторинг tts_node: если подвис (жив, но не реагирует
# на /voice/tts/request) — kill PID, launch.respawn поднимет новый.
#
# Зачем: после PR #1083 (цепочка TTS провайдеров) tts_node может зависнуть
# на внутреннем DDS-callback'е (semafor забит / participant issue / busy
# worker). Признаки:
#   - процесс жив (PID есть, CPU < 5%)
#   - publish'и в /voice/tts/request идут (mcp_server логирует)
#   - tts_node НЕ логирует "TTS: ..." в течение окна
#
# Не лечит сам баг, но снижает MTTR до минут (раньше — до рестарта контейнера).
#
# Использование:
#   bash watchdog-tts.sh                  # single-shot check + auto-recover
#   bash watchdog-tts.sh --check-only    # только проверить, не убивать
#   bash watchdog-tts.sh --quiet         # без INFO-логов, только WARN/ERROR
#
# Cron-обёртка (в cron-loop.sh):
#   every 5m: bash watchdog-tts.sh
#
# Env:
#   ROBOT_HOST=10.1.1.21  (default)
#   SSHPASS=open          (default)
# ============================================================================

set -e

SCRIPT_NAME="watchdog-tts"
LOG_PREFIX="[${SCRIPT_NAME}]"
ROBOT_HOST="${ROBOT_HOST:-10.1.1.21}"
ROBOT_USER="${ROBOT_USER:-ros2}"
SSHPASS_VAL="${SSHPASS:-open}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5"

# thresholds
WINDOW_S="${WINDOW_S:-30}"            # смотрим последние 30s
PUBLISHER_MIN="${PUBLISHER_MIN:-2}"  # минимум 2 TTS-publish'а в окно
LOG_PATTERN='TTS: \['              # в логах tts_node: "🔊 TTS: <text>..."

# flags
AUTO_RECOVER=1
QUIET=0
for arg in "$@"; do
    case "$arg" in
        --check-only) AUTO_RECOVER=0 ;;
        --quiet|-q)    QUIET=1 ;;
        --help|-h)
            sed -n '2,30p' "$0"
            exit 0
            ;;
    esac
done

log()  { [ "$QUIET" = "1" ] && [ "${1:-INFO}" = "INFO" ] && return 0; printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*"; }
err()  { printf '%s %s ERROR %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

robot_ssh() {
    sshpass -p "$SSHPASS_VAL" ssh $SSH_OPTS "${ROBOT_USER}@${ROBOT_HOST}" "$@"
}

# 1. Publisher count на /voice/tts/request через DDS (ros2 topic info).
#    Robust к типу publisher'а (mcp_server, dialogue_node, telegram_node).
PUBLISH_RATE=$(robot_ssh \
    "docker exec voice-assistant bash -c \"
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash
        timeout 5 ros2 topic info /voice/tts/request --verbose 2>/dev/null \
          | grep -E 'Publisher count' \
          | head -1 \
          | grep -oE '[0-9]+' || echo 0
    \"" 2>/dev/null | head -1)
PUBLISH_RATE=${PUBLISH_RATE:-0}

# 2. TTS-processed count (логирование tts_node) за WINDOW_S
TTS_LOG_COUNT=$(robot_ssh \
    "docker logs voice-assistant --since ${WINDOW_S}s 2>&1 | grep -cE 'tts_node.*TTS: \['" \
    2>/dev/null || echo 0)
TTS_LOG_COUNT=$(printf '%s' "$TTS_LOG_COUNT" | grep -oE '[0-9]+' | tail -1)
TTS_LOG_COUNT=${TTS_LOG_COUNT:-0}

log "INFO  publisher_count=${PUBLISH_RATE} tts_log_count=${TTS_LOG_COUNT} (window=${WINDOW_S}s)"

# 3. tts_node PID + CPU%
TTS_PID_CPU=$(robot_ssh \
    "docker exec voice-assistant bash -c \"ps aux | grep tts_node | grep -v grep | awk '{print \\\$2, \\\$3}'\"" \
    2>/dev/null | head -1)
TTS_PID=$(printf '%s' "$TTS_PID_CPU" | awk '{print $1}')
TTS_CPU=$(printf '%s' "$TTS_PID_CPU" | awk '{print $2}')

if [ -z "$TTS_PID" ]; then
    err "tts_node не найден в voice-assistant — оставляю launch.respawn"
    exit 0
fi

log "INFO  tts_node PID=${TTS_PID} CPU=${TTS_CPU}%"

# 4. Решение: есть активный publisher, но tts_node не реагирует (tts_logs=0)
#    + низкий CPU = завис.
if [ "$PUBLISH_RATE" -eq 0 ]; then
    log "INFO  нет активных publishers /voice/tts/request — tts_node простаивает, не вмешиваюсь"
    exit 0
fi

if [ "$TTS_LOG_COUNT" -ge 1 ]; then
    log "INFO  tts_node обработал хотя бы 1 запрос в окне — OK"
    exit 0
fi

# Есть publisher, но logs=0 → завис
CPU_INT=${TTS_CPU%.*}
if [ -n "$CPU_INT" ] && [ "$CPU_INT" -lt 5 ]; then
    err "tts_node ЗАВИС: PID=${TTS_PID} CPU=${TTS_CPU}% publishers=${PUBLISH_RATE} tts_logs=${TTS_LOG_COUNT} (publisher активен, нода не отвечает)"
    if [ "$AUTO_RECOVER" = "1" ]; then
        log "INFO  kill -KILL ${TTS_PID}; launch.respawn поднимет новый через 5s"
        robot_ssh "docker exec voice-assistant bash -c 'kill -KILL ${TTS_PID}'" 2>/dev/null
        log "INFO  recovery: жду 8s (respawn_delay=5s + init), проверяю"
        sleep 8
        NEW_PID=$(robot_ssh \
            "docker exec voice-assistant bash -c \"ps aux | grep tts_node | grep -v grep | awk '{print \\\$2}'\"" \
            2>/dev/null | head -1 | tr -d '\r\n')
        if [ -n "$NEW_PID" ] && [ "$NEW_PID" != "$TTS_PID" ]; then
            log "INFO  tts_node respawned: PID ${TTS_PID} → ${NEW_PID}"
            # опционально: тест-публикация для подтверждения
            if [ -f /tmp/tts_probe.py ]; then
                log "INFO  /tmp/tts_probe.py найден, тест-publish..."
                # probe уже на 21:21 — fallback
                robot_ssh "test -f /tmp/tts_probe.py" 2>/dev/null && {
                    log "INFO  probe-скрипт на роботе: $(robot_ssh "ls -la /tmp/tts_probe.py 2>/dev/null" | head -1)"
                }
            fi
        else
            err "respawn НЕ сработал: PID=${NEW_PID} (статус неизвестен)"
            exit 2
        fi
    else
        log "INFO  --check-only: не убиваю"
    fi
else
    log "WARN  publish>tts_logs, но CPU=${TTS_CPU}% — возможно обрабатывает, не вмешиваюсь"
fi
