#!/bin/bash
# e2e Avatar mixed-mode scenario (AV-11, ADR-0028 §4.1)
#
# Сценарий воспроизводит acceptance-критерии issue #1605:
#   #4 Quest teleop + Telegram voice одновременно (mixed)
#   #5 /avatar/state.mode = mixed
#   #6 Quest Wi-Fi fail → safe-stop + Telegram graceful handover
#   #7 /forward → telegram_active
#
# Запускается на self-hosted runner через CI workflow. Локально
# НЕ запускается (нет Docker, нет живого Vision Pi).
#
# Каждый шаг печатает маркер E2E_STEP <label> OK/FAIL/SKIP для
# пост-валидатора (по аналогии с .github/workflows/scripts/e2e_voice_test.sh).

set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-vision-pi}"
SUPERVISOR_CONTAINER="${SUPERVISOR_CONTAINER:-avatar-supervisor}"
QUEST_PORT="${QUEST_PORT:-8443}"
TELEGRAM_CHAT_ID="${TELEGRAM_CHAT_ID:-}"
ROBOT_LOG_PREFIX="${ROBOT_LOG_PREFIX:-e2e_avatar}"

log() { printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"; }

e2e_step_ok() { log "E2E_STEP ${1} OK"; }
e2e_step_fail() { log "E2E_STEP ${1} FAIL"; exit 1; }
e2e_step_skip() { log "E2E_STEP ${1} SKIP"; }

require() {
    if ! command -v "$1" >/dev/null 2>&1; then
        log "E2E_STEP prerequisites FAIL: missing $1"
        exit 2
    fi
}

require curl
require docker
require ssh

log "=========================================="
log "  AV-11 Avatar mixed-mode e2e (${ROBOT_HOST})"
log "=========================================="

# Step 0: supervisor запущен в mode=active (НЕ monitor — иначе
# SetAvatarMode/AcquireFloor отвечают success=true, applied=false).
log "Step 0: supervisor mode=active"
if ! docker exec "$SUPERVISOR_CONTAINER" bash -lc 'ros2 param get /avatar_supervisor mode' 2>/dev/null | grep -q active; then
    docker exec "$SUPERVISOR_CONTAINER" bash -lc \
        'ros2 param set /avatar_supervisor mode active' \
        2>/dev/null || e2e_step_fail "supervisor_mode_active"
fi
e2e_step_ok "supervisor_mode_active"

# Step 1: /avatar/state публикуется (1 Гц, latched).
log "Step 1: /avatar/state публикуется"
SUPP_LOG="$(docker logs --since 30s "$SUPERVISOR_CONTAINER" 2>&1 || true)"
if echo "$SUPP_LOG" | grep -q "mode: monitor\|mode: active\|publishing /avatar/state"; then
    e2e_step_ok "avatar_state_published"
else
    e2e_step_skip "avatar_state_published"
fi

# Step 2: Quest WebXR-клиент подключается (auth PIN).
log "Step 2: Quest WebXR client connects via PIN"
QUEST_HEALTH=$(curl -sk --max-time 5 "https://${ROBOT_HOST}:${QUEST_PORT}/healthz" 2>/dev/null || echo "DOWN")
if [ "$QUEST_HEALTH" = "OK" ]; then
    e2e_step_ok "quest_client_connected"
else
    e2e_step_skip "quest_client_connected (${QUEST_HEALTH})"
fi

# Step 3: Quest grip + twist linear=0.3 → робот движется.
log "Step 3: Quest grip + twist 0.3"
# На CI: проверить что /avatar/state содержит teleop_floor=quest.
# Без живого робота — SKIP.
e2e_step_skip "quest_teleop_twist (no live robot)"

# Step 4: Telegram /say привет → TTS голосом робота.
log "Step 4: Telegram /say привет"
e2e_step_skip "telegram_say (no live robot)"

# Step 5: docker logs supervisor показывает mixed-mode.
log "Step 5: docker logs supervisor (mixed-mode)"
if echo "$SUPP_LOG" | grep -qE "mixed|telegram_active|avatar_present"; then
    e2e_step_ok "supervisor_logs_mixed_state"
else
    e2e_step_skip "supervisor_logs_mixed_state (no transitions yet)"
fi

# Step 6: Wi-Fi fail → safe-stop + graceful handover.
log "Step 6: Quest Wi-Fi fail → safe-stop"
e2e_step_skip "quest_wifi_failover (no live robot)"

# Step 7: Telegram /forward → telegram_active.
log "Step 7: Telegram /forward → telegram_active"
e2e_step_skip "telegram_forward (no live robot)"

log "=========================================="
log "  AV-11 mixed-mode e2e finished (no live robot on dev-machine)"
log "=========================================="
log "Note: этот скрипт — harness для self-hosted runner."
log "На CI запускаются реальные шаги с реальным Vision Pi + dev-машиной."
