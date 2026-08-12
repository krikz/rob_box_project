#!/bin/bash
# ============================================================================
# agent-flow-drift-detect.sh — drift-детектор для процессных скриптов.
#
# Source of truth: <repo>/scripts/agent_flow/*.sh
# Проверяет, что md5sum между репо и 4 хостами-копиями одинаковые:
#   - /home/builder/hermes-share/rob_box_project/scripts/agent_flow/<file>
#   - /home/builder/.hermes/profiles/agent-flow/scripts/<file>
#   - /home/builder/.hermes/profiles/architect/scripts/<file>
#   - /home/builder/.hermes/profiles/devops/scripts/<file>
#   - /home/builder/.hermes/scripts/<file>
#
# Ретро 12.08 t_24054f6c: install.sh не донёс обновлённый merge-gate/triage
# в 4 host-копии → cron дрифтанулся → ADR-0014 post-merge не работал
# (merge-gate), triage-skip не работал (triage).
#
# Поведение:
#   - OK (все md5 равны)         → exit 0, stdout пустой (no_op)
#   - DRIFT (md5 разные)         → auto-fix: bash install.sh
#                                 → если после fix OK → exit 0, stdout "fixed"
#                                 → если fix не помог → exit 1, stdout alert
#
# Exit codes:
#   0 — нет дрифта ИЛИ автофикс успешен
#   1 — дрифт остался после автофикса (нужна ручная разборка)
#
# Используется cron'ом `Agent Flow Scripts Drift` (no_agent=True, every 6h)
# в профиле devops. Cron scheduler выводит stdout, если непустой — это
# и есть сигнал тревоги. Тихий tick (пустой stdout) означает ОК.
#
# Идемпотентен. Запускать можно вручную:
#   bash /home/builder/.hermes/scripts/agent-flow-drift-detect.sh
# ============================================================================
set -u

REPO_DIR="/home/builder/hermes-share/rob_box_project"
SCRIPT_DIR="$REPO_DIR/scripts/agent_flow"
INSTALL_SH="$SCRIPT_DIR/install.sh"
ALERT_LOG="/home/builder/.hermes/profiles/devops/cron/output/agent-flow-drift.alert.log"

# Какие файлы проверяем (только те, что в install.sh EXPECTED)
FILES=(
    agent-flow-triage.sh
    agent-flow-merge-gate.sh
    agent-flow-e2e-process.sh
    agent-flow-handoff.sh
    round_ensure.sh
    agent-flow-cleanup-249.sh
    cron-loop.sh
    watchdog.sh
    agent-flow-drift-detect.sh
)

TARGETS=(
    "$REPO_DIR/scripts/agent_flow"
    "/home/builder/.hermes/profiles/agent-flow/scripts"
    "/home/builder/.hermes/profiles/architect/scripts"
    "/home/builder/.hermes/profiles/devops/scripts"
    "/home/builder/.hermes/scripts"
)

mkdir -p "$(dirname "$ALERT_LOG")" 2>/dev/null || true

log() {
    # Пишем и в stdout (для cron) и в alert log (для истории)
    local line
    line="[$(date -Iseconds)] $*"
    echo "$line"
    echo "$line" >> "$ALERT_LOG" 2>/dev/null || true
}

DRIFT=0
DRIFT_FILES=()

for f in "${FILES[@]}"; do
    # Берём эталон — md5 в репо (если файл есть)
    if [ ! -f "$SCRIPT_DIR/$f" ]; then
        continue
    fi
    REPO_MD5="$(md5sum "$SCRIPT_DIR/$f" | cut -c1-12)"

    # Собираем md5 всех копий
    HAS_MISMATCH=0
    for t in "${TARGETS[@]}"; do
        if [ "$t" = "$REPO_DIR/scripts/agent_flow" ]; then
            continue  # эталон не сравниваем сам с собой
        fi
        if [ -f "$t/$f" ]; then
            CUR_MD5="$(md5sum "$t/$f" | cut -c1-12)"
            if [ "$CUR_MD5" != "$REPO_MD5" ]; then
                HAS_MISMATCH=1
                break
            fi
        fi
    done

    if [ "$HAS_MISMATCH" = "1" ]; then
        DRIFT=1
        DRIFT_FILES+=("$f")
    fi
done

if [ "$DRIFT" = "0" ]; then
    # Тихий tick — не пишем никуда, exit 0
    exit 0
fi

# === ДРИФТ ОБНАРУЖЕН ===
log "DRIFT detected in ${#DRIFT_FILES[@]} file(s): ${DRIFT_FILES[*]}"
for f in "${DRIFT_FILES[@]}"; do
    log "  $f:"
    for t in "${TARGETS[@]}"; do
        if [ "$t" = "$REPO_DIR/scripts/agent_flow" ]; then
            continue
        fi
        if [ -f "$t/$f" ]; then
            MD="$(md5sum "$t/$f" | cut -c1-12)"
            INO="$(stat -c '%i' "$t/$f" 2>/dev/null || echo '?')"
            log "    $MD  inode=$INO  $t/$f"
        else
            log "    MISSING  $t/$f"
        fi
    done
done

# === АВТОФИКС ===
log "Auto-fix: bash $INSTALL_SH"
if bash "$INSTALL_SH" >> "$ALERT_LOG" 2>&1; then
    log "Auto-fix OK. Re-checking drift..."
    STILL_DRIFT=0
    for f in "${DRIFT_FILES[@]}"; do
        REPO_MD5="$(md5sum "$SCRIPT_DIR/$f" | cut -c1-12)"
        for t in "${TARGETS[@]}"; do
            if [ "$t" = "$REPO_DIR/scripts/agent_flow" ]; then
                continue
            fi
            if [ -f "$t/$f" ]; then
                CUR_MD5="$(md5sum "$t/$f" | cut -c1-12)"
                if [ "$CUR_MD5" != "$REPO_MD5" ]; then
                    STILL_DRIFT=1
                    break 2
                fi
            fi
        done
    done
    if [ "$STILL_DRIFT" = "0" ]; then
        log "FIXED — drift resolved"
        exit 0
    else
        log "FIX FAILED — drift still present after install.sh"
        exit 1
    fi
else
    log "FIX FAILED — install.sh exited non-zero"
    exit 1
fi