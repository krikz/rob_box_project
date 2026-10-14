#!/bin/bash
# agent-flow-install-daily.sh — wrapper для ежедневного install.sh тика.
# Ретро 28.08 t_7ebdfce0: PR #1710 cleanup-логика доезжала только в
# часть профилей; ежедневный запуск install.sh закрывает gap без ручного
# запуска оператора. Используется из cron-job `agent-flow-install-daily`
# (no_agent=True, daily 03:00, devops-профиль). Stdout — silent если
# всё ОК (anti-escape OK + verify md5), иначе печатает diff.
set -e
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"

# --- MAINTENANCE gate (issue #3389) -----------------------------------------
# Inline-проверка (без source lib_agent_flow_common): remote через
# git ls-remote, local fallback через git -C REPO_DIR show. Шифу ставит
# MAINTENANCE-файл в develop чтобы приостановить работу воркеров на время
# ручных правок. Срабатывает → exit 0 (тик пропускается, не ошибка).
_branch="${MAINTENANCE_BRANCH:-develop}"
_file="${MAINTENANCE_FILE:-MAINTENANCE}"
if [ -n "${GH_REPO:-}" ] \
    && git ls-remote "https://github.com/${GH_REPO}.git" "${_branch}:${_file}" \
        2>/dev/null | grep -q .; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] install-daily: [MAINTENANCE] gate active on remote — skip" >&2
    exit 0
fi
if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ] \
    && git -C "$REPO_DIR" show "${_branch}:${_file}" >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] install-daily: [MAINTENANCE] gate active locally in ${REPO_DIR} — skip" >&2
    exit 0
fi
unset _branch _file

exec bash "$REPO_DIR/scripts/agent_flow/install.sh"
