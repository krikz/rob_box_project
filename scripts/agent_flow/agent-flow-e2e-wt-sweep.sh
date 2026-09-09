#!/bin/bash
# ============================================================================
# agent-flow-e2e-wt-sweep.sh — one-shot cleanup для /tmp/agent-flow-e2e-*
# (ретро t_0ff29dcd, issue #1707).
#
# Проблема:
#   agent-flow-e2e-process.sh v1 (commit eacb933c) создавал worktree
#   `/tmp/agent-flow-e2e-<PID>/` на каждый round, но при SIGKILL / OOM /
#   reboot cleanup() через trap НЕ вызывался → /tmp/ забивался (437+
#   worktree = ~87 ГБ). В develop v2 (PR #1889 / commit aef8f8f6)
#   добавили trap + per-tick sweep, но существующий завал остался.
#
# Этот скрипт — одноразовый «manual-trigger» для зачистки текущего завала:
#   1. Перечисляет ВСЕ /tmp/agent-flow-e2e-*/
#   2. Для каждого: проверяет PID (kill -0), если мёртв → удаляет каталог
#      + чистит .git/worktrees/<bn> (если связан с REPO_DIR).
#   3. TTL-clean: find -mtime +${E2E_WT_TTL_DAYS:-7} -type d -exec …
#   4. git worktree prune для зарегистрированных worktree'ов.
#   5. Печатает финальную метрику e2e_worktree_count.
#
# Идемпотентен. Безопасно запускать параллельно с cron-тиком (он делает то
# же самое: _wt_sweep_orphans / _wt_sweep_ttl — см. agent-flow-e2e-process.sh).
#
# Usage:
#   bash scripts/agent_flow/agent-flow-e2e-wt-sweep.sh             # реальный прогон
#   DRY_RUN=true bash scripts/agent_flow/agent-flow-e2e-wt-sweep.sh  # только показать
#
# ENV:
#   REPO_DIR              — путь к локальному клону (для git worktree prune).
#   E2E_WT_TTL_DAYS       — TTL по mtime (default 7).
#   DRY_RUN               — true/false (default false).
#   LOG_FILE              — TSV-лог удалений (default stderr).
#
# Exit: 0 = ok / partial; 1 = критичный сбой (нет write-доступа / REPO_DIR
# битый и т.п.).
# ============================================================================

set -uo pipefail  # без -e — sweep должен быть максимально tolerant

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
E2E_WT_TTL_DAYS="${E2E_WT_TTL_DAYS:-7}"
DRY_RUN="${DRY_RUN:-false}"
LOG_PREFIX="[agent-flow-e2e-wt-sweep]"
LOG_FILE="${LOG_FILE:-}"

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- helpers (one-shot, обособленные от e2e-process.sh) -----------------------

# _ws_is_pid_alive <pid> → 0 если процесс жив (kill -0).
_ws_is_pid_alive() {
    local pid="${1:-}"
    [ -n "$pid" ] && [ "$pid" -gt 0 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null
}

# _ws_remove <path> → снять worktree (git bookkeeping) + удалить каталог.
# Tolerates: missing dir, broken .git, permission errors, не REPO_DIR'ов.
_ws_remove() {
    local wt="$1"
    [ -n "$wt" ] || return 0
    if [ -d "$REPO_DIR" ] && [ -d "$REPO_DIR/.git" ]; then
        bn="$(basename "$wt")"
        # shellcheck disable=SC2034  # git worktree remove выводит в stderr если не его
        git -C "$REPO_DIR" worktree remove --force "$wt" 2>/dev/null || true
        rm -rf "$REPO_DIR/.git/worktrees/$bn" 2>/dev/null || true
    fi
    [ -d "$wt" ] && rm -rf "$wt" 2>/dev/null || true
}

# _ws_count → print count of /tmp/agent-flow-e2e-*/ dirs.
_ws_count() {
    [ -d /tmp ] || { printf '0\n'; return 0; }
    local cnt=0
    for d in /tmp/agent-flow-e2e-*; do
        [ -d "$d" ] || continue
        [ "$d" = "/tmp/agent-flow-e2e-*" ] && break
        cnt=$((cnt+1))
    done
    printf '%s\n' "$cnt"
}

# --- main sweep ---------------------------------------------------------------
log "start: REPO_DIR=${REPO_DIR} TTL_DAYS=${E2E_WT_TTL_DAYS} DRY_RUN=${DRY_RUN}"
log "e2e_worktree_count(before)=$(_ws_count)"

# 1. Sweep orphans (PID already dead).
removed_orphans=0
kept_alive=0
for d in /tmp/agent-flow-e2e-*; do
    [ -d "$d" ] || continue
    [ "$d" = "/tmp/agent-flow-e2e-*" ] && break
    pid="$(printf '%s' "$d" | sed -nE 's@.*/tmp/agent-flow-e2e-(roundonly-)?([0-9]+).*@\2@p')"
    if [ -n "$pid" ] && _ws_is_pid_alive "$pid"; then
        kept_alive=$((kept_alive+1))
        log "skip (alive PID $pid): $d"
        continue
    fi
    log "remove orphan: $d (pid=${pid:-<unrecognized>})"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: rm -rf $d"
    else
        _ws_remove "$d"
        removed_orphans=$((removed_orphans+1))
    fi
done

# 2. TTL sweep (mtime-старые, даже если PID «жив» в zombie-таблице).
removed_ttl=0
while IFS= read -r -d '' d; do
    [ -d "$d" ] || continue
    pid="$(printf '%s' "$d" | sed -nE 's@.*/tmp/agent-flow-e2e-(roundonly-)?([0-9]+).*@\2@p')"
    if [ -n "$pid" ] && _ws_is_pid_alive "$pid"; then
        log "skip TTL (alive PID $pid): $d"
        continue
    fi
    log "remove TTL (>${E2E_WT_TTL_DAYS}d): $d (pid=${pid:-<unrecognized>})"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: rm -rf $d"
    else
        _ws_remove "$d"
        removed_ttl=$((removed_ttl+1))
    fi
done < <(find /tmp -maxdepth 1 -mindepth 1 -type d \
    -name 'agent-flow-e2e-*' -mtime "+${E2E_WT_TTL_DAYS}" -print0 2>/dev/null || true)

# 3. git worktree prune — для зарегистрированных в REPO_DIR worktree'ов.
if [ -d "$REPO_DIR" ] && [ -d "$REPO_DIR/.git" ]; then
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: git -C $REPO_DIR worktree prune"
    else
        log "git -C $REPO_DIR worktree prune"
        git -C "$REPO_DIR" worktree prune 2>&1 | head -5
    fi
fi

# 4. Финальная метрика.
final_count="$(_ws_count)"
log "summary: removed_orphans=${removed_orphans} removed_ttl=${removed_ttl} kept_alive=${kept_alive}"
log "e2e_worktree_count(after)=${final_count}"

# 5. Возврат кода: 0 если всё ok (включая zero-state).
# В dry-run не фейлим — пользователь явно посмотрел бы, что будет.
exit 0
