#!/bin/bash
# ============================================================================
# round_formation.sh — единый модуль формирования e2e test-round (issue #2299)
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/round_formation.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` рядом с остальными процессными
# скриптами (см. EXPECTED[]).
#
# ЗАЧЕМ (issue #2299, соглашение 09.09.2026):
#   До этого та же логика (ls-remote → max-N → freshness-check → create/reuse/
#   recreate) жила копипастой в двух местах:
#     - agent-flow-e2e-process.sh:round_ensure() — счётчик пишется DEFERRED
#       (только после ≥1 run'а в post-tick cleanup, ретро 23.08 t_fdb19f7b)
#     - round_ensure.sh — счётчик пишется IMMEDIATE (после create/reuse)
#
#   Противоречивая семантика → рассинхрон: ручной `round_ensure.sh` на пустом
#   раунде (0 запусков) тоже персистит счётчик → +1 ghost. Ретро t_d3aeaa9b
#   (stale-base fix) приходилось зеркалить руками.
#
#   КАНОН (09.09.2026): канон = DEFERRED-семантика e2e-process. Один модуль
#   владеет записью счётчика ЕДИНОЖДЫ (только после подтверждённого ≥1
#   запуска раунда); ghost-счётчик для пустых веток сохраняется.
#   round_ensure.sh становится тонкой обёрткой (--wait + flock живут там).
#
# КОНТРАКТ:
#   Source: `. round_formation.sh` (после set -euo pipefail, в REPO_DIR-среде).
#   Env (все доступные, default'ы разумные):
#     REPO_DIR               — путь к локальному clone (ls-remote + push)
#     GH_REPO                — owner/repo
#     FOUNDATION_BRANCH      — база для round-N (default: develop)
#     TEST_ROUND_PREFIX      — префикс ветки (default: 'z-{e2e}/test-round-')
#     ROUND_COUNTER_FILE     — файл счётчика (default: $HERMES_HOME/state/...)
#     DRY_RUN                — true → не пушить, только лог
#     WORKTREE_DIR           — если задан, дополнительный fetch в worktree
#     GIT_PUSH_FN            — имя функции push (default: git_push_with_cred_fallback)
#                              (round_ensure.sh может переопределить на raw push,
#                              если обёртка credential-fallback недоступна)
#
#   API:
#     rf_compute_n                           → printf '%s\n' "$n" (next round number)
#     rf_compute_state                       → printf '%s %s\n' "$n" "$max_n" (для caller)
#     round_formation [gh_push_fn]           → main entry: compute + create/reuse/recreate
#                                              Выставляет:
#                                                ROUND_BRANCH        — новая/существующая ветка
#                                                ROUND_FORMATION_CREATED=1 если СОЗДАЛА/ПЕРЕСОЗДАЛА
#                                                ROUND_FORMATION_REUSED=1   если REUSE
#                                                n / max_n / counter_n    — числа для caller
#                                              Возвращает 0 на успех, 1 на ошибке push.
#                                              COUNTER НЕ ПИШЕТ (это ответственность caller,
#                                              см. rf_persist_counter_if_real_round).
#
#     rf_persist_counter_if_real_round      → записать counter (≥1 run);
#                                              вызывать из post-tick cleanup после проверки runs.
#     rf_ghost_round_log_and_metric         → маркер GHOST_ROUND + bump cumulative metric;
#                                              вызывать на 0 run'ов вместо записи counter.
#
#   Тонкая обёртка round_ensure.sh делает:
#     - flock (как у e2e-process, --wait N поддерживается)
#     - source этого модуля
#     - round_formation
#     - print ROUND_BRANCH
#     - persist counter (или ghost-log для симметрии с e2e-process)
#
# СОВМЕСТИМОСТЬ:
#   - Тесты test_e2e_process_round_ensure_counter.sh остаются зелёными (post-tick
#     cleanup по-прежнему вызывает rf_persist_counter_if_real_round /
#     rf_ghost_round_log_and_metric — те же маркеры).
#   - ROUND_CREATED=1 оставлен в e2e-process (для post-tick cleanup логики).
#     Внутри round_formation модуль выставляет ROUND_FORMATION_CREATED=1;
#     e2e-process пробрасывает это в ROUND_CREATED.
# ============================================================================

# --- defaults (env may override) --------------------------------------------
: "${FOUNDATION_BRANCH:=develop}"
: "${TEST_ROUND_PREFIX:=z-{e2e}/test-round-}"
: "${ROUND_COUNTER_FILE:=${HERMES_HOME:-/home/builder/.hermes}/state/agent-flow-e2e-round-counter}"
: "${GHOST_ROUNDS_TOTAL_FILE:=${HERMES_HOME:-/home/builder/.hermes}/state/agent-flow-e2e-ghost-rounds-total}"
: "${DRY_RUN:=false}"
: "${GIT_PUSH_FN:=git_push_with_cred_fallback}"

# Caller-visible state (используется e2e-process / round_ensure.sh)
ROUND_BRANCH=""
ROUND_FORMATION_CREATED=0
ROUND_FORMATION_REUSED=0
n=0
max_n=0
counter_n=0

# Local helpers (используются только внутри модуля).
_rf_log() {
    # Делегируем в caller'овский `log` если есть, иначе printf в stderr.
    if declare -F log >/dev/null 2>&1; then
        log "$@"
    else
        printf '%s [round_formation] %s\n' "$(date -Iseconds 2>/dev/null || date)" "$*" >&2
    fi
}

# --- rf_compute_state -------------------------------------------------------
# Печатает max_n и counter_n в stdout (по одному значению в строке).
# Это diagnostic; основная API — round_formation (ниже).
rf_compute_state() {
    local list
    list="$(git -C "${REPO_DIR:-.}" ls-remote --heads origin "${TEST_ROUND_PREFIX}*" 2>/dev/null \
        | awk '{print $2}' | sed "s#refs/heads/${TEST_ROUND_PREFIX}##" || true)"
    if [ -z "$list" ]; then
        max_n=0
    else
        max_n="$(printf '%s\n' "$list" | sort -n | tail -n1)"
    fi
    counter_n=0
    if [ -f "${ROUND_COUNTER_FILE}" ]; then
        counter_n="$(tr -dc '0-9' < "${ROUND_COUNTER_FILE}" 2>/dev/null || echo 0)"
        counter_n="${counter_n:-0}"
    fi
    if [ "$counter_n" -gt "$max_n" ]; then
        _rf_log "round counter: file=${counter_n} > remote-max=${max_n} (cleanup сбросил ветки?) — берём max из файла"
        max_n="$counter_n"
    fi
    n=$((max_n + 1))
    ROUND_BRANCH="${TEST_ROUND_PREFIX}${n}"
    _rf_log "round number: max=${max_n} -> next=${n}"
}

# --- round_formation [gh_push_fn_override] ----------------------------------
# Main entry. Caller обязан иметь REPO_DIR/GH_REPO/FOUNDATION_BRANCH/DRY_RUN.
# Возвращает 0 на успех (ветка существует локально+remote), 1 на ошибке push.
#
# Side effects:
#   - ROUND_BRANCH        — выставлен
#   - ROUND_FORMATION_CREATED=1 если СОЗДАЛИ или ПЕРЕСОЗДАЛИ (stale-base delete+create)
#   - ROUND_FORMATION_REUSED=1   если REUSE (ветка уже была и содержит foundation)
#   - n / max_n / counter_n — числа доступны caller'у
#   - НЕ пишет ROUND_COUNTER_FILE (caller решает)
round_formation() {
    local push_fn="${1:-${GIT_PUSH_FN}}"

    [ -n "${REPO_DIR:-}" ] || { _rf_log "REPO_DIR must be set"; return 1; }
    [ -d "${REPO_DIR}" ]   || { _rf_log "REPO_DIR does not exist: $REPO_DIR"; return 1; }
    [ -n "${GH_REPO:-}" ]  || { _rf_log "GH_REPO must be set"; return 1; }

    rf_compute_state

    if ! git -C "$REPO_DIR" ls-remote --heads origin "$ROUND_BRANCH" 2>/dev/null | grep -q .; then
        # Ветки нет — создаём из foundation (всегда свежий, НЕ локальный ref).
        _rf_log "creating ${ROUND_BRANCH} from ${FOUNDATION_BRANCH} (fresh fetch)"
        if [ "$DRY_RUN" = "true" ]; then
            _rf_log "DRY-RUN would: push origin/${FOUNDATION_BRANCH}:refs/heads/${ROUND_BRANCH}"
            ROUND_FORMATION_CREATED=1
        else
            if ! git -C "$REPO_DIR" fetch origin "$FOUNDATION_BRANCH" 2>&1 | sed 's/^/  /'; then
                _rf_log "failed to fetch origin/${FOUNDATION_BRANCH}"; return 1
            fi
            if ! "$push_fn" "$REPO_DIR" origin "origin/${FOUNDATION_BRANCH}:refs/heads/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
                _rf_log "failed to create ${ROUND_BRANCH}"; return 1
            fi
            ROUND_FORMATION_CREATED=1
        fi
    else
        # Ветка есть — freshness check (ретро 12.08 t_d3aeaa9b).
        _rf_log "checking ${ROUND_BRANCH} base freshness (must contain origin/${FOUNDATION_BRANCH})"
        if [ "$DRY_RUN" = "true" ]; then
            _rf_log "DRY-RUN would: check ancestry origin/${FOUNDATION_BRANCH}..${ROUND_BRANCH}"
        else
            if ! git -C "$REPO_DIR" fetch origin "$FOUNDATION_BRANCH" 2>&1 | sed 's/^/  /'; then
                _rf_log "failed to fetch origin/${FOUNDATION_BRANCH}"; return 1
            fi
            if git -C "$REPO_DIR" merge-base --is-ancestor "origin/${FOUNDATION_BRANCH}" "origin/${ROUND_BRANCH}" 2>/dev/null; then
                _rf_log "reusing ${ROUND_BRANCH} (база актуальна: содержит origin/${FOUNDATION_BRANCH})"
                ROUND_FORMATION_REUSED=1
            else
                _rf_log "🛑 ${ROUND_BRANCH} база УСТАРЕЛА (не содержит origin/${FOUNDATION_BRANCH}) — удаляю и создам заново (ретро 12.08 t_d3aeaa9b)"
                if ! "$push_fn" "$REPO_DIR" origin --delete "$ROUND_BRANCH" 2>&1 | sed 's/^/  /'; then
                    _rf_log "failed to delete stale ${ROUND_BRANCH} (non-fatal)"; true
                fi
                if ! "$push_fn" "$REPO_DIR" origin "origin/${FOUNDATION_BRANCH}:refs/heads/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
                    _rf_log "failed to recreate ${ROUND_BRANCH}"; return 1
                fi
                ROUND_FORMATION_CREATED=1
                _rf_log "recreated ${ROUND_BRANCH} from fresh origin/${FOUNDATION_BRANCH}"
            fi
        fi
    fi

    # Make sure worktree has the branch (если задан WORKTREE_DIR).
    if [ -n "${WORKTREE_DIR:-}" ] && [ -d "${WORKTREE_DIR}" ]; then
        git -C "$WORKTREE_DIR" fetch origin "$ROUND_BRANCH" --quiet 2>/dev/null || true
    fi
    return 0
}

# --- rf_persist_counter_if_real_round ---------------------------------------
# Пишет счётчик ЕСЛИ n > counter_n (т.е. round реально новый).
# Вызывать из post-tick cleanup ТОЛЬКО когда ≥1 run (иначе ghost).
rf_persist_counter_if_real_round() {
    if [ "${n:-0}" -gt "${counter_n:-0}" ]; then
        if [ "$DRY_RUN" != "true" ]; then
            printf '%s\n' "$n" > "$ROUND_COUNTER_FILE" 2>/dev/null \
                && _rf_log "round counter saved: ${n} -> ${ROUND_COUNTER_FILE}" \
                || _rf_log "WARNING: cannot write round counter ${ROUND_COUNTER_FILE}"
        else
            _rf_log "DRY-RUN would: round counter saved ${n} -> ${ROUND_COUNTER_FILE}"
        fi
    fi
}

# --- rf_ghost_round_log_and_metric ------------------------------------------
# Канонический маркер + cumulative metric для ghost'ов (ретро 23.08 t_fdb19f7b).
# Вызывать когда на round-ветке 0 run'ов — НЕ персистить counter, плюс писать
# GHOST_ROUND counter_rollback для парсера монитора.
#
# Аргументы:
#   $1 — ROUND_BRANCH (если не задан как ROUND_BRANCH выше)
rf_ghost_round_log_and_metric() {
    local branch="${1:-${ROUND_BRANCH:-}}"
    if [ -z "$branch" ]; then
        _rf_log "WARNING: rf_ghost_round_log_and_metric: empty branch"; return 1
    fi
    _rf_log "GHOST_ROUND counter_rollback branch=${branch} n=${n:-?} remote_max=${max_n:-?} prev_counter=${counter_n:-0}"
    if [ "$DRY_RUN" = "true" ]; then
        _rf_log "DRY-RUN would increment ghost-rounds-total ${GHOST_ROUNDS_TOTAL_FILE}"
        return 0
    fi
    local _ghost_prev=0
    if [ -f "$GHOST_ROUNDS_TOTAL_FILE" ]; then
        _ghost_prev="$(tr -dc '0-9' < "$GHOST_ROUNDS_TOTAL_FILE" 2>/dev/null || echo 0)"
        _ghost_prev="${_ghost_prev:-0}"
    fi
    local _ghost_next=$((_ghost_prev + 1))
    printf '%s\n' "$_ghost_next" > "$GHOST_ROUNDS_TOTAL_FILE" 2>/dev/null \
        && _rf_log "ghost-rounds-total: ${_ghost_prev} -> ${_ghost_next} -> ${GHOST_ROUNDS_TOTAL_FILE}" \
        || _rf_log "WARNING: cannot write ghost-rounds-total ${GHOST_ROUNDS_TOTAL_FILE}"
}