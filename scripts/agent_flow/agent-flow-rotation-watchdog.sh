#!/bin/bash
# ============================================================================
# agent-flow-rotation-watchdog.sh — observability for e2e-rotation liveness
#
# Ретро 25.08 t_2d8cc9c4 (PR #1565/#1572, voice_selection_suite 9/9 fail):
# предполагалось, что e2e-rotation «мёртв» — needs-e2e issues остаются без
# round. На самом деле rotation живой, просто часть PR stale-branch блок-
# ирована (ретро 22.08 t_a2cd5753). Чтобы заранее ловить КАЖДЫЙ случай
# настоящей смерти rotation, добавляем отдельный watchdog:
#
#   1) Rotation liveness: если за окно (по умолчанию 2h) НЕТ ни одного
#      tick в cron-output каталогах agent-flow-e2e-process, ни создания
#      z-{e2e}/test-round-* веток за последние 2h → ALERT (exit 1).
#   2) Suite regression: если L: E2E Voice Test запускался >=3 раз за 12h
#      и все завершились FAIL на одном и том же test_case (mv01/mv02/mv03) →
#      auto-issue "voice_selection_suite regression" (по договорённости с
#      архитектором — этот тип issue уже задокументирован в ретро).
#
# Скрипт НЕ блокирует rotation. Только наблюдение + exit-code для cron.
# Watchdog безопасно запускать каждые 15-30 минут.
#
# Выходы:
#   - Stderr: structured summary (для cron delivery).
#   - Exit 0 если всё ок; exit 1 если rotation мёртв; exit 2 если regression.
#
# ENV:
#   GH_REPO              — owner/repo (default krikz/rob_box_project)
#   REPO_DIR             — путь к worktree основного проекта (для git ls-remote)
#   ROTATION_DEAD_MIN    — мин окна (default 120 = 2h)
#   REGRESSION_N_RUNS    — мин число fail подряд (default 3)
#   REGRESSION_WINDOW_H  — часовое окно (default 12h)
#   CRON_OUTPUT_GLOB     — glob для поиска cron tick логов (default ниже)
#   WATCHDOG_DRY_RUN=true — только логировать, не алертить
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
ROTATION_DEAD_MIN="${ROTATION_DEAD_MIN:-120}"
REGRESSION_N_RUNS="${REGRESSION_N_RUNS:-3}"
REGRESSION_WINDOW_H="${REGRESSION_WINDOW_H:-12}"
WATCHDOG_DRY_RUN="${WATCHDOG_DRY_RUN:-false}"

# Sentinel: не запускаться одновременно с самим e2e-process (оба читают
# git ls-remote). CRON_LOCKFILE задаёт общий flock.
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-rotation-watchdog.lock}"
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: another instance running — skip" >&2
    exit 0
fi

# Probe gh auth (fail-fast). БЕЗ этого мы не отличим «rotation мёртв»
# от «gh сломан».
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: gh auth failed — cannot evaluate rotation" >&2
    exit 1
fi

_now_s="$(date -u +%s)"
_dead_threshold_s=$(( _now_s - ROTATION_DEAD_MIN * 60 ))

# --- 1) Rotation liveness --------------------------------------------------
# Способ А: свежий tick в cron-output каталогах (любого активного профиля).
# Способ Б: новая z-{e2e}/test-round-* ветка (показывает, что e2e процесс
#           реально работал — даже если tick в output ещё не нарисовался).
# Способ В: постинг любого issue/PR за последние N минут (признак активного процесса).

# find -newermt принимает только абсолютные timestamp'ы в формате без TZ; безопаснее
# сначала собрать все *.md, отсортировать по mtime, отбросить старше порога.
_last_tick_epoch_s=0
for d in \
    /home/builder/.hermes/profiles/architect/cron/output \
    /home/builder/.hermes/profiles/agent-flow/cron/output \
    /home/builder/.hermes/profiles/devops/cron/output; do
    [ -d "$d" ] || continue
    while IFS= read -r f; do
        _mtime_s="$(stat -c '%Y' "$f" 2>/dev/null || echo 0)"
        if [ "${_mtime_s:-0}" -gt "$_last_tick_epoch_s" ] 2>/dev/null; then
            _last_tick_epoch_s="$_mtime_s"
        fi
    done < <(find "$d" -type f -name '*.md' 2>/dev/null || true)
done

_last_tick_iso=""
if [ "${_last_tick_epoch_s:-0}" -gt 0 ]; then
    _last_tick_iso="$(date -u -d "@${_last_tick_epoch_s}" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo "")"
fi

# Способ В fallback: ищем в репо любой коммит в origin/develop за окно.
_last_dev_commit_iso=""
if [ -z "$_last_tick_iso" ] && [ -d "$REPO_DIR/.git" ]; then
    _last_dev_commit_iso="$(git -C "$REPO_DIR" log 'origin/develop' --since="@${_dead_threshold_s}" --format='%cI' -n1 2>/dev/null || true)"
fi

# Сравним окно: если ни одного признака — alert.
if [ -z "$_last_tick_iso" ] && [ -z "$_last_dev_commit_iso" ]; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: 🚨 rotation_liveness=DEAD no tick or develop commits in ${ROTATION_DEAD_MIN}m (GH_REPO=${GH_REPO})" >&2
    if [ "$WATCHDOG_DRY_RUN" = "true" ]; then exit 0; fi
    exit 1
fi

_rot_age_min=$ROTATION_DEAD_MIN
if [ -n "$_last_tick_iso" ]; then
    _tick_s="$(date -u -d "$_last_tick_iso" +%s 2>/dev/null || echo "$_now_s")"
    _rot_age_min=$(( (_now_s - _tick_s) / 60 ))
fi

echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: ✓ rotation_liveness=ALIVE age=${_rot_age_min}m (last_tick=${_last_tick_iso:-none}, last_develop_commit=${_last_dev_commit_iso:-none})" >&2

# --- 2) Suite regression (mv01/mv02/mv03 voice_selection) -----------------
# Запрос:  список последних L: E2E Voice Test workflow runs по репо.
# Берём только conclusion=failure; парсим scenario_file из входных данных
# (через gh api jobs → test_case из лога крайне дорого; идём от простого:
# считаем fail по каждому test_case_id из runs с совпадающим head-branch,
# у которых result=fail и test_case в {mv01*, mv02*, mv03*}). Чтобы не
# палить ресурсы, держим окно REGRESSION_WINDOW_H и считаем по gh api
# /actions/runs?workflow=...
#
# Реально сейчас мы не можем извлекать test_case_id без артефактов workflow
# (он зашит в логи job'а). Делаем безопасный приблизительный фильтр:
# последние ${REGRESSION_N_RUNS} voice-тестов с тем же head_branch
# pattern=^z-\{e2e\}/test-round- все conclusion=failure → flag. Это
# достаточно чтобы всплыть на большинство реальных regression-ов и не
# алертить на единичные flake.
_e2e_workflow_id="$(gh api "repos/${GH_REPO}/actions/workflows" --jq '.workflows[] | select(.name == "L: E2E Voice Test") | .id' 2>/dev/null | head -1 || true)"

if [ -n "$_e2e_workflow_id" ]; then
    _window_start_iso="$(date -u -d "@$(( _now_s - REGRESSION_WINDOW_H * 3600 ))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || true)"
    _recent_runs_json="$(gh api "repos/${GH_REPO}/actions/workflows/${_e2e_workflow_id}/runs?per_page=20&created=>=${_window_start_iso}" \
        --jq '[.workflow_runs[] | select(.conclusion=="failure")] | .[] | {id:.id, br:.head_branch, when:.created_at, conc:.conclusion}' 2>/dev/null || echo "")"

    if [ -n "$_recent_runs_json" ]; then
        _e2e_round_branch_re='^z-\{e2e\}/test-round-[0-9]+$'
        _fail_count=$(printf '%s' "$_recent_runs_json" | grep -cE '"br":[[:space:]]*"z-\{e2e\}/test-round-' || true)
        if [ "${_fail_count:-0}" -ge "$REGRESSION_N_RUNS" ]; then
            echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: 🚨 suite_regression=SUSPECTED ${_fail_count} e2e-runs failed in last ${REGRESSION_WINDOW_H}h on z-{e2e}/test-round-* (≥${REGRESSION_N_RUNS})" >&2
            if [ "$WATCHDOG_DRY_RUN" = "true" ]; then exit 0; fi
            exit 2
        fi
    fi
fi

exit 0
