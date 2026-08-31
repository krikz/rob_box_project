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
#   Rotation liveness: если за окно (по умолчанию 2h) НЕТ ни одного
#   tick в cron-output каталогах agent-flow-e2e-process и ни одного коммита
#   в origin/develop → ALERT (exit 1).
#
# Скрипт НЕ пишет в GitHub вообще: ни меток, ни комментариев, ни issue.
# Только stderr + exit-code для cron. Безопасно запускать каждые 15-30 минут.
#
# Границы ответственности (дедуп 30.08): «suite regression» — N подряд
# упавших e2e — этот скрипт больше НЕ считает. Тем же занимается
# agent-flow-e2e-fail-streak-watchdog.sh (ретро 28.08 t_faac94b0), который
# подключён к launcher'у и умеет больше: issue-comment при streak ≥ WARN и
# pause-sentinel при streak ≥ PAUSE. Здесь остаётся ровно то, чего не делает
# никто другой: «ротация вообще жива?».
#
# Выходы:
#   - Stderr: structured summary (для cron delivery).
#   - Exit 0 если ротация жива; exit 1 если мертва.
#
# ENV:
#   GH_REPO              — owner/repo (default krikz/rob_box_project)
#   HERMES_HOME          — база для cron-output профилей (default /home/builder/.hermes)
#   REPO_DIR             — путь к worktree основного проекта (для git log)
#   ROTATION_DEAD_MIN    — мин окна (default 120 = 2h)
#   WATCHDOG_DRY_RUN=true — только логировать, не алертить
# ============================================================================
set +e
# shellcheck source=lib_cron_env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/lib_cron_env.sh" || {
    printf "[%s] %s: lib_cron_env preflight failed — exit 1
" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(basename "${BASH_SOURCE[0]:-$0}")" >&2
    exit 1
}
set -euo pipefail

set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
ROTATION_DEAD_MIN="${ROTATION_DEAD_MIN:-120}"
WATCHDOG_DRY_RUN="${WATCHDOG_DRY_RUN:-false}"

# Sentinel: не запускать две копии одновременно. Общий с e2e-process flock
# задаётся снаружи через LOCK_FILE, если это когда-нибудь понадобится.
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

# --- Rotation liveness ------------------------------------------------------
# Признак А: свежий tick в cron-output каталогах (любого активного профиля).
# Признак Б (fallback, если А пуст): коммит в origin/develop за окно.
# Ни одного признака за ROTATION_DEAD_MIN → ротация мертва.

# find -newermt принимает только абсолютные timestamp'ы в формате без TZ; безопаснее
# сначала собрать все *.md, отсортировать по mtime, отбросить старше порога.
_last_tick_epoch_s=0
for d in \
    "${HERMES_HOME}/profiles/architect/cron/output" \
    "${HERMES_HOME}/profiles/agent-flow/cron/output" \
    "${HERMES_HOME}/profiles/devops/cron/output"; do
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

# Признак Б: ищем в репо любой коммит в origin/develop за окно.
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

exit 0
