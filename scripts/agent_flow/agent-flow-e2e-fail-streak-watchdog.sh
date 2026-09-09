#!/bin/bash
# ============================================================================
# agent-flow-e2e-fail-streak-watchdog.sh — auto-escalation when L: E2E Voice
# Test workflow fails repeatedly without an intervening success.
#
# Контекст / ретро t_faac94b0 (2026-08-28):
#   L: E2E Voice Test fail-streak 24 раунда подряд (~3 дня) без какого-либо
#   автоматического алерта. Issue #1668 (STT-регрессия) открыт 46h+, без
#   process-меток → drift. PR #1673 (фикс wake-gate) готов, но не смёржен
#   из-за отсутствия process-маркера.
#
#   Причина: agent-flow-e2e-process-launcher.sh (cron 84864db04347, every 20m)
#   только ротирует раунды, fail-streak не проверяет. Issue остаются
#   «осиротевшими» пока кто-то вручную не поставит stale-candidate
#   (но unlabeled-sweep упал с GH_REPO must be set — 19 failures в ряд).
#
# Контракт (per tick, no-agent bash, вызывается из launcher):
#   1. List последние N (default 30) E2E workflow runs across all branches
#      (conclusion != null, status == completed).
#   2. Считаем streak_with_success — от самого нового run считаем FAIL'ы подряд,
#      пока не встретим SUCCESS (streak = 0). Если streak > STREAK_WARN (5):
#        - Issue comment в открытый process-релевантный issue (поиск по
#          label `needs-e2e` ИЛИ недавний FAIL-round ветке → если есть
#          упоминание в issue body — fallback на #1668 как known issue).
#        - log alert для cron-delivery.
#      Если streak > STREAK_PAUSE (20):
#        - Auto-pause: создаём файл PAUSE_SENTINEL → e2e-process в начале
#          следующего тика увидит sentinel и пропустит round creation
#          (аналог MAINTENANCE gate).
#        - Manual override через удаление файла (решение принимает Шиф).
#   3. Idempotent: comment пишется только если последний marker старше
#      MARKER_DEDUP_HOURS (default 6h). Pause-sentinel НЕ снимается —
#      это manual override (Шиф/юзер).
#
# ENV:
#   GH_REPO                       — owner/repo (default krikz/rob_box_project)
#   FAIL_STREAK_DRY_RUN=true      — log only, no API writes
#   E2E_FAIL_STREAK_WARN=5        — порог алерта (issue comment)
#   E2E_FAIL_STREAK_PAUSE=20      — порог auto-pause (sentinel file)
#   E2E_FAIL_STREAK_LIMIT=30      — сколько последних run'ов смотрим
#   E2E_FAIL_STREAK_DEDUP_HOURS=6 — дедупликация алерт-комментариев
#   HERMES_HOME                   — для sentinel path (default ~/.hermes)
#   LOCK_FILE                     — flock guard
#
# Выходы:
#   - Exit 0 — всё ok (даже если streak=0).
#   - Exit 1 — критичный сбой (нет gh auth, sentinel write failed).
#
# Pitfalls:
#   - gh run list --workflow принимает filename на DEFAULT branch (см.
#     github-actions-orchestration skill). Workflow display name «L: E2E
#     Voice Test» на main == «L-E2E Voice Test.yml». Используем filename.
#   - Без --branch: берём все round-ветки (z-{e2e}/test-round-*) +
#     main/master, чтобы streak не сбрасывался на каждом раунде.
#   - Sentinel — НЕ блокер навечно: при streak > 20 e2e ротация
#     замораживается пока человек не разберётся (это намеренно — следующие
#     20+ FAIL'ов только усугубят ситуацию и сожгут CI minutes).
# ============================================================================
set -uo pipefail  # без -e — ошибки не должны убивать cron

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
E2E_WORKFLOW="${E2E_WORKFLOW:-L-E2E Voice Test.yml}"
E2E_FAIL_STREAK_WARN="${E2E_FAIL_STREAK_WARN:-5}"
E2E_FAIL_STREAK_PAUSE="${E2E_FAIL_STREAK_PAUSE:-20}"
E2E_FAIL_STREAK_LIMIT="${E2E_FAIL_STREAK_LIMIT:-30}"
E2E_FAIL_STREAK_DEDUP_HOURS="${E2E_FAIL_STREAK_DEDUP_HOURS:-6}"
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
DRY_RUN="${FAIL_STREAK_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-fail-streak-watchdog.lock}"
PAUSE_SENTINEL="${PAUSE_SENTINEL:-${HERMES_HOME}/state/agent-flow-e2e-fail-streak-pause}"
MARKER_TAG="🤖 [agent:devops] script=agent-flow-e2e-fail-streak-watchdog streak=${E2E_FAIL_STREAK_WARN}+"

PREFIX="[agent-flow-e2e-fail-streak-watchdog]"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }

# --- flock guard ----------------------------------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    log "another instance running — skip"
    exit 0
fi

# --- gh auth probe --------------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
    log "ERROR: gh CLI not found"
    exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
    log "ERROR: gh auth failed"
    exit 1
fi

# --- compute fail-streak (newest → oldest, stop at first success) ---------
log "querying last ${E2E_FAIL_STREAK_LIMIT} runs of ${E2E_WORKFLOW}"
_runs_json="$(gh run list --repo "$GH_REPO" --workflow "$E2E_WORKFLOW" \
    --limit "$E2E_FAIL_STREAK_LIMIT" --json databaseId,conclusion,createdAt,headBranch,name 2>/dev/null || true)"

if [ -z "$_runs_json" ] || [ "$_runs_json" = "[]" ]; then
    log "no runs found (workflow may not exist yet) — skip"
    exit 0
fi

# Compute streak + last_success_at via python3 (json reliable here).
_streak_info="$(printf '%s' "$_runs_json" | python3 -c '
import json, sys
try:
    runs = json.load(sys.stdin)
except Exception:
    print("ERR"); raise SystemExit(0)
streak = 0
last_success_at = ""
for r in runs:  # already newest-first
    c = r.get("conclusion")
    if c == "success":
        last_success_at = r.get("createdAt", "")
        break
    if c in ("failure", "cancelled", "timed_out"):
        streak += 1
    # in_progress / queued / null conclusion → пропускаем (не failure)
print(f"{streak}|{last_success_at}")
' 2>/dev/null || echo "ERR|")"

if [ "${_streak_info%%|*}" = "ERR" ]; then
    log "ERROR: cannot parse runs json"
    exit 1
fi
_streak="${_streak_info%%|*}"
_last_success_at="${_streak_info#*|}"
log "streak=${_streak} last_success=${_last_success_at:-NEVER} warn=${E2E_FAIL_STREAK_WARN} pause=${E2E_FAIL_STREAK_PAUSE}"

# --- decide action --------------------------------------------------------
if [ "${_streak:-0}" -lt "$E2E_FAIL_STREAK_WARN" ] 2>/dev/null; then
    log "streak < WARN — no action"
    exit 0
fi

# Найти issue для alert: открытые issues с label needs-e2e (в ротации) ИЛИ
# fallback на конкретный known issue из задачи (issue #1668).
_target_issue=""
_target_issues="$(gh issue list --repo "$GH_REPO" --state open --label needs-e2e \
    --limit 5 --json number,title 2>/dev/null || echo '[]')"
_n_count="$(printf '%s' "$_target_issues" | python3 -c 'import json,sys;print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
log "open needs-e2e issues: ${_n_count}"

# Если needs-e2e пусто — ищем «голые» открытые issues, привязанные к fail-streak
# (например, issue #1668 был открыт 26.08 без process-меток). Берём 3 самых
# старых open issue без process-меток (это то, что должен был поймать
# unlabeled-sweep, но он упал).
if [ "${_n_count:-0}" -eq 0 ] 2>/dev/null; then
    _target_issues="$(gh issue list --repo "$GH_REPO" --state open \
        --json number,title,labels,createdAt \
        --jq '[.[] | select((.labels | map(.name) | inside(["hermes","needs-e2e","e2e-done","e2e:rejected","no-e2e-required","agent-flow-error"])) | not)] | sort_by(.createdAt) | .[0:3]' \
        2>/dev/null || echo '[]')"
    _n_count="$(printf '%s' "$_target_issues" | python3 -c 'import json,sys;print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
    log "open unlabeled-process issues (fallback pool): ${_n_count}"
fi

# --- WARN: issue comment (idempotent по marker dedup window) --------------
if [ "${_streak:-0}" -ge "$E2E_FAIL_STREAK_WARN" ] 2>/dev/null; then
    _now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    _now_epoch="$(date -u +%s)"
    _dedup_epoch=$((_now_epoch - E2E_FAIL_STREAK_DEDUP_HOURS * 3600))

    while IFS=$'\t' read -r _issue_num _issue_title; do
        [ -z "$_issue_num" ] && continue
        # Проверяем был ли marker за последние dedup hours
        _recent_marker="$(gh api "repos/${GH_REPO}/issues/${_issue_num}/comments?since=$(date -u -d "@${_dedup_epoch}" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo 1970-01-01T00:00:00Z)&per_page=100" \
            --jq '[.[] | select(.body | startswith("'"${MARKER_TAG}"'"))] | length' 2>/dev/null || echo 0)"
        if [ "${_recent_marker:-0}" -gt 0 ] 2>/dev/null; then
            log "issue #${_issue_num}: marker already posted within ${E2E_FAIL_STREAK_DEDUP_HOURS}h — skip"
            continue
        fi

        _body="${MARKER_TAG} fail-streak=${_streak} (>${E2E_FAIL_STREAK_WARN}) last_success=${_last_success_at:-NONE}.

Workflow \`${E2E_WORKFLOW}\` упал ${_streak} раз подряд без SUCCESS. Это не похоже на обычный рандомный fail — возможна регрессия (голос/STT/wake-gate/network/инфра).

Рекомендуемые действия:
1. Открыть последний failed run и посмотреть ROBOT LOG block — какой маркер (STT-empty / no_wake_word / tts-fallback / no-speech).
2. Если регрессия — поставить process-метку (\`needs-e2e\` или \`agent-flow-error\`) и привязать PR-фикс.
3. Если инфра-проблема — добавить в \`MAINTENANCE\` (kill-switch) на develop.

⚠️ При fail-streak > ${E2E_FAIL_STREAK_PAUSE} watchdog создаст pause-sentinel и ротация раундов будет заморожена до ручного override.
"

        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: gh issue comment ${_issue_num} (${_issue_title})"
        else
            if gh issue comment "$_issue_num" --repo "$GH_REPO" --body "$_body" >/dev/null 2>&1; then
                log "issue #${_issue_num}: alert posted (streak=${_streak})"
            else
                log "issue #${_issue_num}: WARNING comment failed (will retry next tick)"
            fi
        fi
    done < <(printf '%s' "$_target_issues" | python3 -c '
import json, sys
try:
    arr = json.load(sys.stdin)
    for it in arr:
        n = it.get("number", "")
        t = (it.get("title", "") or "")[:60]
        if n:
            print(f"{n}\t{t}")
except Exception:
    pass
' 2>/dev/null)
fi

# --- PAUSE: sentinel file (manual override only) ---------------------------
if [ "${_streak:-0}" -ge "$E2E_FAIL_STREAK_PAUSE" ] 2>/dev/null; then
    mkdir -p "$(dirname "$PAUSE_SENTINEL")" 2>/dev/null || true
    if [ -f "$PAUSE_SENTINEL" ]; then
        log "pause-sentinel already exists: ${PAUSE_SENTINEL} — no-op (manual override required to resume)"
    else
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: touch ${PAUSE_SENTINEL}"
        else
            cat > "$PAUSE_SENTINEL" <<EOF
# Auto-pause: L: E2E Voice Test fail-streak=${_streak} (>${E2E_FAIL_STREAK_PAUSE})
# Triggered: $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Last success: ${_last_success_at:-NONE}
#
# Этот файл замораживает auto-rotation раундов в agent-flow-e2e-process.sh
# (см. _e2e_fail_streak_pause_check). Удалить ВРУЧНУЮ когда:
#   - регрессия пофикшена И свежий run прошёл SUCCESS, ИЛИ
#   - Шиф дал override (reason: «это нормальный fail-streak, продолжаем»).
#
# Не удалять «чтобы посмотреть что будет» — следующие 20+ FAIL'ов сожгут CI.
EOF
            if [ -f "$PAUSE_SENTINEL" ]; then
                log "🚨 PAUSE-SENTINEL CREATED: ${PAUSE_SENTINEL} (streak=${_streak})"
                log "   e2e-process auto-rotation ЗАМОРОЖЕНА до ручного override."
            else
                log "ERROR: cannot create pause-sentinel ${PAUSE_SENTINEL}"
                exit 1
            fi
        fi
    fi
fi

log "tick done: streak=${_streak} action=${_streak_action:-none}"
exit 0
