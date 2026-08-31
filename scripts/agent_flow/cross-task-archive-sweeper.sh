#!/bin/bash
# ============================================================================
# cross-task-archive-sweeper.sh — watchdog для orphan-blocked карточек,
#                                  которые ждут архивации после успешного
#                                  PR/issue (kernel-scope work-around).
#
# Ретро 22.08 t_d9b4c600 / t_6c632ed5:
#   kernel в hermes-agent v0.20.x жёстко ограничивает worker-tools
#   (kanban_complete/block/request_review) scope'ом ТЕКУЩЕЙ задачи. Это
#   правильно для защиты от случайных cross-task мутаций, но ЛОМАЕТ cleanup:
#   recovery-worker (например t_0222e192) не может через worker-tool
#   архивировать свой parent-тикет (t_5e50675b), даже если его работа
#   (PR #1500 CLOSED) уже завершена. В итоге blocked-карточки висят часами.
#
#   Единственный escape-hatch — это операторская CLI `hermes kanban archive`,
#   которая НЕ наследует worker-lock. Но human-operator не всегда на месте.
#   Этот скрипт = автоматизированный escape-hatch.
#
# Стратегия:
#   1. Найти все blocked-карточки assignee=devops, heartbeat stale > 24ч
#      (т.е. воркер не проявлял признаков жизни >24ч и dispatcher их не
#      подобрал — orphan-state когда claim_lock ещё жив, но worker_pid
#      уже мёртв, или карточка уже после завершения run).
#   2. Для каждой карточки:
#      a. Извлечь из body номера PR/issue (regex #NNNNN).
#      b. Спросить `gh pr view <N>` / `gh issue view <N>` → state.
#      c. Если PR/issue = CLOSED or MERGED → карточка завершена по сути.
#      d. Дополнительно проверить remote-ветку: если branch_name указан
#         И remote-ветка удалена (`git ls-remote ... refs/heads/X`
#         возвращает пустоту, или PR уже CLOSED с merged=false и на ветке
#         нет новых коммитов) → orphan-state снят.
#      e. Действие: archive через прямой SQL UPDATE (bypass worker-scope).
#         Эта операция использует тот же row-update что `archive_task()` в
#         hermes_cli/kanban_db.py (status='archived', claim_lock=NULL,
#         claim_expires=NULL, worker_pid=NULL), но без CALL на kernel-worker
#         tools. Так как скрипт запускается ОТ ОПЕРАТОРСКОГО ПРОЦЕССА (cron),
#         worker-scope не применяется.
#   3. Логировать каждое срабатывание.
#
# Whitelist — ДВОЙНАЯ ЗАЩИТА:
#   - assignee='devops' И board='robbox' (по умолчанию). Это профиль, который
#     имеет право запускать cross-task archive по процессным правилам (ретро
#     ADR-0023/0024).
#   - В теле задачи должны быть keywords: "PR #NNNN" или "issue #NNNN" — иначе
#     скрипт пропускает (нельзя автоматически решать за человека, что
#     делать с задачей, чьих PR/issue мы не знаем).
#   - Если ни PR, ни issue не закрыты — тоже skip (нельзя архивировать
#     задачу, чья работа ещё не закрыта).
#
# Cron: devops-профиль, every 1h. no_agent=True (скрипт самодостаточен).
#
# Usage:
#   cross-task-archive-sweeper.sh [--dry-run] [--stale-hours 24] [--limit 50]
#                                  [--board robbox] [--assignee devops]
#
# Env:
#   KANBAN_BOARD        — board slug (default 'robbox')
#   KANBAN_DB           — direct path to kanban.db (overrides default)
#   GH_REPO             — owner/repo (default 'krikz/rob_box_project')
#   STALE_HOURS         — порог stale в часах (default 24)
#   SKIP_BRANCH_CHECK   — '1' отключить remote-branch probe (для debug)
#
# Exit codes:
#   0 — sweep выполнен (с архивацией или без)
#   1 — не удалось открыть БД (критическая проблема — нужно вмешательство)
#   2 — usage error
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

# _LIB_DIR_HERE — каноническое место скрипта (нужно для питоновских
# helper-ов _cross_task_archive_sweeper_*.py, source=helpers).
# Bash-формула совместима с тем, как её делают другие скрипты
# (см. agent-flow-completion-check.sh:75).
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# --- guards ----------------------------------------------------------------
# HERMES_HOME hardcoded = /home/builder/.hermes — тот же паттерн, что в
# agent-flow-completion-check.sh (по факту $HERMES_HOME в env может быть
# /home/builder/.hermes/profiles/<profile>/, что не подходит для абсолютного
# KANBAN_DB). Используем $HOME/.hermes/ для абсолютного пути.
export HOME=/home/builder
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"

KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
KANBAN_DB="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
STALE_HOURS="${STALE_HOURS:-24}"
LIMIT="${LIMIT:-50}"
ASSIGNEE="${ASSIGNEE:-devops}"
DRY_RUN=0
LOG_PREFIX="[cross-task-archive-sweeper]"
LOG_FILE="${LOG_FILE:-$HOME/.local/state/cross_task_archive_sweeper.log}"

mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true

log() {
    local ts
    ts="$(date -Iseconds)"
    printf '%s %s %s\n' "$LOG_PREFIX" "$ts" "$*" >&2
    printf '%s %s %s\n' "$LOG_PREFIX" "$ts" "$*" >> "$LOG_FILE" 2>/dev/null || true
}

usage() {
    grep -E '^# (Usage|Env|Exit)' "$0" | sed 's/^# //'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)   DRY_RUN=1; shift ;;
        --stale-hours) STALE_HOURS="$2"; shift 2 ;;
        --limit)     LIMIT="$2"; shift 2 ;;
        --board)     KANBAN_BOARD="$2"; KANBAN_DB="$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db"; shift 2 ;;
        --assignee)  ASSIGNEE="$2"; shift 2 ;;
        -h|--help)   usage 0 ;;
        *) log "unknown arg: $1"; usage 2 ;;
    esac
done

if [ ! -f "$KANBAN_DB" ]; then
    log "FATAL: KANBAN_DB not found: $KANBAN_DB"
    exit 1
fi

log "START dry_run=$DRY_RUN board=$KANBAN_BOARD assignee=$ASSIGNEE stale_h=$STALE_HOURS limit=$LIMIT"

# --- pre-flight gh auth ----------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
    log "FATAL: gh CLI not found in PATH"
    exit 1
fi
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
    log "FATAL: gh not authenticated (check ~/.config/gh/hosts.yml)"
    exit 1
fi

# --- python helper: scan stuck cards --------------------------------------
# Возвращает JSON-lines список кандидатов с полями:
#   id, title, branch_name, started_at, prs[], issues[], refs[]
# refs[] — список ВСЕХ #NNNN найденных в body (с type=pr|issue|unknown).
SWEEP_OUTPUT="$(mktemp)"
trap 'rm -f "$SWEEP_OUTPUT"' EXIT

# Sweep helper (отдельный inline python script; читает argv, пишет JSON в $SWEEP_OUTPUT).
# Обёрнут в «если фейлится — логируем FATAL». Скрипт НЕ модифицирует tasks
# (read-only SELECT), всё архивирование делается далее через отдельный вызов.
if ! python3 "$_LIB_DIR_HERE/_cross_task_archive_sweeper_scan.py" \
        "$KANBAN_DB" "$ASSIGNEE" "$STALE_HOURS" "$LIMIT" > "$SWEEP_OUTPUT"; then
    log "FATAL: python sweep helper failed (see output above)"
    exit 1
fi

# --- parse + iterate -------------------------------------------------------
total=$(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(len(d['candidates']))" "$SWEEP_OUTPUT")
log "CANDIDATES: $total blocked+${ASSIGNEE} cards with stale>=${STALE_HOURS}h"

if [ "$total" -eq 0 ]; then
    log "END (no candidates)"
    exit 0
fi

# Iterate via xargs-friendly approach (no nested python with shell vars in source).
ARCHIVED=0
SKIPPED=0
NOT_RESOLVED=0
GITHUB_API_ERR=0

while IFS= read -r line; do
    cid="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['id'])")"
    refs_json="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.dumps(json.load(sys.stdin)['refs']))")"
    branch="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin).get('branch_name',''))")"
    stale_s="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin).get('stale_seconds',0))")"
    title="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin).get('title','')[:80])")"

    if [ "$refs_json" = "[]" ]; then
        log "SKIP  $cid (no #NNN refs in body; manual review) title=$title"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    # Проверить каждый ref через gh. Ищем ЛЮБОЙ закрытый (pr=MERGED/CLOSED
    # или issue=CLOSED) → если ни один не закрыт, не архивируем.
    closed_refs=()
    open_refs=()
    for n in $(printf '%s' "$refs_json" | python3 -c "import json,sys; print(' '.join(str(x) for x in json.load(sys.stdin)))"); do
        # Пробуем сначала PR, потом issue — порядок дешёвый, и при первом
        # успехе вторую проверку скипаем (но проверяем ОБА, т.к. может быть
        # один и тот же номер у issue/PR в редких случаях ребейза).
        pr_state=""
        issue_state=""

        pr_out="$(gh pr view "$n" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || true)"
        if [ -n "$pr_out" ]; then
            pr_state="$pr_out"
            case "$pr_state" in
                MERGED|CLOSED) closed_refs+=("$n:pr=$pr_state") ;;
                *) open_refs+=("$n:pr=$pr_state") ;;
            esac
            continue
        fi

        # Не нашли PR — пробуем issue
        issue_out="$(gh issue view "$n" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || true)"
        if [ -n "$issue_out" ]; then
            issue_state="$issue_out"
            case "$issue_state" in
                CLOSED) closed_refs+=("$n:issue=$issue_state") ;;
                *) open_refs+=("$n:issue=$issue_state") ;;
            esac
        else
            # 404 / network / permission — НЕ архивируем (fail-closed)
            log "WARN  $cid ref=#$n — gh returned nothing (404/rate-limit?) — skip card"
            open_refs+=("$n:unknown")
        fi
    done

    if [ "${#closed_refs[@]}" -eq 0 ]; then
        # Ни один ref не закрыт → нельзя архивировать.
        log "KEEP  $cid open_refs=${open_refs[*]:-} branch=$branch stale=${stale_s}s title=$title"
        NOT_RESOLVED=$((NOT_RESOLVED + 1))
        continue
    fi

    # Есть закрытые refs. Доп. проверка: remote-ветка orphan-state снят?
    branch_state=""
    if [ -n "$branch" ] && [ "${SKIP_BRANCH_CHECK:-0}" != "1" ]; then
        repo_url="$(gh repo view "$GH_REPO" --json nameWithOwner --jq '.nameWithOwner' 2>/dev/null || true)"
        if [ -n "$repo_url" ]; then
            if git ls-remote --heads "https://github.com/${repo_url}.git" "$branch" 2>/dev/null | grep -q "$branch"; then
                branch_state="present"
            else
                branch_state="absent"
            fi
        fi
    fi

    # Decision: архивируем, если у карточки есть ХОТЯ БЫ ОДИН закрытый ref И
    # (нет branch_name ИЛИ remote-ветка удалена).
    should_archive=0
    archive_reason=""

    if [ "${#closed_refs[@]}" -gt 0 ]; then
        if [ -z "$branch" ]; then
            should_archive=1
            archive_reason="closed_refs(${closed_refs[*]}) + no_branch"
        elif [ "$branch_state" = "absent" ]; then
            should_archive=1
            archive_reason="closed_refs(${closed_refs[*]}) + branch_absent"
        else
            archive_reason="closed_refs(${closed_refs[*]}) + branch_present(${branch}); will NOT archive without manual review"
        fi
    fi

    if [ "$should_archive" -eq 1 ]; then
        if [ "$DRY_RUN" = "1" ]; then
            log "DRY   $cid WOULD ARCHIVE — $archive_reason title=$title"
        else
            # Прямой SQL UPDATE — bypass worker-scope (см. ADR-0024).
            # Этот же row-update делает hermes_cli.kanban_db.archive_task.
            if arch_out="$(python3 "$_LIB_DIR_HERE/_cross_task_archive_sweeper_archive.py" \
                    "$KANBAN_DB" "$cid" 2>&1)"; then
                log "$arch_out | reason=$archive_reason title=$title"
                ARCHIVED=$((ARCHIVED + 1))
            else
                log "ERR   $cid — sql update failed: $arch_out"
            fi
        fi
    else
        log "KEEP  $cid $archive_reason title=$title"
        SKIPPED=$((SKIPPED + 1))
    fi

done < <(python3 -c "
import json,sys
data = json.load(open(sys.argv[1]))
for c in data['candidates']:
    print(json.dumps(c, ensure_ascii=False))
" "$SWEEP_OUTPUT")

log "END dry_run=$DRY_RUN candidates=$total archived=$ARCHIVED skipped=$SKIPPED not_resolved=$NOT_RESOLVED"
exit 0
