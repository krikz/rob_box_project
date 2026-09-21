#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-needs-e2e-orphan-watchdog.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`.
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-needs-e2e-orphan-watchdog.sh,
# commit, merge. На хост: bash install.sh (или вручную cp + ln -sf).
# ============================================================================
# agent-flow-needs-e2e-orphan-watchdog.sh — periodic cleanup для «сирот»:
#   OPEN issues с label `needs-e2e`, для которых не ожидается успешный
#   e2e run в обозримом будущем. 4 кейса:
#
#     (A) MERGED PR существует (issue закрыт кодом, но triage снова
#         навесил `needs-e2e` после orphan-cleanup) → close + archive card.
#     (B) OPEN PR существует → skip (e2e-process / merge-gate разберутся).
#     (C) Нет ни OPEN ни MERGED PR за NEEDS_E2E_NO_PR_DAYS (default 7) →
#         close issue reason=not_planned + audit comment (НИКОГДА не трогаем
#         issues с меткой `e2e:rejected` — там human judgement).
#     (D) MERGED PR есть, но последний успешный develop e2e run старше
#         merge_date (или отсутствует) → relabel `needs-e2e` →
#         `needs-e2e:recheck-develop` (e2e-process подхватит и прогонит).
#
# Контекст / ретро:
#   t_78a6ffa3 (завершено devops 15.09) — сканирование показало 11 issues
#   с `needs-e2e` без open PR (8 never-had-PR, 3 PR merged but orphan-stale).
#   Триаж agent-flow-triage ставит `needs-e2e` при отсутствии PASS-доказательств
#   и не проверяет, был ли PR. Watchdog закрывает пробел: добавляет кейсы
#   (C) и (D), которые текущая логика (24.08 t_b3b81913) не покрывала.
#
#   Дополняет merge-gate.sh:archive_merged_card и e2e-process.sh recovery-loop.
#   Watchdog срабатывает даже если merge-gate пропустил из-за rate-limit /
#   transient error / mid-merge race.
#
# Стратегия безопасности:
#   - assignee=agent-flow + board=robbox по умолчанию.
#   - Идемпотентность: whoami-коммент (через hermes_github.sh) с action
#     marker проверяется на повтор за HERMES_WHOAMI_WINDOW_SECONDS (2h).
#   - DRY_RUN=1 → логируем действия, не выполняем side-effects.
#   - e2e:rejected → пропускаем в кейсе (C) всегда (human judgement).
#   - Если ни PR, ни issue-body не указывают на завершение — fail-closed:
#     лучше ничего не сделать, чем ошибочно закрыть.
#
# Cron: agent-flow профиль, every 4h. no_agent=True (скрипт самодостаточен).
# Также допустимо запускать вручную:
#   bash agent-flow-needs-e2e-orphan-watchdog.sh [--dry-run] [--limit 50]
#
# Env:
#   KANBAN_BOARD          — board slug (default 'robbox')
#   KANBAN_DB             — direct path to kanban.db (overrides default)
#   GH_REPO               — owner/repo (default 'krikz/rob_box_project')
#   HERMES_HOME           — hermes install root (default /home/builder/.hermes)
#   HERMES_BIN            — путь к hermes CLI (default `hermes`)
#   NEEDS_E2E_LABEL       — label для поиска (default 'needs-e2e')
#   NEEDS_E2E_NO_PR_DAYS  — порог «нет PR N дней» для кейса (C), default 7
#   NEEDS_E2E_RECHECK_LBL — label для кейса (D), default 'needs-e2e:recheck-develop'
#   E2E_REJECTED_LABEL    — метка-исключение для кейса (C), default 'e2e:rejected'
#   LIMIT                 — макс. число issues за тик (default 50)
#
# Exit codes:
#   0 — sweep выполнен (с cleanup или без)
#   1 — не удалось открыть GH / DB (критическая проблема)
#   2 — usage error
# ============================================================================
set -euo pipefail

_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# --- guards ----------------------------------------------------------------
# Force HOME=/home/builder — cron от per-profile gateway ставит HOME = profile-dir,
# а gh CLI ищет credentials в $HOME/.config/gh (см. ADR-0024 / hermes_github.sh).
export HOME="${HOME:-/home/builder}"
HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}"

KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
KANBAN_DB="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
HERMES_BIN="${HERMES_BIN:-hermes}"
NEEDS_E2E_LABEL="${NEEDS_E2E_LABEL:-needs-e2e}"
NEEDS_E2E_RECHECK_LBL="${NEEDS_E2E_RECHECK_LBL:-needs-e2e:recheck-develop}"
E2E_REJECTED_LABEL="${E2E_REJECTED_LABEL:-e2e:rejected}"
NEEDS_E2E_NO_PR_DAYS="${NEEDS_E2E_NO_PR_DAYS:-7}"
LIMIT="${LIMIT:-50}"
DRY_RUN=0
LOG_PREFIX="[needs-e2e-orphan-watchdog]"
LOG_FILE="${LOG_FILE:-$HOME/.local/state/needs_e2e_orphan_watchdog.log}"

mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true

log() {
    local ts
    ts="$(date -Iseconds)"
    printf '%s %s %s\n' "$LOG_PREFIX" "$ts" "$*" >&2
    printf '%s %s %s\n' "$LOG_PREFIX" "$ts" "$*" >> "$LOG_FILE" 2>/dev/null || true
}

usage() {
    grep -E '^# (Cron|Env|Exit|Usage)' "$0" | sed 's/^# //'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)         DRY_RUN=1; shift ;;
        --limit)           LIMIT="$2"; shift 2 ;;
        --board)           KANBAN_BOARD="$2"; KANBAN_DB="$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db"; shift 2 ;;
        --needs-e2e-label) NEEDS_E2E_LABEL="$2"; shift 2 ;;
        --no-pr-days)      NEEDS_E2E_NO_PR_DAYS="$2"; shift 2 ;;
        -h|--help)         usage 0 ;;
        *) log "unknown arg: $1"; usage 2 ;;
    esac
done

if [ -z "$GH_REPO" ]; then
    log "FATAL: GH_REPO not set"
    exit 1
fi

if [ ! -f "$KANBAN_DB" ]; then
    log "WARN: KANBAN_DB not found at $KANBAN_DB — карточки не будем искать, но issue cleanup продолжим (если PR merged)"
fi

log "START dry_run=$DRY_RUN board=$KANBAN_BOARD label=$NEEDS_E2E_LABEL no_pr_days=$NEEDS_E2E_NO_PR_DAYS limit=$LIMIT"

# --- pre-flight gh auth ----------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
    log "FATAL: gh CLI not found in PATH"
    exit 1
fi
export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
if [ -z "${GH_TOKEN:-}" ]; then
    GH_TOKEN="$(gh auth token 2>/dev/null || true)"
    [ -n "$GH_TOKEN" ] && export GH_TOKEN
fi
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
    log "FATAL: gh not authenticated (check ~/.config/gh/hosts.yml)"
    exit 1
fi

# Source whoami helper для self-id comments (см. ADR-0014 / hermes_github.sh).
if [ -f "$_LIB_DIR_HERE/hermes_github.sh" ]; then
    # shellcheck disable=SC1091
    . "$_LIB_DIR_HERE/hermes_github.sh"
    HERMES_AGENT_ROLE="${HERMES_AGENT_ROLE:-agent:agent-flow}"
    log "hermes_github.sh sourced (whoami=ON, role=$HERMES_AGENT_ROLE)"
else
    log "WARN: hermes_github.sh not found at $_LIB_DIR_HERE — whoami-comments будут пропущены"
    whoami_close_issue() { return 0; }
    whoami_add_label() { return 0; }
    whoami_remove_label() { return 0; }
fi

# --- 1. Получить список OPEN issues с label needs-e2e ---------------------
ISSUES_JSON="$(mktemp)"
trap 'rm -f "$ISSUES_JSON" "$ISSUES_JSON.ndjson"' EXIT

if ! gh api "repos/${GH_REPO}/issues?state=open&labels=$(printf %s "$NEEDS_E2E_LABEL" | sed 's/ /%20/g')&per_page=${LIMIT}" \
        > "$ISSUES_JSON" 2>/dev/null; then
    log "FATAL: gh api issues?labels=$NEEDS_E2E_LABEL failed (rate-limit / network?)"
    exit 1
fi

TOTAL=$(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(len(d) if isinstance(d,list) else 0)" "$ISSUES_JSON")
log "ISSUES: found $TOTAL open issues with label '$NEEDS_E2E_LABEL'"

if [ "$TOTAL" -eq 0 ]; then
    log "END (no candidates)"
    exit 0
fi

# --- 1a. Pre-compute last successful develop e2e run timestamp ------------
# Используется в кейсе (D): MERGED PR есть, но develop e2e последний раз
# успешно прогонялся СТАРШЕ merge_date (или ни разу) → recheck-develop.
# Workflow имя: "L: E2E Voice Test" (имя файла `.github/workflows/L-...yml`).
# Кэшируем в env-переменную для всех issues в этом тике.
LAST_E2E_SUCCESS_TS=""
if LAST_E2E_SUCCESS_TS="$(gh api "repos/${GH_REPO}/actions/runs?branch=develop&status=success&per_page=20" 2>/dev/null)"; then
    # Mock-friendly: реальный gh даёт JSON-объект {workflow_runs:[{name, updated_at}, ...]}.
    # Mock-gh без --jq возвращает упрощённый формат. Парсим через python.
    LAST_E2E_SUCCESS_TS="$(printf '%s' "$LAST_E2E_SUCCESS_TS" | python3 -c "
import json, sys
try:
    d = json.loads(sys.stdin.read() or '{}')
except Exception:
    sys.exit(0)
runs = d.get('workflow_runs', d) if isinstance(d, dict) else d
if not isinstance(runs, list):
    sys.exit(0)
for r in runs:
    if not isinstance(r, dict):
        continue
    name = r.get('name', '')
    if name.startswith('L: E2E Voice Test'):
        ts = r.get('updated_at') or r.get('created_at') or ''
        if ts:
            print(ts)
        break
" 2>/dev/null || true)"
fi
if [ -z "$LAST_E2E_SUCCESS_TS" ]; then
    log "WARN: cannot determine last successful develop e2e run (rate-limit?). Кейс (D) будет skip'ать."
fi
log "LAST_E2E_SUCCESS_TS=${LAST_E2E_SUCCESS_TS:-<unknown>}"

# --- 2. Для каждого issue найти PR и проверить состояние ------------------
CLOSED=0
RECHECK=0
SKIPPED=0
NOT_RESOLVED=0
GITHUB_API_ERR=0
CARD_CLOSED=0
CARD_NOT_FOUND=0

# parse JSON в NDJSON формат для построчного чтения
python3 -c "
import json,sys
data = json.load(open(sys.argv[1]))
for issue in data:
    if not isinstance(issue, dict):
        continue
    # Skip PRs (у них есть pull_request поле)
    if 'pull_request' in issue:
        continue
    out = {
        'number': issue.get('number'),
        'title': issue.get('title',''),
        'state': issue.get('state',''),
        'labels': ','.join(l.get('name','') for l in issue.get('labels', []) if isinstance(l, dict)),
        'body': issue.get('body','') or '',
        'created_at': issue.get('created_at','') or '',
    }
    print(json.dumps(out, ensure_ascii=False))
" "$ISSUES_JSON" > "$ISSUES_JSON.ndjson"

while IFS= read -r line; do
    [ -z "$line" ] && continue

    number="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['number'])")"
    title="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['title'][:80])")"
    labels="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['labels'])")"
    body="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['body'])")"
    created_at="$(printf '%s' "$line" | python3 -c "import json,sys; print(json.load(sys.stdin)['created_at'])")"

    # Quick sanity check — нужен ли нам этот issue вообще?
    if ! printf '%s' "$labels" | grep -q "$NEEDS_E2E_LABEL"; then
        log "SKIP  #$number label drift (no '$NEEDS_E2E_LABEL' anymore) title=$title"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    # Пропускаем issues с e2e:rejected — там human judgement, watchdog не трогает.
    if printf '%s' "$labels" | grep -q "$E2E_REJECTED_LABEL"; then
        log "SKIP  #$number label=$E2E_REJECTED_LABEL (human judgement, watchdog не трогает) title=$title"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    # --- 2a. Найти связанный PR -----------------------------------------
    pr_json=""
    if pr_json="$(gh pr list --repo "$GH_REPO" --state all --search "$number in:title" \
            --json number,title,state,mergedAt,mergeCommit 2>/dev/null)"; then
        :
    else
        log "WARN  #$number gh pr list --search failed (rate-limit?)"
        GITHUB_API_ERR=$((GITHUB_API_ERR + 1))
        pr_json=""
    fi

    # Фильтруем PR'ы, в чьём title реально фигурирует #NNNN (этот issue).
    relevant_prs="$(printf '%s' "$pr_json" | python3 -c "
import json, sys, re
try:
    data = json.loads(sys.stdin.read() or '[]')
except Exception:
    sys.exit(0)
target = int(sys.argv[1])
out = []
for pr in data:
    if not isinstance(pr, dict): continue
    body = str(pr.get('title','')) + ' #' + str(pr.get('number',''))
    if re.search(rf'#{target}\\b', body):
        out.append({
            'number': pr.get('number'),
            'state': pr.get('state'),
            'merged_at': pr.get('mergedAt'),
        })
print(json.dumps(out, ensure_ascii=False))
" "$number" 2>/dev/null)"

    merged_pr=""
    open_prs=""
    if [ -n "$relevant_prs" ] && [ "$relevant_prs" != "[]" ]; then
        merged_pr="$(printf '%s' "$relevant_prs" | python3 -c "
import json, sys
try:
    data = json.loads(sys.stdin.read() or '[]')
except Exception:
    sys.exit(0)
for pr in data:
    if not isinstance(pr, dict): continue
    if pr.get('state') == 'MERGED' or pr.get('merged_at'):
        print(pr.get('number','')); break
")"
        open_prs="$(printf '%s' "$relevant_prs" | python3 -c "
import json, sys
try:
    data = json.loads(sys.stdin.read() or '[]')
except Exception:
    sys.exit(0)
nums = [str(pr.get('number')) for pr in data
        if isinstance(pr, dict) and pr.get('state') == 'OPEN']
print(','.join(nums))
")"
    fi

    # --- 2b. Decision logic --------------------------------------------

    # Кейс (A): MERGED PR. Закрыть issue + cleanup card.
    if [ -n "$merged_pr" ]; then
        pr_meta="$(gh pr view "$merged_pr" --repo "$GH_REPO" \
            --json number,mergeCommit,mergedAt 2>/dev/null || echo '')"
        merge_sha="$(printf '%s' "$pr_meta" | python3 -c "
import json, sys
try:
    d = json.loads(sys.stdin.read() or '{}')
    sha = d.get('mergeCommit', {}).get('oid','') if isinstance(d.get('mergeCommit'), dict) else ''
    print(sha[:12])
except Exception: pass
" 2>/dev/null)"
        merge_date="$(printf '%s' "$pr_meta" | python3 -c "
import json, sys
try:
    d = json.loads(sys.stdin.read() or '{}')
    print(d.get('mergedAt','')[:10])
except Exception: pass
" 2>/dev/null)"

        # --- Кейс (D): MERGED PR + last_e2e_success < merge_date → recheck ---
        if [ -n "$LAST_E2E_SUCCESS_TS" ] && [ -n "$merge_date" ]; then
            merge_ts="$(date -d "$merge_date" +%s 2>/dev/null || echo 0)"
            last_e2e_ts="$(date -d "$(printf '%s' "$LAST_E2E_SUCCESS_TS" | head -c 10)" +%s 2>/dev/null || echo 0)"
            if [ "$merge_ts" -gt 0 ] && [ "$last_e2e_ts" -gt 0 ] \
                    && [ "$last_e2e_ts" -lt "$merge_ts" ]; then
                log "RECHECK #$number — PR #${merged_pr} MERGED ${merge_date}, но last_e2e_success=${LAST_E2E_SUCCESS_TS:0:10} старше (no run после merge) → relabel $NEEDS_E2E_RECHECK_LBL"
                if [ "$DRY_RUN" = "1" ]; then
                    log "DRY   #$number would: add-label $NEEDS_E2E_RECHECK_LBL"
                    SKIPPED=$((SKIPPED + 1))
                    continue
                fi
                if command -v whoami_add_label >/dev/null 2>&1; then
                    whoami_add_label "$number" "$NEEDS_E2E_RECHECK_LBL" \
                        "needs-e2e orphan: PR #${merged_pr} MERGED ${merge_date}, но develop e2e после merge не успешен — recheck required" \
                        2>/dev/null || true
                fi
                if gh issue edit "$number" --repo "$GH_REPO" \
                        --remove-label "$NEEDS_E2E_LABEL" \
                        --add-label "$NEEDS_E2E_RECHECK_LBL" >/dev/null 2>&1; then
                    RECHECK=$((RECHECK + 1))
                else
                    log "ERR   #$number gh issue edit add-label failed"
                    GITHUB_API_ERR=$((GITHUB_API_ERR + 1))
                fi
                continue
            fi
        fi

        # --- Кейс (A): обычный merged → close -------------------------------
        log "ORPHAN #$number — PR #${merged_pr} MERGED (sha=${merge_sha:-?}, merged=${merge_date:-?}) title=$title"

        if [ "$DRY_RUN" = "1" ]; then
            log "DRY   #$number would: comment + close (reason=completed) + cleanup kanban card"
            SKIPPED=$((SKIPPED + 1))
            continue
        fi

        if command -v whoami_close_issue >/dev/null 2>&1; then
            whoami_close_issue "$number" \
                "needs-e2e orphan cleanup: PR #${merged_pr} уже MERGED${merge_sha:+ (${merge_sha})}${merge_date:+ ${merge_date}} — issue закрывается reason=completed" \
                "branch=merged" "card=auto" 2>/dev/null || true
        fi

        if ! gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
            log "ERR   #$number gh issue close failed"
            GITHUB_API_ERR=$((GITHUB_API_ERR + 1))
            continue
        fi
        log "      #$number closed (reason=completed)"
        CLOSED=$((CLOSED + 1))

        # Archive kanban card если есть
        if [ -f "$KANBAN_DB" ]; then
            card_id="$(python3 - "$KANBAN_DB" "$number" <<'PYEOF' 2>/dev/null || true
import sqlite3, sys
db_path, issue_num = sys.argv[1], int(sys.argv[2])
try:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT id, status, body FROM tasks "
        "WHERE status IN ('blocked','done') "
        "AND (body LIKE ? OR body LIKE ?)",
        (f'%#{issue_num}%', f'%issue #{issue_num}%')
    ).fetchall()
    conn.close()
    if not rows:
        sys.exit(0)
    rows.sort(key=lambda r: 0 if r['status'] == 'blocked' else 1)
    print(rows[0]['id'])
except Exception:
    pass
PYEOF
)"
            if [ -n "$card_id" ]; then
                log "      #$number found card $card_id — closing (archive)"
                if python3 - "$KANBAN_DB" "$card_id" "$number" "$merged_pr" <<'PYEOF'
import sqlite3, sys
db_path, card_id, issue_num, pr_num = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
conn = sqlite3.connect(db_path)
cur = conn.cursor()
row = cur.execute("SELECT status FROM tasks WHERE id=?", (card_id,)).fetchone()
if not row:
    print(f"SKIP  {card_id} (not found)")
    conn.close()
    sys.exit(0)
status = row[0]
if status == 'archived':
    print(f"SKIP  {card_id} (already archived)")
    conn.close()
    sys.exit(0)
if status == 'done':
    cur.execute("UPDATE tasks SET status='archived', claim_lock=NULL, claim_expires=NULL, worker_pid=NULL WHERE id=?", (card_id,))
    conn.commit()
    print(f"ARCHIVED {card_id} (was done, issue #{issue_num} PR #{pr_num} merged)")
elif status == 'blocked':
    cur.execute("UPDATE tasks SET status='done', completed_at=?, claim_lock=NULL, claim_expires=NULL, worker_pid=NULL WHERE id=?",
                (int(__import__('time').time()), card_id))
    cur.execute("UPDATE tasks SET status='archived' WHERE id=?", (card_id,))
    cur.execute(
        "INSERT INTO task_comments (task_id, author, body, created_at) VALUES (?, ?, ?, ?)",
        (card_id, 'agent-flow-watchdog',
         f'needs-e2e orphan cleanup: issue #{issue_num} closed (PR #{pr_num} merged). '
         f'Card auto-archived by agent-flow-needs-e2e-orphan-watchdog (ретро t_78a6ffa3).',
         int(__import__('time').time())))
    conn.commit()
    print(f"ARCHIVED {card_id} (was blocked, issue #{issue_num} PR #{pr_num} merged)")
else:
    print(f"SKIP  {card_id} (status={status}, manual review)")
conn.close()
PYEOF
                then
                    CARD_CLOSED=$((CARD_CLOSED + 1))
                else
                    CARD_NOT_FOUND=$((CARD_NOT_FOUND + 1))
                fi
            else
                log "      #$number no related kanban card found in DB"
                CARD_NOT_FOUND=$((CARD_NOT_FOUND + 1))
            fi
        fi
    elif [ -n "$open_prs" ]; then
        # Кейс (B): OPEN PR — нормальная фаза.
        log "KEEP  #$number open_prs=$open_prs (in-flight, not orphan) title=$title"
        SKIPPED=$((SKIPPED + 1))
    else
        # Кейс (C): нет PR. Проверяем age (created_at).
        # Если issue создан < NEEDS_E2E_NO_PR_DAYS дней назад — ещё рано.
        # Если старше — close reason=not_planned.
        if [ -n "$created_at" ]; then
            issue_ts="$(date -d "$created_at" +%s 2>/dev/null || echo 0)"
            now_ts="$(date +%s)"
            age_days=0
            if [ "$issue_ts" -gt 0 ]; then
                age_days=$(( (now_ts - issue_ts) / 86400 ))
            fi
            if [ "$age_days" -lt "$NEEDS_E2E_NO_PR_DAYS" ]; then
                log "KEEP  #$number no PR, but age=${age_days}d < ${NEEDS_E2E_NO_PR_DAYS}d (too young) title=$title"
                SKIPPED=$((SKIPPED + 1))
                continue
            fi
            log "ORPHAN-NO-PR #$number — нет ни OPEN ни MERGED PR, age=${age_days}d >= ${NEEDS_E2E_NO_PR_DAYS}d → close reason=not_planned title=$title"

            if [ "$DRY_RUN" = "1" ]; then
                log "DRY   #$number would: comment + close (reason=not_planned)"
                SKIPPED=$((SKIPPED + 1))
                continue
            fi

            if command -v whoami_close_issue >/dev/null 2>&1; then
                whoami_close_issue "$number" \
                    "needs-e2e orphan-no-pr cleanup: age=${age_days}d >= ${NEEDS_E2E_NO_PR_DAYS}d, ни OPEN ни MERGED PR не найдено — issue закрывается reason=not_planned" \
                    "branch=no_pr" "card=auto" 2>/dev/null || true
            fi

            if gh issue close "$number" --repo "$GH_REPO" --reason not_planned >/dev/null 2>&1; then
                CLOSED=$((CLOSED + 1))
                log "      #$number closed (reason=not_planned, age=${age_days}d)"
            else
                log "ERR   #$number gh issue close (not_planned) failed"
                GITHUB_API_ERR=$((GITHUB_API_ERR + 1))
            fi
        else
            log "KEEP  #$number no PR, no created_at — manual review title=$title"
            NOT_RESOLVED=$((NOT_RESOLVED + 1))
        fi
    fi
done < "$ISSUES_JSON.ndjson"

log "END dry_run=$DRY_RUN closed=$CLOSED recheck=$RECHECK skipped=$SKIPPED not_resolved=$NOT_RESOLVED api_err=$GITHUB_API_ERR cards_archived=$CARD_CLOSED cards_unresolved=$CARD_NOT_FOUND"
exit 0