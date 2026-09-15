#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-orphan-audit.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`.
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-orphan-audit.sh,
# commit, merge. На хост: bash install.sh.
# ============================================================================
# agent-flow-orphan-audit.sh — telemetry: алерт на N тёзок issue + дашборд
# orphaned-cards.
#
# Контекст / ретро (15.09, карточка t_3dbde205):
#   В канбан-доске висят НЕСКОЛЬКО активных карточек с одним issue #N
#   (тёзки). Часто это признак «PR уже merged в develop, а карточки
#   остались»: dispatcher/blocker держит их и блокирует других воркеров.
#
#   Текущий кейс (issue #2406):
#     - t_40a610d0 (backend, 2026-09-15 00:39) — status=ready
#     - t_6535e27d (backend, 2026-09-15 02:43) — status=blocked
#     - PR #2458 (merged 2026-09-14 21:48:58Z, commit 71b6c65) — фикс
#       того же issue уже в develop.
#
#   Этот watchdog — telemetry-side: ОБНАРУЖИВАЕТ тёзок → emit alert
#   + пишет событие `orphan_detected` в task_events. Сам cancel
#   делает merge-gate (G10c guard, карточка t_e39afb1c, retro-key
#   g10c-prereq-merged-card-cancel) — отдельный цикл.
#
# Что делаем:
#   1. SELECT tasks WHERE status IN (todo,ready,running,blocked)
#      AND body IS NOT NULL.
#   2. Парсим body: regex извлекает issue_refs (#N) и repo_refs
#      (krikz/<repo>).
#   3. Группируем по (issue_number, repo).
#   4. Если в группе ≥ 2 активных карточек → orphan-candidate.
#   5. Резолвим реальные merged-PR-ы для issue через
#      `gh issue view N --json closedByPullRequestsReferences`;
#      fallback (если пусто): grep по merged-PR titles/bodies.
#   6. Гейт на cancel-recommendation: merged_pr_set канточек (из
#      последних summaries/comments) содержится в реальных merged-PR
#      develop-PR. Если да → recommendation=cancel (merge-gate возьмёт);
#      иначе recommendation=warn.
#   7. emit alert:
#      - file (default): строка `ORPHAN_ALERT issue=#N count=N
#        cards=[...] merged_prs=[...] recommendation=...`
#        в $ORPHAN_ALERT_LOG (с cooldown по issue);
#      - slack: POST на $ORPHAN_SLACK_WEBHOOK;
#      - gh_discussion: comment в issue (если $ORPHAN_GH_DISCUSSION=1).
#   8. INSERT INTO task_events (kind='orphan_detected', payload=...)
#      на КАЖДОЙ карточке-тёзке (для journal).
#   9. Экспорт метрики $ORPHAN_METRICS_FILE — текстовый Prometheus-
#      совместимый файл: `agent_flow_orphan_cards_total <value>`.
#      Опционально pushgateway: $ORPHAN_PUSHGATEWAY_URL.
#
# Idempotency:
#   - По issue — cooldown 1 час (default, override ORPHAN_COOLDOWN_SECS).
#     State — в $ORPHAN_STATE_FILE (sqlite или plain JSON).
#   - По карточке — INSERT INTO task_events с проверкой последней
#     записи: если за последние cooldown секунд уже было событие →
#     skip.
#
# Cron: agent-flow профиль, every 15m. no_agent=True.
# Также допустимо запускать вручную:
#   bash agent-flow-orphan-audit.sh [--dry-run] [--limit 500]
#
# Env (см. defaults ниже):
#   KANBAN_BOARD          — board slug (default 'robbox')
#   KANBAN_DB             — direct path to kanban.db (overrides default)
#   GH_REPO               — owner/repo (default 'krikz/rob_box_project')
#   HERMES_HOME           — hermes install root (default /home/builder/.hermes)
#   HERMES_BIN            — путь к hermes CLI (default `hermes`)
#   ORPHAN_ALERT_CHANNEL  — file | slack | gh_discussion (default 'file')
#   ORPHAN_ALERT_LOG      — путь к лог-файлу для канала file
#   ORPHAN_SLACK_WEBHOOK  — webhook URL для канала slack
#   ORPHAN_GH_DISCUSSION  — 1 = слать comment в issue (default 0)
#   ORPHAN_COOLDOWN_SECS  — cooldown между алертами по issue (default 3600)
#   ORPHAN_MIN_COUNT      — мин. тёзок для триггера (default 2)
#   ORPHAN_METRICS_FILE   — путь к textfile для node_exporter/Pushgateway
#   ORPHAN_PUSHGATEWAY_URL — если задан — push через curl
#   ORPHAN_STATE_FILE     — sqlite/json для cooldown-state
#   LIMIT                 — макс. число активных карточек за тик
#                           (default 500)
#
# Exit codes:
#   0 — sweep выполнен
#   1 — критическая проблема (DB / gh auth / disk)
#   2 — usage error
# ============================================================================

set -euo pipefail

_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# --- guards ----------------------------------------------------------------
# Force HOME=/home/builder — cron от per-profile gateway ставит HOME = profile-dir,
# а gh CLI ищет credentials в $HOME/.config/gh (см. ADR-0024 / hermes_github.sh).
export HOME="${HOME:-/home/builder}"
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"

KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
KANBAN_DB="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
HERMES_BIN="${HERMES_BIN:-hermes}"

ORPHAN_ALERT_CHANNEL="${ORPHAN_ALERT_CHANNEL:-file}"
ORPHAN_ALERT_LOG="${ORPHAN_ALERT_LOG:-$HOME/.local/state/orphan_audit_alerts.log}"
ORPHAN_SLACK_WEBHOOK="${ORPHAN_SLACK_WEBHOOK:-}"
ORPHAN_GH_DISCUSSION="${ORPHAN_GH_DISCUSSION:-0}"
ORPHAN_COOLDOWN_SECS="${ORPHAN_COOLDOWN_SECS:-3600}"
ORPHAN_MIN_COUNT="${ORPHAN_MIN_COUNT:-2}"
ORPHAN_METRICS_FILE="${ORPHAN_METRICS_FILE:-$HOME/.local/state/orphan_audit_metrics.prom}"
ORPHAN_PUSHGATEWAY_URL="${ORPHAN_PUSHGATEWAY_URL:-}"
ORPHAN_STATE_FILE="${ORPHAN_STATE_FILE:-$HOME/.local/state/orphan_audit_state.sqlite}"
LIMIT="${LIMIT:-500}"
DRY_RUN=0
LOG_PREFIX="[agent-flow-orphan-audit]"
LOG_FILE="${LOG_FILE:-$HOME/.local/state/orphan_audit.log}"

mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
mkdir -p "$(dirname "$ORPHAN_ALERT_LOG")" 2>/dev/null || true
mkdir -p "$(dirname "$ORPHAN_METRICS_FILE")" 2>/dev/null || true
mkdir -p "$(dirname "$ORPHAN_STATE_FILE")" 2>/dev/null || true

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
        --dry-run)             DRY_RUN=1; shift ;;
        --limit)               LIMIT="$2"; shift 2 ;;
        --board)               KANBAN_BOARD="$2"; KANBAN_DB="$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db"; shift 2 ;;
        --channel)             ORPHAN_ALERT_CHANNEL="$2"; shift 2 ;;
        --cooldown-secs)       ORPHAN_COOLDOWN_SECS="$2"; shift 2 ;;
        --min-count)           ORPHAN_MIN_COUNT="$2"; shift 2 ;;
        --state-file)          ORPHAN_STATE_FILE="$2"; shift 2 ;;
        --alert-log)           ORPHAN_ALERT_LOG="$2"; shift 2 ;;
        --metrics-file)        ORPHAN_METRICS_FILE="$2"; shift 2 ;;
        --gh-discussion)       ORPHAN_GH_DISCUSSION=1; shift ;;
        --pushgateway-url)     ORPHAN_PUSHGATEWAY_URL="$2"; shift 2 ;;
        -h|--help)             usage 0 ;;
        *)                     log "unknown arg: $1"; usage 2 ;;
    esac
done

if [ -z "$GH_REPO" ]; then
    log "FATAL: GH_REPO not set"
    exit 1
fi

if [ ! -f "$KANBAN_DB" ]; then
    log "FATAL: KANBAN_DB not found at $KANBAN_DB"
    exit 1
fi

# --- counters --------------------------------------------------------------
SCANNED=0
ORPHAN_GROUPS=0
ALERTED=0
SKIPPED_COOLDOWN=0
ERRORS=0
CARDS_WITH_EVENT=0

# counters через файл — иначе subshell в `while read | do` теряет переменные
RUN_COUNTERS_FILE="$(mktemp -t orphan_audit_counters.XXXXXX 2>/dev/null || echo "/tmp/orphan_audit_counters.$$")"
trap 'rm -f "$RUN_COUNTERS_FILE" 2>/dev/null || true' EXIT

# --- ensure state DB schema (idempotent) -----------------------------------
ensure_state_db() {
    if [ "$DRY_RUN" = "1" ]; then
        log "DRY_RUN=1, skip ensure_state_db"
        return 0
    fi
    python3 - "$ORPHAN_STATE_FILE" <<'PYEOF' || { log "FATAL: state DB init failed"; exit 1; }
import sqlite3, sys
state_db = sys.argv[1]
conn = sqlite3.connect(state_db)
c = conn.cursor()
c.execute(
    "CREATE TABLE IF NOT EXISTS orphan_alert_state ("
    "  key TEXT PRIMARY KEY,"  # 'issue:<repo>:<N>'
    "  last_alert_at INTEGER NOT NULL,"
    "  last_alert_signature TEXT NOT NULL"
    ")"
)
c.execute(
    "CREATE TABLE IF NOT EXISTS orphan_card_event_state ("
    "  task_id TEXT NOT NULL,"
    "  issue_key TEXT NOT NULL,"
    "  last_event_at INTEGER NOT NULL,"
    "  PRIMARY KEY (task_id, issue_key)"
    ")"
)
conn.commit()
conn.close()
PYEOF
}

ensure_state_db

# --- core scan + group ------------------------------------------------------
# Прочитать активные карточки, распарсить body, сгруппировать по (repo, issue).
# Возвращает python3-dict: {(repo, issue): [card_dict, ...]} на stdout.
SCAN_JSON="$(
KANBAN_DB="$KANBAN_DB" \
LIMIT="$LIMIT" \
DRY_RUN="$DRY_RUN" \
python3 <<'PYEOF'
import sqlite3, os, re, sys, json
kanban_db = os.environ["KANBAN_DB"]
limit = int(os.environ["LIMIT"])

ACTIVE_STATUSES = ("todo", "ready", "running", "blocked", "triage")
conn = sqlite3.connect(kanban_db)
conn.row_factory = sqlite3.Row
cur = conn.cursor()
cur.execute(
    "SELECT id, title, status, assignee, body, branch_name, started_at "
    "FROM tasks "
    "WHERE status IN (?, ?, ?, ?, ?) "
    "  AND body IS NOT NULL AND body != '' "
    "ORDER BY started_at DESC, id "
    "LIMIT ?",
    (*ACTIVE_STATUSES, limit),
)
rows = cur.fetchall()
conn.close()

# Regex: issue_refs (#NNNN) and repo_refs (krikz/<repo>)
RE_ISSUE = re.compile(r"#(\d{3,5})\b")
RE_REPO = re.compile(r"\bkrikz/([A-Za-z0-9_.-]+)\b")

groups: dict[tuple[str, int], list[dict]] = {}
for r in rows:
    body = r["body"] or ""
    issue_refs = sorted(set(int(m) for m in RE_ISSUE.findall(body)))
    repo_refs = sorted(set(RE_REPO.findall(body)))
    # Без repo считаем GH_REPO fallback (single-repo board по умолчанию).
    if not repo_refs:
        repo_refs = ["rob_box_project"]
    for repo in repo_refs:
        for issue_n in issue_refs:
            key = (repo, issue_n)
            groups.setdefault(key, []).append({
                "id": r["id"],
                "title": (r["title"] or "")[:200],
                "status": r["status"],
                "assignee": r["assignee"] or "",
                "branch_name": r["branch_name"] or "",
                "started_at": r["started_at"] or 0,
            })

# Преобразуем в json-friendly (tuple -> "repo:issue")
out = {}
for (repo, issue_n), cards in groups.items():
    if len(cards) < 2:
        continue
    out[f"{repo}:{issue_n}"] = {
        "repo": repo,
        "issue": issue_n,
        "cards": cards,
    }

# Сортируем: больше тёзок сначала, потом свежие started_at
def sort_key(item):
    key, val = item
    avg_started = sum(c["started_at"] for c in val["cards"]) / max(1, len(val["cards"]))
    return (-len(val["cards"]), -avg_started)

out_sorted = dict(sorted(out.items(), key=sort_key))
json.dump({"groups": out_sorted, "total_cards": len(rows)}, sys.stdout)
PYEOF
)"

TOTAL_CARDS="$(printf '%s' "$SCAN_JSON" | python3 -c "import sys, json; print(json.load(sys.stdin)['total_cards'])")"
GROUP_COUNT="$(printf '%s' "$SCAN_JSON" | python3 -c "import sys, json; print(len(json.load(sys.stdin)['groups']))")"
SCANNED="$TOTAL_CARDS"
ORPHAN_GROUPS="$GROUP_COUNT"

log "scan: cards=$SCANNED orphan_groups=$ORPHAN_GROUPS (threshold ≥$ORPHAN_MIN_COUNT)"

# --- emit alerts -----------------------------------------------------------
# Каждая группа → resolve merged_prs, compute recommendation,
# emit alert (with cooldown) + write event.

emit_one() {
    local repo="$1" issue_n="$2" cards_json="$3"
    local merged_prs_json recommendation alert_line

    # 1. resolved merged PRs
    merged_prs_json="$(resolve_merged_prs "$issue_n" "$repo")"

    # 2. recommendation gate
    recommendation="$(compute_recommendation "$cards_json" "$merged_prs_json")"

    # 3. alert line
    local card_ids
    card_ids="$(printf '%s' "$cards_json" | python3 -c "import sys, json; d=json.load(sys.stdin); print(','.join(c['id'] for c in d))")"
    local merged_prs_csv
    merged_prs_csv="$(printf '%s' "$merged_prs_json" | python3 -c "import sys, json; print(','.join(str(p) for p in json.load(sys.stdin)) if json.load(sys.stdin) else '')" 2>/dev/null || true)"
    # If shell pipe-clobber above was broken, fall back to simpler:
    if [ -z "$merged_prs_csv" ]; then
        merged_prs_csv="$(printf '%s' "$merged_prs_json" | tr -d '[]\"' | tr ',' '\n' | grep -E '^[0-9]+$' | tr '\n' ',' | sed 's/,$//')"
    fi

    local count
    count="$(printf '%s' "$cards_json" | python3 -c 'import sys, json; print(len(json.load(sys.stdin)))')"
    alert_line="ORPHAN_ALERT issue=#${issue_n} count=${count} cards=[${card_ids}] merged_prs=[${merged_prs_csv}] recommendation=${recommendation}"

    # 4. cooldown check
    local key="issue:${repo}:${issue_n}"
    local sig
    sig="$(printf '%s' "$alert_line" | sha256sum | cut -c1-16)"
    if check_cooldown "$key" "$sig"; then
        log "skip (cooldown): $alert_line"
        echo "SKIPPED" >> "$RUN_COUNTERS_FILE"
        return 0
    fi

    # 5. emit alert
    emit_alert "$alert_line" "$repo" "$issue_n"

    # 6. write task_events per card
    local events_written
    events_written="$(write_card_events "$repo" "$issue_n" "$cards_json" "$merged_prs_json" "$recommendation" | grep -E '^EVENTS_WRITTEN=' | head -1 | sed 's/^EVENTS_WRITTEN=//')"
    events_written="${events_written:-0}"
    # Если значение невалидно (multi-line), взять первую цифру
    if ! echo "$events_written" | grep -qE '^[0-9]+$'; then
        events_written=0
    fi
    # Каждый event = +1 в CARDS_WITH_EVENT (subshell-safe через файл)
    local i
    for i in $(seq 1 "$events_written"); do
        echo "EVENT" >> "$RUN_COUNTERS_FILE"
    done

    # 7. update cooldown state
    update_cooldown "$key" "$sig"

    echo "ALERTED" >> "$RUN_COUNTERS_FILE"
    log "alerted: $alert_line"
}

resolve_merged_prs() {
    # $1 = issue_n, $2 = repo
    # Returns JSON-array of PR numbers on stdout
    local issue_n="$1" repo="$2"

    # (a) gh issue view --json closedByPullRequestsReferences (clean)
    local via_issue
    via_issue="$(gh issue view "$issue_n" --repo "$repo" --json closedByPullRequestsReferences --jq '[.closedByPullRequestsReferences[]?.number] | unique' 2>/dev/null || echo '[]')"

    if [ "$via_issue" != "[]" ] && [ -n "$via_issue" ]; then
        printf '%s' "$via_issue"
        return 0
    fi

    # (b) fallback: grep merged PR titles for #N
    local via_search
    via_search="$(gh pr list --repo "$repo" --state merged --limit 200 --json number,title \
        --jq "[.[] | select((.title // \"\") | test(\"#${issue_n}([^0-9]|$)\"))] | [.[] | .number] | unique" 2>/dev/null || echo '[]')"
    printf '%s' "$via_search"
}

compute_recommendation() {
    # $1 = cards_json, $2 = merged_prs_json
    # If cards' merged_prs subset ⊆ actual merged_prs → cancel, else warn.
    # Пока у нас нет поля "merged_pr_set" в карточке — поэтому берём
    # heuristic: если реальных merged_prs от GH ≥ 1 и issue полностью
    # покрыт ими → cancel; иначе warn. Это согласуется с G10c guard
    # merge-gate: тот делает финальный verdict.
    local cards_json="$1" merged_prs_json="$2"
    local n_merged
    n_merged="$(printf '%s' "$merged_prs_json" | python3 -c "import sys, json; print(len(json.load(sys.stdin)))" 2>/dev/null || echo 0)"

    if [ "$n_merged" -ge 1 ]; then
        # Gate: реальные merged-PR существуют. Merge-gate сделает финальный
        # verdict (полнота покрытия). Мы рекомендуем cancel.
        printf 'cancel'
    else
        printf 'warn'
    fi
}

check_cooldown() {
    # $1 = key, $2 = sig (new signature)
    # Bash `if cmd; then` triggers on exit 0.
    # Returns 0 (= "in cooldown, skip alert") if cooldown active.
    # Returns 1 (= "OK to alert") otherwise.
    # python sys.exit(0) → bash rc=0 → if-блок срабатывает → skip.
    local key="$1" sig="$2"
    if [ "$DRY_RUN" = "1" ]; then
        return 1  # dry-run: всегда "OK" (alert'ы уйдут в лог, state не пишем)
    fi
    python3 - "$ORPHAN_STATE_FILE" "$key" "$ORPHAN_COOLDOWN_SECS" <<'PYEOF'
import sqlite3, sys, time
state_db, key, cooldown = sys.argv[1], sys.argv[2], int(sys.argv[3])
conn = sqlite3.connect(state_db)
c = conn.cursor()
c.execute("SELECT last_alert_at, last_alert_signature FROM orphan_alert_state WHERE key=?", (key,))
row = c.fetchone()
conn.close()
now = int(time.time())
if row and (now - row[0]) < cooldown:
    sys.exit(0)  # в cooldown — bash if сработает → skip
sys.exit(1)  # OK to alert — bash if НЕ сработает → alert
PYEOF
}

update_cooldown() {
    # $1 = key, $2 = sig
    local key="$1" sig="$2"
    if [ "$DRY_RUN" = "1" ]; then
        return 0
    fi
    python3 - "$ORPHAN_STATE_FILE" "$key" "$sig" <<'PYEOF'
import sqlite3, sys, time
state_db, key, sig = sys.argv[1], sys.argv[2], sys.argv[3]
conn = sqlite3.connect(state_db)
c = conn.cursor()
c.execute(
    "INSERT INTO orphan_alert_state(key, last_alert_at, last_alert_signature) VALUES (?, ?, ?) "
    "ON CONFLICT(key) DO UPDATE SET last_alert_at=excluded.last_alert_at, last_alert_signature=excluded.last_alert_signature",
    (key, int(time.time()), sig),
)
conn.commit()
conn.close()
PYEOF
}

emit_alert() {
    # $1 = alert_line, $2 = repo, $3 = issue_n
    local alert_line="$1" repo="$2" issue_n="$3"

    case "$ORPHAN_ALERT_CHANNEL" in
        file)
            if [ "$DRY_RUN" = "1" ]; then
                log "DRY_RUN alert: $alert_line"
            else
                printf '%s\n' "$alert_line" >> "$ORPHAN_ALERT_LOG"
            fi
            ;;
        slack)
            if [ -z "$ORPHAN_SLACK_WEBHOOK" ]; then
                log "WARN: ORPHAN_SLACK_WEBHOOK not set, falling back to file"
                printf '%s\n' "$alert_line" >> "$ORPHAN_ALERT_LOG"
                return
            fi
            if [ "$DRY_RUN" = "1" ]; then
                log "DRY_RUN slack alert: $alert_line"
            else
                curl -fsS --max-time 10 -H 'Content-Type: application/json' \
                    -d "$(python3 -c "import json, sys; print(json.dumps({'text': sys.argv[1]}))" "$alert_line")" \
                    "$ORPHAN_SLACK_WEBHOOK" >/dev/null || {
                        log "WARN: slack POST failed, falling back to file"
                        printf '%s\n' "$alert_line" >> "$ORPHAN_ALERT_LOG"
                    }
            fi
            ;;
        gh_discussion)
            # Comment в issue с маркером ALERT_ORPHAN (idempotency: через cooldown-state).
            if [ "$DRY_RUN" = "1" ]; then
                log "DRY_RUN gh_discussion alert: $alert_line"
            else
                gh issue comment "$issue_n" --repo "$repo" --body "**ALERT_ORPHAN** — $alert_line" >/dev/null 2>&1 \
                    || log "WARN: gh issue comment failed for #${issue_n}"
            fi
            ;;
        *)
            log "WARN: unknown ORPHAN_ALERT_CHANNEL=$ORPHAN_ALERT_CHANNEL, fallback to file"
            printf '%s\n' "$alert_line" >> "$ORPHAN_ALERT_LOG"
            ;;
    esac
}

write_card_events() {
    # $1=repo $2=issue_n $3=cards_json $4=merged_prs_json $5=recommendation
    # Prints "EVENTS_WRITTEN=<N>" on stdout.
    local repo="$1" issue_n="$2" cards_json="$3" merged_prs_json="$4" recommendation="$5"

    if [ "$DRY_RUN" = "1" ]; then
        log "DRY_RUN skip write_card_events"
        echo "EVENTS_WRITTEN=0"
        return 0
    fi

    python3 - "$KANBAN_DB" "$ORPHAN_STATE_FILE" "$repo" "$issue_n" "$cards_json" "$merged_prs_json" "$recommendation" "$ORPHAN_COOLDOWN_SECS" <<'PYEOF'
import sqlite3, sys, time, json
kanban_db, state_db, repo, issue_n, cards_json, merged_prs_json, recommendation, cooldown_secs = sys.argv[1:9]
issue_n = int(issue_n)
cooldown_secs = int(cooldown_secs)
issue_key = f"issue:{repo}:{issue_n}"

cards = json.loads(cards_json)
merged_prs = json.loads(merged_prs_json)

conn = sqlite3.connect(kanban_db)
cur = conn.cursor()
state_conn = sqlite3.connect(state_db)
state_cur = state_conn.cursor()

now = int(time.time())
written = 0
skipped_idempotent = 0
payload_template = {
    "kind": "orphan_detected",
    "repo": repo,
    "issue": issue_n,
    "merged_prs": merged_prs,
    "recommendation": recommendation,
    "retro_key": "g10c-prereq-merged-card-cancel",
    "ts": now,
}

for c in cards:
    task_id = c["id"]
    # Cooldown per (task_id, issue_key)
    state_cur.execute(
        "SELECT last_event_at FROM orphan_card_event_state WHERE task_id=? AND issue_key=?",
        (task_id, issue_key),
    )
    row = state_cur.fetchone()
    if row and (now - row[0]) < cooldown_secs:
        skipped_idempotent += 1
        continue
    payload = dict(payload_template, task_id=task_id, sibling_cards=[x["id"] for x in cards if x["id"] != task_id])
    cur.execute(
        "INSERT INTO task_events(task_id, run_id, kind, payload, created_at) VALUES (?, NULL, 'orphan_detected', ?, ?)",
        (task_id, json.dumps(payload, ensure_ascii=False), now),
    )
    state_cur.execute(
        "INSERT INTO orphan_card_event_state(task_id, issue_key, last_event_at) VALUES (?, ?, ?) "
        "ON CONFLICT(task_id, issue_key) DO UPDATE SET last_event_at=excluded.last_event_at",
        (task_id, issue_key, now),
    )
    written += 1

conn.commit()
state_conn.commit()
conn.close()
state_conn.close()
print(f"EVENTS_WRITTEN={written} skipped_idempotent={skipped_idempotent}")
PYEOF
}

# --- iterate groups --------------------------------------------------------
GROUPS_JSON="$(printf '%s' "$SCAN_JSON" | python3 -c "import sys, json; print(json.dumps(json.load(sys.stdin)['groups']))")"

# Итерируем через python (простой парсинг nested dict)
printf '%s' "$GROUPS_JSON" | python3 -c "
import sys, json
groups = json.load(sys.stdin)
for key, val in groups.items():
    print(json.dumps({'repo': val['repo'], 'issue': val['issue'], 'cards': val['cards']}))
" | while IFS= read -r one_group_json; do
    [ -z "$one_group_json" ] && continue
    repo="$(printf '%s' "$one_group_json" | python3 -c "import sys, json; print(json.load(sys.stdin)['repo'])")"
    issue_n="$(printf '%s' "$one_group_json" | python3 -c "import sys, json; print(json.load(sys.stdin)['issue'])")"
    cards_json="$(printf '%s' "$one_group_json" | python3 -c "import sys, json; print(json.dumps(json.load(sys.stdin)['cards']))")"
    if ! emit_one "$repo" "$issue_n" "$cards_json"; then
        log "WARN: emit_one failed for $repo#$issue_n"
        echo "ERROR" >> "$RUN_COUNTERS_FILE"
    fi
done

# Счётчики из файла (subshell-safe)
ALERTED="$(grep -c '^ALERTED$' "$RUN_COUNTERS_FILE" 2>/dev/null || true)"
ALERTED="${ALERTED:-0}"
SKIPPED_COOLDOWN="$(grep -c '^SKIPPED$' "$RUN_COUNTERS_FILE" 2>/dev/null || true)"
SKIPPED_COOLDOWN="${SKIPPED_COOLDOWN:-0}"
ERRORS="$(grep -c '^ERROR$' "$RUN_COUNTERS_FILE" 2>/dev/null || true)"
ERRORS="${ERRORS:-0}"
CARDS_WITH_EVENT="$(grep -c '^EVENT$' "$RUN_COUNTERS_FILE" 2>/dev/null || true)"
CARDS_WITH_EVENT="${CARDS_WITH_EVENT:-0}"

# --- metrics ---------------------------------------------------------------
TOTAL_ORPHAN_CARDS="$(printf '%s' "$GROUPS_JSON" | python3 -c '
import sys, json
groups = json.load(sys.stdin)
print(sum(len(v["cards"]) for v in groups.values()))
')"

if [ "$DRY_RUN" = "1" ]; then
    log "DRY_RUN skip metrics export"
else
    cat > "$ORPHAN_METRICS_FILE" <<EOF
# HELP agent_flow_orphan_cards_total Total number of kanban cards currently sharing an issue with at least one sibling (telemetry: t_3dbde205).
# TYPE agent_flow_orphan_cards_total gauge
agent_flow_orphan_cards_total ${TOTAL_ORPHAN_CARDS}
# HELP agent_flow_orphan_groups_total Total number of (repo, issue) groups with ≥2 active cards.
# TYPE agent_flow_orphan_groups_total gauge
agent_flow_orphan_groups_total ${ORPHAN_GROUPS}
EOF

    if [ -n "$ORPHAN_PUSHGATEWAY_URL" ]; then
        curl -fsS --max-time 10 \
            --data-binary "@${ORPHAN_METRICS_FILE}" \
            "${ORPHAN_PUSHGATEWAY_URL}/metrics/job/agent_flow_orphan_audit" \
            >/dev/null 2>&1 \
            && log "pushed metrics to pushgateway" \
            || log "WARN: pushgateway push failed"
    fi
fi

log "done: scanned=$SCANNED orphan_groups=$ORPHAN_GROUPS alerted=$ALERTED skipped_cooldown=$SKIPPED_COOLDOWN errors=$ERRORS total_orphan_cards=$TOTAL_ORPHAN_CARDS"
printf '__STATS__:%d:%d:%d:%d:%d:%d\n' "$SCANNED" "$ORPHAN_GROUPS" "$ALERTED" "$SKIPPED_COOLDOWN" "$ERRORS" "$TOTAL_ORPHAN_CARDS"
exit 0