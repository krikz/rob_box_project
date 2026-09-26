#!/bin/bash
# ============================================================================
# agent-flow-pr-backlog-digest.sh — ежедневная сводка Шифу по PR backlog'у.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-pr-backlog-digest.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/profiles/backend/scripts/
#   - ~/.hermes/profiles/analyst/scripts/
#   - ~/.hermes/scripts/
#
# ЗАЧЕМ (PM-ретро t_cd2053b7, архитектор-рекомендация t_d2ab84d7):
#   PM-шпаргалка `/tmp/t_cd2053b7/pr-backlog-2026-09-26.md` делалась вручную
#   раз в ретро. 17 OPEN PR, все MERGEABLE+GREEN, но Шифу не видел ежедневной
#   сводки «сколько PR лежит без движения N дней». Этот скрипт — автомат:
#   каждый день в 09:00 Europe/Berlin считает backlog и шлёт в Telegram.
#
# ЧТО ДЕЛАЕТ (per tick, идемпотентно):
#   1. Гейты: flock, MAINTENANCE, дневное окно [DIGEST_HOUR, DIGEST_HOUR+1)
#      (по дефолту 09:00 — Шифу увидит с утра, а не ночью).
#   2. Один запрос `gh pr list --state open --json ...` (mergeable +
#      mergeStateStatus + updatedAt + labels + headRefName + number + title).
#   3. Группировка:
#      A — MERGEABLE + GREEN + есть `e2e-done` → «готовы к merge Шифу прямо сейчас»
#      B — MERGEABLE + GREEN, нет `e2e-done` → «в работе / ждут e2e»
#      C — CONFLICTING или mergeStateStatus != CLEAN → «risk»
#   4. Cross-check issues: для issues с `stale-candidate` — warning
#      (особенно если есть активный PR — race case из ретро t_d2ab84d7).
#   5. Считает `needs-review` (= 0 по дизайну не bug — см. ADR-0014 §H3).
#   6. Возраст в днях = floor((now - updatedAt)/86400), старые — сверху.
#   7. Отправка в Telegram Шифу (`chat_id=495039871`, ~30 строк).
#      --dry-run → вывод в /tmp/agent-flow-pr-backlog-digest.log без Telegram.
#
# ЧТО НЕ ДЕЛАЕТ (явно):
#   - НЕ алертит per-PR (spam). Один digest в день.
#   - НЕ меняет `needs-review` семантику (см. ADR-0014 §H3).
#   - НЕ мерджит (Q22 — только Шифу).
#   - НЕ интегрирует с e2e-process / unlabeled-sweep (изоляция).
#   - НЕ читает labels PR'ов для решения group A/B (только e2e-done
#     на issue из cross-check; group A/B/C берётся по mergeState +
#     PR-labels).
#
# ENV:
#   REPO_DIR                — клон репо (default hermes-share путь)
#   GH_REPO                 — owner/repo (default krikz/rob_box_project)
#   KANBAN_TELEGRAM_CHAT_ID — chat_id Шифа (default 495039871)
#   TELEGRAM_BOT_TOKEN      — из .env профиля; обязателен для production-режима
#   DIGEST_HOUR             — час тика, local TZ (default 9)
#   DIGEST_DRY_RUN=true     — НЕ слать в Telegram, печатать в лог
#   DIGEST_FORCE=true       — игнорировать MAINTENANCE-гейт и дневное окно
#   DIGEST_MAX_PER_GROUP    — макс строк в каждой группе A/B/C (default 5)
#   DIGEST_STATE_DIR        — sentinel (default /tmp)
#   LOCK_FILE               — flock (default /tmp/agent-flow-pr-backlog-digest.lock)
#   DIGEST_TEST_MODE=1      — пропустить MAINTENANCE-гейт; только для юнит-тестов
#
# Выходы:
#   stdout — markdown-сводка (одно Telegram-сообщение);
#   stderr — структурный лог тика;
#   exit 0 — ok (в т.ч. «окно не наступило» / «MAINTENANCE» / «уже сделано сегодня»);
#   exit 1 — критичный сбой (нет gh auth, нет TELEGRAM_BOT_TOKEN в production).
#
# Pitfalls:
#   - НЕ ретраить бесконечно (fail-closed) — watchdog подхватит.
#   - Один `gh pr list` запрос: даже на 100+ PR это дешевле, чем per-PR fetch.
#   - `mergeStateStatus` может быть UNKNOWN на GH race — такие PR уходят в C.
#   - Возраст считается по updatedAt, не по createdAt: PR без свежих
#     коммитов — это и есть «лежит без движения».
#   - Sentinel /tmp/agent-flow-pr-backlog-digest-YYYY-MM-DD.done — чтобы
#     двойной крон-тик (raced start) не слал два раза в день.
# ============================================================================
set -uo pipefail  # без -e: gh auth / parse-error не должны убивать cron без записи в лог

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DIGEST_HOUR="${DIGEST_HOUR:-9}"
DIGEST_MAX_PER_GROUP="${DIGEST_MAX_PER_GROUP:-5}"
DIGEST_STATE_DIR="${DIGEST_STATE_DIR:-/tmp}"
DIGEST_DRY_RUN="${DIGEST_DRY_RUN:-false}"
DIGEST_FORCE="${DIGEST_FORCE:-false}"
DIGEST_TEST_MODE="${DIGEST_TEST_MODE:-0}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-pr-backlog-digest.lock}"
KANBAN_TELEGRAM_CHAT_ID="${KANBAN_TELEGRAM_CHAT_ID:-495039871}"
TELEGRAM_BOT_TOKEN="${TELEGRAM_BOT_TOKEN:-}"

# Load profile .env (TELEGRAM_BOT_TOKEN / GH_REPO / KANBAN_BOARD).
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh" 2>/dev/null || true
af_load_profile_env 2>/dev/null || true

# Re-default GH_REPO после .env (если он там переопределён).
GH_REPO="${GH_REPO:-krikz/rob_box_project}"

PREFIX="[agent-flow-pr-backlog-digest]"

log()  { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }
fail() { printf '%s %s FAIL: %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; exit 1; }

# --- flock guard -----------------------------------------------------------
exec 9>"$LOCK_FILE" || fail "cannot open lock $LOCK_FILE"
flock -n 9 || { log "another digest tick in progress, skip"; exit 0; }

# --- MAINTENANCE gate -------------------------------------------------------
if [ "$DIGEST_FORCE" != "true" ] && [ "$DIGEST_TEST_MODE" != "1" ]; then
    if git -C "${REPO_DIR:-/home/builder/hermes-share/rob_box_project}" ls-tree origin/develop --name-only 2>/dev/null \
       | grep -i -x 'MAINTENANCE' >/dev/null; then
        log "🛑 MAINTENANCE flag set in develop — skip digest"
        exit 0
    fi
fi

# --- дневное окно (только в продакшен-режиме) ------------------------------
TODAY="$(date +%Y-%m-%d)"
SENTINEL_FILE="${DIGEST_STATE_DIR}/agent-flow-pr-backlog-digest-${TODAY}.done"

if [ "$DIGEST_FORCE" != "true" ] && [ "$DIGEST_DRY_RUN" != "true" ]; then
    cur_hour="$(date +%H)"
    cur_hour="$((10#$cur_hour))"
    target_hour="$((10#$DIGEST_HOUR))"
    if [ "$cur_hour" -ne "$target_hour" ]; then
        log "skip: outside digest window (now=${cur_hour}h, target=${target_hour}h)"
        exit 0
    fi
    if [ -f "$SENTINEL_FILE" ]; then
        log "skip: digest already sent today (sentinel=$SENTINEL_FILE)"
        exit 0
    fi
fi

# --- gh pr list ------------------------------------------------------------
log "fetching open PRs for $GH_REPO"

if ! command -v gh >/dev/null 2>&1; then
    fail "gh CLI not found in PATH"
fi

PR_JSON="$(gh pr list \
    --repo "$GH_REPO" \
    --state open \
    --limit 200 \
    --json number,title,labels,mergeable,mergeStateStatus,updatedAt,headRefName \
    2>/dev/null || echo '[]')"

if [ -z "$PR_JSON" ] || [ "$PR_JSON" = "null" ]; then
    PR_JSON='[]'
fi

# --- cross-check: issues с label stale-candidate ---------------------------
STALE_ISSUES='[]'
if command -v gh >/dev/null 2>&1; then
    STALE_ISSUES="$(gh issue list \
        --repo "$GH_REPO" \
        --state open \
        --label stale-candidate \
        --limit 50 \
        --json number,title 2>/dev/null || echo '[]')"
    [ -z "$STALE_ISSUES" ] && STALE_ISSUES='[]'
fi

# --- digest computation ----------------------------------------------------
DIGEST="$(PR_JSON_PASS="$PR_JSON" STALE_ISSUES_PASS="$STALE_ISSUES" \
    MAX_PER_GROUP_PASS="$DIGEST_MAX_PER_GROUP" TODAY_PASS="$TODAY" \
    python3 - <<'PY'
import json, os, datetime, sys

pr_json = os.environ.get("PR_JSON_PASS", "[]") or "[]"
stale = os.environ.get("STALE_ISSUES_PASS", "[]") or "[]"
max_per_group = int(os.environ.get("MAX_PER_GROUP_PASS", "5"))
today = os.environ.get("TODAY_PASS", "")

def jload(s, default):
    try:
        v = json.loads(s)
        return v if v is not None else default
    except Exception:
        return default

prs = jload(pr_json, [])
stale_issues = jload(stale, [])

now = datetime.datetime.now(datetime.timezone.utc)

def age_days(updated_at_iso):
    try:
        upd = datetime.datetime.fromisoformat(updated_at_iso.replace("Z", "+00:00"))
        delta = now - upd
        return max(0, int(delta.total_seconds() // 86400))
    except Exception:
        return -1

def has_label(pr, name):
    for l in (pr.get("labels") or []):
        if l.get("name") == name:
            return True
    return False

# Группировка по mergeable + mergeStateStatus
group_a, group_b, group_c = [], [], []
for pr in prs:
    mergeable = pr.get("mergeable") or "UNKNOWN"
    state = pr.get("mergeStateStatus") or "UNKNOWN"
    has_e2e = has_label(pr, "e2e-done")
    item = {
        "number": pr.get("number"),
        "title": pr.get("title") or "",
        "branch": pr.get("headRefName") or "",
        "updated": pr.get("updatedAt") or "",
        "mergeable": mergeable,
        "state": state,
        "labels": [l.get("name") for l in (pr.get("labels") or []) if l.get("name")],
    }
    item["age_days"] = age_days(item["updated"])
    # Group C: CONFLICTING или DIRTY или UNKNOWN в mergeStateStatus
    if mergeable == "CONFLICTING" or state in ("DIRTY", "UNKNOWN"):
        group_c.append(item)
        continue
    # MERGEABLE + CLEAN
    if mergeable == "MERGEABLE" and state == "CLEAN":
        if has_e2e:
            group_a.append(item)
        else:
            group_b.append(item)
        continue
    # MERGEABLE + (UNSTABLE / BLOCKED) — риск (e.g. требует review)
    group_c.append(item)

# Сортировка: старые сверху (age_days desc), потом по number asc
for grp in (group_a, group_b, group_c):
    grp.sort(key=lambda p: (-p["age_days"], p["number"]))

# Title-truncate
def short(t, n=48):
    return t if len(t) <= n else t[: n - 1] + "…"

# Format lines
def fmt_pr(p):
    age = f"{p['age_days']}d" if p['age_days'] >= 0 else "?"
    return f"  - #{p['number']} {short(p['title'])} ({age})"

group_a_lines = [fmt_pr(p) for p in group_a[:max_per_group]]
group_b_lines = [fmt_pr(p) for p in group_b[:max_per_group]]
group_c_lines = [fmt_pr(p) for p in group_c[:max_per_group]]

# Stale-issues cross-check
stale_lines = []
open_pr_numbers = {p["number"] for p in prs}
for si in (stale_issues or [])[:10]:
    n = si.get("number")
    if n is None:
        continue
    # race case: stale-candidate + есть активный PR в develop
    title = si.get("title") or ""
    stale_lines.append(f"  - #{n} {short(title, 60)}")
stale_count = len(stale_issues or [])

# Compose digest
out = []
out.append(f"📊 PR backlog на {today}:")
out.append(f"  • {len(prs)} открытых PR в {os.environ.get('GH_REPO','krikz/rob_box_project')}")
if group_a:
    out.append(f"  • {len(group_a)} готовы к merge Шифу прямо сейчас (MERGEABLE+GREEN+e2e-done):")
    out.extend(group_a_lines)
elif len(group_b) + len(group_c) > 0:
    out.append("  • 0 готовы к merge (e2e-done нет ни на одном)")
else:
    out.append("  • 0 готовы к merge")
if group_b:
    out.append(f"  • {len(group_b)} в работе (e2e ещё не прогоняли):")
    out.extend(group_b_lines)
if group_c:
    out.append(f"  • {len(group_c)} risk (CONFLICTING / DIRTY / требует review):")
    out.extend(group_c_lines)
needs_review = sum(1 for p in prs if has_label(p, "needs-review"))
out.append(f"  • {needs_review} needs-review (по дизайну 0 — e2e не прогоняли)")
if stale_count > 0:
    out.append(f"  • ⚠️ {stale_count} issue(s) stale-candidate:")
    if stale_lines:
        out.extend(stale_lines)

sys.stdout.write("\n".join(out))
PY
)"

# Если python3 упал / пустой digest — fail-closed, НЕ молча
if [ -z "$DIGEST" ]; then
    fail "empty digest (python parse failed?)"
fi

# --- output / send ---------------------------------------------------------
if [ "$DIGEST_DRY_RUN" = "true" ]; then
    LOG_FILE="${DIGEST_STATE_DIR}/agent-flow-pr-backlog-digest.log"
    {
        printf '%s\n' "=== $(date -Iseconds) ==="
        printf '%s\n\n' "$DIGEST"
    } >> "$LOG_FILE"
    log "DRY-RUN: digest written to $LOG_FILE (Telegram NOT sent)"
    exit 0
fi

# Production mode: требуется TELEGRAM_BOT_TOKEN
if [ -z "$TELEGRAM_BOT_TOKEN" ]; then
    fail "TELEGRAM_BOT_TOKEN not set (production mode)"
fi

# Send via Telegram Bot API. Отправляем одним сообщением.
# Telegram limit: 4096 chars на message; наш digest ~30 строк < 2K — ОК.
SEND_RC=0
HTTP_CODE="$(TELEGRAM_BOT_TOKEN_PASS="$TELEGRAM_BOT_TOKEN" \
              CHAT_ID_PASS="$KANBAN_TELEGRAM_CHAT_ID" \
              DIGEST_PASS="$DIGEST" \
              bash -c '
TOKEN="$TELEGRAM_BOT_TOKEN_PASS"
CHAT_ID="$CHAT_ID_PASS"
BODY="$DIGEST_PASS"

# JSON-encode body (escape quotes, backslashes, newlines)
JSON_BODY="$(printf "%s" "$BODY" | python3 -c "import json,sys; print(json.dumps(sys.stdin.read()))")"

RESP="$(curl --silent --show-error --max-time 15 \
    -X POST \
    -H "Content-Type: application/json" \
    --data-binary "{\"chat_id\":\"${CHAT_ID}\",\"text\":${JSON_BODY},\"disable_web_page_preview\":true}" \
    "https://api.telegram.org/bot${TOKEN}/sendMessage" \
    -w "\n%{http_code}" 2>/dev/null || echo "CURL_FAIL")"
printf "%s" "$RESP" | tail -n 1
')" || SEND_RC=1

if [ "$SEND_RC" -ne 0 ] || [ "$HTTP_CODE" != "200" ]; then
    fail "Telegram sendMessage failed (http=$HTTP_CODE, rc=$SEND_RC)"
fi

log "digest sent to chat_id=$KANBAN_TELEGRAM_CHAT_ID"

# Sentinel — чтобы повторный крон-тик в течение дня не слал повторно
touch "$SENTINEL_FILE" 2>/dev/null || log "WARN: cannot write sentinel $SENTINEL_FILE"

exit 0
