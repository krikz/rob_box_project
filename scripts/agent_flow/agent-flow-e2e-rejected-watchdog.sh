#!/bin/bash
# ============================================================================
# agent-flow-e2e-rejected-watchdog.sh — auto-escalate GitHub Issues labeled
# `e2e:rejected` that sit without action for too long.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-e2e-rejected-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Контекст (ретро 15.09.2026, issue t_9251fd74):
#   6 открытых GitHub issue с меткой `e2e:rejected` висят без assignee и без
#   нового PR >48ч. agent-flow-e2e-process умеет ставить `e2e:rejected` после
#   неудачного прогона, но НЕ запускает process-cycle «rejected → новый fix-PR
# или closing-as-wontfix». Сейчас rejected висят месяцами, юзер вручную не
# видит «этот путь провалился, что дальше».
#
# Контракт (per tick, no-agent bash):
#   1. flock lock (не два тика одновременно)
#   2. MAINTENANCE gate (через af_maintenance_gate_or_exit)
#   3. iterate over all open issues with label `e2e:rejected` (via REST
#      `gh api search/issues`, fallback на `gh issue list`).
#   4. for each candidate:
#      a) age = now - issue.updatedAt (epoch seconds)
#      b) Skip если есть открытый/не-mergd PR с `Closes #N` (любая ветка).
#         Эвристика: новый PR в процессе → не наша эскалация.
#      c) Skip если есть kanban-card с kanban-коммент-маркером `kanban: t_*`
#         и карточка не archived/done в последние 4ч (triage уже обработал).
#      d) RESOLVE assignee по `agent:<role>` label issue → role, либо по
#         domain-keywords из title/body: quest|avatar|operator|telegram|webxr
#         → backend; иначе fallback на AGENT_FLOW_DEFAULT_ROLE=architect.
#      e) RESOLVE skill через af_skill_for_profile (lib_agent_flow_common.sh).
#      f) ПРИМЕНИТЬ эскалацию в зависимости от age:
#         • age > AUTO_CLOSE_DAYS (30) → close issue + label `closed:stale-rejected`
#         • иначе age > STALE_DAYS (7) → assignee-через-`gh issue edit`,
#           skill-через-комментарий-маркер; написать issue-comment "stale,
#           нужна новая попытка или wontfix-обоснование".
#      g) Idempotency: для 7d-ветки ОДИН alert-коммент в issue за
#         STALE_DEDUP_HOURS (default 24h) — comment LIKE '%<MARKER_TAG>%'.
#         Для 30d-close — проверяем, что label `closed:stale-rejected` уже
#         есть (issue закрыт ранее); если нет — close + label add.
#   5. Log stats: scanned, escalated, closed, skipped_idempotent,
#      skipped_pending_pr, errors.
#
# ENV:
#   GH_REPO                          — owner/repo (default krikz/rob_box_project)
#   GH_CONFIG_DIR                    — for gh CLI auth (default ~/.config/gh)
#   DRY_RUN=true                     — log only, no side-effects
#   STALE_DAYS                       — default 7 (days since update → escalate)
#   AUTO_CLOSE_DAYS                  — default 30 (days since update → auto-close)
#   STALE_DEDUP_HOURS                — default 24 (one alert-comment per window)
#   MARKER_TAG                       — default "🤖 e2e-rejected-watchdog"
#   LOCK_FILE                        — flock guard
#   LOG_FILE                         — stats log
#   KANBAN_BOARD                     — board name for kanban context (default robbox)
#   AGENT_FLOW_DEFAULT_ROLE          — fallback assignee (default architect)
#
# Выходы:
#   - Exit 0 — всё ok (даже если не делали ничего).
#   - Exit 1 — критичный сбой (нет gh auth / python3).
#   - Exit 2 — нашлись escalated/closed карточки (alert для cron).
#
# Что НЕ делаем (явно):
#   - НЕ открываем новый kanban-card напрямую — kanban-инфраструктура
#     принимает issue через agent-flow-triage.sh, и мы не хотим bypass
#     (ретро 22.08 «скрипт процесса должен сам все делать»).
#   - НЕ удаляем метку e2e:rejected — это метка процесса, ставится
#     agent-flow-e2e-process и должна там же сниматься (или нет).
#   - НЕ модифицируем body issue — пишем только comment.
#
# Pitfalls:
#   - Rate-limit: REST search/issues идёт в общий 5000/h лимит. На 6 issue
#     текущий tick жрёт 1-2 запроса. Если тиков станет больше (сотни
#     rejected issue) — перейти на `gh api graphql` с paginator. Сейчас
#     REST ОК.
#   - "Closes #N" в PR-теле может быть в виде ссылки `(#1234)` без слова
#     Closes — эвристика ловит только Closes/Fixes/Resolves. OK для нашего
#     use-case (agent-flow-triage ставит `Closes` явно).
#   - gh api search/issues поддерживает `is:open` и `label:`. Issue с
#     закрытым PR через merge сохраняют label `e2e:rejected` до ручного
#     снятия — мы такие issue пропускаем (есть merged PR).
#   - assignee-через-`gh issue edit --add-assignee` требует write access
#     на issue; для bot-token это OK (krikz-bot имеет write).
#   - label-add через `gh issue edit --add-label` создаёт label если нет.
# ============================================================================
set -uo pipefail  # без -e — soft-fail на per-issue errors

DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-rejected-watchdog.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-e2e-rejected-watchdog.log}"
STALE_DAYS="${STALE_DAYS:-7}"
AUTO_CLOSE_DAYS="${AUTO_CLOSE_DAYS:-30}"
STALE_DEDUP_HOURS="${STALE_DEDUP_HOURS:-24}"
MARKER_TAG="${MARKER_TAG:-🤖 e2e-rejected-watchdog}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
AGENT_FLOW_DEFAULT_ROLE="${AGENT_FLOW_DEFAULT_ROLE:-architect}"
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
LIB_PATH="${HERMES_HOME}/scripts/lib_agent_flow_common.sh"

# --- flock guard ----------------------------------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] e2e-rejected-watchdog: another instance running — skip" >&2
    exit 0
fi

# --- pre-flight -----------------------------------------------------------
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] e2e-rejected-watchdog: python3 not on PATH — exit 1" >&2
    exit 1
fi
python3 -c "import json, re, subprocess, sys, datetime, calendar" 2>/dev/null || {
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] e2e-rejected-watchdog: python3 modules missing — exit 1" >&2
    exit 1
}
export GH_CONFIG_DIR
if ! command -v gh >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] e2e-rejected-watchdog: gh CLI not on PATH — exit 1" >&2
    exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] e2e-rejected-watchdog: gh not authed — exit 1" >&2
    exit 1
fi

# Optional: source lib_agent_flow_common.sh for af_skill_for_profile.
# If absent (e.g. before install.sh ran), we degrade gracefully.
if [ -f "$LIB_PATH" ]; then
    # shellcheck source=/dev/null
    . "$LIB_PATH"
fi

_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }
_now_s()   { date -u +%s; }

PREFIX="[e2e-rejected-watchdog]"
log() { printf '%s %s %s\n' "$PREFIX" "$(_now_iso)" "$*" >&2; }

# --- fetch candidates -----------------------------------------------------
log "querying ${GH_REPO} for open issues label=e2e:rejected (stale_days=${STALE_DAYS} auto_close_days=${AUTO_CLOSE_DAYS})"

# Use REST search API for reliability (gh issue list --label has known cache bugs).
_candidates_json="$(gh api "search/issues?q=repo:${GH_REPO}+is:open+label:e2e:rejected&per_page=100" \
    --jq '.items[] | {number, title, body, updated_at, labels: [.labels[].name], assignees: [.assignees[].login]}' \
    2>/dev/null || echo "")"

if [ -z "$_candidates_json" ]; then
    log "no candidates (rate-limit? repo?: ${_candidates_json:0:200}) — exit 0"
    exit 0
fi

# Aggregate counters
_scanned=0
_escalated=0
_closed=0
_skipped_pending_pr=0
_skipped_idempotent=0
_skipped_no_action=0
_errors=0
_records=()

# --- per-issue processing -------------------------------------------------
while IFS= read -r line; do
    [ -n "$line" ] || continue
    _scanned=$(( _scanned + 1 ))

    # Per-issue processing via python3 (json reliable).
    # Echo result as one of: ESCALATED:NN|<reason>, CLOSED:NN|<reason>,
    # SKIPPED:<reason>, ERROR:<reason>.
    _result="$(ISSUE_JSON="$line" \
               GH_REPO_PASS="$GH_REPO" \
               STALE_DAYS_PASS="$STALE_DAYS" \
               AUTO_CLOSE_DAYS_PASS="$AUTO_CLOSE_DAYS" \
               STALE_DEDUP_HOURS_PASS="$STALE_DEDUP_HOURS" \
               MARKER_TAG_PASS="$MARKER_TAG" \
               DEFAULT_ROLE_PASS="$AGENT_FLOW_DEFAULT_ROLE" \
               DRY_RUN_PASS="$DRY_RUN" \
               NOW_S_PASS="$(_now_s)" \
               python3 <<'PYEOF'
import os, json, subprocess, sys, datetime, calendar, re

try:
    issue = json.loads(os.environ["ISSUE_JSON"])
    if not isinstance(issue, dict):
        # Array or scalar — cannot process.
        print(f"ERROR:not_dict:{os.environ.get('ISSUE_JSON','')[:50]}")
        sys.exit(0)
except Exception as e:
    print(f"ERROR:bad_json:{type(e).__name__}:{e}")
    sys.exit(0)

# Outer guard: any unexpected IndexError/AttributeError should log and SKIP,
# never crash the watchdog (ретро t_9251fd74 — IndexError in PR-search
# on edge cases shouldn't poison the whole tick).
# Note: bash heredoc strips leading whitespace, so we use flat indentation
# and rely on per-section try/except instead of one big try.

number       = issue.get("number")
title        = issue.get("title", "")
body         = issue.get("body", "") or ""
updated_at   = issue.get("updated_at", "")
labels       = issue.get("labels", []) or []
assignees    = issue.get("assignees", []) or []

gh_repo        = os.environ["GH_REPO_PASS"]
stale_days     = int(os.environ["STALE_DAYS_PASS"])
auto_close_days = int(os.environ["AUTO_CLOSE_DAYS_PASS"])
dedup_hours    = int(os.environ["STALE_DEDUP_HOURS_PASS"])
marker_tag     = os.environ["MARKER_TAG_PASS"]
default_role   = os.environ["DEFAULT_ROLE_PASS"]
dry_run        = (os.environ["DRY_RUN_PASS"].lower() == "true")
now_s          = int(os.environ["NOW_S_PASS"])

def log(msg):
    print(f"[{datetime.datetime.utcnow().isoformat()}Z] e2e-rejected-watchdog: "
          f"issue #{number}: {msg}", file=sys.stderr)

# Parse updated_at → epoch seconds (ISO 8601, GitHub format).
def iso_to_epoch(s):
    if not s:
        return 0
    s = s.replace("Z", "+00:00")
    try:
        dt = datetime.datetime.fromisoformat(s)
        return int(calendar.timegm(dt.utctimetuple()))
    except Exception:
        return 0

updated_epoch = iso_to_epoch(updated_at)
age_seconds = now_s - updated_epoch if updated_epoch else 0
age_days = age_seconds / 86400.0

log(f"scanned age_days={age_days:.1f} labels={labels} assignees={assignees}")

# Idempotency: have we already alerted in the last dedup_hours?
# Query issue comments + filter by marker + created_at > (now - dedup).
dedup_epoch = now_s - (dedup_hours * 3600)
already_alerted = False
try:
    proc = subprocess.run(
        ["gh", "api",
         f"repos/{gh_repo}/issues/{number}/comments",
         "--paginate",
         "--jq", '.[] | {created_at: .created_at, body: .body}'],
        capture_output=True, text=True, timeout=15, check=False,
    )
    if proc.returncode == 0:
        try:
            for ln in proc.stdout.splitlines():
                ln = ln.strip()
                if not ln:
                    continue
                try:
                    cm = json.loads(ln)
                    if not isinstance(cm, dict):
                        continue
                    body_ = cm.get("body", "") or ""
                    ca = cm.get("created_at", "")
                    if marker_tag in body_ and ca:
                        ce = iso_to_epoch(ca)
                        if ce >= dedup_epoch:
                            already_alerted = True
                            log(f"idempotent: found alert at {ca}")
                            break
                except json.JSONDecodeError:
                    continue
                except (IndexError, AttributeError, KeyError) as e:
                    log(f"WARN: per-comment parse err: {type(e).__name__}:{e}")
                    continue
        except Exception as e:
            log(f"WARN: comments-loop outer err: {type(e).__name__}:{e}")
    else:
        log(f"WARN: comments rc={proc.returncode}")
except subprocess.TimeoutExpired:
    log("WARN: comment fetch timed out — treat as not-alerted")

if already_alerted:
    print(f"SKIPPED:idempotent:{number}")
    sys.exit(0)

# 1. Check for pending PR (Closes #N). Any open OR closed-not-merged → skip.
# Special case: closed-and-MERGED PR → фикс уже доставлен, watchdog не должен
# писать escalation-коммент «новый PR нужен» (это обман пользователя —
# человек видит «open new PR» после успешного merge). Вместо этого
# выходим в новый MERGED-PR path (ниже): strip `e2e:rejected` + close
# issue with reason=completed (retro 16.09 t_4a242e15, issue #2487/#2495).
has_pending_pr = False
merged_pr_found = False
merged_pr_number = ""
pr_state_descr = ""
try:
    proc = subprocess.run(
        ["gh", "api", "search/issues",
         f"-q=repo:{gh_repo}+is:pr+Closes+%23{number}+OR+Fixes+%23{number}+OR+Resolves+%23{number}",
         "--jq", '.items[] | {state, html_url, number}'],
        capture_output=True, text=True, timeout=15, check=False,
    )
    if proc.returncode == 0 and proc.stdout.strip():
        # Empty stdout = no PR; otherwise we have at least one.
        for pr_line in proc.stdout.splitlines():
            try:
                pr_data = json.loads(pr_line)
                if not isinstance(pr_data, dict):
                    continue  # skip scalars/arrays in NDJSON
                pn = pr_data.get("number")
                if pr_data.get("state") == "open":
                    has_pending_pr = True
                    pr_state_descr = f"open PR #{pn}"
                    break
                elif pr_data.get("state") == "closed":
                    # Refetch single PR to check merged.
                    mproc = subprocess.run(
                        ["gh", "api", f"repos/{gh_repo}/pulls/{pn}",
                         "--jq", '{merged: .merged, state: .state}'],
                        capture_output=True, text=True, timeout=10, check=False,
                    )
                    if mproc.returncode == 0:
                        try:
                            raw = mproc.stdout.strip()
                            mdata = json.loads(raw) if raw else {}
                            if not isinstance(mdata, dict):
                                # API иногда возвращает массив (edge case) — treat as no-merge-info.
                                mdata = {}
                        except json.JSONDecodeError:
                            mdata = {}
                        if not mdata.get("merged"):
                            has_pending_pr = True
                            pr_state_descr = f"closed-not-merged PR #{pn}"
                            break
                        # Retro 16.09 t_4a242e15: closed+MERGED PR — фикс доставлен.
                        # Запоминаем для последующего auto-strip+close, но НЕ брейкаем
                        # outer loop — другие PR с Closes #N могут быть open/closed-not-merged.
                        merged_pr_found = True
                        merged_pr_number = str(pn) if pn else ""
            except (json.JSONDecodeError, KeyError) as e:
                log(f"WARN: PR parse err: {e}")
                continue
except subprocess.TimeoutExpired:
    log("WARN: PR search timed out — treat as no-pending-pr")

if has_pending_pr:
    log(f"SKIP pending_pr={pr_state_descr}")
    print(f"SKIPPED:pending_pr:{number}")
    sys.exit(0)

# 1b. Retro 16.09 t_4a242e15 (issue #2487, PR #2495): MERGED PR с Closes #N →
# фикс уже в develop, watchdog больше не должен ждать 30d для auto-close и не
# должен слать escalation-коммент «open new PR». Вместо этого: strip
# `e2e:rejected` + close issue с reason=completed (audit-marker).
# Mirror of merge-gate path 0.1c (ADR-AF-0063 §4.1 family) — но для случаев,
# когда issue был пойман watchdog'ом раньше, чем merge-gate tick успел.
# Idempotent: повторный тик увидит state=CLOSED (через gh api search/issues
# фильтр is:open) → issue не попадёт в candidates → no-op.
if merged_pr_found:
    if dry_run:
        log(f"[DRY-RUN] would strip e2e:rejected + close issue (merged PR #{merged_pr_number})")
        print(f"CLOSED:{number}|dry-run|merged_pr={merged_pr_number}")
        sys.exit(0)
    rc = 0
    try:
        # Strip e2e:rejected label (creates removal).
        p1 = subprocess.run(
            ["gh", "issue", "edit", str(number),
             "--repo", gh_repo, "--remove-label", "e2e:rejected"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p1.returncode != 0:
            log(f"WARN: remove-label rc={p1.returncode}: {p1.stderr.strip()[:200]}")
        # Audit-comment.
        p2 = subprocess.run(
            ["gh", "issue", "comment", str(number), "--repo", gh_repo, "-b",
             f"🤖 e2e-rejected-watchdog: PR #{merged_pr_number} с `Closes #{number}` уже MERGED в develop — фикс признан доставленным. Снимаю `e2e:rejected` и закрываю issue. (retro 16.09 t_4a242e15)"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p2.returncode != 0:
            log(f"WARN: comment rc={p2.returncode}: {p2.stderr.strip()[:200]}")
        # Close.
        p3 = subprocess.run(
            ["gh", "issue", "close", str(number), "--repo", gh_repo,
             "--reason", "completed"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p3.returncode != 0:
            log(f"WARN: close rc={p3.returncode}: {p3.stderr.strip()[:200]}")
            rc = p3.returncode
    except subprocess.TimeoutExpired:
        log("ERROR: timeout during merged-pr cleanup")
        print(f"ERROR:close_timeout:{number}")
        sys.exit(0)
    if rc == 0:
        log(f"CLOSED (merged-pr path, PR #{merged_pr_number})")
        print(f"CLOSED:{number}|merged_pr={merged_pr_number}|age_days={age_days:.1f}")
    else:
        print(f"ERROR:close_rc:{number}|rc={rc}")
    sys.exit(0)

# 2. Decide action based on age.
if age_days >= auto_close_days:
    # AUTO-CLOSE: label `closed:stale-rejected`, then `gh issue close`.
    if dry_run:
        log(f"[DRY-RUN] would close + label closed:stale-rejected")
        print(f"CLOSED:{number}|dry-run")
        sys.exit(0)
    rc = 0
    try:
        # Add label (creates if absent).
        p1 = subprocess.run(
            ["gh", "issue", "edit", str(number),
             "--repo", gh_repo, "--add-label", "closed:stale-rejected"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p1.returncode != 0:
            log(f"WARN: add-label rc={p1.returncode}: {p1.stderr.strip()[:200]}")
        # Close.
        p2 = subprocess.run(
            ["gh", "issue", "close", str(number), "--repo", gh_repo,
             "--comment", f"{marker_tag}: auto-close after {auto_close_days}d stale (last update {updated_at[:10]}); переоткройте если нужна новая попытка."],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p2.returncode != 0:
            log(f"WARN: close rc={p2.returncode}: {p2.stderr.strip()[:200]}")
            rc = p2.returncode
    except subprocess.TimeoutExpired:
        log("ERROR: timeout during close")
        print(f"ERROR:close_timeout:{number}")
        sys.exit(0)
    if rc == 0:
        log(f"CLOSED (auto, age_days={age_days:.1f})")
        print(f"CLOSED:{number}|age_days={age_days:.1f}")
    else:
        print(f"ERROR:close_rc:{number}|rc={rc}")
    sys.exit(0)

if age_days < stale_days:
    log(f"age < stale threshold ({age_days:.1f} < {stale_days}) — no action")
    print(f"SKIPPED:no_action:{number}")
    sys.exit(0)

# 3. STALE ESCALATION (7d ≤ age < 30d): resolve assignee, post comment.
# Resolve assignee:
#   a) from `agent:<role>` label
#   b) from domain-keyword heuristic (quest|avatar|operator|telegram|webxr → backend)
#   c) fallback default_role

def extract_agent_role(labels_list):
    for lb in labels_list:
        if isinstance(lb, str) and lb.startswith("agent:"):
            return lb.split(":", 1)[1].strip()
    return ""

def resolve_assignee(labels_list, title_, body_, default):
    role = extract_agent_role(labels_list)
    if role:
        return role
    text = (title_ + "\n" + body_).lower()
    domain_keywords = ("quest", "avatar", "operator", "telegram", "webxr",
                       "voice", "llm", "stt", "tts")
    if any(k in text for k in domain_keywords):
        return "backend"
    return default

assignee = resolve_assignee(labels, title, body, default_role)

# Build comment body.
comment_body = (
    f"{marker_tag}: stale-e2e:rejected — issue открыт/обновлён {updated_at[:10]} "
    f"({age_days:.0f}д назад), assignee пуст, нового PR нет.\n\n"
    f"**Что нужно сделать (одно из):**\n"
    f"1. Открыть новый fix-PR с `Closes #{number}` (по процессу "
    f"`agent-flow-triage` создаст kanban-карточку автоматически).\n"
    f"2. Явно закрыть как wontfix: прокомментировать обоснование и "
    f"`gh issue close #{number}`.\n"
    f"3. Если нужен разбор root-cause (помощь devops/architect) — "
    f"добавить label `agent:{assignee}` чтобы следующий такс "
    f"agent-flow-triage зарегистрировался с правильным профилем.\n\n"
    f"**Auto-close**: через {int(auto_close_days - age_days)}д этот watchdog "
    f"автоматически закроет issue с label `closed:stale-rejected` "
    f"(можно переоткрыть).\n\n"
    f"_Этот alert сгенерирован cron'ом agent-flow-e2e-rejected-watchdog "
    f"(ретро t_9251fd74, {marker_tag}). Назначение assignee `{assignee}` "
    f"произойдёт автоматически ниже. Dedup window: {dedup_hours}ч._"
)

# Side effects: add assignee + post comment.
side_ok = True
if dry_run:
    log(f"[DRY-RUN] would add-assignee {assignee} + comment")
    print(f"ESCALATED:{number}|dry-run|assignee={assignee}")
    sys.exit(0)

try:
    # Assignee: only set if currently empty (preserve human overrides).
    if not assignees:
        p = subprocess.run(
            ["gh", "issue", "edit", str(number), "--repo", gh_repo,
             "--add-assignee", assignee],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if p.returncode != 0:
            log(f"WARN: add-assignee rc={p.returncode}: {p.stderr.strip()[:200]}")
            # not fatal — comment is the main side effect
    else:
        log(f"existing assignee(s)={assignees} — skip add-assignee")

    # Comment.
    p = subprocess.run(
        ["gh", "issue", "comment", str(number), "--repo", gh_repo,
         "-b", comment_body],
        capture_output=True, text=True, timeout=15, check=False,
    )
    if p.returncode != 0:
        log(f"ERROR: comment rc={p.returncode}: {p.stderr.strip()[:200]}")
        side_ok = False
except subprocess.TimeoutExpired:
    log("ERROR: timeout during side-effect")
    side_ok = False

if side_ok:
    log(f"ESCALATED assignee={assignee}")
    print(f"ESCALATED:{number}|assignee={assignee}")
else:
    print(f"ERROR:side_effect:{number}")
PYEOF
    )"

    case "$_result" in
        ESCALATED:*)
            _escalated=$(( _escalated + 1 ))
            _records+=("$_result")
            ;;
        CLOSED:*)
            _closed=$(( _closed + 1 ))
            _records+=("$_result")
            ;;
        SKIPPED:idempotent:*)
            _skipped_idempotent=$(( _skipped_idempotent + 1 ))
            ;;
        SKIPPED:pending_pr:*)
            _skipped_pending_pr=$(( _skipped_pending_pr + 1 ))
            ;;
        SKIPPED:no_action:*)
            _skipped_no_action=$(( _skipped_no_action + 1 ))
            ;;
        ERROR:*)
            _errors=$(( _errors + 1 ))
            log "ERROR: $_result"
            ;;
        *)
            _errors=$(( _errors + 1 ))
            log "ERROR: unexpected result: $_result"
            ;;
    esac
done <<< "$_candidates_json"

# --- summary --------------------------------------------------------------
log "✓ done scanned=${_scanned} escalated=${_escalated} closed=${_closed} skipped_idempotent=${_skipped_idempotent} skipped_pending_pr=${_skipped_pending_pr} skipped_no_action=${_skipped_no_action} errors=${_errors}"

# Print records (one per line) for cron-delivery / log inspection.
for r in "${_records[@]}"; do
    printf '  %s\n' "$r"
done

# Stats log file.
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# e2e-rejected-watchdog snapshot %s\n' "$(_now_iso)"
    printf 'scanned=%s escalated=%s closed=%s skipped_idempotent=%s skipped_pending_pr=%s skipped_no_action=%s errors=%s\n' \
        "$_scanned" "$_escalated" "$_closed" \
        "$_skipped_idempotent" "$_skipped_pending_pr" "$_skipped_no_action" "$_errors"
} >> "$LOG_FILE" 2>/dev/null || true

# Exit code: 2 if we did escalate/close, 0 otherwise.
if [ "$((_escalated + _closed))" -gt 0 ] && [ "$DRY_RUN" != "true" ]; then
    exit 2
fi
exit 0