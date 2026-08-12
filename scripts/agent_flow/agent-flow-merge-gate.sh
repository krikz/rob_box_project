#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-merge-gate.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-merge-gate.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-merge-gate.sh
#   - ~/.hermes/scripts/agent-flow-merge-gate.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-merge-gate.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
# agent-flow-merge-gate.sh — Phase 3: PR is green -> add label `needs-e2e` to issue.
#
# Pure bash. No LLM. Idempotent. Driven entirely by env (see
# ~/.hermes/profiles/agent-flow/.env).
#
# Pipeline per tick (per AGENT_FLOW_PROPOSAL §3.3, instantiated per §3.2):
#   1. MAINTENANCE gate (remote + local) -> exit 0 if paused
#   2. gh auth check                     -> exit 1 if not authed
#   3. List open issues with label `hermes` (carry state of triage)
#   4. For each issue:
#        a. Skip if it already has `needs-e2e`, `e2e-done`, or `e2e:rejected`
#        b. Look up the kanban-card id from the `kanban: t_<id>` comment marker
#        c. Derive PR branch name: agent/<issue>-<slug>
#        d. Find the PR by head branch (idempotent — gh pr list is the lookup)
#        e. If statusCheckRollup is all SUCCESS and merge_state=clean and
#           the PR is OPEN against `develop` -> add label `needs-e2e`
#        f. Idempotency: skip if label already present (3a short-circuits)
#   5. flock lock prevents parallel ticks.
#
# Gates G2..G7 follow the table in agent-flow SKILL.md (G2 gh auth, G3 GH
# rate-limit, G6 flock sentinel). We never block on the dispatcher (G7).

set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
# NOTE: hardcode /home/builder/.hermes — cron from per-profile gateway sets
# HERMES_HOME to the profile dir; PROFILE_ENV would then point at a
# non-existent path and GH_REPO would never load (see agent-flow-triage.sh).
HERMES_HOME=/home/builder/.hermes
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

# Force HOME=/home/builder — see comments in agent-flow-triage.sh.
export HOME=/home/builder

ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
NEEDS_E2E_LABEL="${NEEDS_E2E_LABEL:-needs-e2e}"
NEEDS_REVIEW_LABEL="${NEEDS_REVIEW_LABEL:-needs-review}"
DONE_LABEL="${DONE_LABEL:-e2e-done}"
REJECTED_LABEL="${REJECTED_LABEL:-e2e:rejected}"
NO_E2E_LABEL="${NO_E2E_LABEL:-no-e2e-required}"
BIG_BANG_OVERRIDE_LABEL="${BIG_BANG_OVERRIDE_LABEL:-big-bang-override}"
# ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md): PR > 50
# commits OR > 3000 lines is forbidden without an explicit `big-bang-override`
# label on the issue. Шифу (товарищ) is the only one allowed to set it. We
# enforce the rule BOTH at merge-gate (before adding needs-e2e) AND at triage
# (before creating the card) so the worker can't even start a 100-commit job
# without the override. See also agent-flow-triage.sh `big_bang_check()`.
BIG_BANG_MAX_COMMITS="${BIG_BANG_MAX_COMMITS:-50}"
BIG_BANG_MAX_LINES="${BIG_BANG_MAX_LINES:-3000}"
DEVELOP_BRANCH="${DEVELOP_BRANCH:-develop}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
AGENT_FLOW_DEFAULT_ROLE="${AGENT_FLOW_DEFAULT_ROLE:-architect}"
DRY_RUN="${DRY_RUN:-false}"
ISSUE_LIMIT="${ISSUE_LIMIT:-50}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-merge-gate.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-merge-gate]}"

# --- source profile .env if present -------------------------------------------
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
    while IFS='=' read -r key val; do
        case "$key" in ''|\#*) continue ;; esac
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$PROFILE_ENV"
fi

# Defensive defaults (in case .env is partial).
: "${KANBAN_BOARD:=robbox}"
: "${MAINTENANCE_BRANCH:=develop}"
: "${MAINTENANCE_FILE:=MAINTENANCE}"
: "${REPO_DIR:=}"
: "${AGENT_FLOW_DEFAULT_ROLE:=architect}"
: "${DRY_RUN:=false}"
: "${ISSUE_LABEL:=hermes}"
: "${ISSUE_LIMIT:=50}"
: "${NEEDS_E2E_LABEL:=needs-e2e}"
: "${NEEDS_REVIEW_LABEL:=needs-review}"
: "${DONE_LABEL:=e2e-done}"
: "${REJECTED_LABEL:=e2e:rejected}"
: "${NO_E2E_LABEL:=no-e2e-required}"
: "${BIG_BANG_OVERRIDE_LABEL:=big-bang-override}"
: "${BIG_BANG_MAX_COMMITS:=50}"
: "${BIG_BANG_MAX_LINES:=3000}"
: "${DEVELOP_BRANCH:=develop}"

# --- helpers -----------------------------------------------------------------
log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
run() { if [ "$DRY_RUN" = "true" ]; then printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*" >&2; else eval "$@"; fi; }

# G6: flock sentinel — skip tick if another instance holds the lock.
exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
if ! flock -n 9; then
    log "another instance holds $LOCK_FILE — skip"; exit 0
fi

# --- G1: MAINTENANCE gate (remote + local) -----------------------------------
if [ -n "${GH_REPO:-}" ]; then
    remote_ref="${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}"
    if git ls-remote "https://github.com/${GH_REPO}.git" "$remote_ref" 2>/dev/null | grep -q .; then
        log "🛑 MAINTENANCE flag set on remote ${remote_ref} — skip"; exit 0
    fi
fi
if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
    if git -C "$REPO_DIR" show "${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}" >/dev/null 2>&1; then
        log "🛑 MAINTENANCE flag set locally in ${REPO_DIR} — skip"; exit 0
    fi
fi

# --- G2: gh auth check -------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    log "gh auth not configured — exit 1"; exit 1
fi

# --- required env ------------------------------------------------------------
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"

# --- pull open issues with `hermes` label -----------------------------------
issues_json="$(gh issue list \
    --repo "$GH_REPO" \
    --label "$ISSUE_LABEL" \
    --state open \
    --limit "$ISSUE_LIMIT" \
    --json number,title,labels,body 2>/dev/null || true)"

# G3: empty output is ambiguous — could be "no issues" OR "rate-limited".
if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
    rate="$(gh api rate_limit --jq '.resources.core.remaining' 2>/dev/null || echo 999)"
    if [ "${rate:-999}" = "0" ]; then
        log "GitHub rate-limit exhausted — skip tick"; exit 0
    fi
    # Ретро-путь (12.08 t_68607832): hermes-issues может не быть вовсе, но
    # смерженные PR, ссылающиеся на немаркированные issues, всё равно нужно
    # обработать (сканы ниже). Раньше здесь был exit 0 → ретро-issues навсегда
    # выпадали из merge-gate, когда очередь hermes пуста.
    log "no issues with label '${ISSUE_LABEL}' — continuing to scan-all-prs + retro-path"
    issues_json='[]'
fi

# --- shared helpers (kept compatible with triage.sh) -------------------------
slugify() {
    # lowercase, non-alnum -> -, collapse, trim, kebab-case, cap 40
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40
}

# Resolve predicate labels from a comma-joined label string.
has_label() {  # $1=labels_csv (lowercased) $2=label_name
    printf '%s' "$1" | tr ',' '\n' | grep -Fxq "$2"
}

# Detect PR kind: "lint" (no e2e needed) vs "functional" (e2e required).
# Signal sources (priority order):
#   1) PR label `${NO_E2E_LABEL}` → lint (explicit worker opt-out)
#   2) PR title prefix `[lint]` / `[refactor]` → lint (worker shorthand)
#   3) otherwise → functional (e2e mandatory)
# Inputs: $1=pr_labels_csv (lowercased), $2=pr_title
# Output: prints "lint" or "functional"; rc=0 always.
detect_pr_kind() {  # $1=labels_csv $2=title
    local labels_csv="$1" title="$2"
    if has_label "$labels_csv" "$NO_E2E_LABEL"; then
        printf '%s' "lint"; return 0
    fi
    case "$title" in
        '[lint]'*|'[refactor]'*) printf '%s' "lint"; return 0 ;;
    esac
    printf '%s' "functional"; return 0
}

# --- process each issue ------------------------------------------------------

# Free stale worktrees on the SAME branch as this card's workspace
# (kanban workspace_path aware — worktrees live in /home/builder/rob_box_project).
free_stale_worktrees_for() {  # $1=task_id (t_<hex>)
    local task_id="$1" my_wt my_branch line wt_path wt_branch owner
    my_wt="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$task_id" --json 2>/dev/null \
        | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); print(d.get("task",{}).get("workspace_path") or "")
except Exception: print("")' 2>/dev/null || true)"
    if [ -z "$my_wt" ] || [ ! -d "$my_wt" ]; then
        return 0
    fi
    my_branch="$(git -C "$my_wt" branch --show-current 2>/dev/null || true)"
    [ -z "$my_branch" ] && return 0
    wt_path=""
    while IFS= read -r line; do
        case "$line" in
            worktree\ *) wt_path="${line#worktree }" ;;
            branch\ *)
                wt_branch="${line#branch refs/heads/}"
                if [ "$wt_branch" = "$my_branch" ] && [ "$wt_path" != "$my_wt" ]; then
                    owner="$(basename "$wt_path")"
                    if [ "$owner" != "$task_id" ]; then
                        git -C "$my_wt" worktree remove --force "$wt_path" 2>/dev/null \
                            && log "  freed stale worktree $wt_path (branch $my_branch, card $owner)"
                    fi
                fi
                ;;
        esac
    done < <(git -C "$my_wt" worktree list --porcelain 2>/dev/null)
    git -C "$my_wt" worktree prune 2>/dev/null || true
    return 0
}

considered=0
labeled=0
skipped=0
errored=0

while IFS=$'\t' read -r number title labels body; do
    [ -z "$number" ] && continue
    considered=$((considered+1))

    labels_norm="$(printf '%s' "$labels" | tr '[:upper:]' '[:lower:]')"

    # Look up the kanban card id from the comment marker. We don't need the
    # card itself here, but its presence is the contract that Phase 1
    # finished and we have a branch convention to rely on.
    task_id="$(gh issue view "$number" --repo "$GH_REPO" --comments --json comments \
        --jq '.comments[].body' 2>/dev/null \
        | grep -Eo '^kanban: t_[a-f0-9]+' \
        | tail -n1 \
        | sed 's/^kanban: //' || true)"

    if [ -z "$task_id" ]; then
        log "issue #${number} has no kanban marker — triage not finished yet — skip"
        skipped=$((skipped+1)); continue
    fi

    # Derive the expected PR head branch.
    # Note: triage.sh uses branch `z-{agent}/<issue>-<slug>` for hermes+default-role
    # issues; service/infra labels get `z-{<slug>}` prefix. We only consider
    # `z-{agent}/<id>-<slug>` here because that's the path that goes through the
    # e2e pipeline. Service/infra branches are off-flow.
    labels_json="$labels"
    is_service=0
    if printf '%s' "$labels_json" | grep -Eq 'service:|infra:|ops:'; then
        is_service=1
    fi
    if [ "$is_service" -eq 1 ]; then
        log "issue #${number} is service/infra — out of merge-gate scope — skip"
        skipped=$((skipped+1)); continue
    fi

    # --- follow-up PR поверх e2e-done (ретро 10.08, архитектор) ------------
    # Раньше e2e-done «намертво» прилипал к issue: idempotency-скип ниже
    # пропускал её, и follow-up PR по той же issue (#1099 поверх #1082,
    # #1098 поверх #1052) навсегда выпадал из ротации — CLEAN PR висел
    # без needs-review/needs-e2e, фикс не доходил до робота.
    # Теперь: если issue e2e-done, но есть ДРУГОЙ OPEN PR с номером issue
    # в title (ветка ≠ канонической merged-ветке) — снимаем e2e-done →
    # needs-e2e, e2e-process возьмёт новую ветку в ротацию.
    # Guard от ping-pong: флипаем ТОЛЬКО если в follow-up PR есть коммиты
    # ПОСЛЕ момента навешивания e2e-done (иначе — уже протестирован, и
    # e2e-done → needs-e2e → e2e-done зациклится каждый тик).
    if has_label "$labels_norm" "$DONE_LABEL"; then
        _followup_json="$(gh pr list --repo "$GH_REPO" --state open \
            --search "${number} in:title" \
            --json number,headRefName,mergeStateStatus,updatedAt \
            --jq '[.[] | select(.mergeStateStatus == "CLEAN" or .mergeStateStatus == "MERGEABLE")][0]' 2>/dev/null || echo "")"
        _followup_pr="$(printf '%s' "$_followup_json" | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); print(d.get("number","") if d else "")
except Exception: print("")' 2>/dev/null || true)"
        if [ -n "$_followup_pr" ] && [ "$_followup_pr" != "null" ]; then
            _done_at="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
                --jq '[.[] | select(.event=="labeled" and .label.name=="'"$DONE_LABEL"'")][-1].created_at' 2>/dev/null || echo '')"
            if [ -z "$_done_at" ] || [ "$_done_at" = "null" ]; then
                log "issue #${number}: follow-up PR #${_followup_pr} найден, но timeline ${DONE_LABEL} недоступен — пропускаем тик (без риска ping-pong)"
                skipped=$((skipped+1)); continue
            fi
            _new_commits="$(gh api "repos/${GH_REPO}/pulls/${_followup_pr}/commits?per_page=100" \
                --jq '[.[] | select(.commit.committer.date >= "'"$_done_at"'")] | length' 2>/dev/null || echo 0)"
            if [ "${_new_commits:-0}" -gt 0 ] 2>/dev/null; then
                log "issue #${number}: follow-up OPEN PR #${_followup_pr} поверх ${DONE_LABEL} (new commits after ${_done_at}) — возврат в ротацию"
                if [ "$DRY_RUN" != "true" ]; then
                    gh issue edit "$number" --repo "$GH_REPO" --remove-label "$DONE_LABEL" >/dev/null 2>&1 || true
                    gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "agent-flow: 🔄 follow-up PR #${_followup_pr} поверх ${DONE_LABEL} — снят ${DONE_LABEL}, поставлен ${NEEDS_E2E_LABEL}. e2e-process протестирует новую ветку." >/dev/null 2>&1 || true
                fi
                labeled=$((labeled+1)); continue
            fi
            log "issue #${number}: follow-up PR #${_followup_pr} без новых коммитов после ${DONE_LABEL} (${_done_at}) — уже протестирован, skip"
        fi
    fi

    branch="z-{agent}/${number}-$(slugify "$title")"

    # Look up PR by head branch (authoritative — no extra state).
    # title + labels нужны для detect_pr_kind (lint vs functional).
    # additions + commits нужны для big-bang-override gate (ADR-0013).
    pr_json="$(gh pr list \
        --repo "$GH_REPO" \
        --state all \
        --head "$branch" \
        --json number,mergeable,mergeStateStatus,statusCheckRollup,baseRefName,state,mergedAt,title,labels,additions,commits 2>/dev/null || true)"

    # Процесс-фикс (09.08): воркеры ретро-карточек создают ветки `wt/<task_id>`
    # (нет issue → конвенция z-{agent}/<id>-<slug> неприменима). Такие PR
    # выпадали из конвейера: merge-gate не находил их и не ставил needs-e2e.
    # Fallback: ищем PR по ветке wt/<task_id> (последняя карточка issue).
    if [ -z "$pr_json" ] || [ "$pr_json" = "[]" ]; then
        if [ -n "$task_id" ]; then
            wt_branch="wt/${task_id}"
            pr_json="$(gh pr list \
                --repo "$GH_REPO" \
                --state all \
                --head "$wt_branch" \
                --json number,mergeable,mergeStateStatus,statusCheckRollup,baseRefName,state,mergedAt,title,labels,additions,commits 2>/dev/null || true)"
            if [ -n "$pr_json" ] && [ "$pr_json" != "[]" ]; then
                branch="$wt_branch"
                log "issue #${number}: PR найден по fallback-ветке ${wt_branch}"
            else
                pr_json=""
            fi
        fi
    fi

    if [ -z "$pr_json" ] || [ "$pr_json" = "[]" ]; then
        log "issue #${number}: no open PR for branch ${branch} yet — skip"
        skipped=$((skipped+1)); continue
    fi

    # Parse the single PR record (there's expected to be just one).
    eval "$(printf '%s' "$pr_json" | python3 -c '
import json, sys, shlex
data = json.load(sys.stdin)
if not data:
    sys.exit(0)
pr = data[0]
rollup = pr.get("statusCheckRollup") or []
# Green = no FAILURE-class conclusions; SKIPPED/NEUTRAL/pending are not red.
# (Integration Tests is often SKIPPED — that must NOT block the gate.)
no_failure = all(
    e.get("conclusion") not in ("FAILURE", "CANCELLED", "TIMED_OUT", "STALE")
    for e in rollup
)
all_pass = bool(rollup) and no_failure
pr_number = str(pr.get("number", ""))
pr_base = str(pr.get("baseRefName", ""))
pr_state = str(pr.get("state", ""))
pr_mergeable = str(pr.get("mergeable", ""))
pr_merge_state = str(pr.get("mergeStateStatus", ""))
pr_rollup_count = len(rollup)
pr_rollup_pass = 1 if all_pass and no_failure else 0
# title + labels_csv для detect_pr_kind (lint vs functional, ретро 10.08 #2)
pr_title = str(pr.get("title", ""))
pr_labels_csv = ",".join(sorted(
    {str(lab.get("name", "")) for lab in (pr.get("labels") or [])}
))
# Размер PR — для big-bang-override gate (ADR-0013). commits в API —
# это СПИСОК (не int), поэтому берём len(). additions — int напрямую.
# Если поле отсутствует (старый API), считаем 0 → gate не сработает
# (no PR, no override needed).
pr_commits_count = len(pr.get("commits") or [])
pr_additions = int(pr.get("additions") or 0)
print(f"pr_number={shlex.quote(pr_number)}")
print(f"pr_base={shlex.quote(pr_base)}")
print(f"pr_state={shlex.quote(pr_state)}")
print(f"pr_mergeable={shlex.quote(pr_mergeable)}")
print(f"pr_merge_state={shlex.quote(pr_merge_state)}")
print(f"pr_rollup_count={pr_rollup_count}")
print(f"pr_rollup_pass={pr_rollup_pass}")
print(f"pr_title={shlex.quote(pr_title)}")
print(f"pr_labels_csv={shlex.quote(pr_labels_csv)}")
print(f"pr_commits_count={pr_commits_count}")
print(f"pr_additions={pr_additions}")
')"

    if [ -z "${pr_number:-}" ]; then
        log "issue #${number}: could not parse PR record for ${branch} — skip"
        errored=$((errored+1)); continue
    fi

    # MERGED (Q22 done manually by user): post-merge reconciliation per
    # ADR-0014 (docs/adr/0014-agent-flow-issue-closure.md).
    #
    # Invariant: issue may close <=> PR MERGED into develop AND issue has
    # e2e-done produced by e2e-process (not by merge-gate itself). Race:
    # e2e-process may set e2e-done after we observed initial labels — so
    # we re-read labels RIGHT BEFORE close. Order: (1) re-read labels +
    # state, (2) close issue if e2e-done, (3) only on success run
    # destructive cleanup (delete branch, free worktrees, archive card,
    # cleanup comment, drop stale labels). Close failure → warning,
    # destructive cleanup is deferred, retry next 5m tick.
    if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]; then
        log "issue #${number}: PR #${pr_number} MERGED into ${pr_base} — post-merge reconcile (ADR-0014)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would reconcile issue #${number} (re-read labels, maybe close, then cleanup ${branch})"
            continue
        fi
        # 0.1) Re-read current labels & state — race with e2e-process
        # (e2e-process may have set e2e-done between our initial issue-list
        # pull and now; also the issue may already be CLOSED from a previous
        # tick in this same merge cycle, making close a no-op).
        _current_labels_csv="$(gh issue view "$number" --repo "$GH_REPO" --json labels \
            --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
        _current_labels_norm="$(printf '%s' "$_current_labels_csv" | tr '[:upper:]' '[:lower:]')"
        _has_e2e_done="0"
        if has_label "$_current_labels_norm" "$DONE_LABEL"; then
            _has_e2e_done="1"
        fi
        _issue_state="$(gh issue view "$number" --repo "$GH_REPO" --json state \
            --jq '.state' 2>/dev/null || echo '')"
        log "issue #${number}: pre-close state=${_issue_state} e2e-done=${_has_e2e_done}"

        # 0.2) Close only when invariant holds. Four branches:
        #   (a) already CLOSED → idempotent skip, proceed to cleanup
        #   (b) e2e-done present, OPEN → close with reason=completed
        #   (c) MERGED but no e2e-done → leave OPEN, defer destructive
        #       cleanup until next tick when e2e-process may set e2e-done
        #       (race merge → label, ADR §5).
        #   (d) state unreadable → defer cleanup: we cannot prove the
        #       issue is closed, so we must not delete the last mapping
        #       (ADR §4 req 4).
        _closed_this_tick=0
        case "$_issue_state" in
            CLOSED)
                # CLOSED already (e.g. closed manually or previous tick) — skip
                # close, go straight to cleanup.
                log "issue #${number}: already CLOSED — close skipped"
                ;;
            "")
                # gh issue view failed / state unreadable — conservative
                # deferral: destructive cleanup must not run on unverifiable
                # state, otherwise we lose the issue→PR mapping (ADR §4 req 4).
                log "issue #${number}: WARNING issue state unreadable — destructive cleanup deferred to next tick"
                labeled=$((labeled+1)); continue
                ;;
            OPEN)
                if [ "$_has_e2e_done" = "1" ]; then
                    if gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
                        _closed_this_tick=1
                        log "issue #${number}: CLOSED (reason=completed, PASS-proven via ${DONE_LABEL})"
                    else
                        # Close API failure — destructive cleanup MUST be
                        # deferred, otherwise we lose the mapping. Warning
                        # only, no `|| true` masking (ADR §4 req 4).
                        log "issue #${number}: WARNING gh issue close failed — destructive cleanup deferred to next tick"
                        labeled=$((labeled+1)); continue
                    fi
                else
                    # MERGED without e2e-done — wait for e2e-process PASS.
                    # Do NOT touch remote branch / archive card yet: a later
                    # tick may close this issue, and we don't want to
                    # archive the card while a worker is still holding it.
                    log "issue #${number}: MERGED but awaiting ${DONE_LABEL} — destructive cleanup deferred"
                    labeled=$((labeled+1)); continue
                fi
                ;;
            *)
                log "issue #${number}: unexpected issue state=${_issue_state} — skip cleanup"
                skipped=$((skipped+1)); continue
                ;;
        esac

        # 1) Find the issue's kanban card (may be done already).
        card_id=""
        if [ -z "${task_id:-}" ]; then
            card_id="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
                | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
    for t in tasks:
        if str(t.get("issue","")) == sys.argv[1]:
            print(t.get("id","")); break
except Exception:
    pass
' "$number" 2>/dev/null || true)"
        else
            card_id="$task_id"
        fi
        # 2) Free stale worktrees on this branch (kanban workspace_path aware).
        if [ -n "$card_id" ]; then
            free_stale_worktrees_for "$card_id" || true
        fi
        # 3) Delete the remote branch (merged — safe). We have already
        # closed the issue above, so the issue→PR mapping is preserved
        # in GitHub issue history regardless of whether the branch ref
        # still exists (ADR §4 req 4).
        if git ls-remote --heads "https://github.com/$GH_REPO.git" "$branch" 2>/dev/null | grep -q "$branch"; then
            gh api -X DELETE "repos/$GH_REPO/git/refs/heads/$branch" >/dev/null 2>&1 \
                && log "issue #${number}: remote branch ${branch} deleted" \
                || log "issue #${number}: WARNING failed to delete branch ${branch}"
        else
            log "issue #${number}: branch ${branch} already gone"
        fi
        # 4) Archive the card (done → archived) so the board stays clean.
        if [ -n "$card_id" ]; then
            card_state="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$card_id" --json 2>/dev/null \
                | python3 -c 'import sys,json
try: print(json.load(sys.stdin).get("task",{}).get("status",""))
except Exception: print("")' 2>/dev/null || true)"
            if [ "$card_state" = "done" ]; then
                "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$card_id" >/dev/null 2>&1 \
                    && log "issue #${number}: card ${card_id} archived (merged)" || true
            fi
        fi
        # 5) Dedup cleanup-коммента (ретро 10.08 t_9caf5d52): раньше коммент
        #    «✅ PR #N смержен» постился КАЖДЫЙ тик (5 мин) → 6 одинаковых на
        #    #1089 (08:35–08:49). Постим только если за последние часы такого
        #    коммента ещё нет. ADR-0014: текст говорит правду — упомянуть
        #    закрытие issue явно.
        _dedup_since="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _dup_count="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_dedup_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("✅ PR #'"${pr_number}"' смержен"))] | length' 2>/dev/null || echo 0)"
        if [ "${_dup_count:-0}" -eq 0 ]; then
            _close_note=""
            if [ "$_closed_this_tick" = "1" ]; then
                _close_note="Issue закрыта (reason=completed, PASS-proven). "
            fi
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "✅ PR #${pr_number} смержен в ${pr_base}. ${_close_note}Cleanup: ветка удалена, worktree освобождены, карточка заархивирована." >/dev/null 2>&1 || true
        else
            log "issue #${number}: merged-cleanup comment already exists (×${_dup_count}) — dedup skip"
        fi
        # 6) Снять stale-метки со смерженного фикса (ретро 10.08 t_9caf5d52):
        #    e2e:rejected/needs-e2e на merged-PR не актуальны. e2e-done НЕ
        #    добавляем сами: PASS-label принадлежит только e2e-process
        #    (ADR-0014 §4 req 2). Если e2e-done уже стоит — он остаётся;
        #    если не стоит — мы НЕ должны его создавать.
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        labeled=$((labeled+1)); continue
    fi

    # --- return path from e2e:rejected (ретро 10.08 t_9caf5d52) ------------
    # Раньше e2e:rejected был ТУПИКОМ: e2e-process скипает такие issue (line 15),
    # и никто не возвращал их в ротацию (#1089/#1077 — stale-метка навсегда).
    # Теперь: merge-gate по push в PR (новые коммиты после момента навешивания
    # e2e:rejected) снимает e2e:rejected → ставит needs-e2e → e2e-process снова
    # возьмёт issue в ротацию. Момент метки берём из timeline issue (последний
    # labeled-евент для REJECTED_LABEL).
    if has_label "$labels_norm" "$REJECTED_LABEL" && [ "$pr_state" = "OPEN" ]; then
        _rejected_at="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
            --jq '[.[] | select(.event=="labeled" and .label.name=="'"$REJECTED_LABEL"'")][-1].created_at' 2>/dev/null || echo '')"
        _return_reason=""
        if [ -n "$_rejected_at" ]; then
            # Сигнал 1: push в PR — новые коммиты после момента метки.
            _new_commits="$(gh api "repos/${GH_REPO}/pulls/${pr_number}/commits?per_page=100" \
                --jq '[.[] | select(.commit.committer.date >= "'"$_rejected_at"'")] | length' 2>/dev/null || echo 0)"
            if [ "${_new_commits:-0}" -gt 0 ] 2>/dev/null; then
                _return_reason="в PR #${pr_number} появились новые коммиты после ${_rejected_at}"
            fi
        fi
        # Сигнал 2 (ретро 10.08 t_9caf5d52): коммент воркера ПОСЛЕ метки.
        # Отличаем от комментов самого процесса (начинаются с agent-flow: или
        # ## 📊 e2e-доклад) — воркер пишет worker-evidence:/свободным текстом.
        if [ -z "$_return_reason" ]; then
            _worker_cmt="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_rejected_at}&per_page=100" \
                --jq '[.[] | select((.body | startswith("agent-flow:") | not) and (.body | startswith("## 📊 e2e-доклад") | not) and (.body | startswith("⛔ CI красный") | not) and (.body | startswith("✅ PR #") | not) and (.body | startswith("🔀 merge conflict") | not) and (.body | startswith("🔄") | not))] | length' 2>/dev/null || echo 0)"
            if [ "${_worker_cmt:-0}" -gt 0 ] 2>/dev/null; then
                _return_reason="воркер прокомментировал issue #${number} после ${_rejected_at}"
            fi
        fi
        if [ -n "$_return_reason" ]; then
            log "issue #${number}: ${REJECTED_LABEL} → ${_return_reason} — returning to rotation"
            if [ "$DRY_RUN" != "true" ]; then
                gh issue edit "$number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
                gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "agent-flow: 🔄 ${REJECTED_LABEL} снят — ${_return_reason}. Поставлен ${NEEDS_E2E_LABEL}; e2e-process снова возьмёт issue в ротацию." >/dev/null 2>&1 || true
            fi
            labeled=$((labeled+1)); continue
        fi
        log "issue #${number} already has ${REJECTED_LABEL} and no new commits/worker comment — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency: skip if already in the post-merge-gate state.
    # (MERGED-cleanup above runs FIRST so merged PRs get cleaned even
    # when the issue carries needs-e2e/e2e-done/e2e:rejected.)
    if has_label "$labels_norm" "$NEEDS_E2E_LABEL" \
        || has_label "$labels_norm" "$DONE_LABEL"; then
        log "issue #${number} already past merge-gate (${labels_norm}) — skip"
        skipped=$((skipped+1)); continue
    fi

    # Guards: PR must be OPEN, targeting `develop`, mergeable, merge_state clean,
    # all checks SUCCESS.
    if [ "$pr_state" != "OPEN" ]; then
        log "issue #${number}: PR #${pr_number} state=${pr_state} — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_base" != "$DEVELOP_BRANCH" ]; then
        log "issue #${number}: PR #${pr_number} base=${pr_base} (want ${DEVELOP_BRANCH}) — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_rollup_count" -eq 0 ]; then
        log "issue #${number}: PR #${pr_number} has no check rollup yet — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_rollup_pass" -ne 1 ]; then
        # Red CI (Q21) — НЕ создаём CI-fix карточку. Комментим причину в issue
        # и разблокируем карточку воркера — он чинит в ТОЙ ЖЕ ветке.
        log "issue #${number}: PR #${pr_number} checks not all SUCCESS — unblocking card ${task_id} (Q21)"
        failed_checks="$(printf '%s' "$pr_json" | python3 -c '
import json, sys
try:
    pr = json.load(sys.stdin)
    names = []
    for c in (pr.get("statusCheckRollup") or []):
        ctx = c.get("context") or c.get("name") or "?"
        st = c.get("conclusion") or c.get("state") or "?"
        if st not in ("SUCCESS", "NEUTRAL", "SKIPPED"):
            names.append(f"{ctx}: {st}")
    print("; ".join(names[:6]) or "unknown checks")
except Exception:
    print("unknown checks")
' 2>/dev/null || echo "unknown checks")"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would comment red-CI on issue #${number} and unblock card ${task_id}"
            skipped=$((skipped+1)); continue
        fi
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "⛔ CI красный (PR #${pr_number}): ${failed_checks}
Карточка разблокирована — исправь в ветке ${branch} и запушь. Merge-gate снова поставит needs-e2e когда всё позеленеет." >/dev/null 2>&1 || true
        # Процесс-фикс (09.08, ретро): освободить ветку от worktree старых
        # done/archived карточек ПЕРЕД unblock — иначе респавн падает
        # «git worktree add failed» и карточка навсегда виснет в blocked.
        free_stale_worktrees_for "$task_id" || true
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock "$task_id" >/dev/null 2>&1; then
            log "issue #${number}: card ${task_id} unblocked (CI red, worktrees freed)"
        else
            log "issue #${number}: WARNING unblock failed for ${task_id} — card may not be blocked yet"
        fi
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_mergeable" != "MERGEABLE" ]; then
        # Процесс-фикс (10.08, ретро t_75e787fd — Шифу прямо: «нахуя отдельную
        # карточку, та же карточка должна знать»). Если PR CONFLICTING →
        # **дописываем reminder в существующую рабочую карточку воркера**
        # (по issue_number находим его task_id, comment с rebase-инструкцией).
        # НЕ создаём новую карточку — это лишняя сущность.
        if [ "$pr_mergeable" = "CONFLICTING" ] && [ "$pr_state" = "OPEN" ] && [ -n "${task_id:-}" ]; then
            log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — appending rebase reminder to existing card ${task_id}"
            _rebase_reminder="## 🔀 merge conflict detected (merge-gate tick, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) → develop = **CONFLICTING**. Develop убежал вперёд, твоя ветка не мерджится напрямую.

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** \`${pr_head_ref}\` — НЕ создавай новую ветку и НЕ новый PR (Шифу прямо).
2. **rebase** на origin/develop: \`git fetch origin develop && git rebase origin/develop\`.
3. Разреши конфликты → \`git add -A && git rebase --continue\`.
4. \`git push --force-with-lease origin ${pr_head_ref}\` (force-with-lease безопасен).
5. Метки снимать НЕ надо (PR уже needs-e2e). Следующий тик merge-gate снова проверит — если MERGEABLE → поставит needs-review / needs-e2e по процессу.

**Когда карточка закрывается:** когда увидишь что PR стал MERGEABLE (rebase прошёл, push дошёл).

**Команды шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
# ... resolve conflicts ...
git add -A
git rebase --continue
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

(Этот reminder автоматически дописан merge-gate. Никакой новой карточки не создано — Шифу прямо: «та же карточка должна знать».)"
            hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_rebase_reminder" >/dev/null 2>&1 \
                || log "issue #${number}: WARNING appending rebase reminder to ${task_id} failed"
        else
            log "issue #${number}: PR #${pr_number} mergeable=${pr_mergeable} — skip"
        fi
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_merge_state" != "CLEAN" ]; then
        # Процесс-фикс (10.08, ретро t_51b5ad24 — Шифу прямо: «оно должно
        # взять себе девелоп сейчас и позеленеть»). Если UNSTABLE (CI fail,
        # но PR mergeable=YES) → НЕ просто skip, а **дописываем reminder в
        # существующую карточку воркера**: «CI красный по известной причине,
        # rebase на origin/develop, в develop уже есть фикс». Тот же PR, та
        # же ветка, никаких новых. Если карточки нет → создаём с assignee
        # по метке issue.
        if [ "$pr_merge_state" = "UNSTABLE" ] && [ "$pr_state" = "OPEN" ]; then
            log "issue #${number}: PR #${pr_number} mergeStateStatus=UNSTABLE — appending rebase reminder"
            # Подтягиваем headRefName — UNSTABLE-блок не имеет его из основного цикла (регрессия t_1146)
            pr_head_ref="$(gh pr view "$pr_number" --repo "$GH_REPO" --json headRefName --jq '.headRefName' 2>/dev/null || echo "")"
            [ -z "${pr_head_ref:-}" ] && log "issue #${number}: WARNING cannot fetch headRefName for PR #${pr_number}" && continue
            _assignee="default"
            for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
                case "$lbl" in
                    agent:backend)    _assignee="backend"; break ;;
                    agent:developer)  _assignee="developer"; break ;;
                    agent:tester)     _assignee="tester"; break ;;
                    agent:devops)     _assignee="devops"; break ;;
                    agent:architect)  _assignee="architect"; break ;;
                esac
            done
            _reminder="## ⚠️ CI UNSTABLE detected (merge-gate tick, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) = **mergeable=MERGEABLE + mergeStateStatus=UNSTABLE** (CI fail, но конфликтов с develop нет).

**Что делать** (по процессу Шифу 10.08 — «оно должно взять себе девелоп сейчас и позеленеть»):
1. **В той же ветке** \`${pr_head_ref}\` (тот же PR, никаких новых).
2. **rebase на origin/develop**: \`git fetch origin develop && git rebase origin/develop\`.
3. Push --force-with-lease: \`git push --force-with-lease origin ${pr_head_ref}\`.
4. Следующий CI-прогон автоматом подхватит develop-фиксы → UNSTABLE → CLEAN → merge-gate поставит needs-e2e → e2e-прогон.
5. Карточка закрывается когда PR станет MERGEABLE+CLEAN (CI зелёный).

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

(Этот reminder автоматически дописан merge-gate — Шифу прямо: «оно должно взять себе девелоп сейчас и позеленеть», не ждать ручного триггера.)"
            if [ -n "${task_id:-}" ]; then
                hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_reminder" >/dev/null 2>&1 \
                    || log "issue #${number}: WARNING appending UNSTABLE reminder to ${task_id} failed"
                log "issue #${number}: UNSTABLE reminder appended to existing card ${task_id}"
            else
                # Нет существующей карточки — создаём (skill из профиля assignee).
                _skill="architecture-doc-review"  # default архитекторский, переопределим ниже
                case "$_assignee" in
                    backend)   _skill="test-driven-development" ;;
                    developer) _skill="test-driven-development" ;;
                    tester)    _skill="test-driven-development" ;;
                    devops)    _skill="hermes-agent-flow" ;;
                esac
                hermes kanban --board "$KANBAN_BOARD" create \
                    --assignee "$_assignee" --skill "$_skill" --priority 80 --max-runtime 1800 \
                    --body "$_reminder" \
                    "⚠️ CI UNSTABLE: rebase \`${pr_head_ref}\` на develop (issue #${number}, PR #${pr_number})" \
                    >/dev/null 2>&1 || log "issue #${number}: WARNING UNSTABLE card create failed"
                log "issue #${number}: UNSTABLE card created for ${_assignee}"
            fi
        else
            log "issue #${number}: PR #${pr_number} mergeStateStatus=${pr_merge_state} — skip"
        fi
        skipped=$((skipped+1)); continue
    fi

    # All green — classify PR by kind (lint vs functional, ретро 10.08 #2).
    # lint: lint/refactor PR (только стиль/докстринги/формат) → e2e НЕ обязателен,
    #       воркер комментирует только по желанию; ставим `needs-review` напрямую.
    # functional: всё остальное → e2e обязателен, ставим `needs-e2e`.
    pr_kind="$(detect_pr_kind "$pr_labels_csv" "$pr_title")"
    log "issue #${number} PR #${pr_number} kind=${pr_kind} (CI green & clean)"

    # --- big-bang-override gate (ADR-0013, ретро t_9726053d) ---------------
    # PR > ${BIG_BANG_MAX_COMMITS} коммитов ИЛИ > ${BIG_BANG_MAX_LINES} строк
    # ЗАПРЕЩЁН без явной метки ${BIG_BANG_OVERRIDE_LABEL} на issue. Метку
    # ставит ТОЛЬКО товарищ Шифу (см. CONTRIBUTING.md §69-71). Поведение:
    #   - НЕ ставить needs-e2e / needs-review (иначе e2e-process поглотит PR)
    #   - оставить PR-комментарий (с дедупликацией как в post-merge close):
    #     "PR #N: size превышает ADR-0013, требуется split ИЛИ @Шифу ставит override"
    #   - comment постится ровно один раз за 24h (как cleanup-коммент в §5
    #     post-merge), иначе 6 одинаковых сообщений за час (ретро 10.08 t_9caf5d52).
    # Override → стандартный путь (needs-e2e / needs-review).
    _bb_reasons=""
    if [ "${pr_commits_count:-0}" -gt "${BIG_BANG_MAX_COMMITS}" ] 2>/dev/null; then
        _bb_reasons="${_bb_reasons}${pr_commits_count} коммитов > ${BIG_BANG_MAX_COMMITS}; "
    fi
    if [ "${pr_additions:-0}" -gt "${BIG_BANG_MAX_LINES}" ] 2>/dev/null; then
        _bb_reasons="${_bb_reasons}${pr_additions} строк > ${BIG_BANG_MAX_LINES}; "
    fi
    if [ -n "$_bb_reasons" ]; then
        if has_label "$labels_norm" "$BIG_BANG_OVERRIDE_LABEL"; then
            log "issue #${number}: big-bang (${_bb_reasons% ;}) но override ${BIG_BANG_OVERRIDE_LABEL} есть — пропускаем gate"
        else
            log "issue #${number}: PR #${pr_number} BIG-BANG (${_bb_reasons% ;}) — ${BIG_BANG_OVERRIDE_LABEL} отсутствует, block needs-e2e"
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would: skip needs-e2e + post big-bang comment to PR #${pr_number} (dedup 24h)"
                labeled=$((labeled+1)); continue
            fi
            # Дедупликация как в post-merge cleanup: за последние 24h
            # постим только один раз. Сейчас PR огромный (4850/100),
            # round-49..54 → 6 одинаковых комментов = спам. Шифу прямо:
            # «один раз label-коммент, round больше не запускается».
            _bb_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _bb_dup_count="$(gh api "repos/${GH_REPO}/issues/${pr_number}/comments?since=${_bb_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🚨 **PR #'"${pr_number}"' BIG-BANG"))] | length' 2>/dev/null || echo 0)"
            if [ "${_bb_dup_count:-0}" -gt 0 ] 2>/dev/null; then
                log "issue #${number}: big-bang comment на PR #${pr_number} уже проставлен (×${_bb_dup_count} за 24h) — dedup skip"
                skipped=$((skipped+1)); continue
            fi
            # Коммент И на issue (чтобы воркер увидел в task), И на PR
            # (чтобы ревьюер/Шифу увидел). Один раз каждый.
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "🚨 **PR #${pr_number} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Требуется:
1. **split** на инкрементальные PR (по 1 эпику на issue), каждый проходит e2e отдельно, ИЛИ
2. **товарищ Шифу** ставит метку \`${BIG_BANG_OVERRIDE_LABEL}\` на этот issue (явный override).

Merge-gate **НЕ поставит ${NEEDS_E2E_LABEL}** без override. Round-процесс не будет автоматически гонять e2e на этом PR.

Ссылки: ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md), CONTRIBUTING.md §69-71." >/dev/null 2>&1 || true
            gh pr comment "$pr_number" --repo "$GH_REPO" --body \
                "🚨 **PR #${pr_number} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Merge-gate блокирует e2e-ротацию: ${NEEDS_E2E_LABEL} не будет поставлен.

Что делать:
- **split** на инкрементальные PR (по 1 эпику), ИЛИ
- **товарищ Шифу** ставит \`${BIG_BANG_OVERRIDE_LABEL}\` на issue #${number}." >/dev/null 2>&1 || true
            # Помечаем issue меткой agent-flow:big-bang-blocked (best-effort) — чтобы
            # triage/воркер/дашборд видели, что issue ждёт решения. Не критично
            # если метка уже есть или label API упадёт.
            gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow:big-bang-blocked" >/dev/null 2>&1 || true
            labeled=$((labeled+1)); continue
        fi
    fi

    if [ "$pr_kind" = "lint" ]; then
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh pr edit ${pr_number} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
            labeled=$((labeled+1)); continue
        fi
        if ! gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
            log "WARNING: failed to add ${NEEDS_REVIEW_LABEL} to PR #${pr_number} — will retry next tick"
            errored=$((errored+1)); continue
        fi
        log "issue #${number}: lint PR #${pr_number} → ${NEEDS_REVIEW_LABEL} (skip e2e)"
        labeled=$((labeled+1)); continue
    fi

    log "issue #${number} PR #${pr_number} is green & clean — labeling ${NEEDS_E2E_LABEL}"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: gh issue edit ${number} --repo ${GH_REPO} --add-label ${NEEDS_E2E_LABEL}"
        labeled=$((labeled+1)); continue
    fi

    if ! gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1; then
        log "WARNING: failed to add label to issue #${number} — will retry next tick"
        errored=$((errored+1)); continue
    fi

    # Best-effort: also propagate the label to the PR so downstream PR-side
    # gates (e.g. GitHub Actions that watch PR labels) stay consistent.
    gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true

    labeled=$((labeled+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for issue in data:
    n = issue["number"]
    t = issue["title"]
    l = ",".join(sorted({lab["name"] for lab in issue.get("labels", [])}))
    b = issue.get("body") or ""
    def esc(s):
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\n", "\\n")
    sys.stdout.write(f"{n}\t{esc(t)}\t{esc(l)}\t{esc(b)}\n")
')

# --- Дополнительный цикл (10.08, ретро t_51b5ad24 — Шифу прямо: «оно должно
# взять себе девелоп сейчас и позеленеть»): сканируем ВСЕ open PR на
# UNSTABLE/CONFLICTING, а не только те что привязаны к issues с меткой
# needs-e2e. Иначе PR без меток (как #1096 downmix тесты) висят красными
# без внимания процесса. Находим связанный task_id по head-branch (wt/<id>-*
# → t_<id>) или по issue_number в body → дописываем reminder в существующую
# карточку (та же карточка должна знать, Шифу прямо).
log "scan-all-prs: scanning ALL open PRs for UNSTABLE/CONFLICTING (not just needs-e2e)"

# Маппинг head-branch → task_id через wt/... ветки (t_51b5ad24-respeaker-downmix-tests → t_51b5ad24)
_prs_json="$(gh pr list --repo "$GH_REPO" --state open \
    --json number,title,headRefName,mergeable,mergeStateStatus,labels 2>/dev/null || echo '[]')"

printf '%s' "$_prs_json" | python3 -c '
import json, sys, re, subprocess
data = json.load(sys.stdin)
for pr in data:
    pr_num = pr["number"]
    head = pr["headRefName"]
    mergeable = pr.get("mergeable", "")
    merge_state = pr.get("mergeStateStatus", "")
    if mergeable not in ("CONFLICTING",) and merge_state not in ("UNSTABLE",):
        continue
    # Определяем issue_number: из PR title (#NNNN) или из branch (z-{agent}/NNNN-*).
    # Сначала пробуем title (#NNNN), иначе ищем NNNN- в branch, но НЕ t_xxxx (это task_id).
    m = re.search(r"#(\d+)", pr.get("title",""))
    issue_num = m.group(1) if m else ""
    if not issue_num:
        m2 = re.search(r"z-\{agent\}/(\d+)-", head)
        issue_num = m2.group(1) if m2 else ""
    # Чистим если это всё ещё task_id (на случай кривой регулярки)
    if issue_num.startswith("t_") or len(issue_num) > 7:
        issue_num = ""
    # Определяем task_id: t_<id> из branch (z-{agent}/t_<id>-...) или wt/t_<id>
    m2 = re.search(r"t_([a-f0-9]+)", head)
    task_id = ("t_" + m2.group(1)) if m2 else ""
    # Fallback: если task_id не найден в branch, но знаем issue_num — ищем
    # карточку по комментам issue (паттерн "kanban: t_xxx", как основной цикл).
    # ВНИМАНИЕ: внутри python3 -c '...' (bash single-quote) нельзя писать
    # одинарную кавычку буквально — используем chr(39) для генерации в bash-строку.
    if not task_id and issue_num:
        q = chr(39)
        # Ищем карточку по комментам issue (паттерн "kanban: t_xxx") и
        # проверяем что она НЕ archived: перебираем кандидатов С КОНЦА
        # (последняя = актуальная), берём первую не-archived.
        r = subprocess.run(
            ["bash", "-c",
             f"gh issue view {issue_num} --repo krikz/rob_box_project --comments --json comments 2>/dev/null "
             f"| grep -Eo {q}kanban: t_[a-f0-9]+{q} | sort -u | tail -n10"],
            capture_output=True, text=True)
        cands = [c.strip().replace("kanban: ", "") for c in r.stdout.splitlines() if c.strip()]
        for cand in reversed(cands):
            st = subprocess.run(
                ["bash", "-c", f"hermes kanban --board robbox show {cand} 2>/dev/null | grep -m1 'status:'"],
                capture_output=True, text=True).stdout.strip()
            if "archived" not in st:
                task_id = cand
                break
    # Выводим с placeholder для пустых полей (иначе read с IFS=\t сливает
    # последовательные табы и поля сдвигаются: issue_num="" теряется)
    issue_num_out = issue_num if issue_num else "-"
    task_id_out = task_id if task_id else "-"
    print(f"{pr_num}\t{head}\t{mergeable}\t{merge_state}\t{issue_num_out}\t{task_id_out}")
' 2>/dev/null | while IFS=$'\t' read -r pr_num head mergeable merge_state issue_num task_id; do
    [ -z "$pr_num" ] && continue
    # Placeholder "-" → пустая строка
    [ "$issue_num" = "-" ] && issue_num=""
    [ "$task_id" = "-" ] && task_id=""
    log "scan-all-prs: PR #${pr_num} (${head}) mergeable=${mergeable} state=${merge_state}"

    # Определяем assignee по меткам issue (если знаем issue_num)
    _assignee="default"
    if [ -n "$issue_num" ]; then
        for lbl in $(gh issue view "$issue_num" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
            case "$lbl" in
                agent:backend)    _assignee="backend"; break ;;
                agent:developer)  _assignee="developer"; break ;;
                agent:tester)     _assignee="tester"; break ;;
                agent:devops)     _assignee="devops"; break ;;
                agent:architect)  _assignee="architect"; break ;;
            esac
        done
    fi

    # Формируем reminder (тот же текст, что в основном цикле)
    if [ "$mergeable" = "CONFLICTING" ]; then
        _reminder="## 🔀 merge conflict detected (merge-gate scan-all-prs, $(date -u +%H:%M:%SZ))

PR #${pr_num} (\`${head}\`) → develop = **CONFLICTING**.

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** \`${head}\` — НЕ создавай новую ветку и НЕ новый PR.
2. **rebase** на origin/develop.
3. Разреши конфликты → \`git add -A && git rebase --continue\`.
4. \`git push --force-with-lease origin ${head}\`.
5. Карточка закрывается когда PR станет MERGEABLE.

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${head}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${head}
\`\`\`"
        _title_prefix="🔀 merge conflict"
    else
        _reminder="## ⚠️ CI UNSTABLE detected (merge-gate scan-all-prs, $(date -u +%H:%M:%SZ))

PR #${pr_num} (\`${head}\`) = **MERGEABLE + UNSTABLE** (CI fail, конфликтов нет).

**Что делать** (по процессу Шифу 10.08 — «взять себе девелоп сейчас и позеленеть»):
1. **В той же ветке** \`${head}\`.
2. \`git fetch origin develop && git rebase origin/develop\`.
3. \`git push --force-with-lease origin ${head}\`.
4. Следующий CI-прогон подхватит develop-фиксы → CLEAN → merge-gate поставит needs-e2e.

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${head}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${head}
\`\`\`"
        _title_prefix="⚠️ CI UNSTABLE: rebase"
    fi

    if [ -n "$task_id" ]; then
        # Дописываем reminder в существующую карточку (та же карточка, Шифу прямо)
        hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_reminder" >/dev/null 2>&1 \
            || log "scan-all-prs: WARNING appending reminder to ${task_id} failed"
        log "scan-all-prs: reminder appended to existing card ${task_id} for PR #${pr_num}"
        # Процесс-фикс (10.08): если карточка done/archived (воркер закрыл, но
        # PR снова CONFLICTING/UNSTABLE из-за новых merge в develop) — requeue
        # её в ready, чтобы воркер снова взял и сделал rebase.
        _card_status="$(hermes kanban --board "$KANBAN_BOARD" show "$task_id" 2>/dev/null | grep -m1 'status:' | sed 's/.*status:[[:space:]]*//' || true)"
        case "$_card_status" in
            done|archived|blocked)
                hermes kanban --board "$KANBAN_BOARD" requeue "$task_id" --reason "scan-all-prs: PR #${pr_num} ${mergeable}/${merge_state} — rebase needed, card reopened" >/dev/null 2>&1 \
                    && log "scan-all-prs: card ${task_id} requeued (was ${_card_status}) for PR #${pr_num}" \
                    || log "scan-all-prs: WARNING requeue ${task_id} failed (status=${_card_status})"
                ;;
        esac
        # Если карточка blocked (воркер упал) — разблокируем
        if [ "$_card_status" = "blocked" ]; then
            hermes kanban --board "$KANBAN_BOARD" unblock "$task_id" --reason "scan-all-prs: PR #${pr_num} ${mergeable}/${merge_state} — rebase reminder appended, retry" >/dev/null 2>&1 \
                && log "scan-all-prs: card ${task_id} unblocked for retry (PR #${pr_num})"
        fi
    else
        # Нет существующей карточки — НЕ создаём руками, только логируем
        # (процесс сам создаст её в основном цикле если issue имеет needs-e2e)
        log "scan-all-prs: no existing card for PR #${pr_num} (${head}); assignee=${_assignee}, issue=${issue_num:-?} — main cycle will pick up if needs-e2e"
    fi
done

# ============================================================================
# Ретро-путь (12.08 t_68607832 + t_061d466e): merged PR → issue без меток /
# с e2e:rejected
# ----------------------------------------------------------------------------
# ПРОБЛЕМА 1 (t_68607832): issues #1138/#1139 были починены ретро-карточками
# (вне label-цикла needs-e2e→e2e-done) и НИКОГДА не получали e2e-done →
# merge-gate молчал → issue висела OPEN при смерженном фиксе. Основной цикл
# выше обрабатывает только issues с меткой `hermes`; ретро-issues меток не
# имеют вовсе.
#
# ПРОБЛЕМА 2 (t_061d466e, 12.08): issue #1041 — ОТКРЫТА с меткой e2e:rejected,
# хотя фикс ВЛИТ (PR #1161 merged, CI зелёный). Петля:
#   - основной цикл находит PR по канонической ветке
#     z-{agent}/1041-fix-l-build-vision-pi-docker-tag-local-g → это CLOSED
#     PR #1155 (закрыт, НЕ смержен); реальный merged PR #1161 имеет ДРУГУЮ
#     ветку (z-{agent}/1041-fix-l-build-dockertag-clean) → основной цикл его
#     не видит;
#   - e2e-process не может прогнать e2e для #1041: ветка конфликтует
#     с develop (см. t_bff6eccf), фикс уже в develop;
#   - ретро-путь НАХОДИЛ merged PR #1161 (title содержит #1041), но скипал
#     issue из-за e2e:rejected → никто не закрывал.
#
# РЕШЕНИЕ: сканируем недавно смерженные PR (base=develop), извлекаем номера
# issues из title/body, и для OPEN issues применяем ретро-путь:
#   - PASS-доказательство есть (e2e run SUCCESS на ветке PR, ИЛИ CI-only PR
#     с зелёным CI) → close issue с комментарием-доказательством; если на
#     issue стоит e2e:rejected — снимаем его ПЕРЕД close (фикс влит, e2e
#     больше не нужен / не может пройти);
#   - иначе (и нет e2e:rejected) → ставим needs-e2e (e2e-process возьмёт
#     issue в ротацию);
#   - e2e:rejected + merged PR, но PASS-доказательства НЕТ → НЕ ставим
#     needs-e2e (иначе e2e-process зациклится: rejected → needs-e2e → снова
#     rejected). Оставляем rejected и логируем — нужен ручной разбор.
#
# Guard от пере-закрытия: issues с hermes (кроме e2e:rejected — см. #1041)/
# needs-e2e/e2e-done/no-e2e-required пропускаем — их обрабатывают основные
# циклы. Дубликаты комментариев дедуплицируются как в post-merge cleanup
# (6h окно).
# ============================================================================
RETRO_MERGED_DAYS="${RETRO_MERGED_DAYS:-14}"
retro_closed=0
retro_labeled=0

log "retro-path: scanning merged PRs referencing unlabeled issues"

# Недавно смерженные PR (base=develop). Ограничиваем окно, чтобы не сканировать
# всю историю репозитория каждый тик.
_retro_since="$(date -u -d "${RETRO_MERGED_DAYS} days ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
_retro_prs_json="$(gh pr list --repo "$GH_REPO" --state merged --base "$DEVELOP_BRANCH" \
    --limit 100 \
    --json number,title,body,headRefName,mergedAt 2>/dev/null || echo '[]')"
# Guard: пустой вывод gh (или сбой API) → валидный пустой список, иначе
# python3 json.load('') уронит pipeline под set -o pipefail.
if [ -z "$_retro_prs_json" ]; then
    _retro_prs_json='[]'
fi

# Извлекаем (issue, pr, head): номера issues, на которые ссылается PR в
# title/body (#N, closes #N, fixes #N). Скипаем PR, смерженные раньше окна,
# и self-reference (номер PR в своём же body, например "PR: #1142").
# ВАЖНО: process substitution (а не pipe), чтобы retro_closed/retro_labeled
# накапливались в текущем shell и попали в summary.
while IFS=$'\t' read -r r_issue r_pr r_head; do
    [ -z "$r_issue" ] && continue
    log "retro-path: issue #${r_issue} referenced by merged PR #${r_pr} (${r_head})"

    # Перечитываем labels/state — race с e2e-process (как в основном цикле).
    _r_labels_csv="$(gh issue view "$r_issue" --repo "$GH_REPO" --json labels \
        --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
    _r_labels_norm="$(printf '%s' "$_r_labels_csv" | tr '[:upper:]' '[:lower:]')"
    # Ретро 12.08 t_061d466e: e2e:rejected больше НЕ скипает ретро-путь —
    # это ровно петля #1041 (фикс влит, e2e не может пройти). Вместо скипа
    # ниже проверяем PASS-доказательство: есть → снять rejected + close,
    # нет → оставить rejected (не ставить needs-e2e, чтобы не зациклить
    # e2e-process).
    _r_was_rejected=0
    if has_label "$_r_labels_norm" "$REJECTED_LABEL"; then
        _r_was_rejected=1
        log "retro-path: issue #${r_issue} имеет ${REJECTED_LABEL} — ищем PASS-доказательство (ретро t_061d466e)"
    elif has_label "$_r_labels_norm" "$ISSUE_LABEL" \
        || has_label "$_r_labels_norm" "$NEEDS_E2E_LABEL" \
        || has_label "$_r_labels_norm" "$DONE_LABEL" \
        || has_label "$_r_labels_norm" "$NO_E2E_LABEL"; then
        log "retro-path: issue #${r_issue} уже в process-цикле (${_r_labels_norm}) — skip"
        continue
    fi
    _r_state="$(gh issue view "$r_issue" --repo "$GH_REPO" --json state \
        --jq '.state' 2>/dev/null || echo '')"
    if [ "$_r_state" != "OPEN" ]; then
        log "retro-path: issue #${r_issue} state=${_r_state} — skip"
        continue
    fi

    # --- PASS-доказательство ---
    _r_evidence=""
    # (a) e2e run SUCCESS на ветке PR (самое сильное доказательство)
    _r_e2e_ok="$(gh run list --repo "$GH_REPO" --branch "$r_head" \
        --workflow "L: E2E Voice Test" --limit 20 \
        --json conclusion --jq '[.[] | select(.conclusion == "success")] | length' 2>/dev/null || echo 0)"
    if [ "${_r_e2e_ok:-0}" -gt 0 ] 2>/dev/null; then
        _r_evidence="e2e run SUCCESS на ветке ${r_head}"
    else
        # (b) CI-only PR (все файлы в CI-контуре) + зелёный CI → e2e не нужен
        _r_files="$(gh pr view "$r_pr" --repo "$GH_REPO" --json files \
            --jq '[.files[].path]' 2>/dev/null || echo '[]')"
        _r_ci_only="$(printf '%s' "$_r_files" | python3 -c '
import json, sys
try:
    files = json.load(sys.stdin)
    ok = bool(files) and all(
        f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
        for f in files
    )
    print("1" if ok else "0")
except Exception:
    print("0")
' 2>/dev/null || echo 0)"
        if [ "$_r_ci_only" = "1" ]; then
            # ВНИМАНИЕ (ретро 12.08 t_061d466e): фильтр обязан разыменовывать
            # .statusCheckRollup[] — иначе jq применяется к объекту
            # {"statusCheckRollup":[...]} и падает «expected an object but got:
            # array» → rollup=1 → CI-only PASS не находится (петля #1041).
            _r_rollup="$(gh pr view "$r_pr" --repo "$GH_REPO" --json statusCheckRollup \
                --jq '[.statusCheckRollup[] | select(.conclusion == "FAILURE" or .conclusion == "CANCELLED" or .conclusion == "TIMED_OUT")] | length' 2>/dev/null || echo 1)"
            if [ "${_r_rollup:-1}" -eq 0 ] 2>/dev/null; then
                _r_evidence="CI-only PR #${r_pr} (только .github/scripts/docs), CI зелёный — e2e не требуется"
            fi
        fi
    fi

    if [ -n "$_r_evidence" ]; then
        log "retro-path: issue #${r_issue} PASS-доказательство: ${_r_evidence} — closing"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would close issue #${r_issue} (retro-path)"
            retro_closed=$((retro_closed+1)); continue
        fi
        # Ретро 12.08 t_061d466e: снять e2e:rejected ПЕРЕД close — фикс влит
        # (merged PR), e2e больше не нужен. Если снять не удалось — не
        # критично, close всё равно выполнится; следующий тик не найдёт
        # issue (она CLOSED).
        if [ "$_r_was_rejected" = "1" ]; then
            gh issue edit "$r_issue" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 \
                && log "retro-path: issue #${r_issue} ${REJECTED_LABEL} снят (PASS-доказательство)" \
                || log "retro-path: WARNING не удалось снять ${REJECTED_LABEL} с #${r_issue}"
        fi
        # Дедупликация комментария (6h) — не спамим каждый тик.
        _r_dedup_since="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _r_dup_count="$(gh api "repos/${GH_REPO}/issues/${r_issue}/comments?since=${_r_dedup_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("✅ ретро-путь"))] | length' 2>/dev/null || echo 0)"
        if [ "${_r_dup_count:-0}" -eq 0 ]; then
            _r_rejected_note=""
            if [ "$_r_was_rejected" = "1" ]; then
                _r_rejected_note=" Снят ${REJECTED_LABEL} (фикс влит, e2e не требуется)."
            fi
            gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                "✅ ретро-путь (ADR-0014 gap, t_68607832/t_061d466e): PR #${r_pr} смержен в ${DEVELOP_BRANCH}. PASS-доказательство: ${_r_evidence}.${_r_rejected_note} Issue закрыта." >/dev/null 2>&1 || true
        fi
        if gh issue close "$r_issue" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
            retro_closed=$((retro_closed+1))
            log "retro-path: issue #${r_issue} CLOSED (reason=completed, ретро-путь)"
        else
            log "retro-path: WARNING close failed for #${r_issue} — retry next tick"
        fi
    else
        if [ "$_r_was_rejected" = "1" ]; then
            # e2e:rejected + merged PR, но PASS-доказательства нет: НЕ ставим
            # needs-e2e — иначе e2e-process снова возьмёт issue и снова
            # поставит rejected (петля). Оставляем rejected — нужен ручной
            # разбор (ретро 12.08 t_061d466e).
            log "retro-path: issue #${r_issue} имеет ${REJECTED_LABEL}, PASS-доказательства нет — НЕ трогаем (нужен ручной разбор)"
            skipped=$((skipped+1))
            continue
        fi
        log "retro-path: issue #${r_issue} без PASS-доказательства — ставим ${NEEDS_E2E_LABEL}"
        if [ "$DRY_RUN" != "true" ]; then
            gh issue edit "$r_issue" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
            gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                "agent-flow: 🔄 ретро-путь: PR #${r_pr} смержен, но PASS-доказательства не найдено. Поставлен ${NEEDS_E2E_LABEL} — e2e-process возьмёт issue в ротацию." >/dev/null 2>&1 || true
        fi
        retro_labeled=$((retro_labeled+1))
    fi
done < <(printf '%s' "$_retro_prs_json" | python3 -c '
import json, sys, re
data = json.load(sys.stdin)
since = sys.argv[1]
seen = set()
for pr in data:
    if (pr.get("mergedAt") or "") < since:
        continue
    pr_num = str(pr.get("number", ""))
    text = (pr.get("title") or "") + "\n" + (pr.get("body") or "")
    head = pr.get("headRefName") or ""
    for m in re.finditer(r"#(\d+)", text):
        issue = m.group(1)
        if issue == pr_num:
            continue
        key = (issue, pr_num)
        if key in seen:
            continue
        seen.add(key)
        print(issue + "\t" + pr_num + "\t" + head)
' "$_retro_since" 2>/dev/null)

# --- summary -----------------------------------------------------------------
log "tick done: considered=${considered} labeled=${labeled} skipped=${skipped} errored=${errored} retro_closed=${retro_closed} retro_labeled=${retro_labeled}"

# Exit non-zero only on hard errors so cron can alert.
if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
