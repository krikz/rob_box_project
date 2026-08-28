#!/bin/bash
# ============================================================================
# agent-flow-blocked-watchdog.sh — auto-close orphan `needs-e2e` issues whose
# implementing PR is already MERGED.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-blocked-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Контекст / ретро t_1d0426e3:
#   Pattern "карточки-призраки": orphan blocked-карточки остаются на доске
#   после того, как задача решена через параллельную ветку (PR merged,
#   но issue НЕ закрыт). Причина — расхождение меток: issue в labels
#   имеет `needs-e2e`, хотя реализация уже в feature/avatar (или develop).
#   Manual cleanup уже проведён для t_547e17a7, t_3aa4c587, t_307bae4a,
#   но pattern системный — нужна автоматизация.
#
# Контракт (per tick):
#   1. List open issues with label `needs-e2e` in repo $GH_REPO
#   2. For each: search MERGED PR (gh pr list --state merged --search "#NNN")
#   3. If MERGED PR found:
#        - Verify mergeCommit exists in base branch (develop / feature/avatar)
#          via `git branch --contains <sha>` (локально) или `gh api`
#        - Comment issue with reason
#        - Close issue with state_reason=completed
#        - Find related kanban-card (t_<id> in issue body OR via label
#          `hermes` if any), comment the card with ready-to-close marker
#   4. Log stats: issues_checked, issues_closed, issues_skipped, errors.
#
# ENV:
#   GH_REPO                — owner/repo (default krikz/rob_box_project)
#   BLOCKED_WATCHDOG_DRY_RUN=true — only log, no side-effects
#   BASE_BRANCHES          — colon-separated, default develop:feature/avatar
#   LOCK_FILE              — flock guard against merge-gate (default
#                            /tmp/agent-flow-blocked-watchdog.lock)
#
# Выходы:
#   - Stderr: structured summary (for cron delivery).
#   - Exit 0 — всё ok (даже если ничего не закрыли).
#   - Exit 1 — критичный сбой (нет gh auth, локальный git недоступен).
#   - Exit 2 — найдены orphan-issue'ы (alert для cron, опционально).
#
# Pitfalls (gotchas):
#   - gh search возвращает MERGED PR'ы в общем списке; надо фильтровать
#     --state merged И проверять mergeCommit.oid (None для closed-not-merged).
#   - Не закрывать issue если базовая ветка — НЕ та, в которой работаем
#     (orphan может быть только для develop/feature/avatar).
#   - comment в issue должен быть idempotent — если за последние 24h уже
#     был написан marker "agent-flow-blocked-watchdog: closing", skip.
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${BLOCKED_WATCHDOG_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-blocked-watchdog.lock}"
BASE_BRANCHES="${BASE_BRANCHES:-develop:feature/avatar}"
WINDOW_HOURS=24  # comment idempotency window

# --- flock guard (avoid race with merge-gate) ------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] blocked-watchdog: another instance running — skip" >&2
    exit 0
fi

# --- gh auth probe ---------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] blocked-watchdog: gh auth failed — exit 1" >&2
    exit 1
fi

# --- helpers ---------------------------------------------------------------
_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

_marker_tag="🤖 [agent:devops] script=agent-flow-blocked-watchdog"

# issue_has_recent_marker <issue_num> — проверяет, был ли уже marker в
# последние WINDOW_HOURS часов в комментариях issue. Возвращает 0 если
# marker найден (→ idempotent skip).
issue_has_recent_marker() {
    local _num="$1"
    local _window_start_iso
    _window_start_iso="$(date -u -d "@$(( $(date -u +%s) - WINDOW_HOURS * 3600 ))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || true)"
    [ -z "$_window_start_iso" ] && return 1

    # gh api возвращает массив; ищем marker + не старше окна
    gh api "repos/${GH_REPO}/issues/${_num}/comments?per_page=30" \
        --jq "[.[] | select(.created_at >= \"${_window_start_iso}\") | .body] | .[]" 2>/dev/null \
        | grep -qF "$_marker_tag"
}

# find_merged_pr <issue_num> — ищет MERGED PR, ссылающийся на issue.
# Возвращает "PR#:sha" (через stdout) или пустую строку.
# ВАЖНО: gh search ловит ЛЮБОЙ issue-number в title (например, PR для #1595
# может содержать "AV-1 #1595" и попасть под поиск "#1605"). Чтобы избежать
# false-positive, фильтруем по exact `#NNNN` в title (regexp) — issue
# должен быть ПЕРВЫМ или ЕДИНСТВЕННЫМ issue-референсом.
find_merged_pr() {
    local _num="$1"
    gh pr list --repo "$GH_REPO" --state merged --search "#${_num}" \
        --json number,mergeCommit,baseRefName,headRefName,mergedAt,title 2>/dev/null \
    | python3 -c "
import json, re, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
issue = '${_num}'
for pr in data:
    mc = pr.get('mergeCommit') or {}
    oid = mc.get('oid') if isinstance(mc, dict) else None
    if not oid:
        continue
    title = pr.get('title') or ''
    # Match exactly: \"#NNNN\" где NNNN = issue. Issue должен быть
    # в title — иначе (только в body) gh search НЕ найдёт эту запись,
    # а мы тут только title проверяем для speed.
    if re.search(r'(^|[^0-9])#' + issue + r'([^0-9]|\$)', title):
        print(f'{pr[\"number\"]}:{oid}')
        sys.exit(0)
"
}

# is_in_base <sha> <base_branch> — проверяет, что mergeCommit sha присутствует
# в base branch (через `git branch --contains`). Требует локальный клон
# репо (REPO_DIR). Возвращает 0 если найден.
is_in_base() {
    local _sha="$1" _base="$2"
    local _repo_dir="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
    [ -d "$_repo_dir/.git" ] || return 1
    # `git branch --contains` выводит имена с префиксами (`* `, `  `, `+ ` для
    # разных типов веток), поэтому нормализуем через awk '{print $NF}'.
    # Используем branch -a для remote-веток + branch для local; объединяем.
    local _branches
    _branches="$(
        { git -C "$_repo_dir" branch -a --contains "$_sha" 2>/dev/null; \
          git -C "$_repo_dir" branch    --contains "$_sha" 2>/dev/null; } \
        | grep -vE '^\*|^\+' \
        | awk '{print $NF}' \
        | sort -u
    )"
    grep -qxF "$_base" <<< "$_branches"
}

# extract_card_id_from_body <body> — вытаскивает первый t_<id> из body issue.
extract_card_id_from_body() {
    local _body="$1"
    printf '%s' "$_body" | grep -oE '\bt_[a-f0-9]{8,}\b' | head -n1 || true
}

# --- main loop -------------------------------------------------------------
_now_s="$(date -u +%s)"
_window_start_iso="$(date -u -d "@$(( _now_s - WINDOW_HOURS * 3600 ))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || true)"

_issues_json="$(gh issue list --repo "$GH_REPO" --state open --label needs-e2e \
    --limit 50 --json number,title 2>/dev/null || echo '[]')"

_checked=0
_closed=0
_skipped=0
_errors=0
_records=()

while IFS=$'\t' read -r issue_num issue_title; do
    [ -n "$issue_num" ] || continue
    _checked=$(( _checked + 1 ))

    # idempotency: уже был наш комментарий за последние 24h
    if issue_has_recent_marker "$issue_num"; then
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] blocked-watchdog: SKIP #${issue_num} (recent marker found)" >&2
        continue
    fi

    # ищем merged PR
    _pr_info="$(find_merged_pr "$issue_num" || true)"
    if [ -z "$_pr_info" ]; then
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] blocked-watchdog: SKIP #${issue_num} (no merged PR found)" >&2
        continue
    fi
    _pr_num="${_pr_info%%:*}"
    _merge_sha="${_pr_info##*:}"

    # проверяем что merge sha присутствует хотя бы в одном из base branches
    _in_base=0
    _in_base_branch=""
    IFS=':' read -r -a _bases <<< "$BASE_BRANCHES"
    for b in "${_bases[@]}"; do
        if is_in_base "$_merge_sha" "$b"; then
            _in_base=1
            _in_base_branch="$b"
            break
        fi
    done

    if [ "$_in_base" -ne 1 ]; then
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] blocked-watchdog: SKIP #${issue_num} PR #${_pr_num} sha=${_merge_sha:0:10} (not in any base branch)" >&2
        continue
    fi

    # close + comment
    _merged_at="$(gh api "repos/${GH_REPO}/pulls/${_pr_num}" --jq '.merged_at' 2>/dev/null || echo unknown)"
    _body_text="${_marker_tag} action=closing reason=orphan-needs-e2e-with-merged-pr pr=#${_pr_num} sha=${_merge_sha:0:10} base=${_in_base_branch} mergedAt=${_merged_at:0:10}"

    # extract kanban card from issue body
    _issue_body="$(gh issue view "$issue_num" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || echo "")"
    _card_id="$(extract_card_id_from_body "$_issue_body" || true)"
    if [ -n "$_card_id" ]; then
        _body_text="${_body_text} card=${_card_id}"
    fi

    if [ "$DRY_RUN" = "true" ]; then
        echo "[$(_now_iso)] blocked-watchdog: [DRY-RUN] #${issue_num} would-close card=${_card_id:-NONE} pr=#${_pr_num}" >&2
        _closed=$(( _closed + 1 ))
        continue
    fi

    # post comment + close
    if gh issue comment "$issue_num" --repo "$GH_REPO" --body "$_body_text" >/dev/null 2>&1 \
        && gh issue close "$issue_num" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
        _closed=$(( _closed + 1 ))
        _records+=("$(printf '%s\tissue #%s\tPR #%s\tsha=%s\tbase=%s\tcard=%s' \
            "$(_now_iso)" "$issue_num" "$_pr_num" "${_merge_sha:0:10}" "$_in_base_branch" "${_card_id:-NONE}")")"
        echo "[$(_now_iso)] blocked-watchdog: CLOSED #${issue_num} PR #${_pr_num} sha=${_merge_sha:0:10} base=${_in_base_branch} card=${_card_id:-NONE}" >&2
    else
        _errors=$(( _errors + 1 ))
        echo "[$(_now_iso)] blocked-watchdog: ERROR closing #${issue_num} PR #${_pr_num}" >&2
    fi
done < <(printf '%s' "$_issues_json" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for it in data:
    print("\t".join([str(it.get("number","")), (it.get("title","") or "")[:80]]))
')

# --- structured summary -----------------------------------------------------
echo "[$(_now_iso)] blocked-watchdog: ✓ done checked=${_checked} closed=${_closed} skipped=${_skipped} errors=${_errors} repo=${GH_REPO}" >&2

# --- write stats log -------------------------------------------------------
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-blocked-watchdog.log}"
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# blocked-watchdog snapshot %s\n' "$(_now_iso)"
    printf 'timestamp\tissue\tpr\tsha\tbase\tcard\n'
    for r in "${_records[@]:-}"; do
        [ -n "$r" ] && printf '%s\n' "$r"
    done
    printf '# checked=%s closed=%s skipped=%s errors=%s repo=%s\n' \
        "$_checked" "$_closed" "$_skipped" "$_errors" "$GH_REPO"
} >> "$LOG_FILE" 2>/dev/null || true

# --- exit code -------------------------------------------------------------
# exit 2 если закрыли хоть один (alert для cron), exit 0 если ничего не
# закрыли (норма). exit 1 только если gh auth упала (выше).
if [ "$_closed" -gt 0 ] && [ "$DRY_RUN" != "true" ]; then
    exit 2
fi
exit 0