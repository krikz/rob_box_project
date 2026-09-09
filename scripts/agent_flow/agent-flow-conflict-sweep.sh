#!/bin/bash
# ============================================================================
# agent-flow-conflict-sweep.sh — reactive sweep для issue, у которых одновременно
# висят метки `needs-e2e` И `e2e-done` (data race: кто-то добавил needs-e2e
# обратно ПОСЛЕ e2e-process sweep, который снял его), И связанный PR
# уже MERGED в develop.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-conflict-sweep.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/  (legacy)
#
# Контекст / ретро t_8fba04b9 (issue #1977):
#   ADR-0014 инвариант выполнен для #1977 (PR #1979 MERGED + base=develop +
#   `e2e-done` стоит), но merge-gate 5-min loop НЕ закрывает issue. При этом
#   `needs-e2e` кто-то добавил обратно после e2e-sweep → label conflict
#   `needs-e2e + e2e-done` сидит на issue открытой после merge.
#
#   blocked-watchdog (every 240m) сканирует `needs-e2e` issues без
#   `e2e-done` — для #1977 другая семантика (там `e2e-done` ЕСТЬ), и он не
#   закрывает.
#
#   merge-gate 5-min loop ТОЖЕ должен был закрыть по ADR-0014 — но
#   "Status: silent (empty output)" 50+ тиков подряд, ничего не пишет.
#   Скорее всего race / pagination / rate-limit пропускает #1977 в его
#   основном цикле.
#
# Решение (этот скрипт): reactive sweep КАК FALLBACK на случай merge-gate silent.
# Расширяет blocked-watchdog контракт: вместо `needs-e2e && !e2e-done` —
#   `needs-e2e && e2e-done && PR.MERGED && PR.base==develop`.
#
# Контракт (per tick):
#   1. Default: scan open issues with BOTH labels `needs-e2e` AND `e2e-done`
#      via `gh issue list --repo X --label needs-e2e --label e2e-done --state open`
#      ИЛИ env override: `CONFLICT_SWEEP_ISSUE_NUM=NNN` — обрабатываем один
#      конкретный issue (one-shot cleanup, например для #1977 когда labels
#      уже сняты).
#   2. For each: find MERGED PR via gh pr list --search "#NNN" (exact-match regex)
#   3. Verify mergeCommit exists in develop (через `git branch --contains`)
#   4. If merged+base=develop:
#        - Comment issue with audit marker (idempotent — skip if marker
#          found within WINDOW_HOURS=24)
#        - `gh issue close N --reason completed`
#        - Remove labels `needs-e2e` И `e2e-done` (cleanup; если их нет — no-op)
#   5. Log stats: scanned, closed, skipped, errors.
#
# ENV:
#   GH_REPO                       — owner/repo (default krikz/rob_box_project)
#   CONFLICT_SWEEP_DRY_RUN=true   — only log, no side-effects
#   CONFLICT_SWEEP_ISSUE_NUM=NNN  — one-shot: scan только этот issue (любые
#                                    labels, для cleanup когда конфликт уже
#                                    разрешён руками). Можно несколько через
#                                    пробел.
#   BASE_BRANCHES                 — colon-separated, default develop:develop
#   LOCK_FILE                     — flock guard (default
#                                    /tmp/agent-flow-conflict-sweep.lock)
#   WINDOW_HOURS                  — marker idempotency window (default 24)
#
# Выходы:
#   - Stderr: structured summary (for cron delivery).
#   - Exit 0 — всё ok (даже если ничего не закрыли).
#   - Exit 1 — критичный сбой (нет gh auth, локальный git недоступен).
#   - Exit 2 — закрыли хоть один issue (alert для cron, опционально).
#
# Pitfalls (gotchas):
#   - gh search ловит ЛЮБОЙ issue-number в title (например, PR для #1595
#     может содержать "AV-1 #1595" и попасть под поиск "#1605"). exact-match
#     regex `(#NNN)([^0-9]|$)` отсекает — issue должен быть ПЕРВЫМ или
#     ЕДИНСТВЕННЫМ issue-референсом.
#   - squash-merge стирает original commit SHA → проверяем через
#     `git branch --contains <merge_commit_sha>`. Если PR только что
#     смержен (eventual consistency), git fetch может не видеть — fallback
#     через `gh api pulls/N` сразу возвращает merge_commit.oid.
#   - cron запускается с фоновым flock — НЕ ставить `set -u` слепо, если
#     массивы пустые (set -euo pipefail ОК, массивы проверяем явно).
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${CONFLICT_SWEEP_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-conflict-sweep.lock}"
BASE_BRANCHES="${BASE_BRANCHES:-develop:develop}"
WINDOW_HOURS="${WINDOW_HOURS:-24}"

# --- flock guard (avoid race with merge-gate) ------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] conflict-sweep: another instance running — skip" >&2
    exit 0
fi

# --- gh auth probe ---------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] conflict-sweep: gh auth failed — exit 1" >&2
    exit 1
fi

# --- helpers ---------------------------------------------------------------
_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

# marker tag для idempotency-comment — agent-flow-blocked-watchdog уже
# использует похожий формат, conflict-sweep использует свой чтобы не было
# ложных skip'ов между watchdog'ами.
_marker_tag="🤖 [agent:devops] script=agent-flow-conflict-sweep"

# issue_has_recent_marker <issue_num> — проверяет, был ли уже marker в
# последние WINDOW_HOURS часов. Возвращает 0 если найден (→ idempotent skip).
# На любой ошибке gh (rate-limit, auth) возвращает 1 (= "не найден"),
# чтобы скрипт не залип на неудачной проверке идемпотентности.
issue_has_recent_marker() {
    local _num="$1"
    local _window_start_iso
    _window_start_iso="$(date -u -d "@$(( $(date -u +%s) - WINDOW_HOURS * 3600 ))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || true)"
    [ -z "$_window_start_iso" ] && return 1

    gh api "repos/${GH_REPO}/issues/${_num}/comments?per_page=30" \
        --jq "[.[] | select(.created_at >= \"${_window_start_iso}\") | .body] | .[]" 2>/dev/null \
        | grep -qF "$_marker_tag"
}

# find_merged_pr_helper() — НЕ вызывается напрямую из main loop (он
# инлайнен для перехвата exit code через PIPESTATUS, см. main loop ниже).
# Оставлено как документация regex'а: exact-match `(#NNN)([^0-9]|$)`
# отсекает false-positive от PR с похожим номером в title.
# ВАЖНО: gh search ловит ЛЮБОЙ issue-number в title (например, PR для #1595
# может содержать "AV-1 #1595" и попасть под поиск "#1605"). Чтобы избежать
# false-positive, фильтруем по exact `#NNNN` в title — issue должен быть
# ПЕРВЫМ или ЕДИНСТВЕННЫМ issue-референсом.

# is_in_base <sha> <base_branch> — проверяет, что mergeCommit sha присутствует
# в base branch (через `git branch --contains`). Возвращает 0 если найден.
# REPO_DIR должен указывать на локальный клон (для hermes-share это
# /home/builder/hermes-share/rob_box_project, для build-host — fallback на
# $REPO_DIR из env, либо cwd).
is_in_base() {
    local _sha="$1" _base="$2"
    local _repo_dir="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
    [ -d "$_repo_dir/.git" ] || return 1
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
# Два режима:
#   A. Default — scan через gh issue list (label filter).
#   B. CONFLICT_SWEEP_ISSUE_NUM=NNN [NNN ...] — обрабатываем только
#      перечисленные issue'ы (one-shot, например для cleanup уже разрешённого
#      конфликта где labels сняли руками а issue остался OPEN).
_issues_to_check=()
if [ -n "${CONFLICT_SWEEP_ISSUE_NUM:-}" ]; then
    # режим B: one-shot
    for n in ${CONFLICT_SWEEP_ISSUE_NUM}; do
        _issues_to_check+=("$n")
    done
    echo "[$(_now_iso)] conflict-sweep: mode=B one-shot count=${#_issues_to_check[@]} repo=${GH_REPO}" >&2
else
    # режим A: label scan
    _issues_json="$(gh issue list --repo "$GH_REPO" --state open \
        --label needs-e2e --label e2e-done \
        --limit 50 --json number 2>/dev/null || echo '[]')"
    while IFS= read -r n; do
        [ -n "$n" ] && _issues_to_check+=("$n")
    done < <(printf '%s' "$_issues_json" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for it in data:
    n = it.get("number")
    if n is not None:
        print(n)
')
    echo "[$(_now_iso)] conflict-sweep: mode=A scan count=${#_issues_to_check[@]} repo=${GH_REPO}" >&2
fi

_checked=0
_closed=0
_skipped=0
_errors=0
_records=()

for issue_num in "${_issues_to_check[@]}"; do
    [ -n "$issue_num" ] || continue
    _checked=$(( _checked + 1 ))

    # idempotency: уже был наш marker за последние WINDOW_HOURS часов
    if issue_has_recent_marker "$issue_num"; then
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] conflict-sweep: SKIP #${issue_num} (recent marker found)" >&2
        continue
    fi

    # ищем merged PR. Если gh вернул non-zero (rate-limit, auth) —
    # логируем reason и считаем как SKIP (fail-closed: лучше пропустить
    # один tick, чем закрыть issue без доказательства merged-PR).
    # ВАЖНО: $(...) это subshell, который проглатывает exit code, поэтому
    # сначала запускаем gh в pipe, затем читаем ${PIPESTATUS[0]}.
    # ВАЖНО: set -o pipefail включён, и gh|exit1+python3|exit0 даст rc=1
    # из-за pipefail. Чтобы перехватить именно rc gh (а не pipefail),
    # запускаем gh отдельно (вне pipe) и пишем stdout в файл, потом
    # python3 читает из файла.
    _gh_rc=0
    : > /tmp/cs_pr_stdout_$$.txt
    : > /tmp/cs_pr_err_$$.txt
    # Отключаем pipefail временно для одного вызова gh — нам нужен
    # РЕАЛЬНЫЙ exit code gh, а не синтетический от pipefail.
    # Также добавляем `|| _gh_rc=$?` (не `|| true`) чтобы:
    #   1. set -e не abort'нул скрипт на non-zero exit
    #   2. _gh_rc всё равно захватил реальный rc gh (после `|| true`
    #      $? был бы 0, а это нам НЕ нужно).
    set +o pipefail
    gh pr list --repo "$GH_REPO" --state merged --search "#${issue_num}" \
        --json number,mergeCommit,baseRefName,headRefName,mergedAt,title \
        > /tmp/cs_pr_stdout_$$.txt \
        2>/tmp/cs_pr_err_$$.txt || _gh_rc=$?
    _gh_rc="${_gh_rc:-0}"  # fallback если gh завершился успешно
    set -o pipefail
    if [ "$_gh_rc" -ne 0 ]; then
        _gh_err="$(cat /tmp/cs_pr_err_$$.txt 2>/dev/null | head -3 | tr '\n' ' ')"
        rm -f /tmp/cs_pr_stdout_$$.txt /tmp/cs_pr_err_$$.txt
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] conflict-sweep: SKIP #${issue_num} (gh pr list failed rc=${_gh_rc}: ${_gh_err:-no stderr})" >&2
        continue
    fi
    _pr_info="$(python3 -c "
import json, re, sys
try:
    data = json.load(open('/tmp/cs_pr_stdout_' + '$$' + '.txt'))
except Exception:
    sys.exit(0)
issue = '${issue_num}'
for pr in data:
    mc = pr.get('mergeCommit') or {}
    oid = mc.get('oid') if isinstance(mc, dict) else None
    if not oid:
        continue
    title = pr.get('title') or ''
    if re.search(r'(^|[^0-9])#' + issue + r'([^0-9]|\$)', title):
        print(f'{pr[\"number\"]}:{oid}:{pr.get(\"baseRefName\",\"\")}:{pr.get(\"headRefName\",\"\")}:{pr.get(\"mergedAt\",\"\")}')
        sys.exit(0)
" 2>/dev/null)"
    rm -f /tmp/cs_pr_stdout_$$.txt /tmp/cs_pr_err_$$.txt
    if [ -z "$_pr_info" ]; then
        _skipped=$(( _skipped + 1 ))
        echo "[$(_now_iso)] conflict-sweep: SKIP #${issue_num} (no merged PR found)" >&2
        continue
    fi
    # parse PR#:sha:baseRef:headRef:mergedAt
    _pr_num="$(printf '%s' "$_pr_info" | awk -F: '{print $1}')"
    _merge_sha="$(printf '%s' "$_pr_info" | awk -F: '{print $2}')"
    _pr_base="$(printf '%s' "$_pr_info" | awk -F: '{print $3}')"
    _pr_head="$(printf '%s' "$_pr_info" | awk -F: '{print $4}')"
    _pr_merged_at="$(printf '%s' "$_pr_info" | awk -F: '{print $5}')"

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
        echo "[$(_now_iso)] conflict-sweep: SKIP #${issue_num} PR #${_pr_num} sha=${_merge_sha:0:10} (not in any base branch)" >&2
        continue
    fi

    # читаем body issue для извлечения card_id
    _issue_body="$(gh issue view "$issue_num" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || echo "")"
    _card_id="$(extract_card_id_from_body "$_issue_body" || true)"

    _body_text="${_marker_tag} action=closing reason=label-conflict-needs-e2e-and-e2e-done-pr-merged pr=#${_pr_num} sha=${_merge_sha:0:10} base=${_in_base_branch} head=${_pr_head} mergedAt=${_pr_merged_at}"
    if [ -n "$_card_id" ]; then
        _body_text="${_body_text} card=${_card_id}"
    fi

    if [ "$DRY_RUN" = "true" ]; then
        echo "[$(_now_iso)] conflict-sweep: [DRY-RUN] #${issue_num} would-close card=${_card_id:-NONE} pr=#${_pr_num}" >&2
        _closed=$(( _closed + 1 ))
        continue
    fi

    # side-effects: comment + close + remove labels
    _success=1
    if ! gh issue comment "$issue_num" --repo "$GH_REPO" --body "$_body_text" >/dev/null 2>&1; then
        _success=0
        echo "[$(_now_iso)] conflict-sweep: ERROR posting comment to #${issue_num} PR #${_pr_num}" >&2
    fi
    if [ "$_success" -eq 1 ] && ! gh issue close "$issue_num" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
        _success=0
        echo "[$(_now_iso)] conflict-sweep: ERROR closing #${issue_num} PR #${_pr_num}" >&2
    fi
    # cleanup labels: remove needs-e2e и e2e-done (no-op если их нет)
    if [ "$_success" -eq 1 ]; then
        # gh issue edit --remove-label идемпотентен: ошибка если метки нет
        # НЕ критична для success — главное issue закрыт.
        gh issue edit "$issue_num" --repo "$GH_REPO" \
            --remove-label "needs-e2e" --remove-label "e2e-done" >/dev/null 2>&1 || true
    fi

    if [ "$_success" -eq 1 ]; then
        _closed=$(( _closed + 1 ))
        _records+=("$(printf '%s\tissue #%s\tPR #%s\tsha=%s\tbase=%s\tcard=%s' \
            "$(_now_iso)" "$issue_num" "$_pr_num" "${_merge_sha:0:10}" "$_in_base_branch" "${_card_id:-NONE}")")
        echo "[$(_now_iso)] conflict-sweep: CLOSED #${issue_num} PR #${_pr_num} sha=${_merge_sha:0:10} base=${_in_base_branch} card=${_card_id:-NONE}" >&2
    else
        _errors=$(( _errors + 1 ))
    fi
done

# --- structured summary -----------------------------------------------------
echo "[$(_now_iso)] conflict-sweep: ✓ done checked=${_checked} closed=${_closed} skipped=${_skipped} errors=${_errors} repo=${GH_REPO}" >&2

# --- write stats log -------------------------------------------------------
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-conflict-sweep.log}"
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# conflict-sweep snapshot %s\n' "$(_now_iso)"
    printf 'timestamp\tissue\tpr\tsha\tbase\tcard\n'
    for r in "${_records[@]:-}"; do
        [ -n "$r" ] && printf '%s\n' "$r"
    done
    printf '# checked=%s closed=%s skipped=%s errors=%s repo=%s dry_run=%s\n' \
        "$_checked" "$_closed" "$_skipped" "$_errors" "$GH_REPO" "$DRY_RUN"
} >> "$LOG_FILE" 2>/dev/null || true

# --- exit code -------------------------------------------------------------
# exit 2 если закрыли хоть один (alert для cron), exit 0 если ничего не
# закрыли (норма). exit 1 только если gh auth упала (выше).
if [ "$_closed" -gt 0 ] && [ "$DRY_RUN" != "true" ]; then
    exit 2
fi
exit 0
