#!/bin/bash
# ============================================================================
# test_conflict_sweep.sh — regression test для agent-flow-conflict-sweep.sh
# (ретро t_8fba04b9, issue #1977).
#
# Тестируем чистую логику (без сетевых вызовов) через PATH-hijack:
# подставляем mock-gh и mock-git, которые возвращают заранее заданные
# JSON / вывод. Покрывает контракт скрипта:
#
# Scenarios:
#   C1. no_conflict_issues: gh issue list возвращает [] → checked=0
#   C2. issue_without_merged_pr: SKIP, no side-effects
#   C3. merged_pr_but_not_in_base: SKIP, no close
#   C4. merged_pr_in_develop_closes_issue: comment + close + remove labels
#   C5. dry_run_mode: log "would-close", NO side-effects в journal
#   C6. one_shot_mode_CONFLICT_SWEEP_ISSUE_NUM: scan конкретный issue
#       даже если label-filter его не находит (для #1977 после ручного
#       label cleanup)
#   C7. exact_match_filter: PR с похожим но не равным #NNNN → SKIP
#   C8. idempotency_marker: повторный вызов с marker в comments → SKIP
#   C9. label_cleanup_idempotent: если labels уже сняты, gh issue edit
#       может вернуть ошибку, но close всё равно success (close не
#       зависит от успешности label-cleanup)
#
# Run:
#   bash scripts/agent_flow/tests/test_conflict_sweep.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SWEEP_SH="${SWEEP_SH:-$TEST_DIR/../agent-flow-conflict-sweep.sh}"

[ -f "$SWEEP_SH" ] || { echo "FAIL: $SWEEP_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required for mock data"; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin" "$WORK/repo/.git"

# Mock git — поддерживает branch --contains и branch -a --contains
cat > "$WORK/bin/git" <<'EOF'
#!/bin/bash
_args=()
_skip_next=0
for arg in "$@"; do
    if [ "$_skip_next" = "1" ]; then _skip_next=0; continue; fi
    case "$arg" in
        -C) _skip_next=1 ;;
        *) _args+=("$arg") ;;
    esac
done
case "${_args[0]} ${_args[1]:-}" in
    "--version"|"") echo "mock git"; exit 0 ;;
esac
if [ "${_args[0]}" = "branch" ]; then
    if [ -n "${MOCK_BRANCH_CONTAINS_FILE:-}" ] && [ -f "${MOCK_BRANCH_CONTAINS_FILE}" ]; then
        cat "${MOCK_BRANCH_CONTAINS_FILE}"
    fi
    exit 0
fi
echo ""
EOF
chmod +x "$WORK/bin/git"

# Mock gh — программируемое поведение по ENV-флагам
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
case "$1" in
    "auth")
        echo "✓ logged in"
        exit 0
        ;;
    "issue")
        case "$2" in
            "list")
                if [ -n "${MOCK_GH_ISSUE_LIST:-}" ] && [ -f "${MOCK_GH_ISSUE_LIST}" ]; then
                    cat "${MOCK_GH_ISSUE_LIST}"
                else
                    echo '[]'
                fi
                exit 0
                ;;
            "view")
                if [ -n "${MOCK_GH_ISSUE_VIEW:-}" ] && [ -f "${MOCK_GH_ISSUE_VIEW}" ]; then
                    if [ -n "${MOCK_GH_ISSUE_VIEW_FOR_NUM:-}" ]; then
                        python3 -c "
import json
d = json.load(open('${MOCK_GH_ISSUE_VIEW}'))
key = '${MOCK_GH_ISSUE_VIEW_FOR_NUM}'
print(json.dumps(d.get(key, d.get('_default', {}))))
"
                    else
                        cat "${MOCK_GH_ISSUE_VIEW}"
                    fi
                else
                    echo '{}'
                fi
                exit 0
                ;;
            "comment"|"close")
                echo "MOCKED: gh issue $2 $*" >> "${JOURNAL_FILE:-/tmp/conflict-sweep-journal.txt}"
                exit 0
                ;;
            "edit")
                # gh issue edit N --repo X --remove-label A --remove-label B
                # Записываем side-effect + simulate "label not present" в
                # одном из кейсов (C9).
                echo "MOCKED: gh issue edit $*" >> "${JOURNAL_FILE:-/tmp/conflict-sweep-journal.txt}"
                if [ "${MOCK_GH_EDIT_FAIL_LABELS:-}" = "true" ]; then
                    # Simulate gh error (label not present) — НЕ exit 0,
                    # чтобы test проверил, что close всё равно success.
                    echo "MOCKED-ERR: label not found" >&2
                    exit 1
                fi
                exit 0
                ;;
        esac
        ;;
    "pr")
        case "$2" in
            "list")
                if [ -n "${MOCK_GH_PR_LIST:-}" ] && [ -f "${MOCK_GH_PR_LIST}" ]; then
                    cat "${MOCK_GH_PR_LIST}"
                else
                    echo '[]'
                fi
                exit 0
                ;;
            "view")
                if [ -n "${MOCK_GH_PR_VIEW:-}" ] && [ -f "${MOCK_GH_PR_VIEW}" ]; then
                    cat "${MOCK_GH_PR_VIEW}"
                else
                    echo '{}'
                fi
                exit 0
                ;;
        esac
        ;;
    "api")
        if [ -n "${MOCK_GH_API:-}" ] && [ -f "${MOCK_GH_API}" ]; then
            cat "${MOCK_GH_API}"
        else
            echo '[]'
        fi
        exit 0
        ;;
esac
echo ""
EOF
chmod +x "$WORK/bin/gh"

run_sweep() {
    local extra_env="$1"
    local journal="$WORK/journal.txt"
    > "$journal"
    local kv exports=""
    for kv in $extra_env; do
        exports="$exports export $kv;"
    done
    JOURNAL_FILE="$journal" bash -c "
        export PATH='$WORK/bin:/usr/bin:/bin'
        export REPO_DIR='$WORK/repo'
        export GH_REPO='krikz/rob_box_project'
        export CONFLICT_SWEEP_DRY_RUN='${CONFLICT_SWEEP_DRY_RUN:-false}'
        $exports
        bash '$SWEEP_SH' 2>/tmp/cs_stderr.txt
    "
    local rc=$?
    STDERR_LOG="$(cat /tmp/cs_stderr.txt 2>/dev/null)"
    JOURNAL="$(cat "$journal" 2>/dev/null)"
    return $rc
}

assert_contains() {
    local needle="$1" haystack="$2" msg="$3"
    if printf '%s' "$haystack" | grep -qF "$needle"; then
        echo "  ok: $msg"
    else
        echo "  FAIL: $msg (expected to contain: $needle)"
        echo "  actual: $haystack"
        return 1
    fi
}

assert_not_contains() {
    local needle="$1" haystack="$2" msg="$3"
    if printf '%s' "$haystack" | grep -qF "$needle"; then
        echo "  FAIL: $msg (must NOT contain: $needle)"
        echo "  actual: $haystack"
        return 1
    else
        echo "  ok: $msg"
    fi
}

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# ============================================================================
# C1. no_conflict_issues
# ============================================================================
echo '[]' > "$WORK/issues.json"
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json"
assert_contains "checked=0 closed=0" "$STDERR_LOG" "C1: пустой список issues → checked=0 closed=0" || fail "C1"

# ============================================================================
# C2. issue без merged PR → SKIP
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
echo '[]' > "$WORK/prs.json"
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json"
assert_contains "SKIP #1977 (no merged PR found)" "$STDERR_LOG" "C2: issue без merged PR → SKIP" || fail "C2"
assert_contains "closed=0" "$STDERR_LOG" "C2: closed=0" || fail "C2 closed counter"

# ============================================================================
# C3. merged PR sha НЕ в base branches → SKIP
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1979,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"z-agent/wip","headRefName":"z-agent/wip","mergedAt":"2026-09-07T11:14:33Z","title":"feat(e2e): #1977 synth fallback chain"}]
JSON
echo "" > "$WORK/branch_contains.txt"
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1977 PR #1979" "$STDERR_LOG" "C3: merged PR sha не в base → SKIP" || fail "C3"
assert_contains "closed=0" "$STDERR_LOG" "C3: closed=0" || fail "C3 closed counter"

# ============================================================================
# C4. merged PR sha В develop → comment + close + remove labels
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1979,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"develop","headRefName":"z-agent/1977-feat-e2e-synth","mergedAt":"2026-09-07T11:14:33Z","title":"feat(e2e): #1977 synth fallback chain"}]
JSON
echo "  develop" > "$WORK/branch_contains.txt"
cat > "$WORK/issue_view.json" <<'JSON'
{"_default":{"body":"linked kanban: t_8fba04b9","state":"OPEN","labels":[{"name":"needs-e2e"},{"name":"e2e-done"}]}}
JSON
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "CLOSED #1977 PR #1979" "$STDERR_LOG" "C4: closed log line" || fail "C4 log"
assert_contains "closed=1" "$STDERR_LOG" "C4: closed counter = 1" || fail "C4 closed counter"
# journal должен содержать comment + close + edit (label cleanup).
# mock-gh пишет: "MOCKED: gh issue <subcmd> $*", поэтому ищем подстроку
# "gh issue comment" + "1977" рядом — это надёжнее, чем искать точную
# форму (mock может менять $@ порядок).
assert_contains "gh issue comment" "$JOURNAL" "C4: comment в journal" || fail "C4 comment"
assert_contains "gh issue close" "$JOURNAL" "C4: close в journal" || fail "C4 close"
assert_contains "gh issue edit" "$JOURNAL" "C4: edit (label cleanup) в journal" || fail "C4 edit"
assert_contains "remove-label" "$JOURNAL" "C4: --remove-label в journal" || fail "C4 remove-label"
# number 1977 в журнале должно присутствовать (хотя бы один раз)
assert_contains "1977" "$JOURNAL" "C4: issue #1977 упомянут в journal" || fail "C4 1977 in journal"

# ============================================================================
# C5. dry_run_mode: log "would-close", NO side-effects в journal
# ============================================================================
> "$WORK/journal.txt"
export CONFLICT_SWEEP_DRY_RUN=true
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
unset CONFLICT_SWEEP_DRY_RUN
assert_contains "[DRY-RUN] #1977 would-close" "$STDERR_LOG" "C5: dry-run показывает would-close" || fail "C5 dry-run log"
assert_contains "closed=1" "$STDERR_LOG" "C5: closed counter = 1 (в dry-run)" || fail "C5 closed counter"
if [ -s "$WORK/journal.txt" ]; then
    echo "  FAIL: C5: при DRY_RUN должны быть NO side-effects в journal:"
    cat "$WORK/journal.txt"
    fail "C5 journal"
else
    echo "  ok: C5: DRY_RUN не делает side-effects (journal пустой)"
fi

# ============================================================================
# C6. one-shot mode CONFLICT_SWEEP_ISSUE_NUM=1977 — сканируем конкретный
#     issue даже если label-filter пустой (для cleanup уже разрешённого
#     конфликта где labels сняли руками).
# ============================================================================
> "$WORK/journal.txt"
echo '[]' > "$WORK/empty_issue_list.json"  # label-scan пустой
cat > "$WORK/prs.json" <<'JSON'
[{"number":1979,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"develop","headRefName":"z-agent/1977","mergedAt":"2026-09-07T11:14:33Z","title":"feat(e2e): #1977 synth fallback chain"}]
JSON
run_sweep "CONFLICT_SWEEP_ISSUE_NUM=1977 MOCK_GH_ISSUE_LIST=$WORK/empty_issue_list.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "mode=B one-shot count=1" "$STDERR_LOG" "C6: one-shot mode detected" || fail "C6 mode log"
assert_contains "CLOSED #1977" "$STDERR_LOG" "C6: one-shot close #1977" || fail "C6 close log"
assert_contains "gh issue close" "$JOURNAL" "C6: close в journal для one-shot" || fail "C6 journal"
assert_contains "1977" "$JOURNAL" "C6: issue #1977 в journal" || fail "C6 1977 in journal"

# ============================================================================
# C7. exact-match filter: PR с похожим но не равным #NNNN → SKIP
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
# PR содержит #1957 в title (похоже но не равно)
cat > "$WORK/prs.json" <<'JSON'
[{"number":1978,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"develop","headRefName":"z-agent/other","mergedAt":"2026-09-07T11:14:33Z","title":"feat: refactor #1957 fallback"}]
JSON
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1977 (no merged PR found)" "$STDERR_LOG" "C7: exact-match фильтр → SKIP для PR #1957" || fail "C7"

# ============================================================================
# C8. idempotency marker: повторный вызов с marker в comments → SKIP
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1979,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"develop","headRefName":"z-agent/1977","mergedAt":"2026-09-07T11:14:33Z","title":"feat(e2e): #1977 synth fallback chain"}]
JSON
NOW="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
cat > "$WORK/comments.json" <<EOF
[{"body":"🤖 [agent:devops] script=agent-flow-conflict-sweep action=closing reason=label-conflict-needs-e2e-and-e2e-done-pr-merged pr=#1979","created_at":"$NOW"}]
EOF
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/comments.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1977 (recent marker found)" "$STDERR_LOG" "C8: idempotency marker → SKIP" || fail "C8"
if [ -s "$WORK/journal.txt" ]; then
    echo "  FAIL: C8: при idempotency-marker должны быть NO side-effects"
    cat "$WORK/journal.txt"
    fail "C8 journal"
else
    echo "  ok: C8: idempotency-marker → no side-effects"
fi

# ============================================================================
# C9. label_cleanup_idempotent: если labels уже сняты, gh issue edit
#     возвращает ошибку, но close ВСЁ РАВНО success (close не зависит от
#     успешности label-cleanup). Это покрывает кейс #1977 где labels
#     сняли руками 21:07 — comment + close должны пройти.
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1979,"mergeCommit":{"oid":"abc1234567890def"},"baseRefName":"develop","headRefName":"z-agent/1977","mergedAt":"2026-09-07T11:14:33Z","title":"feat(e2e): #1977 synth fallback chain"}]
JSON
echo "  develop" > "$WORK/branch_contains.txt"
# issue_view с labels уже снятыми
cat > "$WORK/issue_view_cleaned.json" <<'JSON'
{"_default":{"body":"linked kanban: t_8fba04b9","state":"OPEN","labels":[{"name":"hermes"},{"name":"agent:devops"}]}}
JSON
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view_cleaned.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt MOCK_GH_EDIT_FAIL_LABELS=true"
assert_contains "CLOSED #1977" "$STDERR_LOG" "C9: close success даже когда label-cleanup fails" || fail "C9 log"
assert_contains "closed=1" "$STDERR_LOG" "C9: closed=1 несмотря на edit fail" || fail "C9 counter"
# edit вызван, и close вызван — обе команды в journal
assert_contains "gh issue close" "$JOURNAL" "C9: close в journal" || fail "C9 close journal"
assert_contains "gh issue edit" "$JOURNAL" "C9: edit в journal (label cleanup attempted)" || fail "C9 edit journal"

# ============================================================================
# C10. gh pr list returns non-zero (rate-limit / auth) — script logs SKIP
#      with gh-error reason, NO side-effects. Fail-closed behaviour:
#      лучше пропустить tick, чем закрыть issue без доказательства
#      merged-PR (это та же защита от silent-fail, что и в merge-gate).
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1977,"title":"feat(e2e): synth fallback chain"}]
JSON
# Создаём mock-gh, который на pr list возвращает exit 1 + stderr
# с rate-limit ошибкой.
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
case "$1" in
    "auth") echo "✓ logged in"; exit 0 ;;
    "issue")
        case "$2" in
            "list")
                if [ -n "${MOCK_GH_ISSUE_LIST:-}" ] && [ -f "${MOCK_GH_ISSUE_LIST}" ]; then
                    cat "${MOCK_GH_ISSUE_LIST}"
                else
                    echo '[]'
                fi
                exit 0 ;;
            "view"|"comment"|"close"|"edit") exit 0 ;;
        esac ;;
    "pr")
        case "$2" in
            "list")
                # Rate-limit симуляция: stderr + exit 1, stdout пустой
                echo "GraphQL: API rate limit exceeded" >&2
                exit 1 ;;
            "view") echo '{}'; exit 0 ;;
        esac ;;
    "api")
        if [ -n "${MOCK_GH_API:-}" ] && [ -f "${MOCK_GH_API}" ]; then
            cat "${MOCK_GH_API}"
        else
            echo '[]'
        fi
        exit 0 ;;
esac
echo ""
EOF
chmod +x "$WORK/bin/gh"
run_sweep "MOCK_GH_ISSUE_LIST=$WORK/issues.json"
assert_contains "gh pr list failed rc=1" "$STDERR_LOG" "C10: rate-limit → SKIP с reason в логе" || fail "C10 log"
assert_contains "closed=0" "$STDERR_LOG" "C10: closed=0 (fail-closed)" || fail "C10 closed counter"
# journal должен быть пустой — никаких side-effects
if [ -s "$WORK/journal.txt" ]; then
    echo "  FAIL: C10: при gh-fail должны быть NO side-effects"
    cat "$WORK/journal.txt"
    fail "C10 journal"
else
    echo "  ok: C10: gh-fail → no side-effects (fail-closed)"
fi

pass "conflict-sweep: все 10 кейсов прошли"
exit 0
