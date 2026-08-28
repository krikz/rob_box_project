#!/bin/bash
# ============================================================================
# test_blocked_watchdog.sh — orphan blocked-card watchdog (ретро t_1d0426e3)
#
# Регресс-гард для agent-flow-blocked-watchdog.sh. Тестируем чистую логику
# (без сетевых вызовов) через PATH-hijack: подставляем mock-gh и mock-git,
# которые возвращают заранее заданные JSON / вывод.
#
# Scenarios:
#   W1. no_open_issues: gh issue list возвращает [] → checked=0, closed=0
#   W2. no_merged_pr_for_issue: issue #1605 без merged PR → SKIP, closed=0
#   W3. merged_pr_but_not_in_base: merged PR sha НЕ в base branches → SKIP
#   W4. merged_pr_in_base_closes_issue: DRY_RUN=true → closed counter
#       инкрементируется, NO actual close (grep journal)
#   W5. exact_match_filter: PR с похожим но не равным #NNNN в title
#       НЕ считается merged-for-this-issue
#   W6. dry_run_idempotent: повторный DRY_RUN → всё ещё no-op
#
# Run:
#   bash scripts/agent_flow/tests/test_blocked_watchdog.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-blocked-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin" "$WORK/repo/.git"

# Mock git — отдаёт branch --contains как настроим через MOCK_BRANCH_CONTAINS_FILE
cat > "$WORK/bin/git" <<'EOF'
#!/bin/bash
# Пропускаем `-C <dir>` если есть
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
# поддерживаем branch --contains и branch -a --contains
if [ "${_args[0]}" = "branch" ]; then
    if [ -n "${MOCK_BRANCH_CONTAINS_FILE:-}" ] && [ -f "${MOCK_BRANCH_CONTAINS_FILE}" ]; then
        cat "${MOCK_BRANCH_CONTAINS_FILE}"
    fi
    exit 0
fi
echo ""
EOF
chmod +x "$WORK/bin/git"

# Mock gh — программируемое поведение
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
# Маршрутизация по MOCK_GH_JSON_<n>
case "$1" in
    "auth")
        # gh auth status — всегда OK
        echo "✓ logged in"
        exit 0
        ;;
    "issue")
        case "$2" in
            "list")
                # gh issue list --repo X --state Y --label Z --limit N --json N,T
                if [ -n "${MOCK_GH_ISSUE_LIST:-}" ] && [ -f "${MOCK_GH_ISSUE_LIST}" ]; then
                    cat "${MOCK_GH_ISSUE_LIST}"
                else
                    echo '[]'
                fi
                exit 0
                ;;
            "view")
                # gh issue view N --repo X --json body  ИЛИ --json body,labels,state
                if [ -n "${MOCK_GH_ISSUE_VIEW:-}" ] && [ -f "${MOCK_GH_ISSUE_VIEW}" ]; then
                    # выбираем ключ по файлу MOCK_GH_ISSUE_VIEW_FOR_NUM
                    if [ -n "${MOCK_GH_ISSUE_VIEW_FOR_NUM:-}" ]; then
                        python3 -c "
import json, sys
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
                # Записываем side-effect в JOURNAL (для assert)
                echo "MOCKED: gh issue $2 $*" >> "$WORK/journal.txt"
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

run_watchdog() {
    local extra_env="$1"
    > "$WORK/journal.txt"
    # Парсим extra_env: KEY=VAL KEY2=VAL2 → экспортируем каждый
    local kv
    local exports=""
    for kv in $extra_env; do
        exports="$exports export $kv;"
    done
    bash -c "
        export PATH='$WORK/bin:/usr/bin:/bin'
        export REPO_DIR='$WORK/repo'
        export GH_REPO='krikz/rob_box_project'
        export BLOCKED_WATCHDOG_DRY_RUN='${BLOCKED_WATCHDOG_DRY_RUN:-true}'
        $exports
        bash '$WATCHDOG_SH' 2>/tmp/wd_stderr.txt
    "
    local rc=$?
    STDERR_LOG="$(cat /tmp/wd_stderr.txt 2>/dev/null)"
    JOURNAL="$(cat $WORK/journal.txt 2>/dev/null)"
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

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# ============================================================================
# W1. no_open_issues
# ============================================================================
echo '[]' > "$WORK/issues.json"
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json"
assert_contains "checked=0 closed=0" "$STDERR_LOG" "W1: пустой список issues → checked=0 closed=0" || fail "W1"

# ============================================================================
# W2. issue без merged PR — пропущен
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1605,"title":"AV-11 mixed-mode test"}]
JSON
echo '[]' > "$WORK/prs.json"
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json"
assert_contains "SKIP #1605 (no merged PR found)" "$STDERR_LOG" "W2: issue без merged PR → SKIP" || fail "W2"

# ============================================================================
# W3. merged PR sha НЕ в base branches → SKIP
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1605,"title":"AV-11 mixed-mode test"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1700,"mergeCommit":{"oid":"abcdef1234567890"},"baseRefName":"z-agent/wip","headRefName":"z-agent/wip","mergedAt":"2026-08-28T00:00:00Z","title":"[AV-11] #1605 attempt-7"}]
JSON
echo "" > "$WORK/branch_contains.txt"  # пустой — sha ни в одной ветке
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1605 PR #1700" "$STDERR_LOG" "W3: merged PR sha не в base → SKIP" || fail "W3"
assert_contains "closed=0" "$STDERR_LOG" "W3: closed=0 (не закрыт)" || fail "W3 closed counter"

# ============================================================================
# W4. merged PR sha В feature/avatar → DRY_RUN → counter инкрементируется,
#      но NO actual close в journal.
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1605,"title":"AV-11 mixed-mode test"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1701,"mergeCommit":{"oid":"abcdef1234567890"},"baseRefName":"feature/avatar","headRefName":"z-agent/wip","mergedAt":"2026-08-28T01:00:00Z","title":"[AV-11] #1605 attempt-8"}]
JSON
echo "  feature/avatar" > "$WORK/branch_contains.txt"  # содержит нужную ветку
cat > "$WORK/issue_view.json" <<'JSON'
{"_default":{"body":"linked kanban: t_deadbeef","state":"OPEN","labels":[{"name":"needs-e2e"}]}}
JSON
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "[DRY-RUN] #1605 would-close" "$STDERR_LOG" "W4: dry-run показывает would-close" || fail "W4 dry-run log"
assert_contains "closed=1" "$STDERR_LOG" "W4: closed counter = 1 (в dry-run)" || fail "W4 closed counter"
if [ -s "$WORK/journal.txt" ]; then
    echo "  FAIL: W4: при DRY_RUN должны быть NO side-effects в journal:"
    cat "$WORK/journal.txt"
    fail "W4 journal"
else
    echo "  ok: W4: DRY_RUN не делает side-effects (journal пустой)"
fi

# ============================================================================
# W5. exact-match filter: PR #1606 содержит #1605 в title случайно — НЕ должен
#      считаться merged-PR-для-#1605
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1605,"title":"AV-11 mixed-mode test"}]
JSON
# PR с #1595 в title и хешем, который случайно содержит "1605" (поиск по
# `gh pr list --search "#1605"` найдёт этот PR), но НЕ содержит ровно "#1605"
cat > "$WORK/prs.json" <<'JSON'
[{"number":1606,"mergeCommit":{"oid":"abcdef1234567890"},"baseRefName":"feature/avatar","headRefName":"z-agent/decomp","mergedAt":"2026-08-24T21:11:57Z","title":"feat(avatar AV-1 #1595): decomposition"}]
JSON
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/prs.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1605 (no merged PR found)" "$STDERR_LOG" "W5: PR с похожим но не равным #NNNN → SKIP" || fail "W5"

# ============================================================================
# W6. idempotency-marker: повторный вызов с тем же marker в comments
#      → SKIP без side-effects
# ============================================================================
cat > "$WORK/issues.json" <<'JSON'
[{"number":1605,"title":"AV-11 mixed-mode test"}]
JSON
cat > "$WORK/prs.json" <<'JSON'
[{"number":1701,"mergeCommit":{"oid":"abcdef1234567890"},"baseRefName":"feature/avatar","headRefName":"z-agent/wip","mergedAt":"2026-08-28T01:00:00Z","title":"[AV-11] #1605 attempt-8"}]
JSON
echo "  feature/avatar" > "$WORK/branch_contains.txt"
# Комментарии с нашим marker (только что оставленный)
NOW="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
cat > "$WORK/comments.json" <<EOF
[{"body":"🤖 [agent:devops] script=agent-flow-blocked-watchdog action=closing reason=orphan-needs-e2e-with-merged-pr pr=#1701","created_at":"$NOW"}]
EOF
run_watchdog "MOCK_GH_ISSUE_LIST=$WORK/issues.json MOCK_GH_PR_LIST=$WORK/prs.json MOCK_GH_API=$WORK/comments.json MOCK_GH_ISSUE_VIEW=$WORK/issue_view.json MOCK_BRANCH_CONTAINS_FILE=$WORK/branch_contains.txt"
assert_contains "SKIP #1605 (recent marker found)" "$STDERR_LOG" "W6: idempotency marker → SKIP" || fail "W6"
if [ -s "$WORK/journal.txt" ]; then
    echo "  FAIL: W6: при idempotency-marker должны быть NO side-effects"
    cat "$WORK/journal.txt"
    fail "W6 journal"
else
    echo "  ok: W6: idempotency-marker → no side-effects"
fi

pass "blocked-watchdog: все 6 кейсов прошли"
exit 0