#!/bin/bash
# ============================================================================
# test_e2e_process_pr_list_fallback.sh — bug(process) t_7766fe44
#
# Регрессия: `gh pr list --head X --json` идёт через GraphQL. При исчерпании
# GraphQL-квоты (5000/h) — gh-list возвращает [] или error, e2e-process
# пропускает issues с needs-e2e как «no PR for <branch> — skip» → 12+ issues
# голодают. PR (включая #1561/#1565) фактически найти невозможно.
#
# Workaround (ретро t_7766fe44, 25.08):
#   - gh_pr_state_by_head() — GraphQL primary, REST fallback
#     /pulls?head=OWNER:BRANCH&state=all
#   - gh_pr_open_by_title() — GraphQL primary, REST search fallback
#     /search/issues?q=N+repo:...+type:pr+is:open+in:title
#   - find_open_pr_by_issue() делегирует в gh_pr_open_by_title()
#
# Сценарии (для каждой функции):
#   A. gh-list возвращает НЕпустой массив → используется gh-list,
#      fallback НЕ срабатывает
#   B. gh-list возвращает [] (CLI bug или rate-limit), REST — N
#      → fallback → N entries в выводе функции
#   C. gh-list [], REST [] (действительно пусто) → функция возвращает []
#      (для downstream это эквивалентно «нет PR»)
#   D. GraphQL error (rate-limit) → функция не падает, REST fallback
#      возвращает данные
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_pr_list_fallback.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# --- Test registry ----------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass() {
    local name="$1"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_PASSED=$((TESTS_PASSED+1))
    printf '  %s✓%s %s\n' "$GRN" "$END" "$name"
}

fail() {
    local name="$1" reason="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    TESTS_FAILED=$((TESTS_FAILED+1))
    FAILED_NAMES+=("$name")
    printf '  %s✗%s %s — %s\n' "$RED" "$END" "$name" "$reason"
}

assert_eq() {
    local name="$1" expected="$2" actual="$3"
    if [ "$expected" = "$actual" ]; then
        pass "$name"
    else
        fail "$name" "expected: '$expected', got: '$actual'"
    fi
}

assert_contains() {
    local name="$1" haystack="$2" needle="$3"
    if printf '%s' "$haystack" | grep -qF "$needle"; then
        pass "$name"
    else
        fail "$name" "expected to contain: '$needle', got: '$haystack'"
    fi
}

# --- Source the helpers we want to test --------------------------------------
# Source only the function definitions, NOT the main script.
GH_REPO="test-owner/test-repo"
NEEDS_E2E_LABEL="needs-e2e"
ISSUE_LIMIT=20
log() { :; }   # stub

# Extract each helper function from the script
extract_func() {
    local fname="$1"
    sed -n "/^${fname}()/,/^}$/p" "$E2E_PROCESS"
}

eval "$(extract_func gh_pr_state_by_head)"
eval "$(extract_func gh_pr_open_by_title)"

# Override gh with a mock that simulates different responses
MOCK_DIR="$(mktemp -d)"
trap "rm -rf $MOCK_DIR" EXIT

# Helper state files (exported so mock gh can see them)
export MOCK_DIR
export MOCK_GH_LIST_RESULT="$MOCK_DIR/gh_list_result"
export MOCK_REST_PULLS_RESULT="$MOCK_DIR/rest_pulls_result"
export MOCK_REST_SEARCH_RESULT="$MOCK_DIR/rest_search_result"
export MOCK_GH_LIST_ERROR="$MOCK_DIR/gh_list_error"

cat > "$MOCK_DIR/gh" <<'EOF'
#!/bin/bash
# Mock gh CLI: returns pre-set state, records the call for inspection.
RECORD="$MOCK_DIR/calls.log"
echo "ARGV: $*" >> "$RECORD"

# What command was invoked?
cmd="$1"
shift
# Skip subcommand if present (e.g. "pr list" → consume "list" as $1)
sub=""
if [ $# -gt 0 ]; then
    case "$1" in
        -*) ;;  # flag, not subcommand
        *) sub="$1"; shift ;;
    esac
fi
# Now consume remaining flags
while [ $# -gt 0 ]; do
    case "$1" in
        --repo) GH_REPO_MOCK="$2"; shift 2 ;;
        --head) HEAD_MOCK="$2"; shift 2 ;;
        --state) STATE_MOCK="$2"; shift 2 ;;
        --search) SEARCH_MOCK="$2"; shift 2 ;;
        --label) LABEL_MOCK="$2"; shift 2 ;;
        --limit) LIMIT_MOCK="$2"; shift 2 ;;
        --json) JSON_FIELDS_MOCK="$2"; shift 2 ;;
        --jq) JQ_MOCK="$2"; shift 2 ;;
        *) shift ;;
    esac
done

if [ "$cmd" = "pr" ] && [ "$sub" = "list" ]; then
    # Simulate GraphQL failure if MOCK_GH_LIST_ERROR is set
    if [ -f "$MOCK_GH_LIST_ERROR" ]; then
        cat "$MOCK_GH_LIST_ERROR" >&2
        exit 1
    fi
    # Return gh-list result if set
    if [ -f "$MOCK_GH_LIST_RESULT" ]; then
        cat "$MOCK_GH_LIST_RESULT"
        exit 0
    fi
    echo '[]'
    exit 0
fi

if [ "$cmd" = "api" ]; then
    # `gh api ENDPOINT [--jq EXPR]` — endpoint is positional, no subcommand
    endpoint="$sub"
    if printf '%s' "$endpoint" | grep -q '^repos/.*/pulls?head='; then
        if [ -f "$MOCK_REST_PULLS_RESULT" ]; then
            cat "$MOCK_REST_PULLS_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    if printf '%s' "$endpoint" | grep -q '^search/issues'; then
        if [ -f "$MOCK_REST_SEARCH_RESULT" ]; then
            cat "$MOCK_REST_SEARCH_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    echo '[]'
    exit 0
fi

# rate_limit / auth etc → return neutral
echo '[]'
EOF
chmod +x "$MOCK_DIR/gh"

# Make our mock take precedence
PATH="$MOCK_DIR:$PATH"
export PATH

# Helper to set mock state
set_gh_list() { printf '%s' "$1" > "$MOCK_GH_LIST_RESULT"; }
set_rest_pulls() { printf '%s' "$1" > "$MOCK_REST_PULLS_RESULT"; }
set_rest_search() { printf '%s' "$1" > "$MOCK_REST_SEARCH_RESULT"; }
set_gh_list_error() { touch "$MOCK_GH_LIST_ERROR"; }
clear_mocks() {
    rm -f "$MOCK_GH_LIST_RESULT" "$MOCK_REST_PULLS_RESULT" "$MOCK_REST_SEARCH_RESULT" "$MOCK_GH_LIST_ERROR"
    rm -f "$MOCK_DIR/calls.log"
}

# ============================================================================
# Tests for gh_pr_state_by_head
# ============================================================================
echo ""
echo "=== gh_pr_state_by_head ==="

# A. gh-list непустой → fallback НЕ срабатывает
test_A_nonempty_uses_gh_list() {
    clear_mocks
    set_gh_list '[{"number":1613,"state":"OPEN","headRefName":"develop"}]'
    local _result
    _result="$(gh_pr_state_by_head "develop" "all")"
    # Fallback в REST не должен был сработать — calls.log должен НЕ содержать
    # "repos/.../pulls?head="
    if grep -q '/pulls?head=' "$MOCK_DIR/calls.log" 2>/dev/null; then
        fail "A1: gh-list непустой → REST fallback НЕ должен срабатывать" \
            "calls.log содержит /pulls?head=: $(cat $MOCK_DIR/calls.log)"
    else
        pass "A1: gh-list непустой → REST fallback НЕ сработал"
    fi
    # Output должен быть JSON-массив с number 1613
    if printf '%s' "$_result" | grep -qE '"number":[[:space:]]*1613'; then
        pass "A2: gh-list непустой → возвращён исходный JSON"
    else
        fail "A2: gh-list непустой → возвращён исходный JSON" "got: $_result"
    fi
}
test_A_nonempty_uses_gh_list

# B. gh-list пустой, REST возвращает N → fallback → N entries
test_B_empty_uses_rest() {
    clear_mocks
    set_gh_list '[]'
    set_rest_pulls '[{"number":1613,"state":"open","head":{"ref":"develop"}}]'
    local _result
    _result="$(gh_pr_state_by_head "develop" "all")"
    if printf '%s' "$_result" | grep -qE '"number":[[:space:]]*1613'; then
        pass "B1: gh-list [], REST N → fallback вернул N entries"
    else
        fail "B1: gh-list [], REST N → fallback вернул N entries" "got: $_result"
    fi
    if printf '%s' "$_result" | grep -qE '"headRefName":[[:space:]]*"develop"'; then
        pass "B2: REST fallback нормализовал head.ref → headRefName"
    else
        fail "B2: REST fallback нормализовал head.ref → headRefName" "got: $_result"
    fi
}
test_B_empty_uses_rest

# C. gh-list [], REST [] → []
test_C_both_empty() {
    clear_mocks
    set_gh_list '[]'
    set_rest_pulls '[]'
    local _result
    _result="$(gh_pr_state_by_head "no-such-branch" "all")"
    if [ "$_result" = "[]" ]; then
        pass "C1: gh-list [], REST [] → функция возвращает []"
    else
        fail "C1: gh-list [], REST [] → функция возвращает []" "got: $_result"
    fi
}
test_C_both_empty

# D. GraphQL error → функция не падает, REST fallback срабатывает
test_D_graphql_error_uses_rest() {
    clear_mocks
    set_gh_list_error
    set_rest_pulls '[{"number":1613,"state":"open","head":{"ref":"develop"}}]'
    local _result
    _result="$(gh_pr_state_by_head "develop" "all")" || true
    if printf '%s' "$_result" | grep -qE '"number":[[:space:]]*1613'; then
        pass "D1: GraphQL error → REST fallback сработал"
    else
        fail "D1: GraphQL error → REST fallback сработал" "got: $_result"
    fi
}
test_D_graphql_error_uses_rest

# ============================================================================
# Tests for gh_pr_open_by_title
# ============================================================================
echo ""
echo "=== gh_pr_open_by_title ==="

# E. gh-list непустой → fallback НЕ срабатывает
test_E_nonempty_uses_gh_list() {
    clear_mocks
    set_gh_list '[{"number":1565,"headRefName":"z-{agent}/1561-foo","mergeStateStatus":"CLEAN"}]'
    local _result
    _result="$(gh_pr_open_by_title "1561")"
    if grep -q 'search/issues' "$MOCK_DIR/calls.log" 2>/dev/null; then
        fail "E1: gh-list непустой → REST search НЕ должен срабатывать" \
            "calls.log содержит search/issues: $(cat $MOCK_DIR/calls.log)"
    else
        pass "E1: gh-list непустой → REST search НЕ сработал"
    fi
    if printf '%s' "$_result" | grep -q '^1565	'; then
        pass "E2: gh-list непустой → возвращён number\theadRefName"
    else
        fail "E2: gh-list непустой → возвращён number\theadRefName" "got: $_result"
    fi
}
test_E_nonempty_uses_gh_list

# F. gh-list [], REST search нашёл → fallback → number\theadRefName
test_F_empty_uses_rest_search() {
    clear_mocks
    set_gh_list '[]'
    # gh_pr_open_by_title uses gh api pulls/N for head ref — but our mock
    # only handles /pulls?head= and search/issues. We need to mock the
    # second-level call (pulls/N).
    # Simplify: gh_pr_open_by_title calls `gh api repos/.../pulls/N --jq .head.ref`
    # but our mock only matches endpoints by prefix. Update the mock to also
    # match /pulls/N for the second lookup.
    set_rest_search '[{"number":1565}]'
    # Override mock to handle pulls/N
    cat > "$MOCK_DIR/gh" <<'MOCK'
#!/bin/bash
RECORD="$MOCK_DIR/calls.log"
echo "ARGV: $*" >> "$RECORD"
cmd="$1"; shift
if [ "$cmd" = "api" ]; then
    endpoint="$1"; shift
    if printf '%s' "$endpoint" | grep -qE '^repos/.*/pulls/[0-9]+'; then
        echo "z-{agent}/1561-bug-voice-992-llm-music-tools-execute-mu"
        exit 0
    fi
    if printf '%s' "$endpoint" | grep -q '^repos/.*/pulls?head='; then
        if [ -f "$MOCK_REST_PULLS_RESULT" ]; then
            cat "$MOCK_REST_PULLS_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    if printf '%s' "$endpoint" | grep -q '^search/issues'; then
        shift  # --jq
        if [ -f "$MOCK_REST_SEARCH_RESULT" ]; then
            cat "$MOCK_REST_SEARCH_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    echo '[]'
    exit 0
fi
if [ "$cmd" = "pr" ] && [ "$1" = "list" ]; then
    shift
    if [ -f "$MOCK_GH_LIST_ERROR" ]; then
        cat "$MOCK_GH_LIST_ERROR" >&2
        exit 1
    fi
    if [ -f "$MOCK_GH_LIST_RESULT" ]; then
        cat "$MOCK_GH_LIST_RESULT"
        exit 0
    fi
    echo '[]'
    exit 0
fi
echo '[]'
MOCK
    chmod +x "$MOCK_DIR/gh"
    local _result
    _result="$(gh_pr_open_by_title "1561")"
    if printf '%s' "$_result" | grep -q '^1565	z-{agent}/1561-'; then
        pass "F1: gh-list [], REST search + pulls/N → вернул number\theadRefName"
    else
        fail "F1: gh-list [], REST search + pulls/N → вернул number\theadRefName" "got: $_result"
    fi
}
test_F_empty_uses_rest_search

# G. gh-list [], REST search [] → пустой результат
test_G_both_empty() {
    clear_mocks
    set_gh_list '[]'
    set_rest_search '[]'
    # gh_pr_open_by_title goes to broad-scan /issues fallback next
    # Our mock returns '[]' for unknown endpoints → should fall through
    # and ultimately return empty
    local _result
    _result="$(gh_pr_open_by_title "999999")" || true
    if [ -z "$_result" ]; then
        pass "G1: gh-list [], REST search [], broad scan [] → пустой результат"
    else
        fail "G1: gh-list [], REST search [], broad scan [] → пустой результат" "got: $_result"
    fi
}
test_G_both_empty

# ============================================================================
# Tests for find_open_pr_by_issue (delegation)
# ============================================================================
echo ""
echo "=== find_open_pr_by_issue (delegation to gh_pr_open_by_title) ==="

test_H_finds_via_rest() {
    clear_mocks
    set_gh_list '[]'
    cat > "$MOCK_DIR/gh" <<'MOCK'
#!/bin/bash
RECORD="$MOCK_DIR/calls.log"
cmd="$1"; shift
if [ "$cmd" = "api" ]; then
    endpoint="$1"; shift
    if printf '%s' "$endpoint" | grep -qE '^repos/.*/pulls/[0-9]+'; then
        echo "z-{agent}/1561-bug-voice-992-llm-music-tools-execute-mu"
        exit 0
    fi
    if printf '%s' "$endpoint" | grep -q '^repos/.*/issues?'; then
        if [ -f "$MOCK_REST_SEARCH_RESULT" ]; then
            cat "$MOCK_REST_SEARCH_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    if printf '%s' "$endpoint" | grep -q '^search/issues'; then
        shift  # --jq
        if [ -f "$MOCK_REST_SEARCH_RESULT" ]; then
            cat "$MOCK_REST_SEARCH_RESULT"
            exit 0
        fi
        echo '[]'
        exit 0
    fi
    echo '[]'
    exit 0
fi
if [ "$cmd" = "pr" ] && [ "$1" = "list" ]; then
    shift
    if [ -f "$MOCK_GH_LIST_RESULT" ]; then
        cat "$MOCK_GH_LIST_RESULT"
        exit 0
    fi
    echo '[]'
    exit 0
fi
echo '[]'
MOCK
    chmod +x "$MOCK_DIR/gh"
    # Mock search to return PR #1565
    set_rest_search '[{"number":1565,"title":"Issue #1561: foo bar"}]'
    # Need to eval find_open_pr_by_issue — extract it from the script
    eval "$(sed -n '/^find_open_pr_by_issue()/,/^}$/p' "$E2E_PROCESS")"
    local _result
    _result="$(find_open_pr_by_issue "1561")"
    if printf '%s' "$_result" | grep -q '^1565	z-{agent}/1561-'; then
        pass "H1: find_open_pr_by_issue с REST fallback находит PR #1565"
    else
        fail "H1: find_open_pr_by_issue с REST fallback находит PR #1565" "got: $_result"
    fi
}
test_H_finds_via_rest

# ============================================================================
# Summary
# ============================================================================
echo ""
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%s✗ FAILED:%s %d/%d passed (%d failed)\n' "$RED" "$END" "$TESTS_PASSED" "$TESTS_TOTAL" "$TESTS_FAILED"
    printf 'Failed tests:\n'
    for name in "${FAILED_NAMES[@]}"; do
        printf '  - %s\n' "$name"
    done
    exit 1
fi
printf '%s✓ ALL PASS:%s %d/%d\n' "$GRN" "$END" "$TESTS_PASSED" "$TESTS_TOTAL"
exit 0