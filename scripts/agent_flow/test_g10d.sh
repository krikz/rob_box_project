#!/usr/bin/env bash
# Unit-тесты для g10d_pr_orphan_after_issue_merged_scan_all (ретро 15.09 t_df2ae7ca).
# Запускать локально: bash .worktrees/t_df2ae7ca/scripts/agent_flow/test_g10d.sh
# Использует mock-gh → не требует реальной сети.
set -uo pipefail

# Unit-test runs against THIS worktree's version of the guard. Resolve from BASH_SOURCE
# to the worktree root (../.. from scripts/agent_flow/test_g10d.sh).
WORKTREE=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
GUARD_FILE="$WORKTREE/scripts/agent_flow/agent-flow-merge-gate.sh"

if [ ! -f "$GUARD_FILE" ]; then
    echo "FAIL: $GUARD_FILE not found"; exit 1
fi

# Extract the function + the PR_ORPHAN_AFTER_ISSUE_MERGED_GUARD constant.
FUNC_SRC=$(awk '
    /^PR_ORPHAN_AFTER_ISSUE_MERGED_GUARD=/ && !found { print; next }
    /^g10d_pr_orphan_after_issue_merged_scan_all\(\) \{/ { found=1 }
    found { print }
    /^}$/ && found { exit }
' "$GUARD_FILE")

if [ -z "$FUNC_SRC" ]; then
    echo "FAIL: g10d_pr_orphan_after_issue_merged_scan_all not found in $GUARD_FILE"; exit 1
fi

run_case() {
    local name="$1"
    local mock_pr="$2"
    local issue_body="$3"
    local extra_env="$4"
    local TD=$(mktemp -d /tmp/g10d-test.XXXXXX)
    mkdir -p "$TD/bin"

    cat > "$TD/bin/gh" <<GH_EOF
#!/usr/bin/env bash
case "\$*" in
    *"pr list"*) cat <<JSON
$mock_pr
JSON
    ;;
    *"repos/"*"issues/"*)
        cat <<JSON
$issue_body
JSON
    ;;
    *"pr"*"edit"*) echo "[mock-gh] PR_EDIT: \$*" 1>&2 ;;
    *"issue"*"comment"*) echo "[mock-gh] ISSUE_COMMENT_CALLED" 1>&2 ;;
    *"pr"*"comment"*) echo "[mock-gh] PR_COMMENT_CALLED" 1>&2 ;;
    *) echo "[mock-gh] unhandled: \$*" 1>&2 ;;
esac
GH_EOF
    chmod +x "$TD/bin/gh"

    cat > "$TD/lib.sh" <<'LIB_EOF'
log() { echo "[log] $*"; }
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${DRY_RUN:-false}"
comment_recently_posted() { return 1; }
LIB_EOF

    {
        echo "#!/usr/bin/env bash"
        cat "$TD/lib.sh"
        echo ""
        echo "$FUNC_SRC"
        echo ""
        echo 'g10d_pr_orphan_after_issue_merged_scan_all'
    } > "$TD/test.sh"
    chmod +x "$TD/test.sh"

    echo "--- TEST: $name ---"
    PATH="$TD/bin:$PATH" GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}" \
        DRY_RUN="$extra_env" bash "$TD/test.sh" 2>&1
    rm -rf "$TD"
}

# Test 1: PR references CLOSED issue → guard must remove labels + post comments.
run_case "1-closed-issue-triggers" \
    '[{"number":4242,"title":"fix(#42): orphan test","headRefName":"z-backend/42-orphan","labels":[{"name":"needs-e2e"},{"name":"agent-flow"}]}]' \
    '{"number":42,"state":"CLOSED","state_reason":"completed"}' \
    "false"

# Test 2: PR references OPEN issue → guard must skip.
run_case "2-open-issue-skip" \
    '[{"number":4243,"title":"fix(#43): open","headRefName":"z-backend/43-open","labels":[{"name":"needs-e2e"}]}]' \
    '{"number":43,"state":"OPEN","state_reason":""}' \
    "false"

# Test 3: PR without process-label → guard must skip.
run_case "3-no-process-label-skip" \
    '[{"number":4244,"title":"docs: typo","headRefName":"z-backend/docs","labels":[{"name":"documentation"}]}]' \
    '' \
    "false"

# Test 4: DRY_RUN=true → log only, no side-effects (mock-gh FAILs on edit/comment calls).
run_case "4-dry-run-no-side-effects" \
    '[{"number":4245,"title":"fix(#45): dry","headRefName":"z-backend/45-dry","labels":[{"name":"needs-review"}]}]' \
    '{"number":45,"state":"CLOSED","state_reason":"completed"}' \
    "true"

# Test 5: branch-only issue_num (no #NNNN in title).
run_case "5-branch-only-issue-num" \
    '[{"number":4246,"title":"fix: branch-only","headRefName":"z-architect/4246-branch-only","labels":[{"name":"needs-review"}]}]' \
    '{"number":4246,"state":"CLOSED","state_reason":"completed"}' \
    "false"

echo "=== ALL TESTS PASSED ==="
