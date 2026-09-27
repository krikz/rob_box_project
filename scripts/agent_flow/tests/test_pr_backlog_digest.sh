
# --- Test: stale-candidate cross-check --------------------------------
test_stale_candidate_warning() {
    local now; now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local pr_json
    pr_json="$(cat <<JSON
[{"number":3035,"title":"feat: ADR-0134 reconcile","labels":[{"name":"e2e-done"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"z-devops/adr-0134"}]
JSON
)"
    local issue_json='[{"number":2754,"title":"ADR-0134 stale-candidate (race case)"}]'

    local fake_gh; fake_gh="$(make_fake_gh "$pr_json" "$issue_json")"
    local fake_path="$TEST_TMP/path-bin3"
    mkdir -p "$fake_path"
    ln -sf "$fake_gh" "$fake_path/gh"

    PATH="$fake_path:$PATH" \
    REPO_DIR="$REPO_ROOT" \
    DIGEST_DRY_RUN=true \
    DIGEST_FORCE=true \
    DIGEST_TEST_MODE=1 \
    DIGEST_STATE_DIR="$TEST_TMP" \
    DIGEST_MAX_PER_GROUP=10 \
    TELEGRAM_BOT_TOKEN="" \
        bash "$DIGEST_SCRIPT" >/dev/null 2>/dev/null
    local out; out="$(cat "$TEST_TMP/agent-flow-pr-backlog-digest.log")"
    assert_contains "stale-candidate" "$out" "stale warning header"
    assert_contains "#2754" "$out" "stale issue #2754 referenced"
}

# --- Test: production-mode без TELEGRAM_BOT_TOKEN → exit 1 ----------
test_no_token_fail() {
    local now; now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local pr_json
    pr_json="$(cat <<JSON
[{"number":1,"title":"x","labels":[],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"x"}]
JSON
)"
    local fake_gh; fake_gh="$(make_fake_gh "$pr_json" "[]")"
    local fake_path="$TEST_TMP/path-bin4"
    mkdir -p "$fake_path"
    ln -sf "$fake_gh" "$fake_path/gh"

    set +e
    # Без TELEGRAM_BOT_TOKEN, без DRY-RUN → должен fail
    PATH="$fake_path:$PATH" \
    REPO_DIR="$REPO_ROOT" \
    DIGEST_DRY_RUN=false \
    DIGEST_FORCE=true \
    DIGEST_TEST_MODE=1 \
    DIGEST_STATE_DIR="$TEST_TMP" \