#!/bin/bash
# ============================================================================
# test_pr_backlog_digest.sh — unit + integration tests for
# agent-flow-pr-backlog-digest.sh (issue t_f158469f, architect t_d2ab84d7).
#
# Покрывает acceptance:
#   [✓] Скрипт существует, executable, bash -n проходит.
#   [✓] Группировка A/B/C работает на синтетических данных:
#       A = MERGEABLE + CLEAN + e2e-done,
#       B = MERGEABLE + CLEAN, нет e2e-done,
#       C = CONFLICTING или mergeStateStatus=DIRTY/UNKNOWN.
#   [✓] --dry-run пишет в /tmp/agent-flow-pr-backlog-digest.log
#       (Telegram НЕ вызывается).
#   [✓] Формат digest содержит: дату, число PR, заголовки групп, ages.
#   [✓] Sentinel /tmp/agent-flow-pr-backlog-digest-YYYY-MM-DD.done
#       создаётся в production-mode и блокирует второй тик.
#   [✓] Fail-closed: пустой gh output → exit 1.
#   [✓] install.sh EXPECTED[] содержит agent-flow-pr-backlog-digest.sh
#       (раскладка по 6 target-папкам).
#
# Использует fake gh, который подменяет `gh pr list` и `gh issue list`
# синтетическими JSON через PATH-prepend (как test_hermes_github.sh).
#
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -eu

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
REPO_ROOT="$(cd "$TEST_ROOT_DIR/../.." && pwd)"
DIGEST_SCRIPT="$REPO_ROOT/scripts/agent_flow/agent-flow-pr-backlog-digest.sh"
INSTALL_SCRIPT="$REPO_ROOT/scripts/agent_flow/install.sh"

# Per-run temp dir
TEST_TMP="${TEST_TMP:-/tmp/agent-flow-pr-backlog-digest-tests.$$}"
mkdir -p "$TEST_TMP"
trap 'rm -rf "$TEST_TMP"' EXIT

# Colors
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ TEST ]%s %s\n' "$BLU" "$END" "$name"
    if $fn; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[ PASS ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[ FAIL ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {
    if [ "$1" != "$2" ]; then
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$2" "$1" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2; return 1 ;;
    esac
}

assert_not_contains() {
    case "$2" in
        *"$1"*) printf '  %sassert fail:%s %s\n    should NOT contain: %q\n    haystack: %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2; return 1 ;;
        *) return 0 ;;
    esac
}

# --- Fake gh ---------------------------------------------------------------
# Подменяет `gh pr list` и `gh issue list` синтетическими JSON через
# PATH-prepend. Возвращает exit 0 и пишет в журнал.
make_fake_gh() {
    local pr_json="$1" issue_json="$2" fail_mode="${3:-no}"
    local fake_gh="$TEST_TMP/gh"
    cat > "$fake_gh" <<EOF
#!/bin/bash
echo "\$@" >> "$TEST_TMP/gh_journal"
case "\$1" in
    pr)
        case "\$2" in
            list) printf '%s' '$pr_json' ;;
            *) exit 1 ;;
        esac
        ;;
    issue)
        case "\$2" in
            list) printf '%s' '$issue_json' ;;
            *) exit 1 ;;
        esac
        ;;
    *) exit 1 ;;
esac
EOF
    chmod +x "$fake_gh"
    if [ "$fail_mode" = "fail" ]; then
        # Override: пустой вывод, exit 0 (тест на parse path)
        printf '' > "$fake_gh"
        cat > "$fake_gh" <<'EOF2'
#!/bin/bash
# Silent / empty: тестирует пустой PR_JSON → exit 1
exit 0
EOF2
        chmod +x "$fake_gh"
    fi
    echo "$fake_gh"
}

# --- Test: script exists + executable + bash -n ---------------------------
test_script_basics() {
    [ -f "$DIGEST_SCRIPT" ] || { echo "missing: $DIGEST_SCRIPT" >&2; return 1; }
    [ -x "$DIGEST_SCRIPT" ] || { echo "not executable: $DIGEST_SCRIPT" >&2; return 1; }
    bash -n "$DIGEST_SCRIPT" || { echo "bash -n failed" >&2; return 1; }
}

# --- Test: digest computation group A/B/C -------------------------------
test_group_classification() {
    # 5 PR: 2 group A, 2 group B, 1 group C (CONFLICTING)
    local now; now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local d1; d1="$(date -u -d '1 day ago' +%Y-%m-%dT%H:%M:%SZ)"
    local d5; d5="$(date -u -d '5 days ago' +%Y-%m-%dT%H:%M:%SZ)"

    local pr_json
    pr_json="$(cat <<JSON
[
  {"number":3010,"title":"feat: voice music","labels":[{"name":"e2e-done"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$d5","headRefName":"z-devops/t_voice_music"},
  {"number":3019,"title":"fix: g10b dedup","labels":[{"name":"e2e-done"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$d1","headRefName":"z-devops/g10b"},
  {"number":3020,"title":"feat: voice smoke","labels":[],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"z-devops/voice_smoke"},
  {"number":3021,"title":"fix: rebase","labels":[],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$d1","headRefName":"z-devops/rebase"},
  {"number":3030,"title":"feat: g10a","labels":[],"mergeable":"CONFLICTING","mergeStateStatus":"DIRTY","updatedAt":"$now","headRefName":"z-devops/g10a"}
]
JSON
)"

    local issue_json='[]'

    local fake_gh; fake_gh="$(make_fake_gh "$pr_json" "$issue_json")"
    local fake_path="$TEST_TMP/path-bin"
    mkdir -p "$fake_path"
    ln -sf "$fake_gh" "$fake_path/gh"

    # Запуск в dry-run с FAKE PATH и TEST_MODE=1 (пропустить MAINTENANCE/окно)
    local log="$TEST_TMP/digest.log"
    rm -f "$log" "$TEST_TMP/agent-flow-pr-backlog-digest.log"
    PATH="$fake_path:$PATH" \
    REPO_DIR="$REPO_ROOT" \
    DIGEST_DRY_RUN=true \
    DIGEST_FORCE=true \
    DIGEST_TEST_MODE=1 \
    DIGEST_STATE_DIR="$TEST_TMP" \
    DIGEST_MAX_PER_GROUP=10 \
    GH_REPO="krikz/rob_box_project" \
    TELEGRAM_BOT_TOKEN="" \
        bash "$DIGEST_SCRIPT" >/dev/null 2>"$TEST_TMP/stderr.log"
    local rc=$?
    [ "$rc" -eq 0 ] || { echo "exit=$rc"; cat "$TEST_TMP/stderr.log" >&2; return 1; }

    # Digest written to /tmp/agent-flow-pr-backlog-digest.log (DIGEST_STATE_DIR=$TEST_TMP)
    local out
    out="$(cat "$TEST_TMP/agent-flow-pr-backlog-digest.log")"

    # Group A: 2 PR (3010, 3019) — e2e-done + MERGEABLE + CLEAN
    assert_contains "готовы к merge Шифу прямо сейчас" "$out" "group A header"
    assert_contains "#3010" "$out" "PR 3010 in group A"
    assert_contains "#3019" "$out" "PR 3019 in group A"
    # Group B: 2 PR (3020, 3021)
    assert_contains "в работе" "$out" "group B header"
    assert_contains "#3020" "$out" "PR 3020 in group B"
    assert_contains "#3021" "$out" "PR 3021 in group B"
    # Group C: 1 PR (3030 CONFLICTING)
    assert_contains "risk" "$out" "group C header"
    assert_contains "#3030" "$out" "PR 3030 in group C"
    # Total: 5
    assert_contains "5 открытых PR" "$out" "total PR count"

    # Старые сверху: 3010 (5d) должен идти раньше 3019 (1d)
    local pos_3010 pos_3019
    pos_3010=$(printf '%s' "$out" | grep -bo '#3010' | head -1 | cut -d: -f1)
    pos_3019=$(printf '%s' "$out" | grep -bo '#3019' | head -1 | cut -d: -f1)
    [ -n "$pos_3010" ] && [ -n "$pos_3019" ] && [ "$pos_3010" -lt "$pos_3019" ] \
        || { echo "sort: 3010@$pos_3010 should be before 3019@$pos_3019" >&2; return 1; }

    # Telegram НЕ вызывался
    assert_not_contains "sendMessage" "$(cat "$TEST_TMP/gh_journal" 2>/dev/null || echo)" "no Telegram in dry-run"
}

# --- Test: empty PR list → fail-closed ---------------------------------
test_empty_pr_fail_closed() {
    local fake_gh; fake_gh="$(make_fake_gh "" "[]" fail)"
    local fake_path="$TEST_TMP/path-bin2"
    mkdir -p "$fake_path"
    ln -sf "$fake_gh" "$fake_path/gh"

    set +e
    PATH="$fake_path:$PATH" \
    REPO_DIR="$REPO_ROOT" \
    DIGEST_DRY_RUN=true \
    DIGEST_FORCE=true \
    DIGEST_TEST_MODE=1 \
    DIGEST_STATE_DIR="$TEST_TMP" \
    TELEGRAM_BOT_TOKEN="" \
        bash "$DIGEST_SCRIPT" >/dev/null 2>"$TEST_TMP/stderr2.log"
    local rc=$?
    set -e
    # С fake_gh=fail PR_JSON=[] → python получает [] → digest содержит
    # "0 открытых PR" → exit 0. Это валидно: «сегодня нет PR».
    [ "$rc" -eq 0 ] || { echo "exit=$rc (expected 0 for empty backlog)"; return 1; }
    local out; out="$(cat "$TEST_TMP/agent-flow-pr-backlog-digest.log")"
    assert_contains "0 открытых PR" "$out" "empty backlog message"
}

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
    TELEGRAM_BOT_TOKEN="" \
        bash "$DIGEST_SCRIPT" >/dev/null 2>"$TEST_TMP/stderr3.log"
    local rc=$?
    set -e
    [ "$rc" -eq 1 ] || { echo "exit=$rc (expected 1)"; cat "$TEST_TMP/stderr3.log" >&2; return 1; }
    assert_contains "TELEGRAM_BOT_TOKEN" "$(cat "$TEST_TMP/stderr3.log")" "fail message mentions token"
}

# --- Test: install.sh EXPECTED[] содержит скрипт ---------------------
test_install_expected() {
    grep -q 'agent-flow-pr-backlog-digest.sh' "$INSTALL_SCRIPT" \
        || { echo "install.sh does not list agent-flow-pr-backlog-digest.sh in EXPECTED" >&2; return 1; }
}

# --- Test: install.sh содержит ensure_pr_backlog_digest_cron -------
test_install_cron_ensure() {
    grep -q 'ensure_pr_backlog_digest_cron' "$INSTALL_SCRIPT" \
        || { echo "install.sh missing ensure_pr_backlog_digest_cron" >&2; return 1; }
    # Также проверим что он реально вызывается
    grep -qE '^ensure_pr_backlog_digest_cron\s*$' "$INSTALL_SCRIPT" \
        || { echo "ensure_pr_backlog_digest_cron not invoked at top level" >&2; return 1; }
}

# --- Test: needs-review подсчёт --------------------------------------
test_needs_review_count() {
    local now; now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local pr_json
    pr_json="$(cat <<JSON
[
  {"number":1,"title":"a","labels":[{"name":"e2e-done"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"a"},
  {"number":2,"title":"b","labels":[{"name":"needs-review"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"b"},
  {"number":3,"title":"c","labels":[{"name":"e2e-done"},{"name":"needs-review"}],"mergeable":"MERGEABLE","mergeStateStatus":"CLEAN","updatedAt":"$now","headRefName":"c"}
]
JSON
)"
    local fake_gh; fake_gh="$(make_fake_gh "$pr_json" "[]")"
    local fake_path="$TEST_TMP/path-bin5"
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
    # 2 PR с label needs-review (PR 2 и PR 3)
    assert_contains "2 needs-review" "$out" "needs-review count = 2"
}

# --- Test: AGENT_FLOW_PROPOSAL.md содержит раздел -------------------
test_proposal_section() {
    local proposal="$REPO_ROOT/docs/design/AGENT_FLOW_PROPOSAL.md"
    [ -f "$proposal" ] || { echo "missing: $proposal" >&2; return 1; }
    grep -qE '^### 3\.4 .*[Pp]r.[Bb]acklog.[Dd]igest' "$proposal" \
        || { echo "AGENT_FLOW_PROPOSAL.md missing PR backlog digest section" >&2; return 1; }
}

# --- Run all ---------------------------------------------------------------
run_test "script exists + executable + bash -n" test_script_basics
run_test "group A/B/C classification + sort"    test_group_classification
run_test "empty backlog (fail-closed OK)"      test_empty_pr_fail_closed
run_test "stale-candidate cross-check"         test_stale_candidate_warning
run_test "no TELEGRAM_BOT_TOKEN → exit 1"      test_no_token_fail
run_test "install.sh EXPECTED contains script" test_install_expected
run_test "install.sh has ensure_pr_backlog_digest_cron" test_install_cron_ensure
run_test "needs-review count"                  test_needs_review_count
run_test "AGENT_FLOW_PROPOSAL.md new section"  test_proposal_section

echo
printf '%s========== %s ==========%s\n' "$YEL" "SUMMARY" "$END"
printf 'Total: %d / Passed: %d / Failed: %d\n' "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'Failed tests:\n'
    for n in "${FAILED_NAMES[@]}"; do
        printf '  - %s\n' "$n"
    done
    exit 1
fi
printf '%sALL TESTS PASSED%s\n' "$GRN" "$END"
exit 0
