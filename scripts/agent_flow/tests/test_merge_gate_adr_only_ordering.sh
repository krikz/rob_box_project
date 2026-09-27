#!/usr/bin/env bash
# ============================================================================
# test_merge_gate_adr_only_ordering.sh — ретро 16.09 t_d13a5c65
#
# Регресс-тест для guard'а check_adr_only_ordering() (G11) в
# agent-flow-merge-gate.sh. ADR-only PR от архитектора не должен мержиться
# раньше открытой impl-PR по той же issue — иначе ADR фиксирует design для
# метода, который ещё не написан.
#
# Тест покрывает 9 кейсов acceptance:
#   A. ADR-only PR (явная метка) + impl-PR OPEN MERGEABLE на issue → REJECT.
#   B. ADR-only PR + impl-PR нет на issue → PASS.
#   C. ADR-only PR + impl-PR CLOSED (не OPEN) → PASS.
#   D. ADR-only PR + impl-PR OPEN, но CONFLICTING → PASS (race impossible).
#   E. Override: метка `retro` на issue → PASS.
#   F. Mini-override: PR имеет `no-e2e-required` → PASS.
#   G. Fallback detection: PR меняет ТОЛЬКО docs/adr/* + agent:architect →
#      считается ADR-only (без явной метки `adr-only`).
#   H. PR меняет docs/adr + src/ → НЕ ADR-only (не блокируем).
#   I. Fail-open: gh pr list пустой → return 0.
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_adr_only_ordering.sh
# ============================================================================
set -uo pipefail

TEST_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_LIBS_DIR="$(cd "$TEST_FILE_DIR/lib" && pwd)"

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

# shellcheck source=lib/mock_env.sh
. "$TEST_LIBS_DIR/mock_env.sh"

# Source guard + зависимости (та же стратегия, что в test_merge_gate_adr_collision.sh).
# shellcheck source=lib/lib_eval_func.sh
. "$TEST_LIBS_DIR/lib_eval_func.sh"

MG_LOG="$(extract_func "$REPO_ROOT/agent-flow-merge-gate.sh" "log")"
MG_HAS_LABEL="$(extract_func_or_die "$REPO_ROOT/lib_agent_flow_common.sh" "has_label")"
MG_GUARD="$(extract_func "$REPO_ROOT/agent-flow-merge-gate.sh" "check_adr_only_ordering")"

unset -f log has_label check_adr_only_ordering 2>/dev/null || true
eval "$MG_LOG" >/dev/null
eval "$MG_HAS_LABEL" >/dev/null
eval "$MG_GUARD" >/dev/null

# Глобалы, на которые ссылается G11 guard.
LOG_PREFIX="[merge-gate]"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="${DRY_RUN:-false}"
NEEDS_E2E_LABEL="needs-e2e"
NEEDS_REVIEW_LABEL="needs-review"
DONE_LABEL="e2e-done"
NO_E2E_LABEL="no-e2e-required"
ISSUE_LABEL="hermes"
DEVELOP_BRANCH="develop"
ADR_ONLY_LABEL="adr-only"
ADR_ONLY_BLOCKING_AGENT_LABELS="agent:backend,agent:devops,agent:frontend,agent:developer,agent:llm-expert"
ADR_ONLY_PATH_PATTERN="^docs/adr/"
ADR_ONLY_ORDERING_OVERRIDE_LABEL="retro"
ADR_ONLY_ORDERING_BLOCKED_LABEL="agent-flow:adr-ordering-blocked"
ADR_ONLY_ORDERING_DEDUP_HOURS="24"

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------
install_mocks_for_test() {
    TEST_TMP="$(mktemp -d /tmp/agent-flow-adr-ordering-tests.XXXXXX)"
    install_mocks
    : >"$TEST_TMP/stderr.log"
    export GH_STATE GH_JOURNAL
}

# Issue labels + state (для override-метки retro и impl-PR blocking).
set_issue_state() {
    local issue="$1" labels="$2" comments="$3" labels_json='{"labels":[' first=1 l
    if [ -n "$labels" ]; then
        IFS=',' read -ra LARR <<< "$labels"
        for l in "${LARR[@]}"; do
            [ -z "$l" ] && continue
            [ "$first" -eq 0 ] && labels_json+=','
            labels_json+="{\"name\":\"$l\"}"
            first=0
        done
    fi
    labels_json+=']}'
    set_state "ISSUE_${issue}_LABELS_JSON" "$labels_json"
    set_state "ISSUE_${issue}_STATE_JSON" '{"state":"OPEN"}'
    set_state "ISSUE_${issue}_COMMENTS_SINCE_JSON" "$comments"
}

# PR labels + state (включая mergeStateStatus и agent:* метки).
set_pr_state() {
    local pr="$1" labels="$2" merge_state="$3"
    local labels_json='[' first=1 l
    if [ -n "$labels" ]; then
        IFS=',' read -ra LARR <<< "$labels"
        for l in "${LARR[@]}"; do
            [ -z "$l" ] && continue
            [ "$first" -eq 0 ] && labels_json+=','
            labels_json+="{\"name\":\"$l\"}"
            first=0
        done
    fi
    labels_json+=']'
    set_state "PR_${pr}_LABELS_JSON" "{\"labels\":$labels_json}"
    set_state "PR_${pr}_MERGE_STATE_JSON" "{\"mergeStateStatus\":\"$merge_state\",\"state\":\"OPEN\"}"
}

# PR files (через PR_N_FILES_JSON — используется в gh pr view --json files).
set_pr_files() {
    local pr="$1" input="$2" built
    if [ "${input:0:1}" = "[" ]; then
        built="$(printf '%s' "$input" | python3 -c '
import json,sys
arr=json.load(sys.stdin)
print(json.dumps([{"path":p} for p in arr]))
')"
    else
        built="$(printf '%s\n' "$input" | python3 -c '
import json,sys
lines=[l.strip() for l in sys.stdin if l.strip()]
print(json.dumps([{"path":p} for p in lines]))
')"
    fi
    set_state "PR_${pr}_FILES_JSON" "{\"files\":$built}"
}

# Список OPEN PR (для gh pr list --search "<n> in:title").
# $1=issue $2=JSON массив PR (number, files, labels, mergeStateStatus, state).
# NOTE: mock_env.sh использует общий ключ PR_FOLLOWUP_JSON для всех --search
# запросов (включая followup-PR, retro-path и наш G11). Это shared fixture —
# тесты должны изолировать себя, не перетирая чужие ожидания.
set_open_prs_list() {
    local issue="$1" json="$2"
    set_state "PR_FOLLOWUP_JSON" "$json"
}

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {
    local want="$1" got="$2" msg="$3"
    if [ "$want" != "$got" ]; then
        printf '    %sassert fail:%s want=%q got=%q (%s)\n' "$RED" "$END" "$want" "$got" "$msg" >&2
        return 1
    fi
    return 0
}

assert_ge() {
    local actual="$1" want="$2" msg="$3"
    if [ "${actual:-0}" -lt "${want}" ] 2>/dev/null; then
        printf '    %sassert fail:%s want>=%s got=%s (%s)\n' "$RED" "$END" "$want" "$actual" "$msg" >&2
        return 1
    fi
    return 0
}

# ---------------------------------------------------------------------------
# A. ADR-only PR (явная метка) + impl-PR OPEN MERGEABLE → REJECT.
#    Прямой сценарий #2627: PR #2647 (архитектор, adr-only), PR #2640 (impl,
#    agent:backend на issue).
# ---------------------------------------------------------------------------
test_A_explicit_adr_only_label_blocks() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only,wip" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/0021a-post-turn-music-finalizer.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    # PR #2640 — impl, OPEN MERGEABLE, NO agent:* (метка на issue)
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-refactor","labels":[],"mergeStateStatus":"MERGEABLE","state":"OPEN","title":"refactor(voice #2627): DialogueNode._run_turn CC 59→33"}]'

    local rc=0
    (check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only,wip" \
        '["docs/adr/0021a-post-turn-music-finalizer.md"]' "OPEN" "MERGEABLE") 2>"$TEST_TMP/stderr.log" || rc=$?

    assert_eq "1" "$rc" "guard returns 1 (impl-PR OPEN blocks ADR-only)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 2627' "$GH_JOURNAL" || true)"
    assert_ge "$n_comments" "1" "block comment posted on issue"
    local n_labels
    n_labels="$(grep -c 'gh issue edit 2627 --add-label agent-flow:adr-ordering-blocked' "$GH_JOURNAL" || true)"
    assert_eq "1" "$n_labels" "block label added on issue"
    local log_has_blocking
    log_has_blocking="$(grep -c 'БЛОКИРОВКА' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_blocking" "1" "log says БЛОКИРОВКА"
    local log_has_pr
    log_has_pr="$(grep -c '#2640' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_pr" "1" "log mentions blocking PR #2640"
}

# ---------------------------------------------------------------------------
# B. ADR-only PR + impl-PR нет на issue → PASS.
# ---------------------------------------------------------------------------
test_B_no_impl_pr_passes() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    # OPEN PR list пустой (нет impl-PR)
    set_open_prs_list 2627 '[]'

    local rc=0
    (check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE") 2>"$TEST_TMP/stderr.log" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (no impl-PR)"
    local n_comments
    n_comments="$(grep -c 'gh issue comment 2627' "$GH_JOURNAL" || true)"
    assert_eq "0" "$n_comments" "no comment posted"
    local log_has_ok
    log_has_ok="$(grep -c 'G11 OK' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_ok" "1" "log says G11 OK"
}

# ---------------------------------------------------------------------------
# C. ADR-only PR + impl-PR CLOSED (не OPEN) → PASS.
# ---------------------------------------------------------------------------
test_C_impl_pr_closed_passes() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    # PR #2640 уже CLOSED (state=CLOSED, не OPEN)
    set_open_prs_list 2627 '[]'

    local rc=0
    check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (impl-PR not OPEN)"
}

# ---------------------------------------------------------------------------
# D. ADR-only PR + impl-PR OPEN, но CONFLICTING → PASS (race impossible).
# ---------------------------------------------------------------------------
test_D_impl_pr_conflicting_passes() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    # PR #2640 OPEN, но CONFLICTING (merge-race физически невозможен)
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-x","labels":[],"mergeStateStatus":"CONFLICTING","state":"OPEN","title":"impl with conflict"}]'

    local rc=0
    check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (impl-PR CONFLICTING, race impossible)"
}

# ---------------------------------------------------------------------------
# E. Override: метка `retro` на issue → PASS.
# ---------------------------------------------------------------------------
test_E_retro_override_passes() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend,retro" "[]"
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-x","labels":[],"mergeStateStatus":"MERGEABLE","state":"OPEN","title":"impl"}]'

    local rc=0
    (check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE") 2>"$TEST_TMP/stderr.log" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (retro override)"
    local log_has_override
    log_has_override="$(grep -c 'ADR-ordering override' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_override" "1" "log says ADR-ordering override"
}

# ---------------------------------------------------------------------------
# F. Mini-override: PR имеет `no-e2e-required` → PASS.
# ---------------------------------------------------------------------------
test_F_no_e2e_required_passes() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only,no-e2e-required" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-x","labels":[],"mergeStateStatus":"MERGEABLE","state":"OPEN","title":"impl"}]'

    local rc=0
    (check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only,no-e2e-required" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE") 2>"$TEST_TMP/stderr.log" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (no-e2e-required mini-override)"
    local log_has_override
    log_has_override="$(grep -c 'явный e2e opt-out' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_override" "1" "log says явный e2e opt-out"
}

# ---------------------------------------------------------------------------
# G. Fallback: PR меняет ТОЛЬКО docs/adr/* + agent:architect (без явной
#    метки `adr-only`) → считается ADR-only → BLOCKS.
# ---------------------------------------------------------------------------
test_G_fallback_all_files_in_adr_blocks() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,wip" "MERGEABLE"  # NO adr-only label
    set_pr_files 2647 '["docs/adr/0021a-post-turn-music-finalizer.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-x","labels":[],"mergeStateStatus":"MERGEABLE","state":"OPEN","title":"impl"}]'

    local rc=0
    check_adr_only_ordering 2647 2627 "lint" "agent:architect,wip" \
        '["docs/adr/0021a-post-turn-music-finalizer.md"]' "OPEN" "MERGEABLE" || rc=$?

    assert_eq "1" "$rc" "guard returns 1 (fallback detection of ADR-only via file pattern)"
}

# ---------------------------------------------------------------------------
# H. PR меняет docs/adr + src/ → НЕ ADR-only (mixed files).
# ---------------------------------------------------------------------------
test_H_mixed_files_not_adr_only() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md", "src/voice/foo.py"]'
    set_issue_state 2627 "agent:backend" "[]"
    set_open_prs_list 2627 '[{"number":2640,"headRefName":"z-{agent}/2627-x","labels":[],"mergeStateStatus":"MERGEABLE","state":"OPEN","title":"impl"}]'

    local rc=0
    check_adr_only_ordering 2647 2627 "functional" "agent:architect" \
        '["docs/adr/foo.md","src/voice/foo.py"]' "OPEN" "MERGEABLE" || rc=$?

    assert_eq "0" "$rc" "guard returns 0 (mixed files → not adr-only)"
}

# ---------------------------------------------------------------------------
# I. Fail-open: gh pr list пустой (API flake) → return 0.
# ---------------------------------------------------------------------------
test_I_fail_open_on_empty_pr_list() {
    install_mocks_for_test
    set_pr_state 2647 "agent:architect,adr-only" "MERGEABLE"
    set_pr_files 2647 '["docs/adr/foo.md"]'
    set_issue_state 2627 "agent:backend" "[]"
    # Мок не знает про ISSUE_2627_PRS_OPEN_JSON → gh вернёт ошибку → guard fail-open
    unset GH_STATE 2>/dev/null || true
    export GH_STATE="$TEST_TMP/state"
    : >"$GH_STATE"

    local rc=0
    (check_adr_only_ordering 2647 2627 "lint" "agent:architect,adr-only" \
        '["docs/adr/foo.md"]' "OPEN" "MERGEABLE") 2>"$TEST_TMP/stderr.log" || rc=$?

    assert_eq "0" "$rc" "guard fails open (gh pr list empty)"
    local log_has_failopen
    log_has_failopen="$(grep -c 'fail-open' "$TEST_TMP/stderr.log" || true)"
    assert_ge "$log_has_failopen" "1" "log says fail-open"
}

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
run_test "A_explicit_adr_only_label_blocks"          test_A_explicit_adr_only_label_blocks
run_test "B_no_impl_pr_passes"                        test_B_no_impl_pr_passes
run_test "C_impl_pr_closed_passes"                    test_C_impl_pr_closed_passes
run_test "D_impl_pr_conflicting_passes"               test_D_impl_pr_conflicting_passes
run_test "E_retro_override_passes"                    test_E_retro_override_passes
run_test "F_no_e2e_required_passes"                   test_F_no_e2e_required_passes
run_test "G_fallback_all_files_in_adr_blocks"         test_G_fallback_all_files_in_adr_blocks
run_test "H_mixed_files_not_adr_only"                 test_H_mixed_files_not_adr_only
run_test "I_fail_open_on_empty_pr_list"               test_I_fail_open_on_empty_pr_list

printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
printf 'total:  %d\n' "$TESTS_TOTAL"
printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
    printf 'failures:\n'
    for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
    exit 1
fi
exit 0
