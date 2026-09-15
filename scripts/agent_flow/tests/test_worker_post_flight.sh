#!/usr/bin/env bash
# ============================================================================
# test_worker_post_flight.sh — регресс-тест scripts/agent_flow/worker_post_flight.sh
#
# Покрывает acceptance (issue #2438):
#   1. usage error: пустой task_id → exit 2
#   2. usage error: пустой branch → exit 2
#   3. usage error: неверный формат task_id → exit 2
#   4. вне git worktree → exit 2
#   5. ahead of develop → exit 0 (no-op, behind=0)
#   6. behind=2 → auto-rebase exit 0
#   7. SKIP_POST_FLIGHT=true → exit 0 без fetch
#   8. worktree в rebase-merge state → exit 1 (refuse to operate)
#
# Стратегия:
#   - mkdtemp bare+clone.
#   - В каждом сценарии — known number of commits в origin/develop и known branch.
#
# Run:
#   bash scripts/agent_flow/tests/test_worker_post_flight.sh
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/worker_post_flight.sh"

if [ ! -x "$TARGET" ]; then
    echo "FAIL: $TARGET not executable" >&2
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass_count=0
fail_count=0

assert_exit() {
    local name="$1"
    local expected="$2"
    local actual="$3"
    if [ "$actual" -ne "$expected" ]; then
        fail_count=$((fail_count + 1))
        echo "FAIL [$name]: expected exit $expected, got $actual"
        return 1
    fi
    pass_count=$((pass_count + 1))
    echo "PASS [$name] (exit=$actual)"
    return 0
}

make_bare_with_commits() {
    local bare="$1"
    local work="$2"
    local n="$3"
    git init -q --bare "$bare"
    git clone -q "$bare" "$work"
    (
        cd "$work"
        git config user.email "test@local"
        git config user.name "Test"
        git checkout -q -b develop
        printf 'init\n' > README.md
        git add README.md
        git commit -q -m "init"
        for i in $(seq 1 "$n"); do
            printf 'commit %d\n' "$i" > "file_$i.txt"
            git add "file_$i.txt"
            git commit -q -m "commit $i"
        done
        git push -q origin develop
    )
}

# --- Сценарий 1: usage error (пустой task_id) ---
out=$(bash "$TARGET" "" "z-test/branch" 2>&1)
rc=$?
assert_exit "1_usage_no_task_id" 2 "$rc" || true
echo "$out" | grep -qF "usage:" || { fail_count=$((fail_count + 1)); echo "FAIL [1_usage_no_task_id]: missing 'usage:' in stderr"; }

# --- Сценарий 2: usage error (пустой branch) ---
out=$(bash "$TARGET" "t_abc1234" "" 2>&1)
rc=$?
assert_exit "2_usage_no_branch" 2 "$rc" || true
echo "$out" | grep -qF "branch required" || { fail_count=$((fail_count + 1)); echo "FAIL [2_usage_no_branch]: missing 'branch required' in stderr"; }

# --- Сценарий 3: usage error (неверный формат task_id) ---
out=$(bash "$TARGET" "invalid" "z-test/branch" 2>&1)
rc=$?
assert_exit "3_usage_bad_task_id" 2 "$rc" || true

# --- Сценарий 4: вне git worktree ---
NOWHERE="$(mktemp -d)"
(
    cd "$NOWHERE"
    out=$(bash "$TARGET" "t_abc1234" "z-test/branch" 2>&1)
    rc=$?
    assert_exit "4_not_in_worktree" 2 "$rc" || true
)
rm -rf "$NOWHERE"

# --- Сценарий 4a: branch mismatch → exit 2 (issue #2478) ---
out=$(SKIP_POST_FLIGHT=false bash "$TARGET" "t_abc1234" "z-definitely-not-current-branch" 2>&1)
rc=$?
assert_exit "4a_branch_mismatch" 2 "$rc" || true
echo "$out" | grep -qF "branch mismatch" || { fail_count=$((fail_count + 1)); echo "FAIL [4a_branch_mismatch]: missing 'branch mismatch' in stderr"; }

# --- Сценарий 4b: branch совпадает с HEAD → exit 0 ---
_CENT_HEAD="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
out=$(SKIP_POST_FLIGHT=false bash "$TARGET" "t_abc1234" "$_CENT_HEAD" 2>&1)
rc=$?
assert_exit "4b_branch_match" 0 "$rc" || true

# --- Сценарий 4c: custom BASE_REF (issue #2478) ---
# BASE_REF=HEAD → trivial no-op → exit 0.
_HEAD_SHA="$(git rev-parse HEAD 2>/dev/null)"
out=$(SKIP_POST_FLIGHT=false BASE_REF="$_HEAD_SHA" bash "$TARGET" "t_abc1234" "$_CENT_HEAD" 2>&1)
rc=$?
assert_exit "4c_custom_base_ref" 0 "$rc" || true
unset _HEAD_SHA _CENT_HEAD

# --- Сценарий 5: ahead of develop → exit 0 ---
SETUP5="$WORK/s5"
make_bare_with_commits "$WORK/s5_bare" "$SETUP5" 0
(
    cd "$SETUP5"
    git checkout -q -b "z-test/s5-ahead" HEAD
    for i in 1 2 3; do
        printf 'ahead %d\n' "$i" > "ahead_$i.txt"
        git add "ahead_$i.txt"
        git commit -q -m "ahead $i"
    done
    out=$(SKIP_POST_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s5-ahead" 2>&1)
    rc=$?
    assert_exit "5_ahead_of_develop" 0 "$rc" || true
)

# --- Сценарий 6: behind=2 → auto-rebase exit 0 ---
SETUP6="$WORK/s6"
make_bare_with_commits "$WORK/s6_bare" "$SETUP6" 0
(
    cd "$SETUP6"
    git checkout -q -b "z-test/s6-small-drift" HEAD
    git fetch origin develop --prune >/dev/null 2>&1
    for i in 1 2; do
        printf 'drift %d\n' "$i" > "drift_$i.txt"
        git add "drift_$i.txt"
        git commit -q -m "drift $i"
    done
    git push -q origin develop
    git fetch origin develop --prune >/dev/null 2>&1
    git checkout -q "z-test/s6-small-drift"
    out=$(SKIP_POST_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s6-small-drift" 2>&1)
    rc=$?
    assert_exit "6_behind_2" 0 "$rc" || true
    # Проверяем что behind уменьшился
    behind=$(git rev-list --count "z-test/s6-small-drift..origin/develop" 2>/dev/null || echo "?")
    if [ "$behind" = "0" ]; then
        pass_count=$((pass_count + 1))
        echo "PASS [6_behind_2_after_rebase] (behind=0 after rebase)"
    else
        fail_count=$((fail_count + 1))
        echo "FAIL [6_behind_2_after_rebase]: expected behind=0, got $behind"
    fi
)

# --- Сценарий 7: SKIP_POST_FLIGHT=true → exit 0 без fetch ---
SETUP7="$WORK/s7"
make_bare_with_commits "$WORK/s7_bare" "$SETUP7" 0
(
    cd "$SETUP7"
    git checkout -q -b "z-test/s7-skip" HEAD
    # С SKIP_POST_FLIGHT=true не должно быть fetch/rebase — exit 0 без условий.
    out=$(SKIP_POST_FLIGHT=true bash "$TARGET" "t_abc1234" "z-test/s7-skip" 2>&1)
    rc=$?
    assert_exit "7_skip_post_flight" 0 "$rc" || true
)

# --- Сценарий 8: worktree в rebase-merge state → exit 1 ---
SETUP8="$WORK/s8"
make_bare_with_commits "$WORK/s8_bare" "$SETUP8" 0
(
    cd "$SETUP8"
    git checkout -q -b "z-test/s8-in-rebase" HEAD
    # Имитируем rebase в процессе: создаём .git/rebase-merge
    git rev-parse --git-dir >/dev/null 2>&1
    git_dir="$(git rev-parse --git-dir)"
    mkdir -p "$git_dir/rebase-merge"
    printf 'rebase' > "$git_dir/rebase-merge/head-name"
    out=$(SKIP_POST_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s8-in-rebase" 2>&1)
    rc=$?
    assert_exit "8_rebase_in_progress" 1 "$rc" || true
)

# --- summary ---
echo ""
echo "==== SUMMARY ===="
echo "PASS: $pass_count"
echo "FAIL: $fail_count"
if [ "$fail_count" -gt 0 ]; then
    exit 1
fi
echo "ALL PASS"
exit 0