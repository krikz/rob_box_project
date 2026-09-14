#!/usr/bin/env bash
# ============================================================================
# test_worker_scope_check.sh — регресс-тест scripts/agent_flow/worker_scope_check.sh
#
# Покрывает acceptance (issue #2438 / PR #2443):
#   1. usage error: пустой task_id → exit 2
#   2. usage error: неверный формат task_id → exit 2
#   3. вне git worktree → exit 2
#   4. SKIP_SCOPE_CHECK=true → exit 0
#   5. чистая ветка, нет файлов → exit 0
#   6. INFO-режим (без prefixes): показывает untracked-файлы, exit 0
#   7. INFO-режим с > MAX_OUT_OF_SCOPE файлов → exit 1 (defensive)
#   8. все файлы в allowed prefix → exit 0
#   9. untracked-файл вне scope → exit 1
#  10. committed-файл вне scope (diff vs develop) → exit 1
#
# Run:
#   bash scripts/agent_flow/tests/test_worker_scope_check.sh
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/worker_scope_check.sh"

if [ ! -f "$TARGET" ]; then
    echo "FAIL: $TARGET not found" >&2
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

assert_output_has() {
    local name="$1"
    local out="$2"
    local needle="$3"
    if printf '%s\n' "$out" | grep -qF "$needle"; then
        pass_count=$((pass_count + 1))
        echo "PASS [$name] (contains '$needle')"
        return 0
    fi
    fail_count=$((fail_count + 1))
    echo "FAIL [$name]: output missing '$needle'"
    return 1
}

# make_bare <bare_dir> <work_dir> — bare repo + clone на ветке develop (1 init-коммит).
make_bare() {
    local bare="$1"
    local work="$2"
    git init -q --bare "$bare"
    git clone -q "$bare" "$work" 2>/dev/null
    (
        cd "$work"
        git config user.email "test@local"
        git config user.name "Test"
        git checkout -q -b develop
        printf 'init\n' > README.md
        git add README.md
        git commit -q -m "init"
        git push -q origin develop
        # origin/develop в клоне после push уже есть; для надёжности — явный fetch.
        git fetch -q origin develop
    )
}

# --- Сценарий 1: usage error (пустой task_id) ---
out=$(bash "$TARGET" "" 2>&1)
rc=$?
assert_exit "1_usage_no_task_id" 2 "$rc"
assert_output_has "1_usage_no_task_id_msg" "$out" "usage:"

# --- Сценарий 2: usage error (неверный формат task_id) ---
out=$(bash "$TARGET" "invalid" 2>&1)
rc=$?
assert_exit "2_usage_bad_task_id" 2 "$rc"

# --- Сценарий 3: вне git worktree ---
NOWHERE="$(mktemp -d)"
pushd "$NOWHERE" >/dev/null
out=$(bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
rm -rf "$NOWHERE"
assert_exit "3_not_in_worktree" 2 "$rc"

# --- Сценарий 4: SKIP_SCOPE_CHECK=true → exit 0 ---
out=$(SKIP_SCOPE_CHECK=true bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
assert_exit "4_skip_scope_check" 0 "$rc"

# --- Сценарий 5: чистая ветка, нет файлов → exit 0 ---
SETUP5="$WORK/s5"
make_bare "$WORK/s5_bare" "$SETUP5"
pushd "$SETUP5" >/dev/null
git checkout -q -b "z-test/s5-clean" develop
out=$(bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "5_no_files" 0 "$rc"

# --- Сценарий 6: INFO-режим (без prefixes), untracked-файл → exit 0 + показан ---
SETUP6="$WORK/s6"
make_bare "$WORK/s6_bare" "$SETUP6"
pushd "$SETUP6" >/dev/null
git checkout -q -b "z-test/s6-info" develop
printf 'x\n' > scripts_x.txt
out=$(bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "6_info_mode" 0 "$rc"
assert_output_has "6_info_mode_lists_file" "$out" "scripts_x.txt"

# --- Сценарий 7: INFO-режим, > MAX_OUT_OF_SCOPE файлов → exit 1 ---
SETUP7="$WORK/s7"
make_bare "$WORK/s7_bare" "$SETUP7"
pushd "$SETUP7" >/dev/null
git checkout -q -b "z-test/s7-many" develop
for i in $(seq 1 12); do
    printf 'x\n' > "junk_$i.txt"
done
out=$(MAX_OUT_OF_SCOPE=10 bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "7_info_too_many" 1 "$rc"

# --- Сценарий 8: все файлы в allowed prefix → exit 0 ---
SETUP8="$WORK/s8"
make_bare "$WORK/s8_bare" "$SETUP8"
pushd "$SETUP8" >/dev/null
git checkout -q -b "z-test/s8-ok" develop
mkdir -p scripts
printf 'x\n' > scripts/ok.sh
out=$(PR_ALLOWED_PREFIXES="scripts/" bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "8_all_in_scope" 0 "$rc"

# --- Сценарий 9: untracked-файл вне scope → exit 1 ---
SETUP9="$WORK/s9"
make_bare "$WORK/s9_bare" "$SETUP9"
pushd "$SETUP9" >/dev/null
git checkout -q -b "z-test/s9-untracked-ous" develop
printf 'x\n' > docs_ous.md
out=$(PR_ALLOWED_PREFIXES="scripts/" bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "9_untracked_out_of_scope" 1 "$rc"
assert_output_has "9_untracked_out_of_scope_names_file" "$out" "docs_ous.md"

# --- Сценарий 10: committed-файл вне scope (diff vs develop) → exit 1 ---
SETUP10="$WORK/s10"
make_bare "$WORK/s10_bare" "$SETUP10"
pushd "$SETUP10" >/dev/null
git checkout -q -b "z-test/s10-committed-ous" develop
printf 'x\n' > evil_webxr.ts
git add evil_webxr.ts
git commit -q -m "leftover from another task"
out=$(PR_ALLOWED_PREFIXES="scripts/" bash "$TARGET" "t_abc1234" 2>&1)
rc=$?
popd >/dev/null
assert_exit "10_committed_out_of_scope" 1 "$rc"
assert_output_has "10_committed_out_of_scope_names_file" "$out" "evil_webxr.ts"

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
