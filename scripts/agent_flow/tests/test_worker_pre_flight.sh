#!/usr/bin/env bash
# ============================================================================
# test_worker_pre_flight.sh — регресс-тест scripts/agent_flow/worker_pre_flight.sh
#
# Покрывает acceptance (issue #2438):
#   1. usage error: пустой task_id → exit 2
#   2. usage error: пустой branch → exit 2
#   3. usage error: неверный формат task_id → exit 2
#   4. вне git worktree → exit 2
#   5. behind=0 → exit 0 (no-op)
#   6. behind=5 → exit 0 (no auto-rebase, drift ≤ MAX_BRANCH_BEHIND)
#   7. behind=66 → exit 0 (auto-rebase успешен)
#   8. ahead of develop (т.е. negative behind) → exit 0 (no-op)
#
# Стратегия:
#   - mkdtemp для изолированного bare+clone репо.
#   - init bare repo, clone, push develop с N коммитами.
#   - в worktree: behind=0/5/66 — через разное количество коммитов в origin/develop.
#   - SKIP_PRE_FLIGHT=true для сценариев где не нужны комментарии в GH.
#
# Run:
#   bash scripts/agent_flow/tests/test_worker_pre_flight.sh
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/worker_pre_flight.sh"

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

# make_bare_with_commits <bare_dir> <work_dir> <n_commits>
# Создаёт bare repo, клонирует в work_dir, делает n_commits коммитов на develop.
# Возвращает 0.
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

# make_branch_off <work_dir> <base_sha> <branch_name>
# Создаёт ветку branch_name от base_sha.
make_branch_off() {
    local work="$1"
    local base="$2"
    local branch="$3"
    (
        cd "$work"
        git checkout -q -b "$branch" "$base"
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
# Используем чистый tmp-каталог БЕЗ .git — чтобы rev-parse --is-inside-work-tree
# вернул ошибку. (Тест сам по себе запускается из репо, поэтому bare=False не работает.)
NOWHERE="$(mktemp -d)"
(
    cd "$NOWHERE"
    out=$(bash "$TARGET" "t_abc1234" "z-test/branch" 2>&1)
    rc=$?
    assert_exit "4_not_in_worktree" 2 "$rc" || true
)
rm -rf "$NOWHERE"

# --- Сценарий 4a: branch mismatch → exit 2 (issue #2478) ---
# Caller передаёт branch != HEAD → fail-fast. Проверяем в существующем worktree.
out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "z-definitely-not-current-branch" 2>&1)
rc=$?
assert_exit "4a_branch_mismatch" 2 "$rc" || true
echo "$out" | grep -qF "branch mismatch" || { fail_count=$((fail_count + 1)); echo "FAIL [4a_branch_mismatch]: missing 'branch mismatch' in stderr"; }

# --- Сценарий 4b: branch совпадает с HEAD → exit 0 (no-op) ---
# В worktree (текущий) передаём актуальную ветку — никакой drift → exit 0.
_CENT_HEAD="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "$_CENT_HEAD" 2>&1)
rc=$?
assert_exit "4b_branch_match" 0 "$rc" || true

# --- Сценарий 4c: custom BASE_REF (issue #2478) ---
# Раньше BASE_REF был dead variable. Теперь — реально используется.
# В текущем worktree: BASE_REF=HEAD (тривиальный rebase) → exit 0.
_HEAD_SHA="$(git rev-parse HEAD 2>/dev/null)"
out=$(SKIP_PRE_FLIGHT=false BASE_REF="$_HEAD_SHA" bash "$TARGET" "t_abc1234" "$_CENT_HEAD" 2>&1)
rc=$?
assert_exit "4c_custom_base_ref" 0 "$rc" || true
unset _HEAD_SHA _CENT_HEAD

# --- Сценарий 5: behind=0 → exit 0 ---
SETUP5="$WORK/s5"
make_bare_with_commits "$WORK/s5_bare" "$SETUP5" 0
make_branch_off "$SETUP5" "HEAD" "z-test/s5-no-drift"
(
    cd "$SETUP5"
    out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s5-no-drift" 2>&1)
    rc=$?
    assert_exit "5_behind_0" 0 "$rc" || true
)

# --- Сценарий 6: behind=5 → exit 0 (no rebase, drift ≤ 30) ---
SETUP6="$WORK/s6"
make_bare_with_commits "$WORK/s6_bare" "$SETUP6" 0
# Сначала делаем ветку из HEAD (старого), потом на origin/develop добавляем 5 коммитов.
(
    cd "$SETUP6"
    git checkout -q -b "z-test/s6-small-drift" HEAD
    git fetch origin develop --prune >/dev/null 2>&1
    # Теперь добавляем 5 коммитов в develop
    for i in 1 2 3 4 5; do
        printf 'extra %d\n' "$i" > "extra_$i.txt"
        git add "extra_$i.txt"
        git commit -q -m "extra $i"
    done
    git push -q origin develop
    # Обновляем origin/develop в нашем клоне
    git fetch origin develop --prune >/dev/null 2>&1
    git checkout -q "z-test/s6-small-drift"
    out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s6-small-drift" 2>&1)
    rc=$?
    assert_exit "6_behind_5" 0 "$rc" || true
)

# --- Сценарий 7: behind=66 → exit 0 (auto-rebase успешен) ---
SETUP7="$WORK/s7"
make_bare_with_commits "$WORK/s7_bare" "$SETUP7" 0
(
    cd "$SETUP7"
    git checkout -q -b "z-test/s7-big-drift" HEAD
    git fetch origin develop --prune >/dev/null 2>&1
    for i in $(seq 1 66); do
        printf 'big %d\n' "$i" > "big_$i.txt"
        git add "big_$i.txt"
        git commit -q -m "big $i"
    done
    git push -q origin develop
    git fetch origin develop --prune >/dev/null 2>&1
    git checkout -q "z-test/s7-big-drift"
    out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s7-big-drift" 2>&1)
    rc=$?
    assert_exit "7_behind_66" 0 "$rc" || true
    # Проверяем что behind реально уменьшился
    behind=$(git rev-list --count "z-test/s7-big-drift..origin/develop" 2>/dev/null || echo "?")
    if [ "$behind" = "0" ]; then
        pass_count=$((pass_count + 1))
        echo "PASS [7_behind_66_after_rebase] (behind=0 after rebase)"
    else
        fail_count=$((fail_count + 1))
        echo "FAIL [7_behind_66_after_rebase]: expected behind=0 after rebase, got $behind"
    fi
)

# --- Сценарий 8: ahead of develop (т.е. negative behind=0) → exit 0 ---
SETUP8="$WORK/s8"
make_bare_with_commits "$WORK/s8_bare" "$SETUP8" 0
(
    cd "$SETUP8"
    git checkout -q -b "z-test/s8-ahead" HEAD
    # Делаем свои коммиты впереди develop
    for i in 1 2 3; do
        printf 'ahead %d\n' "$i" > "ahead_$i.txt"
        git add "ahead_$i.txt"
        git commit -q -m "ahead $i"
    done
    git push -q -u origin "z-test/s8-ahead" || true
    git fetch origin develop --prune >/dev/null 2>&1
    out=$(SKIP_PRE_FLIGHT=false bash "$TARGET" "t_abc1234" "z-test/s8-ahead" 2>&1)
    rc=$?
    assert_exit "8_ahead_of_develop" 0 "$rc" || true
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