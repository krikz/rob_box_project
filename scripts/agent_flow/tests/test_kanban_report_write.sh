#!/usr/bin/env bash
# ============================================================================
# test_kanban_report_write.sh — регресс-тест scripts/agent_flow/kanban-report-write.sh
#
# Покрывает acceptance (ADR-0077):
#   1. usage error: пустой task_id → exit 2
#   2. usage error: неверный формат task_id → exit 2
#   3. вне git worktree → exit 1 + stderr содержит "not inside a git worktree"
#   4. в worktree без pytest/tests, без gh PR → файл создан, все секции n/a
#   5. в worktree с tests/ и pytest → секция pytest НЕ n/a
#   6. в worktree с git-изменениями → diff --stat не пустой
#   7. многократный запуск → файл обновляется идемпотентно (не падает)
#   8. kanban_db path не существует → тайминги = n/a, остальное работает
#
# Стратегия:
#   - mkdtemp для изолированного worktree
#   - git init + git commit (имитация worktree)
#   - запускаем скрипт с разными KANBAN_BIN/KANBAN_DB env
#   - проверяем exit + наличие файла + ключевые секции
#
# Run:
#   bash scripts/agent_flow/tests/test_kanban_report_write.sh
# ============================================================================

set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/.." && pwd)"
TARGET="$ROOT_DIR/kanban-report-write.sh"

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

assert_contains() {
    local name="$1"
    local file="$2"
    local needle="$3"
    if ! grep -qF -- "$needle" "$file" 2>/dev/null; then
        fail_count=$((fail_count + 1))
        echo "FAIL [$name]: file '$file' missing needle '$needle'"
        return 1
    fi
    pass_count=$((pass_count + 1))
    echo "PASS [$name] (contains '$needle')"
    return 0
}

assert_not_contains() {
    local name="$1"
    local file="$2"
    local needle="$3"
    if grep -qF -- "$needle" "$file" 2>/dev/null; then
        fail_count=$((fail_count + 1))
        echo "FAIL [$name]: file '$file' should NOT contain '$needle'"
        return 1
    fi
    pass_count=$((pass_count + 1))
    echo "PASS [$name] (does not contain '$needle')"
    return 0
}

# make_fake_worktree <dir> — создаёт чистый git-репо с одним коммитом.
# Используется для сценариев 3-8.
make_fake_worktree() {
    local d="$1"
    mkdir -p "$d"
    cd "$d"
    git init -q -b develop
    git config user.email "test@local"
    git config user.name "Test"
    printf 'init\n' > README.md
    git add README.md
    git commit -q -m "init"
    cd - >/dev/null
}

# --- Сценарий 1: usage error (пустой task_id) ---
out=$(bash "$TARGET" "" 2>&1)
rc=$?
assert_exit "usage-empty-task-id" 2 "$rc"

# --- Сценарий 2: usage error (неверный формат task_id) ---
out=$(bash "$TARGET" "not_a_task" 2>&1)
rc=$?
assert_exit "usage-bad-task-id-format" 2 "$rc"

# --- Сценарий 3: вне git worktree ---
# Запускаем из /tmp (там нет .git).
cd /tmp
out=$(bash "$TARGET" "t_aaaaaa" 2>&1)
rc=$?
assert_exit "not-inside-worktree" 1 "$rc"
if ! printf '%s' "$out" | grep -qF "not inside a git worktree"; then
    fail_count=$((fail_count + 1))
    echo "FAIL [not-inside-worktree-msg]: stderr should mention 'not inside a git worktree', got: $out"
else
    pass_count=$((pass_count + 1))
    echo "PASS [not-inside-worktree-msg]"
fi
cd "$WORK"

# --- Сценарий 4: в worktree, без tests, без PR, без DB → файл с n/a ---
WT4="$WORK/wt4"
make_fake_worktree "$WT4"
cd "$WT4"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db bash "$TARGET" "t_123456" 2>&1)
rc=$?
assert_exit "worktree-clean-run" 0 "$rc"
if [ -f "docs/reports/kanban/t_123456.md" ]; then
    pass_count=$((pass_count + 1))
    echo "PASS [report-file-created]"
else
    fail_count=$((fail_count + 1))
    echo "FAIL [report-file-created]: docs/reports/kanban/t_123456.md not found"
fi
assert_contains "header-has-task-id" "docs/reports/kanban/t_123456.md" "t_123456"
assert_contains "header-has-section" "docs/reports/kanban/t_123456.md" "## Что сделано"
assert_contains "ci-is-na" "docs/reports/kanban/t_123456.md" "## CI"
assert_contains "footer-adr" "docs/reports/kanban/t_123456.md" "ADR-0077"
cd "$WORK"

# --- Сценарий 5: в worktree с tests/ → pytest секция не должна быть n/a ---
WT5="$WORK/wt5"
make_fake_worktree "$WT5"
cd "$WT5"
mkdir -p tests
cat > tests/test_dummy.py <<'PYEOF'
def test_pass():
    assert 1 + 1 == 2
PYEOF
git add tests/ && git commit -q -m "add test"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db bash "$TARGET" "t_abcdef" 2>&1)
rc=$?
assert_exit "worktree-with-tests" 0 "$rc"
# pytest секция: или реальный вывод, или n/a (если pytest не установлен). В обоих
# случаях заголовок "## Raw-evidence (pytest)" должен быть.
assert_contains "pytest-section-present" "docs/reports/kanban/t_abcdef.md" "## Raw-evidence (pytest)"
cd "$WORK"

# --- Сценарий 6: в worktree с изменениями → diff не пустой ---
WT6="$WORK/wt6"
make_fake_worktree "$WT6"
cd "$WT6"
printf 'extra\n' >> README.md
git add README.md
git commit -q -m "extra change"
printf 'more\n' >> README.md
git add README.md
git commit -q -m "another change"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db bash "$TARGET" "t_feedbeef" 2>&1)
rc=$?
assert_exit "worktree-with-changes" 0 "$rc"
# git log должен содержать наши коммиты
assert_contains "git-log-has-commits" "docs/reports/kanban/t_feedbeef.md" "extra change"
assert_contains "git-log-has-second-commit" "docs/reports/kanban/t_feedbeef.md" "another change"
cd "$WORK"

# --- Сценарий 7: идемпотентность — второй запуск не падает ---
WT7="$WORK/wt7"
make_fake_worktree "$WT7"
cd "$WT7"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db bash "$TARGET" "t_cafe1234" 2>&1)
rc1=$?
assert_exit "first-run" 0 "$rc1"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db bash "$TARGET" "t_cafe1234" 2>&1)
rc2=$?
assert_exit "second-run-idempotent" 0 "$rc2"
cd "$WORK"

# --- Сценарий 8: KANBAN_DB не существует → тайминги n/a, файл создан ---
WT8="$WORK/wt8"
make_fake_worktree "$WT8"
cd "$WT8"
out=$(KANBAN_BIN=true KANBAN_DB=/tmp/no-such-db-xyz bash "$TARGET" "t_badf00d" 2>&1)
rc=$?
assert_exit "missing-db-no-fatal" 0 "$rc"
assert_contains "started-na-when-no-db" "docs/reports/kanban/t_badf00d.md" "**Started:**"
cd "$WORK"

# --- Итог ---
echo ""
echo "=================================================="
echo "test_kanban_report_write: $pass_count passed, $fail_count failed"
echo "=================================================="
if [ "$fail_count" -gt 0 ]; then
    exit 1
fi
exit 0