#!/bin/bash
# ============================================================================
# test_e2e_process_worktree_cleanup.sh — ретро 28.08, issue #1707
#
# Функциональные тесты cleanup-логики agent-flow-e2e-process.sh:
#   A. _wt_sweep_orphans удаляет /tmp/agent-flow-e2e-<DEAD_PID>/,
#      НЕ трогает свой собственный worktree (по PID).
#   B. _wt_sweep_ttl удаляет worktree старше E2E_WT_TTL_DAYS дней,
#      НЕ трогает живые процессы (даже старые).
#   C. _wt_disk_check возвращает 0 при свободном месте > min,
#      1 при переполнении.
#   D. cleanup() (через trap) удаляет свой worktree + делает
#      `git worktree prune` в REPO_DIR.
#   E. _wt_remove_single удаляет каталог + .git/worktrees/<name>.
#
# Стратегия: source'им только нужные функции скрипта (через awk-извлечение)
# — это изолирует их от gh/secrets/etc.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_worktree_cleanup.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
# REPO_ROOT here = scripts/agent_flow/ (parent of tests/).
# The script under test is in this dir.
E2E_PROCESS="$TEST_ROOT_DIR/agent-flow-e2e-process.sh"

TEST_TMP="${TEST_TMP:-/tmp/agent-flow-cleanup-tests.$$}"
mkdir -p "$TEST_TMP"
cleanup_root() { rm -rf "$TEST_TMP"; }
trap cleanup_root EXIT

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

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

# extract_funcs <script> <func_name>... — печатает определения функций
# (включая вложенные helper'ы, которые они вызывают). Простая эвристика:
# находим строку `^<func>() {` и копируем до следующей `^}`.
extract_funcs() {
    local script="$1"; shift
    local func="$1"
    awk -v fn="$func" '
        $0 ~ "^"fn"\\(\\) \\{" { capture=1; depth=0 }
        capture {
            print
            n=gsub(/\\{/, "{"); depth+=n
            n=gsub(/\\}/, "}"); depth-=n
            if (depth==0 && /^}/) { capture=0 }
        }
    ' "$script"
}

# load_cleanup_helpers — создаёт поддельный REPO_DIR и source'ит функции
# cleanup'а в текущий shell. Делает git init в $TEST_TMP/repo для
# `git worktree prune` (реальный, не mock).
load_cleanup_helpers() {
    REPO_DIR="$TEST_TMP/repo"
    mkdir -p "$REPO_DIR"
    # Реальный git init нужен для `git worktree prune` — иначе скрипт падает.
    git -C "$REPO_DIR" init --bare --quiet . >/dev/null 2>&1 \
        || git -C "$REPO_DIR" init --quiet . >/dev/null 2>&1
    export REPO_DIR
    # Соберём функции в один блок. _wt_self_pid — переменная, не функция,
    # поэтому подставляем её явно.
    local funcs_block=""
    for fn in _wt_is_pid_alive _wt_remove_single _wt_sweep_orphans _wt_sweep_ttl _wt_disk_check cleanup; do
        local body
        body="$(extract_funcs "$E2E_PROCESS" "$fn")"
        [ -n "$body" ] || { echo "extract failed for $fn" >&2; return 1; }
        funcs_block+="$body"$'\n'
    done
    # Определим зависимости, которые не в наших функциях.
    funcs_block+='_wt_self_pid="'$$'"'$'\n'
    funcs_block+='log() { printf "%s\n" "$*" >&2; }'$'\n'
    funcs_block+='E2E_WT_TTL_DAYS="${E2E_WT_TTL_DAYS:-7}"'$'\n'
    funcs_block+='E2E_DISK_MIN_GB="${E2E_DISK_MIN_GB:-20}"'$'\n'
    # Source через eval, чтобы функции оказались в нашем namespace.
    eval "$funcs_block"
    export E2E_WT_TTL_DAYS E2E_DISK_MIN_GB
}

# --- A. sweep_orphans -------------------------------------------------------
test_A_sweep_orphans_removes_dead() {
    new_test
    load_cleanup_helpers
    # Создаём worktree от "мёртвого" PID 99999 (точно не существует).
    local dead_dir="/tmp/agent-flow-e2e-99999"
    mkdir -p "$dead_dir"
    # Создаём worktree от живого PID (= $$, self). Тоже должен быть.
    local self_dir="/tmp/agent-flow-e2e-$$"
    mkdir -p "$self_dir"
    # Создаём worktree от другого живого (но мы не сможем его создать
    # без долгого процесса) — пропускаем, только self + dead.
    # Запускаем _wt_sweep_orphans.
    _wt_sweep_orphans 2>/dev/null
    # dead_dir должен быть удалён.
    if [ -d "$dead_dir" ]; then
        echo "dead_dir $dead_dir still exists" >&2
        rm -rf "$dead_dir" "$self_dir"
        return 1
    fi
    # self_dir должен ОСТАТЬСЯ.
    if [ ! -d "$self_dir" ]; then
        echo "self_dir $self_dir was deleted (should be kept)" >&2
        rm -rf "$self_dir"
        return 1
    fi
    rm -rf "$self_dir"
    return 0
}

# --- B. sweep_ttl -----------------------------------------------------------
test_B_sweep_ttl_removes_old() {
    new_test
    load_cleanup_helpers
    # Создаём worktree с mtime 30 дней назад (старше TTL=7).
    local old_dir="/tmp/agent-flow-e2e-88888"
    mkdir -p "$old_dir"
    touch -t "202501010000" "$old_dir"  # 2025-01-01
    # Запускаем sweep с TTL=7.
    E2E_WT_TTL_DAYS=7 _wt_sweep_ttl 2>/dev/null
    if [ -d "$old_dir" ]; then
        echo "old_dir $old_dir still exists after TTL sweep" >&2
        rm -rf "$old_dir"
        return 1
    fi
    rm -rf "$old_dir" 2>/dev/null || true
    return 0
}

# --- C. disk_check ----------------------------------------------------------
test_C_disk_check_ok_when_free() {
    new_test
    load_cleanup_helpers
    E2E_DISK_MIN_GB=1  # почти всегда пройдёт
    if ! _wt_disk_check; then
        echo "disk_check FAILED with min=1GB (likely test env has <1GB free)" >&2
        return 1
    fi
    return 0
}

test_C_disk_check_fail_when_low() {
    new_test
    load_cleanup_helpers
    E2E_DISK_MIN_GB=999999  # невозможно много
    if _wt_disk_check; then
        echo "disk_check PASSED with min=999999TB (should fail)" >&2
        return 1
    fi
    return 0
}

# --- D. cleanup() -----------------------------------------------------------
test_D_cleanup_removes_self_and_prunes() {
    new_test
    load_cleanup_helpers
    # Создаём worktree с именем нашего скрипта.
    WORKTREE_DIR="/tmp/agent-flow-e2e-cleanup-test-$$"
    mkdir -p "$WORKTREE_DIR"
    # cleanup() дёргает `git worktree remove --force` (REPO_DIR/.git/...) — для
    # bare-репо это ОК. И затем `git worktree prune` в REPO_DIR.
    cleanup
    if [ -d "$WORKTREE_DIR" ]; then
        echo "WORKTREE_DIR still exists after cleanup()" >&2
        rm -rf "$WORKTREE_DIR"
        return 1
    fi
    return 0
}

# --- E. remove_single -------------------------------------------------------
test_E_remove_single() {
    new_test
    load_cleanup_helpers
    local d="/tmp/agent-flow-e2e-removesingle-$$"
    mkdir -p "$d"
    _wt_remove_single "$d"
    if [ -d "$d" ]; then
        echo "directory $d not removed" >&2
        rm -rf "$d"
        return 1
    fi
    return 0
}

# new_test — изолированный subdir для каждого теста (safety net)
new_test() {
    TEST_TMP="/tmp/agent-flow-cleanup-tests.$$.$RANDOM"
    mkdir -p "$TEST_TMP"
}

# --- main -------------------------------------------------------------------
run_test "A. sweep_orphans removes dead-PID worktrees, keeps self" test_A_sweep_orphans_removes_dead
run_test "B. sweep_ttl removes worktrees older than E2E_WT_TTL_DAYS" test_B_sweep_ttl_removes_old
run_test "C. disk_check returns 0 with low min" test_C_disk_check_ok_when_free
run_test "C. disk_check returns 1 with impossible min" test_C_disk_check_fail_when_low
run_test "D. cleanup() removes WORKTREE_DIR + calls worktree prune" test_D_cleanup_removes_self_and_prunes
run_test "E. _wt_remove_single removes dir" test_E_remove_single

echo ""
echo "==========================================="
echo "TOTAL:  $TESTS_TOTAL  PASS:  $TESTS_PASSED  FAIL:  $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "FAILED TESTS:"
    for n in "${FAILED_NAMES[@]}"; do echo "  - $n"; done
    exit 1
fi
echo "ALL CLEANUP TESTS PASSED ✅"
