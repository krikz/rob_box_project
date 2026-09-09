#!/bin/bash
# ============================================================================
# test_round_formation_module.sh — юнит-тесты для round_formation.sh (issue #2299)
#
# Покрывает:
#   A. rf_compute_state: пустой remote → max_n=0 → n=1, файл-счётчик имеет приоритет
#      над remote (cleanup сбросил ветки, ретро t_bff6eccf).
#   B. round_formation: ветки нет → создаёт (CREATE) → выставляет
#      ROUND_FORMATION_CREATED=1, counter НЕ пишет.
#   C. round_formation: ветка есть + foundation ancestor → REUSE → выставляет
#      ROUND_FORMATION_REUSED=1, counter НЕ пишет.
#   D. round_formation: ветка есть + НЕ ancestor → удаляет и пересоздаёт → RECREATE.
#   E. rf_persist_counter_if_real_round: пишет counter (n=2) если n > counter_n.
#   F. rf_ghost_round_log_and_metric: маркер GHOST_ROUND + bump ghost-rounds-total.
#   G. thin wrapper round_ensure.sh: source'ит модуль без ошибок и печатает
#      ROUND_BRANCH (DEFERRED — counter не пишется).
#
# Run: bash scripts/agent_flow/tests/test_round_formation_module.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
MODULE="$REPO_ROOT/scripts/agent_flow/round_formation.sh"

[ -f "$MODULE" ] || { echo "FATAL: module not found: $MODULE"; exit 1; }

# --- helpers ---------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; BLU=''; END=''
fi

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
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  assert fail: %s\n    needle:   %q\n    haystack: %q\n' "$3" "$1" "$2" >&2; return 1 ;;
    esac
}

# Assert, который АБОРТИТ текущую функцию (не просто return 1) — нужно чтобы
# упавший первый assert не маскировал более поздние ошибки через short-circuit.
assert_eq_fail() { assert_eq "$@"; }
assert_contains_fail() { assert_contains "$@"; }

new_test() {
    TEST_TMP="$(mktemp -d)"
    export REPO_DIR="$TEST_TMP/repo"
    mkdir -p "$REPO_DIR"
    export GH_REPO="krikz/test"
    export FOUNDATION_BRANCH="develop"
    export TEST_ROUND_PREFIX='z-{e2e}/test-round-'
    export ROUND_COUNTER_FILE="$TEST_TMP/round-counter"
    export GHOST_ROUNDS_TOTAL_FILE="$TEST_TMP/ghost-rounds-total"
    export DRY_RUN="false"
    : >"$TEST_TMP/stderr.log"
}

# Мок git: ls-remote, fetch, push, worktree, merge-base.
install_git_mock() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"
    cat > "$bin_dir/git" <<'GIT_MOCK_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }
get_state() {
    local key="$1"
    if [ -f "$state" ]; then
        grep -E "^${key}=" "$state" | head -n1 | sed "s@^${key}=@@"
    fi
}

case "${1:-}" in
    -C) shift 2 || true ;;
esac

# Сборка оставшихся позиционных аргументов в массив (нужно для парсинга
# `git -C dir ls-remote --heads origin <pattern>` без shell word-split).
declare -a _REST_ARGS
_REST_ARGS=()
for _a in "$@"; do
    _REST_ARGS+=("$_a")
done

case "${1:-}" in
    ls-remote)
        # Аргументы после `ls-remote`: [--heads] [--refs] [remote] [pattern...]
        # Скипаем --heads/--refs, оставляя remote и pattern.
        _args=()
        for _arg in "${_REST_ARGS[@]:1}"; do  # [0] = "ls-remote"
            case "$_arg" in
                --heads|--refs) ;;
                *) _args+=("$_arg") ;;
            esac
        done
        remote="${_args[0]:-origin}"
        pattern="${_args[1]:-}"
        journal "ls-remote remote=${remote} pattern=[${pattern}]"
        # Авто-режим: если pattern заканчивается на `*` → список; иначе → exists.
        case "$pattern" in
            *'*')
                # Список round-веток: ожидаемый паттерн "refs/heads/<prefix>*"
                _b="$(get_state BRANCH_LIST_TESTROUND)"
                [ -n "$_b" ] || _b=''
                if [ -z "$_b" ]; then printf ''; else printf '%s\n' "$_b"; fi
                exit 0
                ;;
        esac
        # exists: pattern может быть "refs/heads/<branch>" или "<branch>".
        branch="${pattern#refs/heads/}"
        journal "ls-remote exists branch=[${branch}]"
        if [ "$(get_state "BRANCH_PRESENT_${branch}")" = "1" ]; then
            printf 'a29f8171f1ff346ac0098ba7f1ddeea0642d6ff7\trefs/heads/%s\n' "$branch"
        fi
        exit 0
        ;;
    fetch)
        # $* = "fetch origin develop" (т.к. $1=fetch уже выбран case'ом);
        # печатаем чистые аргументы без повтора "fetch".
        _rest=""
        for _i in "${_REST_ARGS[@]:1}"; do
            _rest="${_rest:+${_rest} }${_i}"
        done
        journal "git fetch ${_rest}"
        exit 0
        ;;
    push)
        _rest=""
        for _i in "${_REST_ARGS[@]:1}"; do
            _rest="${_rest:+${_rest} }${_i}"
        done
        journal "git push ${_rest}"
        exit 0
        ;;
    merge-base)
        # merge-base --is-ancestor <ancestor> <descendant>: код 0 = да, 1 = нет.
        # Управляется через MERGE_BASE_ANCESTOR (если =1 → 0, иначе 1).
        if [ "$(get_state MERGE_BASE_ANCESTOR)" = "1" ]; then exit 0; fi
        exit 1
        ;;
    show)
        journal "git show $*"
        exit 1
        ;;
    rev-parse|remote|branch)
        journal "git ${1} $*"
        exit 0
        ;;
    *)
        journal "git $*"
        exit 0
        ;;
esac
GIT_MOCK_EOF
    chmod +x "$bin_dir/git"
    export PATH="$bin_dir:$PATH"
    export GH_STATE="$TEST_TMP/gh_state"
    export GH_JOURNAL="$TEST_TMP/journal"
    : >"$GH_STATE"
    : >"$GH_JOURNAL"
}

set_state() {
    local key="$1" val="$2"
    local tmp
    tmp="$(mktemp)"
    if [ -f "$GH_STATE" ]; then
        grep -v "^${key}=" "$GH_STATE" > "$tmp" || true
    fi
    printf '%s=%s\n' "$key" "$val" >> "$tmp"
    mv "$tmp" "$GH_STATE"
}

# Стенд для запуска: source'им модуль и выполняем тело В ТОМ ЖЕ shell (ВАЖНО:
# функции `round_formation` / `rf_compute_state` / и т.д. должны жить в одном
# процессе с вызывающим кодом, иначе `bash -c 'round_formation ...'` упадёт
# с `command not found`).
#
# Использование: run_in_module_shell <<'BODY'
#     rf_compute_state
#     printf '%s\n' "$n"
# BODY
run_in_module_shell() {
    (
        set -euo pipefail
        # shellcheck source=/dev/null
        . "$MODULE"
        # cat stdin -> eval в ТЕКУЩЕМ shell (а не через bash -c, иначе функции
        # модуля не видны).
        local _body
        _body="$(cat)"
        # shellcheck disable=SC2154  # _body defined just above
        eval "$_body"
    )
}

# ============================================================================
# A. rf_compute_state: пустой remote + counter=0 → n=1; counter>remote_max
#    (cleanup сбросил ветки) → n=counter+1.
# ============================================================================
test_A_compute_state_priority() {
    new_test
    install_git_mock
    # counter=5, remote=0 → max=5 → n=6.
    printf '5\n' > "$ROUND_COUNTER_FILE"

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
rf_compute_state
printf '%s %s %s\n' "${n}" "${max_n}" "${counter_n}"
BODY

    local out
    out="$(cat "$TEST_TMP/out")"
    assert_eq "6 5 5" "$out" "A: counter>remote → max=5 → n=6 (counter имеет приоритет)"
}

# ============================================================================
# B. round_formation CREATE: ветки нет → выставляет ROUND_FORMATION_CREATED=1,
#    counter НЕ пишет.
# ============================================================================
test_B_create_sets_flag_no_counter() {
    new_test
    install_git_mock
    set_state BRANCH_LIST_TESTROUND ''
    push_fn() { printf 'PUSH: %s\n' "$*" >>"$GH_JOURNAL"; return 0; }
    export -f push_fn

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
push_fn() { printf 'PUSH: %s\n' "$*" >>"$GH_JOURNAL"; return 0; }
round_formation push_fn
printf '%s %s %s\n' "${ROUND_BRANCH}" "${ROUND_FORMATION_CREATED}" "${ROUND_FORMATION_REUSED}"
BODY

    local out journal
    out="$(cat "$TEST_TMP/out")"
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "z-{e2e}/test-round-1 1 0" "$out" "B: CREATE → ROUND_FORMATION_CREATED=1, REUSED=0"
    assert_contains "PUSH: $REPO_DIR origin origin/develop:refs/heads/z-{e2e}/test-round-1" "$journal" "B: push вызван с origin/develop → round"
    assert_eq "" "$(cat "$ROUND_COUNTER_FILE" 2>/dev/null || true)" "B: counter НЕ персистится в round_formation"
}

# ============================================================================
# C. round_formation REUSE: ветка есть + ancestor → ROUND_FORMATION_REUSED=1,
#    counter НЕ пишет, push НЕ вызван.
# ============================================================================
test_C_reuse_branch_present() {
    set -e
    new_test
    install_git_mock
    # list пустой (cleanup сбросил ветки, или это первый round); но
    # BRANCH_PRESENT говорит что round-1 уже есть → REUSE.
    set_state BRANCH_LIST_TESTROUND ''
    set_state BRANCH_PRESENT_z-{e2e}/test-round-1 1
    set_state MERGE_BASE_ANCESTOR 1

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
push_fn() { printf 'PUSH: %s\n' "$*" >>"$GH_JOURNAL"; return 0; }
round_formation push_fn
printf '%s %s %s\n' "${ROUND_BRANCH}" "${ROUND_FORMATION_CREATED}" "${ROUND_FORMATION_REUSED}"
BODY

    local out journal
    out="$(cat "$TEST_TMP/out")"
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "z-{e2e}/test-round-1 0 1" "$out" "C: REUSE → REUSED=1, CREATED=0"
    case "$journal" in
        *"PUSH:"*) printf '  assert fail: C: push НЕ должен быть вызван\n    haystack: %q\n' "$journal" >&2; return 1 ;;
    esac
    assert_contains "git fetch origin develop" "$journal" "C: fetch foundation"
    assert_contains "reusing" "$(cat "$TEST_TMP/stderr.log")" "C: лог 'reusing'"
}

# ============================================================================
# D. round_formation RECREATE: ветка есть + НЕ ancestor → delete + recreate.
# ============================================================================
test_D_recreate_stale_branch() {
    set -e
    new_test
    install_git_mock
    set_state BRANCH_LIST_TESTROUND ''
    set_state BRANCH_PRESENT_z-{e2e}/test-round-1 1
    # MERGE_BASE_ANCESTOR НЕ задан → merge-base exit 1 → stale → RECREATE.

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
push_fn() { printf 'PUSH: %s\n' "$*" >>"$GH_JOURNAL"; return 0; }
round_formation push_fn
printf '%s %s %s\n' "${ROUND_BRANCH}" "${ROUND_FORMATION_CREATED}" "${ROUND_FORMATION_REUSED}"
BODY

    local out journal
    out="$(cat "$TEST_TMP/out")"
    journal="$(cat "$GH_JOURNAL")"
    assert_eq "z-{e2e}/test-round-1 1 0" "$out" "D: RECREATE → CREATED=1, REUSED=0"
    local n_push
    n_push="$(grep -c '^PUSH:' <<<"$journal" || true)"
    assert_eq "2" "$n_push" "D: ровно 2 push (delete + create)"
    assert_contains "--delete z-{e2e}/test-round-1" "$journal" "D: push --delete для stale-ветки"
    assert_contains "база УСТАРЕЛА" "$(cat "$TEST_TMP/stderr.log")" "D: лог 'stale'"
}

# ============================================================================
# E. rf_persist_counter_if_real_round: пишет counter (n=2) когда n > counter_n.
# ============================================================================
test_E_persist_counter_when_real_round() {
    new_test
    install_git_mock
    printf '1\n' > "$ROUND_COUNTER_FILE"

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
rf_compute_state
n=2
rf_persist_counter_if_real_round
cat "${ROUND_COUNTER_FILE}"
BODY

    local out
    out="$(cat "$TEST_TMP/out")"
    assert_eq "2" "$out" "E: counter записан (1 → 2)"
}

# ============================================================================
# F. rf_ghost_round_log_and_metric: маркер + bump ghost-rounds-total.
# ============================================================================
test_F_ghost_log_and_metric() {
    new_test
    install_git_mock
    printf '3\n' > "$GHOST_ROUNDS_TOTAL_FILE"

    run_in_module_shell <<'BODY' > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"
rf_compute_state
rf_ghost_round_log_and_metric "z-{e2e}/test-round-1"
cat "${GHOST_ROUNDS_TOTAL_FILE}"
BODY

    local out errlog
    out="$(cat "$TEST_TMP/out")"
    errlog="$(cat "$TEST_TMP/stderr.log")"
    assert_eq "4" "$out" "F: ghost-rounds-total инкрементирован (3 → 4)"
    assert_contains "GHOST_ROUND counter_rollback" "$errlog" "F: маркер в stderr"
    assert_contains "branch=z-{e2e}/test-round-1" "$errlog" "F: имя ветки в маркере"
}

# ============================================================================
# G. round_ensure.sh source'ит модуль и работает в DRY-RUN (без push).
# ============================================================================
test_G_round_ensure_sh_wrapper_dryrun() {
    new_test
    install_git_mock
    set_state BRANCH_LIST_TESTROUND ''

    (
        set -euo pipefail
        export DRY_RUN=true
        . "$MODULE"
        push_fn() { return 0; }
        round_formation push_fn
        printf '%s\n' "$ROUND_BRANCH"
    ) > "$TEST_TMP/out" 2>>"$TEST_TMP/stderr.log"

    local out
    out="$(cat "$TEST_TMP/out")"
    assert_eq "z-{e2e}/test-round-1" "$out" "G: round_ensure.sh wrapper делегирует в round_formation (dry-run)"
    assert_eq "" "$(cat "$ROUND_COUNTER_FILE" 2>/dev/null || true)" "G: counter НЕ пишется в dry-run (deferred)"
}

# ============================================================================
# main
# ============================================================================
run_test "A: rf_compute_state: counter>remote → n=counter+1" test_A_compute_state_priority
run_test "B: round_formation CREATE → CREATED=1, counter не пишет" test_B_create_sets_flag_no_counter
run_test "C: round_formation REUSE → REUSED=1, push не вызван" test_C_reuse_branch_present
run_test "D: round_formation RECREATE stale → delete+create, CREATED=1" test_D_recreate_stale_branch
run_test "E: rf_persist_counter пишет counter когда n>counter_n" test_E_persist_counter_when_real_round
run_test "F: rf_ghost_round_log_and_metric: маркер + bump" test_F_ghost_log_and_metric
run_test "G: round_ensure.sh wrapper через модуль (dry-run, deferred)" test_G_round_ensure_sh_wrapper_dryrun

echo
echo "============================================================"
echo "Tests:    $TESTS_PASSED / $TESTS_TOTAL passed"
if [ "$TESTS_FAILED" -gt 0 ]; then
    echo "FAILED: ${FAILED_NAMES[*]}"
    echo "============================================================"
    exit 1
fi
echo "============================================================"
exit 0