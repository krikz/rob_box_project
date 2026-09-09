#!/bin/bash
# ============================================================================
# test_e2e_process_round_ensure_counter.sh — ретро 23.08 t_fdb19f7b (Phase 1+2)
#
# Проверяет counter-rollback guard в round_ensure() + ghost-round detection.
#
# Симптом: counter=209 vs remote=193 — счётчик убегает, 16 ghost-rounds
# (ветка создана → 0 run'ов → удалена в post-tick cleanup, counter остался).
#
# Сценарии:
#   A. counter-rollback: round_ensure создаёт ветку, потом кандидат снят
#      → post-tick cleanup удаляет ветку → counter должен откатиться к
#      max(remote_max, prev_counter), НЕ оставаться на n (= max+1).
#   B. ghost-round metric: cumulative счётчик ghost-rounds-total
#      инкрементируется на 1 при удалении пустой round-ветки.
#   C. нормальный round: counter ПЕРСИСТИТСЯ (max + 1) когда build реально
#      запустился (run'ы есть) — не сломали happy path.
#   D. (sanity) явный лог-маркер GHOST_ROUND counter_rollback в stderr.
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_round_ensure_counter.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
E2E_PROCESS="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"
TEST_LIB_DIR="$TEST_DIR/lib"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/mock_env.sh"

# --- Test registry ----------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
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
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

assert_not_contains() {
    case "$2" in
        *"$1"*)
            printf '  %sassert fail:%s %s\n    needle should NOT appear: %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
    return 0
}

# --- helpers (как в test_e2e_process_guard.sh) -----------------------------
new_test() {
    rm -rf "$TEST_TMP" 2>/dev/null || true
    mkdir -p "$TEST_TMP"
    : >"$TEST_TMP/stderr.log"
}

install_e2e_mocks_round() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"

    # Мок gh — расширенный: поддержка run list --json databaseId,
    # api -X DELETE refs/heads/<branch>, и всё что уже было в
    # test_e2e_process_guard.sh.
    cat > "$bin_dir/gh" <<'GH_MOCK_EOF'
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

subcmd="${1:-}"; shift || true

case "$subcmd" in
    issue|search|pr)
        action="${1:-}"; shift || true
        journal "gh ${subcmd} ${action} $*"
        case "${action:-}" in
            list)
                # Сначала --search (блокеры), потом --label (очередь).
                if printf '%s' "$*" | grep -q -- '--search'; then
                    # Как и в test_e2e_process_guard.sh: пустой ответ → null в jq
                    # → hit=null → НЕ блокер.
                    exit 0
                elif printf '%s' "$*" | grep -q -- '--label'; then
                    _data="$(get_state ISSUE_LIST_JSON)"
                    [ -n "$_data" ] || _data='[]'
                    printf '%s' "$_data"
                else
                    printf '[]'
                fi
                exit 0
                ;;
            view)
                _num="${1:-}"; shift || true
                journal "gh ${subcmd} view ${_num} $*"
                # Если хотят конкретное поле (--jq) — возвращаем state-фрагмент.
                _data="$(get_state "ISSUE_${_num}_LABELS_JSON")"
                [ -n "$_data" ] || _data='{"labels":[]}'
                if printf '%s' "$*" | grep -q -- '--jq'; then
                    _jq="$(printf '%s' "$*" | sed -nE 's/.*--jq[[:space:]]+([^ ]+).*/\1/p')"
                    printf '%s' "$_data" | python3 -c "
import sys, json
try:
    d = json.load(sys.stdin)
    expr = '''${_jq}'''.strip()
    # Minimal jq subset: '.labels | map(.name)' → list of names.
    if expr == '.labels | map(.name)':
        print(json.dumps([l.get('name','') for l in d.get('labels',[])]))
    elif expr == '.labels | map(.name) | any(. == \"no-e2e-required\")':
        print('true' if any(l.get('name','')=='no-e2e-required' for l in d.get('labels',[])) else 'false')
    else:
        print(json.dumps(d))
except Exception:
    print('null')
" 2>/dev/null || printf '%s' "$_data"
                else
                    printf '%s' "$_data"
                fi
                exit 0
                ;;
            *)
                exit 0
                ;;
        esac
        ;;
    label)
        journal "gh label $*"
        if printf '%s' "$1" | grep -q 'list'; then
            printf 'e2e:infra-fail\nneeds-e2e\ne2e-done\n'
        fi
        exit 0
        ;;
    run)
        journal "gh run $*"
        if [ "${1:-}" = "list" ] && printf '%s' "$*" | grep -q -- '--branch' \
            && printf '%s' "$*" | grep -q -- '--jq'; then
            _rb="$(printf '%s' "$*" | sed -nE 's/.*--branch[[:space:]]+([^ ]+).*/\1/p')"
            _data="$(get_state "RUN_${_rb}_JSON")"
            [ -n "$_data" ] || _data='[]'
            if printf '%s' "$*" | grep -q 'databaseId'; then
                printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin); print(len(d))
except Exception: print(0)'
                exit 0
            fi
            printf '%s' "$_data" | python3 -c 'import json,sys
try:
    d=json.load(sys.stdin)
    print(sum(1 for r in d if r.get("status") in ("queued","in_progress")))
except Exception: print(0)'
            exit 0
        fi
        _data="$(get_state RUN_LIST_JSON)"
        [ -n "$_data" ] || _data='[]'
        printf '%s' "$_data"
        exit 0
        ;;
    api)
        journal "gh api $*"
        if printf '%s' "$1" | grep -q 'rate_limit'; then
            printf '%s' '{"resources":{"core":{"remaining":5000},"graphql":{"remaining":5000}}}'
        fi
        exit 0
        ;;
    *)
        journal "gh $subcmd $*"
        exit 0
        ;;
esac
GH_MOCK_EOF
    chmod +x "$bin_dir/gh"

    # Мок git: ls-remote, fetch, push, worktree
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

# Сдираем -C <dir> префикс.
case "${1:-}" in
    -C) shift 2 || true ;;
esac

case "${1:-}" in
    ls-remote)
        shift || true
        # git ls-remote --heads origin <ref-pattern>
        if printf '%s' "$*" | grep -q -- '--heads'; then
            shift
            # Ищем pattern; для каждой строки в BRANCH_LIST_<prefix> — эмулируем.
            pattern="${1:-}"
            # Поддержка glob-паттерна: оставляем только префикс до { }.
            # Для простоты — если pattern = 'refs/heads/z-{e2e}/test-round-*',
            # берём BRANCH_LIST_TESTROUND.
            if [ "$pattern" = "refs/heads/z-{e2e}/test-round-*" ]; then
                _b="$(get_state BRANCH_LIST_TESTROUND)"
                [ -n "$_b" ] || _b=''
                # Каждая строка: <sha>\t<ref>
                if [ -z "$_b" ]; then
                    printf ''
                else
                    printf '%s\n' "$_b"
                fi
                exit 0
            fi
            # Точная проверка существования одной ветки.
            branch="${pattern#refs/heads/}"
            if [ "$(get_state "BRANCH_PRESENT_${branch}")" = "1" ]; then
                printf 'a29f8171f1ff346ac0098ba7f1ddeea0642d6ff7\trefs/heads/%s\n' "$branch"
            fi
            exit 0
        fi
        exit 0
        ;;
    fetch)
        journal "git fetch $*"
        exit 0
        ;;
    push)
        journal "git push $*"
        exit 0
        ;;
    worktree)
        journal "git worktree $*"
        exit 0
        ;;
    merge-base)
        # merge-base --is-ancestor origin/develop origin/<round>
        # для тестов round-ветки нет → возвращаем 1 (NOT ancestor).
        exit 1
        ;;
    show)
        # git show develop:MAINTENANCE / develop:RUN_NOW — нет такого ref → exit 1.
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

    cat > "$bin_dir/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
printf '%s\t%s\n' "$(date -Iseconds)" "hermes $*" >>"${GH_JOURNAL:-/dev/null}"
printf '%s' '{"task":{"status":"done"}}'
exit 0
HERMES_MOCK_EOF
    chmod +x "$bin_dir/hermes"

    export PATH="$bin_dir:$PATH"
    export GH_STATE="$TEST_TMP/gh_state"
    export GH_JOURNAL="$TEST_TMP/journal"
    : >"$GH_STATE"
    : >"$GH_JOURNAL"
}

set_state() {
    local key="$1" val="$2"
    # Перезаписать или добавить в GH_STATE
    local tmp
    tmp="$(mktemp)"
    if [ -f "$GH_STATE" ]; then
        grep -v "^${key}=" "$GH_STATE" > "$tmp" || true
    fi
    printf '%s=%s\n' "$key" "$val" >> "$tmp"
    mv "$tmp" "$GH_STATE"
}

make_repo_dir() {
    mkdir -p "$TEST_TMP/repo"
}

run_e2e() {
    (
        GH_REPO="${GH_REPO:-krikz/test-repo}"
        KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
        DRY_RUN="${DRY_RUN:-false}"
        ISSUE_LIMIT="${ISSUE_LIMIT:-20}"
        HERMES_HOME="$TEST_TMP/hermes_home"
        HERMES_BIN=hermes
        REPO_DIR="$TEST_TMP/repo"
        KANBAN_DB_PATH="$TEST_TMP/nonexistent-kanban.db"
        LOCK_FILE="$TEST_TMP/e2e-process.lock"
        ROUND_COUNTER_FILE="$TEST_TMP/round-counter"
        GHOST_ROUNDS_TOTAL_FILE="$TEST_TMP/ghost-rounds-total"
        export GH_REPO KANBAN_BOARD DRY_RUN ISSUE_LIMIT HERMES_HOME HERMES_BIN REPO_DIR KANBAN_DB_PATH LOCK_FILE ROUND_COUNTER_FILE GHOST_ROUNDS_TOTAL_FILE
        export HOME=/tmp
        timeout -k 5 30 bash "$E2E_PROCESS" 2>>"$TEST_TMP/stderr.log" || true
    )
}

# ============================================================================
# A. counter-rollback: round_ensure создал ветку, кандидат снят → cleanup
#    удаляет ветку → counter должен ОТКАТИТЬСЯ (НЕ оставаться на 1).
#
#    До фикса: counter = 1 после прогона (ghost, см. test_e2e_process_guard.sh:H).
#    После фикса: counter = "" (или 0) — round не оставил следа в нумерации.
# ============================================================================
test_A_counter_rollback_on_empty_round() {
    new_test
    install_e2e_mocks_round
    make_repo_dir

    local issue=5001
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    # guard видит кандидата (issue has OPEN PR).
    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    # Живой чек меток ПЕРЕД merge → кандидат уже снят (e2e-done).
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":5002,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_5002_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # ROUND_BRANCHES не задан → round_ensure создаст z-{e2e}/test-round-1.
    # RUN_z-{e2e}/test-round-1_JSON не задан → [] → 0 run'ов → cleanup DELETE.

    run_e2e

    local journal errlog
    journal="$(cat "$GH_JOURNAL")"
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live candidate(s) — создаю round" "$errlog" "A: guard пропустил к round_ensure"
    assert_contains "git push" "$journal" "A: round-ветка создана"
    assert_contains "skip merge" "$errlog" "A: merge скипнут"
    assert_contains "0 run'ов" "$errlog" "A: cleanup увидел 0 run'ов"
    assert_contains "gh api -X DELETE" "$journal" "A: round-ветка удалена через gh api"

    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "" "$counter" "A: counter откатился (НЕ остался на 1) — был ghost, стал чисто"
}

# ============================================================================
# B. ghost-round metric: cumulative счётчик /home/builder/.hermes/state/
#    agent-flow-e2e-ghost-rounds-total инкрементируется на 1 при удалении
#    пустой round-ветки.
#
#    Накопительный, переживает cleanup (как round-counter).
# ============================================================================
test_B_ghost_round_detection_emits_metric() {
    new_test
    install_e2e_mocks_round
    make_repo_dir

    local issue=5101
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":5102,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_5102_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # Стартовый cumulative = 4 (для проверки +1).
    mkdir -p "$(dirname "$TEST_TMP/ghost-rounds-total")"
    printf '4\n' > "$TEST_TMP/ghost-rounds-total"

    run_e2e

    local errlog
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "0 run'ов" "$errlog" "B: cleanup увидел 0 run'ов"

    local ghost_total
    ghost_total="$(cat "$TEST_TMP/ghost-rounds-total" 2>/dev/null || echo '')"
    assert_eq "5" "$ghost_total" "B: ghost-rounds-total инкрементирован (4 → 5)"

    # Явный маркер в логе для парсинга монитором.
    assert_contains "GHOST_ROUND counter_rollback" "$errlog" "B: лог содержит GHOST_ROUND counter_rollback"
}

# ============================================================================
# C. Нормальный round: counter ПЕРСИСТИТСЯ (max + 1) когда run'ы есть —
#    happy path не сломан.
# ============================================================================
test_C_counter_persists_on_real_run() {
    new_test
    install_e2e_mocks_round
    make_repo_dir

    local issue=5201
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" "{\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}]}"
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":5202,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_5202_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1
    # RUN_<round>_JSON = есть 1 run → cleanup НЕ удаляет.
    set_state "RUN_z-{e2e}/test-round-1_JSON" '[{"databaseId":5301,"status":"completed","conclusion":"success"}]'

    run_e2e

    local errlog
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "live candidate(s) — создаю round" "$errlog" "C: round создан"
    # counter должен сохранить 1.
    local counter
    counter="$(cat "$TEST_TMP/round-counter" 2>/dev/null || echo '')"
    assert_eq "1" "$counter" "C: counter персистится (= 1) для настоящего round"
}

# ============================================================================
# D. Sanity: лог-маркер GHOST_ROUND counter_rollback — парсер монитора.
# ============================================================================
test_D_log_marker_on_ghost() {
    new_test
    install_e2e_mocks_round
    make_repo_dir

    local issue=5301
    local title="fix #${issue} demo"
    local slug
    slug="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40)"
    local branch="z-{agent}/${issue}-${slug}"

    set_state ISSUE_LIST_JSON "[{\"number\":${issue},\"title\":\"${title}\",\"labels\":[{\"name\":\"hermes\"},{\"name\":\"needs-e2e\"}],\"body\":\"\"}]"
    set_state "ISSUE_${issue}_COMMENTS_JSON" '{"comments":[]}'
    set_state "ISSUE_${issue}_LABELS_JSON" '{"labels":[{"name":"hermes"},{"name":"e2e-done"}]}'
    set_state "PR_HEAD_${branch}_JSON" "[{\"number\":5302,\"state\":\"OPEN\",\"headRefName\":\"${branch}\"}]"
    set_state "PR_5302_VIEW_JSON" "{\"title\":\"${title}\",\"labels\":[{\"name\":\"agent:devops\"}]}"
    set_state "BRANCH_PRESENT_${branch}" 1

    run_e2e

    local errlog
    errlog="$(cat "$TEST_TMP/stderr.log")"

    assert_contains "GHOST_ROUND counter_rollback" "$errlog" "D: маркер GHOST_ROUND counter_rollback в логе"
}

# ============================================================================
# main
# ============================================================================
run_test "A: counter rollback on empty round"             test_A_counter_rollback_on_empty_round
run_test "B: ghost-round metric increments"               test_B_ghost_round_detection_emits_metric
run_test "C: counter persists on real run (happy path)"   test_C_counter_persists_on_real_run
run_test "D: GHOST_ROUND log marker on ghost"             test_D_log_marker_on_ghost

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
