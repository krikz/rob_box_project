#!/bin/bash
# ============================================================================
# test_agents_sleep.sh — авто-сон агентов по расписанию DeepSeek peak/off-peak.
#
# Покрывает acceptance issue #1281:
#   - PEAK    (04:00–07:00, 09:00–13:00 MSK) → MAINTENANCE появляется в develop
#   - OFF-PEAK                               → MAINTENANCE удаляется (auto)
#   - ручной MAINTENANCE (без маркера)       → НЕ трогается (human window)
#   - идемпотентность                        → повторный тик без нового коммита
#   - DRY_RUN=true                           → никаких git-операций
#
# Два уровня:
#   1. Unit: source скрипта + вызов is_peak() на границах окон.
#   2. Integration: локальный bare-repo как "origin" (без сети), реальные
#      git clone/fetch/commit/push — проверяем состояние origin/develop.
#
# Run:
#   bash scripts/agent_flow/tests/test_agents_sleep.sh
# Returns exit 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
AGENTS_SLEEP="$TEST_ROOT_DIR/agents_sleep.sh"
SCHEDULE_CONF="$TEST_ROOT_DIR/agents_sleep_schedule.conf"

TEST_TMP="${TEST_TMP:-/tmp/agent-flow-agents-sleep-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP"

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {  # $1=name, $2=function
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

# --- unit: is_peak() ---------------------------------------------------------
# Source the script (main не запускается — source-guard в конце файла).
# shellcheck source=../agents_sleep.sh
. "$AGENTS_SLEEP"

test_is_peak_boundaries() {
    # Границы окон [start, end): 04:00-07:00, 09:00-13:00 MSK
    is_peak "03:59" && return 1
    is_peak "04:00" || return 1
    is_peak "06:59" || return 1
    is_peak "07:00" && return 1
    is_peak "08:59" && return 1
    is_peak "09:00" || return 1
    is_peak "12:59" || return 1
    is_peak "13:00" && return 1
    is_peak "13:01" && return 1
    is_peak "00:00" && return 1
    is_peak "23:59" && return 1
    return 0
}

test_is_peak_custom_conf() {
    # Кастомное расписание через env (как если бы conf задал другое)
    local saved="$PEAK_HOURS"
    PEAK_HOURS="22:00-23:30"
    is_peak "22:00" || { PEAK_HOURS="$saved"; return 1; }
    is_peak "23:29" || { PEAK_HOURS="$saved"; return 1; }
    is_peak "23:30" && { PEAK_HOURS="$saved"; return 1; }
    is_peak "21:59" && { PEAK_HOURS="$saved"; return 1; }
    PEAK_HOURS="$saved"
    return 0
}

# --- integration: локальный bare-repo как origin -----------------------------
# setup_origin: создаёт bare origin + локальный clone (для seed-коммитов).
# После setup: $ORIGIN_DIR — bare, $SEED_DIR — рабочее дерево с develop.
ORIGIN_DIR=""
SEED_DIR=""
SLEEP_REPO_DIR=""

# Мок merge-gate для resume-шага (ретро t_2c814334): не ходим в сеть/gh,
# просто пишем маркер, что merge-gate был бы вызван.
MOCK_MERGE_GATE="$TEST_TMP/mock-merge-gate.sh"
MOCK_MERGE_GATE_MARKER="$TEST_TMP/merge-gate-called.marker"
cat > "$MOCK_MERGE_GATE" <<EOF
#!/bin/bash
# mock merge-gate: фиксируем факт вызова
echo "merge-gate called at \$(date -Iseconds)" >> "$MOCK_MERGE_GATE_MARKER"
exit 0
EOF
chmod +x "$MOCK_MERGE_GATE"

reset_merge_gate_marker() {
    rm -f "$MOCK_MERGE_GATE_MARKER"
}

merge_gate_was_called() {
    [ -f "$MOCK_MERGE_GATE_MARKER" ]
}

setup_origin() {
    ORIGIN_DIR="$TEST_TMP/origin.git"
    SEED_DIR="$TEST_TMP/seed"
    # каждый тест стартует с чистого origin (тесты изолированы)
    rm -rf "$ORIGIN_DIR" "$SEED_DIR"
    git init --bare -q "$ORIGIN_DIR"
    git init -q "$SEED_DIR"
    git -C "$SEED_DIR" config user.email test@example.com
    git -C "$SEED_DIR" config user.name "Test Seed"
    printf 'base\n' > "$SEED_DIR/README.md"
    git -C "$SEED_DIR" add README.md
    git -C "$SEED_DIR" commit -q -m "base commit"
    git -C "$SEED_DIR" branch -M develop
    git -C "$SEED_DIR" remote add origin "$ORIGIN_DIR"
    git -C "$SEED_DIR" push -q -u origin develop
}

# reset_sleep_repo: удаляет клон скрипта, чтобы следующий запуск клонировал заново.
reset_sleep_repo() {
    SLEEP_REPO_DIR="$TEST_TMP/sleep-repo"
    rm -rf "$SLEEP_REPO_DIR"
}

# run_agents_sleep <now_msk> [extra_env...] — запуск скрипта в изоляции.
run_agents_sleep() {
    local now="$1"; shift
    env \
        AGENTS_SLEEP_REPO="$SLEEP_REPO_DIR" \
        AGENTS_SLEEP_REMOTE="$ORIGIN_DIR" \
        AGENTS_SLEEP_CONF="$SCHEDULE_CONF" \
        NOW_MSK="$now" \
        LOCK_FILE="$TEST_TMP/agents-sleep.lock" \
        RESUME_MERGE_GATE_CMD="$MOCK_MERGE_GATE" \
        "$@" \
        bash "$AGENTS_SLEEP" 2>&1 || return $?
}

# origin_has_maintenance: 0 если MAINTENANCE есть на origin/develop
origin_has_maintenance() {
    git --git-dir="$ORIGIN_DIR" ls-tree develop --name-only 2>/dev/null | grep -qx MAINTENANCE
}

# origin_maintenance_content — содержимое MAINTENANCE на origin/develop
origin_maintenance_content() {
    git --git-dir="$ORIGIN_DIR" show develop:MAINTENANCE 2>/dev/null || true
}

# origin_commit_count — число коммитов на develop (идемпотентность)
origin_commit_count() {
    git --git-dir="$ORIGIN_DIR" rev-list --count develop 2>/dev/null
}

test_peak_creates_maintenance() {
    setup_origin
    reset_sleep_repo
    run_agents_sleep "05:30" >/dev/null
    origin_has_maintenance || return 1
    # содержимое — с авто-маркером
    assert_contains "auto-sleep:" "$(origin_maintenance_content)" "auto marker in MAINTENANCE content"
}

test_peak_idempotent_no_new_commit() {
    setup_origin
    reset_sleep_repo
    run_agents_sleep "05:30" >/dev/null
    local before
    before="$(origin_commit_count)"
    # повторный тик в том же окне — не должен плодить коммит
    run_agents_sleep "06:00" >/dev/null
    local after
    after="$(origin_commit_count)"
    assert_eq "$before" "$after" "no new commit on repeated peak tick"
    origin_has_maintenance || return 1
}

test_offpeak_removes_auto_maintenance() {
    setup_origin
    reset_sleep_repo
    run_agents_sleep "05:30" >/dev/null   # peak → MAINTENANCE появился
    origin_has_maintenance || return 1
    run_agents_sleep "14:00" >/dev/null   # off-peak → auto MAINTENANCE снят
    if origin_has_maintenance; then return 1; fi   # MAINTENANCE должен исчезнуть
}

test_offpeak_idempotent_no_new_commit() {
    setup_origin
    reset_sleep_repo
    run_agents_sleep "05:30" >/dev/null   # peak → создать
    run_agents_sleep "14:00" >/dev/null   # off-peak → снять
    local before
    before="$(origin_commit_count)"
    run_agents_sleep "15:00" >/dev/null   # снова off-peak, MAINTENANCE нет
    local after
    after="$(origin_commit_count)"
    assert_eq "$before" "$after" "no new commit on repeated off-peak tick"
}

test_offpeak_preserves_manual_maintenance() {
    setup_origin
    # ручной MAINTENANCE без авто-маркера (человеческое окно обслуживания)
    printf 'maintenance: pause agent-flow crons — fixing voice chain (human)\n' > "$SEED_DIR/MAINTENANCE"
    git -C "$SEED_DIR" add MAINTENANCE
    git -C "$SEED_DIR" commit -q -m "maintenance: manual pause (human)"
    git -C "$SEED_DIR" push -q origin develop
    reset_sleep_repo
    run_agents_sleep "14:00" >/dev/null   # off-peak
    origin_has_maintenance || return 1    # ручной — НЕ трогаем
}

test_peak_keeps_existing_maintenance() {
    setup_origin
    printf 'maintenance: pause agent-flow crons — manual (human)\n' > "$SEED_DIR/MAINTENANCE"
    git -C "$SEED_DIR" add MAINTENANCE
    git -C "$SEED_DIR" commit -q -m "maintenance: manual pause"
    git -C "$SEED_DIR" push -q origin develop
    reset_sleep_repo
    local before
    before="$(origin_commit_count)"
    run_agents_sleep "05:30" >/dev/null   # peak, MAINTENANCE уже есть
    local after
    after="$(origin_commit_count)"
    assert_eq "$before" "$after" "no commit when MAINTENANCE already present at peak"
}

test_dry_run_does_not_touch_git() {
    setup_origin
    reset_sleep_repo
    local before
    before="$(origin_commit_count)"
    local out
    out="$(DRY_RUN=true run_agents_sleep "05:30")"
    local after
    after="$(origin_commit_count)"
    assert_eq "$before" "$after" "DRY_RUN: no commits"
    origin_has_maintenance && return 1
    assert_contains "DRY-RUN" "$out" "DRY_RUN prints decision"
}

test_second_window_creates_again() {
    # После снятия в off-peak следующий peak снова ставит MAINTENANCE (цикл)
    setup_origin
    reset_sleep_repo
    run_agents_sleep "05:30" >/dev/null
    run_agents_sleep "14:00" >/dev/null
    origin_has_maintenance && return 1
    run_agents_sleep "10:00" >/dev/null
    origin_has_maintenance || return 1
}

# --- resume → merge-gate backfill-скан по всем open PR (ретро t_2c814334) ----

test_resume_runs_merge_gate() {
    setup_origin
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "05:30" >/dev/null   # peak → MAINTENANCE появился
    origin_has_maintenance || return 1
    run_agents_sleep "14:00" >/dev/null   # off-peak → resume
    if origin_has_maintenance; then return 1; fi
    merge_gate_was_called || return 1     # merge-gate вызван после resume
}

test_peak_does_not_run_merge_gate() {
    setup_origin
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "05:30"   # peak (создание MAINTENANCE)
    if merge_gate_was_called; then return 1; fi     # merge-gate НЕ вызывается на peak
}

test_idle_offpeak_does_not_run_merge_gate() {
    setup_origin
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "14:00"   # off-peak, MAINTENANCE нет — идемпотентно
    if merge_gate_was_called; then return 1; fi     # merge-gate НЕ вызывается (не resume)
}

test_resume_disabled_skips_merge_gate() {
    setup_origin
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "05:30" >/dev/null
    origin_has_maintenance || return 1
    run_agents_sleep "14:00" RESUME_MERGE_GATE_ENABLED=false >/dev/null
    if origin_has_maintenance; then return 1; fi    # resume всё равно произошёл
    if merge_gate_was_called; then return 1; fi     # но merge-gate не вызван (рубильник)
}

test_dry_run_resume_does_not_run_merge_gate() {
    setup_origin
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "05:30" >/dev/null
    origin_has_maintenance || return 1
    local out
    out="$(DRY_RUN=true run_agents_sleep "14:00")"
    if ! origin_has_maintenance; then return 1; fi    # DRY_RUN: MAINTENANCE остался
    if merge_gate_was_called; then return 1; fi     # merge-gate НЕ вызван (DRY_RUN)
    assert_contains "DRY-RUN: would run merge-gate" "$out" "DRY_RUN prints merge-gate decision"
}

test_manual_maintenance_resume_does_not_run_merge_gate() {
    setup_origin
    # ручной MAINTENANCE без авто-маркера (человеческое окно обслуживания)
    printf 'maintenance: pause agent-flow crons — fixing voice chain (human)\n' > "$SEED_DIR/MAINTENANCE"
    git -C "$SEED_DIR" add MAINTENANCE
    git -C "$SEED_DIR" commit -q -m "maintenance: manual pause (human)"
    git -C "$SEED_DIR" push -q origin develop
    reset_sleep_repo
    reset_merge_gate_marker
    run_agents_sleep "14:00" >/dev/null   # off-peak
    origin_has_maintenance || return 1    # ручной — НЕ трогаем
    if merge_gate_was_called; then return 1; fi     # и merge-gate НЕ вызываем (resume не было)
}

# ===========================================================================
# Run
# ===========================================================================
run_test "unit: is_peak границы окон (04-07,09-13 MSK)" test_is_peak_boundaries
run_test "unit: is_peak кастомное расписание через conf" test_is_peak_custom_conf
run_test "integration: peak → MAINTENANCE появляется (auto marker)" test_peak_creates_maintenance
run_test "integration: повторный peak → без нового коммита" test_peak_idempotent_no_new_commit
run_test "integration: off-peak → auto MAINTENANCE снят" test_offpeak_removes_auto_maintenance
run_test "integration: повторный off-peak → без нового коммита" test_offpeak_idempotent_no_new_commit
run_test "integration: ручной MAINTENANCE в off-peak → НЕ трогаем" test_offpeak_preserves_manual_maintenance
run_test "integration: peak + уже есть MAINTENANCE → без коммита" test_peak_keeps_existing_maintenance
run_test "integration: DRY_RUN → git не тронут" test_dry_run_does_not_touch_git
run_test "integration: цикл peak→off→peak (снова MAINTENANCE)" test_second_window_creates_again
run_test "integration: resume → merge-gate backfill-скан вызван" test_resume_runs_merge_gate
run_test "integration: peak → merge-gate НЕ вызывается" test_peak_does_not_run_merge_gate
run_test "integration: off-peak без MAINTENANCE → merge-gate НЕ вызывается" test_idle_offpeak_does_not_run_merge_gate
run_test "integration: RESUME_MERGE_GATE_ENABLED=false → resume без merge-gate" test_resume_disabled_skips_merge_gate
run_test "integration: DRY_RUN resume → merge-gate НЕ вызывается" test_dry_run_resume_does_not_run_merge_gate
run_test "integration: ручной MAINTENANCE → merge-gate НЕ вызывается" test_manual_maintenance_resume_does_not_run_merge_gate

summary() {
    echo
    echo "=============================================================="
    if [ "$TESTS_FAILED" -eq 0 ]; then
        printf '%sALL PASS%s (%d/%d)\n' "$GRN" "$END" "$TESTS_PASSED" "$TESTS_TOTAL"
        return 0
    fi
    printf '%sFAIL%s (%d/%d):\n' "$RED" "$END" "$TESTS_FAILED" "$TESTS_TOTAL"
    for n in "${FAILED_NAMES[@]}"; do
        printf '  - %s\n' "$n"
    done
    return 1
}

summary
