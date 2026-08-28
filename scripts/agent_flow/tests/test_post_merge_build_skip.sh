#!/usr/bin/env bash
# ============================================================================
# test_post_merge_build_skip.sh — issue #1625 acceptance (Шифу 25.08)
#
# Проверяет, что после фикса:
#   A. PR_BASE=develop → post-merge-build.sh EXIT 0, НЕ дёргает gh workflow run
#   B. PR_BASE=main    → post-merge-build.sh триггерит gh workflow run
#   C. DISABLE_POST_MERGE_BUILD=1 + PR_BASE=main → exit 0, НЕ триггерит
#   D. merge-gate с pr_base=develop (MERGED PR) НЕ вызывает post-merge-build
#   E. merge-gate с pr_base=main (MERGED PR) ВЫЗЫВАЕТ post-merge-build
#   F. develop-skip происходит ДО pre-dispatch dedup и auth/проверок workflow
#      (экономим API calls, логи: "skipped post-merge build for develop")
#
# Запуск:
#   bash scripts/agent_flow/tests/test_post_merge_build_skip.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(mktemp -d /tmp/test_pmb_skip.XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT

SCRIPT_DIR_REAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PMB_SCRIPT="$SCRIPT_DIR_REAL/agent-flow-post-merge-build.sh"
MERGE_GATE_SCRIPT="$SCRIPT_DIR_REAL/agent-flow-merge-gate.sh"

if [ ! -f "$PMB_SCRIPT" ]; then
    echo "FAIL: $PMB_SCRIPT не найден" >&2
    exit 1
fi

# --- Mock gh shim --------------------------------------------------------
# gh auth status → ok; gh workflow view → ok; gh workflow run → journal+1
# gh run list → empty (нет недавних runs → pre-dispatch dedup miss → trigger).
# ВАЖНО: пишем и в journal, и в stderr — чтобы видеть вызовы в общем логе
# теста (где скрипт post-merge-build.sh пишет свои логи в stderr).
mkdir -p "$TEST_DIR/bin"
cat > "$TEST_DIR/bin/gh" <<'GH_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; printf '%s\t%s\n' "$ts" "$*" >&2; }

subcmd="${1:-}"; shift || true
case "$subcmd" in
    auth)
        journal "gh auth status"
        exit 0
        ;;
    workflow)
        action="${1:-}"; shift || true
        case "$action" in
            view)
                journal "gh workflow view $*"
                exit 0
                ;;
            run)
                _cnt_file="${state}.wf_run_count"
                _cnt=0
                [ -f "$_cnt_file" ] && _cnt="$(cat "$_cnt_file" 2>/dev/null || echo 0)"
                _cnt=$((_cnt + 1))
                printf '%s' "$_cnt" > "$_cnt_file"
                journal "gh workflow run [$_cnt] $*"
                touch "${state}.had_run"
                exit 0
                ;;
        esac
        ;;
    run)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                journal "gh run list $*"
                # Никаких recent runs → pre-dispatch dedup пропускает.
                echo '[]'
                exit 0
                ;;
        esac
        ;;
esac
journal "gh $subcmd (UNHANDLED) $*"
exit 0
GH_EOF
chmod +x "$TEST_DIR/bin/gh"

# --- counter helper ------------------------------------------------------
count_wf_runs() {
    local state_dir="$1"
    local f="${state_dir}.wf_run_count"
    if [ -f "$f" ]; then
        cat "$f"
    else
        echo 0
    fi
}

# --- runner: post-merge-build.sh -----------------------------------------
# $1 = state_dir, остальные env через экспорт
# Скрипт пишет логи в stderr — добавляем stderr в journal (2>>), чтобы
# assert_journal_contains видел и "skipped post-merge build for develop"
# (лог скрипта), и "gh workflow run ..." (вызовы mock gh).
run_pmb() {
    local state_dir="$1"
    local pr_number="$2"
    local pr_base="$3"
    local extra_env="${4:-}"
    : >"$state_dir.journal"
    GH_STATE="$state_dir" \
    GH_JOURNAL="$state_dir.journal" \
    PATH="$TEST_DIR/bin:/usr/bin:/bin" \
    bash -c "$extra_env bash '$PMB_SCRIPT' '$pr_number' '$pr_base' 2>>'$state_dir.journal'"
}

PASS=0
FAIL=0
fail_log=()

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" = "$2" ]; then
        return 0
    fi
    printf '  FAIL: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
    fail_log+=("$3")
    return 1
}

assert_journal_contains() {  # $1=file $2=needle $3=msg
    # `-F` = literal string, `--` = stop treating leading-dash args as flags
    # (важно для needle вида "--ref main" / "DISABLED by env").
    if grep -qF -- "$2" "$1"; then
        return 0
    fi
    printf '  FAIL: %s\n    needle:   %q\n    journal:  %s\n' "$3" "$2" "$(cat "$1")" >&2
    fail_log+=("$3")
    return 1
}

assert_journal_missing() {  # $1=file $2=needle $3=msg
    if ! grep -qF -- "$2" "$1"; then
        return 0
    fi
    printf '  FAIL: %s\n    needle should NOT appear: %q\n    journal:  %s\n' "$3" "$2" "$(cat "$1")" >&2
    fail_log+=("$3")
    return 1
}

run_scenario() {
    local name="$1"
    local fn="$2"
    echo ""
    echo "=== Scenario: $name ==="
    if "$fn"; then
        echo "  PASS: $name"
        PASS=$((PASS+1))
    else
        echo "  FAIL: $name"
        FAIL=$((FAIL+1))
    fi
}

# ============================================================================
# A. PR_BASE=develop → exit 0, 0 gh workflow run, лог "skipped"
# ============================================================================
scenario_A_develop_skips() {
    local st="$TEST_DIR/A"
    run_pmb "$st" "1625" "develop" "" >/dev/null

    # 0 workflow runs.
    local n
    n="$(count_wf_runs "$st")"
    assert_eq "0" "$n" "develop: 0 gh workflow run calls"

    # Нет "gh workflow view" и "gh auth status" — skip ДО pre-flight checks.
    assert_journal_missing "$st.journal" "gh workflow view" "develop: skip happens before workflow view pre-flight"
    assert_journal_missing "$st.journal" "gh auth status" "develop: skip happens before gh auth pre-flight"

    # Лог содержит маркер "skipped post-merge build for develop".
    assert_journal_contains "$st.journal" "skipped post-merge build for develop" "develop: log shows skip marker"
}

# ============================================================================
# B. PR_BASE=main → 1 gh workflow run, лог "eligible ... production safety"
# ============================================================================
scenario_B_main_triggers() {
    local st="$TEST_DIR/B"
    run_pmb "$st" "1625" "main" "" >/dev/null

    local n
    n="$(count_wf_runs "$st")"
    assert_eq "1" "$n" "main: exactly 1 gh workflow run call"

    # gh workflow view вызван (pre-flight passed).
    assert_journal_contains "$st.journal" "gh workflow view" "main: pre-flight workflow view ran"

    # Запустили с --ref main.
    assert_journal_contains "$st.journal" "--ref main" "main: --ref main passed to gh workflow run"

    # Лог "eligible" присутствует.
    assert_journal_contains "$st.journal" "eligible for post-merge build" "main: log shows eligible marker"
}

# ============================================================================
# C. DISABLE_POST_MERGE_BUILD=1 + main → 0 runs, лог "DISABLED by env"
# ============================================================================
scenario_C_env_disable_overrides() {
    local st="$TEST_DIR/C"
    run_pmb "$st" "1625" "main" "DISABLE_POST_MERGE_BUILD=1 " >/dev/null

    local n
    n="$(count_wf_runs "$st")"
    assert_eq "0" "$n" "env=1 main: 0 gh workflow run calls"

    assert_journal_missing "$st.journal" "gh workflow view" "env=1: skip before pre-flight (cheap path)"
    assert_journal_contains "$st.journal" "DISABLED by env" "env=1: log shows DISABLED marker"

    # Также для develop: env=1 → всё равно skip (env=1 — hard kill).
    local st2="$TEST_DIR/C2"
    run_pmb "$st2" "1625" "develop" "DISABLE_POST_MERGE_BUILD=1 " >/dev/null
    assert_eq "0" "$(count_wf_runs "$st2")" "env=1 develop: also 0 calls (kill switch is global)"
    assert_journal_contains "$st2.journal" "DISABLED by env" "env=1 develop: log shows DISABLED marker"
}

# ============================================================================
# D. merge-gate с pr_base=develop НЕ вызывает post-merge-build (regression).
#    Прямой grep по коду merge-gate — это unit-проверка наличия guard'а.
#    Ищем именно ту конструкцию, которую мы вставили в issue #1625.
# ============================================================================
scenario_D_merge_gate_has_develop_guard() {
    local has_guard
    has_guard="$(grep -cE 'pr_base.*=.*DEVELOP_BRANCH.*then|"\$pr_base" = "\$DEVELOP_BRANCH"' "$MERGE_GATE_SCRIPT" || true)"
    if [ "${has_guard:-0}" -lt 1 ]; then
        printf '  FAIL: merge-gate не содержит develop-guard для post-merge-build (grep нашёл %s совпадений)\n' "$has_guard" >&2
        fail_log+=("merge-gate develop-guard")
        return 1
    fi

    # Дополнительно: guard должен быть в окрестности "skipping post-merge build"
    # (страховка от случайной регрессии — например, если кто-то удалит skip-log).
    local skip_log_count
    skip_log_count="$(grep -c 'skipping post-merge build' "$MERGE_GATE_SCRIPT" || true)"
    if [ "${skip_log_count:-0}" -lt 1 ]; then
        printf '  FAIL: merge-gate не содержит "skipping post-merge build" log-маркер (issue #1625)\n' >&2
        fail_log+=("merge-gate skip log")
        return 1
    fi

    return 0
}

# ============================================================================
# E. merge-gate base-check: develop → skip log; main → вызов скрипта.
#    Симулируем через mock_env.sh-подобную обвязку в мини-форме.
# ============================================================================
# Создаём минимальную обвязку: стаб для post-merge-build.sh (вместо
# реального скрипта — записывает вызов в журнал), и стаб gh, чтобы
# merge-gate прошёл остальные стадии. Затем — вытаскиваем из merge-gate
# ровно тот if-блок с ADR-0022 extension и проверяем его поведение в
# изоляции через subshell source.
scenario_E_merge_gate_guard_isolated() {
    local st="$TEST_DIR/E"

    # Создаём stub post-merge-build.sh — пишет факт вызова в journal.
    mkdir -p "$st/repo/scripts/agent_flow"
    cat > "$st/repo/scripts/agent_flow/agent-flow-post-merge-build.sh" <<EOF
#!/bin/bash
echo "POST_MERGE_BUILD_CALLED pr=\$1 base=\$2" >> "$st.journal"
exit 0
EOF
    chmod +x "$st/repo/scripts/agent_flow/agent-flow-post-merge-build.sh"

# Извлекаем guard-блок по строкам 1813-1830 (где мы его вставили в #1625).
# Используем номера строк, а не awk-паттерн — надёжнее (паттерн может
# пересечься с другими местами файла).
local guard_first_line guard_last_line
guard_first_line="$(grep -n 'ADR-0022 extension (issue #1475): после merge' "$MERGE_GATE_SCRIPT" | head -1 | cut -d: -f1)"
guard_last_line="$(awk -v start="$guard_first_line" 'NR>=start && /Re-read current labels/ {print NR; exit}' "$MERGE_GATE_SCRIPT")"

if [ -z "$guard_first_line" ] || [ -z "$guard_last_line" ]; then
    printf '  FAIL: не удалось найти guard-блок (first=%s last=%s)\n' "$guard_first_line" "$guard_last_line" >&2
    fail_log+=("merge-gate guard locate")
    return 1
fi

local guard_src
guard_src="$(sed -n "${guard_first_line},${guard_last_line}p" "$MERGE_GATE_SCRIPT")"

    # Подставляем переменные и запускаем дважды: для develop и для main.
    log() { :; }  # noop для log() в блоке

    # Develop: guard должен сработать, stub НЕ вызван.
    : >"$st.journal"
    local number=1 pr_number=1625 pr_base=develop DEVELOP_BRANCH=develop REPO_DIR="$st/repo"
    eval "$guard_src" || true
    local dev_lines
    dev_lines="$(wc -l < "$st.journal")"
    assert_eq "0" "$dev_lines" "develop: post-merge-build stub NOT called (guard active)"

    # Main: guard НЕ срабатывает, stub вызван ровно 1 раз.
    : >"$st.journal"
    pr_base=main
    eval "$guard_src" || true
    local main_lines
    main_lines="$(wc -l < "$st.journal")"
    assert_eq "1" "$main_lines" "main: post-merge-build stub called exactly once"
    assert_journal_contains "$st.journal" "POST_MERGE_BUILD_CALLED pr=1625 base=main" "main: stub received correct args"
}

# ============================================================================
# F. develop-skip происходит ДО pre-dispatch dedup:
#    journal НЕ содержит "gh run list" (dedup-вызов) для develop.
# ============================================================================
scenario_F_develop_skip_before_dedup() {
    local st="$TEST_DIR/F"
    run_pmb "$st" "1625" "develop" "" >/dev/null

    # develop-ветка должна skip'нуться ДО pre-dispatch dedup → не дёргаем
    # `gh run list` (это и есть acceptance #1625: дешёвый skip).
    assert_journal_missing "$st.journal" "gh run list" "develop: skip happens before pre-dispatch dedup (no gh run list call)"
    assert_journal_missing "$st.journal" "gh workflow run" "develop: skip happens before any trigger attempt"
}

# --- main ----------------------------------------------------------------
run_scenario "A: develop → exit 0, 0 trigger, log 'skipped'" scenario_A_develop_skips
run_scenario "B: main → 1 trigger, log 'eligible'" scenario_B_main_triggers
run_scenario "C: DISABLE_POST_MERGE_BUILD=1 → 0 trigger на main и develop" scenario_C_env_disable_overrides
run_scenario "D: merge-gate содержит develop-guard (regression)" scenario_D_merge_gate_has_develop_guard
run_scenario "E: merge-gate base-check изолированно (develop skip / main call)" scenario_E_merge_gate_guard_isolated
run_scenario "F: develop-skip ДО pre-dispatch dedup (no gh run list)" scenario_F_develop_skip_before_dedup

echo ""
echo "================================================================="
echo "test_post_merge_build_skip: PASS=$PASS FAIL=$FAIL"
echo "================================================================="
if [ "$FAIL" -gt 0 ]; then
    echo "Failures:"
    for f in "${fail_log[@]}"; do
        echo "  - $f"
    done
    exit 1
fi
exit 0
