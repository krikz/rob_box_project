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
# E. merge-gate НЕ триггерит post-merge build (issue #2294, ADR-AF-0064).
#
# ЧТО БЫЛО ДО ФИКСА (и почему тест маскировал баг):
#   merge-gate.sh содержал внутри reconcile-блока конструкцию
#     if [ "$pr_base" = "$DEVELOP_BRANCH" ]; then  log skip
#     elif [ -f .../agent-flow-post-merge-build.sh ]; then bash ...  fi
#   Внешний guard блока — `pr_state=MERGED && pr_base=$DEVELOP_BRANCH` —
#   гарантирует $pr_base == develop, поэтому ветка `elif` была МЁРТВОЙ.
#   Старый сценарий E вырезал кусок скрипта через sed по номерам строк,
#   сорсил его отдельно и подставлял pr_base=main В ОБХОД внешнего guard'а.
#   Тест «зеленел» на коде, который в проде не исполняется никогда, —
#   классическая маскировка (issue #2294).
#
# ЧТО ТЕПЕРЬ: сценарий гоняет РЕАЛЬНЫЙ agent-flow-merge-gate.sh целиком
# через штатный харнес tests/lib/mock_env.sh (mock gh/git/hermes, фикстура
# MERGED-PR), см. tests/lib/pmb_merge_gate_probe.sh. Проверяем:
#   E1. base=develop → merge-gate входит в reconcile, пишет skip-лог,
#       post-merge-build.sh НЕ вызван ни разу;
#   E2. base=main    → merge-gate вообще не входит в reconcile-блок
#       (внешний guard), post-merge-build.sh НЕ вызван. main-build
#       обеспечивает workflow «G-Auto-merge to Main» (ADR-AF-0064);
#   E3. mutation control: если вернуть вызов post-merge-build.sh в
#       reconcile-блок, probe ОБЯЗАН его увидеть (PMB_CALLS>0). Иначе
#       E1/E2 были бы вакуумно-зелёными.
#
# probe запускается отдельным процессом: lib/mock_env.sh несёт собственные
# счётчики тестов и assert_*, и его нельзя сорсить внутрь этого файла.
# ============================================================================
PROBE_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/pmb_merge_gate_probe.sh"

# probe_field <probe_output> <KEY> → значение KEY= из вывода probe.
probe_field() {
    printf '%s\n' "$1" | grep -E "^$2=" | head -1 | cut -d= -f2
}

scenario_E_merge_gate_never_triggers_post_merge_build() {
    local rc=0

    if [ ! -f "$PROBE_SCRIPT" ]; then
        printf '  FAIL: probe не найден: %s\n' "$PROBE_SCRIPT" >&2
        fail_log+=("merge-gate probe missing")
        return 1
    fi

    # --- E1: base=develop ---------------------------------------------
    local out_dev
    out_dev="$(bash "$PROBE_SCRIPT" develop 2>/dev/null || true)"
    assert_eq "1" "$(probe_field "$out_dev" RECONCILE)" \
        "E1 develop: merge-gate вошёл в post-merge reconcile блок" || rc=1
    assert_eq "1" "$(probe_field "$out_dev" SKIP_LOG)" \
        "E1 develop: skip-лог 'skipping post-merge build' присутствует" || rc=1
    assert_eq "0" "$(probe_field "$out_dev" PMB_CALLS)" \
        "E1 develop: post-merge-build.sh НЕ вызван" || rc=1

    # --- E2: base=main -------------------------------------------------
    # Внешний guard (MERGED && base=develop) не пускает main в reconcile;
    # ниже по коду issue с e2e-done уходит в idempotency-skip. В обоих
    # случаях post-merge-build.sh не должен быть вызван.
    local out_main
    out_main="$(bash "$PROBE_SCRIPT" main 2>/dev/null || true)"
    assert_eq "0" "$(probe_field "$out_main" RECONCILE)" \
        "E2 main: merge-gate НЕ входит в reconcile (внешний guard)" || rc=1
    assert_eq "0" "$(probe_field "$out_main" PMB_CALLS)" \
        "E2 main: post-merge-build.sh НЕ вызван (main build → G-Auto-merge to Main)" || rc=1

    # --- E3: mutation control -----------------------------------------
    # Возвращаем мёртвый вызов в копию merge-gate и убеждаемся, что probe
    # его ЛОВИТ. Без этого шага E1/E2 могли бы «зеленеть» просто потому,
    # что probe ничего не измеряет.
    #
    # merge-gate сорсит соседние lib_agent_flow_common.sh и вызывает
    # sibling-скрипты через dirname "$BASH_SOURCE" — поэтому мутант кладём
    # в каталог-зеркало со симлинками на всё содержимое scripts/agent_flow.
    local mutant_dir="$TEST_DIR/mg"
    local mutant="$mutant_dir/agent-flow-merge-gate.sh"
    mkdir -p "$mutant_dir"
    local sib
    for sib in "$SCRIPT_DIR_REAL"/*; do
        [ "$(basename "$sib")" = "agent-flow-merge-gate.sh" ] && continue
        ln -sfn "$sib" "$mutant_dir/$(basename "$sib")"
    done

    local marker='log "issue #${number}: skipping post-merge build for ${pr_base}'
    if ! grep -qF -- "$marker" "$MERGE_GATE_SCRIPT"; then
        printf '  FAIL: skip-лог маркер не найден в merge-gate — тест рассинхронизирован\n' >&2
        fail_log+=("merge-gate skip marker sync")
        return 1
    fi
    # После строки-маркера вставляем восстановленный вызов post-merge-build.
    awk -v ins='        bash "${REPO_DIR}/scripts/agent_flow/agent-flow-post-merge-build.sh" "${pr_number}" "${pr_base}" 2>/dev/null || true' '
        { print }
        index($0, "skipping post-merge build for ${pr_base}") > 0 && !done {
            print ins
            done = 1
        }
    ' "$MERGE_GATE_SCRIPT" > "$mutant"

    if ! grep -qF 'agent-flow-post-merge-build.sh" "${pr_number}"' "$mutant"; then
        printf '  FAIL: mutation не применилась (awk не вставил вызов)\n' >&2
        fail_log+=("merge-gate mutation apply")
        return 1
    fi
    if ! bash -n "$mutant" 2>/dev/null; then
        printf '  FAIL: mutant merge-gate не проходит bash -n\n' >&2
        fail_log+=("merge-gate mutant syntax")
        return 1
    fi

    local out_mut
    out_mut="$(PMB_MERGE_GATE_OVERRIDE="$mutant" bash "$PROBE_SCRIPT" develop 2>/dev/null || true)"
    local mut_calls
    mut_calls="$(probe_field "$out_mut" PMB_CALLS)"
    if [ "${mut_calls:-0}" -lt 1 ]; then
        printf '  FAIL: mutation control — probe НЕ увидел восстановленный вызов post-merge-build (PMB_CALLS=%s). E1/E2 вакуумны.\n' "${mut_calls:-?}" >&2
        fail_log+=("merge-gate probe mutation control")
        rc=1
    fi

    return $rc
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
run_scenario "E: merge-gate НЕ триггерит post-merge build (реальный скрипт, #2294)" scenario_E_merge_gate_never_triggers_post_merge_build
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
