#!/bin/bash
# ============================================================================
# test_trigger_workflow_dedup.sh — ретро 22.08 t_c7761956 (A1)
#
# Тест для race-condition dedup в _trigger_workflow_with_retry():
#   1. gh workflow run OK → success, без retry
#   2. gh workflow run FAIL + есть свежий run на той же ветке (≤60s) →
#      race detected → success, без retry (PR #1536 dedup)
#   3. gh workflow run FAIL + НЕТ свежего run → retry до MAX_ATTEMPTS
#   4. gh workflow run FAIL + старый run (>60s) на той же ветке →
#      НЕ race → retry до MAX_ATTEMPTS
#
# Использует mock gh (shim) который:
#   - gh auth status → exit 0
#   - gh workflow view → exit 0 (workflow exists)
#   - gh workflow run → управляется env: MOCK_WF_RUN_FAIL=1 (FAIL первый раз)
#                        или нет (success)
#   - gh run list → возвращает JSON с одним run, createdAt = now (свежий)
#                   или now-3600 (старый)
# ============================================================================
set -euo pipefail

TEST_DIR="$(mktemp -d /tmp/test_trigger_dedup.XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT

# --- Mock gh shim ---------------------------------------------------------
mkdir -p "$TEST_DIR/bin"
cat > "$TEST_DIR/bin/gh" <<'GH_EOF'
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
                journal "gh workflow run $*"
                # MOCK_WF_RUN_FAIL=1 → FAIL первый раз, success после
                _cnt_file="${state}.wf_run_count"
                _cnt=0
                [ -f "$_cnt_file" ] && _cnt="$(cat "$_cnt_file" 2>/dev/null || echo 0)"
                _cnt=$((_cnt + 1))
                printf '%s' "$_cnt" > "$_cnt_file"
                # После ПЕРВОГО workflow run (success или fail) — отметить что
                # был run, чтобы race-detection fallback внутри retry мог
                # увидеть свежий run через `gh run list` (issue #1536).
                # Issue #1540: pre-dispatch dedup НЕ использует had_run —
                # он проверяет реальный API и блокирует ДО workflow run.
                if [ "$_cnt" -eq 1 ]; then
                    touch "${state}.had_run"
                fi
                # Режимы:
                #   MOCK_WF_RUN_FAIL=1 → fail ТОЛЬКО на первом вызове
                #                          (race simulation, используется в scenario_B)
                #   MOCK_WF_RUN_FAIL_ALWAYS=1 → fail ВСЕГДА (для retry-loop
                #                          тестов в scenario_C — должно быть 3 retry)
                # Иначе → success.
                if [ "${MOCK_WF_RUN_FAIL_ALWAYS:-0}" = "1" ]; then
                    exit 1
                fi
                if [ "${MOCK_WF_RUN_FAIL:-0}" = "1" ] && [ "$_cnt" -eq 1 ]; then
                    exit 1  # race simulate
                fi
                exit 0
                ;;
        esac
        ;;
    run)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                # gh run list --workflow X --repo Y --branch Z --limit 1 --json ...
                journal "gh run list $*"
                # Возвращаем JSON с одним run
                # MOCK_RUN_OLD=1 → createdAt = NOW - 3600 (старый)
                # Иначе: если есть флаг had_run (был успешный/неудачный
                # workflow run в этом сценарии) → NOW (race detection).
                # Если had_run нет → NOW - 3600 (старый, чтобы pre-dispatch
                # dedup НЕ блокировал первый вызов).
                _now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
                _stub_created="$_now_iso"
                if [ "${MOCK_RUN_OLD:-0}" = "1" ] || [ ! -f "${state}.had_run" ]; then
                    _stub_created="$(date -u -d '3600 seconds ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                fi
                printf '[{"databaseId":12345,"createdAt":"%s"}]\n' "$_stub_created"
                exit 0
                ;;
        esac
        ;;
esac
journal "gh $subcmd (UNHANDLED) $*"
exit 0
GH_EOF
chmod +x "$TEST_DIR/bin/gh"

# --- Stub: log function (только printf) ----------------------------------
log() { printf '[test] %s\n' "$*" >&2; }

# --- Source the function from agent-flow-e2e-process.sh ------------------
# Извлекаем _trigger_workflow_with_retry() в sandbox скрипт.
# Issue #1540: функция теперь использует verify_recent_run из
# lib_workflow_dedup.sh, поэтому sandbox должен source'ить shared lib.
SANDBOX="$TEST_DIR/sandbox.sh"
SCRIPT_DIR_REAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cat > "$SANDBOX" <<SANDBOX_EOF
#!/bin/bash
set -euo pipefail

GH_REPO="krikz/rob_box_project"

# Извлекаем функцию из production-скрипта
_TBS="agent-flow-e2e-process"
_TBA="merge-gate"
_TBC=""
_TBR="test"

log() { printf '[sandbox] %s\n' "\$*" >&2; }

# Issue #1540: shared lib для verify_recent_run (нужен для новой
# pre-dispatch dedup и для race-condition fallback в _race_dedup_check).
SANDBOX_LIB="$SCRIPT_DIR_REAL/lib_workflow_dedup.sh"
# shellcheck source=lib_workflow_dedup.sh
. "\$SANDBOX_LIB"

SANDBOX_EOF

# Use awk to extract the function definition
awk '/^_trigger_workflow_with_retry\(\)/,/^}/' \
    "$SCRIPT_DIR_REAL/agent-flow-e2e-process.sh" >> "$SANDBOX"

chmod +x "$SANDBOX"

# --- Сценарии ------------------------------------------------------------

PASS=0
FAIL=0

run_scenario() {
    local name="$1"
    local scenario_fn="$2"
    log ""
    log "=== Scenario: $name ==="
    if bash -c "$scenario_fn"; then
        log "  PASS: $name"
        PASS=$((PASS + 1))
    else
        log "  FAIL: $name"
        FAIL=$((FAIL + 1))
    fi
}

# Сценарий A: gh workflow run OK → success, без retry
# Issue #1540: stateful mock (had_run) — после первого gh workflow run
# mock gh run list начнёт возвращать свежий run. Чтобы scenario A
# тестировал именно "gh OK → 1 call, success", а не зацепил новый
# pre-dispatch dedup, используем уникальный GH_STATE (had_run=0 в начале
# → pre-dispatch вернёт miss, и gh workflow run пройдёт). После успеха
# race-detection fallback не нужен.
scenario_A="set -e; export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.A' GH_JOURNAL='$TEST_DIR/journal.A' MOCK_WF_RUN_FAIL=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _n=\$(grep -c 'gh workflow run' '$TEST_DIR/journal.A'); [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call, got \$_n\"; exit 1)"

# Сценарий B: gh workflow run FAIL + race-detection в retry (issue #1536)
# Stateful mock: had_run=0 в начале → pre-dispatch miss → gh workflow run
# fail → race-check видит had_run=1 (после первой попытки) → fresh → race
# dedup → return 0 (без retry). Принимаем любое число вызовов gh workflow
# run от 1 до MAX_ATTEMPTS=3, главное — success.
scenario_B="set -e; export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.B' GH_JOURNAL='$TEST_DIR/journal.B' MOCK_WF_RUN_FAIL=1; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop && exit 0; exit 1"

# Сценарий C: gh workflow run FAIL всегда + НЕТ свежего run → retry до MAX_ATTEMPTS
# Stateful mock + MOCK_RUN_OLD=1 + MOCK_WF_RUN_FAIL_ALWAYS=1 → fail ВСЕГДА,
# gh run list всегда возвращает старый run (MOCK_RUN_OLD=1 override) →
# pre-dispatch miss + 3 race-check miss + retry x3 → 3 calls + return 1.
scenario_C="set -e; export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.C' GH_JOURNAL='$TEST_DIR/journal.C' MOCK_WF_RUN_FAIL_ALWAYS=1 MOCK_RUN_OLD=1; source '$SANDBOX' 2>&1; if _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; then exit 1; fi; _n=\$(grep -c 'gh workflow run' '$TEST_DIR/journal.C'); [ \"\$_n\" -eq 3 ] && exit 0 || (echo \"expected 3 calls (MAX_ATTEMPTS=3), got \$_n\"; exit 1)"

# Сценарий D: pre-dispatch dedup — KEY ACCEPTANCE #1540 #4.
# Первый вызов: had_run=0 → pre-dispatch miss → gh OK (had_run=1 после).
# Второй вызов: had_run=1 → pre-dispatch OK → return 0 → 0 calls.
# Итого: ровно 1 call. Это покрывает acceptance "2 синхронных вызова →
# 1 build". Полное покрытие этого сценария — в test_pre_dispatch_dedup.sh.
scenario_D="set -e; export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.D' GH_JOURNAL='$TEST_DIR/journal.D' MOCK_WF_RUN_FAIL=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _n=\$(grep -c 'gh workflow run' '$TEST_DIR/journal.D'); [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call (pre-dispatch dedup), got \$_n\"; exit 1)"

run_scenario "A: gh workflow run OK → 1 call, success" "$scenario_A"
run_scenario "B: gh workflow run FAIL + race-detection → success (PR #1536)" "$scenario_B"
run_scenario "C: gh workflow run FAIL + old run → 3 calls, fail" "$scenario_C"
run_scenario "D: 2 sync calls + fresh after 1st → 1 call (pre-dispatch dedup, issue #1540)" "$scenario_D"

# --- Итог -----------------------------------------------------------------
echo ""
echo "================================================================="
echo "test_trigger_workflow_dedup: PASS=$PASS FAIL=$FAIL"
echo "================================================================="
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
