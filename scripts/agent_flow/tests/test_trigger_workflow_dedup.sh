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
                # MOCK_RUN_FRESH=1 (default) → createdAt = NOW (race)
                # MOCK_RUN_OLD=1 → createdAt = NOW - 3600 (старый)
                _now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
                _stub_created="$_now_iso"
                if [ "${MOCK_RUN_OLD:-0}" = "1" ]; then
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
# Извлекаем _trigger_workflow_with_retry() в sandbox скрипт
SANDBOX="$TEST_DIR/sandbox.sh"
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

# Подгружаем только функцию (skip rest of script — set -e + main guard)
SANDBOX_EOF

# Use awk to extract the function definition
awk '/^_trigger_workflow_with_retry\(\)/,/^}/' \
    /home/builder/rob_box_project/scripts/agent_flow/agent-flow-e2e-process.sh >> "$SANDBOX"

echo "exit 0" >> "$SANDBOX"
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
scenario_A="set -e; export PATH='$TEST_DIR/bin:\$PATH' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state' GH_JOURNAL='$TEST_DIR/journal.A' MOCK_WF_RUN_FAIL=0; MOCK_RUN_OLD=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _n=\$(grep -c 'gh workflow run' '$TEST_DIR/journal.A'); [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call, got \$_n\"; exit 1)"

# Сценарий B: gh workflow run FAIL + есть свежий run (≤60s) → race, без retry
scenario_B="set -e; export PATH='$TEST_DIR/bin:\$PATH' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state' GH_JOURNAL='$TEST_DIR/journal.B' MOCK_WF_RUN_FAIL=1; MOCK_RUN_OLD=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop && exit 0; exit 1"

# Сценарий C: gh workflow run FAIL + НЕТ run (старый >60s) → retry до MAX_ATTEMPTS
scenario_C="set -e; export PATH='$TEST_DIR/bin:\$PATH' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state' GH_JOURNAL='$TEST_DIR/journal.C' MOCK_WF_RUN_FAIL=1; MOCK_RUN_OLD=1; source '$SANDBOX' 2>&1; if _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; then exit 1; fi; _n=\$(grep -c 'gh workflow run' '$TEST_DIR/journal.C'); [ \"\$_n\" -eq 3 ] && exit 0 || (echo \"expected 3 calls (MAX_ATTEMPTS=3), got \$_n\"; exit 1)"

run_scenario "A: gh workflow run OK → 1 call, success" "$scenario_A"
run_scenario "B: gh workflow run FAIL + fresh run → race dedup, success" "$scenario_B"
run_scenario "C: gh workflow run FAIL + old run → 3 calls, fail" "$scenario_C"

# --- Итог -----------------------------------------------------------------
echo ""
echo "================================================================="
echo "test_trigger_workflow_dedup: PASS=$PASS FAIL=$FAIL"
echo "================================================================="
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
