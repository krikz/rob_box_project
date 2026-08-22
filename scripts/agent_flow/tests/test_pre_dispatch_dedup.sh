#!/bin/bash
# ============================================================================
# test_pre_dispatch_dedup.sh — ретро 22.08 issue #1540 acceptance test
#
# Acceptance criteria из issue #1540:
#   [ ] 2 параллельных тика merge-gate → 1 build на ветке (вместо 2)
#   [ ] Тесты: 2 синхронных вызова e2e-process → 1 build
#
# Тест проверяет: pre-dispatch dedup через verify_recent_run() блокирует
# второй `gh workflow run` если для той же workflow+branch уже есть свежий
# run за последние E2E_PRE_DISPATCH_WINDOW секунд (default 60).
#
# Сценарии:
#   A: invoke 1 раз с MOCK_RUN_OLD=1 (старый run) → 1 gh workflow run
#   B: invoke 2 раза подряд с MOCK_RUN_FRESH=1 → 1 gh workflow run
#      (KEY ACCEPTANCE #4: dedup блокирует второй)
#   C: invoke 2 раза подряд с MOCK_RUN_OLD=1 → 2 gh workflow run
#      (dedup НЕ сработал, оба вызова прошли)
#   D: invoke 1 раз с MOCK_RUN_FRESH=1 → 0 или 1 gh workflow run
#      (допустимо оба — мы не знаем был ли раньше run, но не больше 1)
# ============================================================================
set -euo pipefail

TEST_DIR="$(mktemp -d /tmp/test_pre_dispatch_dedup.XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT

# --- Mock gh shim --------------------------------------------------------
# Симулирует API: после `gh workflow run` (state.wf_run_count > 0)
# `gh run list` начинает возвращать createdAt=NOW (свежий run).
# MOCK_RUN_OLD=1 принудительно старит ответ gh run list.
mkdir -p "$TEST_DIR/bin"
cat > "$TEST_DIR/bin/gh" <<'GH_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }

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
                # После успешного trigger — отметить что был run (чтобы
                # следующий gh run list вернул fresh createdAt).
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
                # По умолчанию возвращаем NOW (fresh run).
                # Если есть флаг had_run И нет MOCK_RUN_OLD — fresh.
                # Если MOCK_RUN_OLD=1 — old (NOW - 3600).
                _now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
                _stub_created=""
                if [ "${MOCK_RUN_OLD:-0}" = "1" ] || [ ! -f "${state}.had_run" ]; then
                    _stub_created="$(date -u -d '3600 seconds ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                else
                    _stub_created="$_now_iso"
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

# --- Stub: log function --------------------------------------------------
log() { printf '[test] %s\n' "$*" >&2; }

# --- Source shared lib + извлечь _trigger_workflow_with_retry() в sandbox --
# Issue #1540: используем абсолютный путь к worktree (не develop).
SANDBOX="$TEST_DIR/sandbox.sh"
WORKTREE_DIR="/home/builder/rob_box_project/.worktrees/t_8c3a6a82/scripts/agent_flow"
cat > "$SANDBOX" <<SANDBOX_EOF
#!/bin/bash
set -euo pipefail

GH_REPO="krikz/rob_box_project"
_TBS="agent-flow-e2e-process"
_TBA="merge-gate"
_TBC=""
_TBR="pre-dispatch-test"

log() { printf '[sandbox] %s\n' "\$*" >&2; }

SANDBOX_LIB="$WORKTREE_DIR/lib_workflow_dedup.sh"
# shellcheck source=lib_workflow_dedup.sh
. "\$SANDBOX_LIB"

SANDBOX_EOF

# Извлекаем функцию из production-скрипта в worktree
awk '/^_trigger_workflow_with_retry\(\)/,/^}/' \
    "$WORKTREE_DIR/agent-flow-e2e-process.sh" >> "$SANDBOX"

chmod +x "$SANDBOX"

# --- Сценарии ------------------------------------------------------------
# Счётчик вызовов gh workflow run берём из state-файла (как в
# test_trigger_workflow_dedup.sh). Журнал GH_JOURNAL недоступен через
# bash -c из-за single-quotes вокруг \$TEST_DIR.
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

# A: invoke 1 раз с MOCK_RUN_OLD=1 → 1 call (базовый случай, dedup не сработал)
# $TEST_DIR/bin ставим ПЕРВЫМ в PATH, чтобы mock gh перебивал реальный.
# date/python3 (нужные для verify_recent_run) тоже добавляем.
scenario_A="export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.A' MOCK_RUN_OLD=1; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _cnt_file='$TEST_DIR/state.A.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call, got \$_n\"; exit 1)"

# B: invoke 2 раза подряд с MOCK_RUN_FRESH=1 → 1 call (pre-dispatch dedup блокирует второй)
scenario_B="export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.B' MOCK_RUN_OLD=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _cnt_file='$TEST_DIR/state.B.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call (pre-dispatch dedup), got \$_n\"; exit 1)"

# C: invoke 2 раза подряд с MOCK_RUN_OLD=1 → 2 calls (dedup НЕ сработал)
scenario_C="export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.C' MOCK_RUN_OLD=1; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _cnt_file='$TEST_DIR/state.C.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 2 ] && exit 0 || (echo \"expected 2 calls (no dedup, old run), got \$_n\"; exit 1)"

# D: invoke 1 раз с MOCK_RUN_FRESH=1 → 0 или 1 call (dedup может блокировать первый)
scenario_D="export PATH='$TEST_DIR/bin:/usr/bin:/bin' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.D' MOCK_RUN_OLD=0; source '$SANDBOX' 2>&1; _trigger_workflow_with_retry L-Build-All-Services.yml --ref develop; _cnt_file='$TEST_DIR/state.D.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -le 1 ] && exit 0 || (echo \"expected 0 or 1 call, got \$_n\"; exit 1)"

run_scenario "A: invoke 1 раз, old run → 1 call (no dedup)" "$scenario_A"
run_scenario "B: invoke 2 раза подряд, fresh run → 1 call (pre-dispatch dedup, KEY ACCEPTANCE #4)" "$scenario_B"
run_scenario "C: invoke 2 раза подряд, old run → 2 calls (no dedup)" "$scenario_C"
run_scenario "D: invoke 1 раз, fresh run → 0-1 call (pre-dispatch dedup may block first)" "$scenario_D"

# --- Итог -----------------------------------------------------------------
echo ""
echo "================================================================="
echo "test_pre_dispatch_dedup: PASS=$PASS FAIL=$FAIL"
echo "================================================================="
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
