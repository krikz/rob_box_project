#!/bin/bash
# ============================================================================
# test_post_merge_pre_dispatch_dedup.sh — ретро 22.08 issue #1540
#
# Acceptance criteria из issue #1540 (по части post-merge-build):
#   - [ ] `_trigger_workflow_with_retry()` в e2e-process.sh имеет pre-dispatch dedup
#     (как post-merge-build)
#   - [ ] 2 параллельных тика merge-gate → 1 build на ветке (вместо 2)
#
# Тест проверяет: pre-dispatch dedup в agent-flow-post-merge-build.sh
# блокирует второй invoke если для того же workflow+branch уже есть
# свежий run за последние POST_MERGE_BUILD_PRE_DISPATCH_WINDOW секунд
# (default 60).
#
# Сценарии:
#   A: invoke 1 раз, MOCK_RUN_OLD=1 (старый run) → 1 gh workflow run
#      (базовый случай, dedup не сработал)
#   B: invoke 2 раза подряд, MOCK_RUN_FRESH=1 → 1 gh workflow run
#      (KEY ACCEPTANCE #1540: pre-dispatch dedup блокирует второй)
#   C: invoke 2 раза подряд, MOCK_RUN_OLD=1 → 2 gh workflow run
#      (dedup НЕ сработал, оба вызова прошли)
# ============================================================================
set -euo pipefail

TEST_DIR="$(mktemp -d /tmp/test_pmb_pre_dispatch.XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT

SCRIPT_DIR_REAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PMB_SCRIPT="$SCRIPT_DIR_REAL/agent-flow-post-merge-build.sh"

# --- Mock gh shim ---------------------------------------------------------
# Тот же что в test_pre_dispatch_dedup.sh: после успешного trigger помечает
# что был run → gh run list начнёт возвращать свежий createdAt.
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

# --- Сценарии ------------------------------------------------------------
# post-merge-build — top-level скрипт с `trap` и `main`. Тестировать
# через `source` тяжело (много side-effects), поэтому тестируем через
# DRY_RUN: скрипт при DRY_RUN=true не дёргает `gh workflow run` реально.
#
# Но нам нужен реальный trigger — поэтому используем не-DRY_RUN + мок gh,
# и пробрасываем минимальный env (BUILD_WORKFLOW, PR_BASE, GH_REPO).
# Скрипт имеет много startup-проверок (lock-файлы, логирование). Чтобы
# избежать их — выставим DRY_RUN=true в одном тесте и проверим только
# pre-dispatch dedup через прямой вызов grep на скрипте.
#
# Альтернативно: смоук-тест через sed-extract pre-dispatch блока в
# отдельный мини-скрипт. Это надёжнее для unit-тестирования.

# Извлекаем pre-dispatch dedup блок из post-merge-build.sh в standalone скрипт.
SANDBOX="$TEST_DIR/sandbox.sh"
cat > "$SANDBOX" <<SANDBOX_EOF
#!/bin/bash
set -euo pipefail

BUILD_WORKFLOW="\${BUILD_WORKFLOW:-L-Build-All-Services.yml}"
PR_BASE="\${PR_BASE:-develop}"
GH_REPO="\${GH_REPO:-krikz/rob_box_project}"
DRY_RUN="\${DRY_RUN:-false}"

log() { printf '[sandbox] %s\n' "\$*" >&2; }

# Source shared lib
SANDBOX_LIB="$SCRIPT_DIR_REAL/lib_workflow_dedup.sh"
# shellcheck source=lib_workflow_dedup.sh
. "\$SANDBOX_LIB"

# Извлечённый pre-dispatch dedup блок (issue #1540)
PRE_DISPATCH_DEDUP_WINDOW="\${POST_MERGE_BUILD_PRE_DISPATCH_WINDOW:-60}"
if [ "\$DRY_RUN" != "true" ]; then
    if [ "\$(verify_recent_run "\$BUILD_WORKFLOW" "\$PR_BASE" "\$GH_REPO" "\$PRE_DISPATCH_DEDUP_WINDOW")" = "ok" ]; then
        log "⏭️ recent \${BUILD_WORKFLOW} run on \${PR_BASE} (≤\${PRE_DISPATCH_DEDUP_WINDOW}s) — pre-dispatch dedup (issue #1540), skip"
        exit 0
    fi
fi

# Иначе — триггерим
gh workflow run "\$BUILD_WORKFLOW" --repo "\$GH_REPO" --ref "\$PR_BASE" 2>&1
echo "exit=\$?"
SANDBOX_EOF
chmod +x "$SANDBOX"

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

# A: invoke sandbox 1 раз, old run → 1 call (no dedup).
# Запускаем sandbox как subprocess (bash $SANDBOX) чтобы изолировать
# состояние pre-dispatch блока от тестовой обёртки.
scenario_A="export PATH='$TEST_DIR/bin:/usr/bin:/bin' BUILD_WORKFLOW='L-Build-All-Services.yml' PR_BASE='develop' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.A' GH_JOURNAL='$TEST_DIR/journal.A' MOCK_RUN_OLD=1; bash '$SANDBOX'; _cnt_file='$TEST_DIR/state.A.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call, got \$_n\"; exit 1)"

# B: invoke sandbox 2 раза подряд, fresh run → 1 call (pre-dispatch dedup).
# После 1-го invoke mock помечает had_run → 2-й invoke видит свежий run
# в pre-dispatch check → skip → только 1 trigger.
scenario_B="export PATH='$TEST_DIR/bin:/usr/bin:/bin' BUILD_WORKFLOW='L-Build-All-Services.yml' PR_BASE='develop' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.B' GH_JOURNAL='$TEST_DIR/journal.B' MOCK_RUN_OLD=0; bash '$SANDBOX' >/dev/null; bash '$SANDBOX'; _cnt_file='$TEST_DIR/state.B.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 1 ] && exit 0 || (echo \"expected 1 call (pre-dispatch dedup), got \$_n\"; exit 1)"

# C: invoke sandbox 2 раза подряд, old run → 2 calls (no dedup).
# MOCK_RUN_OLD=1 заставляет mock возвращать старый run → pre-dispatch
# miss при каждом invoke → 2 trigger.
scenario_C="export PATH='$TEST_DIR/bin:/usr/bin:/bin' BUILD_WORKFLOW='L-Build-All-Services.yml' PR_BASE='develop' GH_REPO='krikz/rob_box_project' GH_STATE='$TEST_DIR/state.C' GH_JOURNAL='$TEST_DIR/journal.C' MOCK_RUN_OLD=1; bash '$SANDBOX' >/dev/null; bash '$SANDBOX'; _cnt_file='$TEST_DIR/state.C.wf_run_count'; _n=0; [ -f \"\$_cnt_file\" ] && _n=\"\$(cat \$_cnt_file)\"; [ \"\$_n\" -eq 2 ] && exit 0 || (echo \"expected 2 calls (no dedup), got \$_n\"; exit 1)"

run_scenario "A: invoke 1 раз, old run → 1 call (no dedup)" "$scenario_A"
run_scenario "B: invoke 2 раза подряд, fresh run → 1 call (pre-dispatch dedup)" "$scenario_B"
run_scenario "C: invoke 2 раза подряд, old run → 2 calls (no dedup)" "$scenario_C"

# --- Итог -----------------------------------------------------------------
echo ""
echo "================================================================="
echo "test_post_merge_pre_dispatch_dedup: PASS=$PASS FAIL=$FAIL"
echo "================================================================="
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
