#!/bin/bash
# ============================================================================
# test_e2e_fail_streak_auto_issue.sh — регресс-гард для auto-create-issue
# ветки в agent-flow-e2e-fail-streak-watchdog.sh (ADR-FS-001, kanban t_401e52de).
#
# Контекст: 8 fail-прогонов подряд прошли молча без issue. Watchdog должен
# создавать ОДИН issue с лейблом `e2e-fail-streak` + rate-limit (default 4ч),
# чтобы избежать шторма issues на каждый тик.
#
# Scenarios (PATH-hijack mock-gh / mock-git, без сети):
#   S1. streak < threshold (3) → NO gh issue create
#   S2. streak >= threshold (5) + cooldown absent + no existing → DRY-RUN:
#       пишет в лог "DRY-RUN would: gh issue create"
#   S3. streak >= threshold (5) + cooldown FRESH → skip, log "ISSUE_COOLDOWN active"
#   S4. streak >= threshold (5) + existing open e2e-fail-streak issue → skip
#   S5. streak >= threshold (5) + cooldown OK + no existing → gh issue create
#       вызван с правильным label/title/body, cooldown записан
#   S6. streak=8 → ровно ОДИН вызов gh issue create за прогон (acceptance)
#   S7. cooldown STALE (>4ч назад) → НЕ skip
#   S8. gh issue create fails (rc=1) → cooldown НЕ записан, exit 0
#   S9. E2E_FAIL_STREAK_ISSUE_ASSIGNEE → передан в --assignee
#   Sanity: маркер присутствует в issue body
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_fail_streak_auto_issue.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-e2e-fail-streak-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"

# ----------------------------------------------------------------------------
# Mock gh: программируемые ответы через env-переменные
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
# strip-parsing: просто перечисляем аргументы в очищенный массив,
# убирая --repo X и подобные (значения флагов)
_args=("$@")
_clean=()
_skip=0
for a in "${_args[@]}"; do
    if [ "$_skip" = "1" ]; then _skip=0; continue; fi
    case "$a" in
        --repo|--label|--state|--limit|--json|--workflow|-w) _skip=1 ;;
        --assignee|--title|--body) _skip=1 ;;
        --jq|-q) _skip=1 ;;
    esac
    _clean+=("$a")
done

case "${_clean[0]} ${_clean[1]:-}" in
    "auth status")
        if [ "${MOCK_GH_AUTH_OK:-0}" = "0" ]; then
            echo "logged in"
            exit 0
        else
            echo "not logged in" >&2
            exit 1
        fi
        ;;
    "run list")
        if [ -n "${MOCK_RUNS_JSON_FILE:-}" ] && [ -f "${MOCK_RUNS_JSON_FILE}" ]; then
            cat "${MOCK_RUNS_JSON_FILE}"
        else
            echo '[]'
        fi
        exit 0
        ;;
    "issue list")
        if [ -n "${MOCK_OPEN_LABEL_JSON:-}" ] && [ -f "${MOCK_OPEN_LABEL_JSON}" ]; then
            cat "${MOCK_OPEN_LABEL_JSON}"
        else
            echo '[]'
        fi
        exit 0
        ;;
    "issue create")
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then
            echo "issue_create" >> "${MOCK_GH_CALL_LOG}"
        fi
        if [ -n "${MOCK_GH_ISSUE_CREATE_BODY_LOG:-}" ]; then
            for ((i=0; i<${#_args[@]}; i++)); do
                case "${_args[$i]}" in
                    --body) printf 'body<<%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --label) printf 'label=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --title) printf 'title=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --assignee) printf 'assignee=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                esac
            done
        fi
        rc="${MOCK_GH_ISSUE_CREATE_RC:-0}"
        if [ "$rc" = "0" ]; then
            echo "${MOCK_GH_ISSUE_CREATE_OUT:-https://github.com/krikz/rob_box_project/issues/9999}"
        else
            echo "ERROR: gh issue create simulated failure (rc=$rc)" >&2
        fi
        exit "$rc"
        ;;
    *) echo '{"error":"unmocked gh subcommand"}' >&2; exit 2 ;;
esac
EOF

# Mock git: возвращает заранее заданный HEAD и список merges
cat > "$WORK/bin/git" <<'EOF'
#!/bin/bash
case "${1:-}" in
    rev-parse)
        echo "${MOCK_DEVELOP_HEAD:-abcdef0}"
        exit 0
        ;;
    log)
        if [ -n "${MOCK_GIT_LOG_OUT:-}" ] && [ -f "${MOCK_GIT_LOG_OUT}" ]; then
            cat "${MOCK_GIT_LOG_OUT}"
        else
            echo ""
        fi
        exit 0
        ;;
    *) echo ""; exit 0 ;;
esac
EOF

chmod +x "$WORK/bin/gh" "$WORK/bin/git"

# ----------------------------------------------------------------------------
# Хелпер: запустить watchdog с mock-данными, поймать stderr/log/calls
#   $1 = label (для имени файлов лога/calls)
#   $2 = streak-runs-json file
#   $3 = open-label-json file
#   $4 = mock gh issue create rc
#   $5 = cooldown state: fresh|stale|absent
#   $6 = FAIL_STREAK_DRY_RUN value
#   $7 = E2E_FAIL_STREAK_ISSUE_ASSIGNEE (пусто = unset)
# Использует sub-shell-стиль: всё пишется в $WORK/log_<tag> и $WORK/calls_<tag>
# ----------------------------------------------------------------------------
run_watchdog() {
    local tag="$1"
    local mock_runs="$2"
    local mock_open="$3"
    local mock_create_rc="$4"
    local cooldown_state="$5"
    local dry_run="$6"
    local assignee="${7:-}"

    local cooldown="$WORK/cooldown_${tag}"
    local logf="$WORK/log_${tag}"
    local callf="$WORK/calls_${tag}"
    local bodyf="$WORK/body_${tag}"
    rm -f "$cooldown" "$logf" "$callf" "$bodyf"
    : > "$callf"    # pre-create so wc -l works even when no gh calls are made
    : > "$bodyf"
    case "$cooldown_state" in
        fresh) date -u +%s > "$cooldown" ;;
        # stale: устанавливаем mtime (а не содержимое) на 5 часов назад,
        # потому что watchdog читает `stat -c %Y` (mtime), а не содержимое файла
        stale)
            date -u +%s > "$cooldown"
            touch -d '5 hours ago' "$cooldown" 2>/dev/null || \
                touch -t "$(date -u -d '5 hours ago' +%Y%m%d%H%M.%S 2>/dev/null)" "$cooldown" 2>/dev/null || true
            ;;
        absent) rm -f "$cooldown" ;;
    esac

    # When no assignee, use a clean env chain. We don't want an empty
    # interpolation to cause bash to misinterpret the `\` continuation.
    if [ -n "$assignee" ]; then
        PATH="$WORK/bin:$PATH:/usr/bin:/bin" \
            GH_REPO="krikz/rob_box_project" \
            HERMES_HOME="$WORK/hermes" \
            E2E_FAIL_STREAK_LIMIT="30" \
            E2E_FAIL_STREAK_WARN="5" \
            E2E_FAIL_STREAK_PAUSE="20" \
            E2E_FAIL_STREAK_ISSUE_THRESHOLD="5" \
            E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS="4" \
            E2E_FAIL_STREAK_ISSUE_LABEL="e2e-fail-streak" \
            E2E_FAIL_STREAK_ISSUE_ASSIGNEE="$assignee" \
            FAIL_STREAK_DRY_RUN="$dry_run" \
            REPO_DIR="$WORK/repo" \
            MOCK_RUNS_JSON_FILE="$mock_runs" \
            MOCK_OPEN_LABEL_JSON="$mock_open" \
            MOCK_GH_ISSUE_CREATE_RC="$mock_create_rc" \
            MOCK_GH_ISSUE_CREATE_OUT="https://github.com/krikz/rob_box_project/issues/4242" \
            MOCK_GH_AUTH_OK="0" \
            MOCK_DEVELOP_HEAD="d17e107" \
            MOCK_GIT_LOG_OUT="" \
            MOCK_GH_CALL_LOG="$callf" \
            MOCK_GH_ISSUE_CREATE_BODY_LOG="$bodyf" \
            LOCK_FILE="$WORK/lock_${tag}" \
            ISSUE_COOLDOWN_FILE="$cooldown" \
            bash "$WATCHDOG_SH" >/dev/null 2>"$logf"
    else
        PATH="$WORK/bin:$PATH:/usr/bin:/bin" \
            GH_REPO="krikz/rob_box_project" \
            HERMES_HOME="$WORK/hermes" \
            E2E_FAIL_STREAK_LIMIT="30" \
            E2E_FAIL_STREAK_WARN="5" \
            E2E_FAIL_STREAK_PAUSE="20" \
            E2E_FAIL_STREAK_ISSUE_THRESHOLD="5" \
            E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS="4" \
            E2E_FAIL_STREAK_ISSUE_LABEL="e2e-fail-streak" \
            FAIL_STREAK_DRY_RUN="$dry_run" \
            REPO_DIR="$WORK/repo" \
            MOCK_RUNS_JSON_FILE="$mock_runs" \
            MOCK_OPEN_LABEL_JSON="$mock_open" \
            MOCK_GH_ISSUE_CREATE_RC="$mock_create_rc" \
            MOCK_GH_ISSUE_CREATE_OUT="https://github.com/krikz/rob_box_project/issues/4242" \
            MOCK_GH_AUTH_OK="0" \
            MOCK_DEVELOP_HEAD="d17e107" \
            MOCK_GIT_LOG_OUT="" \
            MOCK_GH_CALL_LOG="$callf" \
            MOCK_GH_ISSUE_CREATE_BODY_LOG="$bodyf" \
            LOCK_FILE="$WORK/lock_${tag}" \
            ISSUE_COOLDOWN_FILE="$cooldown" \
            bash "$WATCHDOG_SH" >/dev/null 2>"$logf"
    fi
    echo "rc=$?"
    echo "log=$logf"
    echo "calls=$callf"
    echo "body=$bodyf"
    echo "cooldown=$cooldown"
}

# Фикстуры runs-json: streak=3 / 5 / 8 (все failure, без success)
mk_runs() {
    python3 - "$1" > "$2" <<'PYEOF'
import json, sys
n = int(sys.argv[1])
runs = [{
    "databaseId": 34779000000 + i,
    "conclusion": "failure",
    "createdAt": f"2026-09-13T20:{i:02d}:00Z",
    "headBranch": "develop",
    "headSha": f"4ab3a0a{i:05d}",
    "name": "L: E2E Voice Test",
} for i in range(n)]
print(json.dumps(runs))
PYEOF
}

mk_runs 3 "$WORK/runs3.json"
mk_runs 5 "$WORK/runs5.json"
mk_runs 8 "$WORK/runs8.json"
echo '[]' > "$WORK/open_none.json"
echo '[{"number": 9999}]' > "$WORK/open_one.json"

# ----------------------------------------------------------------------------
# S1
# ----------------------------------------------------------------------------
echo "── S1: streak=3, no create expected ──"
run_watchdog s1 "$WORK/runs3.json" "$WORK/open_none.json" 0 absent true "" >/dev/null
n=$(wc -l < "$WORK/calls_s1")
tick_done=$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+$' "$WORK/log_s1" | tail -n1)
if [ "$n" = "0" ] && [[ "$tick_done" == *"tick done: streak=3 action=noop"* ]]; then
    echo "  PASS: 0 calls, tick done summary format=action=noop"
    PASS=$((PASS+1))
else
    echo "  FAIL: calls=$n, tick_done=${tick_done:-(none)}"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S2
# ----------------------------------------------------------------------------
echo "── S2: streak=5, DRY-RUN, would-log ──"
run_watchdog s2 "$WORK/runs5.json" "$WORK/open_none.json" 0 absent true "" >/dev/null
n=$(wc -l < "$WORK/calls_s2")
tick_done=$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+$' "$WORK/log_s2" | tail -n1)
if [ "$n" = "0" ] && grep -q "DRY-RUN would: gh issue create" "$WORK/log_s2" && [[ "$tick_done" == *"tick done: streak=5 action=issue-dry-run"* ]]; then
    echo "  PASS: DRY-RUN logged, no real call, tick done summary format=action=issue-dry-run"
    PASS=$((PASS+1))
else
    echo "  FAIL: calls=$n, dry-run-log=$(grep -c 'DRY-RUN would' "$WORK/log_s2"), tick_done=${tick_done:-(none)}"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S3
# ----------------------------------------------------------------------------
echo "── S3: streak=5, fresh cooldown, skip ──"
run_watchdog s3 "$WORK/runs5.json" "$WORK/open_none.json" 0 fresh false "" >/dev/null
n=$(wc -l < "$WORK/calls_s3")
if [ "$n" = "0" ] && grep -q "ISSUE_COOLDOWN active" "$WORK/log_s3"; then
    echo "  PASS: skipped due to fresh cooldown"
    PASS=$((PASS+1))
else
    echo "  FAIL: calls=$n, log-marker=$(grep -c ISSUE_COOLDOWN "$WORK/log_s3")"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S4
# ----------------------------------------------------------------------------
echo "── S4: streak=5, existing open issue, skip ──"
run_watchdog s4 "$WORK/runs5.json" "$WORK/open_one.json" 0 absent false "" >/dev/null
n=$(wc -l < "$WORK/calls_s4")
if [ "$n" = "0" ] && grep -q "open e2e-fail-streak issues: 1" "$WORK/log_s4"; then
    echo "  PASS: skipped due to existing open issue"
    PASS=$((PASS+1))
else
    echo "  FAIL: calls=$n, log-marker=$(grep -c 'open e2e-fail-streak' "$WORK/log_s4")"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S5
# ----------------------------------------------------------------------------
echo "── S5: streak=5, real create call, label/title/body check ──"
run_watchdog s5 "$WORK/runs5.json" "$WORK/open_none.json" 0 absent false "" >/dev/null
n=$(wc -l < "$WORK/calls_s5")
label_ok=$(grep -c '^label=e2e-fail-streak$' "$WORK/body_s5")
title_ok=$(grep -c '^title=\[e2e-fail-streak\]' "$WORK/body_s5")
body_music=$(grep -c 'music-fix regression' "$WORK/body_s5")
cooldown_written="no"
[ -f "$WORK/cooldown_s5" ] && cooldown_written="yes"
tick_done=$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+$' "$WORK/log_s5" | tail -n1)
if [ "$n" = "1" ] && [ "$label_ok" -ge 1 ] && [ "$title_ok" -ge 1 ] && [ "$body_music" -ge 1 ] && [ "$cooldown_written" = "yes" ] && [[ "$tick_done" == *"tick done: streak=5 action=issue-created"* ]]; then
    echo "  PASS: 1 create call, label/title/body correct, cooldown written, tick done summary format=action=issue-created"
    PASS=$((PASS+1))
else
    echo "  FAIL: n=$n label=$label_ok title=$title_ok body_music=$body_music cooldown=$cooldown_written tick_done=${tick_done:-(none)}"
    head -10 "$WORK/body_s5" | sed 's/^/    | /'
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S6: acceptance — 8 fails → ровно 1 issue
# ----------------------------------------------------------------------------
echo "── S6: streak=8 → exactly 1 issue ──"
run_watchdog s6 "$WORK/runs8.json" "$WORK/open_none.json" 0 absent false "" >/dev/null
n=$(wc -l < "$WORK/calls_s6")
if [ "$n" = "1" ]; then
    echo "  PASS: exactly 1 create call for 8-fail streak"
    PASS=$((PASS+1))
else
    echo "  FAIL: $n calls (expected 1)"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S7
# ----------------------------------------------------------------------------
echo "── S7: cooldown stale → NOT skipped ──"
run_watchdog s7 "$WORK/runs5.json" "$WORK/open_none.json" 0 stale false "" >/dev/null
n=$(wc -l < "$WORK/calls_s7")
if [ "$n" = "1" ]; then
    echo "  PASS: stale cooldown did NOT skip"
    PASS=$((PASS+1))
else
    echo "  FAIL: $n calls (expected 1)"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S8
# ----------------------------------------------------------------------------
echo "── S8: gh issue create fails → exit 0, cooldown NOT written ──"
PATH="$WORK/bin:/usr/bin:/bin" \
    GH_REPO="krikz/rob_box_project" \
    HERMES_HOME="$WORK/hermes" \
    E2E_FAIL_STREAK_LIMIT=30 \
    E2E_FAIL_STREAK_WARN=5 \
    E2E_FAIL_STREAK_PAUSE=20 \
    E2E_FAIL_STREAK_ISSUE_THRESHOLD=5 \
    E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS=4 \
    E2E_FAIL_STREAK_ISSUE_LABEL=e2e-fail-streak \
    FAIL_STREAK_DRY_RUN=false \
    REPO_DIR="$WORK/repo" \
    MOCK_RUNS_JSON_FILE="$WORK/runs5.json" \
    MOCK_OPEN_LABEL_JSON="$WORK/open_none.json" \
    MOCK_GH_ISSUE_CREATE_RC=1 \
    MOCK_GH_AUTH_OK=0 \
    MOCK_DEVELOP_HEAD=d17e107 \
    MOCK_GIT_LOG_OUT='' \
    MOCK_GH_CALL_LOG='' \
    MOCK_GH_ISSUE_CREATE_BODY_LOG='' \
    LOCK_FILE="$WORK/lock_s8b" \
    ISSUE_COOLDOWN_FILE="$WORK/cooldown_s8b" \
    bash "$WATCHDOG_SH" >/dev/null 2>"$WORK/log_s8b"
rc_line=$?
cooldown_written="no"
[ -f "$WORK/cooldown_s8b" ] && cooldown_written="yes"
tick_done=$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+$' "$WORK/log_s8b" | tail -n1)
if [ "$rc_line" = "0" ] && [ "$cooldown_written" = "no" ] && [[ "$tick_done" == *"tick done: streak=5 action=noop"* ]]; then
    echo "  PASS: rc=0, cooldown NOT written, tick done summary format=action=noop"
    PASS=$((PASS+1))
else
    echo "  FAIL: rc=$rc_line cooldown=$cooldown_written tick_done=${tick_done:-(none)}"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# S9
# ----------------------------------------------------------------------------
echo "── S9: E2E_FAIL_STREAK_ISSUE_ASSIGNEE=krikz → passed ──"
run_watchdog s9 "$WORK/runs5.json" "$WORK/open_none.json" 0 absent false krikz >/dev/null
assignee_ok=$(grep -c '^assignee=krikz$' "$WORK/body_s9")
if [ "$assignee_ok" -ge 1 ]; then
    echo "  PASS: assignee passed"
    PASS=$((PASS+1))
else
    echo "  FAIL: assignee not found"
    FAIL=$((FAIL+1))
fi

# ----------------------------------------------------------------------------
# Sanity: marker в body
# ----------------------------------------------------------------------------
echo "── Sanity: marker in body ──"
if grep -q '🤖 \[agent:devops\] script=agent-flow-e2e-fail-streak-watchdog action=auto-create-issue' "$WORK/body_s5"; then
    echo "  PASS: marker present"
    PASS=$((PASS+1))
else
    echo "  FAIL: marker missing"
    FAIL=$((FAIL+1))
fi

echo ""
echo "=== Total: $((PASS+FAIL)) / Passed: $PASS / Failed: $FAIL ==="
[ "$FAIL" = "0" ]