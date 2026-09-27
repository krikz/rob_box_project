#!/bin/bash
# ============================================================================
# test_e2e_fail_streak_flaky_detect.sh — регресс-тест для flaky-detect ветки
# в agent-flow-e2e-fail-streak-watchdog.sh (issue t_f33ecbf8).
#
# Контекст: при streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD (5) watchdog должен
# ПЕРЕД auto-create e2e-fail-streak issue решить, это РЕАЛЬНАЯ регрессия
# (fails на разных headSha) или FLAKY (≥ FLAKY_DETECT_MIN fails на ОДНОМ headSha
# с ratio ≥ FLAKY_DETECT_RATIO). В flaky-ветке auto-create подавляется, и
# пишется [flaky-detect] marker-комментарий (24h dedup).
#
# Scenarios (PATH-hijack mock-gh / mock-git, без сети):
#   S1. streak=5, 5 fails на ОДНОМ headSha → NO gh issue create, flaky log
#       emitted (или comment если open fail-streak issue есть).
#   S2. streak=5, fails на РАЗНЫХ headSha → gh issue create (regression path,
#       существующая логика).
#   S3. streak=3 (< FLAKY_DETECT_THRESHOLD) → no action.
#   S4. streak=5 flaky + FLAKY_DEDUP_FILE mtime=now → dedup skip, NO gh issue
#       create, NO flaky comment, log "FLAKY_DEDUP active".
#   S5. streak=5 flaky-border (3 same + 2 diff sha, ratio=0.6) → flaky marker
#       (граница проходит).
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_fail_streak_flaky_detect.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-e2e-fail-streak-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null   || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"

# ----------------------------------------------------------------------------
# Mock gh: программируемые ответы через env-переменные. Минимально
# достаточный для watchdog: auth status, run list, issue list (с label),
# issue list --label, issue create, issue comment.
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
_args=("$@")
_clean=()
_skip_next=0
for a in "${_args[@]}"; do
    if [ "$_skip_next" = "1" ]; then
        _skip_next=0
        continue
    fi
    case "$a" in
        --repo|--label|--state|--limit|--json|--workflow|-w) _skip_next=1 ;;
        --assignee|--title|--body) _skip_next=1 ;;
        --jq|-q|--per_page|--branch) _skip_next=1 ;;
        *) _clean+=("$a") ;;
    esac
done

case "${_clean[0]} ${_clean[1]:-}" in
    "auth status")
        if [ "${MOCK_GH_AUTH_OK:-0}" = "0" ]; then
            echo "logged in"; exit 0
        else
            echo "not logged in" >&2; exit 1
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
        # Если label был e2e-fail-streak — отдаём MOCK_OPEN_LABEL_JSON.
        # В watchdog есть несколько `gh issue list --label X` вызовов; мы
        # упрощаем: отдаём MOCK_OPEN_LABEL_JSON для ЛЮБОГО label, если он
        # задан. Это корректно для нашего теста — мы контролируем оба.
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
        rc="${MOCK_GH_ISSUE_CREATE_RC:-0}"
        if [ "$rc" = "0" ]; then
            echo "${MOCK_GH_ISSUE_CREATE_OUT:-https://github.com/krikz/rob_box_project/issues/9999}"
        else
            echo "ERROR: gh issue create simulated failure (rc=$rc)" >&2
        fi
        exit "$rc"
        ;;
    "issue comment")
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then
            for ((i=0; i<${#_args[@]}; i++)); do
                case "${_args[$i]}" in
                    --*) ;;
                    *)
                        echo "issue_comment:${_args[$i]}" >> "${MOCK_GH_CALL_LOG}"
                        break
                        ;;
                esac
            done
        fi
        rc="${MOCK_GH_ISSUE_COMMENT_RC:-0}"
        if [ "$rc" != "0" ]; then
            echo "ERROR: gh issue comment simulated failure (rc=$rc)" >&2
        fi
        exit "$rc"
        ;;
    "api")
        # watchdog НЕ использует `gh api` напрямую в flaky-ветке — заглушка.
        echo '{}'; exit 0
        ;;
    *) echo '{"error":"unmocked gh subcommand"}' >&2; exit 2 ;;
esac
EOF

cat > "$WORK/bin/git" <<'EOF'
#!/bin/bash
case "${1:-}" in
    rev-parse) echo "${MOCK_DEVELOP_HEAD:-abcdef0}"; exit 0 ;;
    log)
        if [ -n "${MOCK_GIT_LOG_OUT:-}" ] && [ -f "${MOCK_GIT_LOG_OUT}" ]; then
            cat "${MOCK_GIT_LOG_OUT}"
        else
            echo ""
        fi
        exit 0
        ;;
    show) exit 1 ;;  # MAINTENANCE gate check: not present
    *) echo ""; exit 0 ;;
esac
EOF

# flock wrapper, чтобы mock-gh через наш PATH не зависел от системы.
cat > "$WORK/bin/flock" <<'EOF'
#!/bin/bash
exit 0
EOF
chmod +x "$WORK/bin/gh" "$WORK/bin/git" "$WORK/bin/flock"

# ----------------------------------------------------------------------------
# Helper: запустить watchdog с mock-данными
#   $1 = label (для имени файлов логов)
#   $2 = streak-runs-json file
#   $3 = open-fail-streak-issue-json file ('[]' или '[{"number":N}]')
#   $4 = cooldown state: fresh|stale|absent
#   $5 = flaky-dedup state:  fresh|stale|absent
#   $6 = FAIL_STREAK_DRY_RUN value
# ----------------------------------------------------------------------------
run_watchdog() {
    local tag="$1"
    local mock_runs="$2"
    local mock_open="$3"
    local cooldown_state="$4"
    local flaky_dedup_state="$5"
    local dry_run="$6"

    local cooldown="$WORK/cooldown_${tag}"
    local fdedup="$WORK/fdedup_${tag}"
    local logf="$WORK/log_${tag}"
    local callf="$WORK/calls_${tag}"
    rm -f "$cooldown" "$fdedup" "$logf" "$callf"

    case "$cooldown_state" in
        fresh) date -u +%s > "$cooldown" ;;
        stale)
            date -u +%s > "$cooldown"
            touch -d '5 hours ago' "$cooldown" 2>/dev/null || true
            ;;
        absent) : ;;
    esac
    case "$flaky_dedup_state" in
        fresh) date -u +%s > "$fdedup" ;;
        stale)
            date -u +%s > "$fdedup"
            touch -d '25 hours ago' "$fdedup" 2>/dev/null || true
            ;;
        absent) : ;;
    esac

    : > "$callf"
    PATH="$WORK/bin:$PATH:/usr/bin:/bin" \
        GH_REPO="krikz/rob_box_project" \
        HERMES_HOME="$WORK/hermes" \
        E2E_FAIL_STREAK_LIMIT="30" \
        E2E_FAIL_STREAK_WARN="5" \
        E2E_FAIL_STREAK_PAUSE="20" \
        E2E_FAIL_STREAK_ISSUE_THRESHOLD="5" \
        E2E_FAIL_STREAK_ISSUE_RATE_LIMIT_HOURS="4" \
        E2E_FAIL_STREAK_ISSUE_LABEL="e2e-fail-streak" \
        E2E_FLAKY_DETECT_MIN="3" \
        E2E_FLAKY_DETECT_RATIO="0.6" \
        E2E_FLAKY_DEDUP_HOURS="24" \
        FLAKY_DETECT_LABEL="e2e:flaky-detection" \
        FAIL_STREAK_DRY_RUN="$dry_run" \
        MOCK_RUNS_JSON_FILE="$mock_runs" \
        MOCK_OPEN_LABEL_JSON="$mock_open" \
        MOCK_GH_CALL_LOG="$callf" \
        MOCK_GH_AUTH_OK="0" \
        MOCK_DEVELOP_HEAD="d17e107" \
        LOCK_FILE="$WORK/lock_${tag}" \
        ISSUE_COOLDOWN_FILE="$cooldown" \
        FLAKY_DEDUP_FILE="$fdedup" \
        bash "$WATCHDOG_SH" >/dev/null 2>"$logf"
    local rc=$?
    echo "$rc" > "$WORK/rc_${tag}"
}

# Фикстуры runs-json: head_sha параметризуется через аргументы.
mk_runs() {
    # $1 = streak (число), $2 = output file, $3 = sha (одинаковый для всех)
    local n="$1" out="$2" sha="$3"
    python3 - "$n" "$out" "$sha" >/dev/null <<'PYEOF'
import json, sys
n = int(sys.argv[1])
out = sys.argv[2]
sha = sys.argv[3]
runs = [{
    "databaseId": 34779000000 + i,
    "conclusion": "failure",
    "createdAt": f"2026-09-27T1{i%9}:{i:02d}:00Z",
    "headBranch": "develop",
    "headSha": sha,
    "name": "L: E2E Voice Test",
} for i in range(n)]
with open(out, "w") as f:
    f.write(json.dumps(runs))
PYEOF
}

# mk_runs_diff_shas: 5 fails на разных sha (для regression-сценария).
mk_runs_diff_shas() {
    local out="$1"
    python3 - "$out" >/dev/null <<'PYEOF'
import json, sys
out = sys.argv[1]
runs = [{
    "databaseId": 34779000000 + i,
    "conclusion": "failure",
    "createdAt": f"2026-09-27T1{i%9}:{i:02d}:00Z",
    "headBranch": "develop",
    "headSha": f"{i:03x}aaaa",
    "name": "L: E2E Voice Test",
} for i in range(5)]
with open(out, "w") as f:
    f.write(json.dumps(runs))
PYEOF
}

# mk_runs_flaky_border: 3 fails sha=ecfc454b + 2 fails на других sha.
mk_runs_flaky_border() {
    local out="$1"
    python3 - "$out" >/dev/null <<'PYEOF'
import json, sys
out = sys.argv[1]
runs = [
    {"databaseId": 34779000001, "conclusion": "failure", "createdAt": "2026-09-27T10:00:00Z",
     "headBranch": "develop", "headSha": "ecfc454b1f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d", "name": "L: E2E Voice Test"},
    {"databaseId": 34779000002, "conclusion": "failure", "createdAt": "2026-09-27T09:30:00Z",
     "headBranch": "develop", "headSha": "diff-sha-1aaaa", "name": "L: E2E Voice Test"},
    {"databaseId": 34779000003, "conclusion": "failure", "createdAt": "2026-09-27T09:00:00Z",
     "headBranch": "develop", "headSha": "ecfc454b1f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d", "name": "L: E2E Voice Test"},
    {"databaseId": 34779000004, "conclusion": "failure", "createdAt": "2026-09-27T08:30:00Z",
     "headBranch": "develop", "headSha": "diff-sha-2aaaa", "name": "L: E2E Voice Test"},
    {"databaseId": 34779000005, "conclusion": "failure", "createdAt": "2026-09-27T08:00:00Z",
     "headBranch": "develop", "headSha": "ecfc454b1f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d", "name": "L: E2E Voice Test"},
]
with open(out, "w") as f:
    f.write(json.dumps(runs))
PYEOF
}

# S1: 5 fails все на одном sha + streak >= threshold → FLAKY (NO issue_create)
echo "── S1: streak=5, 5 fails same sha → flaky (NO issue_create) ──"
mk_runs 5 "$WORK/runs_s1.json" "ecfc454b1f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d"
echo '[]' > "$WORK/open_none.json"  # no open fail-streak issue → log-only
run_watchdog s1 "$WORK/runs_s1.json" "$WORK/open_none.json" absent absent "true"
rc_s1="$(cat "$WORK/rc_s1")"
n_create="$(grep -c '^issue_create$' "$WORK/calls_s1" || true)"
# shellcheck disable=SC2034  # n_comment диагностический; не влияет на S1 PASS-condition
n_comment="$(grep -c '^issue_comment:' "$WORK/calls_s1" || true)"
flaky_log="$(grep -c 'flaky-detect:.*same-headsha=' "$WORK/log_s1" || true)"
action="$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+' "$WORK/log_s1" | tail -n1 || true)"

if [ "$rc_s1" = "0" ] && [ "$n_create" = "0" ] && [ "$flaky_log" -ge 1 ] \
    && [[ "$action" == *"action=flaky-skip"* ]]; then
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok1="S1 PASS: no issue_create, flaky log present, action=flaky-skip"
else
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok1="S1 FAIL: rc=$rc_s1, n_create=$n_create, flaky_log=$flaky_log, action=${action:-(none)}"
fi

# S2: 5 fails на РАЗНЫХ sha → REGRESSION → issue_create вызван.
# dry_run=false чтобы mock-gh issue_create действительно сработал
# (с dry_run=true ветка просто логирует 'DRY-RUN would').
echo "── S2: streak=5, 5 fails different shas → regression (issue_create) ──"
mk_runs_diff_shas "$WORK/runs_s2.json"
echo '[]' > "$WORK/open_none.json"
run_watchdog s2 "$WORK/runs_s2.json" "$WORK/open_none.json" absent absent "false"
rc_s2="$(cat "$WORK/rc_s2")"
n_create_s2="$(grep -c '^issue_create$' "$WORK/calls_s2" || true)"
regression_log="$(grep -c 'flaky-detect:.*decision=regression' "$WORK/log_s2" || true)"

if [ "$rc_s2" = "0" ] && [ "$n_create_s2" = "1" ] && [ "$regression_log" -ge 1 ]; then
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok2="S2 PASS: 1 issue_create, decision=regression logged"
else
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok2="S2 FAIL: rc=$rc_s2, n_create=$n_create_s2, regression_log=$regression_log"
fi

# S3: streak=3 (< threshold) → no action
echo "── S3: streak=3 < threshold → no action ──"
mk_runs 3 "$WORK/runs_s3.json" "ecfc454b1f9d6c1b2e8f3a4c5b7e8d1c2a3b4c5d"
echo '[]' > "$WORK/open_none.json"
run_watchdog s3 "$WORK/runs_s3.json" "$WORK/open_none.json" absent absent "true"
rc_s3="$(cat "$WORK/rc_s3")"
n_create_s3="$(grep -c '^issue_create$' "$WORK/calls_s3" || true)"
action_s3="$(grep -E 'tick done: streak=[0-9]+ action=[a-z-]+' "$WORK/log_s3" | tail -n1 || true)"

if [ "$rc_s3" = "0" ] && [ "$n_create_s3" = "0" ] \
    && [[ "$action_s3" == *"action=noop"* ]]; then
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok3="S3 PASS: no issue_create, action=noop (streak < threshold)"
else
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok3="S3 FAIL: rc=$rc_s3, n_create=$n_create_s3, action=${action_s3:-(none)}"
fi

# S4: streak=5 flaky + FLAKY_DEDUP_FILE fresh → dedup skip, NO issue_create,
#     NO issue_comment, log "FLAKY_DEDUP active".
echo "── S4: streak=5 flaky + flaky-dedup FRESH → dedup skip ──"
echo '[{"number": 7777}]' > "$WORK/open_one.json"  # open issue для flaky
run_watchdog s4 "$WORK/runs_s1.json" "$WORK/open_one.json" absent fresh "false"
rc_s4="$(cat "$WORK/rc_s4")"
n_create_s4="$(grep -c '^issue_create$' "$WORK/calls_s4" || true)"
n_comment_s4="$(grep -c '^issue_comment:7777$' "$WORK/calls_s4" || true)"
dedup_active="$(grep -c 'FLAKY_DEDUP active' "$WORK/log_s4" || true)"

if [ "$rc_s4" = "0" ] && [ "$n_create_s4" = "0" ] && [ "$n_comment_s4" = "0" ] \
    && [ "$dedup_active" = "1" ]; then
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok4="S4 PASS: dedup active, no issue_create, no issue_comment"
else
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok4="S4 FAIL: rc=$rc_s4, n_create=$n_create_s4, n_comment=$n_comment_s4, dedup_active=$dedup_active"
fi

# S5: streak=5 flaky-border (3 same + 2 different sha, ratio=0.6) → flaky marker
echo "── S5: streak=5 flaky-border (3+2) → flaky (ratio=0.6 граница) ──"
mk_runs_flaky_border "$WORK/runs_s5.json"
run_watchdog s5 "$WORK/runs_s5.json" "$WORK/open_one.json" absent absent "true"
rc_s5="$(cat "$WORK/rc_s5")"
n_create_s5="$(grep -c '^issue_create$' "$WORK/calls_s5" || true)"
flaky_log_s5="$(grep -c 'flaky-detect:.*same-headsha=3/' "$WORK/log_s5" || true)"

if [ "$rc_s5" = "0" ] && [ "$n_create_s5" = "0" ] && [ "$flaky_log_s5" = "1" ]; then
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok5="S5 PASS: 3/5 ratio=0.6 → flaky (no issue_create, decision=flaky)"
else
    # shellcheck disable=SC2034  # читается через ${!label} в summary
    ok5="S5 FAIL: rc=$rc_s5, n_create=$n_create_s5, flaky_log=$flaky_log_s5"
fi

# ----------------------------------------------------------------------------
# T6 sanity: shellcheck-clean (если есть)
# ----------------------------------------------------------------------------
shellcheck_clean() {
    if ! command -v shellcheck >/dev/null 2>&1; then
        return 0
    fi
    shellcheck -x "$WATCHDOG_SH" >"$WORK/shellcheck.out" 2>&1
}

# --- summary ---------------------------------------------------------------
PASS=0; FAIL=0
for label in ok1 ok2 ok3 ok4 ok5; do
    val="${!label:-}"
    case "$val" in
        *PASS*) echo "  ✓ ${val#*PASS: }"; PASS=$((PASS+1)) ;;
        *FAIL*) echo "  ✗ ${val#*FAIL: }"; FAIL=$((FAIL+1)) ;;
        *)      echo "  ? ${label} missing"; FAIL=$((FAIL+1)) ;;
    esac
done

if shellcheck_clean; then
    echo "  ✓ shellcheck clean (или shellcheck недоступен — пропущен)"
else
    echo "  ✗ shellcheck found issues — см. $WORK/shellcheck.out"
    FAIL=$((FAIL+1))
fi

echo
printf 'PASS=%s FAIL=%s\n' "$PASS" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
printf '\n=== test_e2e_fail_streak_flaky_detect.sh: PASS ===\n'
exit 0
