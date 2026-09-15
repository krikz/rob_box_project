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
# strip-parsing: перечисляем аргументы в очищенный массив _clean[],
# убирая флаги-принимающие-значение (--repo X, --label Y, ...) И их значения.
# FIX (R9, issue #2483): предыдущий вариант всегда делал `_clean+=("$a")`
# даже когда _skip=1, поэтому флаг И его значение попадали в _clean[].
# Случайно работало, потому что gh-аргументы идут в правильном порядке и
# `issue create` всё равно парсил "issue" + всё остальное как "create" без args.
# Ломалось бы на любом рефакторе (например, перенос `issue` между слотами).
#
# Новый вариант: accumulator-pattern — добавляем в _clean ТОЛЬКО когда
# не нужно skip'ить ни текущий флаг, ни следующий за ним value.
_args=("$@")
_clean=()
_skip_next=0
for a in "${_args[@]}"; do
    if [ "$_skip_next" = "1" ]; then
        # Значение предыдущего флага (--repo X, --label Y, ...) — skip + reset.
        _skip_next=0
        continue
    fi
    case "$a" in
        --repo|--label|--state|--limit|--json|--workflow|-w) _skip_next=1 ;;
        --assignee|--title|--body) _skip_next=1 ;;
        --jq|-q) _skip_next=1 ;;
        *) _clean+=("$a") ;;
    esac
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
    "issue comment")
        # FIX (R12, issue #2483): mock для `gh issue comment` — раньше
        # падал с "unmocked gh subcommand", поэтому alert-ветка watchdog
        # (lines 263-273 a9b04981) никогда не проверялась в тестах.
        # Возвращаем mock success; в MOCK_GH_CALL_LOG пишем `comment:<num>`.
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then
            # Извлечь issue number (первый позиционный после subcommand).
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
#   $3 = open-label-json file (для gh issue list --label e2e-fail-streak)
#   $4 = mock gh issue create rc
#   $5 = cooldown state: fresh|stale|absent
#   $6 = FAIL_STREAK_DRY_RUN value
#   $7 = E2E_FAIL_STREAK_ISSUE_ASSIGNEE (пусто = unset)
#   $8 = mock-needs-e2e-json file (для gh issue list --label needs-e2e, пусто = [])
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
    local mock_needs_e2e="${8:-}"

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

    # MOCK_OPEN_NEEDS_E2E_JSON пробрасывается в обоих ветках: пустое значение
    # = mock-gh вернёт '[]' (needs-e2e pool пустой). Делаем две ветки вместо
    # одной с опциональной строкой, потому что пустая строка в env-chain
    # ломает `\` continuation в bash.
    if [ -n "$assignee" ]; then
        if [ -n "$mock_needs_e2e" ]; then
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
                MOCK_OPEN_NEEDS_E2E_JSON="$mock_needs_e2e" \
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
                E2E_FAIL_STREAK_ISSUE_ASSIGNEE="$assignee" \
                FAIL_STREAK_DRY_RUN="$dry_run" \
                REPO_DIR="$WORK/repo" \
                MOCK_RUNS_JSON_FILE="$mock_runs" \
                MOCK_OPEN_LABEL_JSON="$mock_open" \
                MOCK_OPEN_NEEDS_E2E_JSON="" \
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
    else
        if [ -n "$mock_needs_e2e" ]; then
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
                MOCK_OPEN_NEEDS_E2E_JSON="$mock_needs_e2e" \
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
                MOCK_OPEN_NEEDS_E2E_JSON="" \
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
# FIX (R12, issue #2483): после появления mock-ветки `gh issue comment` (alert
# в needs-e2e issues, lines 231-274 watchdog) calls_s4 может содержать
# `issue_comment:N` записи — это ВАЛИДНОЕ production-поведение (alert-comment
# срабатывает независимо от auto-create-ветки). Мы проверяем только, что
# `gh issue create` НЕ был вызван (auto-create skip по gh-truth guard).
# ----------------------------------------------------------------------------
echo "── S4: streak=5, existing open issue, skip ──"
run_watchdog s4 "$WORK/runs5.json" "$WORK/open_one.json" 0 absent false "" >/dev/null
create_n=$(grep -c '^issue_create$' "$WORK/calls_s4" || true)
comment_n=$(grep -c '^issue_comment:' "$WORK/calls_s4" || true)
if [ "$create_n" = "0" ] && grep -q "open e2e-fail-streak issues: 1" "$WORK/log_s4"; then
    echo "  PASS: create=0, alert-comments=$comment_n, gh-truth skipped create"
    PASS=$((PASS+1))
else
    echo "  FAIL: create=$create_n comments=$comment_n, log-marker=$(grep -c 'open e2e-fail-streak' "$WORK/log_s4")"
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
# FIX (R14, issue #2483): S8 раньше делал `bash "$WATCHDOG_SH" >/dev/null 2>&1`
# без capture — если watchdog логирует ERROR в stderr, S8 silently swallows.
# Нет способа отладить S8 failure. Теперь прогоняем через run_watchdog (тот
# же helper, что S1-S7/S9) с tag=s8, и читаем log_s8 в финальном assert.
# Также (R12, issue #2483): этот тест проверяет что production watchdog
# при gh-fail сделал ОДИН `gh issue create` вызов (rc=1 → mock fail),
# cooldown НЕ записан, alert-ветка `gh issue comment` НЕ активна (потому
# что нет needs-e2e issues в open_none.json). Раньше тест only-проверял
# cooldown file → green даже при сломанном alert-сценарии.
# ----------------------------------------------------------------------------
echo "── S8: gh issue create fails → exit 0, cooldown NOT written, log captured ──"
run_watchdog s8 "$WORK/runs5.json" "$WORK/open_none.json" 1 absent false "" >/dev/null
cooldown_written="no"
[ -f "$WORK/cooldown_s8" ] && cooldown_written="yes"
log_nonempty="no"
[ -s "$WORK/log_s8" ] && log_nonempty="yes"
create_n=$(grep -c '^issue_create$' "$WORK/calls_s8" || true)
comment_n=$(grep -c '^issue_comment:' "$WORK/calls_s8" || true)
gh_fail_in_log="no"
grep -q "gh issue create failed" "$WORK/log_s8" && gh_fail_in_log="yes"
if [ "$cooldown_written" = "no" ] && [ "$log_nonempty" = "yes" ] \
    && [ "$create_n" = "1" ] && [ "$comment_n" = "0" ] \
    && [ "$gh_fail_in_log" = "yes" ]; then
    echo "  PASS: create=1 (rc=1), alert-comments=0, cooldown NOT written, gh-fail logged"
    PASS=$((PASS+1))
else
    echo "  FAIL: cooldown=$cooldown_written log_nonempty=$log_nonempty create=$create_n comments=$comment_n gh_fail_logged=$gh_fail_in_log"
    echo "  --- log_s8 ---"
    cat "$WORK/log_s8" | head -5 | sed 's/^/    | /'
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
# S10 (R12, issue #2483): явный сценарий для alert-comment ветки.
# До R12 mock `gh issue comment` падал с "unmocked gh subcommand" — alert-ветка
# (lines 231-274 watchdog) была немой: если production ломал alert-comment,
# S4 падал с unmocked gh. Этот тест: streak=5 + needs-e2e issue в пуле +
# никакого existing e2e-fail-streak → alert-comment ВЫЗВАН (1 вызов) +
# auto-create тоже ВЫЗВАН (1 вызов) + cooldown записан + оба log-marker'а есть.
# ----------------------------------------------------------------------------
echo "── S10: streak=5 + needs-e2e → alert-comment + auto-create ──"
# open_one.json содержит 1 issue — watchdog найдёт его как needs-e2e pool
# (mock-gh возвращает его для gh issue list --label needs-e2e И для --label e2e-fail-streak).
# Чтобы alert-comment СРАБОТАЛ, нам нужно, чтобы e2e-fail-streak list вернул 0
# (иначе skip create). Перезапишем open_one.json → только для alert-ветки
# нам нужен отдельный сценарий: alert ВЫЗВАН, но auto-create SKIP'нут.
# Используем два прогона:
#   - S10a: alert + create (open_e2e.json=[] ; open_label.json=[needs-e2e])
#   - S10b: alert + skip-create (open_e2e.json=[e2e] ; open_label.json=[needs-e2e])
echo '[]' > "$WORK/open_alert_target.json"
echo '[{"number": 42, "title": "test-needs-e2e-issue"}]' > "$WORK/open_needs_e2e.json"
# gh issue list --label needs-e2e → open_needs_e2e.json (1 issue)
# gh issue list --label e2e-fail-streak → open_alert_target.json (0 issue → create allowed)
# Fallback pool — пустой (т.к. _n_count > 0).

# Нужно подменить mock-gh так чтобы он возвращал разный json для разных --label.
# Сделаем это через MOCK_OPEN_LABEL_JSON_BYPASS: добавим в mock-gh маршрутизацию
# по --label значению. Перегенерим mock-gh для S10.
cat > "$WORK/bin/gh" <<'GH_EOF'
#!/bin/bash
_args=("$@")
_clean=()
_skip_next=0
for a in "${_args[@]}"; do
    if [ "$_skip_next" = "1" ]; then _skip_next=0; continue; fi
    case "$a" in
        --repo|--label|--state|--limit|--json|--workflow|-w) _skip_next=1 ;;
        --assignee|--title|--body) _skip_next=1 ;;
        --jq|-q) _skip_next=1 ;;
        *) _clean+=("$a") ;;
    esac
done

case "${_clean[0]} ${_clean[1]:-}" in
    "auth status")
        if [ "${MOCK_GH_AUTH_OK:-0}" = "0" ]; then echo "logged in"; exit 0; fi
        echo "not logged in" >&2; exit 1 ;;
    "run list")
        if [ -n "${MOCK_RUNS_JSON_FILE:-}" ] && [ -f "${MOCK_RUNS_JSON_FILE}" ]; then cat "${MOCK_RUNS_JSON_FILE}"; else echo '[]'; fi
        exit 0 ;;
    "issue list")
        # Роутинг по --label
        _label=""
        for ((i=0; i<${#_args[@]}; i++)); do
            if [ "${_args[$i]}" = "--label" ]; then _label="${_args[$((i+1))]}"; break; fi
        done
        case "$_label" in
            needs-e2e)
                if [ -n "${MOCK_OPEN_NEEDS_E2E_JSON:-}" ] && [ -f "${MOCK_OPEN_NEEDS_E2E_JSON}" ]; then
                    cat "${MOCK_OPEN_NEEDS_E2E_JSON}"
                else echo '[]'; fi
                ;;
            e2e-fail-streak)
                if [ -n "${MOCK_OPEN_LABEL_JSON:-}" ] && [ -f "${MOCK_OPEN_LABEL_JSON}" ]; then
                    cat "${MOCK_OPEN_LABEL_JSON}"
                else echo '[]'; fi
                ;;
            *) echo '[]' ;;
        esac
        exit 0 ;;
    "issue create")
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then echo "issue_create" >> "${MOCK_GH_CALL_LOG}"; fi
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
        if [ "$rc" = "0" ]; then echo "${MOCK_GH_ISSUE_CREATE_OUT:-https://github.com/krikz/rob_box_project/issues/9999}"; else echo "ERROR: gh issue create simulated failure (rc=$rc)" >&2; fi
        exit "$rc" ;;
    "issue comment")
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then
            for ((i=0; i<${#_args[@]}; i++)); do
                case "${_args[$i]}" in
                    --*) ;;
                    *) echo "issue_comment:${_args[$i]}" >> "${MOCK_GH_CALL_LOG}"; break ;;
                esac
            done
        fi
        rc="${MOCK_GH_ISSUE_COMMENT_RC:-0}"
        if [ "$rc" != "0" ]; then echo "ERROR: gh issue comment simulated failure (rc=$rc)" >&2; fi
        exit "$rc" ;;
    *) echo '{"error":"unmocked gh subcommand"}' >&2; exit 2 ;;
esac
GH_EOF
chmod +x "$WORK/bin/gh"

# S10a: alert-comment + auto-create (оба срабатывают)
# mock_open = e2e-fail-streak list → [] (0 → create allowed)
# mock_needs_e2e = needs-e2e list → 1 issue → alert-comment ВЫЗВАН
run_watchdog s10a "$WORK/runs5.json" "$WORK/open_alert_target.json" 0 absent false "" "$WORK/open_needs_e2e.json" >/dev/null
create_n=$(grep -c '^issue_create$' "$WORK/calls_s10a" || true)
comment_n=$(grep -c '^issue_comment:' "$WORK/calls_s10a" || true)
cooldown_written="no"
[ -f "$WORK/cooldown_s10a" ] && cooldown_written="yes"
needs_e2e_logged=$(grep -c 'open needs-e2e issues: 1' "$WORK/log_s10a" || true)
alert_posted_logged=$(grep -c 'issue #42: alert posted' "$WORK/log_s10a" || true)
if [ "$create_n" = "1" ] && [ "$comment_n" = "1" ] && [ "$cooldown_written" = "yes" ] \
    && [ "$needs_e2e_logged" = "1" ] && [ "$alert_posted_logged" = "1" ]; then
    echo "  PASS: 1 alert-comment + 1 auto-create + cooldown written"
    PASS=$((PASS+1))
else
    echo "  FAIL: create=$create_n comment=$comment_n cooldown=$cooldown_written needs_e2e=$needs_e2e_logged alert_posted=$alert_posted_logged"
    head -10 "$WORK/log_s10a" | sed 's/^/    | /'
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