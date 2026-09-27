#!/bin/bash
# ============================================================================
# test_orphan_needs_e2e_watchdog.sh — orphan needs-e2e sweep guard (ретро t_78a6ffa3)
#
# Регресс-гард для agent-flow-needs-e2e-orphan-watchdog.sh. Тестируем
# чистую логику (без сетевых вызовов) через PATH-hijack: подставляем mock-gh,
# который возвращает заранее заданные JSON / вывод и записывает side-effects
# в journal для assert.
#
# Scenarios:
#   C1. no_open_issues: gh api issues возвращает [] → closed=0, recheck=0, skipped=0
#   C2. merged_pr_but_recent_e2e_success → close reason=completed
#   C3. merged_pr_but_old_e2e_success → relabel needs-e2e:recheck-develop (НЕ close)
#   C4. no_pr_old_issue → close reason=not_planned
#   C5. no_pr_young_issue → SKIP (age < NEEDS_E2E_NO_PR_DAYS)
#   C6. e2e_rejected_label → SKIP (никогда не трогаем)
#   C7. open_pr_in_flight → SKIP
#
# Run:
#   bash scripts/agent_flow/tests/test_orphan_needs_e2e_watchdog.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-needs-e2e-orphan-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required (test uses it for date math)"; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"

# Mock gh — программируемое поведение через файлы-фикстуры:
#   ISSUES_LIST_JSON     — массив issues, что вернёт `gh api issues?labels=needs-e2e`
#   PR_LIST_<n>_JSON     — массив PR для issue #N (что вернёт `gh pr list --search N in:title`)
#   PR_META_<n>_JSON     — {mergeCommit, mergedAt} для `gh pr view N --json ...`
#   LAST_E2E_SUCCESS_TS  — строка, что вернёт `gh api actions/runs?branch=develop...`
#   ACTIONS_RUNS_FAIL=1  — форсит ошибку для `gh api actions/runs...` (rate-limit)
#   GH_AUTH_FAIL=1       — форсит ошибку auth-status
# Все side-effects пишутся в JOURNAL для assert.
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
journal="${JOURNAL_FILE:-/tmp/_journal_unset}"
get_state() {
    local key="$1"
    if [ -f "$STATE_FILE" ]; then
        grep -E "^${key}=" "$STATE_FILE" | head -n1 | sed "s@^${key}=@@"
    fi
}
log_to_journal() {
    printf '%s\n' "$*" >> "$journal" 2>/dev/null || true
}

case "$1" in
    auth)
        if [ "${GH_AUTH_FAIL:-}" = "1" ]; then
            echo "FATAL: gh not authenticated" >&2
            exit 1
        fi
        log_to_journal "gh auth status OK"
        exit 0
        ;;
    api)
        shift
        case "$*" in
            *"actions/runs"*)
                log_to_journal "gh api actions/runs"
                if [ "${ACTIONS_RUNS_FAIL:-}" = "1" ]; then
                    exit 1
                fi
                # Mock возвращает workflow_runs-массив (1 элемент) с полями
                # .name и .updated_at, чтобы фильтр watchdog
                # '[.workflow_runs[] | select(.name | startswith("L: E2E Voice Test"))] | .[0].updated_at // empty'
                # отработал. STATE: LAST_E2E_SUCCESS_TS=2026-08-20T00:00:00Z (без кавычек).
                v="$(get_state LAST_E2E_SUCCESS_TS)"
                if [ -n "$v" ]; then
                    # Без кавычек вок значения — имитируем json-строку.
                    printf '{"workflow_runs":[{"name":"L: E2E Voice Test","updated_at":"%s"}]}' "$v"
                else
                    printf '{"workflow_runs":[]}'
                fi
                exit 0
                ;;
            *"issues?state=open"*)
                log_to_journal "gh api issues?labels=needs-e2e"
                v="$(get_state ISSUES_LIST_JSON)"
                if [ -n "$v" ]; then printf '%s' "$v"; else printf '[]'; fi
                exit 0
                ;;
        esac
        # Unknown api path
        log_to_journal "gh api (unknown path): $*"
        exit 1
        ;;
    issue)
        action="$2"; shift 2
        case "$action" in
            close)
                issue_num="${1:-}"
                reason="$(printf '%s' "$*" | sed -nE 's/.*--reason[[:space:]]+([^ ]+).*/\1/p')"
                log_to_journal "gh issue close $issue_num --reason $reason"
                exit 0
                ;;
            edit)
                issue_num="${1:-}"
                add="$(printf '%s' "$*" | sed -nE 's/.*--add-label[[:space:]]+([^ ]+).*/\1/p')"
                rem="$(printf '%s' "$*" | sed -nE 's/.*--remove-label[[:space:]]+([^ ]+).*/\1/p')"
                if [ -n "$add" ]; then
                    log_to_journal "gh issue edit $issue_num --add-label $add"
                fi
                if [ -n "$rem" ]; then
                    log_to_journal "gh issue edit $issue_num --remove-label $rem"
                fi
                exit 0
                ;;
        esac
        log_to_journal "gh issue $action (other)"
        exit 0
        ;;
    pr)
        action="$2"; shift 2
        case "$action" in
            list)
                # gh pr list --repo X --state all --search "2222 in:title" --json ...
                # Извлекаем первое число перед " in:title" (после любого нечислового).
                target="$(printf '%s' "$*" | sed -nE 's/.*[^0-9]([0-9]+) in:title.*/\1/p')"
                if [ -n "$target" ]; then
                    v="$(get_state PR_LIST_${target}_JSON)"
                    if [ -n "$v" ]; then printf '%s' "$v"; else printf '[]'; fi
                else
                    printf '[]'
                fi
                exit 0
                ;;
            view)
                pr_num="${1:-}"
                log_to_journal "gh pr view $pr_num"
                v="$(get_state PR_META_${pr_num}_JSON)"
                if [ -n "$v" ]; then printf '%s' "$v"; else printf '{}'; fi
                exit 0
                ;;
        esac
        log_to_journal "gh pr $action (other)"
        exit 0
        ;;
    *)
        log_to_journal "gh (unknown subcommand): $*"
        exit 0
        ;;
esac
EOF
chmod +x "$WORK/bin/gh"

run_watchdog() {
    local extra_env="${1:-}"
    : > "$WORK/journal"
    > "$WORK/stderr.log"
    local exports=""
    local kv
    for kv in $extra_env; do
        exports="$exports export $kv;"
    done
    bash -c "
        export PATH='$WORK/bin:/usr/bin:/bin'
        export HOME='/tmp'
        export GH_REPO='krikz/rob_box_project'
        export KANBAN_DB='$WORK/nonexistent-kanban.db'
        export DRY_RUN='${DRY_RUN:-false}'
        export NEEDS_E2E_NO_PR_DAYS='${NEEDS_E2E_NO_PR_DAYS:-7}'
        export LOG_FILE='$WORK/wd.log'
        export JOURNAL_FILE='$WORK/journal'
        export STATE_FILE='$WORK/gh_state'
        $exports
        bash '$WATCHDOG_SH' 2>'$WORK/stderr.log'
    "
    local rc=$?
    WD_LOG="$(cat "$WORK/wd.log" 2>/dev/null)"
    STDERR_LOG="$(cat "$WORK/stderr.log" 2>/dev/null)"
    JOURNAL="$(cat "$WORK/journal" 2>/dev/null)"
    return $rc
}

assert_contains() {
    local needle="$1" haystack="$2" msg="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        printf '  ok: %s\n' "$msg"
        return 0
    else
        printf '  FAIL: %s\n    needle: %q\n    haystack: %q\n' "$msg" "$needle" "$haystack"
        return 1
    fi
}

assert_not_contains() {
    local needle="$1" haystack="$2" msg="$3"
    if printf '%s' "$haystack" | grep -qF -- "$needle"; then
        printf '  FAIL: %s\n    needle should NOT appear: %q\n    haystack: %q\n' "$msg" "$needle" "$haystack"
        return 1
    fi
    printf '  ok: %s\n' "$msg"
    return 0
}

pass() { printf '  pass: %s\n' "$1"; }
fail() { printf 'FAIL: %s\n' "$1"; TOTAL_FAIL=$((TOTAL_FAIL+1)); }

TOTAL_FAIL=0
TOTAL_RUN=0

# ============================================================================
# C1. no_open_issues
# ============================================================================
run_test_c1() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C1: no open issues ===\n'
    rm -f "$WORK/gh_state"
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog "ISSUES_LIST_JSON=[]"
    if assert_contains "ISSUES: found 0" "$WD_LOG" "no issues → found=0"; then
        if assert_contains "END (no candidates)" "$WD_LOG" "no issues → END no_candidates"; then
            pass C1
            return 0
        fi
    fi
    fail C1
    return 1
}

# ============================================================================
# C2. merged PR + recent e2e success → close reason=completed
# ============================================================================
run_test_c2() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C2: merged PR, recent e2e success → close ===\n'
    rm -f "$WORK/gh_state"
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":2222,"title":"stale ADR ref","state":"open","labels":[{"name":"needs-e2e"},{"name":"bug"}],"body":"","created_at":"2026-09-09T00:43:14Z"}]
PR_LIST_2222_JSON=[{"number":2227,"title":"fix ADR ref #2222","state":"MERGED","mergedAt":"2026-09-09T12:33:03Z"}]
PR_META_2227_JSON={"mergeCommit":{"oid":"abc1234567890"},"mergedAt":"2026-09-09T12:33:03Z"}
LAST_E2E_SUCCESS_TS=2026-09-15T04:00:00Z
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "ORPHAN #2222 — PR #2227 MERGED" "$WD_LOG" "C2: merged PR detected"; then
        if assert_contains "closed=1" "$WD_LOG" "C2: closed counter = 1"; then
            if assert_contains "gh issue close 2222 --reason completed" "$JOURNAL" "C2: gh issue close reason=completed"; then
                pass C2
                return 0
            fi
        fi
    fi
    fail C2
    return 1
}

# ============================================================================
# C3. merged PR + OLD e2e success (< merge_date) → recheck-develop, НЕ close
# ============================================================================
run_test_c3() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C3: merged PR, last_e2e older than merge → recheck-develop ===\n'
    rm -f "$WORK/gh_state"
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":1736,"title":"deploy issues 29.08","state":"open","labels":[{"name":"needs-e2e"},{"name":"deployment"}],"body":"","created_at":"2026-08-29T15:31:20Z"}]
PR_LIST_1736_JSON=[{"number":1741,"title":"fix mcp race #1736","state":"MERGED","mergedAt":"2026-08-29T16:00:00Z"}]
PR_META_1741_JSON={"mergeCommit":{"oid":"def4567890123"},"mergedAt":"2026-08-29T16:00:00Z"}
LAST_E2E_SUCCESS_TS=2026-08-20T00:00:00Z
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "RECHECK #1736" "$WD_LOG" "C3: recheck-develop path taken"; then
        if assert_contains "recheck=1" "$WD_LOG" "C3: recheck counter = 1"; then
            if assert_contains "gh issue edit 1736 --add-label needs-e2e:recheck-develop" "$JOURNAL" "C3: relabel needs-e2e:recheck-develop"; then
                if assert_contains "gh issue edit 1736 --remove-label needs-e2e" "$JOURNAL" "C3: remove-label needs-e2e"; then
                    if assert_not_contains "gh issue close 1736" "$JOURNAL" "C3: НЕ закрываем #1736 (только relabel)"; then
                        pass C3
                        return 0
                    fi
                fi
            fi
        fi
    fi
    fail C3
    return 1
}

# ============================================================================
# C4. no PR + old issue (age >= NEEDS_E2E_NO_PR_DAYS) → close reason=not_planned
# ============================================================================
run_test_c4() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C4: no PR + old issue → close not_planned ===\n'
    rm -f "$WORK/gh_state"
    # Создаём дату 30 дней назад
    old_date=$(python3 -c "from datetime import datetime, timezone, timedelta; print((datetime.now(timezone.utc) - timedelta(days=30)).strftime('%Y-%m-%dT%H:%M:%SZ'))")
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":1911,"title":"[AV-19] teleop","state":"open","labels":[{"name":"needs-e2e"},{"name":"agent:backend"}],"body":"","created_at":"${old_date}"}]
PR_LIST_1911_JSON=[]
PR_META_1911_JSON={}
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "ORPHAN-NO-PR #1911" "$WD_LOG" "C4: no-pr-orphan path taken"; then
        if assert_contains "closed=1" "$WD_LOG" "C4: closed counter = 1"; then
            if assert_contains "gh issue close 1911 --reason not_planned" "$JOURNAL" "C4: gh issue close reason=not_planned"; then
                pass C4
                return 0
            fi
        fi
    fi
    fail C4
    return 1
}

# ============================================================================
# C5. no PR + YOUNG issue (age < NEEDS_E2E_NO_PR_DAYS) → SKIP
# ============================================================================
run_test_c5() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C5: no PR + young issue → SKIP ===\n'
    rm -f "$WORK/gh_state"
    # Создаём дату 2 дня назад (< 7 дней)
    young_date=$(python3 -c "from datetime import datetime, timezone, timedelta; print((datetime.now(timezone.utc) - timedelta(days=2)).strftime('%Y-%m-%dT%H:%M:%SZ'))")
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":2400,"title":"[AV-XX] recent","state":"open","labels":[{"name":"needs-e2e"},{"name":"agent:backend"}],"body":"","created_at":"${young_date}"}]
PR_LIST_2400_JSON=[]
PR_META_2400_JSON={}
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "KEEP  #2400 no PR, but age=2d" "$WD_LOG" "C5: too young → KEEP (age < threshold)"; then
        if assert_not_contains "gh issue close 2400" "$JOURNAL" "C5: НЕ закрываем #2400"; then
            pass C5
            return 0
        fi
    fi
    fail C5
    return 1
}

# ============================================================================
# C6. e2e:rejected label → SKIP всегда
# ============================================================================
run_test_c6() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C6: e2e:rejected → SKIP (human judgement) ===\n'
    rm -f "$WORK/gh_state"
    # 30 дней назад + e2e:rejected label → без watchdog-обработки
    old_date=$(python3 -c "from datetime import datetime, timezone, timedelta; print((datetime.now(timezone.utc) - timedelta(days=30)).strftime('%Y-%m-%dT%H:%M:%SZ'))")
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":2138,"title":"quest voice picker","state":"open","labels":[{"name":"needs-e2e"},{"name":"e2e:rejected"},{"name":"bug"}],"body":"","created_at":"${old_date}"}]
PR_LIST_2138_JSON=[]
PR_META_2138_JSON={}
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "SKIP  #2138 label=e2e:rejected (human judgement" "$WD_LOG" "C6: e2e:rejected → SKIP"; then
        if assert_not_contains "gh issue close 2138" "$JOURNAL" "C6: НЕ закрываем #2138 (e2e:rejected)"; then
            pass C6
            return 0
        fi
    fi
    fail C6
    return 1
}

# ============================================================================
# C7. open PR in flight → SKIP
# ============================================================================
run_test_c7() {
    TOTAL_RUN=$((TOTAL_RUN+1))
    printf '=== C7: open PR in flight → SKIP ===\n'
    rm -f "$WORK/gh_state"
    cat > "$WORK/gh_state" <<EOF
ISSUES_LIST_JSON=[{"number":2406,"title":"prompt fix","state":"open","labels":[{"name":"needs-e2e"},{"name":"priority:high"}],"body":"","created_at":"2026-09-14T11:21:21Z"}]
PR_LIST_2406_JSON=[{"number":2458,"title":"fix(prompt #2406)","state":"OPEN","mergedAt":null}]
PR_META_2458_JSON={}
EOF
    DRY_RUN=false NEEDS_E2E_NO_PR_DAYS=7 run_watchdog
    if assert_contains "KEEP  #2406 open_prs=2458 (in-flight, not orphan)" "$WD_LOG" "C7: open PR → KEEP"; then
        if assert_not_contains "gh issue close 2406" "$JOURNAL" "C7: НЕ закрываем #2406"; then
            pass C7
            return 0
        fi
    fi
    fail C7
    return 1
}

run_test_c1
run_test_c2
run_test_c3
run_test_c4
run_test_c5
run_test_c6
run_test_c7

printf '\n==== Summary ====\n'
printf 'total:  %d\n' "$TOTAL_RUN"
printf 'failed: %d\n' "$TOTAL_FAIL"
if [ "$TOTAL_FAIL" -gt 0 ]; then
    exit 1
fi
exit 0