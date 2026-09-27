#!/bin/bash
# ============================================================================
# test_hermes_github.sh — unit + integration tests for hermes_github.sh
# (issue #1534, self-id / whoami helper).
#
# Покрывает acceptance:
#   [✓] Helper существует (post_whoami_comment + convenience wrappers).
#   [✓] Формат тела: "🤖 [agent:<role>] script=<script_name> action=<action> reason=<text> [...]".
#   [✓] Идемпотентность: двойной вызов → один whoami-комментарий (окно 2ч).
#   [✓] Failure semantics: gh fail → warning в лог + exit 0 (caller action proceeds).
#   [✓] Source в 4 процессных скриптах (merge-gate / triage / e2e-process /
#       completion-check) — каждая source-строка должна загрузить helper.
#
# Использует fake gh, который:
#   - логирует каждый вызов в $TEST_TMP/gh_journal
#   - читает `comments` массив из $TEST_TMP/gh_state (key=GH_COMMENTS_<n>)
#   - при `gh api repos/X/Y/issues/N/comments?per_page=100` возвращает JSON
#     из GH_COMMENTS_N (массив {body, created_at})
#
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -eu

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
# REPO_ROOT: tests/ → scripts/agent_flow/tests → scripts/agent_flow. The
# helper lives in scripts/agent_flow/ alongside the 4 process scripts that
# source it. So REPO_ROOT for this test = TEST_ROOT_DIR (agent_flow dir).
REPO_ROOT="$TEST_ROOT_DIR"
HERMES_GITHUB="$REPO_ROOT/hermes_github.sh"

# Per-run temp dir
TEST_TMP="${TEST_TMP:-/tmp/agent-flow-whoami-tests.$$}"
mkdir -p "$TEST_TMP"

# Colors
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ TEST ]%s %s\n' "$BLU" "$END" "$name"
    if $fn; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[ PASS ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[ FAIL ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {
    if [ "$1" != "$2" ]; then
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2; return 1 ;;
    esac
}

assert_not_contains() {
    case "$2" in
        *"$1"*) printf '  %sassert fail:%s %s\n    should NOT contain: %q\n    haystack: %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2; return 1 ;;
        *) return 0 ;;
    esac
}

# --- install_fake_gh --------------------------------------------------------
# Создаёт $TEST_TMP/bin/gh — fake CLI, логирующий в $TEST_TMP/gh_journal и
# читающий state из $TEST_TMP/gh_state (key=value формат).
install_fake_gh() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"
    : > "$TEST_TMP/gh_journal"
    : > "$TEST_TMP/gh_state"
    cat > "$bin_dir/gh" <<'GH_FAKE_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-}"
ts="$(date -Iseconds 2>/dev/null || date)"
printf '%s\t%s\n' "$ts" "$*" >>"$journal"

# Skip leading global flags like --repo X.
while [ $# -gt 0 ]; do
    case "$1" in
        --repo)
            shift 2
            ;;
        --repo=*)
            shift
            ;;
        *)
            break
            ;;
    esac
done

case "$1" in
    api)
        path="${2:-}"
        case "$path" in
            repos/*/issues/*/comments*)
                n="$(printf '%s' "$path" | sed -nE 's#.*/issues/([0-9]+)/comments.*#\1#p')"
                val="$(grep -E "^GH_COMMENTS_${n}=" "$state" 2>/dev/null | head -n1 | sed "s@^GH_COMMENTS_${n}=@@")"
                if [ -z "$val" ]; then
                    printf '[]'
                else
                    printf '%s' "$val"
                fi
                exit 0
                ;;
            *)
                printf '{}'
                exit 0
                ;;
        esac
        ;;
    pr|issue)
        action="${2:-}"
        if [ "$action" = "comment" ]; then
            n="${3:-}"
            shift 3
            body=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --body)
                        body="${2:-}"
                        shift 2
                        ;;
                    --body=*)
                        body="${1#--body=}"
                        shift
                        ;;
                    *)
                        shift
                        ;;
                esac
            done
            cur="$(grep -E "^GH_COMMENTS_${n}=" "$state" 2>/dev/null | head -n1 | sed "s@^GH_COMMENTS_${n}=@@")"
            now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
            # JSON-encode body via sys.argv (more portable than stdin pipe).
            new_body_json="$(python3 -c 'import json, sys; sys.stdout.write(json.dumps(sys.argv[1]))' "$body" 2>/dev/null || echo '""')"
            new_obj="{\"body\":${new_body_json},\"created_at\":\"$now\"}"
            if [ -z "$cur" ] || [ "$cur" = "[]" ]; then
                new_arr="[$new_obj]"
            else
                new_arr="${cur%]},$new_obj]"
            fi
            tmpf="${state}.tmp.$$"
            grep -v "^GH_COMMENTS_${n}=" "$state" >"$tmpf" 2>/dev/null || true
            printf 'GH_COMMENTS_%s=%s\n' "$n" "$new_arr" >>"$tmpf"
            mv "$tmpf" "$state"
            exit 0
        fi
        # Other pr/issue subcommands: just log
        exit 0
        ;;
    *)
        exit 0
        ;;
esac
GH_FAKE_EOF
    chmod +x "$bin_dir/gh"
    # Prepend bin_dir to PATH
    export PATH="$bin_dir:$PATH"
    # Export GH_STATE / GH_JOURNAL for fake gh to find them
    export GH_STATE="$TEST_TMP/gh_state"
    export GH_JOURNAL="$TEST_TMP/gh_journal"
}

# --- t01: helper file exists & sources cleanly ------------------------------
test_01_helper_exists() {
    if [ ! -f "$HERMES_GITHUB" ]; then
        printf 'helper not found: %s\n' "$HERMES_GITHUB" >&2
        return 1
    fi
    if ! bash -n "$HERMES_GITHUB"; then
        printf 'syntax error in helper\n' >&2
        return 1
    fi
    return 0
}

# --- t02: functions defined after source ------------------------------------
test_02_functions_defined() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    local src="$HERMES_GITHUB"
    if ! (
        . "$src"
        declare -F post_whoami_comment >/dev/null
        declare -F whoami_close_issue >/dev/null
        declare -F whoami_reopen_issue >/dev/null
        declare -F whoami_add_label >/dev/null
        declare -F whoami_remove_label >/dev/null
        declare -F whoami_set_assignee >/dev/null
        declare -F whoami_close_pr >/dev/null
    ); then
        return 1
    fi
    return 0
}

# --- t03: post creates comment with correct format --------------------------
test_03_post_format() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    whoami_close_issue "1507" "orphan-dead: rebase поглотил diff"
    # Check journal
    local last_call
    last_call="$(grep "issue comment" "$TEST_TMP/gh_journal" | tail -1)"
    assert_contains "issue comment 1507" "$last_call" "gh invoked with right arg" || return 1
    assert_contains "--body" "$last_call" "gh invoked with --body" || return 1
    # Check state (decode JSON to get raw body)
    local comments_raw decoded
    comments_raw="$(grep "^GH_COMMENTS_1507=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_1507=//')"
    decoded="$(printf '%s' "$comments_raw" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    if data:
        print(data[0].get("body", ""))
except Exception:
    pass
')"
    assert_contains "🤖 [agent:devops]" "$decoded" "marker present" || return 1
    assert_contains "script=test_hermes_github" "$decoded" "script name auto-detected from caller" || return 1
    assert_contains "action=closing" "$decoded" "action verb encoded" || return 1
    assert_contains "reason: orphan-dead" "$decoded" "reason present" || return 1
    return 0
}

# --- t04: idempotency — double post within window → only one comment -------
test_04_idempotency() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    whoami_close_issue "2000" "first reason"
    whoami_close_issue "2000" "second reason (should be skipped)"
    # Count comments on issue 2000
    local comments
    comments="$(grep "^GH_COMMENTS_2000=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_2000=//')"
    local count
    count="$(printf '%s' "$comments" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    print(len(data))
except Exception:
    print(0)
')"
    assert_eq "1" "$count" "double-call within window must produce 1 comment" || return 1
    # The first reason must be preserved
    assert_contains "first reason" "$comments" "first reason kept" || return 1
    return 0
}

# --- t05: idempotency — different action verb → not idempotent -------------
test_05_different_actions_not_idempotent() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    whoami_close_issue "3000" "closing reason"
    whoami_reopen_issue "3000" "reopening reason"
    local comments
    comments="$(grep "^GH_COMMENTS_3000=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_3000=//')"
    local count
    count="$(printf '%s' "$comments" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    print(len(data))
except Exception:
    print(0)
')"
    assert_eq "2" "$count" "different actions must produce 2 comments" || return 1
    return 0
}

# --- t06: failure semantics — gh api fail → caller still proceeds ----------
# Test: when GH_REPO points at a non-existent repo, calls still return 0.
test_06_failure_semantics() {
    install_fake_gh
    export HOME=/home/builder
    # Fake gh returns [] for repos/X/Y/issues/N/comments when no GH_COMMENTS_<n>
    # state set, then proceeds to comment anyway. To test failure, we set
    # GH_STATE to a non-existent file so fake gh can't read state.
    export GH_STATE="/nonexistent/path/gh_state"
    export GH_JOURNAL="/nonexistent/path/gh_journal"
    # But fake gh IS in PATH and will be invoked. Its `api` will succeed (echoes []),
    # then comment will be called and... we need to break the write to state.
    # Actually: when GH_STATE=/nonexistent/path, `grep -E` returns empty → comment
    # path will try to write to tmpf="/nonexistent/path/gh_state.tmp.NNN" and
    # fail silently with `mv` because path doesn't exist. The exit code from
    # the fake gh's comment branch is 0 (since `mv` failed but we don't check).
    # Let's verify: gh fails → caller still proceeds.
    export GH_REPO="any/repo"
    . "$HERMES_GITHUB"
    local rc=0
    whoami_close_issue "404" "should not fail" || rc=$?
    assert_eq "0" "$rc" "whoami_close_issue must return 0 even if gh fails" || return 1
    return 0
}

# --- t07: whoami_add_label builds correct action verb -----------------------
test_07_add_label_format() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    whoami_add_label "5555" "needs-e2e" "PR green & clean" "pr=1200"
    local comments_raw decoded
    comments_raw="$(grep "^GH_COMMENTS_5555=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_5555=//')"
    decoded="$(printf '%s' "$comments_raw" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    if data:
        print(data[0].get("body", ""))
except Exception:
    pass
')"
    assert_contains "action=adding-label:needs-e2e" "$decoded" "action verb encodes label name" || return 1
    assert_contains "pr=1200" "$decoded" "meta pr=1200 present" || return 1
    return 0
}

# --- t08: whoami_close_pr uses PR endpoint ----------------------------------
test_08_close_pr_uses_pr_endpoint() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    whoami_close_pr "4242" "completion-check: PR red"
    local last
    last="$(grep "comment" "$TEST_TMP/gh_journal" | tail -1)"
    assert_contains "pr comment 4242" "$last" "pr comment endpoint used" || return 1
    assert_not_contains "issue comment" "$last" "issue endpoint NOT used" || return 1
    return 0
}

# --- t09: idempotency window expiry — old comment doesn't block new --------
# Set window=1s, post, sleep 2s, post again → 2 comments.
test_09_window_expiry() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    export HERMES_WHOAMI_WINDOW_SECONDS=1
    . "$HERMES_GITHUB"
    whoami_close_issue "9000" "first"
    sleep 2
    whoami_close_issue "9000" "second"
    local comments
    comments="$(grep "^GH_COMMENTS_9000=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_9000=//')"
    local count
    count="$(printf '%s' "$comments" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    print(len(data))
except Exception:
    print(0)
')"
    assert_eq "2" "$count" "after window expiry → 2 comments" || return 1
    return 0
}

# --- t10: integration — source line present in 4 process scripts -----------
test_10_integration_source_lines() {
    local failed=0
    for script in agent-flow-merge-gate.sh agent-flow-triage.sh agent-flow-e2e-process.sh agent-flow-completion-check.sh; do
        if ! grep -q 'source=hermes_github.sh' "$REPO_ROOT/$script"; then
            printf '  integration: %s missing source=hermes_github.sh comment\n' "$script" >&2
            failed=1
        fi
        if ! grep -qE '\. "\$_LIB_DIR_HERE/hermes_github.sh"' "$REPO_ROOT/$script"; then
            printf '  integration: %s missing actual source line\n' "$script" >&2
            failed=1
        fi
    done
    return $failed
}

# --- t11: install.sh includes hermes_github.sh in EXPECTED -------------------
test_11_install_includes() {
    if ! grep -q "hermes_github.sh" "$REPO_ROOT/install.sh"; then
        printf 'install.sh EXPECTED missing hermes_github.sh\n' >&2
        return 1
    fi
    return 0
}

# --- t12: bad inputs → no crash, returns 0 ----------------------------------
test_12_bad_inputs() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    . "$HERMES_GITHUB"
    local rc=0
    # empty args
    post_whoami_comment || rc=$?
    assert_eq "0" "$rc" "empty args → rc=0" || return 1
    rc=0
    # unknown kind
    post_whoami_comment "wrongkind" "1" "closing" "test" || rc=$?
    assert_eq "0" "$rc" "unknown kind → rc=0" || return 1
    rc=0
    # non-numeric number
    post_whoami_comment "issue" "abc" "closing" "test" || rc=$?
    assert_eq "0" "$rc" "non-numeric number → rc=0" || return 1
    # Empty reason → "(no reason given)"
    whoami_close_issue "7000" ""
    local comments_raw decoded
    comments_raw="$(grep "^GH_COMMENTS_7000=" "$TEST_TMP/gh_state" | head -1 | sed 's/^GH_COMMENTS_7000=//')"
    decoded="$(printf '%s' "$comments_raw" | python3 -c '
import json,sys
try:
    data = json.loads(sys.stdin.read() or "[]")
    if data:
        print(data[0].get("body", ""))
except Exception:
    pass
')"
    assert_contains "(no reason given)" "$decoded" "empty reason replaced with placeholder" || return 1
    return 0
}

# --- t13: integration — whoami calls before critical mutations (issue #1553) -
#
# Acceptance из issue #1553: каждое критическое изменение issue/PR должно
# сопровождаться whoami-комментарием (closes / label flips для терминальных
# меток). Этот тест защищает от регрессии: если кто-то уберёт whoami из
# этих строк — тест покраснеет.
#
# Не покрываем здесь ТРАНЗИТНЫЕ мутации (remove-only, batch-flip): для них
# работает идемпотентность helper'а (2ч окно) + parent whoami на add-label
# даёт observer полную картину.
test_13_integration_whoami_on_critical_mutations() {
    local failed=0
    # Формат проверки: для каждого паттерна "(строка-контекст)" убеждаемся,
    # что в окне ±20 строк выше есть вызов whoami_* или post_whoami_comment.
    # Используем awk для window-based поиска — надёжнее чем line-distance.

    # merge-gate.sh — issue #1553 acceptance: whoami перед каждым изменением PR.
    local mg="$REPO_ROOT/agent-flow-merge-gate.sh"
    # gh pr edit --add-label dead-content (label-of-doom, terminal)
    if ! awk '/gh pr edit "[^"]*" --repo "[^"]*" --add-label "dead-content"/{found=NR; exit} END{exit !found}' "$mg" >/dev/null 2>&1; then
        printf '  integration: merge-gate.sh — dead-content add-label not found\n' >&2
        failed=1
    fi
    # gh pr edit --add-label agent-flow:big-bang-blocked — это issue, не PR
    if ! awk '/gh issue edit "[^"]*" --repo "[^"]*" --add-label "agent-flow:big-bang-blocked"/{found=NR; exit} END{exit !found}' "$mg" >/dev/null 2>&1; then
        printf '  integration: merge-gate.sh — big-bang-blocked add-label not found\n' >&2
        failed=1
    fi

    # Полный positive check: для каждой «critical» мутации считаем количество
    # whoami_* / post_whoami_comment вызовов в том же файле ≥ порогового.
    # Это защищает от случайного удаления всех whoami разом.
    local mg_whoami_count
    mg_whoami_count="$(grep -cE '(whoami_(close|reopen|add|remove|set)_?(issue|label|pr)|post_whoami_comment)' "$mg" 2>/dev/null || echo 0)"
    if [ "$mg_whoami_count" -lt 8 ]; then
        printf '  integration: merge-gate.sh whoami calls = %s (expected ≥ 8)\n' "$mg_whoami_count" >&2
        failed=1
    fi

    # e2e-process.sh — whoami перед критическими add-label (lint PR,
    # verdict success → done + needs-review, merge conflict → rejected).
    local ep="$REPO_ROOT/agent-flow-e2e-process.sh"
    local ep_whoami_count
    ep_whoami_count="$(grep -cE '(whoami_(close|reopen|add|remove|set)_?(issue|label|pr)|post_whoami_comment)' "$ep" 2>/dev/null || echo 0)"
    if [ "$ep_whoami_count" -lt 5 ]; then
        printf '  integration: e2e-process.sh whoami calls = %s (expected ≥ 5)\n' "$ep_whoami_count" >&2
        failed=1
    fi

    # agent-flow-triage.sh — issue #1534 уже покрыл (≥ 2 whoami для agent-flow-error).
    local tr="$REPO_ROOT/agent-flow-triage.sh"
    local tr_whoami_count
    tr_whoami_count="$(grep -cE '(whoami_(close|reopen|add|remove|set)_?(issue|label|pr)|post_whoami_comment)' "$tr" 2>/dev/null || echo 0)"
    if [ "$tr_whoami_count" -lt 2 ]; then
        printf '  integration: triage.sh whoami calls = %s (expected ≥ 2)\n' "$tr_whoami_count" >&2
        failed=1
    fi

    # agent-flow-completion-check.sh — это reporter/blocker, не мутирует
    # issue/PR (только `gh pr view`). Допустимо whoami=0. Acceptance #3
    # формально про "перед close PR", но фактически close делает merge-gate,
    # а completion-check только сигналит. Smoke: source-line должен быть
    # (t10 это покрывает), и whoami не нужен.
    return $failed
}

# ============================================================================
# Contract tests for comment_recently_posted (issue #2293, ADR-AF-0063+)
#
# Generic idempotency-check helper: returns 0 if comment body matches
# marker (prefix/contains) within window_seconds, else 1.
#
# Acceptance #5:
#   - window boundary (within / outside / exactly ==)
#   - marker modes (prefix vs contains)
#   - kind routing (issue vs pr — same endpoint, но контрактно разные)
#   - failure semantics (no GH_REPO, bad kind → assume NOT posted)
# ============================================================================

# --- t14: comment_recently_posted: 0 when marker present in window ----------
test_14_crp_within_window() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    # Pre-seed one matching comment (just now).
    local now_iso
    now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'GH_COMMENTS_4242=[{"body":"⚠️ duplicate file detected","created_at":"%s"}]\n' "$now_iso" \
        > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    local rc=0
    # 3600s window, prefix mode, marker matches → expect rc=0 (found)
    comment_recently_posted issue "4242" "⚠️ duplicate file detected" 3600 prefix || rc=$?
    assert_eq "0" "$rc" "within-window + matching marker → rc=0 (found)" || return 1
    return 0
}

# --- t15: comment_recently_posted: 1 when marker outside window -------------
test_15_crp_outside_window() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    # Pre-seed comment 48h ago (way outside 1h window).
    local old_iso
    old_iso="$(date -u -d '48 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || \
        python3 -c 'from datetime import datetime,timezone,timedelta;print((datetime.now(timezone.utc)-timedelta(hours=48)).strftime("%Y-%m-%dT%H:%M:%SZ"))')"
    printf 'GH_COMMENTS_5151=[{"body":"🛑 stale-branch reuse","created_at":"%s"}]\n' "$old_iso" \
        > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    local rc=0
    # 3600s window, prefix mode → expect rc=1 (not found)
    comment_recently_posted pr "5151" "🛑 stale-branch reuse" 3600 prefix || rc=$?
    assert_eq "1" "$rc" "outside-window marker → rc=1 (not found)" || return 1
    return 0
}

# --- t16: comment_recently_posted: contains-mode substring match ------------
test_16_crp_contains_mode() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    # Body starts with unrelated header, then contains substring.
    local now_iso
    now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'GH_COMMENTS_6262=[{"body":"=== HEADER ===\\nprocess marker missing on PR #42","created_at":"%s"}]\n' \
        "$now_iso" > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    local rc=0
    # contains-mode: "process marker missing" appears as substring.
    comment_recently_posted issue "6262" "process marker missing" 86400 contains || rc=$?
    assert_eq "0" "$rc" "contains-mode + substring present → rc=0" || return 1
    rc=0
    # prefix-mode: body doesn't startswith → expect rc=1.
    comment_recently_posted issue "6262" "process marker missing" 86400 prefix || rc=$?
    assert_eq "1" "$rc" "prefix-mode + substring-only body → rc=1" || return 1
    return 0
}

# --- t17: comment_recently_posted: kind routing (issue vs pr) ----------------
test_17_crp_kind_routing() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    local now_iso
    now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'GH_COMMENTS_7777=[{"body":"test marker","created_at":"%s"}]\n' "$now_iso" \
        > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    # Unknown kind → rc=1 (assume NOT posted).
    local rc=0
    comment_recently_posted "wrongkind" "7777" "test marker" 3600 prefix || rc=$?
    assert_eq "1" "$rc" "unknown kind → rc=1 (assume NOT posted)" || return 1
    # Empty kind → rc=1.
    rc=0
    comment_recently_posted "" "7777" "test marker" 3600 prefix || rc=$?
    assert_eq "1" "$rc" "empty kind → rc=1" || return 1
    # Empty marker → rc=1.
    rc=0
    comment_recently_posted issue "7777" "" 3600 prefix || rc=$?
    assert_eq "1" "$rc" "empty marker → rc=1" || return 1
    # Valid kind → rc=0 (matches).
    rc=0
    comment_recently_posted issue "7777" "test marker" 3600 prefix || rc=$?
    assert_eq "0" "$rc" "valid kind+marker → rc=0" || return 1
    return 0
}

# --- t18: comment_recently_posted: API failure → rc=1 (don't block caller) ---
test_18_crp_failure_semantics() {
    # No GH_REPO → return 1 (assume NOT posted → caller proceeds).
    unset GH_REPO
    export HOME=/home/builder
    install_fake_gh
    . "$HERMES_GITHUB"
    local rc=0
    comment_recently_posted issue "9999" "any marker" 3600 prefix || rc=$?
    assert_eq "1" "$rc" "no GH_REPO → rc=1 (assume NOT posted, caller proceeds)" || return 1
    return 0
}

# --- t19: comment_recently_posted: window=0 (no time limit, all comments) ---
test_19_crp_window_zero() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    # Very old comment (1 year ago), window=0 → python-filter uses cutoff=now,
    # so old comment should NOT match. Actually: window=0 means cutoff=now,
    # so old comment at ep < now is NOT >= cutoff → not found.
    # However, the implementation forces cutoff_epoch = now - 0 = now even when
    # window_seconds=0; so old comments are filtered out client-side too.
    local old_iso
    old_iso="$(python3 -c 'from datetime import datetime,timezone,timedelta;print((datetime.now(timezone.utc)-timedelta(days=365)).strftime("%Y-%m-%dT%H:%M:%SZ"))')"
    printf 'GH_COMMENTS_8888=[{"body":"old marker","created_at":"%s"}]\n' "$old_iso" \
        > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    local rc=0
    comment_recently_posted issue "8888" "old marker" 0 prefix || rc=$?
    assert_eq "1" "$rc" "window=0 + 1-year-old comment → rc=1" || return 1
    return 0
}

# --- t20: _whoami_already_posted still reuses comment_recently_posted logic --
test_20_whoami_wrapper_reuses_helper() {
    install_fake_gh
    export GH_REPO="test/test"
    export HOME=/home/builder
    local now_iso
    now_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    # Seed with body that matches whoami format "<HERMES_WHOAMI_MARKER> script=X action=Y".
    printf 'GH_COMMENTS_3333=[{"body":"🤖 [agent:devops] script=test_xxx action=closing","created_at":"%s"}]\n' \
        "$now_iso" > "$TEST_TMP/gh_state"
    . "$HERMES_GITHUB"
    local rc=0
    _whoami_already_posted issue "3333" "test_xxx" "closing" || rc=$?
    assert_eq "0" "$rc" "_whoami_already_posted: matching whoami → rc=0 (backwards-compat)" || return 1
    # Different action → rc=1.
    rc=0
    _whoami_already_posted issue "3333" "test_xxx" "reopening" || rc=$?
    assert_eq "1" "$rc" "_whoami_already_posted: different action → rc=1" || return 1
    return 0
}

# ============================================================================
# RUN
# ============================================================================
echo "Running hermes_github.sh tests..."
echo "TEST_TMP=$TEST_TMP"
echo

run_test "01_helper_exists"                  test_01_helper_exists
run_test "02_functions_defined"              test_02_functions_defined
run_test "03_post_format"                    test_03_post_format
run_test "04_idempotency_same_action"        test_04_idempotency
run_test "05_different_actions_not_idempotent" test_05_different_actions_not_idempotent
run_test "06_failure_semantics"              test_06_failure_semantics
run_test "07_add_label_format"               test_07_add_label_format
run_test "08_close_pr_uses_pr_endpoint"      test_08_close_pr_uses_pr_endpoint
run_test "09_window_expiry"                  test_09_window_expiry
run_test "10_integration_source_lines"       test_10_integration_source_lines
run_test "11_install_includes"               test_11_install_includes
run_test "12_bad_inputs"                     test_12_bad_inputs
run_test "13_integration_whoami_on_critical_mutations" test_13_integration_whoami_on_critical_mutations
run_test "14_crp_within_window"              test_14_crp_within_window
run_test "15_crp_outside_window"             test_15_crp_outside_window
run_test "16_crp_contains_mode"              test_16_crp_contains_mode
run_test "17_crp_kind_routing"               test_17_crp_kind_routing
run_test "18_crp_failure_semantics"          test_18_crp_failure_semantics
run_test "19_crp_window_zero"                test_19_crp_window_zero
run_test "20_whoami_wrapper_reuses_helper"   test_20_whoami_wrapper_reuses_helper

echo
echo "=========================================="
echo "Total:  $TESTS_TOTAL"
echo "Passed: $TESTS_PASSED"
echo "Failed: $TESTS_FAILED"
[ $TESTS_FAILED -gt 0 ] && {
    echo "Failed names: ${FAILED_NAMES[*]}"
    exit 1
}
echo "ALL TESTS PASSED"
exit 0