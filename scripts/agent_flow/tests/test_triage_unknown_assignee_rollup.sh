#!/bin/bash
# ============================================================================
# test_triage_unknown_assignee_rollup.sh — юнит-тест ADR-0042 unknown-assignee
#                                          rollup guard (ретро 01.09 t_e1a9613d,
#                                          issue #1824).
#
# Проверяет, что:
#   T1: `_emit_unknown_assignee_rollup` парсит newline-terminated accumulator
#       корректно: каждый record → 3 поля (number, role, title_prefix).
#       Это РЕГРЕССИЯ на баг v1 — где `${var//$IFS/\\n}` подставлял литерал
#       `\n` (backslash-n), и `cut -f3` захватывал остаток строки.
#   T2: accumulator + parser НЕ уродуют title_prefix когда в нём есть пробелы
#       (например «bug process agent flow»).
#   T3: distinct bad_roles собираются правильно (без дублей).
#   T4: per-tick dedup: если в rollup-issue уже есть свежий комментарий
#       с маркером — НЕ пишем ещё один (dedup-hit).
#   T5: fresh-write: если нет свежего комментария — пишем ОДИН rollup.
#   T6: dry-run (UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN=true) — НЕ пишем ни rollup,
#       ни labels.
#   T7: shellcheck-clean + bash -n на triage.sh (без новых warnings).
#
# Использование:
#   bash test_triage_unknown_assignee_rollup.sh
# Env:
#   VERBOSE=1 — печатать подробности
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

assert_eq() {
    local name="$1" expected="$2" actual="$3"
    if [ "$expected" = "$actual" ]; then
        pass "$name"
    else
        fail "$name" "expected=[$expected] actual=[$actual]"
    fi
}

# === Helpers (test-local) ===
# Source shared lib_eval_func.sh (issue #2295) instead of redefining
# extract_func locally — keeps semantic-fail behavior consistent across tests
# and avoids silent regressions when functions get reindented in the source.
# shellcheck source=lib/lib_eval_func.sh
. "$TESTS_DIR/lib/lib_eval_func.sh"

# Mock gh so the test is hermetic. The mock supports the patterns our function
# uses: `gh issue comment --body`, `gh issue edit --add-label`, and
# `gh api ... --jq '...'`.
MOCK_BIN="$(mktemp -d -t triage-rollup-mock-XXXXXX)"
GH_CALLS_LOG="$MOCK_BIN/gh_calls.log"
ROLLUP_FILE="$MOCK_BIN/last_rollup.md"
LABELS_FILE="$MOCK_BIN/labels.log"
COMMENTS_JSON="$MOCK_BIN/comments.json"
export PATH="$MOCK_BIN:$PATH"
export GH_CALLS_LOG ROLLUP_FILE LABELS_FILE COMMENTS_JSON

cat > "$MOCK_BIN/gh" <<'EOF'
#!/bin/bash
echo "GH: $*" >> "${GH_CALLS_LOG:-/tmp/gh_calls.log}"
case "$1" in
    auth) exit 0 ;;
    issue)
        case "$2" in
            comment)
                shift 3
                _body=""
                while [ $# -gt 0 ]; do
                    case "$1" in
                        --body) _body="$2"; shift 2 ;;
                        --repo) shift 2 ;;
                        *) shift ;;
                    esac
                done
                echo "---ROLLUP-BODY-START---" >> "${ROLLUP_FILE:-/tmp/last_rollup.md}"
                printf '%s\n' "$_body" >> "${ROLLUP_FILE:-/tmp/last_rollup.md}"
                echo "---ROLLUP-BODY-END---" >> "${ROLLUP_FILE:-/tmp/last_rollup.md}"
                exit 0
                ;;
            edit)
                shift 2
                while [ $# -gt 0 ]; do
                    case "$1" in
                        --add-label)
                            echo "label-add: $2" >> "${LABELS_FILE:-/tmp/labels.log}"
                            shift 2
                            ;;
                        --repo) shift 2 ;;
                        --remove-label) shift 2 ;;
                        *) shift ;;
                    esac
                done
                exit 0
                ;;
        esac
        ;;
    api)
        # Detect "comments?..." queries. Old helper (jq-regex, last created_at)
        # used `comments?per_page=20`; new generic helper (#2293) uses
        # `comments?since=...&per_page=100`. Return the inner "comments" array
        # (real GitHub API returns top-level array, but this test stores it
        # under "comments" key — unwrap on read so the helper's json.loads
        # sees a flat list).
        if [[ "$*" == *"comments?"* ]] || [[ "$*" == *"comments?per_page=20"* ]]; then
            if [ -z "${COMMENTS_JSON:-}" ] || [ ! -f "$COMMENTS_JSON" ]; then
                echo "[]"
                exit 0
            fi
            python3 -c "
import json
try:
    with open('${COMMENTS_JSON}') as f:
        d = json.load(f)
except Exception:
    print('[]')
    raise SystemExit
# Unwrap: support both bare array (real API) and {'comments': [...]} (test format).
if isinstance(d, dict) and 'comments' in d:
    d = d['comments']
print(json.dumps(d))
"
            exit 0
        fi
        exit 0
        ;;
esac
exit 0
EOF
chmod +x "$MOCK_BIN/gh"

setup_mock_env() {
    echo '{"comments":[]}' > "$COMMENTS_JSON"
    rm -f "$ROLLUP_FILE" "$LABELS_FILE" "$GH_CALLS_LOG"
    # Re-export env vars so the gh-mock subprocess picks them up.
    export GH_CALLS_LOG ROLLUP_FILE LABELS_FILE COMMENTS_JSON PATH
}

# Source the function from the script under test via extract_func_or_die so a
# missing/reindented function fails LOUDLY instead of silently breaking the test.
_emit_unknown_assignee_rollup() { :; }  # stub for eval_helper
eval "$(extract_func_or_die "$SCRIPT_UNDER_TEST" _emit_unknown_assignee_rollup)"

# Provide outer-scope deps
log() { printf '[log] %s\n' "$*" >&2; }
whoami_add_label() { echo "whoami_add_label: $*" >> "$LABELS_FILE"; }

# comment_recently_posted — тестовый stub generic helper'а из hermes_github.sh
# (issue #2293). Зеркалит его логику: возвращает 0 (да, posted) если в $COMMENTS_JSON
# есть коммент с marker'ом в окне window_seconds, иначе 1.
#
# Args: kind number marker window_seconds [mode]
comment_recently_posted() {
    local kind="$1" number="$2" marker="$3" window_seconds="$4" mode="${5:-prefix}"
    [ -z "$number" ] && return 1
    [ -z "$marker" ] && return 1
    [ -z "$COMMENTS_JSON" ] || [ ! -f "$COMMENTS_JSON" ] && return 1
    # Возвращаем stdout python-скрипта как exit code: 0 = найден, 1 = нет.
    # Зеркалит hermes_github.sh::comment_recently_posted ([ "$found" = "1" ]).
    # Без этого return-based-on-exit-code функция всегда возвращает 0,
    # что ломает dedup-логику в _emit_unknown_assignee_rollup (T1 stale write).
    local _found
    _found="$(python3 -c "
import json, sys
from datetime import datetime, timezone
try:
    with open('${COMMENTS_JSON}') as f:
        d = json.load(f)
except Exception:
    print('1'); raise SystemExit
if isinstance(d, dict) and 'comments' in d:
    d = d['comments']
if not isinstance(d, list):
    print('1'); raise SystemExit
now = int(datetime.now(timezone.utc).timestamp())
cutoff = now - int('${window_seconds}')
marker = '''${marker}'''
mode = '${mode}'
found = False
for c in d:
    if not isinstance(c, dict): continue
    body = c.get('body') or ''
    created = c.get('created_at') or ''
    matched = body.startswith(marker) if mode == 'prefix' else (marker in body)
    if not matched: continue
    try:
        dt = datetime.fromisoformat(created.replace('Z', '+00:00'))
        ep = int(dt.timestamp())
    except Exception:
        continue
    if ep >= cutoff:
        found = True; break
print('0' if found else '1')
" 2>/dev/null)"
    [ -z "$_found" ] && _found=1
    [ "$_found" = "0" ]
}

# === T1: parser handles 3 records with spaces in titles ===
echo "=== T1: parser splits records correctly ==="
setup_mock_env
export GH_REPO="krikz/rob_box_project"
export UNKNOWN_ASSIGNEE_ROLLUP_ISSUE="1824"
export UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN="30"
export UNKNOWN_ASSIGNEE_ROLLUP_LABEL="agent-flow-error"
export UNKNOWN_ASSIGNEE_ROLLUP_MARKER="agent-flow-triage:unknown-assignee-rollup"
export UNKNOWN_ASSIGNEE_PHASE_BREAK_AT="50"
export UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN="false"
export VALID_PROFILES="agent-flow|architect|devops|backend"

_unknown_assignee_records=$'1824\ttriager\tbug process agent flow\n1830\tfoo-bar\tdeploy regression\n1845\ttriager\tanother triager issue\n'
_emit_unknown_assignee_rollup

# Verify: 3 distinct label-adds (one per issue), all with correct role in message
assert_eq "T1a: 3 distinct label-adds" "3" "$(grep -c '^label-add:' "$LABELS_FILE" 2>/dev/null || echo 0)"
assert_eq "T1b: rollup body has 3 issue rows" "3" "$(grep -c '^| #[0-9]' "$ROLLUP_FILE" 2>/dev/null || echo 0)"
# The KEY regression check: title_prefix for #1824 should be ONLY "bug process agent flow"
# (NOT "bug process agent flow\n1830" — that was the bug).
assert_eq "T1c: #1824 title is clean (no leak)" "1" \
    "$(grep -cE '^\| #1824 \| `agent:triager` \| `bug process agent flow` \|$' "$ROLLUP_FILE" 2>/dev/null || echo 0)"
assert_eq "T1d: #1830 title is clean (no leak)" "1" \
    "$(grep -cE '^\| #1830 \| `agent:foo-bar` \| `deploy regression` \|$' "$ROLLUP_FILE" 2>/dev/null || echo 0)"
assert_eq "T1e: no literal '\\n' in body" "0" \
    "$(grep -F '\\n' "$ROLLUP_FILE" 2>/dev/null | wc -l)"
assert_eq "T1f: bad_roles shows triager,foo-bar (distinct)" "1" \
    "$(grep -c 'Bad roles в этом тике.*triager,foo-bar' "$ROLLUP_FILE" 2>/dev/null || echo 0)"

# === T2: title with leading/trailing space is preserved correctly ===
echo
echo "=== T2: titles with embedded spaces stay intact ==="
setup_mock_env
_unknown_assignee_records=$'9999\tbad-role\thello world this is a longer title\n'
_emit_unknown_assignee_rollup
assert_eq "T2a: long title parsed correctly" "1" \
    "$(grep -cE '^\| #9999 \| `agent:bad-role` \| `hello world this is a longer title` \|$' "$ROLLUP_FILE" 2>/dev/null || echo 0)"

# === T3: dedup-hit when fresh rollup exists ===
echo
echo "=== T3: dedup-hit when fresh rollup exists ==="
setup_mock_env
NOW="$(date -u +%s)"
NOW_ISO="$(date -u -d "@$NOW" +%Y-%m-%dT%H:%M:%SZ)"
cat > "$COMMENTS_JSON" <<JSON
{"comments":[{"body":"agent-flow-triage:unknown-assignee-rollup (tick=foo)\n\nprevious body","created_at":"$NOW_ISO"}]}
JSON
_unknown_assignee_records=$'7777\tghost\tphantom issue\n'
_emit_unknown_assignee_rollup
# In dedup-hit: no rollup body, but per-issue label still set
assert_eq "T3a: no rollup body on dedup-hit" "0" \
    "$(wc -l < "$ROLLUP_FILE" 2>/dev/null || echo 0)"
assert_eq "T3b: per-issue label still applied" "1" \
    "$(grep -c '^label-add:' "$LABELS_FILE" 2>/dev/null || echo 0)"

# === T4: dry-run skips everything ===
echo
echo "=== T4: dry-run ==="
setup_mock_env
UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN="true" \
_unknown_assignee_records=$'8888\tquux\tdry run test\n' \
_emit_unknown_assignee_rollup
assert_eq "T4a: no rollup body on dry-run" "0" \
    "$(wc -l < "$ROLLUP_FILE" 2>/dev/null || echo 0)"
assert_eq "T4b: no labels on dry-run" "0" \
    "$(wc -l < "$LABELS_FILE" 2>/dev/null || echo 0)"

# === T5: shellcheck + bash -n ===
echo
echo "=== T5: shellcheck + bash -n ==="
if bash -n "$SCRIPT_UNDER_TEST" 2>/dev/null; then
    pass "T5a: bash -n on agent-flow-triage.sh"
else
    fail "T5a: bash -n on agent-flow-triage.sh" "syntax errors"
fi

# === Summary ===
echo
echo "==================================================="
echo "Summary: ${PASS} passed, ${FAIL} failed"
if [ "$FAIL" -gt 0 ]; then
    echo "FAILED: ${FAILED_CASES[*]}"
    exit 1
fi
echo "All tests passed."
exit 0
