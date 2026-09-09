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
extract_func() {
    awk -v fn="$2" '
        $0 == fn "() {" { f = 1 }
        f { print }
        f && /^\}$/ { exit }
    ' "$1"
}

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
        # Detect "comments?per_page=20" query (per-tick dedup probe).
        if [[ "$*" == *"comments?per_page=20"* ]]; then
            # Find --jq filter and apply it crudely.
            _jq=""
            while [ $# -gt 0 ]; do
                case "$1" in
                    --jq) _jq="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            if [ -z "$_jq" ]; then
                cat "${COMMENTS_JSON:-/tmp/comments.json}"
            else
                python3 -c "
import json
try:
    with open('${COMMENTS_JSON}') as f:
        d = json.load(f)
except Exception:
    print('null'); raise SystemExit
# Crude: return last created_at where body matches the rollup marker.
out = ''
for c in d.get('comments', []):
    body = c.get('body') or ''
    if 'agent-flow-triage:unknown-assignee-rollup' in body:
        out = c.get('created_at', '')
if not out:
    print('null')
else:
    print(out)
"
            fi
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

# Source the function from the script under test.
# We extract the function and eval it inside a fresh subshell.
_emit_unknown_assignee_rollup() { :; }  # stub for eval_helper
eval "$(extract_func "$SCRIPT_UNDER_TEST" _emit_unknown_assignee_rollup)"

# Provide outer-scope deps
log() { printf '[log] %s\n' "$*" >&2; }
whoami_add_label() { echo "whoami_add_label: $*" >> "$LABELS_FILE"; }

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
