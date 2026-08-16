#!/bin/bash
# ============================================================================
# test_merge_gate_post_merge.sh — ADR-0014 acceptance tests
#
# Mocked-GitHub tests for the post-merge close logic in
# scripts/agent_flow/agent-flow-merge-gate.sh.
#
# Strategy: replace `gh` and `hermes` with shell scripts that read
# fixture state from a per-test directory and record every call to a
# journal file. Each scenario (A..H) is one shell-script test that:
#   1. Writes fixture state describing the issue / PR / labels.
#   2. Sources this lib to install mocks.
#   3. Runs merge-gate in DRY_RUN=false against a fake repo.
#   4. Asserts against the journal what close / label / branch ops ran.
#
# Tests must NOT touch real GitHub state — they only assert side effects
# captured in the journal.
#
# Invocation:
#   bash tests/test_merge_gate_post_merge.sh
# Returns exit 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

# Anchor: tests/ directory
TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT_DIR="$(cd "$TEST_LIB_DIR/.." && pwd)"
REPO_ROOT="$(cd "$TEST_ROOT_DIR/.." && pwd)"
MERGE_GATE="$REPO_ROOT/agent-flow-merge-gate.sh"

# Per-run temp dir
TEST_TMP="${TEST_TMP:-/tmp/agent-flow-merge-gate-tests.$$}"
mkdir -p "$TEST_TMP"

# Colors (if attached to a terminal)
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BLU=$'\033[34m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; BLU=''; END=''
fi

# --- Test registry ----------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {  # $1=name, $2=function
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '%s[ RUN     ]%s %s\n' "$BLU" "$END" "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '%s[   PASS  ]%s %s\n' "$GRN" "$END" "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '%s[   FAIL  ]%s %s\n' "$RED" "$END" "$name"
    fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        printf '  %sassert fail:%s %s\n    expected: %q\n    actual:   %q\n' \
            "$RED" "$END" "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  %sassert fail:%s %s\n    needle:   %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

assert_not_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*)
            printf '  %sassert fail:%s %s\n    needle should NOT appear: %q\n    haystack: %q\n' \
                "$RED" "$END" "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
    return 0
}

# --- Install mocks ----------------------------------------------------------
# Creates $TEST_TMP/bin with stubs for `gh` and `hermes` that read state
# from $TEST_TMP/gh_state and append every invocation to $TEST_TMP/journal.
install_mocks() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"

    # Mock `gh` ---------------------------------------------------------
    cat > "$bin_dir/gh" <<'GH_MOCK_EOF'
#!/bin/bash
# State file path: $GH_STATE
# Journal file: $GH_JOURNAL
# Each call appends one line: TIMESTAMP\t<command summary>
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"

# Helper: pull a JSON field from state (very simple)
# State format: key=value lines; for arrays/lists, use the jq-subset below.
# Use '@' as sed delimiter because keys may contain '/' (e.g. branch
# names like "z-{agent}/1082-followup-demo").
get_state() {
    local key="$1"
    if [ -f "$state" ]; then
        grep -E "^${key}=" "$state" | head -n1 | sed "s@^${key}=@@"
    fi
}

# Helper: append to journal
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }

# Helper: emit JSON value or empty
emit_json() { printf '%s' "$1"; }

# Apply a tiny subset of --jq filter so callers can use --jq '[...]' to
# narrow the result. Supported patterns (substr-based, very forgiving):
#   - '[.[] | select(.EVENT == "VAL")][-1].field'   — last matching element,
#     then optional '.field' extraction. Used by issue timeline lookup.
#   - '[.[] | select(.a == "A" or .b == "B")][0]'   — first matching element.
#   - '[.[] | select(.field >= "DATE")] | length'   — count of matching items.
#   - '.field.subfield'                              — simple field extraction.
# Anything else is returned as-is (caller will deal with raw JSON).
apply_jq() {
    local data="$1" filter="$2"
    if [ -z "$filter" ] || [ -z "$data" ]; then printf '%s' "$data"; return; fi
    python3 -c '
import sys, json, re
data_raw = sys.argv[1]
filt = sys.argv[2]
try:
    data = json.loads(data_raw)
except Exception:
    print(data_raw); sys.exit(0)

def emit(v):
    if v is None: print("null"); return
    if isinstance(v, (dict, list)): print(json.dumps(v)); return
    print(v); return

# Pattern: [.[] | select(.EVENT == "VAL")][-1].field
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w.]+)\s*==\s*\"([^\"]+)\"\)\]\[-1\](?:\.([\w]+))?$", filt)
if m:
    field, val, sub = m.group(1), m.group(2), m.group(3)
    for el in reversed(data):
        v = el
        for part in field.split("."):
            if isinstance(v, dict): v = v.get(part)
            else: v = None
        if v == val:
            if sub and isinstance(el, dict):
                emit(el.get(sub))
            else:
                emit(el)
            sys.exit(0)
    print("null"); sys.exit(0)

# Pattern: [.[] | select(.EVENT == "VAL" and .OTHER == "VAL2")][-1].field
# (used by issue timeline lookup for label events)
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w]+)\s*==\s*\"([^\"]+)\"\s+and\s+\.([\w.]+)\s*==\s*\"([^\"]+)\"\)\]\[-1\](?:\.([\w]+))?$", filt)
if m:
    f1, v1, f2, v2, sub = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
    for el in reversed(data):
        a = el.get(f1) if isinstance(el, dict) else None
        b = el
        for part in f2.split("."):
            if isinstance(b, dict): b = b.get(part)
            else: b = None
        if a == v1 and b == v2:
            if sub and isinstance(el, dict):
                emit(el.get(sub))
            else:
                emit(el)
            sys.exit(0)
    print("null"); sys.exit(0)

# Pattern: [.[] | select(.a == "A" or .b == "B")][0]
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w]+)\s*==\s*\"([^\"]+)\"\s+or\s+\.([\w]+)\s*==\s*\"([^\"]+)\"\)\]\[0\]$", filt)
if m:
    f1, v1, f2, v2 = m.group(1), m.group(2), m.group(3), m.group(4)
    for el in data:
        if el.get(f1) == v1 or el.get(f2) == v2:
            emit(el); sys.exit(0)
    print("null"); sys.exit(0)

# Pattern: [.[] | select(.field >= "DATE")] | length
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w.]+)\s*>=\s*\"([^\"]+)\"\)\]\s*\|\s*length$", filt)
if m:
    field, val = m.group(1), m.group(2)
    count = 0
    for el in data:
        v = el
        for part in field.split("."):
            if isinstance(v, dict): v = v.get(part)
            else: v = None
        if isinstance(v, str) and v >= val: count += 1
    print(count); sys.exit(0)

# Pattern: [.field[].subfield] | join("SEP")
m = re.match(r"^\[\.([\w]+)\[\]\.([\w]+)\]\s*\|\s*join\(\"([^\"]+)\"\)$", filt)
if m:
    outer, inner, sep = m.group(1), m.group(2), m.group(3)
    coll = data.get(outer) if isinstance(data, dict) else None
    vals = []
    for sub in (coll or []):
        cur = sub
        for part in inner.split("."):
            if isinstance(cur, dict): cur = cur.get(part)
            else: cur = None
        if cur is not None: vals.append(str(cur))
    print(sep.join(vals)); sys.exit(0)

# Pattern: [.field[].subfield] | .[]  — iterate, print each value on its own
# line (used by merge-gate assignee lookup: --jq with .labels[].name).
m = re.match(r"^\[\.([\w]+)\[\]\.([\w]+)\]\s*\|\s*\.\[\]$", filt)
if m:
    outer, inner = m.group(1), m.group(2)
    coll = data.get(outer) if isinstance(data, dict) else None
    for sub in (coll or []):
        cur = sub
        for part in inner.split("."):
            if isinstance(cur, dict): cur = cur.get(part)
            else: cur = None
        if cur is not None: emit(cur)
    sys.exit(0)

# Pattern: [.[] | select(.field | startswith("PREFIX"))] | length
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w]+)\s*\|\s*startswith\(\"([^\"]+)\"\)\)\]\s*\|\s*length$", filt)
if m:
    field, prefix = m.group(1), m.group(2)
    count = 0
    for el in data:
        v = el.get(field) if isinstance(el, dict) else None
        if isinstance(v, str) and v.startswith(prefix): count += 1
    print(count); sys.exit(0)

# Pattern: [.[] | select(.field | contains("SUBSTR"))] | length
# (orphan-comment dedup, ретро 13.08 t_0b76514f: contains-подстрока тела,
#  не startswith — иначе префикс не совпадает с реальным телом коммена.)
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w]+)\s*\|\s*contains\(\"([^\"]+)\"\)\)\]\s*\|\s*length$", filt)
if m:
    field, substr = m.group(1), m.group(2)
    count = 0
    for el in data:
        v = el.get(field) if isinstance(el, dict) else None
        if isinstance(v, str) and substr in v: count += 1
    print(count); sys.exit(0)

# Pattern: .field(.subfield)*
if filt.startswith("."):
    # Special: .[0].field // "default" — used by stale-branch guard
    # (gh pr list --state merged --head X --jq .[0].number // empty).
    m0 = re.match(r"^\.\[0\]\.([\w]+)\s*//\s*\"([^\"]*)\"$", filt)
    if m0:
        field, default = m0.group(1), m0.group(2)
        if isinstance(data, list) and data and isinstance(data[0], dict) and data[0].get(field) is not None:
            emit(data[0].get(field))
        else:
            print(default)
        sys.exit(0)
    # Special: .a[].b  — iterate a, print b for each (each on new line)
    m2 = re.match(r"^\.([\w]+)\[\]\.(.+)$", filt)
    if m2:
        outer, inner = m2.group(1), m2.group(2)
        # `data` here is usually an object like {"comments":[...]}; take the
        # array from the outer field and iterate it.
        coll = data.get(outer) if isinstance(data, dict) else None
        for sub in (coll or []):
            cur = sub
            for part in inner.split("."):
                if isinstance(cur, dict): cur = cur.get(part)
                else: cur = None
            emit(cur)
        sys.exit(0)
    # Plain: .a.b.c
    v = data
    for part in filt[1:].split("."):
        if not part: continue
        if isinstance(v, dict): v = v.get(part)
        else: v = None
    emit(v); sys.exit(0)

# Pattern: [.[] | select(.field == "VAL")] | length  (equality count — retro-path)
m = re.match(r"^\[\.\[\]\s*\|\s*select\(\.([\w]+)\s*==\s*\"([^\"]+)\"\)\]\s*\|\s*length$", filt)
if m:
    field, val = m.group(1), m.group(2)
    count = 0
    for el in data:
        v = el.get(field) if isinstance(el, dict) else None
        if isinstance(v, str) and v == val: count += 1
    print(count); sys.exit(0)

# Pattern: [.[] | select(.a == "X" or .b == "Y" or .c == "Z")] | length
# (multi-condition OR count — retro-path CI-only rollup check)
m = re.match(r"^\[\.\[\]\s*\|\s*select\((.+?)\)\]\s*\|\s*length$", filt)
if m:
    conds = re.findall(r"\.([\w]+)\s*==\s*\"([^\"]+)\"", m.group(1))
    count = 0
    for el in data:
        if not isinstance(el, dict): continue
        for field, val in conds:
            if el.get(field) == val:
                count += 1
                break
    print(count); sys.exit(0)

# Pattern: [.field[] | select(.a == "X" or .b == "Y")] | length
# (multi-condition OR count inside an OBJECT field — ретро 12.08 t_061d466e.
#  gh pr view --json statusCheckRollup returns {"statusCheckRollup":[...]},
#  so the real filter dereferences the field; iterate its array.)
m = re.match(r"^\[\.([\w]+)\[\]\s*\|\s*select\((.+?)\)\]\s*\|\s*length$", filt)
if m:
    outer, conds = m.group(1), re.findall(r"\.([\w]+)\s*==\s*\"([^\"]+)\"", m.group(2))
    coll = data.get(outer) if isinstance(data, dict) else None
    count = 0
    for el in (coll or []):
        if not isinstance(el, dict): continue
        for field, val in conds:
            if el.get(field) == val:
                count += 1
                break
    print(count); sys.exit(0)

# Pattern: [.files[].path]  (array of subfield values — retro-path PR files)
m = re.match(r"^\[\.([\w]+)\[\]\.([\w]+)\]$", filt)
if m:
    outer, inner = m.group(1), m.group(2)
    coll = data.get(outer) if isinstance(data, dict) else None
    vals = []
    for sub in (coll or []):
        cur = sub
        for part in inner.split("."):
            if isinstance(cur, dict): cur = cur.get(part)
            else: cur = None
        if cur is not None: vals.append(cur)
    emit(vals); sys.exit(0)

# Default: pass through
emit(data)
' "$data" "$filter"
}

# Extract --jq <filter> from remaining args.
# Outputs the filter (empty string if none) on stdout, rest of args
# remain in the caller's positional params. We use a temp file so we
# don't lose IFS / quoting.
_jq_filter=""
_rest_args=()
_collect=0
for _arg in "$@"; do
    if [ "$_collect" = "1" ]; then
        _jq_filter="$_arg"; _collect=0; continue
    fi
    case "$_arg" in
        --jq) _collect=1 ;;
        --jq=*) _jq_filter="${_arg#--jq=}" ;;
        *) _rest_args+=("$_arg") ;;
    esac
done
# Replace the caller's positional params with the de-jq'd args.
set -- "${_rest_args[@]}"
# _jq_filter is now in this scope.

# Parse subcommand (first arg)
subcmd="${1:-}"; shift || true

case "$subcmd" in
    auth)
        journal "gh auth status"
        exit 0
        ;;
    issue)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                journal "gh issue list"
                # Ретро 15.08 t_238ff3f7: deploy-issue reconcile запрашивает
                # `--label deployment` (отдельный список). Если флаг есть —
                # берём ISSUE_LIST_DEPLOYMENT_JSON (fallback ISSUE_LIST_JSON).
                if printf '%s' "$*" | grep -q -- '--label deployment'; then
                    _data="$(get_state ISSUE_LIST_DEPLOYMENT_JSON)"
                    [ -n "$_data" ] || _data="$(get_state ISSUE_LIST_JSON)"
                else
                    _data="$(get_state ISSUE_LIST_JSON)"
                fi
                apply_jq "$_data" "$_jq_filter"
                ;;
            view)
                issue_num="$1"; shift || true
                # Detect: --comments present?
                if printf '%s' "$*" | grep -q -- '--comments'; then
                    journal "gh issue view $issue_num --comments"
                    _data="$(get_state ISSUE_${issue_num}_COMMENTS_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q -- 'labels'; then
                    journal "gh issue view $issue_num --json labels"
                    _data="$(get_state ISSUE_${issue_num}_LABELS_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q 'state'; then
                    journal "gh issue view $issue_num --json state"
                    _data="$(get_state ISSUE_${issue_num}_STATE_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                else
                    journal "gh issue view $issue_num (other)"
                    _data="$(get_state ISSUE_${issue_num}_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                fi
                ;;
            edit)
                issue_num="$1"; shift || true
                if printf '%s' "$*" | grep -q -- '--add-label'; then
                    label="$(printf '%s' "$*" | sed -nE 's/.*--add-label[[:space:]]+([^ ]+).*/\1/p')"
                    journal "gh issue edit $issue_num --add-label $label"
                    # Update state: add label to ISSUEN_LABELS_JSON
                    if [ -f "$state" ]; then
                        key="ISSUE_${issue_num}_LABELS_JSON"
                        current="$(get_state "$key")"
                        # Naïve update: append "label" inside the JSON array.
                        # For test purposes we use a fixed JSON shape:
                        #   {"labels":[{"name":"L1"},...]}  → just re-emit a
                        # state-machine-friendly replacement.
                        printf '%s' "$current" | grep -q "\"$label\"" && exit 0
                        # Insert before closing brace of labels array.
                        local updated
                        updated="$(printf '%s' "$current" \
                            | sed -E "s/(\"name\":\"[^\"]+\"\\])(,*)/\\1,{\"name\":\"${label}\"}\\2/")"
                        if [ "$updated" = "$current" ]; then
                            # Fallback: replace empty array
                            updated="$(printf '%s' "$current" \
                                | sed -E "s/\\[\\]/[{\"name\":\"${label}\"}]/")"
                        fi
                        # Persist via tmp swap
                        tmpf="${state}.tmp.$$"
                        grep -v "^${key}=" "$state" >"$tmpf" || true
                        printf '%s=%s\n' "$key" "$updated" >>"$tmpf"
                        mv "$tmpf" "$state"
                    fi
                elif printf '%s' "$*" | grep -q -- '--remove-label'; then
                    label="$(printf '%s' "$*" | sed -nE 's/.*--remove-label[[:space:]]+([^ ]+).*/\1/p')"
                    journal "gh issue edit $issue_num --remove-label $label"
                else
                    journal "gh issue edit $issue_num (other: $*)"
                fi
                ;;
            close)
                issue_num="$1"; shift || true
                # Honor GH_CLOSE_FAIL_ISSUE_<n>=1 to force failure.
                if [ "$(get_state "GH_CLOSE_FAIL_ISSUE_${issue_num}")" = "1" ]; then
                    journal "gh issue close $issue_num --reason completed (FORCED FAIL)"
                    echo "simulated close failure" >&2
                    exit 1
                fi
                journal "gh issue close $issue_num --reason completed"
                # Update issue state to CLOSED + flip ISSUE_n_STATE_JSON.
                if [ -f "$state" ]; then
                    key="ISSUE_${issue_num}_STATE_JSON"
                    tmpf="${state}.tmp.$$"
                    grep -v "^${key}=" "$state" >"$tmpf" || true
                    printf '%s=%s\n' "$key" '{"state":"CLOSED"}' >>"$tmpf"
                    mv "$tmpf" "$state"
                fi
                ;;
            comment)
                issue_num="$1"; shift || true
                # Journal the body so tests can assert on the cleanup-comment
                # text (e.g. "Issue закрыта"). Real invocation:
                #   gh issue comment <n> --repo <repo> --body <text>
                _body="$(printf '%s' "$*" | sed -nE 's/.*--body[[:space:]]+(.*)/\1/p')"
                journal "gh issue comment $issue_num --body $_body"
                ;;
            *)
                journal "gh issue $action (other)"
                ;;
        esac
        ;;
    pr)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                # Detect: --head <branch> (PR lookup / stale-branch guard) vs
                # --state merged (retro-path scan) vs --search vs scan-all.
                # ВАЖНО: --head проверяется ПЕРВЫМ — guard вызывает
                # `--state merged --head <branch>`, и такой запрос должен уйти
                # в PR_MERGED_HEAD_* (см. ниже), а НЕ в ретро-ветку
                # PR_LIST_MERGED_JSON (иначе merged PR не находится и
                # stale-branch guard молчит). Ретро-путь зовёт `--state merged`
                # без `--head` → попадёт во вторую ветку.
                if printf '%s' "$*" | grep -q -- '--head'; then
                    head_branch="$(printf '%s' "$*" | sed -nE 's/.*--head[[:space:]]+([^ ]+).*/\1/p')"
                    # Ретро 12.08 t_d3aeaa9b: stale-branch guard запрашивает
                    # `--state merged --head <branch>`. Отдельный ключ
                    # PR_MERGED_HEAD_<branch>_JSON (fallback: PR_HEAD_*), чтобы
                    # тесты могли симулировать «ветка уже влита».
                    if printf '%s' "$*" | grep -q -- '--state merged'; then
                        journal "gh pr list --state merged --head $head_branch"
                        _data="$(get_state PR_MERGED_HEAD_${head_branch}_JSON)"
                        if [ -z "$_data" ]; then
                            _data="$(get_state PR_HEAD_${head_branch}_JSON)"
                        fi
                    else
                        journal "gh pr list --head $head_branch"
                        _data="$(get_state PR_HEAD_${head_branch}_JSON)"
                    fi
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q -- '--state merged'; then
                    journal "gh pr list --state merged (retro-path)"
                    _data="$(get_state PR_LIST_MERGED_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q -- '--search'; then
                    journal "gh pr list --search (followup)"
                    _data="$(get_state PR_FOLLOWUP_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                else
                    journal "gh pr list --state open (scan-all)"
                    _data="$(get_state PR_LIST_ALL_OPEN_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                fi
                ;;
            view)
                pr_num="$1"; shift || true
                if printf '%s' "$*" | grep -q -- '--json files'; then
                    journal "gh pr view $pr_num --json files"
                    _data="$(get_state PR_${pr_num}_FILES_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q -- '--json statusCheckRollup'; then
                    journal "gh pr view $pr_num --json statusCheckRollup"
                    _data="$(get_state PR_${pr_num}_ROLLUP_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                elif printf '%s' "$*" | grep -q -- '--json number'; then
                    # Ретро-путь guard PR/issue (13.08, надзор): gh pr view N
                    # на не-PR-номере падает с exit 1 — так ведёт себя настоящий
                    # gh. Fixture-флаг PR_EXISTS_<n>=1 означает «это PR».
                    journal "gh pr view $pr_num --json number"
                    if [ "$(get_state PR_EXISTS_${pr_num})" = "1" ]; then
                        printf '{"number":%s}' "$pr_num"
                        exit 0
                    fi
                    echo "simulated: no pull request #$pr_num" >&2
                    exit 1
                else
                    journal "gh pr view $pr_num (other)"
                    _data="$(get_state PR_${pr_num}_VIEW_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                fi
                ;;
            edit)
                journal "gh pr edit $*"
                ;;
            *)
                journal "gh pr $action $*"
                ;;
        esac
        ;;
    run)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                # gh run list --repo X --branch <branch> --workflow ... --json conclusion
                if printf '%s' "$*" | grep -q -- '--branch'; then
                    br="$(printf '%s' "$*" | sed -nE 's/.*--branch[[:space:]]+([^ ]+).*/\1/p')"
                    journal "gh run list --branch $br"
                    _data="$(get_state RUN_LIST_${br}_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                else
                    journal "gh run list (other)"
                    _data="$(get_state RUN_LIST_JSON)"
                    apply_jq "$_data" "$_jq_filter"
                fi
                ;;
            *)
                journal "gh run $action $*"
                ;;
        esac
        ;;
    api)
        # Detect path patterns:
        #   repos/X/Y/issues/N/comments?since=...     → ISSUE_N_COMMENTS_SINCE_JSON
        #   repos/X/Y/issues/N/timeline?...           → ISSUE_N_TIMELINE_JSON
        #   repos/X/Y/pulls/N/commits?...             → PR_N_COMMITS_JSON
        #   repos/X/Y/git/refs/heads/<branch> (DELETE) → journal "delete branch <branch>"
        #   rate_limit                                  → RATE_LIMIT_JSON
        #   repos/X/Y/issues/N/comments (POST = from issue comment) → skip (issue comment goes via `gh issue comment`)
        # The real merge-gate invokes `gh api -X DELETE <path>`; extract the
        # path that follows the optional `-X <METHOD>` prefix.
        if [ "${1:-}" = "-X" ]; then
            _method="$2"
            path="$3"
        else
            path="$1"
        fi
        case "$path" in
            rate_limit)
                journal "gh api rate_limit"
                _data="$(get_state RATE_LIMIT_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/issues/*/comments*)
                issue_num="$(printf '%s' "$path" | sed -nE 's#.*/issues/([0-9]+)/comments.*#\1#p')"
                journal "gh api $path"
                _data="$(get_state ISSUE_${issue_num}_COMMENTS_SINCE_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/issues/*/timeline*)
                issue_num="$(printf '%s' "$path" | sed -nE 's#.*/issues/([0-9]+)/timeline.*#\1#p')"
                journal "gh api $path"
                _data="$(get_state ISSUE_${issue_num}_TIMELINE_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/pulls/*/commits*)
                pr_num="$(printf '%s' "$path" | sed -nE 's#.*/pulls/([0-9]+)/commits.*#\1#p')"
                journal "gh api $path"
                _data="$(get_state PR_${pr_num}_COMMITS_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/pulls/*/files*)
                # Ретро 15.08 t_20383d32: duplicate-file scan тянет
                # pulls/N/files (REST, filename+sha) для детекта двух open PR с
                # идентичным blob. Fixture: PR_<n>_FILES_JSON = JSON-массив
                # [{"filename":"...","sha":"..."}]. Реальный gh api вызывается
                # БЕЗ --jq (merge-gate парсит JSON в python) → apply_jq
                # passthrough вернёт данные как есть.
                pr_num="$(printf '%s' "$path" | sed -nE 's#.*/pulls/([0-9]+)/files.*#\1#p')"
                journal "gh api $path (files)"
                _data="$(get_state PR_${pr_num}_FILES_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/pulls?state=open*)
                # Ретро 15.08 t_2c814334 (pr-orphan-no-labels): REST-based
                # backfill-скан open PR (gh api pulls — core-квота, отдельная
                # от graphql). Fixture: PR_LIST_ALL_OPEN_REST_JSON = массив
                # REST-объектов pull (number, title, head.ref, mergeable,
                # mergeable_state, draft, labels[].name, created_at).
                journal "gh api $path (pulls open REST backfill)"
                _data="$(get_state PR_LIST_ALL_OPEN_REST_JSON)"
                apply_jq "$_data" "$_jq_filter"
                ;;
            repos/*/pulls/[0-9]*)
                # Ретро-путь guard PR/issue (ретро 13.08 t_2d78fbdd, #942):
                # скрипт проверяет существование PR через REST gh api pulls/N
                # (НЕ gh pr view --json number — тот для одного поля number не
                # ходит в API и возвращает success для любого числа, из-за
                # чего guard скипал ВСЕ ретро-issues). Здесь эмулируем REST:
                # 200 + {"number":N} если PR_EXISTS_<n>=1 (это PR), иначе 404.
                pr_num="$(printf '%s' "$path" | sed -nE 's#.*/pulls/([0-9]+).*#\1#p')"
                journal "gh api $path (pulls guard)"
                if [ "$(get_state PR_EXISTS_${pr_num})" = "1" ]; then
                    printf '{"number":%s}' "$pr_num"
                    exit 0
                fi
                echo "simulated: no pull request #$pr_num (HTTP 404)" >&2
                exit 1
                ;;
            repos/*/git/refs/heads/*)
                branch="$(printf '%s' "$path" | sed -nE 's#.*/git/refs/heads/(.+)$#\1#p')"
                journal "gh api -X DELETE $path"
                # Honor GH_BRANCH_DELETE_FAIL_BRANCH_<name>=1
                if [ "$(get_state "GH_BRANCH_DELETE_FAIL_BRANCH_${branch}")" = "1" ]; then
                    echo "simulated branch delete failure" >&2
                    exit 1
                fi
                # Update state: branch removed
                if [ -f "$state" ]; then
                    tmpf="${state}.tmp.$$"
                    grep -v "^BRANCH_PRESENT_${branch}=" "$state" >"$tmpf" || true
                    mv "$tmpf" "$state"
                fi
                ;;
            *)
                journal "gh api $path (other)"
                ;;
        esac
        ;;
    *)
        journal "gh $subcmd $*"
        ;;
esac
GH_MOCK_EOF
    chmod +x "$bin_dir/gh"

    # Mock `git` (only ls-remote used by merge-gate for branch presence) ---
    cat > "$bin_dir/git" <<'GIT_MOCK_EOF'
#!/bin/bash
# Only intercept `git ls-remote --heads <url> <branch>`; the state file
# key BRANCH_PRESENT_<branch> controls whether the branch exists.
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"

case "$1" in
    ls-remote)
        branch="${@: -1}"
        if [ -f "$state" ] && grep -Eq "^BRANCH_PRESENT_${branch}=1$" "$state"; then
            printf '%s\t%s\n' "$(printf '%040x' $RANDOM)" "refs/heads/${branch}"
            exit 0
        fi
        exit 1
        ;;
    *)
        # For anything else, delegate to the real git — but tests should
        # not need it. We just fail loudly.
        echo "mock git: unsupported subcommand: $*" >&2
        exit 1
        ;;
esac
GIT_MOCK_EOF
    chmod +x "$bin_dir/git"

    # Mock `hermes` (used for kanban CLI) ------------------------------
    cat > "$bin_dir/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
journal_file="${GH_JOURNAL:-/dev/null}"
state="${GH_STATE:-}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal_file"; }
journal "hermes $*"
# `kanban --board <b> list --json` → emit JSON array of tasks so the merge-gate
# card_status fallback (kanban_card_status, ретро 12.08 t_8af6bf29) is exercised.
# Fixture key: KANBAN_LIST_JSON (array of {id,status,...}); default [].
if printf '%s' "$*" | grep -q -- ' list '; then
    if [ -f "$state" ]; then
        _v="$(grep -E "^KANBAN_LIST_JSON=" "$state" | head -n1 | sed "s@^KANBAN_LIST_JSON=@@")"
        if [ -n "$_v" ]; then printf '%s' "$_v"; exit 0; fi
    fi
    printf '%s' '[]'
    exit 0
fi
# `kanban --board <b> show <card> --json` → emit card state JSON so the
# merge-gate card_state parse (embedded python) is exercised.
# Fixture key: KANBAN_SHOW_<card_id>_JSON, default {"task":{"status":"done"}}.
if printf '%s' "$*" | grep -q -- ' show '; then
    card_id="$(printf '%s' "$*" | sed -nE 's/.* show ([^ ]+) .*/\1/p')"
    if [ -f "$state" ]; then
        _v="$(grep -E "^KANBAN_SHOW_${card_id}_JSON=" "$state" | head -n1 | sed "s@^KANBAN_SHOW_${card_id}_JSON=@@")"
        if [ -n "$_v" ]; then printf '%s' "$_v"; exit 0; fi
    fi
    # Default: card is done → archive path must fire after successful close.
    printf '%s' '{"task":{"status":"done"}}'
    exit 0
fi
# `kanban --board <b> create ...` → emit "Created t_<id> (ready, ...)" so
# ensure_conflict_recovery_card can parse the new card id (ретро 12.08
# t_8af6bf29). Fixture key: KANBAN_CREATE_ID, default t_recovery.
if printf '%s' "$*" | grep -q -- ' create '; then
    _create_id="$(grep -E '^KANBAN_CREATE_ID=' "$state" 2>/dev/null | head -n1 | sed 's/^KANBAN_CREATE_ID=//')"
    [ -n "$_create_id" ] || _create_id="t_recovery"
    printf 'Created %s  (ready, assignee=default)\n' "$_create_id"
    exit 0
fi
# `kanban --board <b> archive <id>...` → record and succeed (ретро 14.08
# t_36c9ac4e: retro-card-archive pass archives done cards). Journal line
# already written above ("hermes ... archive ..."), assertions grep it.
if printf '%s' "$*" | grep -q -- ' archive '; then
    exit 0
fi
# Simulate success; specific subcommands are recorded for assertions.
HERMES_MOCK_EOF
    chmod +x "$bin_dir/hermes"

    export PATH="$bin_dir:$PATH"
    export GH_STATE="$TEST_TMP/gh_state"
    export GH_JOURNAL="$TEST_TMP/journal"
    : >"$GH_STATE"
    : >"$GH_JOURNAL"
}

# Write fixture key=value pairs (subsequent calls replace the key)
set_state() {  # $1=key $2=value
    local key="$1" val="$2" tmpf
    tmpf="${GH_STATE}.tmp.$$"
    grep -v "^${key}=" "$GH_STATE" >"$tmpf" 2>/dev/null || true
    printf '%s=%s\n' "$key" "$val" >>"$tmpf"
    mv "$tmpf" "$GH_STATE"
}

# Run merge-gate against the current fixture state.
run_merge_gate() {
    (
        # shellcheck disable=SC2034  # many vars are read by sourced script
        GH_REPO="${GH_REPO:-krikz/test-repo}"
        KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
        DRY_RUN="${DRY_RUN:-false}"
        ISSUE_LIMIT="${ISSUE_LIMIT:-50}"
        HERMES_HOME=/tmp/_unused
        HERMES_BIN=hermes  # mocked
        # Ретро 12.08 t_8af6bf29: merge-gate читает kanban-статус напрямую из
        # sqlite (KANBAN_DB), а не через `hermes kanban show` (падает после
        # v0.20.0). В тестах БД недоступна → хелпер фолбэчится на мок hermes
        # show --json (см. bin/hermes ниже). Указываем несуществующий путь,
        # чтобы тесты НЕ читали реальную /home/builder/.hermes/.../kanban.db.
        # ВАЖНО: merge-gate форсит HOME=/home/builder (стр. 46), поэтому
        # переменная ДОЛЖНА называться KANBAN_DB (не KANBAN_DB_PATH — её
        # скрипт не читает, и тест G падал, читая реальную БД: ретро 14.08
        # t_0bd15be9).
        KANBAN_DB="$TEST_TMP/nonexistent-kanban.db"
        # Isolate the flock sentinel: the production merge-gate cron holds
        # /tmp/agent-flow-merge-gate.lock and would make tests flaky.
        LOCK_FILE="$TEST_TMP/merge-gate.lock"
        export GH_REPO KANBAN_BOARD DRY_RUN ISSUE_LIMIT HERMES_HOME HERMES_BIN KANBAN_DB LOCK_FILE
        # The script sources a profile .env if present — override HOME
        # and PROFILE_ENV paths so it can't load real config.
        export HOME=/tmp
        bash "$MERGE_GATE" 2>>"$TEST_TMP/stderr.log"
    )
    local rc=$?
    return $rc
}

# Per-test scratch: every test runs in its own GH_STATE file.
new_test() {
    TEST_TMP="$(mktemp -d /tmp/agent-flow-merge-gate-tests.XXXXXX)"
    install_mocks
    : >"$TEST_TMP/stderr.log"
}

# Helper: read all journal lines matching a pattern
journal_grep() {  # $1=pattern
    grep -F "$1" "$GH_JOURNAL" || true
}

summary() {
    printf '\n%s==== Summary ====%s\n' "$YEL" "$END"
    printf 'total:  %d\n' "$TESTS_TOTAL"
    printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
    if [ "$TESTS_FAILED" -gt 0 ]; then
        printf '%sfailed: %d%s\n' "$RED" "$TESTS_FAILED" "$END"
        printf 'failures:\n'
        for n in "${FAILED_NAMES[@]}"; do printf '  - %s\n' "$n"; done
        exit 1
    fi
    printf '%spassed: %d%s\n' "$GRN" "$TESTS_PASSED" "$END"
    exit 0
}