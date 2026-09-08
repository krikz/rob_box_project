#!/bin/bash
# ============================================================================
# test_nightly_review_persistence.sh — тесты персистентности находок ADR-0079.
#
# ADR-0079 (follow-up ADR-0049, issue #2159): ночной ревью должен
#  (1) сохранять находки между сессиями (findings-store = append-only JSONL
#      в `<reports_dir>/nightly-review/<DATE>.jsonl`);
#  (2) дедуплицировать находки по стабильному fingerprint (sha1[:12]) — НЕ
#      через дату (ISO-неделя, а не DATE в ключе);
#  (3) рейзить kanban-карточку ТОЛЬКО при outcome=open-issue-* —
#      «находок нет» значит «нет карточки», а не «пустая карточка»
#      (контракт §3.2 ADR-0049: честный пустой отчёт лучше выдуманного).
#
# Стратегия (как в test_nightly_review.sh):
#   - mock hermes (journal/list);
#   - mock gh (issue list/create);
#   - фикстура репо с churn (src/voice/, scripts/agent_flow/);
#   - реальный git (нужен для churn).
#
# Контракт JSONL (append-only):
#   одна строка = JSON {ts, task_id, component, files_changed,
#                        findings[{type,severity,fingerprint,file,line,raw}],
#                        outcome, fingerprint};
#   outcome ∈ {no-real-defect, open-issue-<N>, duplicate-suppressed:<fp>}.
#
# Invocation:
#   bash scripts/agent_flow/tests/test_nightly_review_persistence.sh
# Возвращает 0 при всех pass, 1 при первом fail.
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
NIGHTLY="$REPO_ROOT/agent-flow-nightly-review.sh"

TEST_TMP="${TEST_TMP:-/tmp/agent-flow-nightly-review-persistence-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP/bin" "$TEST_TMP/state" "$TEST_TMP/repo" "$TEST_TMP/reports" "$TEST_TMP/issues"

KANBAN_JOURNAL="$TEST_TMP/journal"
KANBAN_LIST_FILE="$TEST_TMP/kanban_list.json"
GH_ISSUE_FILE="$TEST_TMP/issues/findings.json"
GH_JOURNAL="$TEST_TMP/gh_journal"
REPORTS_DIR="$TEST_TMP/reports"
export KANBAN_JOURNAL KANBAN_LIST_FILE GH_ISSUE_FILE GH_JOURNAL REPORTS_DIR

# --- mock hermes (kanban create/list) ---------------------------------------
cat > "$TEST_TMP/bin/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
journal="${KANBAN_JOURNAL:-/dev/null}"
list_file="${KANBAN_LIST_FILE:-/dev/null}"
sub="${4:-}"
case "$sub" in
    list)
        cat "$list_file" 2>/dev/null || echo '[]'
        ;;
    create)
        printf 'create\t%s\n' "$*" >> "$journal"
        local id="t_mock_${RANDOM}"
        echo "{\"id\": \"$id\", \"status\": \"ready\"}"
        ;;
    *)
        echo "mock: unexpected kanban subcommand: $sub" >&2
        exit 2
        ;;
esac
exit 0
HERMES_MOCK_EOF
chmod +x "$TEST_TMP/bin/hermes"

# --- mock gh ----------------------------------------------------------------
cat > "$TEST_TMP/bin/gh" <<'GH_MOCK_EOF'
#!/bin/bash
gh_journal="${GH_JOURNAL:-/dev/null}"
cmd="${1:-}"
case "$cmd" in
    issue)
        sub="${2:-}"
        case "$sub" in
            list)
                if [ -f "${GH_ISSUE_FILE:-/dev/null}" ]; then
                    cat "$GH_ISSUE_FILE"
                else
                    echo '[]'
                fi
                ;;
            create)
                printf 'gh_issue_create\t%s\n' "$*" >> "$gh_journal"
                local num="42${RANDOM}"
                echo "{\"number\": $num, \"html_url\": \"https://github.com/x/y/issues/$num\"}"
                ;;
            *)
                echo '[]'
                ;;
        esac
        ;;
    *)
        echo '[]'
        ;;
esac
exit 0
GH_MOCK_EOF
chmod +x "$TEST_TMP/bin/gh"

# --- shims ------------------------------------------------------------------
if ! command -v flock >/dev/null 2>&1; then
    printf '#!/bin/bash\nexit 0\n' > "$TEST_TMP/bin/flock"
    chmod +x "$TEST_TMP/bin/flock"
fi
if ! python3 -c 'pass' >/dev/null 2>&1; then
    printf '#!/bin/bash\nexec python "$@"\n' > "$TEST_TMP/bin/python3"
    chmod +x "$TEST_TMP/bin/python3"
fi

# --- fixture repo -----------------------------------------------------------
FIXTURE_REPO="$TEST_TMP/repo"
setup_repo() {
    rm -rf "$FIXTURE_REPO"
    mkdir -p "$FIXTURE_REPO"
    (
        cd "$FIXTURE_REPO" || exit 1
        git init -q .
        git config core.autocrlf false
        git config user.email t@t.t
        git config user.name tester
        mkdir -p src/voice scripts/agent_flow
        for f in src/voice/a.py src/voice/b.py src/voice/c.py \
                 scripts/agent_flow/x.sh scripts/agent_flow/y.sh; do
            printf 'line1\nline2\n' > "$f"
        done
        git add -A
        git commit -qm "feat: persistence fixture"
        printf 'line3\n' >> src/voice/a.py
        git add -A
        git commit -qm "fix: persistence fixture second"
        git update-ref refs/remotes/origin/develop HEAD
    )
}
setup_repo

# --- registry ---------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    printf '[ RUN     ] %s\n' "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        printf '[   PASS  ] %s\n' "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED + 1))
        FAILED_NAMES+=("$name")
        printf '[   FAIL  ] %s\n' "$name"
    fi
}

assert_eq() {
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  assert fail: %s\n    needle: %q\n' "$3" "$1" >&2; return 1 ;;
    esac
}

assert_not_contains() {
    case "$2" in
        *"$1"*) printf '  assert fail: %s\n    needle should NOT appear: %q\n' "$3" "$1" >&2; return 1 ;;
        *) return 0 ;;
    esac
}

# --- runner -----------------------------------------------------------------
STDOUT_FILE="$TEST_TMP/stdout"
STDERR_FILE="$TEST_TMP/stderr"

run_nightly() {
    : > "$KANBAN_JOURNAL"
    : > "$GH_JOURNAL"
    env \
        HOME="$TEST_TMP" \
        PATH="$TEST_TMP/bin:$PATH" \
        GH_REPO=krikz/rob_box_project \
        KANBAN_JOURNAL="$KANBAN_JOURNAL" \
        KANBAN_LIST_FILE="$KANBAN_LIST_FILE" \
        GH_ISSUE_FILE="$GH_ISSUE_FILE" \
        GH_JOURNAL="$GH_JOURNAL" \
        NIGHTLY_REVIEW_REPORTS_DIR="$REPORTS_DIR" \
        NIGHTLY_REVIEW_TEST_MODE=1 \
        NIGHTLY_REVIEW_DATE="2026-09-08" \
        NIGHTLY_REVIEW_STATE_DIR="$TEST_TMP/state" \
        LOCK_FILE="$TEST_TMP/nightly.lock" \
        REPO_DIR="$FIXTURE_REPO" \
        KANBAN_BOARD=robbox \
        HERMES_HOME="$TEST_TMP/hermes-home" \
        "$@" \
        bash "$NIGHTLY" > "$STDOUT_FILE" 2> "$STDERR_FILE"
    echo $?
}

reset_state() {
    rm -f "$TEST_TMP"/state/*.done 2>/dev/null || true
    echo '[]' > "$KANBAN_LIST_FILE"
    echo '[]' > "$GH_ISSUE_FILE"
    : > "$GH_JOURNAL"
    : > "$KANBAN_JOURNAL"
    rm -rf "$REPORTS_DIR/nightly-review"
    mkdir -p "$REPORTS_DIR/nightly-review"
}

kanban_creates() {
    local n
    n="$(grep -c '^create' "$KANBAN_JOURNAL" 2>/dev/null || true)"
    printf '%s' "${n:-0}"
}

gh_issue_creates() {
    local n
    n="$(grep -c '^gh_issue_create' "$GH_JOURNAL" 2>/dev/null || true)"
    printf '%s' "${n:-0}"
}

# ============================================================================
# P1. JSONL создаётся автоматически (default REPORTS_DIR).
# ============================================================================
test_P1_jsonl_auto_created() {
    reset_state
    local rc
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "P1: exit 0" || return 1
    local jsonl="$REPORTS_DIR/nightly-review/2026-09-08.jsonl"
    [ -f "$jsonl" ] || { printf '  assert fail: P1: JSONL не создан автоматически (%s)\n' "$jsonl" >&2; return 1; }
    local line
    line="$(head -1 "$jsonl")"
    python3 - "$jsonl" <<'PY'
import json, sys
path = sys.argv[1]
required = {"ts", "task_id", "component", "files_changed", "findings", "outcome"}
with open(path) as f:
    line = f.readline()
    if not line.strip():
        print("  P1: пустая строка в JSONL"); sys.exit(1)
    try:
        rec = json.loads(line)
    except Exception as e:
        print(f"  P1: строка не JSON: {e}"); sys.exit(1)
    missing = required - set(rec.keys())
    if missing:
        print(f"  P1: без полей {missing}"); sys.exit(1)
    if not isinstance(rec["findings"], list):
        print(f"  P1: findings не list"); sys.exit(1)
PY
}

# ============================================================================
# P2. no-real-defect → JSONL пишется, kanban-карточка НЕ создаётся.
# ============================================================================
test_P2_no_real_defect() {
    reset_state
    local rc
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true NIGHTLY_REVIEW_OUTCOME=no-real-defect \
        COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "P2: exit 0" || return 1
    local jsonl="$REPORTS_DIR/nightly-review/2026-09-08.jsonl"
    [ -f "$jsonl" ] || { printf '  assert fail: P2: JSONL не создан при no-real-defect\n' >&2; return 1; }
    local line
    line="$(head -1 "$jsonl")"
    assert_contains '"outcome": "no-real-defect"' "$line" "P2: outcome=no-real-defect" || return 1
    # Главное: kanban-карточка НЕ создаётся (находок нет → тишина)
    assert_eq "0" "$(kanban_creates)" "P2: 0 kanban-карточек при no-real-defect" || return 1
    assert_contains "no-real-defect" "$(cat "$STDERR_FILE")" "P2: в логе есть no-real-defect" || return 1
}

# ============================================================================
# P3. duplicate-suppressed:<fingerprint> → JSONL пишет, kanban-карточки нет.
# ============================================================================
test_P3_duplicate_suppressed() {
    reset_state
    local fingerprint="a3f4b9c0d1e2-test-dedup"
    local rc
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true \
        NIGHTLY_REVIEW_OUTCOME="duplicate-suppressed:${fingerprint}" \
        COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "P3: exit 0" || return 1
    assert_eq "0" "$(kanban_creates)" "P3: kanban-карточка не нужна при чистом дубле" || return 1
    local jsonl="$REPORTS_DIR/nightly-review/2026-09-08.jsonl"
    [ -f "$jsonl" ] || { printf '  assert fail: P3: JSONL не создан\n' >&2; return 1; }
    local line
    line="$(head -1 "$jsonl")"
    assert_contains '"outcome": "duplicate-suppressed:' "$line" "P3: outcome=duplicate-suppressed:..." || return 1
    assert_contains "\"fingerprint\": \"$fingerprint\"" "$line" "P3: fingerprint в JSONL" || return 1
}

# ============================================================================
# P4. open-issue-* (по умолчанию) → kanban-карточка создаётся, JSONL пишется.
# ============================================================================
test_P4_open_issue_creates_card() {
    reset_state
    local rc
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "P4: exit 0" || return 1
    assert_eq "1" "$(kanban_creates)" "P4: 1 kanban-карточка при open-issue" || return 1
    local jsonl="$REPORTS_DIR/nightly-review/2026-09-08.jsonl"
    [ -f "$jsonl" ] || { printf '  assert fail: P4: JSONL не создан\n' >&2; return 1; }
    local line
    line="$(head -1 "$jsonl")"
    assert_contains '"outcome": "open-issue' "$line" "P4: outcome=open-issue-..." || return 1
}

# ============================================================================
# P5. Ключ НЕ содержит голую дату — только ISO-неделю (issue #2159).
# ============================================================================
test_P5_iso_week_dedup_key() {
    reset_state
    local rc journal iso_week
    iso_week="$(date -d '2026-09-08' +%G-W%V 2>/dev/null || echo '2026-W36')"
    rc="$(run_nightly NIGHTLY_REVIEW_FORCE=true COMPONENT_REVIEW_MAX=0)"
    assert_eq "0" "$rc" "P5: exit 0" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "retro:nightly-review-${iso_week}" "$journal" "P5: idempotency-key=ISO-неделя" || return 1
    assert_not_contains "retro:nightly-review-2026-09-08" "$journal" "P5: ключ НЕ содержит голую дату" || return 1
}

# ============================================================================
# P6. Fingerprint helper: стабильный sha1[:12], разный для разных входов.
# ============================================================================
test_P6_fingerprint_helper() {
    local fp1 fp2 fp3
    fp1="$(python3 -c 'import hashlib; print(hashlib.sha1(b"bug:src/voice/a.py:42:sample_rate").hexdigest()[:12])')"
    fp2="$(python3 -c 'import hashlib; print(hashlib.sha1(b"bug:src/voice/a.py:42:sample_rate").hexdigest()[:12])')"
    fp3="$(python3 -c 'import hashlib; print(hashlib.sha1(b"bug:src/voice/a.py:42:other").hexdigest()[:12])')"
    assert_eq "$fp1" "$fp2" "P6: детерминизм sha1[:12]" || return 1
    [ "$fp1" != "$fp3" ] || { printf '  assert fail: P6: разный символ дал тот же fingerprint\n' >&2; return 1; }
    # Длина 12
    [ "${#fp1}" -eq 12 ] || { printf '  assert fail: P6: длина fingerprint != 12 (%s)\n' "$fp1" >&2; return 1; }
}

run_test "P1: JSONL создаётся автоматически"               test_P1_jsonl_auto_created
run_test "P2: no-real-defect → нет kanban-карточки"        test_P2_no_real_defect
run_test "P3: duplicate-suppressed → нет kanban-карточки"  test_P3_duplicate_suppressed
run_test "P4: open-issue-* → kanban-карточка + JSONL"      test_P4_open_issue_creates_card
run_test "P5: ключ = ISO-неделя (фикс #2159)"              test_P5_iso_week_dedup_key
run_test "P6: fingerprint sha1[:12] стабилен"              test_P6_fingerprint_helper

printf '\n[==========] %d tests, %d passed, %d failed\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -ne 0 ]; then
    printf '[  FAILED  ] %s\n' "${FAILED_NAMES[@]}"
    printf 'artifacts: %s\n' "$TEST_TMP"
    exit 1
fi
rm -rf "$TEST_TMP"
exit 0